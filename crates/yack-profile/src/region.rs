// yack: Planar regions bounded by lines and circular arcs, in sketch millimetres: outlines combined,
// Adapted from springrep@a03e8a1 src/profile/region.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/profile/region.rs to yack-profile (M3 W4); `.expect`/`.unwrap` as counted `panic!`s (gate 9).
// M3 hardening: loops walked as cyclic pairs, boundary linking without unwraps, checked access
// instead of indexing, named rounding limits; the same arithmetic in the same order.
//! Planar regions bounded by lines and circular arcs, in sketch millimetres: outlines combined,
//! their corners rounded and the whole offset in the sketch plane, before one sweep makes the
//! solid. Every boundary piece carries the names of the outline edges it lies on, so a piece where
//! two outlines' edges coincide carries both.

use std::f64::consts::{PI, TAU};

use super::{NamedSegment, Names, Profile, ProfileLoop, Segment, cyclic_pairs, polar, shoelace};
use crate::{
    error::{Error, Result},
    math::Vec2,
    tol,
};

#[derive(Clone, Debug, PartialEq)]
pub enum Geom {
    Line {
        a: Vec2,
        b: Vec2,
    },
    /// From angle `a0` about `c` through `sweep`, counter-clockwise when positive; a whole circle
    /// when `|sweep|` is a full turn.
    Arc {
        c: Vec2,
        r: f64,
        a0: f64,
        sweep: f64,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Seg {
    pub geom: Geom,
    pub names: Names,
}

/// A closed chain of segments with the region on its left.
#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    pub segs: Vec<Seg>,
}

/// Loops with material on their left: outer boundaries counter-clockwise, holes clockwise.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Region {
    pub loops: Vec<Loop>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Add,
    Cut,
    Intersect,
}

/// Below this a length is zero, for a direction divided by it.
const NO_LENGTH: f64 = 1e-300; // tol: an underflow guard

/// A sweep this close to a full turn is a whole circle.
const FULL_TURN_ROUNDING: f64 = 1e-9; // tol: angle rounding, in radians

/// How far inside a hole, across its boundary, the point that finds its outer loop lies.
const HOLE_PROBE: f64 = 1e-6; // tol: a probing step, not a tolerance

/// `a` wrapped into (-π, π].
fn wrap(a: f64) -> f64 {
    let w = a.rem_euclid(TAU);
    if w > PI { w - TAU } else { w }
}

fn unsupported<T>(m: impl Into<String>) -> Result<T> {
    Err(Error::Unsupported(m.into()))
}

impl Geom {
    pub fn at(&self, t: f64) -> Vec2 {
        match *self {
            Geom::Line { a, b } => a.lerp(b, t),
            Geom::Arc { c, r, a0, sweep } => polar(c, r, a0 + sweep * t),
        }
    }

    pub fn start(&self) -> Vec2 {
        self.at(0.0)
    }

    pub fn end(&self) -> Vec2 {
        self.at(1.0)
    }

    pub fn len(&self) -> f64 {
        match *self {
            Geom::Line { a, b } => a.dist(b),
            Geom::Arc { r, sweep, .. } => r * sweep.abs(),
        }
    }

    pub fn is_full(&self) -> bool {
        matches!(*self, Geom::Arc { sweep, .. } if sweep.abs() >= TAU - tol::ANGULAR)
    }

    /// The unit direction of travel at `t`.
    pub fn tangent(&self, t: f64) -> Vec2 {
        match *self {
            Geom::Line { a, b } => (b - a) / a.dist(b).max(NO_LENGTH),
            Geom::Arc { a0, sweep, .. } => {
                let a = a0 + sweep * t;
                Vec2::new(-a.sin(), a.cos()) * sweep.signum()
            }
        }
    }

    pub fn sub(&self, t0: f64, t1: f64) -> Geom {
        match *self {
            Geom::Line { .. } => Geom::Line { a: self.at(t0), b: self.at(t1) },
            Geom::Arc { c, r, a0, sweep } => {
                Geom::Arc { c, r, a0: a0 + sweep * t0, sweep: sweep * (t1 - t0) }
            }
        }
    }

    pub fn reversed(&self) -> Geom {
        match *self {
            Geom::Line { a, b } => Geom::Line { a: b, b: a },
            Geom::Arc { c, r, a0, sweep } => Geom::Arc { c, r, a0: a0 + sweep, sweep: -sweep },
        }
    }

    /// The parameter of the point of the carrier nearest `p`. On an arc, angles past either end
    /// come back as the nearer of `t < 0` and `t > 1`.
    pub fn param(&self, p: Vec2) -> f64 {
        match *self {
            Geom::Line { a, b } => {
                let d = b - a;
                (p - a).dot(d) / d.dot(d).max(NO_LENGTH)
            }
            Geom::Arc { c, a0, sweep, .. } => {
                let s = sweep.signum() * ((p.y - c.y).atan2(p.x - c.x) - a0);
                let s = s.rem_euclid(TAU);
                let span = sweep.abs();
                if self.is_full() {
                    return s / TAU;
                }
                if s > span && s - span > TAU - s { (s - TAU) / span } else { s / span }
            }
        }
    }

    /// The distance from `p` to the segment and the parameter of its nearest point.
    pub fn closest(&self, p: Vec2) -> (f64, f64) {
        let t = self.param(p).clamp(0.0, 1.0);
        let mut best = (p.dist(self.at(t)), t);
        for e in [0.0, 1.0] {
            let d = p.dist(self.at(e));
            if d < best.0 {
                best = (d, e);
            }
        }
        best
    }

    /// The angle the segment turns through as seen from `p`, which is not on it.
    fn winding(&self, p: Vec2) -> f64 {
        let chord = |a: Vec2, b: Vec2| {
            let (u, v) = (a - p, b - p);
            u.cross(v).atan2(u.dot(v))
        };
        match *self {
            Geom::Line { a, b } => chord(a, b),
            Geom::Arc { c, r, sweep, .. } => {
                let (a, b) = (self.start(), self.end());
                let inside_disc = p.dist(c) < r;
                if self.is_full() {
                    return if inside_disc { TAU * sweep.signum() } else { 0.0 };
                }
                // The arc and its chord back bound a circular segment, wound the arc's way. One
                // cross product decides both the chord's angle and the side, so rounding cannot
                // split them; a point on the chord sees the arc turn half a turn its way.
                let (u, v) = (a - p, b - p);
                let x = (b - a).cross(p - a);
                if x == 0.0 && u.dot(v) < 0.0 {
                    return PI * sweep.signum();
                }
                let m = self.at(0.5);
                let side = x * (b - a).cross(m - a) > 0.0;
                let corr = if inside_disc && side { TAU * sweep.signum() } else { 0.0 };
                x.atan2(u.dot(v)) + corr
            }
        }
    }
}

/// Points where the carriers of two segments (infinite line, whole circle) meet. Tangent contact
/// within the linear tolerance is one point.
fn carrier_crossings(g: &Geom, h: &Geom) -> Vec<Vec2> {
    match (g, h) {
        (Geom::Line { a, b }, Geom::Line { a: p, b: q }) => {
            let (d, e) = (*b - *a, *q - *p);
            let den = d.cross(e);
            if den.abs() <= tol::ANGULAR * d.norm() * e.norm() {
                return vec![];
            }
            let t = (*p - *a).cross(e) / den;
            vec![*a + d * t]
        }
        (Geom::Line { a, b }, Geom::Arc { c, r, .. })
        | (Geom::Arc { c, r, .. }, Geom::Line { a, b }) => {
            let d = (*b - *a) / a.dist(*b);
            let foot = *a + d * (*c - *a).dot(d);
            let h = foot.dist(*c);
            if (h - r).abs() <= tol::LINEAR {
                vec![foot]
            } else if h < *r {
                let s = (r * r - h * h).sqrt();
                vec![foot + d * s, foot - d * s]
            } else {
                vec![]
            }
        }
        (Geom::Arc { c: c1, r: r1, .. }, Geom::Arc { c: c2, r: r2, .. }) => {
            let dv = *c2 - *c1;
            let d = dv.norm();
            if d <= tol::LINEAR {
                return vec![];
            }
            if d > r1 + r2 + tol::LINEAR || d < (r1 - r2).abs() - tol::LINEAR {
                return vec![];
            }
            let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
            let h2 = r1 * r1 - a * a;
            let m = *c1 + dv * (a / d);
            let n = dv.perp() / d;
            if h2 <= 0.0
                || (d - (r1 + r2)).abs() <= tol::LINEAR
                || (d - (r1 - r2).abs()).abs() <= tol::LINEAR
            {
                vec![m]
            } else {
                let h = h2.sqrt();
                vec![m + n * h, m - n * h]
            }
        }
    }
}

/// Parameters on `g` and on `h` where the two segments meet, ends included.
fn meetings(g: &Geom, h: &Geom) -> Vec<(f64, f64)> {
    let mut pts = carrier_crossings(g, h);
    pts.extend([g.start(), g.end(), h.start(), h.end()]);
    let mut out = Vec::new();
    for p in pts {
        let (dg, tg) = g.closest(p);
        let (dh, th) = h.closest(p);
        if dg <= tol::LINEAR && dh <= tol::LINEAR {
            out.push((tg, th));
        }
    }
    out
}

/// The segment cut at the parameters `ts`; a whole circle is cut into arcs between them.
fn split(seg: &Seg, ts: &[f64]) -> Vec<Seg> {
    let len = seg.geom.len().max(NO_LENGTH);
    let close = |a: f64, b: f64| (a - b).abs() * len <= tol::LINEAR;
    let mut ts: Vec<f64> = ts.iter().map(|t| t.clamp(0.0, 1.0)).collect();
    ts.sort_by(f64::total_cmp);
    let piece = |g: Geom| Seg { geom: g, names: seg.names.clone() };
    if seg.geom.is_full() {
        let mut cuts: Vec<f64> = Vec::new();
        for t in ts {
            let t = if close(t, 1.0) { 0.0 } else { t };
            if !cuts.iter().any(|&c| close(c, t)) {
                cuts.push(t);
            }
        }
        cuts.sort_by(f64::total_cmp);
        if cuts.is_empty() {
            return vec![seg.clone()];
        }
        // Each cut to the next, the last round to the first a turn on.
        let ends = cuts.iter().skip(1).copied().chain(cuts.first().map(|&c| c + 1.0));
        return cuts.iter().zip(ends).map(|(&t0, t1)| piece(seg.geom.sub(t0, t1))).collect();
    }
    let mut cuts = vec![0.0];
    for t in ts {
        if !close(t, *cuts.last().unwrap_or(&0.0)) && !close(t, 1.0) {
            cuts.push(t);
        }
    }
    cuts.push(1.0);
    cuts.windows(2)
        .filter_map(|w| match *w {
            [t0, t1] => Some(piece(seg.geom.sub(t0, t1))),
            _ => None,
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Class {
    Inside,
    Outside,
    /// On the other region's boundary, running the same way as it or against it.
    On {
        same: bool,
    },
}

impl Region {
    pub fn segs(&self) -> impl Iterator<Item = &Seg> {
        self.loops.iter().flat_map(|l| l.segs.iter())
    }

    pub fn winding(&self, p: Vec2) -> f64 {
        self.segs().map(|s| s.geom.winding(p)).sum::<f64>() / TAU
    }

    pub fn contains(&self, p: Vec2) -> bool {
        self.winding(p).round() != 0.0
    }

    /// Where a piece of boundary lies against this region, and the names of the boundary it lies
    /// on.
    fn classify(&self, g: &Geom) -> (Class, Names) {
        let m = g.at(0.5);
        for s in self.segs() {
            let (d, t) = s.geom.closest(m);
            if d <= tol::LINEAR {
                let same = g.tangent(0.5).dot(s.geom.tangent(t)) > 0.0;
                return (Class::On { same }, s.names.clone());
            }
        }
        let class = if self.contains(m) { Class::Inside } else { Class::Outside };
        (class, Names::new())
    }

    /// This region combined with `other`.
    pub fn combine(&self, other: &Region, op: Op) -> Result<Region> {
        let a: Vec<&Seg> = self.segs().collect();
        let b: Vec<&Seg> = other.segs().collect();
        let mut ta = vec![Vec::new(); a.len()];
        let mut tb = vec![Vec::new(); b.len()];
        for (sa, cuts_a) in a.iter().zip(&mut ta) {
            for (sb, cuts_b) in b.iter().zip(&mut tb) {
                for (s, t) in meetings(&sa.geom, &sb.geom) {
                    cuts_a.push(s);
                    cuts_b.push(t);
                }
            }
        }
        let mut kept: Vec<Seg> = Vec::new();
        for (side, (segs, ts)) in [(a, ta), (b, tb)].into_iter().enumerate() {
            let against = if side == 0 { other } else { self };
            for (s, t) in segs.iter().zip(ts) {
                for piece in split(s, &t) {
                    let (class, on_names) = against.classify(&piece.geom);
                    let keep = match (op, side, class) {
                        (_, _, Class::On { .. }) if side == 1 => None,
                        (Op::Add, _, Class::Outside) => Some(false),
                        (Op::Add, _, Class::On { same: true }) => Some(false),
                        (Op::Cut, 0, Class::Outside) => Some(false),
                        (Op::Cut, 0, Class::On { same: false }) => Some(false),
                        (Op::Cut, 1, Class::Inside) => Some(true),
                        (Op::Intersect, _, Class::Inside) => Some(false),
                        (Op::Intersect, 0, Class::On { same: true }) => Some(false),
                        _ => None,
                    };
                    if let Some(rev) = keep {
                        let mut names = piece.names.clone();
                        names.extend(on_names);
                        kept.push(Seg {
                            geom: if rev { piece.geom.reversed() } else { piece.geom.clone() },
                            names,
                        });
                    }
                }
            }
        }
        Ok(Region { loops: link(kept)? })
    }

    /// Rounds the corner where an edge named `a` meets one named `b`, at every such corner, with
    /// an arc of `radius` tangent to both. Returns how many corners it rounded.
    pub fn fillet(&mut self, a: &str, b: &str, radius: f64) -> Result<usize> {
        let mut done = 0;
        loop {
            let named = |s: &Seg, x: &str| s.names.contains(x);
            // The first corner (a loop, and the index of the segment after it) between the two.
            let found = self.loops.iter().enumerate().find_map(|(li, lp)| {
                let n = lp.segs.len();
                if n < 2 {
                    return None;
                }
                let before = lp.segs.iter().cycle().skip(n - 1);
                before
                    .zip(&lp.segs)
                    .position(|(p, q)| (named(p, a) && named(q, b)) || (named(p, b) && named(q, a)))
                    .map(|i| (li, i))
            });
            let Some((lp, i)) = found.and_then(|(li, i)| Some((self.loops.get_mut(li)?, i))) else {
                return Ok(done);
            };
            round_corner(lp, i, radius, a, b)?;
            done += 1;
        }
    }

    /// The region grown by `d` all round (shrunk when negative), each corner it grows round
    /// becoming an arc about the corner.
    pub fn offset(&self, d: f64) -> Result<Region> {
        let mut loops = Vec::new();
        for lp in &self.loops {
            loops.push(offset_loop(lp, d)?);
        }
        let out = Region { loops };
        out.check_simple()?;
        Ok(out)
    }

    /// Errors unless no two segments of the region cross or touch except at their shared ends.
    pub fn check_simple(&self) -> Result<()> {
        // Every segment, with its loop, its place in the loop and the loop's length.
        let all: Vec<(usize, usize, usize, &Seg)> = self
            .loops
            .iter()
            .enumerate()
            .flat_map(|(l, lp)| {
                let n = lp.segs.len();
                lp.segs.iter().enumerate().map(move |(i, s)| (l, i, n, s))
            })
            .collect();
        for (x, &(la, ia, n, sa)) in all.iter().enumerate() {
            for &(lb, ib, _, sb) in all.iter().skip(x + 1) {
                let near = |t: f64, at: f64, g: &Geom| (t - at).abs() * g.len() <= tol::LINEAR;
                for (s, t) in meetings(&sa.geom, &sb.geom) {
                    let shared = la == lb
                        && ((ib == (ia + 1) % n
                            && near(s, 1.0, &sa.geom)
                            && near(t, 0.0, &sb.geom))
                            || (ia == (ib + 1) % n
                                && near(s, 0.0, &sa.geom)
                                && near(t, 1.0, &sb.geom)));
                    if !shared {
                        let p = sa.geom.at(s);
                        return unsupported(format!(
                            "the region crosses itself near ({:.6}, {:.6})",
                            p.x, p.y
                        ));
                    }
                }
            }
        }
        Ok(())
    }

    /// Joins neighbouring segments of each loop that lie on one line or circle, their names
    /// united.
    pub fn merged(&self) -> Region {
        Region { loops: self.loops.iter().map(merge_loop).collect() }
    }

    /// The region of one outline drawn either way round, turned counter-clockwise.
    pub fn outline_ccw(mut self) -> Region {
        for l in &mut self.loops {
            if shoelace(&polygon(l)) < 0.0 {
                l.segs.reverse();
                for s in &mut l.segs {
                    s.geom = s.geom.reversed();
                }
            }
        }
        self
    }

    /// The outer loops, each with the holes inside it.
    pub fn faces(&self) -> Vec<(Loop, Vec<Loop>)> {
        let area = |l: &Loop| shoelace(&polygon(l));
        let mut outers: Vec<(Loop, f64, Vec<Loop>)> = Vec::new();
        let mut holes = Vec::new();
        for l in &self.loops {
            let a = area(l);
            if a > 0.0 {
                outers.push((l.clone(), a, Vec::new()));
            } else {
                holes.push(l.clone());
            }
        }
        for h in holes {
            // A hole with no boundary bounds nothing.
            let Some(first) = h.segs.first() else { continue };
            let p = first.geom.at(0.5);
            let probe = first.geom.at(0.5) + first.geom.tangent(0.5).perp() * -HOLE_PROBE;
            let owner = outers
                .iter_mut()
                .filter(|(o, _, _)| {
                    let r = Region { loops: vec![o.clone()] };
                    r.contains(probe) || r.contains(p)
                })
                .min_by(|x, y| x.1.total_cmp(&y.1));
            if let Some(o) = owner {
                o.2.push(h);
            }
        }
        outers.into_iter().map(|(o, _, h)| (o, h)).collect()
    }
}

impl Geom {
    fn segment(&self) -> Segment {
        match *self {
            Geom::Line { a, b } => Segment::Line { a, b },
            Geom::Arc { c, r, sweep, .. } if sweep.abs() >= TAU - FULL_TURN_ROUNDING => {
                Segment::Circle { center: c, radius: r, ccw: sweep > 0.0 }
            }
            Geom::Arc { c, r, a0, sweep } => {
                Segment::Arc { center: c, radius: r, start: a0, sweep }
            }
        }
    }
}

impl Region {
    /// The polygon through `points`, its edges unnamed.
    pub fn polygon(points: &[Vec2]) -> Region {
        let mut pts = points.to_vec();
        if shoelace(points) < 0.0 {
            pts.reverse();
        }
        let segs = cyclic_pairs(&pts)
            .map(|(&a, &b)| Seg { geom: Geom::Line { a, b }, names: Names::new() })
            .collect();
        Region { loops: vec![Loop { segs }] }
    }

    /// The region `profile` encloses. Only lines, arcs and whole circles bound a region.
    pub fn of(profile: &Profile) -> Result<Region> {
        let mut loops = Vec::new();
        for lp in &profile.normalized()?.loops {
            let mut segs = Vec::new();
            for s in &lp.segments {
                let geom = match s.segment {
                    Segment::Line { a, b } => Geom::Line { a, b },
                    Segment::Arc { center, radius, start, sweep } => {
                        Geom::Arc { c: center, r: radius, a0: start, sweep }
                    }
                    Segment::Circle { center, radius, ccw } => Geom::Arc {
                        c: center,
                        r: radius,
                        a0: 0.0,
                        sweep: if ccw { TAU } else { -TAU },
                    },
                    _ => {
                        return unsupported("a region bounded by a curve other than a line or arc");
                    }
                };
                segs.push(Seg { geom, names: s.names.clone() });
            }
            loops.push(Loop { segs });
        }
        Ok(Region { loops })
    }

    /// The region as one profile for each outer loop, with the holes inside it, every edge with
    /// its names.
    pub fn profiles(&self) -> Result<Vec<Profile>> {
        let lp = |l: &Loop| ProfileLoop {
            segments: l
                .segs
                .iter()
                .map(|s| NamedSegment { segment: s.geom.segment(), names: s.names.clone() })
                .collect(),
        };
        let out: Vec<Profile> = self
            .faces()
            .iter()
            .map(|(outer, holes)| Profile {
                loops: std::iter::once(outer).chain(holes).map(lp).collect(),
            })
            .collect();
        if out.is_empty() {
            return Err(Error::InvalidInput("the region encloses nothing".into()));
        }
        Ok(out)
    }
}

fn polygon(l: &Loop) -> Vec<Vec2> {
    let mut out = Vec::new();
    for s in &l.segs {
        let n = match s.geom {
            Geom::Line { .. } => 1,
            Geom::Arc { .. } => 32,
        };
        out.extend((0..n).map(|i| s.geom.at(i as f64 / n as f64)));
    }
    out
}

/// Kept boundary pieces joined end to end into loops. Where several pieces leave one point, the
/// one turning most to the left from the way in is taken, so each loop bounds one face.
fn link(mut pieces: Vec<Seg>) -> Result<Vec<Loop>> {
    pieces.retain(|s| s.geom.len() > tol::LINEAR || s.geom.is_full());
    let mut used = vec![false; pieces.len()];
    let mut loops = Vec::new();
    for (first, piece) in pieces.iter().enumerate() {
        match used.get_mut(first) {
            Some(u) if !*u => *u = true,
            _ => continue,
        }
        let mut lp = vec![piece.clone()];
        let start = piece.geom.start();
        // The last piece taken.
        let mut last = piece;
        let mut guard = 0;
        while last.geom.end().dist(start) > tol::LINEAR {
            let (end, back) = (last.geom.end(), -last.geom.tangent(1.0));
            let mut best: Option<(usize, f64)> = None;
            for (k, (p, &taken)) in pieces.iter().zip(&used).enumerate() {
                if taken || p.geom.start().dist(end) > tol::LINEAR {
                    continue;
                }
                // Clockwise angle from the way back to the way out, in (0, 2π].
                let out = p.geom.tangent(0.0);
                let mut cw = (-(back.cross(out))).atan2(back.dot(out));
                if cw <= 0.0 {
                    cw += TAU;
                }
                if best.is_none_or(|(_, b)| cw < b) {
                    best = Some((k, cw));
                }
            }
            let (Some((k, _)), Some(next)) = (best, best.and_then(|(k, _)| pieces.get(k))) else {
                return Err(Error::InvalidResult(vec![format!(
                    "a region boundary is open at ({:.6}, {:.6})",
                    end.x, end.y
                )]));
            };
            if let Some(u) = used.get_mut(k) {
                *u = true;
            }
            lp.push(next.clone());
            last = next;
            guard += 1;
            if guard > pieces.len() {
                return Err(Error::Numerical("region boundary did not close".into()));
            }
        }
        loops.push(merge_loop(&Loop { segs: lp }));
    }
    Ok(loops)
}

fn same_carrier(g: &Geom, h: &Geom) -> bool {
    match (g, h) {
        (Geom::Line { a, b }, Geom::Line { a: p, b: q }) => {
            let d = (*b - *a) / a.dist(*b).max(NO_LENGTH);
            let off = |x: Vec2| (x - *a).cross(d).abs();
            off(*p) <= tol::LINEAR && off(*q) <= tol::LINEAR && d.dot(*q - *p) > 0.0
        }
        (Geom::Arc { c, r, sweep: s1, .. }, Geom::Arc { c: c2, r: r2, sweep: s2, .. }) => {
            c.dist(*c2) <= tol::LINEAR && (r - r2).abs() <= tol::LINEAR && s1 * s2 > 0.0
        }
        _ => false,
    }
}

fn join(g: &Geom, h: &Geom) -> Geom {
    match (g, h) {
        (Geom::Line { a, .. }, Geom::Line { b, .. }) => Geom::Line { a: *a, b: *b },
        (Geom::Arc { c, r, a0, sweep }, Geom::Arc { sweep: s2, .. }) => {
            let total = sweep + s2;
            let whole = total.abs() >= TAU - FULL_TURN_ROUNDING;
            Geom::Arc {
                c: *c,
                r: *r,
                a0: if whole { 0.0 } else { *a0 },
                sweep: if whole { TAU * total.signum() } else { total },
            }
        }
        _ => g.clone(),
    }
}

fn merge_loop(l: &Loop) -> Loop {
    let mut segs = l.segs.clone();
    while segs.len() > 1 {
        // The first segment that runs on along the next one's carrier, joined with it.
        let Some((i, joined)) = cyclic_pairs(&segs).enumerate().find_map(|(i, (s, t))| {
            same_carrier(&s.geom, &t.geom).then(|| {
                let mut names = s.names.clone();
                names.extend(t.names.iter().cloned());
                (i, Seg { geom: join(&s.geom, &t.geom), names })
            })
        }) else {
            break;
        };
        let j = (i + 1) % segs.len();
        if let Some(s) = segs.get_mut(i) {
            *s = joined;
        }
        segs.remove(j);
    }
    Loop { segs }
}

/// The carriers of `g` moved `r` to either side: a line's two parallels, a circle's two
/// concentric circles of positive radius.
fn parallels(g: &Geom, r: f64) -> Vec<Geom> {
    match *g {
        Geom::Line { a, b } => {
            let n = ((b - a) / a.dist(b)).perp() * r;
            vec![Geom::Line { a: a + n, b: b + n }, Geom::Line { a: a - n, b: b - n }]
        }
        Geom::Arc { c, r: rr, .. } => [rr + r, rr - r]
            .into_iter()
            .filter(|&x| x > tol::LINEAR)
            .map(|x| Geom::Arc { c, r: x, a0: 0.0, sweep: TAU })
            .collect(),
    }
}

fn foot(g: &Geom, x: Vec2) -> Vec2 {
    match *g {
        Geom::Line { a, b } => {
            let d = (b - a) / a.dist(b);
            a + d * (x - a).dot(d)
        }
        Geom::Arc { c, r, .. } => c + (x - c) * (r / x.dist(c)),
    }
}

fn round_corner(lp: &mut Loop, i: usize, radius: f64, na: &str, nb: &str) -> Result<()> {
    let n = lp.segs.len();
    let ip = (i + n - 1) % n;
    let (Some(p), Some(q)) = (lp.segs.get(ip).cloned(), lp.segs.get(i).cloned()) else {
        return Err(Error::InvalidInput(format!("no corner where {na} meets {nb}")));
    };
    let corner = q.geom.start();
    let mut best: Option<(Vec2, f64, f64)> = None;
    for op in parallels(&p.geom, radius) {
        for oq in parallels(&q.geom, radius) {
            for c in carrier_crossings(&op, &oq) {
                let (fp, fq) = (foot(&p.geom, c), foot(&q.geom, c));
                let (dp, tp) = p.geom.closest(fp);
                let (dq, tq) = q.geom.closest(fq);
                let on = dp <= tol::LINEAR && dq <= tol::LINEAR;
                let apart = fp.dist(corner) > tol::LINEAR && fq.dist(corner) > tol::LINEAR;
                if on && apart && best.is_none_or(|(b, _, _)| c.dist(corner) < b.dist(corner)) {
                    best = Some((c, tp, tq));
                }
            }
        }
    }
    let Some((c, tp, tq)) = best else {
        return Err(Error::InvalidInput(format!(
            "a fillet of {radius} mm does not fit where {na} meets {nb}"
        )));
    };
    let (fp, fq) = (p.geom.at(tp), q.geom.at(tq));
    let a0 = (fp.y - c.y).atan2(fp.x - c.x);
    let sweep = wrap((fq.y - c.y).atan2(fq.x - c.x) - a0);
    let arc = Geom::Arc { c, r: radius, a0, sweep };
    if arc.tangent(0.0).dot(p.geom.tangent(tp)) < 0.0 {
        return Err(Error::Numerical(format!(
            "the fillet where {na} meets {nb} turns against its edges"
        )));
    }
    let mut new = Vec::new();
    for (k, s) in lp.segs.iter().enumerate() {
        if k == ip {
            if tp * s.geom.len() > tol::LINEAR {
                new.push(Seg { geom: s.geom.sub(0.0, tp), names: s.names.clone() });
            }
            new.push(Seg { geom: arc.clone(), names: Names::new() });
        } else if k == i {
            if (1.0 - tq) * s.geom.len() > tol::LINEAR {
                new.push(Seg { geom: s.geom.sub(tq, 1.0), names: s.names.clone() });
            }
        } else {
            new.push(s.clone());
        }
    }
    // A fillet on the loop's last corner puts its arc after the last segment; keep the loop's
    // start where it was.
    lp.segs = new;
    Ok(())
}

fn offset_geom(g: &Geom, d: f64) -> Result<Geom> {
    Ok(match *g {
        Geom::Line { a, b } => {
            let n = -((b - a) / a.dist(b)).perp() * d;
            Geom::Line { a: a + n, b: b + n }
        }
        Geom::Arc { c, r, a0, sweep } => {
            let r2 = r + d * sweep.signum();
            if r2 <= tol::LINEAR {
                return unsupported("an offset that swallows an arc");
            }
            Geom::Arc { c, r: r2, a0, sweep }
        }
    })
}

/// `g` with its end moved along its carrier to `p` (its start, when `at_end` is false).
fn trim_to(g: &Geom, p: Vec2, at_end: bool) -> Geom {
    match *g {
        Geom::Line { a, b } => {
            if at_end {
                Geom::Line { a, b: p }
            } else {
                Geom::Line { a: p, b }
            }
        }
        Geom::Arc { c, r, a0, sweep } => {
            let ap = (p.y - c.y).atan2(p.x - c.x);
            let k = sweep.signum();
            if at_end {
                let s = k * (k * (ap - a0)).rem_euclid(TAU);
                Geom::Arc { c, r, a0, sweep: s }
            } else {
                let s = k * (k * (a0 + sweep - ap)).rem_euclid(TAU);
                Geom::Arc { c, r, a0: ap, sweep: s }
            }
        }
    }
}

fn offset_loop(lp: &Loop, d: f64) -> Result<Loop> {
    let n = lp.segs.len();
    if let [only] = lp.segs.as_slice() {
        return Ok(Loop {
            segs: vec![Seg { geom: offset_geom(&only.geom, d)?, names: Names::new() }],
        });
    }
    let mut segs: Vec<Geom> =
        lp.segs.iter().map(|s| offset_geom(&s.geom, d)).collect::<Result<_>>()?;
    let mut joins: Vec<Option<Geom>> = vec![None; n];
    // Each corner: the original segments either side of it, in place `i` and the next.
    for (i, (before, after)) in cyclic_pairs(&lp.segs).enumerate() {
        let j = (i + 1) % n;
        let v = after.geom.start();
        let (Some(gi), Some(gj)) = (segs.get(i).cloned(), segs.get(j).cloned()) else {
            return unsupported("an offset loop shorter than its corners");
        };
        let (e, s) = (gi.end(), gj.start());
        if e.dist(s) <= tol::LINEAR {
            continue;
        }
        let turn = before.geom.tangent(1.0).cross(after.geom.tangent(0.0));
        if turn * d > 0.0 {
            let a0 = (e.y - v.y).atan2(e.x - v.x);
            let sweep = wrap((s.y - v.y).atan2(s.x - v.x) - a0);
            if let Some(join) = joins.get_mut(i) {
                *join = Some(Geom::Arc { c: v, r: d.abs(), a0, sweep });
            }
        } else {
            let Some(p) = carrier_crossings(&gi, &gj)
                .into_iter()
                .min_by(|x, y| x.dist(v).total_cmp(&y.dist(v)))
            else {
                return unsupported("an offset corner whose edges never meet");
            };
            if let Some(g) = segs.get_mut(i) {
                *g = trim_to(&gi, p, true);
            }
            if let Some(g) = segs.get_mut(j) {
                *g = trim_to(&gj, p, false);
            }
        }
    }
    let mut out = Vec::new();
    for ((g, original), join) in segs.into_iter().zip(&lp.segs).zip(joins) {
        let orig = &original.geom;
        let same_way = match (&g, orig) {
            (Geom::Line { a, b }, Geom::Line { a: p, b: q }) => (*b - *a).dot(*q - *p) > 0.0,
            (Geom::Arc { sweep, .. }, Geom::Arc { sweep: s0, .. }) => sweep * s0 > 0.0,
            _ => false,
        };
        if !same_way || g.len() <= tol::LINEAR {
            return unsupported("an offset that swallows an edge");
        }
        out.push(Seg { geom: g, names: Names::new() });
        if let Some(j) = join {
            out.push(Seg { geom: j, names: Names::new() });
        }
    }
    Ok(Loop { segs: out })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names(n: &str) -> Names {
        [n.to_string()].into_iter().collect()
    }

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64, p: &str) -> Region {
        let pts = [Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)];
        let side = ["v_min", "u_max", "v_max", "u_min"];
        Region {
            loops: vec![Loop {
                segs: (0..4)
                    .map(|i| Seg {
                        geom: Geom::Line { a: pts[i], b: pts[(i + 1) % 4] },
                        names: names(&format!("{p}_{}", side[i])),
                    })
                    .collect(),
            }],
        }
    }

    fn circle(c: Vec2, r: f64, p: &str) -> Region {
        Region {
            loops: vec![Loop {
                segs: vec![Seg {
                    geom: Geom::Arc { c, r, a0: 0.0, sweep: TAU },
                    names: names(&format!("{p}_side")),
                }],
            }],
        }
    }

    fn area(r: &Region) -> f64 {
        // Green's theorem, exact on lines and arcs.
        r.segs()
            .map(|s| match s.geom {
                Geom::Line { a, b } => a.cross(b) / 2.0,
                Geom::Arc { c, r, a0, sweep } => {
                    let a1 = a0 + sweep;
                    (r * r * sweep + c.x * r * (a1.sin() - a0.sin())
                        - c.y * r * (a1.cos() - a0.cos()))
                        / 2.0
                }
            })
            .sum()
    }

    #[test]
    fn overlapping_rects_union_cut_and_intersect() {
        let a = rect(0.0, 0.0, 4.0, 2.0, "a");
        let b = rect(2.0, 0.0, 6.0, 3.0, "b");
        let u = a.combine(&b, Op::Add).unwrap();
        assert_eq!(u.loops.len(), 1);
        assert!((area(&u) - 16.0).abs() < 1e-12);
        // The two bottoms lie on one line: one segment carrying both names.
        let bottom: Vec<&Seg> = u
            .segs()
            .filter(|s| s.names.contains("a_v_min") || s.names.contains("b_v_min"))
            .collect();
        assert_eq!(bottom.len(), 1);
        assert!(bottom[0].names.contains("a_v_min") && bottom[0].names.contains("b_v_min"));
        let c = a.combine(&b, Op::Cut).unwrap();
        assert!((area(&c) - 4.0).abs() < 1e-12);
        let i = a.combine(&b, Op::Intersect).unwrap();
        assert!((area(&i) - 4.0).abs() < 1e-12);
    }

    #[test]
    fn a_circle_through_a_notchs_corners_tangent_to_its_sides_rounds_its_end() {
        // A cable channel as a design gives it, in metres: the floor circle's chord and
        // then the pillar's end run through the centre of the circle cut next, where an arc's
        // winding used to come out a half turn wrong and drop the pillar's end.
        let m = |x: f64| x * 1000.0;
        let poly = |pts: &[(f64, f64)], p: &str| {
            let v: Vec<Vec2> = pts.iter().map(|&(x, y)| Vec2::new(m(x), m(y))).collect();
            Region {
                loops: vec![Loop {
                    segs: (0..v.len())
                        .map(|i| Seg {
                            geom: Geom::Line { a: v[i], b: v[(i + 1) % v.len()] },
                            names: names(&format!("{p}_{i}")),
                        })
                        .collect(),
                }],
            }
        };
        let channel = poly(
            &[
                (0.0524, 0.0305),
                (0.0524, 0.0335),
                (0.0514, 0.0335),
                (0.0514, 0.0415),
                (0.0524, 0.0415),
                (0.0524, 0.0445),
                (0.056, 0.0445),
                (0.056, 0.0305),
            ],
            "ch",
        );
        let c = Vec2::new(m(0.0514), m(0.0375));
        let steps: Vec<(Region, Op)> = vec![
            (circle(c, m(0.008) / 2.0, "floor"), Op::Add),
            (
                poly(
                    &[(0.0514, 0.0355), (0.0514, 0.0395), (0.057, 0.0395), (0.057, 0.0355)],
                    "pil",
                )
                .outline_ccw(),
                Op::Cut,
            ),
            (circle(c, m(0.004) / 2.0, "back"), Op::Cut),
        ];
        let mut r = channel.outline_ccw();
        for (s, op) in steps {
            r = r.combine(&s, op).unwrap();
        }
        assert_eq!(r.loops.len(), 1);
        assert!((area(&r) - (40.0 + 6.0 * PI)).abs() < 1e-9, "{}", area(&r));
    }

    #[test]
    fn hole_and_slot_of_circles() {
        let plate = rect(-5.0, -5.0, 5.0, 5.0, "p");
        let holed = plate.combine(&circle(Vec2::ZERO, 2.0, "h"), Op::Cut).unwrap();
        assert_eq!(holed.loops.len(), 2);
        assert!((area(&holed) - (100.0 - 4.0 * PI)).abs() < 1e-9);
        assert_eq!(holed.faces().len(), 1);
        // A stadium: the rect's sides are tangent to both end circles.
        let slot = rect(-3.0, -1.0, 3.0, 1.0, "r")
            .combine(&circle(Vec2::new(-3.0, 0.0), 1.0, "a"), Op::Add)
            .unwrap()
            .combine(&circle(Vec2::new(3.0, 0.0), 1.0, "b"), Op::Add)
            .unwrap();
        assert_eq!(slot.loops.len(), 1);
        assert_eq!(slot.loops[0].segs.len(), 4);
        assert!((area(&slot) - (12.0 + PI)).abs() < 1e-9);
    }

    #[test]
    fn fillets_and_offsets() {
        let mut r = rect(0.0, 0.0, 10.0, 6.0, "b");
        assert_eq!(r.fillet("b_u_max", "b_v_max", 2.0).unwrap(), 1);
        assert!((area(&r) - (60.0 - (4.0 - PI))).abs() < 1e-9);
        // A concave corner is filled.
        let mut l = rect(0.0, 0.0, 10.0, 10.0, "a")
            .combine(&rect(5.0, 5.0, 10.0, 10.0, "c"), Op::Cut)
            .unwrap();
        assert_eq!(l.fillet("c_u_min", "c_v_min", 1.0).unwrap(), 1);
        assert!((area(&l) - (75.0 + (1.0 - PI / 4.0))).abs() < 1e-9);
        let grown = rect(0.0, 0.0, 4.0, 2.0, "x").offset(1.0).unwrap();
        assert!((area(&grown) - (8.0 + 2.0 * 6.0 + PI)).abs() < 1e-9);
        let shrunk = rect(0.0, 0.0, 4.0, 2.0, "x").offset(-0.5).unwrap();
        assert!((area(&shrunk) - 3.0).abs() < 1e-12);
    }
}
