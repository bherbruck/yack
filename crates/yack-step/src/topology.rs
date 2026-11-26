// yack: STEP B-rep topology into a kernel solid: vertices and edges shared across faces, each face's
// Adapted from springrep@a03e8a1 src/step/topology.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the raw builder moved to build/raw.rs (M2 T5); the file side stays here; then to yack-step (M3 W9c), paths and visibility only.
//! STEP B-rep topology into a kernel solid: vertices and edges shared across faces, each face's
//! loops given pcurves that close in its parameter plane, with the seams and poles the kernel's
//! representation needs (doc/design.md, "Representation").
//!
//! A solid is read in three passes: the file's topology into [`Raw`], [`crate::build::heal`] where the
//! file's geometry disagrees with itself by more than the kernel tolerance, then the arena.

#[allow(clippy::disallowed_types)] use std::collections::HashMap; // D26: BTree* in hardening, proven by digests

use super::geometry::Geometry;
use yack_p21::Param;

use super::part21;
use crate::{
    Error, Result,
    build::{Raw, RawEdge, RawFace, Use, assemble, edge_range, heal, param_of},
    geom::{Circle, Curve, Line, Surface},
    math::{Frame, Point3},
    tol,
    topo::Brep,
};

fn bad<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::InvalidInput(format!("STEP #{id}: {what}")))
}

fn unsupported<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::Unsupported(format!("STEP #{id}: {what}")))
}

/// A table of the solid being read lacks an entry its own topology names: refused, not guessed at.
fn broken(what: &str) -> Error {
    Error::InvalidInput(format!("STEP: the topology names {what} it does not hold"))
}

/// The representative of `i` in the union-find `parent`, halving the path on the way.
fn root(parent: &mut [usize], mut i: usize) -> usize {
    while let Some(&up) = parent.get(i) {
        if up == i {
            break;
        }
        let grand = parent.get(up).copied().unwrap_or(up);
        if let Some(slot) = parent.get_mut(i) {
            *slot = grand;
        }
        i = grand;
    }
    i
}

/// `raw` without the edges split in two after they were read, which no face uses any more.
#[allow(clippy::disallowed_types)] fn without_split(mut raw: Raw, splits: &HashMap<usize, [usize; 2]>) -> Result<Raw> { // D26: BTree* in hardening, proven by digests
    if splits.is_empty() {
        return Ok(raw);
    }
    // Each old edge's new index; a split one has none.
    let mut index = Vec::with_capacity(raw.edges.len());
    let mut kept = Vec::with_capacity(raw.edges.len());
    for (i, e) in raw.edges.drain(..).enumerate() {
        if splits.contains_key(&i) {
            index.push(usize::MAX);
        } else {
            index.push(kept.len());
            kept.push(e);
        }
    }
    raw.edges = kept;
    for f in &mut raw.faces {
        for (_, uses) in &mut f.bounds {
            for u in uses {
                u.edge = *index.get(u.edge).ok_or_else(|| broken("an edge"))?;
            }
        }
    }
    Ok(raw)
}

/// `raw` with its coincident vertices made one, and then its coincident edges. A solid that
/// touches itself is written, as STEP's manifold solids require, with each side of the contact
/// given its own vertices and edges; two vertices of one valid solid are never that close, so
/// coincident ones are such a contact, and joining them gives it back.
#[allow(clippy::disallowed_types)] fn join_contacts(mut raw: Raw) -> Result<Raw> { // D26: BTree* in hardening, proven by digests
    let n = raw.points.len();
    let mut parent: Vec<usize> = (0..n).collect();
    let x_of = |i: usize| raw.points.get(i).map_or(f64::NAN, |p| p.x);
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| x_of(a).total_cmp(&x_of(b)));
    let mut joined = false;
    for (k, &i) in order.iter().enumerate() {
        let Some(&pi) = raw.points.get(i) else { continue };
        for &j in order.iter().skip(k + 1) {
            let Some(&pj) = raw.points.get(j) else { continue };
            if pj.x - pi.x > tol::LINEAR {
                break;
            }
            if pi.dist(pj) <= tol::LINEAR {
                let (a, b) = (root(&mut parent, i), root(&mut parent, j));
                if a != b {
                    if let Some(slot) = parent.get_mut(a.max(b)) {
                        *slot = a.min(b);
                    }
                    joined = true;
                }
            }
        }
    }
    if joined {
        // The points kept, renumbered in order.
        let mut index = vec![usize::MAX; n];
        let (mut points, mut ids) = (Vec::new(), Vec::new());
        for ((i, slot), (&p, &id)) in index.iter_mut().enumerate().zip(raw.points.iter().zip(&raw.vertex_ids)) {
            if root(&mut parent, i) == i {
                *slot = points.len();
                points.push(p);
                ids.push(id);
            }
        }
        let at = (0..n)
            .map(|i| index.get(root(&mut parent, i)).copied().ok_or_else(|| broken("a point")))
            .collect::<Result<Vec<usize>>>()?;
        let moved = |v: usize| at.get(v).copied().ok_or_else(|| broken("a vertex"));
        raw.points = points;
        raw.vertex_ids = ids;
        for e in &mut raw.edges {
            e.start = moved(e.start)?;
            e.end = moved(e.end)?;
        }
        for f in &mut raw.faces {
            for v in &mut f.points {
                *v = moved(*v)?;
            }
        }
    }
    // Edges between the same vertices that lie along each other: each later one is the first,
    // its uses turned where it runs the other way.
    let mut ends: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (i, e) in raw.edges.iter().enumerate() {
        ends.entry((e.start.min(e.end), e.start.max(e.end)))
            .or_default()
            .push(i);
    }
    let m = raw.edges.len();
    let mut into: Vec<(usize, bool)> = (0..m).map(|i| (i, false)).collect();
    let mut any = false;
    let ends_of = |e: &RawEdge| Some((*raw.points.get(e.start)?, *raw.points.get(e.end)?));
    for group in ends.values().filter(|g| g.len() > 1) {
        for (k, &i) in group.iter().enumerate() {
            if into.get(i).is_none_or(|&(to, _)| to != i) {
                continue;
            }
            let Some(a) = raw.edges.get(i) else { continue };
            let Some((pa, pz)) = ends_of(a) else { continue };
            let Some((t0, t1)) = edge_range(&a.curve, pa, pz, a.start == a.end) else {
                continue;
            };
            let (q, d) = a.curve.d1(0.5 * (t0 + t1));
            for &j in group.iter().skip(k + 1) {
                let Some(b) = raw.edges.get(j) else { continue };
                if into.get(j).is_none_or(|&(to, _)| to != j) {
                    continue;
                }
                // On `b`'s curve within its range: two arcs of one circle share their ends too.
                let Some((pb, pe)) = ends_of(b) else { continue };
                let Some((u0, u1)) = edge_range(&b.curve, pb, pe, b.start == b.end) else {
                    continue;
                };
                let mut t = param_of(&b.curve, q);
                if let Some(p) = b.curve.period() {
                    t = u0 + (t - u0).rem_euclid(p);
                }
                let (r, dr) = b.curve.d1(t);
                if (u0..=u1).contains(&t)
                    && r.dist(q) <= tol::LINEAR
                    && let Some(slot) = into.get_mut(j)
                {
                    *slot = (i, d.dot(dr) < 0.0);
                    any = true;
                }
            }
        }
    }
    if !any {
        return Ok(raw);
    }
    // Each edge's place among those kept, `usize::MAX` for one merged into another.
    let mut kept = Vec::with_capacity(m);
    let mut edges = Vec::new();
    for (i, (&(to, _), e)) in into.iter().zip(&raw.edges).enumerate() {
        if to == i {
            kept.push(edges.len());
            edges.push(e.clone());
        } else {
            kept.push(usize::MAX);
        }
    }
    for (i, (&(to, _), e)) in into.iter().zip(&raw.edges).enumerate() {
        if to != i {
            let into_edge = kept.get(to).and_then(|&k| edges.get_mut(k)).ok_or_else(|| broken("an edge"))?;
            into_edge.faces.extend(e.faces.iter().copied());
        }
    }
    raw.edges = edges;
    for f in &mut raw.faces {
        for (_, uses) in &mut f.bounds {
            for u in uses {
                let &(to, flip) = into.get(u.edge).ok_or_else(|| broken("an edge"))?;
                u.edge = *kept.get(to).ok_or_else(|| broken("an edge"))?;
                u.reversed ^= flip;
            }
        }
    }
    Ok(raw)
}

/// The vertex a use starts at.
/// A point's angle about `frame`'s axis.
fn about(frame: &Frame) -> impl Fn(Point3) -> f64 + '_ {
    move |p| {
        let l = frame.to_local(p);
        l.y.atan2(l.x)
    }
}

fn use_start(raw: &Raw, u: &Use) -> Option<usize> {
    let e = raw.edges.get(u.edge)?;
    Some(if u.reversed { e.end } else { e.start })
}

/// The point at the vertex a use starts at.
fn use_point(raw: &Raw, u: &Use) -> Option<Point3> {
    raw.points.get(use_start(raw, u)?).copied()
}

/// Reads one solid's topology, sharing vertices and edges between its faces.
#[allow(clippy::disallowed_types)] struct Reader<'g, 'a> { // D26: BTree* in hardening, proven by digests
    geo: &'g mut Geometry<'a>,
    raw: Raw,
    vertices: HashMap<u64, usize>,
    /// Each edge's index and whether it runs against the STEP edge's direction.
    edges: HashMap<u64, (usize, bool)>,
    /// A `POLY_LOOP`'s corners, by their coordinates, and its sides, by their two corners.
    corners: HashMap<[u64; 3], usize>,
    sides: HashMap<(usize, usize), usize>,
    /// Edges split in two after they were read, to give a seam a vertex: each one's two halves.
    splits: HashMap<usize, [usize; 2]>,
}

/// One bound of a face as the file gives it.
enum Bound {
    /// Whether the file calls it the outer bound, and its uses in loop order.
    Edges(bool, Vec<Use>),
    /// A `VERTEX_LOOP`: the face closes on this one vertex, the apex of a cone or the pole of a
    /// sphere, or, anywhere else, holds it as a point where something touches the face.
    Vertex(usize),
}

impl<'g, 'a> Reader<'g, 'a> {
    fn params(&self, id: u64, kinds: &[&str]) -> Result<(&'a str, &'a [Param])> {
        let Some(e) = self.geo.file.get(id) else {
            return bad(id, "missing instance");
        };
        for r in e.records() {
            if kinds.contains(&r.name.as_str()) {
                return Ok((r.name.as_str(), r.params.as_slice()));
            }
        }
        unsupported(
            id,
            &format!("{} where {} was expected", part21::kind(e), kinds.join(" or ")),
        )
    }

    fn vertex(&mut self, id: u64) -> Result<usize> {
        if let Some(&v) = self.vertices.get(&id) {
            return Ok(v);
        }
        let (_, a) = self.params(id, &["VERTEX_POINT"])?;
        let p = self.geo.point(self.geo.reference(id, a.get(1))?)?;
        self.raw.points.push(p);
        self.raw.vertex_ids.push(id);
        let v = self.raw.points.len() - 1;
        self.vertices.insert(id, v);
        Ok(v)
    }

    fn edge(&mut self, id: u64) -> Result<(usize, bool)> {
        if let Some(&e) = self.edges.get(&id) {
            return Ok(e);
        }
        let (_, a) = self.params(id, &["EDGE_CURVE"])?;
        let vs = self.vertex(self.geo.reference(id, a.get(1))?)?;
        let ve = self.vertex(self.geo.reference(id, a.get(2))?)?;
        let curve = self.geo.curve(self.geo.reference(id, a.get(3))?)?;
        let flip = a.get(4).and_then(Param::boolean) == Some(false);
        let (start, end) = if flip { (ve, vs) } else { (vs, ve) };
        self.raw.edges.push(RawEdge {
            id,
            curve,
            start,
            end,
            faces: Vec::new(),
        });
        let e = (self.raw.edges.len() - 1, flip);
        self.edges.insert(id, e);
        Ok(e)
    }

    /// The vertex at a `POLY_LOOP` corner; corners at one place are one vertex.
    fn corner(&mut self, id: u64) -> Result<usize> {
        let p = self.geo.point(id)?;
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        if let Some(&v) = self.corners.get(&key) {
            return Ok(v);
        }
        self.raw.points.push(p);
        self.raw.vertex_ids.push(id);
        let v = self.raw.points.len() - 1;
        self.corners.insert(key, v);
        Ok(v)
    }

    /// The uses of a `POLY_LOOP`'s straight sides, each side shared with the face across it.
    fn poly_loop(&mut self, lp: u64, la: &[Param], forward: bool) -> Result<Vec<Use>> {
        let mut corners = Vec::new();
        for p in la.get(1).and_then(Param::list).unwrap_or(&[]) {
            corners.push(self.corner(self.geo.reference(lp, Some(p))?)?);
        }
        corners.dedup();
        while corners.len() > 1 && corners.first() == corners.last() {
            corners.pop();
        }
        if corners.len() < 3 {
            return bad(lp, "a polygon needs three distinct corners");
        }
        let mut uses = Vec::new();
        // Each corner and the next, the last's next the first.
        for (&a, &b) in corners.iter().zip(corners.iter().cycle().skip(1)) {
            let edge = match self.sides.get(&(a.min(b), a.max(b))) {
                Some(&e) => e,
                None => {
                    let (Some(&pa), Some(&pb)) = (self.raw.points.get(a), self.raw.points.get(b)) else {
                        return Err(broken("a polygon corner"));
                    };
                    let Some(dir) = (pb - pa).unit() else {
                        return bad(lp, "a polygon side has no length");
                    };
                    self.raw.edges.push(RawEdge {
                        id: lp,
                        curve: Curve::Line(Line { origin: pa, dir }),
                        start: a,
                        end: b,
                        faces: Vec::new(),
                    });
                    let e = self.raw.edges.len() - 1;
                    self.sides.insert((a.min(b), a.max(b)), e);
                    e
                }
            };
            let along = self.raw.edges.get(edge).ok_or_else(|| broken("a polygon side"))?.start == a;
            uses.push(Use {
                edge,
                reversed: along != forward,
            });
        }
        if !forward {
            uses.reverse();
        }
        Ok(uses)
    }

    /// The edge uses of one `FACE_BOUND`, in the kernel's direction for the face.
    fn bound(&mut self, id: u64, flip_face: bool) -> Result<Bound> {
        let (kind, a) = self.params(id, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
        let outer = kind == "FACE_OUTER_BOUND";
        let lp = self.geo.reference(id, a.get(1))?;
        let orient = a.get(2).and_then(Param::boolean) != Some(false);
        let (lk, la) = self.params(lp, &["EDGE_LOOP", "VERTEX_LOOP", "POLY_LOOP"])?;
        match lk {
            "VERTEX_LOOP" => {
                return Ok(Bound::Vertex(
                    self.vertex(self.geo.reference(lp, la.get(1))?)?,
                ));
            }
            "POLY_LOOP" => {
                return Ok(Bound::Edges(
                    outer,
                    self.poly_loop(lp, la, orient != flip_face)?,
                ));
            }
            _ => {}
        }
        let mut uses = Vec::new();
        for oe in la.get(1).and_then(Param::list).unwrap_or(&[]) {
            let oe = self.geo.reference(lp, Some(oe))?;
            let (_, p) = self.params(oe, &["ORIENTED_EDGE"])?;
            let (edge, flip) = self.edge(self.geo.reference(oe, p.get(3))?)?;
            let along = p.get(4).and_then(Param::boolean) != Some(false);
            uses.push(Use {
                edge,
                reversed: ((along == orient) ^ flip_face) == flip,
            });
        }
        if !orient ^ flip_face {
            uses.reverse();
        }
        if uses.is_empty() {
            return bad(lp, "empty loop");
        }
        Ok(Bound::Edges(outer, self.expand(&uses)))
    }

    /// A seam from `from` to `apex` along `surface`, for a face that closes on its apex: a
    /// generator of a cone, a meridian of a sphere.
    fn seam(&mut self, id: u64, surface: &Surface, from: usize, apex: usize) -> Result<usize> {
        let (Some(&p), Some(&q)) = (self.raw.points.get(from), self.raw.points.get(apex)) else {
            return Err(broken("a seam's end"));
        };
        let curve = match surface {
            Surface::Cone(_) => match (q - p).unit() {
                Some(dir) => Curve::Line(Line { origin: p, dir }),
                None => return bad(id, "the vertex loop sits on its face's other bound"),
            },
            Surface::Sphere(s) => {
                let c = s.frame.origin;
                let x = p - c;
                let y = (q - c) - x.unit_or_zero() * (q - c).dot(x.unit_or_zero());
                match Frame::new(c, x.cross(y), x) {
                    Ok(frame) => Curve::Circle(Circle {
                        frame,
                        radius: s.radius,
                    }),
                    Err(_) => return bad(id, "the vertex loop is not at a pole of its sphere"),
                }
            }
            other => {
                return unsupported(id, &format!("VERTEX_LOOP bound on a {} face", other.kind()));
            }
        };
        self.raw.edges.push(RawEdge {
            id,
            curve,
            start: from,
            end: apex,
            faces: Vec::new(),
        });
        Ok(self.raw.edges.len() - 1)
    }

    /// `uses` with each use of a split edge as the uses of its halves.
    fn expand(&self, uses: &[Use]) -> Vec<Use> {
        let mut out = Vec::with_capacity(uses.len());
        for &u in uses {
            match self.splits.get(&u.edge) {
                Some(&[a, b]) => {
                    let halves = [Use { edge: a, ..u }, Use { edge: b, ..u }];
                    let ordered = if u.reversed {
                        [halves[1], halves[0]]
                    } else {
                        halves
                    };
                    out.extend(self.expand(&ordered));
                }
                None => out.push(u),
            }
        }
        out
    }

    /// Splits edge `e` at its parameter `t` into two edges meeting at a new vertex there, and
    /// gives every face read so far, and `bounds`, the halves in its place. Returns the vertex.
    fn split_edge(&mut self, e: usize, t: f64, bounds: &mut [(bool, Vec<Use>)]) -> Option<usize> {
        let edge = self.raw.edges.get(e)?.clone();
        let p = edge.curve.eval(t);
        self.raw.points.push(p);
        self.raw.vertex_ids.push(edge.id);
        let v = self.raw.points.len() - 1;
        self.raw.edges.push(RawEdge {
            end: v,
            ..edge.clone()
        });
        self.raw.edges.push(RawEdge { start: v, ..edge });
        let n = self.raw.edges.len();
        self.splits.insert(e, [n - 2, n - 1]);
        for f in 0..self.raw.faces.len() {
            let count = self.raw.faces.get(f).map_or(0, |face| face.bounds.len());
            for b in 0..count {
                let Some(old) = self.raw.faces.get(f).and_then(|face| face.bounds.get(b)) else { continue };
                let uses = self.expand(&old.1);
                if let Some(slot) = self.raw.faces.get_mut(f).and_then(|face| face.bounds.get_mut(b)) {
                    slot.1 = uses;
                }
            }
        }
        for (_, uses) in bounds.iter_mut() {
            *uses = self.expand(uses);
        }
        Some(v)
    }

    /// Where `uses` cross the angle `at`, as an edge and its parameter, `angle` giving a point's.
    fn crossing(
        &self,
        angle: &dyn Fn(Point3) -> f64,
        uses: &[Use],
        at: f64,
    ) -> Option<(usize, f64)> {
        let off = |p: Point3| {
            (angle(p) - at + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI
        };
        for u in uses {
            let e = self.raw.edges.get(u.edge)?;
            let (ps, pe) = (*self.raw.points.get(e.start)?, *self.raw.points.get(e.end)?);
            let (t0, t1) = edge_range(&e.curve, ps, pe, e.start == e.end)?;
            let n = 64;
            for k in 0..n {
                let (mut lo, mut hi) = (
                    t0 + (t1 - t0) * k as f64 / n as f64,
                    t0 + (t1 - t0) * (k + 1) as f64 / n as f64,
                );
                let (dl, dh) = (off(e.curve.eval(lo)), off(e.curve.eval(hi)));
                // A sign change away from the wrap at half a turn.
                if dl.signum() == dh.signum() || dl.abs().max(dh.abs()) > 1.0 {
                    continue;
                }
                for _ in 0..100 {
                    let mid = 0.5 * (lo + hi);
                    if off(e.curve.eval(mid)).signum() == dl.signum() {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                return Some((u.edge, 0.5 * (lo + hi)));
            }
        }
        None
    }

    /// The angle `uses` sweep about `frame`'s axis, from samples along each edge in its use's
    /// direction; `None` where an edge's span between its vertices cannot be found, or where the
    /// loop passes over the axis, as a seam through a pole does, and its angle jumps.
    fn swept(&self, frame: &Frame, uses: &[Use]) -> Option<f64> {
        self.swept_by(&about(frame), uses)
    }

    /// The angle `uses` sweep, `angle` giving a point's: [`Reader::swept`] about any measure of it.
    fn swept_by(&self, angle: &dyn Fn(Point3) -> f64, uses: &[Use]) -> Option<f64> {
        let mut total = 0.0;
        for u in uses {
            let e = self.raw.edges.get(u.edge)?;
            let (ps, pe) = (*self.raw.points.get(e.start)?, *self.raw.points.get(e.end)?);
            let (t0, t1) = edge_range(&e.curve, ps, pe, e.start == e.end)?;
            let n = 64;
            let mut ts: Vec<f64> = (0..=n)
                .map(|k| t0 + (t1 - t0) * k as f64 / n as f64)
                .collect();
            if u.reversed {
                ts.reverse();
            }
            for w in ts.windows(2) {
                let &[a, z] = w else { continue };
                let d = angle(e.curve.eval(z)) - angle(e.curve.eval(a));
                let d = (d + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                if d.abs() > 1.0 {
                    return None;
                }
                total += d;
            }
        }
        Some(total)
    }

    /// A face of revolution (cylinder, cone, sphere, torus) with two loops that each go once round
    /// its axis, the ways opposite, and no seam: those two become one loop, the first, a seam
    /// along the surface to a vertex of the second at the same angle, the second, and the seam
    /// back. Its pcurves then close in the parameter plane, as the kernel's faces do. Other loops,
    /// holes through the band, stay. `reversed` is the face's orientation against its surface's.
    /// `None` when the face is not such a band, or no vertex of the second lies on the seam's line.
    fn seamed_band(
        &mut self,
        id: u64,
        surface: &Surface,
        reversed: bool,
        bounds: &mut Vec<(bool, Vec<Use>)>,
    ) -> Option<()> {
        let frame = match surface {
            Surface::Cylinder(c) => c.frame,
            Surface::Cone(c) => c.frame,
            Surface::Sphere(s) => s.frame,
            Surface::Torus(t) => t.frame,
            _ => return None,
        };
        let full = |w: f64| (w.abs() - std::f64::consts::TAU).abs() < 0.1;
        let mut winding = Vec::new();
        for (k, (_, uses)) in bounds.iter().enumerate() {
            let w = self.swept(&frame, uses)?;
            if full(w) {
                winding.push((k, w));
            }
        }
        let &[(ka, wa), (kb, wb)] = winding.as_slice() else {
            return None;
        };
        if wa * wb >= 0.0 {
            return None;
        }
        let (Some((_, a)), Some((_, b))) = (bounds.get(ka), bounds.get(kb)) else {
            return None;
        };
        // A vertex of each loop on one line of the surface: a cylinder's lines run along its
        // axis, a cone's through its apex.
        let on_line = |p: Point3, q: Point3| {
            let (lp, lq) = (frame.to_local(p), frame.to_local(q));
            let radial = |l: Point3| (l.x * l.x + l.y * l.y).sqrt();
            let (rp, rq) = (radial(lp), radial(lq));
            if rp <= tol::LINEAR || rq <= tol::LINEAR {
                return false;
            }
            let (ux, uy) = (lp.x / rp, lp.y / rp);
            (lq.x - ux * rq).hypot(lq.y - uy * rq) <= tol::LINEAR
        };
        let aligned = a
            .iter()
            .enumerate()
            .flat_map(|(i, ua)| b.iter().enumerate().map(move |(j, ub)| (i, j, ua, ub)))
            .find(|&(_, _, ua, ub)| match (use_point(&self.raw, ua), use_point(&self.raw, ub)) {
                (Some(p), Some(q)) => on_line(p, q),
                _ => false,
            })
            .map(|(i, j, _, _)| (i, j));
        let (a, b, ia, ib) = match aligned {
            Some((i, j)) => (a.clone(), b.clone(), i, j),
            None => {
                // No vertex of the second loop at the first's angle: split the second's edge
                // there, in every face that uses it.
                let va = use_start(&self.raw, a.first()?)?;
                let la = frame.to_local(*self.raw.points.get(va)?);
                let (e, t) = self.crossing(&about(&frame), b, la.y.atan2(la.x))?;
                let v = self.split_edge(e, t, bounds)?;
                let (a, b) = (bounds.get(ka)?.1.clone(), bounds.get(kb)?.1.clone());
                let ia = a.iter().position(|u| use_start(&self.raw, u) == Some(va))?;
                let ib = b.iter().position(|u| use_start(&self.raw, u) == Some(v))?;
                (a, b, ia, ib)
            }
        };
        let (va, vb) = (use_start(&self.raw, a.get(ia)?)?, use_start(&self.raw, b.get(ib)?)?);
        let (pa, pb) = (*self.raw.points.get(va)?, *self.raw.points.get(vb)?);
        // The seam's line at the vertices' angle about the axis.
        let la = frame.to_local(pa);
        let radial = (frame.x * la.x + frame.y * la.y).unit()?;
        let meridian = |centre: Point3, radius: f64, toward_v: bool| {
            // A circle whose parameter is the surface's v there, or runs against it.
            let normal = radial.cross(frame.z) * if toward_v { 1.0 } else { -1.0 };
            Frame::new(centre, normal, radial)
                .ok()
                .map(|frame| Curve::Circle(Circle { frame, radius }))
        };
        let curve = match surface {
            Surface::Cylinder(_) | Surface::Cone(_) => Curve::Line(Line {
                origin: pa,
                dir: (pb - pa).unit()?,
            }),
            // A latitude band: the meridian between the two loops, which v crosses one way.
            Surface::Sphere(s) => {
                let v = |p: Point3| {
                    frame
                        .to_local(p)
                        .z
                        .atan2(frame.to_local(p).x.hypot(frame.to_local(p).y))
                };
                meridian(s.frame.origin, s.radius, v(pb) > v(pa))?
            }
            // A ring of the tube: the first loop's material lies on its left, seen from outside,
            // which is the way v grows from it when it runs the way u grows on a face along its
            // surface's normal.
            Surface::Torus(t) => {
                let toward_v = (wa > 0.0) != reversed;
                meridian(frame.origin + radial * t.major, t.minor, toward_v)?
            }
            _ => return None,
        };
        self.raw.edges.push(RawEdge {
            id,
            curve,
            start: va,
            end: vb,
            faces: Vec::new(),
        });
        let seam = self.raw.edges.len() - 1;
        let mut out: Vec<Use> = a.iter().skip(ia).chain(a.iter().take(ia)).copied().collect();
        out.push(Use {
            edge: seam,
            reversed: false,
        });
        out.extend(b.iter().skip(ib).chain(b.iter().take(ib)).copied());
        out.push(Use {
            edge: seam,
            reversed: true,
        });
        bounds.remove(ka.max(kb));
        bounds.remove(ka.min(kb));
        bounds.insert(0, (true, out));
        Some(())
    }

    /// A spline face closed round one of its parameters with two loops that each go once round
    /// it, the ways opposite, and no seam: [`Reader::seamed_band`] with that parameter for the
    /// angle, and the surface's own iso curve for the seam. `None` when the face is not such a
    /// band.
    fn seamed_spline_band(
        &mut self,
        id: u64,
        surface: &Surface,
        bounds: &mut Vec<(bool, Vec<Use>)>,
    ) -> Option<()> {
        let Surface::Nurbs(n) = surface else {
            return None;
        };
        let (in_v, period) = match (surface.u_period(), surface.v_period()) {
            (None, Some(p)) => (true, p),
            (Some(p), None) => (false, p),
            _ => return None,
        };
        const TAU: f64 = std::f64::consts::TAU;
        let angle = |p: Point3| {
            let uv = surface.project(p);
            (if in_v { uv.y } else { uv.x }) / period * TAU
        };
        let full = |w: f64| (w.abs() - TAU).abs() < 0.1;
        let mut winding = Vec::new();
        for (k, (_, uses)) in bounds.iter().enumerate() {
            let w = self.swept_by(&angle, uses)?;
            if full(w) {
                winding.push((k, w));
            }
        }
        let &[(ka, wa), (kb, wb)] = winding.as_slice() else {
            return None;
        };
        if wa * wb >= 0.0 {
            return None;
        }
        // The seam runs where it is farthest round from every vertex of the two loops, the middle
        // of the widest gap between their angles, so that splitting a loop for it there leaves no
        // sliver beside a vertex a file placed a little off the line.
        let mut angles: Vec<f64> = [ka, kb]
            .iter()
            .filter_map(|&k| bounds.get(k))
            .flat_map(|(_, uses)| uses.iter())
            .filter_map(|u| use_point(&self.raw, u))
            .map(|p| angle(p).rem_euclid(TAU))
            .collect();
        angles.sort_by(f64::total_cmp);
        // Each angle and the next round, the last's next the first.
        let at = angles
            .iter()
            .zip(angles.iter().cycle().skip(1))
            .map(|(&lo, &hi)| {
                let gap = (hi - lo).rem_euclid(TAU);
                let gap = if gap == 0.0 { TAU } else { gap };
                (gap, lo + 0.5 * gap)
            })
            .max_by(|x, y| x.0.total_cmp(&y.0))?
            .1;
        let seam = Curve::Nurbs(Box::new(n.iso_curve(!in_v, at / TAU * period).ok()?));
        let on_seam = |raw: &Raw, u: &Use| {
            use_point(raw, u).is_some_and(|q| seam.eval(param_of(&seam, q)).dist(q) <= tol::LINEAR)
        };
        // A vertex of each loop on the seam, the loop's edge split there where it has none.
        let mut ends = [0usize; 2];
        for (end, k) in ends.iter_mut().zip([ka, kb]) {
            let found = bounds.get(k)?.1.iter().find(|u| on_seam(&self.raw, u)).copied();
            *end = match found {
                Some(u) => use_start(&self.raw, &u)?,
                None => {
                    let uses = bounds.get(k)?.1.clone();
                    let (e, t) = self.crossing(&angle, &uses, at)?;
                    self.split_edge(e, t, bounds)?
                }
            };
        }
        let [va, vb] = ends;
        let (a, b) = (bounds.get(ka)?.1.clone(), bounds.get(kb)?.1.clone());
        let ia = a.iter().position(|u| use_start(&self.raw, u) == Some(va))?;
        let ib = b.iter().position(|u| use_start(&self.raw, u) == Some(vb))?;
        // An edge runs the way its curve does: from whichever end comes first along the iso.
        let back = param_of(&seam, *self.raw.points.get(va)?) > param_of(&seam, *self.raw.points.get(vb)?);
        let (start, end) = if back { (vb, va) } else { (va, vb) };
        self.raw.edges.push(RawEdge {
            id,
            curve: seam,
            start,
            end,
            faces: Vec::new(),
        });
        let edge = self.raw.edges.len() - 1;
        let mut out: Vec<Use> = a.iter().skip(ia).chain(a.iter().take(ia)).copied().collect();
        out.push(Use {
            edge,
            reversed: back,
        });
        out.extend(b.iter().skip(ib).chain(b.iter().take(ib)).copied());
        out.push(Use {
            edge,
            reversed: !back,
        });
        bounds.remove(ka.max(kb));
        bounds.remove(ka.min(kb));
        bounds.insert(0, (true, out));
        Some(())
    }

    /// A sphere or cone face with one loop that goes once round its axis and no seam: a cap over
    /// the pole on the loop's left seen from outside, or over the cone's apex, the only side that
    /// closes. As a vertex loop there would, it gives the loop a seam up to the pole and back. A
    /// loop through a pole of a sphere's frame, such as a hemisphere's great circle, is a latitude
    /// of the frame about its circle's normal, which then becomes the surface's frame. Other loops,
    /// holes through the cap, stay. False when the face is not such a cap.
    fn capped(
        &mut self,
        id: u64,
        surface: &mut Surface,
        reversed: bool,
        bounds: &mut [(bool, Vec<Use>)],
    ) -> Result<bool> {
        let (base, sphere) = match &*surface {
            Surface::Sphere(s) => (s.frame, true),
            Surface::Cone(c) if c.half_angle.abs() > tol::ANGULAR => (c.frame, false),
            _ => return Ok(false),
        };
        let mut found = None;
        for (k, (_, uses)) in bounds.iter().enumerate() {
            // A loop through the apex, or a pole, already closes on it by its own seam.
            let at_pole = uses.iter().any(|u| {
                use_point(&self.raw, u).is_some_and(|p| {
                    let l = base.to_local(p);
                    l.x.hypot(l.y) <= tol::LINEAR
                })
            });
            let frame = match uses.as_slice() {
                _ if !at_pole => base,
                [u] if sphere => match self.raw.edges.get(u.edge).map(|e| &e.curve) {
                    Some(Curve::Circle(c)) => match Frame::new(base.origin, c.frame.z, c.frame.x) {
                        Ok(f) => f,
                        Err(_) => continue,
                    },
                    _ => continue,
                },
                _ => continue,
            };
            let Some(w) = self.swept(&frame, uses) else {
                continue;
            };
            if (w.abs() - std::f64::consts::TAU).abs() < 0.1 {
                if found.is_some() {
                    return Ok(false);
                }
                found = Some((k, frame, w));
            }
        }
        let Some((k, frame, w)) = found else {
            return Ok(false);
        };
        let Some(&first) = bounds.get(k).and_then(|(_, uses)| uses.first()) else {
            return Err(broken("a cap's loop"));
        };
        let pole = match surface {
            Surface::Sphere(s) => {
                s.frame = frame;
                // Along the outward normal, a loop running the way u grows has +z on its left.
                let up = (w > 0.0) != reversed;
                frame.origin + frame.z * if up { s.radius } else { -s.radius }
            }
            Surface::Cone(c) => c.frame.origin - c.frame.z * (c.radius / c.half_angle.tan()),
            _ => return Ok(false),
        };
        self.raw.points.push(pole);
        self.raw.vertex_ids.push(id);
        let apex = self.raw.points.len() - 1;
        let from = use_start(&self.raw, &first).ok_or_else(|| broken("a cap's edge"))?;
        let seam = self.seam(id, surface, from, apex)?;
        if let Some((_, uses)) = bounds.get_mut(k) {
            uses.extend([Use { edge: seam, reversed: false }, Use { edge: seam, reversed: true }]);
        }
        Ok(true)
    }

    fn face(&mut self, id: u64, flip_face: bool) -> Result<usize> {
        let (_, a) = self.params(id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
        let name = a.first().and_then(Param::string).unwrap_or("").to_string();
        let mut surface = self.geo.surface(self.geo.reference(id, a.get(2))?)?;
        let same_sense = a.get(3).and_then(Param::boolean) != Some(false);
        let mut bounds = Vec::new();
        let mut apexes = Vec::new();
        for bd in a.get(1).and_then(Param::list).unwrap_or(&[]) {
            match self.bound(self.geo.reference(id, Some(bd))?, flip_face)? {
                Bound::Edges(outer, uses) => bounds.push((outer, uses)),
                Bound::Vertex(v) => apexes.push(v),
            }
        }
        let (mut apexes, points): (Vec<usize>, Vec<usize>) = apexes
            .into_iter()
            .partition(|&v| self.raw.points.get(v).is_some_and(|&p| closes_at(&surface, p)));
        for &v in &points {
            let p = *self.raw.points.get(v).ok_or_else(|| broken("a vertex loop's vertex"))?;
            let uv = surface.project(p);
            if surface.eval(uv.x, uv.y).dist(p) > tol::LINEAR {
                return bad(id, "a VERTEX_LOOP off its face's surface");
            }
        }
        // A whole sphere, bounded only by a vertex loop at one pole: the kernel's seam, a
        // meridian from the other pole to it, used both ways.
        if let ([apex], 0, Surface::Sphere(sp)) = (apexes.as_slice(), bounds.len(), &surface) {
            let (c, p) = (sp.frame.origin, *self.raw.points.get(*apex).ok_or_else(|| broken("a pole"))?);
            let other = c * 2.0 - p;
            self.raw.points.push(other);
            self.raw.vertex_ids.push(id);
            let from = self.raw.points.len() - 1;
            let x = (other - c).unit_or_zero();
            let Ok(frame) = Frame::new(c, x.cross(sp.frame.x), x) else {
                return bad(id, "the vertex loop is not at a pole of its sphere");
            };
            self.raw.edges.push(RawEdge {
                id,
                curve: Curve::Circle(Circle {
                    frame,
                    radius: sp.radius,
                }),
                start: from,
                end: *apex,
                faces: Vec::new(),
            });
            let seam = self.raw.edges.len() - 1;
            bounds.push((
                true,
                vec![
                    Use {
                        edge: seam,
                        reversed: false,
                    },
                    Use {
                        edge: seam,
                        reversed: true,
                    },
                ],
            ));
            apexes.clear();
        }
        // A face that closes on a vertex is, in the kernel, its other bound followed by a seam
        // up to the vertex and back, the pole between the seam's two uses.
        match (apexes.as_slice(), bounds.len()) {
            ([], _) => {}
            (&[apex], 1) => {
                let Some(from) = bounds.first().and_then(|(_, uses)| uses.first()).and_then(|u| use_start(&self.raw, u)) else {
                    return Err(broken("a bound's first edge"));
                };
                let seam = self.seam(id, &surface, from, apex)?;
                if let Some((_, uses)) = bounds.first_mut() {
                    uses.extend([Use { edge: seam, reversed: false }, Use { edge: seam, reversed: true }]);
                }
            }
            _ => {
                return unsupported(
                    id,
                    "a vertex loop other than one face's apex beside one other bound",
                );
            }
        }
        if bounds.is_empty() {
            return unsupported(id, "face without bounds");
        }
        let reversed = !same_sense ^ flip_face;
        if self
            .seamed_band(id, &surface, reversed, &mut bounds)
            .is_none()
            && self.seamed_spline_band(id, &surface, &mut bounds).is_none()
            && apexes.is_empty()
        {
            self.capped(id, &mut surface, reversed, &mut bounds)?;
        }
        let f = self.raw.faces.len();
        for (_, uses) in &bounds {
            for u in uses {
                self.raw.edges.get_mut(u.edge).ok_or_else(|| broken("an edge"))?.faces.push(f);
            }
        }
        self.raw.faces.push(RawFace {
            id,
            name,
            surface,
            reversed: !same_sense ^ flip_face,
            bounds,
            points,
        });
        Ok(f)
    }

    fn solid(&mut self, id: u64) -> Result<()> {
        let Some(e) = self.geo.file.get(id) else {
            return bad(id, "missing instance");
        };
        let Some(first) = e.records().first() else {
            return bad(id, "an instance with no record");
        };
        let (kind, a) = (&first.name, &first.params);
        let mut shells: Vec<u64> = Vec::new();
        match kind.as_str() {
            "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => {
                shells.push(self.geo.reference(id, a.get(1))?)
            }
            "BREP_WITH_VOIDS" => {
                shells.push(self.geo.reference(id, a.get(1))?);
                for v in a.get(2).and_then(Param::list).unwrap_or(&[]) {
                    shells.push(self.geo.reference(id, Some(v))?);
                }
            }
            other => return unsupported(id, &format!("solid {other}")),
        }
        for mut sh in shells {
            let mut flip = false;
            if let Some(p) = self
                .geo
                .file
                .get(sh)
                .and_then(|e| e.part("ORIENTED_CLOSED_SHELL"))
            {
                flip = p.get(3).and_then(Param::boolean) == Some(false);
                sh = self.geo.reference(sh, p.get(2))?;
            }
            let (_, p) = self.params(sh, &["CLOSED_SHELL"])?;
            let mut faces = Vec::new();
            for f in p.get(1).and_then(Param::list).unwrap_or(&[]) {
                faces.push(self.face(self.geo.reference(sh, Some(f))?, flip)?);
            }
            self.raw.shells.push(faces);
        }
        Ok(())
    }
}

/// Whether a face on `s` bounded by a `VERTEX_LOOP` at `p` closes there: at a cone's apex or a
/// sphere's pole, where every parameter line meets.
fn closes_at(s: &Surface, p: Point3) -> bool {
    let near = |q: Point3| q.dist(p) <= tol::LINEAR;
    match s {
        Surface::Cone(c) => near(c.frame.origin - c.frame.z * (c.radius / c.half_angle.tan())),
        Surface::Sphere(sp) => {
            let (o, z) = (sp.frame.origin, sp.frame.z);
            near(o + z * sp.radius) || near(o - z * sp.radius)
        }
        _ => false,
    }
}

/// A solid read, its faces' STEP ids and names in order, and how far healing moved its geometry.
pub(super) type ReadSolid = (Brep, Vec<(u64, String)>, Option<heal::Healed>);

/// Builds the solid of a `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS`, healed where its geometry
/// disagrees with itself, and checked. Returns the solid, its faces' STEP ids and names in order,
/// and how far healing moved its geometry.
#[allow(clippy::disallowed_types)] pub(super) fn solid(geo: &mut Geometry, id: u64) -> Result<ReadSolid> { // D26: BTree* in hardening, proven by digests
    if geo
        .file
        .get(id)
        .is_some_and(|e| e.part("TESSELLATED_SOLID").is_some())
    {
        return tessellated(geo, id).map(|b| {
            let names = vec![(0, String::new()); b.face_count()];
            (b, names, None)
        });
    }
    let mut rd = Reader {
        geo,
        raw: Raw::default(),
        vertices: HashMap::new(),
        edges: HashMap::new(),
        corners: HashMap::new(),
        sides: HashMap::new(),
        splits: HashMap::new(),
    };
    rd.solid(id)?;
    let file = join_contacts(without_split(rd.raw, &rd.splits)?)?;
    let mut raw = file.clone();
    let names = || file.faces.iter().map(|f| (f.id, f.name.clone())).collect();
    let within = rd.geo.uncertainty().filter(|&u| u > tol::LINEAR);
    if let Some(b) = heal::as_written(&file, within, |r, t| assemble(r, &t.ranges, Some(t)).ok()) {
        return Ok((b, names(), None));
    }
    let exact = heal::heal(&mut raw)
        .and_then(|(ranges, healed)| assemble(&raw, &ranges, None).map(|b| (b, healed)));
    // Healing past the file's stated uncertainty changed the shape: keep the file's geometry then.
    let why = match exact {
        Ok((b, healed)) if within.is_none_or(|w| healed.as_ref().is_none_or(|h| h.moved <= w)) => {
            return Ok((b, names(), healed));
        }
        other => other,
    };
    // Geometry that cannot be made to meet within the kernel tolerance is kept as the file has
    // it, its edges and vertices given the tolerances it does meet within, whatever the file
    // states.
    let tolerant = |raw: &Raw| {
        let mut raw = raw.clone();
        heal::tolerate(&mut raw, within)
            .map_err(|t| format!("kept as the file has it, {t}"))
            .and_then(|t| {
                assemble(&raw, &t.ranges, Some(&t))
                    .map_err(|e| format!("kept as the file has it, with local tolerances: {e}"))
            })
    };
    match (tolerant(&file), why) {
        (Ok(b), _) => Ok((b, names(), None)),
        (Err(_), Ok((b, healed))) => Ok((b, names(), healed)),
        (Err(t), Err(e)) => {
            // Healed where it can be, and held by tolerances where it cannot.
            let mut loose = file.clone();
            let healed = heal::heal_loosely(&mut loose);
            let l = match (tolerant(&loose), healed.as_ref(), within) {
                (Ok(_), Some(h), Some(within)) if h.moved > within => format!(
                    "healing would move {} {:.1e} mm, more than the file's {within:.1e} mm",
                    h.what, h.moved
                ),
                (Ok(b), _, _) => return Ok((b, names(), healed)),
                (Err(l), _, _) => l,
            };
            Err(Error::InvalidInput(format!(
                "{e}; {t}; healed where it can be and {l}"
            )))
        }
    }
}

/// An AP242 `TESSELLATED_SOLID`: each triangle of its `TRIANGULATED_FACE`s and
/// `COMPLEX_TRIANGULATED_FACE`s a plane face, the triangles sharing the edges between their
/// corners. The triangles run counter-clockwise seen from outside, as the schema has them.
#[allow(clippy::disallowed_types)] fn tessellated(geo: &mut Geometry, id: u64) -> Result<Brep> { // D26: BTree* in hardening, proven by digests
    let unit = geo.length_unit();
    let Some(solid) = geo
        .file
        .get(id)
        .and_then(|e| e.part("TESSELLATED_SOLID"))
        .map(<[Param]>::to_vec)
    else {
        return bad(id, "expected a tessellated solid");
    };
    // Corners index their own face's list; one list's point is one vertex wherever it is used.
    let mut points: Vec<Point3> = Vec::new();
    let mut index: HashMap<(u64, usize), usize> = HashMap::new();
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    for item in solid.get(1).and_then(Param::list).unwrap_or(&[]) {
        let fid = geo.reference(id, Some(item))?;
        let entity = geo.file.get(fid);
        let (face, complex) = match (
            entity.and_then(|e| e.part("TRIANGULATED_FACE")),
            entity.and_then(|e| e.part("COMPLEX_TRIANGULATED_FACE")),
        ) {
            (Some(f), _) => (f.to_vec(), false),
            (None, Some(f)) => (f.to_vec(), true),
            _ => return unsupported(fid, "a tessellated face other than a triangulated one"),
        };
        let list = geo.reference(fid, face.get(1))?;
        let Some(coords) = geo
            .file
            .get(list)
            .and_then(|e| e.part("COORDINATES_LIST"))
            .map(<[Param]>::to_vec)
        else {
            return bad(list, "expected a coordinates list");
        };
        let mut xyz = Vec::new();
        for c in coords.get(2).and_then(Param::list).unwrap_or(&[]) {
            match *geo.numbers(list, Some(c))?.as_slice() {
                [x, y, z] if x.is_finite() && y.is_finite() && z.is_finite() => xyz.push(Point3::new(x, y, z) * unit),
                _ => return bad(list, "expected three finite coordinates"),
            }
        }
        // Triangle corners index the face's own points, which index the coordinates list.
        let own: Vec<usize> = face
            .get(5)
            .and_then(Param::list)
            .unwrap_or(&[])
            .iter()
            .map(|p| part21::integral(p).map_or(0, |i| i as usize))
            .collect();
        let corner = |k: i64| -> Option<usize> {
            let k = usize::try_from(k).ok()?.checked_sub(1)?;
            let i = if own.is_empty() {
                k
            } else {
                own.get(k)?.checked_sub(1)?
            };
            (i < xyz.len()).then_some(i)
        };
        let lists = |at: usize| -> Vec<Vec<i64>> {
            face.get(at)
                .and_then(Param::list)
                .unwrap_or(&[])
                .iter()
                .map(|l| {
                    l.list()
                        .unwrap_or(&[])
                        .iter()
                        .filter_map(part21::integral)
                        .collect()
                })
                .collect()
        };
        let tris = if complex {
            strips_and_fans(&lists(6), &lists(7))
        } else {
            lists(6)
        };
        for ks in tris {
            let idx: Option<Vec<usize>> = ks.iter().map(|&k| corner(k)).collect();
            let Some(idx) = idx.filter(|v| v.len() == 3) else {
                return bad(fid, "a triangle needs three corners in its list");
            };
            let mut corners = [0; 3];
            for (corner, &i) in corners.iter_mut().zip(&idx) {
                // `corner` has checked `i` against the list.
                let p = *xyz.get(i).ok_or_else(|| broken("a coordinate"))?;
                *corner = *index.entry((list, i)).or_insert_with(|| {
                    points.push(p);
                    points.len() - 1
                });
            }
            triangles.push(corners);
        }
    }
    if triangles.is_empty() {
        return bad(id, "a tessellated solid with no triangles");
    }
    crate::build::from_indexed(&points, &triangles)
}

/// The triangles of a `COMPLEX_TRIANGULATED_FACE`'s strips and fans, each keeping the face's
/// turn: in a strip each point after the first two makes a triangle with the two before it, every
/// second one taken the other way round; in a fan each point after the second makes one with the
/// point before it and the first. A triangle with a corner twice, which joins two strips, has no
/// area and is left out.
fn strips_and_fans(strips: &[Vec<i64>], fans: &[Vec<i64>]) -> Vec<Vec<i64>> {
    let strip = strips.iter().flat_map(|s| {
        s.windows(3).enumerate().filter_map(|(i, w)| match (i % 2, w) {
            (0, &[a, b, c]) => Some([a, b, c]),
            (_, &[a, b, c]) => Some([b, a, c]),
            _ => None,
        })
    });
    let fan = fans.iter().flat_map(|f| {
        f.windows(2).skip(1).filter_map(move |w| match (f.first(), w) {
            (Some(&o), &[a, b]) => Some([o, a, b]),
            _ => None,
        })
    });
    strip
        .chain(fan)
        .filter(|&[a, b, c]| a != b && b != c && a != c)
        .map(|t| t.to_vec())
        .collect()
}

/// Every face of the shells in `file`, each with its own vertices and edges, sewn into one solid:
/// vertices within `tolerance` of each other become one, edges that then share their ends and lie
/// together become one, faces are turned so each shared edge runs opposite ways in its two faces,
/// and the solid is healed onto its faces' surfaces as a read one is.
#[allow(clippy::disallowed_types)] pub(super) fn sew(geo: &mut Geometry, tolerance: f64) -> Result<Brep> { // D26: BTree* in hardening, proven by digests
    let shells: Vec<u64> = part21::entities(geo.file)
        .filter(|(_, e)| matches!(part21::kind(e), "OPEN_SHELL" | "CLOSED_SHELL"))
        .map(|(id, _)| id)
        .collect();
    let mut rd = Reader {
        geo,
        raw: Raw::default(),
        vertices: HashMap::new(),
        edges: HashMap::new(),
        corners: HashMap::new(),
        sides: HashMap::new(),
        splits: HashMap::new(),
    };
    let mut faces = Vec::new();
    for sh in shells {
        let (_, p) = rd.params(sh, &["OPEN_SHELL", "CLOSED_SHELL"])?;
        for f in p.get(1).and_then(Param::list).unwrap_or(&[]) {
            faces.push(rd.face(rd.geo.reference(sh, Some(f))?, false)?);
        }
    }
    if faces.is_empty() {
        return Err(Error::InvalidInput(
            "STEP: the file holds no faces to sew".into(),
        ));
    }
    let mut raw = without_split(rd.raw, &rd.splits)?;
    // Points within the tolerance of each other are one, at their mean.
    let n = raw.points.len();
    let mut rep: Vec<usize> = (0..n).collect();
    /// The representative of `i`, which then points straight at it.
    fn root(rep: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while let Some(&up) = rep.get(r)
            && up != r
        {
            r = up;
        }
        if let Some(slot) = rep.get_mut(i) {
            *slot = r;
        }
        r
    }
    for (i, &pi) in raw.points.iter().enumerate() {
        for (j, &pj) in raw.points.iter().enumerate().skip(i + 1) {
            if pi.dist(pj) <= tolerance {
                let (a, b) = (root(&mut rep, i), root(&mut rep, j));
                if a != b
                    && let Some(slot) = rep.get_mut(b)
                {
                    *slot = a;
                }
            }
        }
    }
    let mut sum: HashMap<usize, (Point3, f64)> = HashMap::new();
    for (i, &p) in raw.points.iter().enumerate() {
        let r = root(&mut rep, i);
        let e = sum.entry(r).or_insert((Point3::ZERO, 0.0));
        e.0 += p;
        e.1 += 1.0;
    }
    for (i, p) in raw.points.iter_mut().enumerate() {
        let r = root(&mut rep, i);
        let &(s, k) = sum.get(&r).ok_or_else(|| broken("a point"))?;
        *p = s * (1.0 / k);
    }
    // Edges that now run between one pair of points, and lie together along it, are one: the
    // first stands for the rest, and their uses are turned to its direction.
    let ends = |e: &RawEdge, rep: &mut [usize]| (root(rep, e.start), root(rep, e.end));
    let mut keep: Vec<usize> = (0..raw.edges.len()).collect();
    let mut turned = vec![false; raw.edges.len()];
    let mid = |e: &RawEdge| {
        let (s, z) = (*raw.points.get(e.start)?, *raw.points.get(e.end)?);
        let t = param_of(&e.curve, s.lerp(z, 0.5));
        Some(e.curve.eval(t))
    };
    for (i, ei) in raw.edges.iter().enumerate() {
        if keep.get(i) != Some(&i) {
            continue;
        }
        let (a, b) = ends(ei, &mut rep);
        let mi = mid(ei).ok_or_else(|| broken("an edge's end"))?;
        for (j, ej) in raw.edges.iter().enumerate().skip(i + 1) {
            if keep.get(j) != Some(&j) {
                continue;
            }
            let (c, d) = ends(ej, &mut rep);
            let same = (a, b) == (c, d);
            if !(same || (a, b) == (d, c)) || mid(ej).ok_or_else(|| broken("an edge's end"))?.dist(mi) > tolerance {
                continue;
            }
            if let (Some(k), Some(tu)) = (keep.get_mut(j), turned.get_mut(j)) {
                *k = i;
                *tu = !same;
            }
        }
    }
    for f in &mut raw.faces {
        for (_, uses) in &mut f.bounds {
            for u in uses.iter_mut() {
                let (&to, &turn) = keep.get(u.edge).zip(turned.get(u.edge)).ok_or_else(|| broken("an edge"))?;
                if to != u.edge {
                    u.reversed ^= turn;
                    u.edge = to;
                }
            }
        }
    }
    for e in &mut raw.edges {
        e.start = root(&mut rep, e.start);
        e.end = root(&mut rep, e.end);
    }
    // Each face's neighbours across its edges, and whether they agree: a shared edge runs one way
    // in one face and the other way in the other.
    let mut uses: HashMap<usize, Vec<(usize, bool)>> = HashMap::new();
    for (fi, f) in raw.faces.iter().enumerate() {
        for (_, us) in &f.bounds {
            for u in us {
                uses.entry(u.edge).or_default().push((fi, u.reversed));
            }
        }
    }
    let mut flip: Vec<Option<bool>> = vec![None; raw.faces.len()];
    for start in 0..raw.faces.len() {
        if flip.get(start).is_none_or(Option::is_some) {
            continue;
        }
        if let Some(slot) = flip.get_mut(start) {
            *slot = Some(false);
        }
        let mut stack = vec![start];
        while let Some(f) = stack.pop() {
            let mine = flip.get(f).copied().flatten().unwrap_or(false);
            let Some(face) = raw.faces.get(f) else { continue };
            for (_, us) in &face.bounds {
                for u in us {
                    for &(g, rev) in uses.get(&u.edge).into_iter().flatten() {
                        let Some(slot) = flip.get_mut(g) else { continue };
                        if g == f || slot.is_some() {
                            continue;
                        }
                        // Agreeing faces use the edge opposite ways once both are turned as set.
                        let want = (u.reversed ^ mine) == rev;
                        *slot = Some(want);
                        stack.push(g);
                    }
                }
            }
        }
    }
    for (f, t) in raw.faces.iter_mut().zip(&flip) {
        if *t == Some(true) {
            f.reversed = !f.reversed;
            for (_, us) in &mut f.bounds {
                us.reverse();
                for u in us.iter_mut() {
                    u.reversed = !u.reversed;
                }
            }
        }
    }
    // Only the edges and points still used are kept, renumbered.
    // Each old edge's place among those kept, `usize::MAX` for one merged into another.
    let mut edge_at = Vec::with_capacity(raw.edges.len());
    let mut edges = Vec::new();
    for (i, e) in raw.edges.drain(..).enumerate() {
        if keep.get(i) == Some(&i) {
            edge_at.push(edges.len());
            edges.push(e);
        } else {
            edge_at.push(usize::MAX);
        }
    }
    raw.edges = edges;
    for f in &mut raw.faces {
        for (_, us) in &mut f.bounds {
            for u in us.iter_mut() {
                u.edge = *edge_at.get(u.edge).ok_or_else(|| broken("an edge"))?;
            }
        }
    }
    // Each point still used, renumbered in the order first met.
    let mut point_at = vec![usize::MAX; raw.points.len()];
    let (mut points, mut ids) = (Vec::new(), Vec::new());
    let mut renumber = |v: usize| -> Result<usize> {
        let slot = point_at.get_mut(v).ok_or_else(|| broken("a vertex"))?;
        if *slot == usize::MAX {
            *slot = points.len();
            points.push(*raw.points.get(v).ok_or_else(|| broken("a point"))?);
            ids.push(*raw.vertex_ids.get(v).ok_or_else(|| broken("a vertex id"))?);
        }
        Ok(*slot)
    };
    let mut ends_at = Vec::with_capacity(raw.edges.len());
    for e in &raw.edges {
        ends_at.push((renumber(e.start)?, renumber(e.end)?));
    }
    let mut faces_at = Vec::with_capacity(raw.faces.len());
    for f in &raw.faces {
        faces_at.push(f.points.iter().map(|&p| renumber(root(&mut rep, p))).collect::<Result<Vec<usize>>>()?);
    }
    for (e, (s, z)) in raw.edges.iter_mut().zip(ends_at) {
        e.start = s;
        e.end = z;
    }
    for (f, ps) in raw.faces.iter_mut().zip(faces_at) {
        f.points = ps;
    }
    raw.points = points;
    raw.vertex_ids = ids;
    raw.edges.iter_mut().for_each(|e| e.faces.clear());
    for (fi, f) in raw.faces.iter().enumerate() {
        for (_, us) in &f.bounds {
            for u in us {
                raw.edges.get_mut(u.edge).ok_or_else(|| broken("an edge"))?.faces.push(fi);
            }
        }
    }
    raw.shells = vec![(0..raw.faces.len()).collect()];
    let (ranges, _) = heal::heal(&mut raw)?;
    assemble(&raw, &ranges, None)
}

