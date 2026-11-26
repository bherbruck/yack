// yack: a solid's raw topology (points, edges, faces and their uses) built into an arena.
// Adapted from springrep@a03e8a1 src/step/topology.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from step/topology.rs to build/raw.rs (M2 T5): the builder the STEP reader, the
// blends and edit::affine share; paths and visibility only; then to yack-build (M3 W7), paths and visibility only;
// M3 hardening (w-h3): a use of an edge with no curve gives no samples (an error) instead of a panic;
// chunk E: `Raw::check_ids` at every entry, then every look-up by id through `entity`/`entity_mut`,
// the one indexing contract; a loop's uses walked with the next by iterators.
//! A solid's raw topology, as a file or a rebuild gives it, built into an arena: its points, its
//! edges and their uses, its faces and their loops, then each edge's range and pcurves.

use super::assemble::assemble;
use super::heal;
use crate::{
    Error, Result,
    geom::{Curve, Surface},
    math::{Point3, Vec2},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceId, PCurve, VertexId, project_on},
};

/// A face's use of an edge, in the kernel edge's direction or against it.
#[derive(Clone, Copy, Debug)]
pub struct Use {
    pub edge: usize,
    pub reversed: bool,
}

/// An edge as the file gives it, turned to run along its curve: from `start` to `end`.
#[derive(Clone, Debug)]
pub struct RawEdge {
    pub id: u64,
    pub curve: Curve,
    pub start: usize,
    pub end: usize,
    /// The faces that use it, once per use.
    pub faces: Vec<usize>,
}

#[derive(Clone, Debug)]
pub struct RawFace {
    pub id: u64,
    pub name: String,
    pub surface: Surface,
    pub reversed: bool,
    /// Each bound: whether the file calls it the outer one, and its uses in loop order.
    pub bounds: Vec<(bool, Vec<Use>)>,
    /// The points its `VERTEX_LOOP`s hold inside it, where something touches it there.
    pub points: Vec<usize>,
}

/// One solid's topology and geometry as read, before any pcurve.
#[derive(Clone, Debug, Default)]
pub struct Raw {
    pub points: Vec<Point3>,
    /// Each point's `VERTEX_POINT` instance.
    pub vertex_ids: Vec<u64>,
    pub edges: Vec<RawEdge>,
    pub faces: Vec<RawFace>,
    pub shells: Vec<Vec<usize>>,
}

/// The entity at `index` in one of a [`Raw`]'s lists or a list kept beside them. Every id a
/// `Raw` holds is checked when it reaches an entry ([`Raw::check_ids`]), and the lists beside it
/// are built one to one with its own, so an index past the end is a bug: the module's one look-up
/// that panics.
#[allow(clippy::indexing_slicing)]
pub(crate) fn entity<T>(items: &[T], index: usize) -> &T {
    &items[index]
}

/// [`entity`], mutably.
#[allow(clippy::indexing_slicing)]
pub(crate) fn entity_mut<T>(items: &mut [T], index: usize) -> &mut T {
    &mut items[index]
}

impl Raw {
    /// Edge `e`, mutably.
    pub(crate) fn edge_mut(&mut self, e: usize) -> &mut RawEdge {
        entity_mut(&mut self.edges, e)
    }

    /// Every id the topology holds names one of its entities: each edge's ends and faces, each
    /// face's uses and points, each shell's faces, and a `VERTEX_POINT` instance per point. An
    /// error naming the first that does not.
    pub fn check_ids(&self) -> Result<()> {
        let bad = |what: &str, id: usize| {
            let message = format!("a solid's topology names {what} {id}, which it does not have");
            Err(Error::InvalidInput(message))
        };
        if self.vertex_ids.len() != self.points.len() {
            return Err(Error::InvalidInput(
                "a solid's topology has a point without its vertex, or a vertex without its point"
                    .into(),
            ));
        }
        let (points, edges, faces) = (self.points.len(), self.edges.len(), self.faces.len());
        for e in &self.edges {
            if let Some(v) = [e.start, e.end].into_iter().find(|&v| v >= points) {
                return bad("point", v);
            }
            if let Some(&f) = e.faces.iter().find(|&&f| f >= faces) {
                return bad("face", f);
            }
        }
        for f in &self.faces {
            let mut uses = f.bounds.iter().flat_map(|(_, uses)| uses);
            if let Some(u) = uses.find(|u| u.edge >= edges) {
                return bad("edge", u.edge);
            }
            if let Some(&v) = f.points.iter().find(|&&v| v >= points) {
                return bad("point", v);
            }
        }
        if let Some(&f) = self.shells.iter().flatten().find(|&&f| f >= faces) {
            return bad("face", f);
        }
        Ok(())
    }
}

/// Each item with the one after it round the loop, the last with the first.
fn round<T>(items: &[T]) -> impl Iterator<Item = (&T, &T)> {
    items.iter().zip(items.iter().cycle().skip(1))
}

/// The parameter of the point of `c` nearest `p`; a periodic curve's within its first period.
pub fn param_of(c: &Curve, p: Point3) -> f64 {
    let (d0, d1) = c.domain();
    match c.period() {
        // A search over one period can stall at its ends, so a periodic curve is searched over
        // two periods half a period apart: the nearest point is inside at least one of them.
        Some(per) => {
            let t = [d0, d0 + 0.5 * per]
                .map(|a| c.project(p, a, a + per))
                .into_iter()
                .min_by(|&a, &b| c.eval(a).dist(p).total_cmp(&c.eval(b).dist(p)))
                .unwrap_or(d0);
            d0 + (t - d0).rem_euclid(per)
        }
        None if d0.is_finite() => c.project(p, d0, d1),
        None => c.project(p, -1e12, 1e12),
    }
}

/// The parameter range of an edge of `c` from `ps` to `pe`; `closed` when both are one vertex.
pub fn edge_range(c: &Curve, ps: Point3, pe: Point3, closed: bool) -> Option<(f64, f64)> {
    let (d0, d1) = c.domain();
    let t0 = param_of(c, ps);
    let mut t1 = param_of(c, pe);
    match c.period() {
        Some(p) if closed => t1 = t0 + p,
        Some(p) => {
            t1 = t0 + (t1 - t0).rem_euclid(p);
            // A vertex at the join of the period projects to either end of it.
            if (t1 - t0).abs() <= tol::param(c.d1(t0).1.norm()) {
                t1 = t0 + p;
            }
        }
        None if closed => return None,
        None => {}
    }
    let snap = |t: f64| {
        if c.period().is_some() || !d0.is_finite() {
            return t;
        }
        let near = tol::param(c.d1(t).1.norm());
        if (t - d0).abs() <= near {
            d0
        } else if (t - d1).abs() <= near {
            d1
        } else {
            t
        }
    };
    let (t0, t1) = (snap(t0), snap(t1));
    (t1 > t0).then_some((t0, t1))
}

/// One coedge's samples in the parameter plane, in its own direction: never empty.
#[derive(Clone, Debug)]
struct Track {
    ts: Vec<f64>,
    uvs: Vec<Vec2>,
}

impl Track {
    /// Where it starts.
    fn first_uv(&self) -> Vec2 {
        self.uvs.first().copied().unwrap_or_default()
    }

    /// Where it ends.
    fn last_uv(&self) -> Vec2 {
        self.uvs.last().copied().unwrap_or_default()
    }
}

/// Builds the arena of a read (and, where needed, healed) solid.
struct Builder {
    b: Brep,
    vertices: Vec<VertexId>,
    edges: Vec<EdgeId>,
}

impl Builder {
    fn new(raw: &Raw, ranges: &[(f64, f64)]) -> Builder {
        let mut b = Brep::new();
        let vertices: Vec<VertexId> = raw.points.iter().map(|&p| b.add_vertex(p)).collect();
        let edges = raw
            .edges
            .iter()
            .zip(ranges)
            .map(|(e, &r)| {
                let (start, end) = (*entity(&vertices, e.start), *entity(&vertices, e.end));
                b.add_edge(e.curve.clone(), r, start, end)
            })
            .collect();
        Builder { b, vertices, edges }
    }

    fn face(&mut self, f: &RawFace) -> Result<()> {
        let surface = &f.surface;
        let want = if f.reversed { -1.0 } else { 1.0 };
        let single = f.bounds.len() == 1;
        let mut tracks: Vec<(bool, &[Use], Vec<Track>)> = Vec::new();
        for (outer, uses) in &f.bounds {
            let sign = (*outer || single).then_some(want);
            let t = self
                .track_loop(surface, uses, sign)
                .or_else(|| self.track_loop(surface, uses, None));
            let Some(t) = t else {
                return Err(Error::InvalidInput(format!(
                    "STEP #{}: a loop of this {} face does not close in its parameters",
                    f.id,
                    surface.kind()
                )));
            };
            tracks.push((*outer, uses, t));
        }
        // The outer loop: the one enclosing area the face's way, the file's outer first, then the
        // largest.
        let outer = tracks
            .iter()
            .enumerate()
            .map(|(i, (file_outer, _, t))| (i, *file_outer, area2(t)))
            .filter(|&(_, _, area)| area * want > 0.0)
            .max_by(|&(_, a_outer, a_area), &(_, b_outer, b_area)| {
                (a_outer, a_area.abs())
                    .partial_cmp(&(b_outer, b_area.abs()))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map_or(0, |(i, _, _)| i);
        tracks.swap(0, outer);
        let (lo, hi) = tracks.first().map_or_else(|| uv_box(&[]), |(_, _, t)| uv_box(t));
        for (_, _, t) in tracks.iter_mut().skip(1) {
            shift_into(surface, t, lo, hi);
        }
        let face = self.b.add_face(surface.clone(), f.reversed, []);
        for (_, uses, ts) in &tracks {
            let specs = self.specs(surface, uses, ts);
            self.b.add_loop(face, specs);
        }
        for &p in &f.points {
            let v = *entity(&self.vertices, p);
            let uv = surface.project(self.b.vertex(v).point);
            let at = [Track { ts: vec![0.0], uvs: vec![uv] }];
            let origin = period_shift(surface, &at, lo, hi).map_or(uv, |d| uv + d);
            let edge = self.b.add_degenerate_edge(v, (0.0, 1.0));
            let spec = CoedgeSpec {
                edge,
                reversed: false,
                pcurve: PCurve::Affine { origin, dir: Vec2::ZERO },
            };
            self.b.add_loop(face, vec![spec]);
        }
        Ok(())
    }

    fn specs(&mut self, s: &Surface, uses: &[Use], tracks: &[Track]) -> Vec<CoedgeSpec> {
        let mut out = Vec::new();
        // Each use with its track, and the track of the use after it round the loop.
        for (u, (tr, next_track)) in uses.iter().zip(round(tracks)) {
            let (edge, reversed) = (*entity(&self.edges, u.edge), u.reversed);
            let pcurve = match (s, &self.b.edge(edge).curve, tr.ts.as_slice()) {
                (Surface::Plane(_), Some(Curve::Line(_)), &[t0, .., t1]) => {
                    let (a, z) = (tr.first_uv(), tr.last_uv());
                    let dir = (z - a) / (t1 - t0);
                    PCurve::Affine { origin: a - dir * t0, dir }
                }
                _ => PCurve::Projected { seed: tr.first_uv() },
            };
            out.push(CoedgeSpec { edge, reversed, pcurve });
            let end = tr.last_uv();
            let next = next_track.first_uv();
            if gap_is_pole(s, end, next) {
                let e = self.b.edge(edge);
                let v = if reversed { e.start } else { e.end };
                let d = self.b.add_degenerate_edge(v, (0.0, 1.0));
                out.push(CoedgeSpec {
                    edge: d,
                    reversed: false,
                    pcurve: PCurve::Affine {
                        origin: end,
                        dir: next - end,
                    },
                });
            }
        }
        out
    }

    /// The 3-D samples of a use of an edge, in its direction.
    /// `None` for a pole, which a STEP edge never is.
    fn samples(&self, u: Use) -> Option<(Vec<f64>, Vec<Point3>)> {
        let e = self.b.edge(*entity(&self.edges, u.edge));
        let c = e.curve.as_ref()?;
        let (t0, t1) = e.range;
        let mut cuts = vec![t0];
        cuts.extend(c.breakpoints(t0, t1));
        cuts.push(t1);
        let per = match c {
            Curve::Line(_) => 2,
            _ => (16 / (cuts.len() - 1)).max(4),
        };
        let mut ts = Vec::new();
        for (&from, &to) in cuts.iter().zip(cuts.iter().skip(1)) {
            for k in 0..per {
                ts.push(from + (to - from) * k as f64 / per as f64);
            }
        }
        ts.push(t1);
        if u.reversed {
            ts.reverse();
        }
        let pts = ts.iter().map(|&t| c.eval(t)).collect();
        Some((ts, pts))
    }

    /// Each use's parameter-plane samples, continuous along the loop and closing back on its
    /// start; `None` if no choice of periods closes it. Where a loop leaves a pole the period it
    /// leaves in is free, and `sign`, when given, picks the choice that encloses area that way.
    fn track_loop(&self, s: &Surface, uses: &[Use], sign: Option<f64>) -> Option<Vec<Track>> {
        let samples: Vec<(Vec<f64>, Vec<Point3>)> =
            uses.iter().map(|&u| self.samples(u)).collect::<Option<_>>()?;
        let start = s.project(*samples.first()?.1.first()?);
        let mut acc = Vec::with_capacity(uses.len());
        self.track_from(s, uses, &samples, 0, start, sign, &mut acc)
            .then_some(acc)
    }

    #[allow(clippy::too_many_arguments)]
    fn track_from(
        &self,
        s: &Surface,
        uses: &[Use],
        samples: &[(Vec<f64>, Vec<Point3>)],
        k: usize,
        hint: Vec2,
        sign: Option<f64>,
        acc: &mut Vec<Track>,
    ) -> bool {
        if k == uses.len() {
            let (Some(last), Some(first)) = (acc.last(), acc.first()) else { return false };
            let (end, first) = (last.last_uv(), first.first_uv());
            let closed = closes(s, end, first) || gap_is_pole(s, end, first);
            return closed && sign.is_none_or(|w| area2(acc) * w > 0.0);
        }
        let (Some((ts, pts)), Some(&this_use), Some(&p0)) =
            (samples.get(k), uses.get(k), samples.get(k).and_then(|(_, pts)| pts.first()))
        else {
            return false;
        };
        let first = s.project_near(p0, hint);
        let free = pole_free(s, first);
        let mut starts = vec![first];
        if let (true, Some(&p1)) = (free != (false, false), pts.get(1)) {
            // A use leaving a pole may leave at any angle: take the one its next point implies,
            // in each period near the hint.
            let inner = s.project_near(p1, hint);
            starts.clear();
            let shifts = |on: bool, p: Option<f64>| match p {
                Some(p) if on => vec![0.0, -p, p],
                _ => vec![0.0],
            };
            for du in shifts(free.0, s.u_period()) {
                for dv in shifts(free.1, s.v_period()) {
                    let mut c = first;
                    if free.0 {
                        c.x = inner.x + du;
                    }
                    if free.1 {
                        c.y = inner.y + dv;
                    }
                    starts.push(c);
                }
            }
            starts.sort_by(|a, b| a.dist(hint).total_cmp(&b.dist(hint)));
        }
        // A use running back over an earlier use of its edge is a slit, which a face may hold
        // where something touched it; a seam's second use runs a period away, so that is tried
        // first.
        for (slit, &st) in [false, true]
            .into_iter()
            .flat_map(|w| starts.iter().map(move |s| (w, s)))
        {
            let mut uvs = Vec::with_capacity(pts.len());
            uvs.push(st);
            let mut prev = st;
            for p in pts.iter().skip(1) {
                prev = s.project_near(*p, prev);
                uvs.push(prev);
            }
            let n = uvs.len();
            // At a pole the free parameter is the one the edge leaves it by, found as the
            // checker and every later reader of the pcurve finds it.
            let curve = self.b.edge(*entity(&self.edges, this_use.edge)).curve.as_ref();
            if let (Some(c), &[t_first, t_second, ..], &[.., t_before, t_last]) =
                (curve.filter(|_| n > 1), ts.as_slice(), ts.as_slice())
            {
                let at_pole = |uv: &Vec2| pole_free(s, *uv) != (false, false);
                if let Some(start) = uvs.first_mut().filter(|uv| at_pole(uv)) {
                    *start = project_on(s, c, t_first, t_second, st);
                }
                let before_end = uvs.get(n - 2).copied();
                if let (Some(before_end), Some(end)) = (before_end, uvs.last_mut())
                    && pole_free(s, *end) != (false, false)
                {
                    *end = project_on(s, c, t_last, t_before, before_end);
                }
            }
            if coincides_with_twin(uses, acc, this_use, &uvs) != slit {
                continue;
            }
            let end = uvs.last().copied().unwrap_or(st);
            acc.push(Track {
                ts: ts.clone(),
                uvs,
            });
            if self.track_from(s, uses, samples, k + 1, end, sign, acc) {
                return true;
            }
            acc.pop();
        }
        false
    }
}

/// Whether an earlier use of the same edge as `this` in this loop, with its track in `acc`, runs
/// back over the same parameters: a slit, where a seam's would run a period away.
fn coincides_with_twin(uses: &[Use], acc: &[Track], this: Use, uvs: &[Vec2]) -> bool {
    let n = uvs.len();
    let mid = uvs.len().checked_sub(1 + n / 2).and_then(|i| uvs.get(i));
    uses.iter().zip(acc).any(|(earlier, track)| {
        earlier.edge == this.edge && {
            let other = &track.uvs;
            other.len() == n
                && matches!((other.get(n / 2), mid), (Some(a), Some(&b)) if a.dist(b) < 1e-6)
        }
    })
}

/// Whether the surface is singular at `uv`: `(u free, v free)` where moving in that parameter
/// does not move the point.
fn pole_free(s: &Surface, uv: Vec2) -> (bool, bool) {
    let (_, su, sv) = s.d1(uv.x, uv.y);
    (su.norm() <= tol::LINEAR, sv.norm() <= tol::LINEAR)
}

/// Whether `a` and `b` are the same point of the parameter plane, not a period apart.
fn closes(s: &Surface, a: Vec2, b: Vec2) -> bool {
    let near = |d: f64, p: Option<f64>| p.is_none_or(|p| d.abs() < 0.25 * p);
    near(a.x - b.x, s.u_period()) && near(a.y - b.y, s.v_period())
}

/// Whether the step from `a` to `b` is a pole's span: one point in space, apart in parameters.
fn gap_is_pole(s: &Surface, a: Vec2, b: Vec2) -> bool {
    let free = pole_free(s, a);
    if free == (false, false) {
        return false;
    }
    let (du, dv) = ((a.x - b.x).abs(), (a.y - b.y).abs());
    let (moved, held) = (1e-9, 1e-6);
    (free.0 && du > moved && dv <= held) || (free.1 && dv > moved && du <= held)
}

fn area2(tracks: &[Track]) -> f64 {
    // Each track's points but its last, the next track's first.
    let pts: Vec<Vec2> = tracks
        .iter()
        .flat_map(|t| t.uvs.split_last().map_or(&[][..], |(_, rest)| rest).iter().copied())
        .collect();
    round(&pts).map(|(a, &b)| a.cross(b)).sum()
}

fn uv_box(tracks: &[Track]) -> (Vec2, Vec2) {
    let mut lo = Vec2::new(f64::INFINITY, f64::INFINITY);
    let mut hi = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in tracks.iter().flat_map(|t| t.uvs.iter()) {
        lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
        hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
    }
    (lo, hi)
}

/// Moves a hole's samples by whole periods so its centre lies inside the outer loop's box.
fn shift_into(s: &Surface, tracks: &mut [Track], lo: Vec2, hi: Vec2) {
    if let Some(d) = period_shift(s, tracks, lo, hi) {
        for t in tracks {
            for uv in &mut t.uvs {
                *uv = *uv + d;
            }
        }
    }
}

/// The whole periods that move the samples' centre inside the box `lo`–`hi`; `None` where none
/// are needed.
fn period_shift(s: &Surface, tracks: &[Track], lo: Vec2, hi: Vec2) -> Option<Vec2> {
    let (a, b) = uv_box(tracks);
    let c = (a + b) * 0.5;
    let shift = |x: f64, lo: f64, hi: f64, p: Option<f64>| -> f64 {
        match p {
            Some(p) if x < lo || x > hi => ((0.5 * (lo + hi) - x) / p).round() * p,
            _ => 0.0,
        }
    };
    let d = Vec2::new(
        shift(c.x, lo.x, hi.x, s.u_period()),
        shift(c.y, lo.y, hi.y, s.v_period()),
    );
    (d.x != 0.0 || d.y != 0.0).then_some(d)
}

/// The arena of a solid read as `raw`, as the file orients it, unchecked.
pub fn build(raw: &Raw, ranges: &[(f64, f64)], tolerated: Option<&heal::Tolerated>) -> Result<Brep> {
    raw.check_ids()?;
    let too_many = |what: &str| Err(Error::InvalidInput(format!("more {what} than the solid has")));
    if ranges.len() < raw.edges.len() {
        return Err(Error::InvalidInput("fewer edge ranges than the solid has edges".into()));
    }
    if let Some(t) = tolerated {
        if t.vertices.len() > raw.points.len() {
            return too_many("vertex tolerances");
        }
        if t.edges.len() > raw.edges.len() {
            return too_many("edge tolerances");
        }
    }
    let mut bld = Builder::new(raw, ranges);
    if let Some(t) = tolerated {
        for (&v, &x) in bld.vertices.iter().zip(&t.vertices) {
            bld.b.set_vertex_tolerance(v, x);
        }
        for (&e, &x) in bld.edges.iter().zip(&t.edges) {
            bld.b.set_edge_tolerance(e, x);
        }
    }
    for f in &raw.faces {
        bld.face(f)?;
    }
    let mut b = bld.b;
    let mut base = 0;
    let mut shells = Vec::new();
    for sh in &raw.shells {
        let ids: Vec<_> = b.face_ids().skip(base).take(sh.len()).collect();
        base += sh.len();
        shells.push(b.add_shell(ids));
    }
    b.add_solid(shells);
    // A solid that touches itself along an edge uses it four times; the file does not say which
    // faces pair across the material there, the faces round it do.
    b.pair_radial();
    Ok(b)
}

/// `b` with each vertex at `points[v]`, each edge on `curves[e]` and each face on `surfaces[f]`,
/// its pcurves and parameter ranges found again as a file's are; `turned` runs every loop the
/// other way, for geometry that a mirror has turned inside out.
pub fn rebuilt(
    b: &Brep,
    points: Vec<Point3>,
    curves: Vec<Option<Curve>>,
    surfaces: Vec<Surface>,
    turned: bool,
) -> Result<Brep> {
    let mut raw = Raw {
        vertex_ids: (0..points.len() as u64).collect(),
        points,
        ..Raw::default()
    };
    // Each arena edge's raw edge, `None` for one without a curve.
    let mut edge_at: Vec<Option<usize>> = vec![None; curves.len()];
    for (((e, id), c), slot) in b.edge_ids().enumerate().zip(curves).zip(edge_at.iter_mut()) {
        if let Some(curve) = c {
            let x = b.edge(id);
            *slot = Some(raw.edges.len());
            raw.edges.push(RawEdge {
                id: e as u64,
                curve,
                start: x.start.index(),
                end: x.end.index(),
                faces: Vec::new(),
            });
        }
    }
    let faces: Vec<FaceId> = b
        .face_ids()
        .filter(|&f| b.face(f).shell.is_some())
        .collect();
    for (&f, surface) in faces.iter().zip(surfaces) {
        let face = b.face(f);
        let mut bounds = Vec::new();
        for (k, &l) in face.loops.iter().enumerate() {
            let mut uses: Vec<Use> = b
                .lp(l)
                .coedges
                .iter()
                .map(|&c| b.coedge(c))
                .filter_map(|c| {
                    let edge = edge_at.get(c.edge.index()).copied().flatten()?;
                    Some(Use { edge, reversed: c.reversed ^ turned })
                })
                .collect();
            if turned {
                uses.reverse();
            }
            let face_index = raw.faces.len();
            for u in &uses {
                raw.edge_mut(u.edge).faces.push(face_index);
            }
            bounds.push((k == 0, uses));
        }
        raw.faces.push(RawFace {
            id: f.index() as u64,
            name: String::new(),
            surface,
            reversed: face.reversed ^ turned,
            bounds,
            points: Vec::new(),
        });
    }
    for sh in b.shell_ids() {
        let ids = &b.shell(sh).faces;
        raw.shells.push(
            ids.iter()
                .filter_map(|f| faces.iter().position(|g| g == f))
                .collect(),
        );
    }
    let (ranges, _) = heal::heal(&mut raw)?;
    let mut out = assemble(&raw, &ranges, None)?;
    for (g, &f) in out.face_ids().collect::<Vec<_>>().into_iter().zip(&faces) {
        out.set_face_names(g, b.face(f).names.clone());
    }
    Ok(out)
}

/// A solid as loose geometry and who uses what: the form a file gives one in, and the way to
/// build one whose faces are known by their surfaces and bounding edges alone.
#[derive(Clone, Debug, Default)]
pub struct Parts {
    pub points: Vec<Point3>,
    /// Each edge's curve and its ends, as indices into `points`.
    pub edges: Vec<(Curve, usize, usize)>,
    pub faces: Vec<PartFace>,
}

#[derive(Clone, Debug)]
pub struct PartFace {
    pub surface: Surface,
    pub reversed: bool,
    /// Each loop's uses of edges in order, the outer loop first: the edge, and whether the loop
    /// runs against it.
    pub loops: Vec<Vec<(usize, bool)>>,
    pub names: std::collections::BTreeSet<crate::topo::FaceName>,
    /// Which of the solid's shells it lies in, the outer one first.
    pub shell: usize,
}

/// The parts of `b`, one solid bounded by one outer shell and any voids. Degenerate edges, the
/// kernel's own, are left out.
pub fn parts_of(b: &Brep) -> Result<Parts> {
    let mut solids = b.solid_ids();
    let (Some(solid), None) = (solids.next(), solids.next()) else {
        return Err(Error::Unsupported("the parts of several solids".into()));
    };
    let solid = b.solid(solid);
    if solid.outer != 1 {
        return Err(Error::Unsupported(
            "the parts of a solid that touches itself".into(),
        ));
    }
    let mut parts = Parts {
        points: b.vertex_ids().map(|v| b.vertex(v).point).collect(),
        ..Parts::default()
    };
    // Each arena edge's part, `None` for a degenerate one.
    let mut edge_at: Vec<Option<usize>> = vec![None; b.edge_ids().count()];
    for (e, slot) in b.edge_ids().zip(edge_at.iter_mut()) {
        let x = b.edge(e);
        if let Some(curve) = &x.curve {
            *slot = Some(parts.edges.len());
            parts.edges.push((curve.clone(), x.start.index(), x.end.index()));
        }
    }
    for (shell, &sh) in solid.shells.iter().enumerate() {
        for &f in &b.shell(sh).faces {
            let face = b.face(f);
            let loops = face
                .loops
                .iter()
                .map(|&l| {
                    b.lp(l)
                        .coedges
                        .iter()
                        .map(|&c| b.coedge(c))
                        .filter_map(|c| {
                            Some((edge_at.get(c.edge.index()).copied().flatten()?, c.reversed))
                        })
                        .collect()
                })
                .collect();
            parts.faces.push(PartFace {
                surface: face.surface.clone(),
                reversed: face.reversed,
                loops,
                names: face.names.clone(),
                shell,
            });
        }
    }
    Ok(parts)
}

/// The solid `parts` describe, its pcurves and parameter ranges found as a file's are. Points and
/// edges no face uses are left out.
pub fn solid_of(parts: &Parts) -> Result<Brep> {
    parts.check_ids()?;
    let mut raw = Raw::default();
    // Each part edge's and point's raw one, once a face uses it.
    let mut edge_at: Vec<Option<usize>> = vec![None; parts.edges.len()];
    let mut point_at: Vec<Option<usize>> = vec![None; parts.points.len()];
    // The faces, shell by shell, each first as the parts list it.
    let mut order: Vec<(usize, &PartFace)> = parts.faces.iter().enumerate().collect();
    order.sort_by_key(|(_, f)| f.shell);
    for (fi, &(_, f)) in order.iter().enumerate() {
        let mut bounds = Vec::new();
        for (k, l) in f.loops.iter().enumerate() {
            let mut uses = Vec::new();
            for &(e, reversed) in l {
                let edge = match *entity(&edge_at, e) {
                    Some(edge) => edge,
                    None => {
                        let (curve, start, end) = entity(&parts.edges, e);
                        let mut point = |v: usize| match *entity(&point_at, v) {
                            Some(p) => p,
                            None => {
                                let p = raw.points.len();
                                *entity_mut(&mut point_at, v) = Some(p);
                                raw.vertex_ids.push(p as u64);
                                raw.points.push(*entity(&parts.points, v));
                                p
                            }
                        };
                        let (start, end) = (point(*start), point(*end));
                        let edge = raw.edges.len();
                        *entity_mut(&mut edge_at, e) = Some(edge);
                        raw.edges.push(RawEdge {
                            id: e as u64,
                            curve: curve.clone(),
                            start,
                            end,
                            faces: Vec::new(),
                        });
                        edge
                    }
                };
                raw.edge_mut(edge).faces.push(fi);
                uses.push(Use { edge, reversed });
            }
            bounds.push((k == 0, uses));
        }
        raw.faces.push(RawFace {
            id: fi as u64,
            name: String::new(),
            surface: f.surface.clone(),
            reversed: f.reversed,
            bounds,
            points: Vec::new(),
        });
        if raw.shells.len() <= f.shell {
            raw.shells.resize(f.shell + 1, Vec::new());
        }
        entity_mut(&mut raw.shells, f.shell).push(fi);
    }
    let (ranges, _) = heal::heal(&mut raw)?;
    let mut out = assemble(&raw, &ranges, None)?;
    for (g, &(_, f)) in out.face_ids().collect::<Vec<_>>().into_iter().zip(&order) {
        out.set_face_names(g, f.names.clone());
    }
    Ok(out)
}

impl Parts {
    /// Every edge's ends name points, and every loop's uses name edges: an error naming the
    /// first id that does not.
    pub fn check_ids(&self) -> Result<()> {
        let bad = |what: &str, id: usize| {
            let message = format!("a solid's parts name {what} {id}, which they do not have");
            Err(Error::InvalidInput(message))
        };
        let points = self.points.len();
        for &(_, start, end) in &self.edges {
            if let Some(v) = [start, end].into_iter().find(|&v| v >= points) {
                return bad("point", v);
            }
        }
        let mut uses = self.faces.iter().flat_map(|f| f.loops.iter().flatten());
        if let Some(&(e, _)) = uses.find(|&&(e, _)| e >= self.edges.len()) {
            return bad("edge", e);
        }
        Ok(())
    }
}
