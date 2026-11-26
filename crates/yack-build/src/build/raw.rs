// yack: a solid's raw topology (points, edges, faces and their uses) built into an arena.
// Adapted from springrep@a03e8a1 src/step/topology.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from step/topology.rs to build/raw.rs (M2 T5): the builder the STEP reader, the
// blends and edit::affine share; paths and visibility only; then to yack-build (M3 W7), paths and visibility only;
// M3 hardening (w-h3): a use of an edge with no curve gives no samples (an error) instead of a panic.
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

/// One coedge's samples in the parameter plane, in its own direction.
#[derive(Clone, Debug)]
struct Track {
    ts: Vec<f64>,
    uvs: Vec<Vec2>,
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
            .map(|(e, &r)| b.add_edge(e.curve.clone(), r, vertices[e.start], vertices[e.end]))
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
        let areas: Vec<f64> = tracks.iter().map(|(_, _, t)| area2(t)).collect();
        let outer = (0..tracks.len())
            .filter(|&i| areas[i] * want > 0.0)
            .max_by(|&i, &j| {
                let key = |k: usize| (tracks[k].0, areas[k].abs());
                key(i)
                    .partial_cmp(&key(j))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(0);
        tracks.swap(0, outer);
        let (lo, hi) = uv_box(&tracks[0].2);
        for (_, _, t) in tracks.iter_mut().skip(1) {
            shift_into(surface, t, lo, hi);
        }
        let face = self.b.add_face(surface.clone(), f.reversed, []);
        for (_, uses, ts) in &tracks {
            let specs = self.specs(surface, uses, ts);
            self.b.add_loop(face, specs);
        }
        for &p in &f.points {
            let v = self.vertices[p];
            let mut at = [Track {
                ts: vec![0.0],
                uvs: vec![surface.project(self.b.vertex(v).point)],
            }];
            shift_into(surface, &mut at, lo, hi);
            let edge = self.b.add_degenerate_edge(v, (0.0, 1.0));
            let spec = CoedgeSpec {
                edge,
                reversed: false,
                pcurve: PCurve::Affine {
                    origin: at[0].uvs[0],
                    dir: Vec2::ZERO,
                },
            };
            self.b.add_loop(face, vec![spec]);
        }
        Ok(())
    }

    fn specs(&mut self, s: &Surface, uses: &[Use], tracks: &[Track]) -> Vec<CoedgeSpec> {
        let mut out = Vec::new();
        let n = uses.len();
        for k in 0..n {
            let (edge, reversed) = (self.edges[uses[k].edge], uses[k].reversed);
            let tr = &tracks[k];
            let pcurve = match (s, &self.b.edge(edge).curve) {
                (Surface::Plane(_), Some(Curve::Line(_))) => {
                    let (t0, t1) = (tr.ts[0], tr.ts[tr.ts.len() - 1]);
                    let (a, z) = (tr.uvs[0], tr.uvs[tr.uvs.len() - 1]);
                    let dir = (z - a) / (t1 - t0);
                    PCurve::Affine {
                        origin: a - dir * t0,
                        dir,
                    }
                }
                _ => PCurve::Projected { seed: tr.uvs[0] },
            };
            out.push(CoedgeSpec {
                edge,
                reversed,
                pcurve,
            });
            let end = tr.uvs[tr.uvs.len() - 1];
            let next = tracks[(k + 1) % n].uvs[0];
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
        let e = self.b.edge(self.edges[u.edge]);
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
        for w in cuts.windows(2) {
            for k in 0..per {
                ts.push(w[0] + (w[1] - w[0]) * k as f64 / per as f64);
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
        let start = s.project(samples[0].1[0]);
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
            let (end, first) = (acc[k - 1].uvs[acc[k - 1].uvs.len() - 1], acc[0].uvs[0]);
            let closed = closes(s, end, first) || gap_is_pole(s, end, first);
            return closed && sign.is_none_or(|w| area2(acc) * w > 0.0);
        }
        let (ts, pts) = &samples[k];
        let first = s.project_near(pts[0], hint);
        let free = pole_free(s, first);
        let mut starts = vec![first];
        if free != (false, false) && pts.len() > 1 {
            // A use leaving a pole may leave at any angle: take the one its next point implies,
            // in each period near the hint.
            let inner = s.project_near(pts[1], hint);
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
            for p in &pts[1..] {
                let prev = uvs[uvs.len() - 1];
                uvs.push(s.project_near(*p, prev));
            }
            let n = uvs.len();
            // At a pole the free parameter is the one the edge leaves it by, found as the
            // checker and every later reader of the pcurve finds it.
            let curve = self.b.edge(self.edges[uses[k].edge]).curve.as_ref();
            if let Some(c) = curve.filter(|_| n > 1) {
                if pole_free(s, uvs[0]) != (false, false) {
                    uvs[0] = project_on(s, c, ts[0], ts[1], st);
                }
                if pole_free(s, uvs[n - 1]) != (false, false) {
                    uvs[n - 1] = project_on(s, c, ts[n - 1], ts[n - 2], uvs[n - 2]);
                }
            }
            if coincides_with_twin(uses, acc, k, &uvs) != slit {
                continue;
            }
            let end = uvs[n - 1];
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

/// Whether an earlier use of the same edge in this loop runs back over the same parameters: a slit,
/// where a seam's would run a period away.
fn coincides_with_twin(uses: &[Use], acc: &[Track], k: usize, uvs: &[Vec2]) -> bool {
    let n = uvs.len();
    (0..k).any(|j| {
        uses[j].edge == uses[k].edge && {
            let other = &acc[j].uvs;
            other.len() == n && other[n / 2].dist(uvs[n - 1 - n / 2]) < 1e-6
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
    let pts: Vec<Vec2> = tracks
        .iter()
        .flat_map(|t| t.uvs[..t.uvs.len() - 1].iter().copied())
        .collect();
    (0..pts.len())
        .map(|i| pts[i].cross(pts[(i + 1) % pts.len()]))
        .sum()
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
    if d.x != 0.0 || d.y != 0.0 {
        for t in tracks {
            for uv in &mut t.uvs {
                *uv = *uv + d;
            }
        }
    }
}

/// The arena of a solid read as `raw`, as the file orients it, unchecked.
pub fn build(raw: &Raw, ranges: &[(f64, f64)], tolerated: Option<&heal::Tolerated>) -> Result<Brep> {
    let mut bld = Builder::new(raw, ranges);
    if let Some(t) = tolerated {
        for (v, &x) in t.vertices.iter().enumerate() {
            bld.b.set_vertex_tolerance(bld.vertices[v], x);
        }
        for (e, &x) in t.edges.iter().enumerate() {
            bld.b.set_edge_tolerance(bld.edges[e], x);
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
    let mut edge_at = vec![usize::MAX; curves.len()];
    for ((e, id), c) in b.edge_ids().enumerate().zip(curves) {
        if let Some(curve) = c {
            let x = b.edge(id);
            edge_at[e] = raw.edges.len();
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
                .filter(|c| edge_at[c.edge.index()] != usize::MAX)
                .map(|c| Use {
                    edge: edge_at[c.edge.index()],
                    reversed: c.reversed ^ turned,
                })
                .collect();
            if turned {
                uses.reverse();
            }
            for u in &uses {
                raw.edges[u.edge].faces.push(raw.faces.len());
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
    let mut edge_at = vec![usize::MAX; b.edge_ids().count()];
    for e in b.edge_ids() {
        let x = b.edge(e);
        if let Some(curve) = &x.curve {
            edge_at[e.index()] = parts.edges.len();
            parts
                .edges
                .push((curve.clone(), x.start.index(), x.end.index()));
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
                        .filter(|c| edge_at[c.edge.index()] != usize::MAX)
                        .map(|c| (edge_at[c.edge.index()], c.reversed))
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
    let mut raw = Raw::default();
    let mut edge_at = vec![usize::MAX; parts.edges.len()];
    let mut point_at = vec![usize::MAX; parts.points.len()];
    let mut order: Vec<usize> = (0..parts.faces.len()).collect();
    order.sort_by_key(|&f| parts.faces[f].shell);
    for (fi, &at) in order.iter().enumerate() {
        let f = &parts.faces[at];
        let mut bounds = Vec::new();
        for (k, l) in f.loops.iter().enumerate() {
            let mut uses = Vec::new();
            for &(e, reversed) in l {
                if edge_at[e] == usize::MAX {
                    let (curve, start, end) = &parts.edges[e];
                    let mut ends = [*start, *end];
                    for v in &mut ends {
                        if point_at[*v] == usize::MAX {
                            point_at[*v] = raw.points.len();
                            raw.vertex_ids.push(raw.points.len() as u64);
                            raw.points.push(parts.points[*v]);
                        }
                        *v = point_at[*v];
                    }
                    edge_at[e] = raw.edges.len();
                    raw.edges.push(RawEdge {
                        id: e as u64,
                        curve: curve.clone(),
                        start: ends[0],
                        end: ends[1],
                        faces: Vec::new(),
                    });
                }
                raw.edges[edge_at[e]].faces.push(fi);
                uses.push(Use {
                    edge: edge_at[e],
                    reversed,
                });
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
        raw.shells[f.shell].push(fi);
    }
    let (ranges, _) = heal::heal(&mut raw)?;
    let mut out = assemble(&raw, &ranges, None)?;
    for (g, &at) in out.face_ids().collect::<Vec<_>>().into_iter().zip(&order) {
        out.set_face_names(g, parts.faces[at].names.clone());
    }
    Ok(out)
}
