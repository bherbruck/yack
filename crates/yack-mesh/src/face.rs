// yack: One face's mesh: a constrained Delaunay triangulation of its boundary samples in a scaled
// Adapted from springrep@a03e8a1 src/mesh/face.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/mesh/face.rs to yack-mesh (M3 W6); paths and visibility only;
// hardening (w-h1): one `Vertex` per mesh point in place of five parallel arrays, read through
// checked accessors, and the triangulation through `Cdt`'s.
//! One face's mesh: a constrained Delaunay triangulation of its boundary samples in a scaled
//! parameter plane, refined by inserting points where a triangle strays from the surface.
//!
//! The plane is scaled per face so that one unit along `u` or `v` is about the longest step the
//! deflection limits allow in that direction, so a cylinder's triangles run long along its
//! straight lines and short round it. A triangle is refined at its circumcentre in that plane, or
//! at its worst sample when the circumcentre is outside the face.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::FRAC_PI_2;

use super::{
    MeshOptions,
    cdt::{BOX, Cdt, Located, NONE, third},
    predicates::{circumcenter, orient},
};
use crate::{
    Error, Result,
    geom::Surface,
    math::{Point3, Vec2, Vec3},
    tol,
    topo::{Brep, FaceId},
};

/// The most vertices one face's mesh may have before meshing gives up.
const MAX_FACE_POINTS: usize = 8_000_000;

/// In a curved face, a triangle whose smallest angle in the scaled plane is under about 20° is
/// refined while its circumcircle is wider than one unit.
const MIN_ANGLE_SINE: f64 = 0.34;

/// A refinement point closer than this fraction of its triangle's longest side to one of the
/// triangle's corners, or in a triangle no taller than it, is not inserted.
const CROWDED: f64 = 1e-3;

/// One point of a face's mesh.
#[derive(Clone, Copy, Debug)]
struct Vertex {
    uv: Vec2,
    pos: Point3,
    /// The global mesh point of a boundary sample; interior points have none.
    gid: Option<u32>,
    /// At a pole, the parameter-plane direction the pole spans.
    pole: Option<Vec2>,
    /// The surface normal there, once asked for.
    normal: Option<Vec3>,
}

impl Vertex {
    /// A corner of the triangulation's box, which is no point of the face.
    const BOX: Vertex = Vertex { uv: Vec2::ZERO, pos: Point3::ZERO, gid: None, pole: None, normal: None };
}

pub(super) struct FaceMesh {
    pub cdt: Cdt,
    verts: Vec<Vertex>,
    inside: Vec<bool>,
    scale: Vec2,
    curved: bool,
}

/// A face's boundary samples, loop by loop, as (parameters, global point).
type Loops = Vec<Vec<(Vec2, u32)>>;

pub(super) fn mesh_face(
    b: &Brep,
    f: FaceId,
    opt: &MeshOptions,
    samples: &[Vec<(f64, u32)>],
    positions: &[Point3],
) -> Result<FaceMesh> {
    let face = b.face(f);
    let surf = &face.surface;
    let mut loops: Loops = Vec::new();
    let mut poles: BTreeMap<u32, Vec2> = BTreeMap::new();
    let mut runs: Vec<Vec<super::Run>> = Vec::new();
    for &lp in &face.loops {
        let mut pts = Vec::new();
        let mut run = Vec::new();
        for &co in &b.lp(lp).coedges {
            let c = b.coedge(co);
            let Some(mut s) = samples.get(c.edge.index()).cloned() else { continue };
            if c.reversed {
                s.reverse();
            }
            let ts: Vec<f64> = s.iter().map(|x| x.0).collect();
            let uvs = b.coedge_uvs(co, &ts);
            let (Some(&first), Some(&last), Some(&(_, g0))) = (uvs.first(), uvs.last(), s.first())
            else {
                continue;
            };
            run.push((pts.len(), last));
            if b.edge(c.edge).is_degenerate() {
                let d = last - first;
                if d.norm() > 0.0 {
                    poles.insert(g0, d / d.norm());
                }
            }
            pts.extend(uvs.iter().zip(&s).take(s.len() - 1).map(|(&uv, &(_, g))| (uv, g)));
        }
        runs.push(run);
        loops.push(pts);
    }
    super::one_sheet((surf.u_period(), surf.v_period()), &mut loops, &runs);
    let mut flat: Vec<(Vec2, u32)> = loops.iter().flatten().copied().collect();
    super::snap_repeats(&mut flat);
    // One snapped point per loop point, in loop order.
    for (p, snapped) in loops.iter_mut().flatten().zip(flat) {
        *p = snapped;
    }
    let (scale, curved) = metric(surf, &loops, &poles, positions, opt);
    let mut fm = FaceMesh {
        cdt: Cdt::new(Vec2::ZERO, Vec2::ZERO),
        verts: vec![Vertex::BOX; BOX as usize],
        inside: Vec::new(),
        scale,
        curved,
    };
    let all = loops.iter().flatten();
    let (mut lo, mut hi) = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for (uv, _) in all {
        let m = fm.m(*uv);
        lo = Vec2::new(lo.x.min(m.x), lo.y.min(m.y));
        hi = Vec2::new(hi.x.max(m.x), hi.y.max(m.y));
    }
    if !(lo.x.is_finite() && hi.x.is_finite() && lo.y.is_finite() && hi.y.is_finite()) {
        return Err(Error::Numerical(
            "face boundary has no finite points".into(),
        ));
    }
    fm.cdt = Cdt::new(lo, hi);
    let mut ids: Vec<Vec<u32>> = Vec::new();
    for lp in &loops {
        let mut row = Vec::with_capacity(lp.len());
        for &(uv, g) in lp {
            let h = fm.cdt.hint();
            let v = match fm.cdt.insert(fm.m(uv), h) {
                Ok(v) => {
                    let Some(&at) = positions.get(g as usize) else {
                        return Err(Error::Numerical("a face boundary sample has no mesh point".into()));
                    };
                    fm.push(uv, at, Some(g), poles.get(&g).copied());
                    v
                }
                Err(x) if x != NONE && fm.gid(x) == Some(g) => x,
                Err(_) => {
                    return Err(Error::Numerical("face boundary touches itself".into()));
                }
            };
            row.push(v);
        }
        ids.push(row);
    }
    let area: f64 = ids
        .iter()
        .map(|row| {
            super::closed_sides(row.iter().copied())
                .map(|(a, c)| fm.cdt.point(a).cross(fm.cdt.point(c)))
                .sum::<f64>()
        })
        .sum();
    if area < 0.0 {
        for row in &mut ids {
            row.reverse();
        }
    }
    let mut segs: Vec<(u32, u32)> = Vec::new();
    for row in &ids {
        for (a, c) in super::closed_sides(row.iter().copied()) {
            if a != c {
                segs.push((a, c));
            }
        }
    }
    for &(a, c) in &segs {
        fm.cdt
            .insert_segment(a, c)
            .map_err(|m| Error::Numerical(m.into()))?;
    }
    fm.cdt.legalize_all();
    fm.classify(&segs)?;
    fm.refine(b, f, opt)?;
    fm.unflatten();
    Ok(fm)
}

/// The parameter-plane scale for a face: one unit along each parameter is about the longest step
/// the deflection limits allow along it, at most the face's size. Whether the face is curved.
fn metric(
    surf: &Surface,
    loops: &Loops,
    poles: &BTreeMap<u32, Vec2>,
    positions: &[Point3],
    opt: &MeshOptions,
) -> (Vec2, bool) {
    let pts: Vec<(Vec2, u32)> = loops.iter().flatten().copied().collect();
    let (mut lo, mut hi) = (
        Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        -Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
    );
    for p in pts.iter().filter_map(|&(_, g)| positions.get(g as usize)) {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let size = (hi - lo).norm();
    let size = if size.is_finite() && size > 0.0 {
        size
    } else {
        1.0
    };
    // A step no longer than the deflection cannot stray by more, so the turn alone never
    // shortens one below it, up to a quarter turn: a wire thinner than the deflection takes a
    // few chords round, as its rims do.
    let allowed = |k: f64| {
        if k > 1e-9 / size {
            let turn = (opt.angular / k).max(opt.linear.min(FRAC_PI_2 / k));
            (8.0 * opt.linear / k).sqrt().min(turn)
        } else {
            f64::INFINITY
        }
    };
    let stride = (pts.len() / 256).max(1);
    let (mut su_sum, mut sv_sum, mut n) = (0.0, 0.0, 0.0);
    let (mut du, mut dv) = (f64::INFINITY, f64::INFINITY);
    for &(uv, g) in pts.iter().step_by(stride) {
        if poles.contains_key(&g) {
            continue;
        }
        let (_, su, sv) = surf.d1(uv.x, uv.y);
        let (lu, lv) = (su.norm(), sv.norm());
        if !(lu > 1e-12 && lv > 1e-12) {
            continue;
        }
        su_sum += lu;
        sv_sum += lv;
        n += 1.0;
        let h = 1e-4 * size;
        let (hu, hv) = (h / lu, h / lv);
        // One-sided turns, the smaller kept: a boundary point can sit a hair from a singularity
        // just outside the face (a spindle torus's apex), where the normal flips.
        let n0 = surf.normal(uv.x, uv.y);
        let turn = |du: f64, dv: f64| {
            let a = surf.normal(uv.x + du, uv.y + dv).dist(n0);
            let b = surf.normal(uv.x - du, uv.y - dv).dist(n0);
            a.min(b) / h
        };
        let ku = turn(hu, 0.0);
        let kv = turn(0.0, hv);
        du = du.min(allowed(ku) / lu);
        dv = dv.min(allowed(kv) / lv);
    }
    if n == 0.0 {
        return (Vec2::new(1.0, 1.0), true);
    }
    let curved = du.is_finite() || dv.is_finite();
    let du = du.min(size / (su_sum / n));
    let dv = dv.min(size / (sv_sum / n));
    (Vec2::new(1.0 / du, 1.0 / dv), curved)
}

/// Whether a triangle needs refining, and the parameters of its worst sample.
enum Verdict {
    Fine,
    Refine(Vec2),
    /// Fine on the surface but badly shaped: refine at the circumcentre if it can go in.
    Shape,
}

impl FaceMesh {
    fn m(&self, uv: Vec2) -> Vec2 {
        Vec2::new(uv.x * self.scale.x, uv.y * self.scale.y)
    }

    fn push(&mut self, uv: Vec2, pos: Point3, gid: Option<u32>, pole: Option<Vec2>) {
        self.verts.push(Vertex { uv, pos, gid, pole, normal: None });
    }

    /// Point `v`. Every vertex of the triangulation has one; a debug build checks it.
    fn vertex(&self, v: u32) -> Vertex {
        let vertex = self.verts.get(v as usize).copied();
        debug_assert!(vertex.is_some(), "mesh point {v} is past the {}", self.verts.len());
        vertex.unwrap_or(Vertex::BOX)
    }

    fn uv(&self, v: u32) -> Vec2 {
        self.vertex(v).uv
    }

    fn pos(&self, v: u32) -> Point3 {
        self.vertex(v).pos
    }

    /// The global mesh point of point `v`, if it is a boundary sample.
    pub(super) fn gid(&self, v: u32) -> Option<u32> {
        self.verts.get(v as usize).and_then(|x| x.gid)
    }

    /// Each point's global mesh point, if a boundary sample, and its position, in order.
    pub(super) fn points(&self) -> impl Iterator<Item = (Option<u32>, Point3)> + '_ {
        self.verts.iter().map(|x| (x.gid, x.pos))
    }

    /// Whether triangle `t` is inside the face.
    fn is_inside(&self, t: u32) -> bool {
        self.inside.get(t as usize).copied().unwrap_or(false)
    }

    fn set_inside(&mut self, t: u32) {
        if let Some(x) = self.inside.get_mut(t as usize) {
            *x = true;
        }
    }

    /// Marks the triangles inside the face: those on the left of each boundary segment and every
    /// triangle reached from them without crossing a segment.
    fn classify(&mut self, segs: &[(u32, u32)]) -> Result<()> {
        let n = self.cdt.triangle_count();
        self.inside = vec![false; n];
        let mut stack = Vec::new();
        for &(a, c) in segs {
            let Some((t, _)) = self.cdt.find_edge(a, c) else {
                return Err(Error::Numerical("face boundary segment is missing".into()));
            };
            if !self.is_inside(t) {
                self.set_inside(t);
                stack.push(t);
            }
        }
        while let Some(t) = stack.pop() {
            let tri = self.cdt.tri(t);
            for k in 0..3 {
                let nb = tri.across(k);
                if nb != NONE && !tri.is_fixed(k) && !self.is_inside(nb) {
                    self.set_inside(nb);
                    stack.push(nb);
                }
            }
        }
        let directed: BTreeSet<(u32, u32)> = segs.iter().copied().collect();
        for t in 0..n as u32 {
            if self.is_inside(t) && self.cdt.tri(t).corners.iter().any(|&v| v < BOX) {
                return Err(Error::Numerical("face boundary does not close".into()));
            }
        }
        for &(a, c) in segs {
            if directed.contains(&(c, a)) {
                continue;
            }
            if let Some((t, _)) = self.cdt.find_edge(c, a)
                && self.is_inside(t)
            {
                return Err(Error::Numerical(
                    "face boundary loops do not bound one region".into(),
                ));
            }
        }
        Ok(())
    }

    /// Parameters to sample a triangle at: a pole corner is moved along its pole to face the
    /// triangle's other corners, since every point of the pole is the same point.
    fn sample_uvs(&self, c: [u32; 3]) -> Option<[Vec2; 3]> {
        let mut sum = Vec2::ZERO;
        let mut n = 0.0;
        for &v in &c {
            let x = self.vertex(v);
            if x.pole.is_none() {
                sum = sum + x.uv;
                n += 1.0;
            }
        }
        if n == 0.0 {
            return None;
        }
        let toward = sum / n;
        Some(c.map(|v| self.facing(v, toward)))
    }

    /// Point `v`'s parameters, a pole's moved along it to face `toward`.
    fn facing(&self, v: u32, toward: Vec2) -> Vec2 {
        let Vertex { uv, pole, .. } = self.vertex(v);
        match pole {
            Some(d) => uv + d * (toward - uv).dot(d),
            None => uv,
        }
    }

    /// Whether points `a` and `c` are one boundary sample.
    fn same_point(&self, a: u32, c: u32) -> bool {
        let ga = self.vertex(a).gid;
        ga.is_some() && ga == self.vertex(c).gid
    }

    fn corner_normal(&mut self, surf: &Surface, v: u32, at: Vec2, toward: Vec2) -> Vec3 {
        if self.vertex(v).pole.is_some() {
            // A hair in from the pole, towards the triangle: on the pole itself a cone's radius is
            // zero to rounding, and its normal takes its sign from that radius.
            let at = at + (toward - at) * 1e-6;
            return surf.normal(at.x, at.y);
        }
        match self.verts.get_mut(v as usize) {
            Some(x) => *x.normal.get_or_insert_with(|| surf.normal(at.x, at.y)),
            None => surf.normal(at.x, at.y),
        }
    }

    fn assess(&mut self, surf: &Surface, t: u32, opt: &MeshOptions) -> Verdict {
        let c = self.cdt.tri(t).corners;
        if c.iter().any(|&v| v < BOX) {
            return Verdict::Fine;
        }
        let p = c.map(|v| self.pos(v));
        // However small the face, a triangle across which the surface turns more than a right angle
        // stands for none of it: a ball narrower than the deflection, sampled at its poles and
        // seam alone, would be triangles from pole to pole that fold flat.
        if let Some(s) = self.sample_uvs(c) {
            for i in 0..3 {
                let j = (i + 1) % 3;
                if self.same_point(third(c, i), third(c, j)) {
                    continue;
                }
                let mid = (s[0] + s[1] + s[2]) / 3.0;
                let ni = self.corner_normal(surf, third(c, i), third(s, i), mid);
                let nj = self.corner_normal(surf, third(c, j), third(s, j), mid);
                if ni.dot(nj) < 0.0 {
                    return Verdict::Refine((s[0] + s[1] + s[2]) / 3.0);
                }
            }
        }
        let diam = p[0].dist(p[1]).max(p[1].dist(p[2])).max(p[2].dist(p[0]));
        // A triangle no wider than the deflection cannot stray from the surface by more.
        if diam <= opt.linear {
            return Verdict::Fine;
        }
        let Some(s) = self.sample_uvs(c) else {
            return Verdict::Fine;
        };
        let fixed = self.cdt.tri(t).fixed;
        // A boundary segment is shared with the next face and never split here; what the surface
        // strays from it is the edge's error, so the centroid may stray that much more.
        let mut allow: f64 = 0.0;
        let mut worst: Option<(f64, Vec2)> = None;
        let mut bad = false;
        for (i, &on_boundary) in fixed.iter().enumerate() {
            let j = (i + 1) % 3;
            let (ci, cj) = (third(c, i), third(c, j));
            if self.same_point(ci, cj) {
                continue;
            }
            let ui = self.facing(ci, self.uv(cj));
            let uj = self.facing(cj, self.uv(ci));
            let mid = (ui + uj) * 0.5;
            let dev = surf.eval(mid.x, mid.y).dist((third(p, i) + third(p, j)) * 0.5);
            if on_boundary {
                allow = allow.max(dev);
                continue;
            }
            if dev > opt.linear {
                bad = true;
                if worst.is_none_or(|w| dev > w.0) {
                    worst = Some((dev, mid));
                }
            }
            let mid = (s[0] + s[1] + s[2]) / 3.0;
            let ni = self.corner_normal(surf, ci, third(s, i), mid);
            let nj = self.corner_normal(surf, cj, third(s, j), mid);
            // The turn between two corners is taken across the step between them the way the
            // normal turns. Where that is no longer than the deflection the surface there is
            // within it of the triangle whatever the turn, as on a wire thinner than it.
            let across = (nj - ni).unit().map_or(0.0, |t| (third(p, j) - third(p, i)).dot(t).abs());
            if across > opt.linear && ni.angle(nj) > opt.angular {
                bad = true;
            }
        }
        let cuv = (s[0] + s[1] + s[2]) / 3.0;
        let cdev = surf.eval(cuv.x, cuv.y).dist((p[0] + p[1] + p[2]) / 3.0);
        if cdev > opt.linear + allow {
            bad = true;
            if worst.is_none_or(|w| cdev - allow > w.0) {
                worst = Some((cdev - allow, cuv));
            }
        }
        if bad {
            let at = worst.map_or(cuv, |w| w.1);
            return Verdict::Refine(at);
        }
        if self.curved && self.skinny(c) {
            return Verdict::Shape;
        }
        Verdict::Fine
    }

    /// Badly shaped in the scaled plane and larger than one unit there.
    fn skinny(&self, c: [u32; 3]) -> bool {
        let m = c.map(|v| self.cdt.point(v));
        let l = [m[0].dist(m[1]), m[1].dist(m[2]), m[2].dist(m[0])];
        let area2 = orient(m[0], m[1], m[2]).abs();
        let lmax = l[0].max(l[1]).max(l[2]);
        // Circumradius = abc / (4·area); the smallest angle's sine = 2·area / (the two longer).
        let r = l[0] * l[1] * l[2] / (2.0 * area2).max(1e-300);
        let mut ls = l;
        ls.sort_by(f64::total_cmp);
        let sine = area2 / (ls[1] * ls[2]).max(1e-300);
        lmax > 1.0 && r > 1.0 && sine < MIN_ANGLE_SINE
    }

    /// Adds an interior point at `uv`, walking from `t`. Returns it, or `None` when the point is
    /// outside the face or on an existing point or boundary segment.
    fn add(&mut self, surf: &Surface, t: u32, uv: Vec2) -> Option<u32> {
        if !(uv.x.is_finite() && uv.y.is_finite()) {
            return None;
        }
        let m = self.m(uv);
        let loc = self.cdt.locate(t, m);
        if let Located::Inside(x) | Located::OnEdge(x, _) = loc {
            if !self.is_inside(x) {
                return None;
            }
            // A triangle whose error lies along a boundary segment it cannot split would otherwise
            // be refined again and again a little nearer that segment, leaving slivers.
            let tri = self.cdt.tri(x).corners;
            let c = tri.map(|v| self.cdt.point(v));
            let longest = c[0].dist(c[1]).max(c[1].dist(c[2])).max(c[2].dist(c[0]));
            let nearest = c.iter().map(|q| q.dist(m)).fold(f64::INFINITY, f64::min);
            let height = orient(c[0], c[1], c[2]).abs() / longest;
            if nearest.min(height) < CROWDED * longest {
                return None;
            }
            // Nor one within the tolerance of a side's line in space, where it would make a flat
            // triangle that no flip may mend (a pole's narrowing spline face, past the stretch the
            // scaled plane sees).
            let q = surf.eval(uv.x, uv.y);
            let flat = super::closed_sides(tri.iter().map(|&v| self.pos(v)))
                .any(|(a, b)| (b - a).cross(q - a).norm() <= tol::LINEAR * a.dist(b));
            if flat {
                return None;
            }
        }
        let n_before = self.cdt.triangle_count();
        let v = self.cdt.insert_at(m, loc).ok()?;
        self.push(uv, surf.eval(uv.x, uv.y), None, None);
        self.inside.resize(self.cdt.triangle_count(), true);
        debug_assert!(self.cdt.triangle_count() >= n_before);
        Some(v)
    }

    fn refine(&mut self, b: &Brep, f: FaceId, opt: &MeshOptions) -> Result<()> {
        let surf = &b.face(f).surface;
        let mut queued = self.inside.clone();
        let mut queue: Vec<u32> = (0..self.cdt.triangle_count() as u32)
            .filter(|&t| self.is_inside(t))
            .collect();
        let mut pops: usize = 0;
        while let Some(t) = queue.pop() {
            if let Some(q) = queued.get_mut(t as usize) {
                *q = false;
            }
            pops += 1;
            if self.verts.len() > MAX_FACE_POINTS {
                return Err(Error::Unsupported(format!(
                    "a face that needs more than {MAX_FACE_POINTS} points at a deflection of {} mm",
                    opt.linear
                )));
            }
            if pops > 40 * MAX_FACE_POINTS {
                return Err(Error::Numerical("tessellation does not converge".into()));
            }
            let at = match self.assess(surf, t, opt) {
                Verdict::Fine => continue,
                Verdict::Refine(at) => Some(at),
                Verdict::Shape => None,
            };
            let c = self.cdt.tri(t).corners.map(|v| self.cdt.point(v));
            let center = circumcenter(c[0], c[1], c[2])
                .map(|m| Vec2::new(m.x / self.scale.x, m.y / self.scale.y));
            let mut v = center.and_then(|uv| self.add(surf, t, uv));
            if v.is_none()
                && let Some(at) = at
            {
                v = self.add(surf, t, at);
                if v.is_none() {
                    let cuv = self.cdt.tri(t)
                        .corners
                        .iter()
                        .fold(Vec2::ZERO, |s, &x| s + self.uv(x))
                        / 3.0;
                    v = self.add(surf, t, cuv);
                }
            }
            let Some(v) = v else { continue };
            queued.resize(self.cdt.triangle_count(), false);
            for x in self
                .cdt
                .around(v)
                .into_iter()
                .map(|(x, _)| x)
                .chain(std::iter::once(t))
            {
                if let Some(q) = queued.get_mut(x as usize)
                    && !*q
                {
                    *q = true;
                    queue.push(x);
                }
            }
        }
        Ok(())
    }

    /// A triangle's height over its longest side in space, and that side.
    fn height(&self, c: [u32; 3]) -> (f64, usize) {
        let p = c.map(|v| self.pos(v));
        let side = |k: usize| third(p, k).dist(third(p, (k + 1) % 3));
        let k = (0..3)
            .max_by(|&a, &b| side(a).total_cmp(&side(b)))
            .unwrap_or(0);
        let area2 = (p[1] - p[0]).cross(p[2] - p[0]).norm();
        (area2 / side(k).max(1e-300), k)
    }

    /// Flips the longest side of each triangle that is flat in space, within the linear tolerance,
    /// where that leaves both triangles taller. On a face far narrower one way than the other (a
    /// cylinder of radius 1e-3 mm, 4 mm long) the scaled plane stretches the narrow way, so a
    /// triangle well shaped there can have a corner within 1e-7 mm of its opposite side in space.
    fn unflatten(&mut self) {
        for _ in 0..4 {
            let mut flipped = false;
            for t in 0..self.cdt.triangle_count() as u32 {
                let tri = self.cdt.tri(t);
                let c = tri.corners;
                if !self.is_inside(t) || c.iter().any(|&v| v < BOX) {
                    continue;
                }
                let (h, k) = self.height(c);
                if h > tol::LINEAR {
                    continue;
                }
                let u = tri.across(k);
                if u == NONE || tri.is_fixed(k) || !self.is_inside(u) {
                    continue;
                }
                let d = self.cdt.tri(u).corners;
                if d.iter().any(|&v| v < BOX) {
                    continue;
                }
                let before = h.min(self.height(d).0);
                let (a, b, x) = (third(c, k), third(c, (k + 1) % 3), third(c, (k + 2) % 3));
                let Some(&y) = d.iter().find(|&&v| v != a && v != b) else {
                    continue;
                };
                let after = self.height([x, a, y]).0.min(self.height([y, b, x]).0);
                if after > before && self.cdt.flip(t, k).is_some() {
                    flipped = true;
                }
            }
            if !flipped {
                break;
            }
        }
    }

    /// Splits the interior edge `k` of triangle `t` at its middle.
    pub(super) fn split_chord(&mut self, surf: &Surface, t: u32, k: usize) {
        let tri = self.cdt.tri(t);
        let (a, c) = (tri.corner(k), tri.corner((k + 1) % 3));
        let ua = self.facing(a, self.uv(c));
        let uc = self.facing(c, self.uv(a));
        let uv = (ua + uc) * 0.5;
        let m = (self.cdt.point(a) + self.cdt.point(c)) * 0.5;
        let v = self.cdt.point_count() as u32;
        self.cdt.push_point(m, t);
        self.cdt.split_edge(t, k, v);
        self.push(uv, surf.eval(uv.x, uv.y), None, None);
        self.inside.resize(self.cdt.triangle_count(), true);
    }

    /// The face's triangles, as corner indices.
    pub(super) fn triangles(&self) -> impl Iterator<Item = (u32, [u32; 3])> + '_ {
        (0..self.cdt.triangle_count() as u32)
            .filter(|&t| self.is_inside(t))
            .map(|t| (t, self.cdt.tri(t).corners))
    }
}
