// yack: faces of one solid that share no edge but meet: where, and how deep.
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/check.rs to yack-check (M3 W6); split out of lib.rs into its
// own module (M3 hardening); its maps and sets are BTreeMap/BTreeSet (no order reaches an output).
//! Faces of one solid that share no edge but meet: where, and how deep.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    geom::{Curve, Surface, distance_to},
    intersect::csi::{curve_surface, signed_distance},
    math::{Aabb, Point3, Vec3},
    measure, tol,
    topo::{Brep, EdgeId, FaceId, VertexId, domain::Domain, face_domain},
};

/// How far face `a`'s boundary reaches through face `b`'s surface: the lesser of how far it lies on
/// each side, from its vertices and 8 points along each edge. A boundary on one side only touches.
pub(crate) fn depth(b: &Brep, fa: FaceId, fb: FaceId) -> f64 {
    let s = &b.face(fb).surface;
    let (mut above, mut below) = (0.0f64, 0.0f64);
    for &lp in &b.face(fa).loops {
        for &c in &b.lp(lp).coedges {
            let e = b.edge(b.coedge(c).edge);
            let pts: Vec<Point3> = match &e.curve {
                Some(cv) => (0..=8)
                    .map(|k| cv.eval(e.range.0 + (e.range.1 - e.range.0) * k as f64 / 8.0))
                    .collect(),
                None => vec![b.vertex(e.start).point],
            };
            for p in pts {
                let d = signed_distance(s, p);
                above = above.max(d);
                below = below.max(-d);
            }
        }
    }
    above.min(below)
}

/// A point where two faces that share no edge meet, away from any vertex they share: the solid
/// passes through itself there, or touches itself.
pub(crate) struct Crossing {
    pub(crate) a: FaceId,
    pub(crate) b: FaceId,
    pub(crate) at: Point3,
}

pub(crate) fn padded_box(bb: Aabb) -> Aabb {
    let pad = Vec3::new(tol::LINEAR, tol::LINEAR, tol::LINEAR);
    Aabb { min: bb.min - pad, max: bb.max + pad }
}

fn boxes_meet(a: &Aabb, b: &Aabb) -> bool {
    (0..3).all(|k| a.min[k] <= b.max[k] && b.min[k] <= a.max[k])
}

pub(crate) fn in_box(p: Point3, bb: &Aabb) -> bool {
    (0..3).all(|k| p[k] >= bb.min[k] && p[k] <= bb.max[k])
}

/// What the pair tests of one check share: each face's domain, built when a point is first
/// located in it, each edge's box, and each edge's meeting with each face.
#[derive(Default)]
struct PairCache {
    // D26: BTree* in hardening, proven by digests
    domains: BTreeMap<FaceId, Domain>,
    edge_boxes: BTreeMap<EdgeId, Aabb>,
    meetings: BTreeMap<(EdgeId, FaceId), Option<Point3>>,
}

impl PairCache {
    fn edge_box(&mut self, b: &Brep, e: EdgeId) -> Option<Aabb> {
        let edge = b.edge(e);
        let curve = edge.curve.as_ref()?;
        Some(
            *self
                .edge_boxes
                .entry(e)
                .or_insert_with(|| padded_box(curve.bbox(edge.range.0, edge.range.1))),
        )
    }

    /// A box that holds face `f`: its loose box where that is already worked out, otherwise one
    /// cheaper to find, its edges' boxes and, where the surface can bulge past them, the box of a
    /// B-spline's control points or of a whole sphere or torus.
    fn face_box(&mut self, b: &Brep, f: FaceId) -> Aabb {
        if let Some(bb) = b.memo(f).loose_box.get() {
            return padded_box(*bb);
        }
        let s = &b.face(f).surface;
        let mut bb = match s {
            Surface::Plane(_) | Surface::Cylinder(_) | Surface::Cone(_) | Surface::Extrusion(_) => {
                Aabb::EMPTY
            }
            Surface::Nurbs(n) => n.hull_box(),
            Surface::Sphere(sp) => {
                let r = Vec3::new(sp.radius, sp.radius, sp.radius);
                Aabb { min: sp.frame.origin - r, max: sp.frame.origin + r }
            }
            Surface::Torus(t) => {
                let r = t.major + t.minor;
                let r = Vec3::new(r, r, r);
                Aabb { min: t.frame.origin - r, max: t.frame.origin + r }
            }
            _ => return padded_box(measure::face_box(b, f)),
        };
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                let e = b.coedge(c).edge;
                match self.edge_box(b, e) {
                    Some(eb) => bb.union(&eb),
                    None => bb.add(b.vertex(b.edge(e).start).point),
                }
            }
        }
        padded_box(bb)
    }
}

/// Faces that share no edge but meet: an edge of one crosses or touches the other away from the
/// vertices the two share. Face pairs are found by their boxes, sorted along `x`. A pair whose
/// memos were found apart before is not tested again, and neither is a pair `sides` gives one
/// input of a boolean. With `each`, only faces of one solid are paired, as an import's parts may
/// rest on one another. Two curved faces that meet only along a closed curve inside both are not
/// found: no edge reaches it.
pub(crate) fn crossings(b: &Brep, sides: Option<&[Option<usize>]>, each: bool) -> Vec<Crossing> {
    // D26: BTree* in hardening, proven by digests
    let faces: Vec<FaceId> = b.face_ids().collect();
    let solid_of = |f: FaceId| b.face(f).shell.and_then(|s| b.shell(s).solid);
    let mut cache = PairCache::default();
    let boxes: Vec<Aabb> = faces.iter().map(|&f| cache.face_box(b, f)).collect();
    let mut order: Vec<usize> = (0..faces.len()).collect();
    order.sort_by(|&i, &j| boxes[i].min.x.total_cmp(&boxes[j].min.x));

    let mut by_edge: BTreeMap<EdgeId, Vec<FaceId>> = BTreeMap::new();
    let mut verts: BTreeMap<FaceId, Vec<VertexId>> = BTreeMap::new();
    for &f in &faces {
        let vs = verts.entry(f).or_default();
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                let e = b.coedge(c).edge;
                by_edge.entry(e).or_default().push(f);
                vs.push(b.edge(e).start);
                vs.push(b.edge(e).end);
            }
        }
    }
    let adjacent: BTreeSet<(FaceId, FaceId)> = by_edge
        .values()
        .flat_map(|fs| fs.iter().flat_map(|&x| fs.iter().map(move |&y| (x, y))))
        .collect();
    let mut out = Vec::new();
    for (k, &i) in order.iter().enumerate() {
        for &j in &order[k + 1..] {
            if boxes[j].min.x > boxes[i].max.x {
                break;
            }
            let (fa, fb) = (faces[i], faces[j]);
            if !boxes_meet(&boxes[i], &boxes[j])
                || adjacent.contains(&(fa, fb))
                || (each && solid_of(fa) != solid_of(fb))
            {
                continue;
            }
            if sides.is_some_and(|s| s[fa.index()].is_some() && s[fa.index()] == s[fb.index()]) {
                b.mark_apart(fa, fb);
                continue;
            }
            if b.known_apart(fa, fb) {
                continue;
            }
            // Where either face's box is clear of the other's surface, they cannot meet.
            let (sa, sb) = (&b.face(fa).surface, &b.face(fb).surface);
            if (analytic(sb) && clear_of(&boxes[i], sb))
                || (analytic(sa) && clear_of(&boxes[j], sa))
            {
                b.mark_apart(fa, fb);
                continue;
            }
            let mut hit = None;
            'pair: for (x, y, ybox) in [(fa, fb, &boxes[j]), (fb, fa, &boxes[i])] {
                for &lp in &b.face(x).loops {
                    for &c in &b.lp(lp).coedges {
                        let e = b.coedge(c).edge;
                        let Some(eb) = cache.edge_box(b, e) else {
                            continue;
                        };
                        if !boxes_meet(&eb, ybox) {
                            continue;
                        }
                        hit = match cache.meetings.get(&(e, y)) {
                            Some(&m) => m,
                            None => {
                                let m = edge_meets_face(b, e, y, ybox, &verts[&y], &mut cache);
                                cache.meetings.insert((e, y), m);
                                m
                            }
                        };
                        if hit.is_some() {
                            break 'pair;
                        }
                    }
                }
            }
            match hit {
                Some(at) => out.push(Crossing { a: fa, b: fb, at }),
                None => b.mark_apart(fa, fb),
            }
        }
    }
    out
}

/// The distance from `p` to `s`, or a lower bound of it: the analytic surfaces' signed distances
/// change no faster than the point moves, so they bound the distance to the surface from below.
fn gap(s: &Surface, p: Point3) -> f64 {
    match s {
        Surface::Plane(_)
        | Surface::Cylinder(_)
        | Surface::Cone(_)
        | Surface::Sphere(_)
        | Surface::Torus(_) => signed_distance(s, p).abs(),
        _ => distance_to(s, p),
    }
}

fn analytic(s: &Surface) -> bool {
    matches!(
        s,
        Surface::Plane(_)
            | Surface::Cylinder(_)
            | Surface::Cone(_)
            | Surface::Sphere(_)
            | Surface::Torus(_)
    )
}

/// Whether no point of `bb` lies within the tolerance of `s`: the box lies wholly on one side of
/// a plane, or the surface is farther from its centre than its corners are.
fn clear_of(bb: &Aabb, s: &Surface) -> bool {
    let reach = match s {
        Surface::Plane(p) => {
            let h = (bb.max - bb.min) * 0.5;
            let n = p.frame.z;
            n.x.abs() * h.x + n.y.abs() * h.y + n.z.abs() * h.z
        }
        _ => 0.5 * bb.diagonal(),
    };
    gap(s, (bb.min + bb.max) * 0.5) > reach + tol::LINEAR
}

/// The stretches of `range` along which `curve`, whose box over all of it is `whole`, may come
/// within the tolerance of `s`, inside the box `bound`. A stretch is dropped where its box misses
/// `bound` or is [`clear_of`] the surface. Kept stretches are halved, so a curve that only passes
/// near the surface costs a few distances, not the sampling a crossing is found by.
fn near_ranges(
    curve: &Curve,
    range: (f64, f64),
    whole: Aabb,
    s: &Surface,
    bound: &Aabb,
) -> Vec<(f64, f64)> {
    fn halve(
        c: &Curve,
        (t0, t1): (f64, f64),
        bb: Aabb,
        depth: u32,
        s: &Surface,
        bound: &Aabb,
        out: &mut Vec<(f64, f64)>,
    ) {
        if !boxes_meet(&bb, bound) || (analytic(s) && clear_of(&bb, s)) {
            return;
        }
        let m = 0.5 * (t0 + t1);
        let p = c.eval(m);
        let g = gap(s, p);
        // Off the analytic surfaces a distance costs a projection: the one from the curve's middle
        // also bounds the stretch, which lies within the box's farthest corner of it.
        if !analytic(s) {
            let mut reach: f64 = 0.0;
            for k in 0..8 {
                let corner = Vec3::new(
                    if k & 1 == 0 { bb.min.x } else { bb.max.x },
                    if k & 2 == 0 { bb.min.y } else { bb.max.y },
                    if k & 4 == 0 { bb.min.z } else { bb.max.z },
                );
                reach = reach.max(corner.dist(p));
            }
            if g > reach + tol::LINEAR {
                return;
            }
        }
        // Where the curve's own middle is on the surface, halving drops nothing that matters: the
        // stretch is sampled whole.
        if depth == 0 || g <= tol::LINEAR {
            match out.last_mut() {
                Some(last) if last.1 == t0 => last.1 = t1,
                _ => out.push((t0, t1)),
            }
            return;
        }
        for r in [(t0, m), (m, t1)] {
            let bb = padded_box(c.bbox(r.0, r.1));
            halve(c, r, bb, depth - 1, s, bound, out);
        }
    }
    let mut out = Vec::new();
    halve(curve, range, whole, 8, s, bound, &mut out);
    out
}

/// Where edge `e` meets face `y`, whose box is `bound`, other than at the face's own vertices
/// `own`: inside it or on its boundary. Near a shared vertex, a meeting the edge reaches without
/// leaving the surface by more than the tolerance is that vertex, however far along it rounding put
/// the root. A stretch of edge that lies in the face's surface meets the face where it crosses the
/// face's boundary, or else lies wholly inside or outside it, as its middle does.
fn edge_meets_face(
    b: &Brep,
    e: EdgeId,
    y: FaceId,
    bound: &Aabb,
    own: &[VertexId],
    cache: &mut PairCache,
) -> Option<Point3> {
    let edge = b.edge(e);
    let curve = edge.curve.as_ref()?;
    let whole = cache.edge_box(b, e)?;
    let s = &b.face(y).surface;
    let ends: Vec<(f64, Point3, f64)> = [(edge.range.0, edge.start), (edge.range.1, edge.end)]
        .into_iter()
        .filter(|(_, v)| own.contains(v))
        .map(|(t, v)| (t, b.vertex(v).point, b.vertex_tolerance(v)))
        .collect();
    let along = b.edge_tolerance(e);
    let at_shared = |t: f64, p: Point3| {
        ends.iter().any(|&(tv, q, within)| {
            q.dist(p) <= within
                || (0..=8).all(|k| {
                    let x = curve.eval(t + (tv - t) * k as f64 / 8.0);
                    signed_distance(s, x).abs() <= along
                })
        })
    };
    let near_end = |p: Point3| ends.iter().any(|&(_, q, within)| q.dist(p) <= within);
    let mut found: Vec<Point3> = Vec::new();
    for range in near_ranges(curve, edge.range, whole, s, bound) {
        let hits = curve_surface(curve, range, s);
        if !hits.on_surface {
            found.extend(
                hits.ts
                    .iter()
                    .map(|&t| (t, curve.eval(t)))
                    .filter(|&(t, p)| !at_shared(t, p))
                    .map(|(_, p)| p),
            );
            continue;
        }
        found.extend((1..4).map(|k| curve.eval(range.0 + (range.1 - range.0) * k as f64 / 4.0)));
    }
    // A root is checked against the surface itself: beside a spline face's boundary a point
    // projects onto the boundary, and its signed distance along the normal there reads nil
    // however far off it lies in the tangent plane.
    let reach = 10.0 * along.max(tol::LINEAR);
    found.retain(|&p| in_box(p, bound) && !near_end(p) && gap(s, p) <= reach);
    if found.is_empty() {
        return None;
    }
    let dom = cache.domains.entry(y).or_insert_with(|| face_domain(b, y));
    found.into_iter().find(|&p| dom.on_boundary(p) || dom.exact_side(p).is_some())
}
