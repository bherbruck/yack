// yack: An edge's length, points spread over a face, how far a face or a solid reaches along a
// Adapted from springrep@a03e8a1 src/measure/sample.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! An edge's length, points spread over a face, how far a face or a solid reaches along a
//! direction, and one solid's box and area.

use super::{face_area, face_box, reached_patches, stationary, uv_polygons};
use crate::{
    topo::face_domain,
    geom::Surface,
    math::{Aabb, Point3, Vec2, Vec3},
    numeric::integrate,
    topo::{Brep, EdgeId, FaceId, SolidId},
};

/// The length of edge `e`, from its exact curve.
pub fn edge_length(b: &Brep, e: EdgeId) -> f64 {
    let edge = b.edge(e);
    let Some(c) = &edge.curve else { return 0.0 };
    let (t0, t1) = edge.range;
    let mut cuts = vec![t0];
    cuts.extend(c.breakpoints(t0.min(t1), t0.max(t1)));
    cuts.push(t1);
    cuts.sort_by(f64::total_cmp);
    let mut speed = |t: f64| c.d1(t).1.norm();
    let pieces = cuts.windows(2);
    pieces
        .map(|w| integrate(&mut speed, w[0], w[1], 1e-10))
        .sum()
}

/// The box of face `f`'s parameters: the least and greatest `(u, v)` its outline reaches.
pub fn face_uv_bounds(b: &Brep, f: FaceId) -> (Vec2, Vec2) {
    let domain = face_domain(b, f);
    let (mut lo, mut hi) = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for uv in domain.samples().iter().flatten().flatten() {
        lo = Vec2::new(lo.x.min(uv.x), lo.y.min(uv.y));
        hi = Vec2::new(hi.x.max(uv.x), hi.y.max(uv.y));
    }
    (lo, hi)
}

/// How far face `f` reaches along `dir`: the least and greatest of `p · dir` over its points,
/// `dir` made a unit, from the exact geometry. Its edges' extremes, and where a curved face
/// bulges past them, the point where it turns.
pub fn face_extent(b: &Brep, f: FaceId, dir: Vec3) -> (f64, f64) {
    let Some(d) = dir.unit() else {
        return (0.0, 0.0);
    };
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut reach = |p: Point3| {
        let s = (p - Point3::new(0.0, 0.0, 0.0)).dot(d);
        (lo, hi) = (lo.min(s), hi.max(s));
    };
    let face = b.face(f);
    for &lp in &face.loops {
        for &co in &b.lp(lp).coedges {
            let edge = b.edge(b.coedge(co).edge);
            reach(b.vertex(edge.start).point);
            reach(b.vertex(edge.end).point);
            if let Some(c) = &edge.curve {
                for t in c.extremes(d, edge.range.0, edge.range.1) {
                    reach(c.eval(t));
                }
            }
        }
    }
    let s = &face.surface;
    let bulges = matches!(
        s,
        Surface::Sphere(_)
            | Surface::Torus(_)
            | Surface::Revolution(_)
            | Surface::Ruled(_)
            | Surface::Nurbs(_)
    );
    if bulges {
        let polys = uv_polygons(b, f);
        // A B-spline lies inside its patches' boxes: only a patch that reaches past what the
        // edges reach can widen the extent, and only there is a turning point looked for.
        let starts = match s {
            Surface::Nurbs(n) => {
                let mut starts = Vec::new();
                for q in reached_patches(n, &polys) {
                    let (bl, bh) = (q.bbox.min, q.bbox.max);
                    let along = |k: usize| {
                        let pick = |i: usize| if k >> i & 1 == 1 { bh[i] } else { bl[i] };
                        pick(0) * d.x + pick(1) * d.y + pick(2) * d.z
                    };
                    let (ql, qh) = (0..8)
                        .map(along)
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |a, x| {
                            (a.0.min(x), a.1.max(x))
                        });
                    if ql >= lo && qh <= hi {
                        continue;
                    }
                    for i in 0..3 {
                        for j in 0..3 {
                            let at =
                                |a: f64, z: f64, k: usize| a + (z - a) * (k as f64 + 0.5) / 3.0;
                            starts.push(Vec2::new(at(q.lo.x, q.hi.x, i), at(q.lo.y, q.hi.y, j)));
                        }
                    }
                }
                Some(starts)
            }
            _ => None,
        };
        if starts.as_ref().is_none_or(|s| !s.is_empty()) {
            let turning = stationary(s, &polys, d, starts);
            let mut reach = |p: Point3| {
                let x = (p - Point3::new(0.0, 0.0, 0.0)).dot(d);
                (lo, hi) = (lo.min(x), hi.max(x));
            };
            for uv in turning {
                reach(s.eval(uv.x, uv.y));
            }
        }
    }
    (lo, hi)
}

/// How far the arena reaches along `dir`: the least and greatest of `p · dir` over its faces.
pub fn extent(b: &Brep, dir: Vec3) -> (f64, f64) {
    let each = b.face_ids().map(|f| face_extent(b, f, dir));
    each.fold((f64::INFINITY, f64::NEG_INFINITY), |a, e| {
        (a.0.min(e.0), a.1.max(e.1))
    })
}

/// A box holding solid `s` of the arena: the smallest, but where it has B-spline faces, which may
/// widen it a little.
pub fn solid_bbox(b: &Brep, s: SolidId) -> Aabb {
    let mut bb = Aabb::EMPTY;
    for f in b.solid_faces(s) {
        bb.union(&face_box(b, f));
    }
    bb
}

/// The area of solid `s`'s faces.
pub fn solid_area(b: &Brep, s: SolidId) -> f64 {
    b.solid_faces(s).into_iter().map(|f| face_area(b, f)).sum()
}

/// Points spread over face `f`, each with the solid's outward normal there: the points of an
/// `n × n` grid across the face's parameters that fall inside it. A face narrower than the grid's
/// spacing can come back with none.
pub fn face_samples(b: &Brep, f: FaceId, n: usize) -> Vec<(Point3, Vec3)> {
    let domain = face_domain(b, f);
    let face = b.face(f);
    let (lo, hi) = face_uv_bounds(b, f);
    let mut out = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let at = |k: usize, a: f64, b: f64| a + (k as f64 + 0.5) / n as f64 * (b - a);
            let uv = Vec2::new(at(i, lo.x, hi.x), at(j, lo.y, hi.y));
            if domain.winding(uv) == 0 {
                continue;
            }
            let normal = face.surface.normal(uv.x, uv.y);
            let outward = if face.reversed { -normal } else { normal };
            out.push((face.surface.eval(uv.x, uv.y), outward));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{boolean::difference, math::Frame, profile::Profile, sweep::extrude};

    fn named(b: &Brep, name: &str) -> FaceId {
        let has = |f: &FaceId| b.face(*f).names.iter().any(|n| n.to_string() == name);
        b.face_ids().find(has).unwrap()
    }

    #[test]
    fn a_block_s_edges_and_a_rod_s_rims_have_their_lengths() {
        // A 20 × 10 × 5 block has four edges of each length.
        let block = extrude(
            &Profile::rect(Vec2::ZERO, 20.0, 10.0),
            &Frame::WORLD,
            5.0,
            "b",
        );
        let block = block.unwrap();
        let total: f64 = block.edge_ids().map(|e| edge_length(&block, e)).sum();
        assert!((total - 4.0 * 35.0).abs() < 1e-9, "{total}");
        // A Ø10 rod 7 long: two rims of 10π and the seam.
        let rod = extrude(&Profile::circle(Vec2::ZERO, 10.0), &Frame::WORLD, 7.0, "r").unwrap();
        let mut lengths: Vec<f64> = rod.edge_ids().map(|e| edge_length(&rod, e)).collect();
        lengths.sort_by(f64::total_cmp);
        assert_eq!(lengths.len(), 3);
        assert!((lengths[0] - 7.0).abs() < 1e-9, "{lengths:?}");
        assert!((lengths[1] - 10.0 * PI).abs() < 1e-9 && (lengths[2] - 10.0 * PI).abs() < 1e-9);
    }

    #[test]
    fn samples_lie_on_the_face_with_its_outward_normal_and_miss_a_hole() {
        // The top of a 20 × 20 × 5 block with a Ø10 hole through its middle: a 5 × 5 grid has
        // points 4 mm apart from −8 to 8, and the nine within 5 mm of the middle fall in the hole
        // but for the four at (±4, ±4), which are 5.66 mm out.
        let block = extrude(
            &Profile::rect(Vec2::ZERO, 20.0, 20.0),
            &Frame::WORLD,
            5.0,
            "b",
        );
        let below = Frame::WORLD.translated(Vec3::Z * -1.0);
        let hole = extrude(&Profile::circle(Vec2::ZERO, 10.0), &below, 7.0, "h").unwrap();
        let drilled = difference(&block.unwrap(), &hole).unwrap();
        let samples = face_samples(&drilled, named(&drilled, "b.end"), 5);
        assert_eq!(samples.len(), 25 - 5);
        for (p, n) in &samples {
            assert!((p.z - 5.0).abs() < 1e-12 && (*n - Vec3::Z).norm() < 1e-12);
            assert!(p.x.hypot(p.y) > 5.0);
        }
        // The hole's wall faces into the hole.
        for (p, n) in face_samples(&drilled, named(&drilled, "h.side"), 4) {
            assert!((p.x.hypot(p.y) - 5.0).abs() < 1e-9);
            assert!((n.x * p.x + n.y * p.y + 5.0).abs() < 1e-9, "{n:?} at {p:?}");
        }
    }

    #[test]
    fn a_rod_reaches_its_radius_across_and_its_length_along_whatever_the_direction() {
        // A Ø10 rod 7 long on the z axis: ±5 along any direction square to it, 0 to 7 along it,
        // and along the diagonal of x and z the rim's far point, (5 + 7) / √2.
        let rod = extrude(&Profile::circle(Vec2::ZERO, 10.0), &Frame::WORLD, 7.0, "r").unwrap();
        let near = |got: (f64, f64), want: (f64, f64)| {
            assert!(
                (got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9,
                "{got:?}"
            );
        };
        near(extent(&rod, Vec3::new(3.0, 4.0, 0.0)), (-5.0, 5.0));
        near(extent(&rod, Vec3::Z), (0.0, 7.0));
        let slant = 1.0 / 2f64.sqrt();
        near(
            extent(&rod, Vec3::new(1.0, 0.0, 1.0)),
            (-5.0 * slant, 12.0 * slant),
        );
        // Its wall alone runs the rod's length, and its parameters a whole turn and that length.
        let wall = named(&rod, "r.side");
        near(face_extent(&rod, wall, Vec3::Z), (0.0, 7.0));
        let (lo, hi) = face_uv_bounds(&rod, wall);
        assert!(
            (hi.x - lo.x - std::f64::consts::TAU).abs() < 1e-9,
            "{lo:?} {hi:?}"
        );
        assert!((hi.y - lo.y - 7.0).abs() < 1e-9, "{lo:?} {hi:?}");
    }

    #[test]
    fn a_ball_reaches_its_radius_past_any_edge_it_has() {
        // A ball of radius 3 centred 1 above the origin: its one face bulges to 4 and −2 along z
        // and ±3 along a slant through its centre, where no edge of it reaches.
        let centre = Frame::WORLD.translated(Vec3::Z);
        let ball = crate::primitive::sphere(&centre, 3.0, "b").unwrap();
        let (lo, hi) = extent(&ball, Vec3::Z);
        assert!(
            (lo + 2.0).abs() < 1e-9 && (hi - 4.0).abs() < 1e-9,
            "{lo} {hi}"
        );
        let (lo, hi) = extent(&ball, Vec3::new(1.0, 1.0, 0.0));
        assert!(
            (lo + 3.0).abs() < 1e-9 && (hi - 3.0).abs() < 1e-9,
            "{lo} {hi}"
        );
    }

    #[test]
    fn each_solid_of_an_arena_has_its_own_box_and_area() {
        // A 2 × 2 × 2 block at the origin and a 4 × 4 × 1 one 10 along x, in one arena.
        let small = extrude(
            &Profile::rect(Vec2::ZERO, 2.0, 2.0),
            &Frame::WORLD,
            2.0,
            "a",
        );
        let far = Frame::WORLD.translated(Vec3::X * 10.0);
        let wide = extrude(&Profile::rect(Vec2::ZERO, 4.0, 4.0), &far, 1.0, "b").unwrap();
        let mut both = small.unwrap();
        both.append(&wide);
        let solids: Vec<SolidId> = both.solid_ids().collect();
        assert_eq!(solids.len(), 2);
        let (first, second) = (solid_bbox(&both, solids[0]), solid_bbox(&both, solids[1]));
        assert!((first.max.x - 1.0).abs() < 1e-12 && (first.max.z - 2.0).abs() < 1e-12);
        assert!((second.min.x - 8.0).abs() < 1e-12 && (second.max.z - 1.0).abs() < 1e-12);
        assert!((solid_area(&both, solids[0]) - 24.0).abs() < 1e-9);
        assert!((solid_area(&both, solids[1]) - 48.0).abs() < 1e-9);
    }

    #[test]
    fn a_barrel_s_spline_side_reaches_its_middle_section_past_both_rims() {
        // A smooth loft through circles of radius 5, 10 and 5 at z = 0, 5 and 10: its one side
        // is a spline face whose rims reach 5 and whose middle reaches 10, where no edge is.
        use crate::loft::{Section, loft};
        let section = |z: f64, r: f64| Section {
            profile: Profile::circle(Vec2::ZERO, 2.0 * r),
            frame: Frame::WORLD.translated(Vec3::Z * z),
        };
        let sections = [section(0.0, 5.0), section(5.0, 10.0), section(10.0, 5.0)];
        let barrel = loft(&sections, false, "b").unwrap();
        for dir in [Vec3::X, Vec3::new(1.0, 1.0, 0.0)] {
            let (lo, hi) = extent(&barrel, dir);
            assert!(
                (lo + 10.0).abs() < 1e-6 && (hi - 10.0).abs() < 1e-6,
                "{lo} {hi}"
            );
        }
        let (lo, hi) = extent(&barrel, Vec3::Z);
        assert!(lo.abs() < 1e-9 && (hi - 10.0).abs() < 1e-9, "{lo} {hi}");
    }
}
