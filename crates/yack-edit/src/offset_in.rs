// yack: the inward offset, a solid eroded by a ball (OF2, moved out of lib.rs).
//! The inward offset: every face moved in, the concave edges rounded about where they were, and the
//! refusals where the erosion would change the solid's topology.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    blend,
    error::{Error, Result},
    geom::Surface,
    math::{Point3, Vec2},
    measure, offset_faces, tol, topo,
    topo::{Brep, EdgeId, FaceId},
};

/// The edges of `b` where its faces meet concavely, the solid's inside a wedge wider than a half
/// turn: [`convex_edges`]'s other side.
pub fn concave_edges(b: &Brep) -> Vec<EdgeId> {
    creased_edges(b, false)
}

/// The edges of `b` between two faces at a crease, convex ones or concave ones.
pub(crate) fn creased_edges(b: &Brep, convex: bool) -> Vec<EdgeId> {
    let outward = |f: FaceId, p: Point3| {
        let s = &b.face(f).surface;
        let uv = s.project(p);
        let n = s.normal(uv.x, uv.y);
        if b.face(f).reversed { -n } else { n }
    };
    let mut out = Vec::new();
    for e in b.edge_ids() {
        let edge = b.edge(e);
        let Some(c) = &edge.curve else { continue };
        let &[cf, cg] = b.edge_uses(e) else { continue };
        let (f, g) = (b.coedge_face(cf), b.coedge_face(cg));
        if f == g {
            continue;
        }
        let (p, dp) = c.d1(0.5 * (edge.range.0 + edge.range.1));
        let (n, m) = (outward(f, p), outward(g, p));
        if n.cross(m).norm() <= 1e-7 {
            continue;
        }
        let t = if b.coedge(cf).reversed { -dp } else { dp };
        let side = m.dot(n.cross(t));
        if (convex && side < 0.0) || (!convex && side > 0.0) {
            out.push(e);
        }
    }
    out
}

/// The points of `s` farther than `d` inside it: its erosion by a ball of radius `d`. Every face
/// moves `d` inward; where two faces meet concavely the moved ones part, and the erosion's boundary
/// there is a cylinder of radius `d` about the old edge, the moved solid's concave edge rounded
/// at `d` (a ball's patch at each concave corner). Convex edges and corners stay sharp: their
/// moved faces meet. A round of radius at most `d` vanishes: the erosion there is the erosion of
/// the edge it rounded, so it is taken out first, its neighbours meeting in that edge. Declines
/// where another round face would collapse (a rod, ball or tube no thicker than `d`), which
/// changes the solid's topology.
pub(crate) fn offset_inward(s: &Brep, d: f64, feature: &str) -> Result<Brep> {
    let by: BTreeMap<FaceId, f64> = s.face_ids().map(|f| (f, -d)).collect();
    let sharp = crate::tweak::drop_vanishing_rounds(s, &by)?.map(|(b, _)| b);
    let s = sharp.as_ref().unwrap_or(s);
    if topo::local::radius_collapses(s, None, -d, false) {
        return Err(Error::Unsupported(
            "an inward offset that collapses a round face (a rod, ball or tube no thicker than the offset)".into(),
        ));
    }
    if thin_wall(s, d) {
        return Err(Error::Unsupported(
            "an inward offset past a flat wall's half thickness (the solid parts or vanishes there)".into(),
        ));
    }
    if bends_tighter_than(s, d) {
        return Err(Error::Unsupported(
            "an inward offset past a face's least radius of curvature (its offset folds over)".into(),
        ));
    }
    // Past the guard the input is a fair one: a step that calls it invalid (a round too big for its
    // faces) has met a case this rung does not build, and the next rung may.
    let declined = |e: Error| match e {
        Error::InvalidInput(m) => Error::Unsupported(format!("an inward offset: {m}")),
        e => e,
    };
    let by: BTreeMap<FaceId, f64> = s.face_ids().map(|f| (f, -d)).collect();
    let moved = offset_faces(s, &by).map_err(declined)?;
    let edges = concave_edges(&moved);
    if edges.is_empty() {
        return Ok(moved);
    }
    if rounds_cross(&moved, &edges) {
        return Err(Error::Unsupported(
            "an inward offset whose rounds meet over a convex edge (they cross in a crease there)".into(),
        ));
    }
    blend::fillet_edges(&moved, &edges, d, feature).map_err(declined)
}

/// [`offset_faces`] with the rounds the move goes past taken out first (a convex one moved in, a
/// concave one out, past its radius; see [`crate::tweak::drop_vanishing_rounds`]): their
/// neighbours meet in the edge they rounded, which then moves as any edge does.
pub(crate) fn offset_faces_past_rounds(s: &Brep, by: &BTreeMap<FaceId, f64>) -> Result<Brep> {
    match crate::tweak::drop_vanishing_rounds(s, by)? {
        None => offset_faces(s, by),
        Some((sharp, map)) => {
            let by: BTreeMap<FaceId, f64> = by.iter().filter_map(|(f, &d)| Some((*map.get(f)?, d))).collect();
            offset_faces(&sharp, &by)
        }
    }
}

/// Whether `s` has a flat wall no thicker than `2d`: two plane faces turned apart with the
/// material between them, whose boxes overlap across the gap. Eroding by `d` there parts the solid
/// (a dumbbell's neck) or leaves nothing of it. Boxes overlap where faces need not, so this may
/// decline more than it must, never less.
fn thin_wall(s: &Brep, d: f64) -> bool {
    let planes: Vec<_> = s
        .face_ids()
        .filter_map(|f| match &s.face(f).surface {
            Surface::Plane(pl) => {
                let n = if s.face(f).reversed { -pl.frame.z } else { pl.frame.z };
                Some((n, n.dot(pl.frame.origin), measure::face_bbox(s, f)))
            }
            _ => None,
        })
        .collect();
    planes.iter().enumerate().any(|(i, &(n, c, ref a))| {
        planes.iter().skip(i + 1).any(|&(m, e, ref b)| {
            // Turned apart: m = −n, and the material lies between the planes n·x = c and n·x = −e.
            let gap = c + e;
            n.dot(m) <= -1.0 + tol::TANGENT
                && gap > 0.0
                && gap <= 2.0 * d
                && [(a.min.x, a.max.x, b.min.x, b.max.x), (a.min.y, a.max.y, b.min.y, b.max.y), (a.min.z, a.max.z, b.min.z, b.max.z)]
                    .iter()
                    .all(|&(lo, hi, blo, bhi)| lo <= bhi + gap && blo <= hi + gap)
        })
    })
}

/// Whether a curved face of `s` bends convexly tighter than `d` somewhere: an inward offset there
/// passes the face's centre of curvature and folds over (an elliptic wall past b²/a). Sampled on a
/// 9 × 9 grid of each curved face's parameter box, at the points inside its trims.
fn bends_tighter_than(s: &Brep, d: f64) -> bool {
    let outlines = measure::Outlines::new(s);
    s.face_ids().any(|f| {
        let surface = &s.face(f).surface;
        if matches!(surface, Surface::Plane(_)) {
            return false;
        }
        let dom = topo::face_domain(s, f);
        let at = |i: u32, lo: f64, hi: f64| lo + (hi - lo) * f64::from(i) / 8.0;
        (0..=8).any(|i| {
            (0..=8).any(|j| {
                let uv = Vec2::new(at(i, dom.lo.x, dom.hi.x), at(j, dom.lo.y, dom.hi.y));
                dom.winding(uv) != 0 && outlines.curvature(surface.eval(uv.x, uv.y)).is_some_and(|[k1, _]| k1 * d >= 1.0)
            })
        })
    })
}

/// Whether two of `edges` (the concave ones of `b`) meet at a vertex another edge also reaches.
/// The erosion's cylinders about them cross there in a crease; a fillet would swing a ball round
/// it instead. A vertex where only concave edges meet takes a ball's patch, which is the erosion's
/// own boundary there, and a vertex with one concave edge ends its round on the faces it meets.
pub(crate) fn rounds_cross(b: &Brep, edges: &[EdgeId]) -> bool {
    let concave: BTreeSet<EdgeId> = edges.iter().copied().collect();
    let mut at = BTreeMap::new();
    for e in b.edge_ids() {
        let edge = b.edge(e);
        if edge.curve.is_none() {
            continue;
        }
        let ends: BTreeSet<_> = [edge.start, edge.end].into_iter().collect();
        for v in ends {
            let (all, hollow) = at.entry(v).or_insert((0_usize, 0_usize));
            *all += 1;
            if concave.contains(&e) {
                *hollow += 1;
            }
        }
    }
    at.values().any(|&(all, hollow)| hollow >= 2 && all > hollow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Frame;
    use std::f64::consts::PI;

    fn prism(pts: &[[f64; 2]], h: f64) -> Brep {
        let pts: Vec<Vec2> = pts.iter().map(|p| Vec2::new(p[0], p[1])).collect();
        let names: Vec<String> = (0..pts.len()).map(|i| format!("e{i}")).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        crate::sweep::extrude(&crate::profile::Profile::polygon(&pts, &names), &Frame::WORLD, h, "p").unwrap()
    }

    fn close(got: f64, want: f64) {
        assert!((got - want).abs() < 1e-7 * want.max(1.0), "{got} against {want}");
    }

    #[test]
    fn an_inward_offset_keeps_convex_edges_sharp() {
        // A 10 × 8 × 6 block offset −1 is the 8 × 6 × 4 block: an erosion's convex edges stay sharp.
        let b = prism(&[[0.0, 0.0], [10.0, 0.0], [10.0, 8.0], [0.0, 8.0]], 6.0);
        let o = crate::offset(&b, -1.0, "o").unwrap();
        close(measure::volume(&o), 192.0);
        close(measure::area(&o), 208.0);
    }

    #[test]
    fn an_inward_offset_rounds_a_concave_edge_about_it() {
        // An L (arms 20 × 5 and 5 × 15), 8 tall, offset −1: its outline eroded is the arms' boxes
        // 18 × 3 and 3 × 10, and at the concave corner the unit square past their mitre less the
        // quarter disc about the old edge, 1 − π/4; 6 tall. Its outline's length is 60 + π/2.
        let b = prism(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]], 8.0);
        let o = crate::offset(&b, -1.0, "o").unwrap();
        let plan = 85.0 - PI / 4.0;
        close(measure::volume(&o), plan * 6.0);
        close(measure::area(&o), 2.0 * plan + (60.0 + PI / 2.0) * 6.0);
    }

    #[test]
    fn a_torus_offset_inward_thins_its_tube() {
        // Torus R 10, r 3, offset −1.5: the torus of tube 1.5, V = 2π²Rr², A = 4π²Rr.
        let t = crate::primitive::torus(&Frame::WORLD, 10.0, 3.0, "t").unwrap();
        let o = crate::offset(&t, -1.5, "o").unwrap();
        close(measure::volume(&o), 2.0 * PI * PI * 10.0 * 1.5 * 1.5);
        close(measure::area(&o), 4.0 * PI * PI * 10.0 * 1.5);
    }

    #[test]
    fn an_inward_offset_past_a_wall_or_a_bend_declines() {
        // A dumbbell whose neck is 2 thick: offset −1.5 parts it; −0.5 keeps it whole. A ball of
        // radius 2 bends tighter than 2.5, not than 1.5.
        let bell = prism(
            &[[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [14.0, 4.0], [14.0, 0.0], [24.0, 0.0], [24.0, 10.0], [14.0, 10.0], [14.0, 6.0], [10.0, 6.0], [10.0, 10.0], [0.0, 10.0]],
            6.0,
        );
        assert!(matches!(crate::offset(&bell, -1.5, "o"), Err(Error::Unsupported(_))));
        assert!(!thin_wall(&bell, 0.5));
        let ball = crate::primitive::sphere(&Frame::WORLD, 2.0, "b").unwrap();
        assert!(bends_tighter_than(&ball, 2.5) && !bends_tighter_than(&ball, 1.5));
    }

    /// `s` with its convex edges rounded `r`.
    fn rounded(s: Brep, r: f64) -> Brep {
        let e: Vec<EdgeId> = crate::convex_edges(&s).unwrap();
        blend::fillet_edges(&s, &e, r, "r").unwrap()
    }

    #[test]
    fn an_inward_offset_past_a_puck_s_rounds_is_the_sharp_puck_s() {
        // A puck of radius 6, 10 tall, both rims rounded 2: offset −3 (past the rounds) is the
        // puck of radius 3, 4 tall; offset −2 (just to them) the puck of radius 4, 6 tall.
        let puck = rounded(crate::primitive::cylinder(&Frame::WORLD, 6.0, 10.0, "c").unwrap(), 2.0);
        let past = crate::offset(&puck, -3.0, "o").unwrap();
        close(measure::volume(&past), PI * 9.0 * 4.0);
        close(measure::area(&past), 2.0 * PI * 9.0 + 2.0 * PI * 3.0 * 4.0);
        let by = crate::offset(&puck, -2.0, "o").unwrap();
        close(measure::volume(&by), PI * 16.0 * 6.0);
        close(measure::area(&by), 2.0 * PI * 16.0 + 2.0 * PI * 4.0 * 6.0);
    }

    #[test]
    fn an_inward_offset_past_a_hemisphere_s_round_is_a_cap() {
        // A hemisphere of radius 5, its rim rounded 1, offset −1.5: the ball of radius 3.5 above
        // the plane 1.5 up, a cap 2 high, V = πh²(3R − h)/3, A = 2πRh + π(R² − (R − h)²).
        let ball = crate::primitive::sphere(&Frame::WORLD, 5.0, "b").unwrap();
        let below = crate::primitive::block(&Frame::WORLD.translated(crate::math::Vec3::new(-6.0, -6.0, -6.0)), crate::math::Vec3::new(12.0, 12.0, 6.0), "k").unwrap();
        let half = rounded(crate::boolean::difference(&ball, &below).unwrap(), 1.0);
        let o = crate::offset(&half, -1.5, "o").unwrap();
        let (r, h) = (3.5, 2.0);
        close(measure::volume(&o), PI * h * h * (3.0 * r - h) / 3.0);
        close(measure::area(&o), 2.0 * PI * r * h + PI * (r * r - (r - h) * (r - h)));
    }

    #[test]
    fn an_inward_offset_past_a_frustum_s_rounds_is_the_sharp_frustum_s() {
        // A frustum of radii 6 and 3, 6 tall, both rims rounded 1.5, offset −2: the sharp frustum's
        // erosion, its flat ends 2 in and its side's line ρ = 6 − z/2 moved 2 in along its normal,
        // √5 across: the frustum from z = 2 to 4 of radii 5 − √5 and 4 − √5, its side √5 long.
        let k = rounded(crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 6.0, "k").unwrap(), 1.5);
        let o = crate::offset(&k, -2.0, "o").unwrap();
        let (a, c) = (5.0 - 5f64.sqrt(), 4.0 - 5f64.sqrt());
        close(measure::volume(&o), PI * 2.0 / 3.0 * (a * a + a * c + c * c));
        close(measure::area(&o), PI * (a * a + c * c) + PI * (a + c) * 5f64.sqrt());
    }

    #[test]
    fn an_inward_offset_past_a_turned_post_s_round_is_the_sharp_post_s() {
        // A post of radius 5, 6 tall, under a frustum 5 to 2 over 4, the step between rounded 1,
        // offset −1.5: the cylinder of radius 3.5 from z = 1.5 meets the cone's side moved in
        // (ρ = 3.125 − 0.75·(z − 6)) at z = 5.5, which runs to the top moved down to 8.5, ρ = 1.25.
        let post = crate::boolean::union(
            &crate::primitive::cylinder(&Frame::WORLD, 5.0, 6.0, "c").unwrap(),
            &crate::primitive::cone(&Frame::WORLD.translated(crate::math::Vec3::new(0.0, 0.0, 6.0)), 5.0, 2.0, 4.0, "k").unwrap(),
        )
        .unwrap();
        let step: Vec<EdgeId> = crate::convex_edges(&post)
            .unwrap()
            .into_iter()
            .filter(|&e| post.edge(e).curve.as_ref().is_some_and(|c| (c.eval(post.edge(e).range.0).z - 6.0).abs() < 1e-6))
            .collect();
        let post = blend::fillet_edges(&post, &step, 1.0, "r").unwrap();
        let o = crate::offset(&post, -1.5, "o").unwrap();
        close(measure::volume(&o), PI * (12.25 * 4.0 + (12.25 + 3.5 * 1.25 + 1.5625)));
        close(measure::area(&o), PI * (12.25 + 28.0 + 4.75 * 3.75 + 1.5625));
    }

    #[test]
    fn an_inward_offset_past_a_round_by_a_concave_edge_declines() {
        // A puck whose top rim is rounded 2, with a boss of radius 3.5 on top (on the flat, inside the round): its foot, a
        // concave edge 2.5 from the rim the round rounded, could change the erosion there, so
        // offset −3 declines.
        let puck = crate::primitive::cylinder(&Frame::WORLD, 6.0, 10.0, "c").unwrap();
        let rim: Vec<EdgeId> = crate::convex_edges(&puck)
            .unwrap()
            .into_iter()
            .filter(|&e| puck.edge(e).curve.as_ref().is_some_and(|c| (c.eval(puck.edge(e).range.0).z - 10.0).abs() < 1e-6))
            .collect();
        let puck = blend::fillet_edges(&puck, &rim, 2.0, "r").unwrap();
        let boss = crate::primitive::cylinder(&Frame::WORLD.translated(crate::math::Vec3::new(0.0, 0.0, 10.0)), 3.5, 1.0, "b").unwrap();
        let bossed = crate::boolean::union(&puck, &boss).unwrap();
        let e = crate::offset(&bossed, -3.0, "o").unwrap_err();
        assert!(matches!(&e, Error::Unsupported(m) if m.contains("concave edge lies near")), "{e}");
    }

    #[test]
    fn a_growth_past_a_root_round_is_the_sharp_root_s() {
        // A puck of radius 6, 4 tall, under a boss of radius 2, 4 tall, the root between them
        // rounded 0.5: grown by 1 the root's round is gone, and the solid is the sharp stepped
        // puck grown by 1, built here without the round.
        let at = |z: f64| Frame::WORLD.translated(crate::math::Vec3::new(0.0, 0.0, z));
        let sharp = crate::boolean::union(
            &crate::primitive::cylinder(&Frame::WORLD, 6.0, 4.0, "p").unwrap(),
            &crate::primitive::cylinder(&at(4.0), 2.0, 4.0, "b").unwrap(),
        )
        .unwrap();
        let root = concave_edges(&sharp);
        assert_eq!(root.len(), 1);
        let rooted = blend::fillet_edges(&sharp, &root, 0.5, "r").unwrap();
        let (grown, want) = (crate::offset(&rooted, 1.0, "o").unwrap(), crate::offset(&sharp, 1.0, "o").unwrap());
        close(measure::volume(&grown), measure::volume(&want));
        close(measure::area(&grown), measure::area(&want));
        // Short of the round's radius the round stays, a round of 0.25 filling the root's corner.
        assert!(measure::volume(&crate::offset(&rooted, 0.25, "o").unwrap()) > measure::volume(&crate::offset(&sharp, 0.25, "o").unwrap()));
    }

    #[test]
    fn an_inward_offset_that_eats_a_rod_declines() {
        // A rod of radius 1, 10 long: offset −0.5 is the rod of radius 0.5, 9 long; offset −1.5
        // leaves nothing of its round face, a topology change this rung declines.
        let rod = crate::primitive::cylinder(&Frame::WORLD, 1.0, 10.0, "rod").unwrap();
        close(measure::volume(&crate::offset(&rod, -0.5, "o").unwrap()), PI * 0.25 * 9.0);
        assert!(matches!(crate::offset(&rod, -1.5, "o"), Err(Error::Unsupported(_))));
    }

    /// Not a check: what following an inward offset through its vanishing rounds gives on the
    /// bench's rounded solids (step 2's probe). Run with `--ignored --nocapture`.
    #[test]
    #[ignore]
    fn probe_rounds_past_their_radius() {
        let at = |z: f64| Frame::WORLD.translated(crate::math::Vec3::new(0.0, 0.0, z));
        let rounded = |s: Brep, r: f64| {
            let e: Vec<EdgeId> = crate::convex_edges(&s).unwrap();
            blend::fillet_edges(&s, &e, r, "r").unwrap()
        };
        let puck = rounded(crate::primitive::cylinder(&Frame::WORLD, 6.0, 10.0, "c").unwrap(), 2.0);
        let frustum = rounded(crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 6.0, "k").unwrap(), 1.5);
        let post = crate::boolean::union(
            &crate::primitive::cylinder(&Frame::WORLD, 5.0, 6.0, "c").unwrap(),
            &crate::primitive::cone(&at(6.0), 5.0, 2.0, 4.0, "k").unwrap(),
        )
        .unwrap();
        let post_edge: Vec<EdgeId> = post
            .edge_ids()
            .filter(|&e| post.edge(e).curve.as_ref().is_some_and(|c| (c.eval(post.edge(e).range.0).z - 6.0).abs() < 1e-6))
            .collect();
        let post = blend::fillet_edges(&post, &post_edge, 1.0, "r").unwrap();
        for (name, s, d) in [("puck past", &puck, 3.0), ("puck by", &puck, 2.0), ("frustum past", &frustum, 2.0), ("post past", &post, 1.5)] {
            let by: BTreeMap<FaceId, f64> = s.face_ids().map(|f| (f, -d)).collect();
            match crate::tweak::offset_followed(s, &by) {
                Ok((b, changed)) => eprintln!(
                    "PROBE {name}: ok changed={changed} faces={} V={} valid={:?}",
                    b.face_ids().count(),
                    measure::volume(&b),
                    crate::check::check(&b).len()
                ),
                Err(e) => eprintln!("PROBE {name}: {e}"),
            }
            match crate::offset(s, -d, "o") {
                Ok(b) => eprintln!("PROBE {name} offset: V={} faces={}", measure::volume(&b), b.face_ids().count()),
                Err(e) => eprintln!("PROBE {name} offset: {e}"),
            }
        }
    }
}
