// yack: fillets over existing rounds and fillets meeting at corners, measured against hand-worked volumes.
// Adapted from springrep@a03e8a1 tests/fillets_over_rounds.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); edges named by face-name pairs are
// selected by role: springrep's `feature.face` is `faces_by(feature, role)` on a solid `named(feature)`,
// `start`/`end` are `Role::Start`/`Role::End` and a profile's n-th side is `Role::Side("s{n}")` (a
// rectangle's v_min, u_max, v_max, u_min are s0..s3, an outline's sides are numbered in the original's
// order; the comments give springrep's names); fillets follow the tangent chain, as springrep's do;
// springrep's centred `Profile::rect` is written as `Profile::rectangle(min, max)`; `unify` has no §4.3
// counterpart and is dropped (it only preceded checks and measures); `Surface::Nurbs` is read as
// `SurfaceKind::Spline`; "watertight" is `Mesh::is_watertight` at chord 0.01; the tangency walk visits
// each patch's edges through `edges_of`/`faces_of`, samples them with `EdgeRef::points` and compares
// `FaceRef::normal_at` (each face's own surface normal at the point, as springrep's project + normal).
//! Fillets that run over rounds already on the body, and fillets that meet at a corner. A fillet
//! no smaller than a round it crosses rolls round it in a torus; a larger one closes the corner
//! with a patch tangent to every face it meets. Each result is measured against a volume worked
//! by hand from its sections.

use std::f64::consts::{PI, TAU};

use yack::prelude::*;

/// The side swept from a profile's n-th segment.
fn side(n: u32) -> Role {
    Role::Side(format!("s{n}").into())
}

/// A rectangle's sides as springrep names them.
fn v_min() -> Role {
    side(0)
}
fn u_max() -> Role {
    side(1)
}
fn v_max() -> Role {
    side(2)
}
fn u_min() -> Role {
    side(3)
}

type Name<'a> = (&'a str, Role);

/// Every edge where a face named `a` meets one named `b`, for each `(a, b)` of `pairs`.
fn edges_named(b: &Solid, pairs: &[(Name, Name)]) -> Vec<EdgeId> {
    let mut out = Vec::new();
    for ((ta, ra), (tb, rb)) in pairs {
        for fa in b.faces_by(*ta, ra.clone()) {
            for fb in b.faces_by(*tb, rb.clone()) {
                for e in b.edges_between(fa, fb).unwrap() {
                    if !out.contains(&e) {
                        out.push(e);
                    }
                }
            }
        }
    }
    assert!(!out.is_empty(), "no edge named {pairs:?}");
    out
}

fn upright_edges() -> [(Name<'static>, Name<'static>); 4] {
    [
        (("block", u_min()), ("block", v_min())),
        (("block", v_min()), ("block", u_max())),
        (("block", u_max()), ("block", v_max())),
        (("block", v_max()), ("block", u_min())),
    ]
}

/// The 20 × 20 × 10 block on the xy plane, centred on the origin.
fn block() -> Solid {
    extrude(&Frame::WORLD, &Profile::rectangle([-10.0, -10.0], [10.0, 10.0]).unwrap(), 10.0).unwrap().named("block")
}

fn line(a: (f64, f64), b: (f64, f64)) -> Segment {
    Segment::Line { a: [a.0, a.1].into(), b: [b.0, b.1].into() }
}

fn arc(center: (f64, f64), radius: f64, start: f64, sweep: f64) -> Segment {
    Segment::Arc { center: [center.0, center.1].into(), radius, start: rad(start), sweep: rad(sweep) }
}

fn outline(segments: Vec<Segment>) -> Profile {
    Profile::new(Loop::new(segments).unwrap(), vec![]).unwrap()
}

/// `profile` as a boss 4 tall on the block's top, or with `cut` a pocket 4 deep in it.
fn on_block(profile: &Profile, cut: bool, id: &str) -> Solid {
    if cut {
        block().difference(&extrude(&Frame::at([0.0, 0.0, 6.0]), profile, 4.0).unwrap().named(id)).unwrap()
    } else {
        block().union(&extrude(&Frame::at([0.0, 0.0, 10.0]), profile, 4.0).unwrap().named(id)).unwrap()
    }
}

fn watertight(b: &Solid) -> bool {
    // springrep: MeshOptions { linear: 0.01, angular: 0.2 } is chord 0.01.
    b.mesh(0.01).expect("the result meshes").is_watertight()
}

fn faces_of(b: &Solid, pick: fn(&SurfaceKind) -> bool) -> usize {
    b.faces_where(|f| pick(&f.kind())).len()
}

fn patches_of(b: &Solid) -> Vec<FaceId> {
    b.faces_where(|f| matches!(f.kind(), SurfaceKind::Spline))
}

/// Each of `patches` is tangent to every face it meets, exactly: their outward normals agree all
/// along each edge between them.
fn assert_tangent_all_round(b: &Solid, patches: &[FaceId]) {
    let normal = |f: FaceId, p: Point3| b.face(f).unwrap().normal_at(p).expect("a normal");
    for &f in patches {
        for e in b.edges_of(f).unwrap() {
            // A seam's other side is the patch itself.
            let faces = b.faces_of(e).unwrap();
            assert!(!faces.is_empty(), "every edge has two faces");
            let other = faces.iter().copied().find(|&g| g != f).unwrap_or(f);
            // springrep: t = t0 + (t1 − t0)·i/34 for i in 1..34 (none for an edge without a curve).
            let points = b.edge(e).unwrap().points(35);
            for &p in points.iter().skip(1).take(33) {
                let angle = normal(f, p).cross(normal(other, p)).length();
                assert!(angle < 1e-9, "a crease of {angle} rad at {p:?}");
                assert!(normal(f, p).dot(normal(other, p)) > 0.0);
            }
        }
    }
}

/// The block's upright edges rounded `small`, then the edges of its `face` (`end`, the top, or
/// `start`, the bottom) `big`.
fn rounded_then_filleted(small: f64, big: f64, face: &str) -> Solid {
    let b0 = block();
    let b = b0.fillet(&edges_named(&b0, &upright_edges()), small).unwrap();
    let rim = if face == "end" { Role::End } else { Role::Start };
    b.fillet(&edges_named(&b, &[(("block", rim), ("block", u_min()))]), big).unwrap()
}

#[test]
fn a_fillet_round_large_and_small_rounds_rolls_round_one_and_patches_the_other() {
    // The R1.5 top fillet runs round two R6 upright rounds, which it rolls round in tori, and two
    // R0.5 ones, which it closes with patches. Above z = 8.5 the section at height z, with
    // k = √(1 − ((z − 8.5)/1.5)²) and d = 1.5(1 − k), is a square of half-width 10 − d whose R6
    // corners are 6 − d and whose R0.5 corners are 0.5k. A rounded square of side s with corner
    // radii cᵢ has area s² − (1 − π/4)·Σcᵢ², so with ∫dz = 1.5, ∫k dz = 1.5·π/4, ∫k² dz = 1.5·2/3,
    //   V = 8.5·A₀ + ∫ (17 + 3k)² − (1 − π/4)(2(4.5 + 1.5k)² + 2(0.5k)²) dz.
    let (r, big, small) = (1.5, 6.0, 0.5);
    let b0 = block();
    let b = b0.fillet(&edges_named(&b0, &upright_edges()[..2]), big).unwrap();
    let b = b.fillet(&edges_named(&b, &upright_edges()[2..]), small).unwrap();
    let b = b.fillet(&edges_named(&b, &[(("block", Role::End), ("block", u_min()))]), r).unwrap();
    assert!(b.check().is_empty());
    let cut = 1.0 - PI / 4.0;
    let (i1, ik, ik2) = (r, r * PI / 4.0, r * 2.0 / 3.0);
    let a0 = 400.0 - cut * (2.0 * big * big + 2.0 * small * small);
    let square = 289.0 * i1 + 102.0 * ik + 9.0 * ik2;
    let tori = 2.0 * (20.25 * i1 + 13.5 * ik + 2.25 * ik2);
    let want = (10.0 - r) * a0 + square - cut * (tori + 2.0 * small * small * ik2);
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
    assert_eq!(faces_of(&b, |s| matches!(s, SurfaceKind::Spline)), 2);
    assert_eq!(faces_of(&b, |s| matches!(s, SurfaceKind::Torus)), 2);
}

#[test]
fn a_fillet_over_a_smaller_round_closes_each_corner_with_an_exact_patch() {
    // The R1.5 top fillet runs over R0.5 upright rounds. Each corner's ball sits r = 1.5 from the
    // top and both walls, at c = (±8.5, ±8.5, 8.5), and every horizontal section above z = 8.5 is
    // the section at z = 8.5 (a square of half-width 10 with R0.5 corners) scaled about the corner
    // axes by k = √(1 − ((z − 8.5)/1.5)²): a square of half-width 8.5 + 1.5k with corners ρk. So
    //   V = 8.5·A + ∫ ((17 + 3k)² − δk²) dz,  A = 400 − δ,  δ = 4(1 − π/4)·0.5²,
    // with ∫k dz = 1.5·π/4 and ∫k² dz = 1.5·2/3 over the top 1.5.
    // The bottom rim, filleted alike, is its mirror image: the same volume, and patches whose
    // poles lie the other way up.
    for face in ["end", "start"] {
        exact_corner_patches(face);
    }
}

fn exact_corner_patches(face: &str) {
    let (r, rho) = (1.5, 0.5);
    let b = rounded_then_filleted(rho, r, face);
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    let delta = 4.0 * (1.0 - PI / 4.0) * rho * rho;
    let (ik, ik2) = (r * PI / 4.0, r * 2.0 / 3.0);
    let want = (10.0 - r) * (400.0 - delta) + 289.0 * r + 102.0 * ik + 9.0 * ik2 - delta * ik2;
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
    // Six planes, four upright rounds cut back, four straight fillets and four corner patches.
    assert_eq!(b.faces().len(), 18);
    let patches = patches_of(&b);
    assert_eq!(patches.len(), 4);
    assert_tangent_all_round(&b, &patches);
}

/// The 20 × 20 × 10 block with a 10 × 10 pocket 4 deep in its top, the pocket's corners rounded
/// R0.5.
fn rounded_pocket() -> Solid {
    let pocket = on_block(&Profile::rectangle([-5.0, -5.0], [5.0, 5.0]).unwrap(), true, "pocket");
    let corners = [
        (("pocket", u_min()), ("pocket", v_min())),
        (("pocket", v_min()), ("pocket", u_max())),
        (("pocket", u_max()), ("pocket", v_max())),
        (("pocket", v_max()), ("pocket", u_min())),
    ];
    pocket.fillet(&edges_named(&pocket, &corners), 0.5).unwrap()
}

#[test]
fn a_pocket_floor_fillet_over_smaller_corner_rounds_closes_each_corner_with_a_patch() {
    // The R1.5 floor fillet is concave and the R0.5 corner rounds bend the same way, so the ball
    // sits in each corner, 1.5 from the floor and both walls. At height h above the floor, with
    // k = √(1 − ((1.5 − h)/1.5)²), the pocket's section is the one at h = 1.5 (a square of side 10
    // with R0.5 corners) scaled about the corner axes by k: side 7 + 3k, corners 0.5k. So the
    // pocket holds (4 − 1.5)(100 − δ) + ∫ (7 + 3k)² − δk² dh, δ = 4(1 − π/4)·0.5², with
    // ∫k dh = 1.5·π/4 and ∫k² dh = 1.5·2/3 over the bottom 1.5.
    let (r, rho) = (1.5, 0.5);
    let p = rounded_pocket();
    let b = p.fillet(&edges_named(&p, &[(("pocket", Role::Start), ("pocket", u_min()))]), r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    let delta = 4.0 * (1.0 - PI / 4.0) * rho * rho;
    let (ik, ik2) = (r * PI / 4.0, r * 2.0 / 3.0);
    let pocket = (4.0 - r) * (100.0 - delta) + 49.0 * r + 42.0 * ik + 9.0 * ik2 - delta * ik2;
    let want = 4000.0 - pocket;
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
    assert_eq!(patches_of(&b).len(), 4);
}

#[test]
fn a_pocket_rim_fillet_rolls_round_corner_rounds_bent_the_other_way() {
    // The R1.5 rim fillet is convex and the pocket's R0.5 corners concave: the ball rolls round
    // each in a torus. At depth t below the top, with k = √(1 − ((1.5 − t)/1.5)²), the pocket is
    // widened by 1.5(1 − k) all round: side 13 − 3k, corners 2 − 1.5k.
    let (r, rho) = (1.5, 0.5);
    let p = rounded_pocket();
    let b = p.fillet(&edges_named(&p, &[(("block", Role::End), ("pocket", u_min()))]), r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    let cut = 1.0 - PI / 4.0;
    let (ik, ik2) = (r * PI / 4.0, r * 2.0 / 3.0);
    let top = 169.0 * r - 78.0 * ik + 9.0 * ik2 - 4.0 * cut * (4.0 * r - 6.0 * ik + 2.25 * ik2);
    let pocket = (4.0 - r) * (100.0 - 4.0 * cut * rho * rho) + top;
    let want = 4000.0 - pocket;
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
    assert_eq!(patches_of(&b).len(), 0);
}

/// The share of an r × r square outside the quarter disc in its corner, times its distance from
/// that corner's far side: the R-fillet's section area and its centroid's distance from the edge.
fn fillet_section(r: f64) -> (f64, f64) {
    ((1.0 - PI / 4.0) * r * r, r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI))
}

/// An L: 20 × 20 less its +x +y quarter, 10 tall; edges e0..e5 from (-10, -10) anticlockwise, the
/// inner corner between e2 (y = 0) and e3 (x = 0).
fn l_block() -> Solid {
    let points = [[-10.0, -10.0], [10.0, -10.0], [10.0, 0.0], [0.0, 0.0], [0.0, 10.0], [-10.0, 10.0]];
    extrude(&Frame::WORLD, &Profile::polygon(&points).unwrap(), 10.0).unwrap().named("l")
}

#[test]
fn convex_fillets_meeting_at_an_inner_corner_swing_round_it() {
    // The L's top edges along its inner walls meet at the inner corner, whose vertical edge stays
    // sharp. Each fillet runs its edge's full 10; were they to run on past the corner into each
    // other's wall, at distances s₁, s₂ from the two walls the depth removed would be
    // h(max(s₁, s₂)), h(s) = r − √(r² − (r − s)²), which over the r × r square holds
    // 2∫ h(s)·s ds = r³(5/3 − π/2). The ball instead swings round the vertical edge, touching it:
    // in each slice s above its centres' plane it takes only the quarter disc of radius
    // r − √(r² − s²) about the edge, not the square, and keeps (1 − π/4)·r³(5/3 − π/2) of that.
    let r = 1.5;
    let l = l_block();
    let edges = edges_named(&l, &[(("l", Role::End), ("l", side(2))), (("l", Role::End), ("l", side(3)))]);
    let b = l.fillet(&edges, r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    let (area, _) = fillet_section(r);
    let want = 3000.0 - 2.0 * 10.0 * area - (PI / 4.0) * r.powi(3) * (5.0 / 3.0 - PI / 2.0);
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
    assert_eq!(faces_of(&b, |s| matches!(s, SurfaceKind::Cylinder { .. })), 2);
    assert_eq!(faces_of(&b, |s| matches!(s, SurfaceKind::Torus)), 1);
}

#[test]
fn a_mixed_corner_fillets_its_concave_edge_first_and_rolls_the_rim_round_it() {
    // All three edges of the L's inner corner at R1.5: the concave vertical edge's fillet adds
    // 10 × the section; the top rim then runs tangentially from one wall round that fillet's top
    // arc to the other, two runs of 8.5 and a quarter turn about the arc's axis, whose section's
    // centroid lies 1.5 plus its distance from the edge out from the axis.
    let r = 1.5;
    let l = l_block();
    let edges = edges_named(
        &l,
        &[(("l", Role::End), ("l", side(2))), (("l", Role::End), ("l", side(3))), (("l", side(2)), ("l", side(3)))],
    );
    let b = l.fillet(&edges, r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    let (area, centroid) = fillet_section(r);
    let want = 3000.0 + 10.0 * area - 2.0 * 8.5 * area - area * (r + centroid) * PI / 2.0;
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
}

#[test]
fn concave_fillets_meeting_at_a_boss_s_sharp_corner_cross_in_a_crease() {
    // Base fillets on two sides of a sharp 10 × 10 × 4 boss meet at its convex vertical edge,
    // which stays sharp. Each run is 10 long; past the corner the material goes only where both
    // add it, r³(5/3 − π/2) as at an inner corner.
    let r = 1.5;
    let boss = on_block(&Profile::rectangle([-5.0, -5.0], [5.0, 5.0]).unwrap(), false, "boss");
    let edges =
        edges_named(&boss, &[(("block", Role::End), ("boss", u_max())), (("block", Role::End), ("boss", v_max()))]);
    let b = boss.fillet(&edges, r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    let (area, _) = fillet_section(r);
    let want = 4400.0 + 2.0 * 10.0 * area + r.powi(3) * (5.0 / 3.0 - PI / 2.0);
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
    assert_eq!(faces_of(&b, |s| matches!(s, SurfaceKind::Torus)), 0);
}

#[test]
fn a_larger_fillet_into_a_smaller_one_at_a_corner_closes_with_a_patch() {
    // A block corner's top +x edge rounded ρ, then its top +y edge r > ρ: that edge runs on
    // tangentially round the first round's end arc and down the vertical edge, so the arc is a
    // smaller round under a larger fillet. Sliced parallel to the +y face at depth b ≤ r, with
    // k = √(1 − ((r − b)/r)²) and d = r(1 − k), the section is the corner with its walls moved in
    // by d and its round shrunk to ρk:
    //   V = 4000 − ∫ (30d − d² + (1 − π/4)(ρk)²) db − (20 − r)(1 − π/4)ρ²,
    // with ∫d = r²(1 − π/4), ∫d² = r³(5/3 − π/2) and ∫k² = 2r/3 over the top r.
    for (rho, r) in [(1.0, 1.5), (1.5, 2.0), (0.5, 1.5)] {
        let b0 = block();
        let b = b0.fillet(&edges_named(&b0, &[(("block", Role::End), ("block", u_max()))]), rho).unwrap();
        let b = b.fillet(&edges_named(&b, &[(("block", Role::End), ("block", v_max()))]), r).unwrap();
        assert!(b.check().is_empty());
        assert!(watertight(&b));
        let cut = 1.0 - PI / 4.0;
        let (id, id2, ik2) = (r * r * cut, r * r * r * (5.0 / 3.0 - PI / 2.0), 2.0 * r / 3.0);
        let want = 4000.0 - (30.0 * id - id2 + cut * rho * rho * ik2) - (20.0 - r) * cut * rho * rho;
        let got = b.volume();
        assert!((got - want).abs() < 1e-9 * want, "ρ {rho} r {r}: volume {got} for {want}");
        assert_eq!(faces_of(&b, |s| matches!(s, SurfaceKind::Spline)), 1);
    }
}

#[test]
fn a_fillet_no_larger_than_the_round_runs_round_it_as_before() {
    // With the rounds R1.5 the fillet's ball follows them: a torus at each corner, no patch.
    let b = rounded_then_filleted(1.5, 1.5, "end");
    assert!(b.check().is_empty());
    assert!(patches_of(&b).is_empty());
}

/// A half disc of radius 5, its flat back on x = 0 facing +x: a boss 4 tall on the block, or with
/// `cut` a pocket 4 deep in it. The corners where the back meets the arc are rounded R0.5.
fn d_on_block(cut: bool) -> Solid {
    let d = outline(vec![
        arc((0.0, 0.0), 5.0, PI / 2.0, PI), // s0: round_side
        line((0.0, -5.0), (0.0, 5.0)),      // s1: back_u_min
    ]);
    let s = on_block(&d, cut, "d");
    s.fillet(&edges_named(&s, &[(("d", side(0)), ("d", side(1)))]), 0.5).unwrap()
}

#[test]
fn a_fillet_over_a_smaller_round_beside_a_curved_wall_closes_with_an_exact_patch() {
    // The R1.5 top fillet of the R5 half disc runs round its R0.5 corner rounds. At each corner
    // the ball sits at c, 1.5 from the back and from the arc: (−1.5, ±√10). The torus along the
    // arc stops on the half plane through c and the arc's axis, and the corner's base runs from
    // the back's tangent round the round and along the arc to there. At depth t below the top,
    // with k = √(1 − ((1.5 − t)/1.5)²), the section is the back moved in by 1.5(1 − k), each
    // corner's base scaled by k about its corner axis, and the arc shrunk to 5 − 1.5(1 − k)
    // between the corners: arcs and lines, so by Green's theorem its area is α + βk + γk², fixed by
    // three values of k, and the top 1.5 holds αr + βr·π/4 + γ·2r/3.
    let (big, rho, r) = (5.0f64, 0.5f64, 1.5f64);
    let d = d_on_block(false);
    let b = d.fillet(&edges_named(&d, &[(("d", Role::End), ("d", side(0)))]), r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    type P = (f64, f64);
    let cross = |p: P, q: P| p.0 * q.1 - p.1 * q.0;
    let ang = |o: P, p: P| (p.1 - o.1).atan2(p.0 - o.0);
    // ½∮(x dy − y dx) along a circle about `o` from angle `a` to `b`.
    let arc = |o: P, rad: f64, a: f64, b: f64| {
        let (pa, pb) = ((o.0 + rad * a.cos(), o.1 + rad * a.sin()), (o.0 + rad * b.cos(), o.1 + rad * b.sin()));
        0.5 * (rad * rad * (b - a) + cross(o, (pb.0 - pa.0, pb.1 - pa.1)))
    };
    let yr = ((big - rho).powi(2) - rho * rho).sqrt();
    let round_centre = (-rho, yr);
    let c = (-r, ((big - r).powi(2) - r * r).sqrt());
    let a1 = (0.0, yr);
    let a2 = (-rho * big / (big - rho), yr * big / (big - rho));
    let p2 = (c.0 * big / (big - r), c.1 * big / (big - r));
    let flip = |p: P| (p.0, -p.1);
    let area = |k: f64| {
        let sc = |p: P| (c.0 + k * (p.0 - c.0), c.1 + k * (p.1 - c.1));
        let (u1, u2, up, ur, uo) = (sc(a1), sc(a2), sc(p2), sc(round_centre), sc((0.0, 0.0)));
        let mut sum = 0.0;
        if k > 0.0 {
            // The upper corner, round the round from the back and along the arc to its meridian;
            // the lower one, its mirror, walked back.
            let upper = arc(ur, rho * k, ang(ur, u1), ang(ur, u2)) + arc(uo, big * k, ang(uo, u2), ang(uo, up));
            let (l1, l2, lp, lr, lo) = (flip(u1), flip(u2), flip(up), flip(ur), flip(uo));
            let lower = arc(lo, big * k, ang(lo, lp), ang(lo, l2)) + arc(lr, rho * k, ang(lr, l2), ang(lr, l1));
            sum += upper + lower;
        }
        let shrunk = big - r * (1.0 - k);
        sum += arc((0.0, 0.0), shrunk, ang((0.0, 0.0), up), TAU + ang((0.0, 0.0), flip(up)));
        sum + 0.5 * cross(flip(u1), u1)
    };
    let (f0, fh, f1) = (area(0.0), area(0.5), area(1.0));
    let gamma = 2.0 * (f1 - 2.0 * fh + f0);
    let beta = f1 - f0 - gamma;
    let top = f0 * r + beta * r * PI / 4.0 + gamma * 2.0 * r / 3.0;
    let want = 4000.0 + f1 * (4.0 - r) + top;
    let got = b.volume();
    assert!((got - want).abs() < 1e-12 * want, "volume {got} for {want}");
    let patches = patches_of(&b);
    assert_eq!(patches.len(), 2);
    assert_tangent_all_round(&b, &patches);
    // The same D cut 4 deep into the block with its floor filleted is the boss's mirror image:
    // the pocket's hollow is the boss's material, so the block keeps 4000 less the boss's share.
    let dp = d_on_block(true);
    let pocket = dp.fillet(&edges_named(&dp, &[(("d", Role::Start), ("d", side(0)))]), r).unwrap();
    assert!(pocket.check().is_empty());
    assert!(watertight(&pocket));
    let got = pocket.volume();
    let want = 8000.0 - want;
    assert!((got - want).abs() < 1e-12 * want, "pocket volume {got} for {want}");
    assert_eq!(faces_of(&pocket, |s| matches!(s, SurfaceKind::Spline)), 2);
}

/// A lens on the block: the discs of radius 5 about (∓3, 0) intersected, 4 tall, its tips at
/// (0, ±4) rounded R0.5.
fn lens_boss() -> Solid {
    let half = 4f64.atan2(3.0);
    let lens = outline(vec![
        arc((-3.0, 0.0), 5.0, -half, 2.0 * half),    // s0: a_side
        arc((3.0, 0.0), 5.0, PI - half, 2.0 * half), // s1: b_side
    ]);
    let s = on_block(&lens, false, "l");
    s.fillet(&edges_named(&s, &[(("l", side(0)), ("l", side(1)))]), 0.5).unwrap()
}

#[test]
fn a_fillet_over_smaller_rounds_between_two_curved_walls_closes_with_exact_patches() {
    // The lens's two walls are the discs' cylinders, and the R1.5 top fillet runs round its R0.5
    // tips. At each tip the ball sits at c = (0, ±√(3.5² − 3²)), 3.5 from both discs' axes; each
    // torus stops on the half plane through c and its own axis, and the corner's base runs along
    // one wall, round the tip and along the other, three arcs. As for the D, the section at depth
    // t is the two walls shrunk to 5 − 1.5(1 − k) between the tips and each tip's base scaled by k
    // about its corner axis, its area α + βk + γk².
    let (big, rho, r, h) = (5.0f64, 0.5f64, 1.5f64, 3.0f64);
    let lens = lens_boss();
    let b = lens.fillet(&edges_named(&lens, &[(("l", Role::End), ("l", side(0)))]), r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    type P = (f64, f64);
    let cross = |p: P, q: P| p.0 * q.1 - p.1 * q.0;
    let ang = |o: P, p: P| (p.1 - o.1).atan2(p.0 - o.0);
    // ½∮(x dy − y dx) along a circle about `o`, anticlockwise from the angle of `p` to that of `q`.
    let arc = |o: P, rad: f64, p: P, q: P| {
        let a = ang(o, p);
        let mut b = ang(o, q);
        while b < a {
            b += TAU;
        }
        let (pa, pb) = ((o.0 + rad * a.cos(), o.1 + rad * a.sin()), (o.0 + rad * b.cos(), o.1 + rad * b.sin()));
        0.5 * (rad * rad * (b - a) + cross(o, (pb.0 - pa.0, pb.1 - pa.1)))
    };
    let toward = |o: P, p: P, rad: f64| {
        let d = (p.0 - o.0).hypot(p.1 - o.1);
        (o.0 + (p.0 - o.0) * rad / d, o.1 + (p.1 - o.1) * rad / d)
    };
    let (wa, wb) = ((-h, 0.0), (h, 0.0));
    let (yc, yr) = (((big - r).powi(2) - h * h).sqrt(), ((big - rho).powi(2) - h * h).sqrt());
    let area = |k: f64| {
        let shrunk = big - r * (1.0 - k);
        let mut sum = 0.0;
        // Each tip's base from wall `from` round the round to wall `to`, anticlockwise.
        let tip = |sign: f64, from: P, to: P| {
            let (c, rc) = ((0.0, sign * yc), (0.0, sign * yr));
            let sc = |p: P| (c.0 + k * (p.0 - c.0), c.1 + k * (p.1 - c.1));
            let (pf, tf) = (toward(from, c, big), toward(from, rc, big));
            let (tt, pt) = (toward(to, rc, big), toward(to, c, big));
            let base = arc(sc(from), big * k, sc(pf), sc(tf))
                + arc(sc(rc), rho * k, sc(tf), sc(tt))
                + arc(sc(to), big * k, sc(tt), sc(pt));
            (base, toward(from, c, big), toward(to, c, big))
        };
        let (top, top_a, top_b) = tip(1.0, wa, wb);
        let (bottom, bottom_b, bottom_a) = tip(-1.0, wb, wa);
        let ct = (0.0, yc);
        let cb = (0.0, -yc);
        let sct = |p: P| (ct.0 + k * (p.0 - ct.0), ct.1 + k * (p.1 - ct.1));
        let scb = |p: P| (cb.0 + k * (p.0 - cb.0), cb.1 + k * (p.1 - cb.1));
        if k > 0.0 {
            sum += top + bottom;
        }
        // The walls between the tips: A's from the bottom meridian up, B's from the top down.
        sum += arc(wa, shrunk, scb(bottom_a), sct(top_a));
        sum += arc(wb, shrunk, sct(top_b), scb(bottom_b));
        sum
    };
    let (f0, fh, f1) = (area(0.0), area(0.5), area(1.0));
    let gamma = 2.0 * (f1 - 2.0 * fh + f0);
    let beta = f1 - f0 - gamma;
    let top = f0 * r + beta * r * PI / 4.0 + gamma * 2.0 * r / 3.0;
    let want = 4000.0 + f1 * (4.0 - r) + top;
    let got = b.volume();
    assert!((got - want).abs() < 1e-12 * want, "volume {got} for {want}");
    let patches = patches_of(&b);
    assert_eq!(patches.len(), 2);
    assert_tangent_all_round(&b, &patches);
}

/// A 10 × 10 square with a bite out of its +x side, the disc of radius 5 about (9, 0), leaving an
/// arc from (5, 3) to (5, −3), its corners with the side rounded R0.5: a boss 4 tall on the block,
/// or with `cut`, a pocket 4 deep in it.
fn bitten(cut: bool) -> Solid {
    let half = 3f64.atan2(4.0);
    let square = outline(vec![
        line((-5.0, -5.0), (5.0, -5.0)),              // s0: box_v_min
        line((5.0, -5.0), (5.0, -3.0)),               // s1: box_u_max
        arc((9.0, 0.0), 5.0, half - PI, -2.0 * half), // s2: bite_side
        line((5.0, 3.0), (5.0, 5.0)),                 // s3: box_u_max
        line((5.0, 5.0), (-5.0, 5.0)),                // s4: box_v_max
        line((-5.0, 5.0), (-5.0, -5.0)),              // s5: box_u_min
    ]);
    let s = on_block(&square, cut, "b");
    // (b.box_u_max, b.bite_side): box_u_max names both short sides.
    s.fillet(&edges_named(&s, &[(("b", side(1)), ("b", side(2))), (("b", side(3)), ("b", side(2)))]), 0.5).unwrap()
}

#[test]
fn a_fillet_over_smaller_rounds_beside_a_concave_wall_closes_with_exact_patches() {
    // The rim of the bitten square, filleted R1.5, runs along its flat sides, round the R0.5
    // rounds and along the bite, a concave wall whose torus has the ball 6.5 from the bite's axis.
    // At each corner the ball sits at c = (3.5, ±√12), 1.5 inside the side and 6.5 from the axis;
    // the torus stops on the half plane through c and the axis, and the base runs from the side
    // round the round and along the bite to there. At depth t the section is the square moved in by
    // 1.5(1 − k), creased at its sharp corners, the bite grown to 5 + 1.5(1 − k) between the
    // corners, and each corner's base scaled by k about its axis: area α + βk + γk², as for the D.
    let (big, rho, r) = (5.0f64, 0.5f64, 1.5f64);
    // (face, b.box_u_min), (face, b.box_v_min), (face, b.box_v_max), (face, b.box_u_max) (both
    // sides of that name), (face, b.bite_side).
    let rim = |s: &Solid, face: Role| {
        let f = || ("b", face.clone());
        edges_named(
            s,
            &[
                (f(), ("b", side(5))),
                (f(), ("b", side(0))),
                (f(), ("b", side(4))),
                (f(), ("b", side(1))),
                (f(), ("b", side(3))),
                (f(), ("b", side(2))),
            ],
        )
    };
    let boss = bitten(false);
    let b = boss.fillet(&rim(&boss, Role::End), r).unwrap();
    assert!(b.check().is_empty());
    assert!(watertight(&b));
    type P = (f64, f64);
    let cross = |p: P, q: P| p.0 * q.1 - p.1 * q.0;
    // ½∮(x dy − y dx) along a circle about `o` from the angle of `p` to that of `q`.
    let arc = |o: P, rad: f64, p: P, q: P, anticlockwise: bool| {
        let a = (p.1 - o.1).atan2(p.0 - o.0);
        let mut b = (q.1 - o.1).atan2(q.0 - o.0);
        while anticlockwise && b < a {
            b += TAU;
        }
        while !anticlockwise && b > a {
            b -= TAU;
        }
        let (pa, pb) = ((o.0 + rad * a.cos(), o.1 + rad * a.sin()), (o.0 + rad * b.cos(), o.1 + rad * b.sin()));
        0.5 * (rad * rad * (b - a) + cross(o, (pb.0 - pa.0, pb.1 - pa.1)))
    };
    let line = |p: P, q: P| 0.5 * cross(p, q);
    let toward = |o: P, p: P, rad: f64| {
        let d = (p.0 - o.0).hypot(p.1 - o.1);
        (o.0 + (p.0 - o.0) * rad / d, o.1 + (p.1 - o.1) * rad / d)
    };
    let bite = (9.0, 0.0);
    // Each corner: its centre, its round's centre, the round's tangents on the side and the bite,
    // and the bite's point nearest the centre.
    let corner = |sign: f64| {
        let y = |d: f64| sign * ((big + d).powi(2) - (bite.0 - (5.0 - d)).powi(2)).sqrt();
        let (rc, c) = ((5.0 - rho, y(rho)), (5.0 - r, y(r)));
        (c, rc, (5.0, rc.1), toward(bite, rc, big), toward(bite, c, big))
    };
    let area = |k: f64| {
        let e = 5.0 - r * (1.0 - k);
        let (cl, rcl, a1l, a2l, pl) = corner(-1.0);
        let (cu, rcu, a1u, a2u, pu) = corner(1.0);
        let sc = |c: P, p: P| (c.0 + k * (p.0 - c.0), c.1 + k * (p.1 - c.1));
        let mut sum = line((-e, -e), (e, -e)) + line((e, -e), sc(cl, a1l));
        if k > 0.0 {
            sum += arc(sc(cl, rcl), rho * k, sc(cl, a1l), sc(cl, a2l), true)
                + arc(sc(cl, bite), big * k, sc(cl, a2l), sc(cl, pl), false);
        }
        sum += arc(bite, big + r * (1.0 - k), sc(cl, pl), sc(cu, pu), false);
        if k > 0.0 {
            sum += arc(sc(cu, bite), big * k, sc(cu, pu), sc(cu, a2u), false)
                + arc(sc(cu, rcu), rho * k, sc(cu, a2u), sc(cu, a1u), true);
        }
        sum + line(sc(cu, a1u), (e, e)) + line((e, e), (-e, e)) + line((-e, e), (-e, -e))
    };
    let (f0, fh, f1) = (area(0.0), area(0.5), area(1.0));
    let gamma = 2.0 * (f1 - 2.0 * fh + f0);
    let beta = f1 - f0 - gamma;
    let top = f0 * r + beta * r * PI / 4.0 + gamma * 2.0 * r / 3.0;
    let want = 4000.0 + f1 * (4.0 - r) + top;
    let got = b.volume();
    assert!((got - want).abs() < 1e-12 * want, "volume {got} for {want}");
    let patches = patches_of(&b);
    assert_eq!(patches.len(), 2);
    assert_tangent_all_round(&b, &patches);
    // Cut as a pocket with its floor filleted, it is the boss's mirror image, its wall now bulging
    // into the hollow.
    let cut = bitten(true);
    let pocket = cut.fillet(&rim(&cut, Role::Start), r).unwrap();
    assert!(pocket.check().is_empty());
    assert!(watertight(&pocket));
    let got = pocket.volume();
    let want = 8000.0 - want;
    assert!((got - want).abs() < 1e-12 * want, "pocket volume {got} for {want}");
    assert_eq!(faces_of(&pocket, |s| matches!(s, SurfaceKind::Spline)), 2);
}
