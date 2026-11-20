// yack: chamfers and fillets on small swept solids against closed-form volume and area.
// Adapted from springrep@a03e8a1 tests/blends.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); edges named by face-name
// pairs ("box.end", "box.v_min") are picked with edge_at on the edge, circular edges by a
// CurveKind::Circle predicate, and a tangent rim springrep reaches from one or two picks is
// selected whole (edges_of the top face, or a predicate), since §4.3 does not say whether a
// blend follows tangent chains; springrep's Profile::circle takes a diameter, yack's a radius;
// MeshOptions { linear: 0.2 } is chord 0.2, checked watertight; springrep's per-face blend
// names ("f.round_0", "f.corner", "ch.bevel_0", ...) are the op's feature tag (`named("f")`),
// counted with faces_tagged, so a count is the number of faces carrying any of the op's names;
// checks on names of single input faces ("box.end") dropped (API GAP below).

//! Chamfers and fillets on small swept solids against closed-form volume and area, each result
//! checked valid and watertight, with the blend faces named by the operation.

use std::f64::consts::PI;

use yack::prelude::*;
use yack::{CurveKind, EdgeId, SurfaceKind};

const COARSE: f64 = 0.2;

/// springrep's `block(id, w, d, h)`: the rectangle (0, 0)–(w, d) extruded h up z.
fn block_at(w: f64, d: f64, h: f64) -> Solid {
    extrude(&Frame::WORLD, &Profile::rectangle([0.0, 0.0], [w, d]).unwrap(), h).unwrap()
}

/// springrep's `disc(id, z, diameter, h)`.
fn disc(z: f64, diameter: f64, h: f64) -> Solid {
    extrude(&Frame::at([0.0, 0.0, z]), &Profile::circle([0.0, 0.0], diameter / 2.0).unwrap(), h).unwrap()
}

fn assert_solid(b: &Solid, volume: f64, area: f64) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
    let v = b.volume();
    let a = b.area();
    assert!((v - volume).abs() <= 1e-9 * volume, "volume {v} vs {volume}");
    assert!((a - area).abs() <= 1e-9 * area, "area {a} vs {area}");
    assert!(b.mesh(COARSE).unwrap().is_watertight());
}

fn kinds(b: &Solid, keep: fn(&SurfaceKind) -> bool) -> usize {
    b.faces_where(|f| keep(&f.kind())).len()
}

fn is_cylinder(k: &SurfaceKind) -> bool {
    matches!(k, SurfaceKind::Cylinder { .. })
}
fn is_cone(k: &SurfaceKind) -> bool {
    matches!(k, SurfaceKind::Cone)
}
fn is_torus(k: &SurfaceKind) -> bool {
    matches!(k, SurfaceKind::Torus)
}
fn is_sphere(k: &SurfaceKind) -> bool {
    matches!(k, SurfaceKind::Sphere { .. })
}

/// Every piece of the circular edge of radius `r` at height `z` (a circle may be split at a seam).
fn circle_edges(b: &Solid, r: f64, z: f64) -> Vec<EdgeId> {
    let picked = b.edges_where(|e| {
        matches!(e.kind(), CurveKind::Circle { radius, .. } if (radius - r).abs() < 1e-9)
            && (e.midpoint().z - z).abs() < 1e-9
    });
    assert!(!picked.is_empty(), "no circle of radius {r} at z = {z}");
    picked
}

fn invalid(r: &Result<Solid>) -> bool {
    matches!(r, Err(e) if matches!(e.kind, ErrorKind::Invalid))
}

#[test]
fn a_straight_convex_edge_takes_a_cylinder_fillet() {
    // ("box.end", "box.v_min"): the top edge on y = 0.
    let body = block_at(40.0, 30.0, 20.0);
    let b = body.fillet(&[body.edge_at([20.0, 0.0, 20.0]).unwrap()], 5.0).unwrap().named("f");
    let spandrel = 25.0 * (1.0 - PI / 4.0);
    let area = 5200.0 - 2.0 * 5.0 * 40.0 - 2.0 * spandrel + PI / 2.0 * 5.0 * 40.0;
    assert_solid(&b, 24000.0 - spandrel * 40.0, area);
    assert_eq!(b.faces().len(), 7);
    assert_eq!(kinds(&b, is_cylinder), 1);
    // "f.round_0": the one round.
    assert_eq!(b.faces_tagged("f").len(), 1);
    // API GAP: names(&b, "box.end") == 1; a tag names every face of the op that made it, so one
    // face of a constructor (the block's top) has no name of its own.
}

#[test]
fn a_straight_concave_edge_takes_a_fillet_that_adds_material() {
    let pts = [[0.0, 0.0], [40.0, 0.0], [40.0, 10.0], [10.0, 10.0], [10.0, 30.0], [0.0, 30.0]];
    let ell = extrude(&Frame::WORLD, &Profile::polygon(&pts).unwrap(), 20.0).unwrap();
    // ("l.e2", "l.e3"): the inside corner at (10, 10).
    let b = ell.fillet(&[ell.edge_at([10.0, 10.0, 10.0]).unwrap()], 4.0).unwrap().named("f");
    let spandrel = 16.0 * (1.0 - PI / 4.0);
    let area = 2.0 * (600.0 + spandrel) + 20.0 * (140.0 - 8.0 + 2.0 * PI);
    assert_solid(&b, 600.0 * 20.0 + spandrel * 20.0, area);
    // "f.round_0".
    assert_eq!(b.faces_tagged("f").len(), 1);
    // API GAP: names(&b, "l.e2") == 1; profile segment labels are not face tags.
}

#[test]
fn a_circular_edge_takes_a_cone_chamfer() {
    let body = disc(0.0, 40.0, 20.0);
    // ("c.end", "c.side"): the top rim.
    let b = body.chamfer(&circle_edges(&body, 20.0, 20.0), 2.0).unwrap().named("ch");
    let volume = PI * 400.0 * 18.0 + PI * 2.0 / 3.0 * (400.0 + 360.0 + 324.0);
    let area = 400.0 * PI + 2.0 * PI * 20.0 * 18.0 + 324.0 * PI + PI * 38.0 * 2.0 * 2f64.sqrt();
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_cone), 1);
    // "ch.bevel_0".
    assert_eq!(b.faces_tagged("ch").len(), 1);
}

#[test]
fn a_concave_circular_edge_takes_a_torus_fillet() {
    let plate = extrude(&Frame::WORLD, &Profile::rectangle([-30.0, -30.0], [30.0, 30.0]).unwrap(), 10.0).unwrap();
    let body = plate.union(&disc(10.0, 20.0, 10.0)).unwrap();
    // ("plate.end", "boss.side"): the boss's foot.
    let b = body.fillet(&circle_edges(&body, 10.0, 10.0), 3.0).unwrap().named("f");
    // Pappus: the spandrel's centroid sits 3·(10 − 3π)/(12 − 3π) from the corner, in ρ and z.
    let spandrel = 9.0 * (1.0 - PI / 4.0);
    let offset = 3.0 * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let volume = 36000.0 + PI * 1000.0 + 2.0 * PI * (10.0 + offset) * spandrel;
    let area = (3600.0 - PI * 169.0)
        + 3600.0
        + 2400.0
        + 2.0 * PI * 70.0
        + PI * 100.0
        + 2.0 * PI * (13.0 - 6.0 / PI) * (1.5 * PI);
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_torus), 1);
    // "f.round_0".
    assert_eq!(b.faces_tagged("f").len(), 1);
}

#[test]
fn three_fillets_at_a_corner_close_with_a_sphere() {
    // ("box.end", "box.u_max"), ("box.end", "box.v_max"), ("box.u_max", "box.v_max").
    let body = block_at(20.0, 20.0, 20.0);
    let edges = [
        body.edge_at([20.0, 10.0, 20.0]).unwrap(),
        body.edge_at([10.0, 20.0, 20.0]).unwrap(),
        body.edge_at([20.0, 20.0, 10.0]).unwrap(),
    ];
    let b = body.fillet(&edges, 4.0).unwrap().named("f");
    let spandrel = 16.0 * (1.0 - PI / 4.0);
    let volume = 8000.0 - 3.0 * 16.0 * spandrel - (64.0 - PI * 64.0 / 6.0);
    let area = 2400.0 - 3.0 * 4.0 * 16.0 * 2.0 - 3.0 * 16.0 - 3.0 * spandrel
        + 3.0 * PI / 2.0 * 4.0 * 16.0
        + PI * 16.0 / 2.0;
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_sphere), 1);
    // "f.corner" once and "f.round_0", "f.round_1", "f.round_2" once each: four faces. yack's one
    // tag cannot tell the corner from the rounds; the sphere count above does.
    assert_eq!(b.faces_tagged("f").len(), 1 + 3);
}

#[test]
fn a_blend_wider_than_its_face_is_refused() {
    // ("box.end", "box.v_min") on a block 4 high.
    let body = block_at(40.0, 30.0, 4.0);
    let r = body.chamfer(&[body.edge_at([20.0, 0.0, 4.0]).unwrap()], 5.0);
    assert!(invalid(&r), "{:?}", r.err());
}

/// Fillets of `r` on the top edge of `body`'s side y = `side`, from x = `from` to 60, whose end at
/// `from` dies into a curved face: a point w in from the side at x lies past that face where
/// w > `w0(x)`, which reaches r at `x1`. The round takes the part of its spandrel short of w₀,
/// S(u) = r·u − ∫_{r−u}^{r} √(r² − v²) dv, and the whole S(r) = r²(1 − π/4) from x₁ to 60.
fn assert_dies_into(body: &Solid, side: f64, r: f64, from: f64, x1: f64, w0: impl Fn(f64) -> f64) {
    let edge = body.edge_at([30.0, side, 4.0]).unwrap();
    let b = body.fillet(&[edge], r).unwrap().named("f");
    let g = |v: f64| v / 2.0 * (r * r - v * v).max(0.0).sqrt() + r * r / 2.0 * (v / r).asin();
    let s = |u: f64| r * u - (g(r) - g(r - u));
    let n = 4000;
    let h = (x1 - from) / n as f64;
    let simpson: f64 = (0..=n)
        .map(|i| {
            let weight = if i == 0 || i == n { 1.0 } else { [2.0, 4.0][i % 2] };
            weight * s(w0(from + h * i as f64).clamp(0.0, r))
        })
        .sum::<f64>()
        * h
        / 3.0;
    let removed = simpson + (60.0 - x1) * s(r);
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
    let taken = body.volume() - b.volume();
    assert!((taken - removed).abs() <= 1e-6 * removed, "took {taken} vs {removed}");
    assert_eq!(b.faces().len(), body.faces().len() + 1);
    // "f.round_0".
    assert_eq!(b.faces_tagged("f").len(), 1);
}

#[test]
fn a_fillet_ending_on_a_hub_it_meets_at_a_glancing_angle_dies_into_it() {
    // A hub of radius 25 and a bar out to x = 60, both 4 thick, the bar's side on the line y = −d
    // with d = √624: the side leaves the hub at x = 1, at 1/25 of a right angle from its tangent.
    // A fillet of 2 on the side's top edge ends on the hub, and leaves the hub's own edge sharp.
    let d = 624f64.sqrt();
    let bar = extrude(&Frame::WORLD, &Profile::rectangle([0.0, -d], [60.0, d]).unwrap(), 4.0).unwrap();
    let body = disc(0.0, 50.0, 4.0).union(&bar).unwrap();
    let x1 = (625.0 - (d - 2.0) * (d - 2.0)).sqrt();
    assert_dies_into(&body, -d, 2.0, 1.0, x1, |x| d - (625.0 - x * x).sqrt());
}

#[test]
fn a_fillet_end_whose_rim_bows_out_behind_its_edge_runs_no_further() {
    // A plate 4 thick whose side y = 0 runs from x = 0 to 60. At x = 0 its rim turns onto an arc
    // about (−1, 50) through the origin, which bows out below y = 0 and comes back to it short of
    // x = −2, ending 5e-8 above it, within the tolerance of the line, in a reflex corner. The
    // fillet of 2 on the side's top edge meets the arc at a glancing angle and dies into it; the
    // rim past the arc never reaches into the fillet's strip.
    let (c, radius, lift) = ([-1.0f64, 50.0f64], 2501f64.sqrt(), 5e-8);
    let end = [c[0] - (radius * radius - (c[1] - lift).powi(2)).sqrt(), lift];
    let at = |p: [f64; 2]| (p[1] - c[1]).atan2(p[0] - c[0]);
    let line = |a: [f64; 2], b: [f64; 2]| Segment::Line { a: a.into(), b: b.into() };
    let corners = [[0.0, 0.0], [60.0, 0.0], [60.0, 20.0], [-10.0, 20.0], [-10.0, -0.5]];
    // "rim", then "side", "end", "back", "heel", then "toe".
    let mut segments = vec![Segment::Arc {
        center: c.into(),
        radius,
        start: rad(at(end)),
        sweep: rad(at(corners[0]) - at(end)),
    }];
    for i in 0..4 {
        segments.push(line(corners[i], corners[i + 1]));
    }
    segments.push(line(corners[4], end));
    let profile = Profile::new(Loop::new(segments).unwrap(), vec![]).unwrap();
    let body = extrude(&Frame::WORLD, &profile, 4.0).unwrap();
    let x1 = (2501.0f64 - 48.0 * 48.0).sqrt() - 1.0;
    assert_dies_into(&body, 0.0, 2.0, 0.0, x1, |x| 50.0 - (2501.0 - (x + 1.0) * (x + 1.0)).sqrt());
}

#[test]
fn a_boss_fillet_wider_than_its_ledge_ends_on_the_ledge_side() {
    // A disc of radius 10 and height 4 with a boss of radius 8 and height 6 on it: the ledge is 2
    // wide and a fillet of 3 at the boss's foot runs over its rim, ending on the disc's side.
    let body = disc(0.0, 20.0, 4.0).union(&disc(4.0, 16.0, 6.0)).unwrap();
    // ("base.end", "boss.side"): the boss's foot.
    let b = body.fillet(&circle_edges(&body, 8.0, 4.0), 3.0).unwrap().named("f");
    // In the section, s out from the boss and h up from the ledge, the arc's centre is (3, 3):
    // h = 3 − √(9 − w²), w = 3 − s, kept for s ≤ 2 (w ≥ 1). With J = ∫₁³ √(9 − w²) dw
    // = 9π/4 − √2 − 4.5·asin(1/3), the spandrel kept is 6 − J and its moment about the boss's
    // side ∫₁³ (3 − w)(3 − √(9 − w²)) dw = 6 − 3J + 16√2/3. The arc meets the disc's side at
    // h = 3 − √8 after turning acos(1/3), its moment about the boss 9·acos(1/3) − 6√2.
    let j = 9.0 * PI / 4.0 - 2f64.sqrt() - 4.5 * (1.0f64 / 3.0).asin();
    let (kept, moment) = (6.0 - j, 6.0 - 3.0 * j + 16.0 * 2f64.sqrt() / 3.0);
    let turn = (1.0f64 / 3.0).acos();
    let volume = 784.0 * PI + 2.0 * PI * (8.0 * kept + moment);
    let area = (376.0 - 36.0 - 48.0) * PI
        + 20.0 * PI * (3.0 - 8f64.sqrt())
        + 2.0 * PI * (33.0 * turn - 6.0 * 2f64.sqrt());
    assert_solid(&b, volume, area);
    assert_eq!(b.faces().len(), 5);
    assert_eq!(kinds(&b, is_torus), 1);
    // "f.round_0".
    assert_eq!(b.faces_tagged("f").len(), 1);
    // API GAP: names(&b, "base.end") == 0 (the ledge is used up); one face of a constructor has
    // no name of its own.
}

#[test]
fn a_concave_fillet_wider_than_its_ledge_ends_on_the_ledge_face() {
    // A ledge 10 deep (u), 4 high (v) and 30 long (z), and on it a wall 4 thick and 6 high standing
    // 2 back from the ledge's front (u = 0), running 20 of its length. A fillet of 3 where the
    // ledge's top meets the wall needs 3 of ledge and finds 2: it runs over the ledge's front edge
    // and ends on the front face, which is carried up to meet it.
    let ledge = extrude(&Frame::WORLD, &Profile::rectangle([0.0, 0.0], [10.0, 4.0]).unwrap(), 30.0).unwrap();
    let wall = extrude(&Frame::WORLD, &Profile::rectangle([2.0, 4.0], [6.0, 10.0]).unwrap(), 20.0).unwrap();
    let body = ledge.union(&wall).unwrap();
    // ("ledge.v_max", "wall.u_min"): the inside corner along z at u = 2, v = 4.
    let b = body.fillet(&[body.edge_at([2.0, 4.0, 10.0]).unwrap()], 3.0).unwrap().named("f");
    // The arc's centre is at (−1, 7). The full spandrel is 9 − 9π/4; past u = 0 it would hold
    // ∫₀¹ (3 − √(9 − s²)) ds = 3 − √2 − 4.5·asin(1/3). The arc meets u = 0 at v = 7 − √8, having
    // turned acos(1/3) from the wall. The body's area is 1208.
    let kept = 6.0 - 9.0 * PI / 4.0 + 2f64.sqrt() + 4.5 * (1.0f64 / 3.0).asin();
    let rise = 3.0 - 8f64.sqrt();
    let area = 1208.0 + 2.0 * kept + 20.0 * rise - 40.0 - 60.0 + 60.0 * (1.0f64 / 3.0).acos();
    assert_solid(&b, 1680.0 + 20.0 * kept, area);
    assert_eq!(b.faces().len(), 11);
    // "f.round_0".
    assert_eq!(b.faces_tagged("f").len(), 1);
    // API GAP: names(&b, "ledge.u_min") == 1; one face of a constructor has no name of its own.
}

#[test]
fn a_pair_that_names_no_edge_is_invalid() {
    // ("box.end", "box.start"): the top and the bottom share no edge.
    // yack: the pair is the edges between the two faces; either that walk or the fillet of the
    // edges it finds (none) is refused as invalid.
    let body = block_at(4.0, 4.0, 4.0);
    let (top, bottom) = (body.face_at([2.0, 2.0, 4.0]).unwrap(), body.face_at([2.0, 2.0, 0.0]).unwrap());
    let r = body.edges_between(top, bottom).and_then(|edges| body.fillet(&edges, 1.0));
    assert!(invalid(&r), "{:?}", r.err());
}

/// A `w × d` rectangle centred on the origin with corners of radius `r`: sides `s0..s3`
/// counter-clockwise from the bottom, each followed by its corner arc `c0..c3`.
fn rounded(w: f64, d: f64, r: f64) -> Profile {
    let (x, y) = (w / 2.0 - r, d / 2.0 - r);
    let (hw, hd) = (w / 2.0, d / 2.0);
    let arc = |cx: f64, cy: f64, start: f64| Segment::Arc {
        center: [cx, cy].into(),
        radius: r,
        start: rad(start),
        sweep: rad(PI / 2.0),
    };
    let line = |a: [f64; 2], b: [f64; 2]| Segment::Line { a: a.into(), b: b.into() };
    let segments = vec![
        line([-x, -hd], [x, -hd]),
        arc(x, -y, -PI / 2.0),
        line([hw, -y], [hw, y]),
        arc(x, y, 0.0),
        line([x, hd], [-x, hd]),
        arc(-x, y, PI / 2.0),
        line([-hw, y], [-hw, -y]),
        arc(-x, -y, PI),
    ];
    Profile::new(Loop::new(segments).unwrap(), vec![]).unwrap()
}

#[test]
fn a_chamfer_follows_a_tangent_chain_round_a_pocket() {
    // A rounded-rectangle pocket through a plate: picking two opposite sides takes the corner arcs
    // and the other sides with them, so the whole rim is bevelled, the arcs as cones.
    let plate = extrude(&Frame::WORLD, &Profile::rectangle([-30.0, -20.0], [30.0, 20.0]).unwrap(), 10.0);
    let slot = extrude(&Frame::WORLD, &rounded(30.0, 12.0, 5.0), 10.0);
    let body = plate.unwrap().difference(&slot.unwrap()).unwrap();
    let d = 1.0;
    // ("plate.end", "slot.s0") and ("plate.end", "slot.s2"), and the tangent chain they reach:
    // yack: every edge of the pocket's top rim, picked whole.
    let rim = body.edges_where(|e| {
        let m = e.midpoint();
        (m.z - 10.0).abs() < 1e-9 && m.x.abs() <= 15.0 + 1e-9 && m.y.abs() <= 6.0 + 1e-9
    });
    let b = body.chamfer(&rim, d).unwrap().named("ch");
    // The band a hole's chamfer takes grows as the outline's outward offset: perimeter L + 2πw.
    let l = 2.0 * 20.0 + 2.0 * 2.0 + 2.0 * PI * 5.0;
    let hole = 30.0 * 12.0 - (4.0 - PI) * 25.0;
    let volume = (2400.0 - hole) * 10.0 - (l * d * d / 2.0 + PI * d * d * d / 3.0);
    let area = 2.0 * (2400.0 - hole) + 10.0 * (200.0 + l) - (l * d + PI * d * d) - l * d
        + 2f64.sqrt() * (l * d + PI * d * d);
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_cone), 4);
    // names(&b, "ch.bevel_0") + names(&b, "ch.bevel_1") == 8.
    assert_eq!(b.faces_tagged("ch").len(), 8);
}

#[test]
fn an_edge_two_pairs_pick_is_blended_once() {
    // ("box.end", "box.v_min") and ("box.v_min", "box.end"): the same edge twice.
    let body = block_at(40.0, 30.0, 20.0);
    let e = body.edge_at([20.0, 0.0, 20.0]).unwrap();
    let b = body.fillet(&[e, e], 5.0).unwrap().named("f");
    let spandrel = 25.0 * (1.0 - PI / 4.0);
    assert!((b.volume() - (24000.0 - spandrel * 40.0)).abs() <= 1e-9 * 24000.0);
    // "f.round_0" once and "f.round_1" once, on the one cylinder below: one face carries the tag.
    assert_eq!(b.faces_tagged("f").len(), 1);
    assert_eq!(kinds(&b, is_cylinder), 1);
}

#[test]
fn chamfers_meeting_at_corners_are_mitred() {
    let (w, d, h, c) = (20.0, 12.0, 8.0, 1.5);
    // ("box.end", "box.u_min" | "box.u_max" | "box.v_min" | "box.v_max"): the four top edges.
    let body = block_at(w, d, h);
    let edges = [
        body.edge_at([0.0, d / 2.0, h]).unwrap(),
        body.edge_at([w, d / 2.0, h]).unwrap(),
        body.edge_at([w / 2.0, 0.0, h]).unwrap(),
        body.edge_at([w / 2.0, d, h]).unwrap(),
    ];
    let b = body.chamfer(&edges, c).unwrap();
    // A hipped top: at depth z the section is the rectangle inset by c − z on every side.
    let volume = w * d * h - (c * c * (w + d) - 4.0 * c * c * c / 3.0);
    let area = (w - 2.0 * c) * (d - 2.0 * c) + w * d + 2.0 * (w + d) * (h - c) + 2.0 * 2f64.sqrt() * c * (w - c + d - c);
    assert_solid(&b, volume, area);
    assert_eq!(b.faces().len(), 10);
}

#[test]
fn a_chamfer_ending_under_an_overhang_stops_at_its_end_face() {
    // A bar 14 × 20 × 40 with a web 14 × 19 × 10 on top, flush with it at x = 0, x = 14 and
    // y = 0, which leaves a strip of the bar's top 1 wide along y = 20. A chamfer of 4 on the bar's
    // edge at x = 14, y = 20 runs past that strip's 1 mm edge, under the web, and into the concave
    // corner where the strip meets the web's side; it ends square on the strip's plane, z = 40.
    let rect = |w: f64, d: f64, z: f64, h: f64| {
        extrude(&Frame::at([0.0, 0.0, z]), &Profile::rectangle([0.0, 0.0], [w, d]).unwrap(), h).unwrap()
    };
    let body = rect(14.0, 20.0, 0.0, 40.0).union(&rect(14.0, 19.0, 40.0, 10.0)).unwrap();
    // ("bar.u_max", "bar.v_max"): the bar's vertical edge at x = 14, y = 20.
    let b = body.chamfer(&[body.edge_at([14.0, 20.0, 20.0]).unwrap()], 4.0).unwrap().named("ch");
    // The bevel takes the triangle x + y ≥ 30 off the bar's 40 height: 8 × 40 of volume, 4 of each
    // wall and 4√2 of bevel across, and 8 off the bottom. At z = 40 it takes 3.5 off the strip,
    // and leaves the web's underside over (14, 16), (14, 19), (11, 19) bare: 4.5. The body's area
    // is 3940.
    let area = 3940.0 - 320.0 + 160.0 * 2f64.sqrt() - 8.0 - 3.5 + 4.5;
    assert_solid(&b, 13860.0 - 320.0, area);
    assert_eq!(b.faces().len(), 10);
    // "ch.bevel_0".
    assert_eq!(b.faces_tagged("ch").len(), 1);
}

#[test]
fn a_chamfer_ending_on_a_slanted_face_is_trimmed_by_it() {
    // A trapezoid prism whose right end leans back; the chamfered edge between its top and its
    // end cap runs from the square left end into that slant.
    let (l, s, hh, t, d) = (30.0, 6.0, 10.0, 8.0, 1.0);
    let outline = Profile::polygon(&[[0.0, 0.0], [l, 0.0], [l - s, hh], [0.0, hh]]).unwrap();
    let body = extrude(&Frame::WORLD, &outline, t).unwrap();
    // ("p.top", "p.end"): the edge along x where the top y = hh meets the end cap z = t.
    let b = body.chamfer(&[body.edge_at([(l - s) / 2.0, hh, t]).unwrap()], d).unwrap().named("ch");
    // Each line along x through the bevel's section ends on the slant x = l − s·y/hh.
    let tri = d * d / 2.0;
    let reach = |y: f64| l - s * y / hh;
    let volume = (l + l - s) / 2.0 * hh * t - tri * reach(hh - d / 3.0);
    let slant = (s * s + hh * hh).sqrt();
    let a0 = (l + l - s) * hh + t * (l + (l - s) + hh + slant);
    let cap_strip = d * l - s * (hh * hh - (hh - d) * (hh - d)) / (2.0 * hh);
    let area = a0 - d * (l - s) - cap_strip - tri * slant / hh - tri + d * 2f64.sqrt() * reach(hh - d / 2.0);
    assert_solid(&b, volume, area);
    // "ch.bevel_0".
    assert_eq!(b.faces_tagged("ch").len(), 1);
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

/// The area `½∮(x dy − y dx)` contributes along the shorter arc about `o` from `a` to `b`, and
/// that arc's length.
fn arc_green(o: [f64; 2], r: f64, a: [f64; 2], b: [f64; 2]) -> (f64, f64) {
    let (ao, bo) = (sub(a, o), sub(b, o));
    let (sa, sb) = (ao[1].atan2(ao[0]), bo[1].atan2(bo[0]));
    let sw = (sb - sa + PI).rem_euclid(2.0 * PI) - PI;
    let chord = o[0] * (sb.sin() - sa.sin()) - o[1] * (sb.cos() - sa.cos());
    (0.5 * (r * r * sw + r * chord), r * sw.abs())
}

#[test]
fn a_straight_edge_between_a_plane_and_a_bore_takes_a_fillet() {
    // A key standing in a bore: the root of each side is filleted against the bore's wall.
    let plate = extrude(&Frame::WORLD, &Profile::rectangle([-30.0, -30.0], [30.0, 30.0]).unwrap(), 10.0);
    let bore = extrude(&Frame::WORLD, &Profile::circle([0.0, 0.0], 20.0).unwrap(), 10.0);
    let key = extrude(&Frame::WORLD, &Profile::rectangle([15.0, -3.0], [21.0, 3.0]).unwrap(), 10.0);
    let body = plate.unwrap().difference(&bore.unwrap()).unwrap().union(&key.unwrap()).unwrap();
    let r = 1.0;
    // ("key.v_max", "bore.side") and ("key.v_min", "bore.side"): the key's sides y = ±3 meet
    // the bore at x = √391.
    let edges = [
        body.edge_at([391f64.sqrt(), 3.0, 5.0]).unwrap(),
        body.edge_at([391f64.sqrt(), -3.0, 5.0]).unwrap(),
    ];
    let b = body.fillet(&edges, r).unwrap().named("f");
    // The round's centre is 1 off the side y = 3 and 1 inside the bore of radius 20.
    let p = [391f64.sqrt(), 3.0];
    let c = [345f64.sqrt(), 4.0];
    let (t0, t1) = ([c[0], 3.0], [c[0] * (20.0 / 19.0), c[1] * (20.0 / 19.0)]);
    let (g_round, round_len) = arc_green(c, r, t0, t1);
    let (g_bore, bore_len) = arc_green([0.0, 0.0], 20.0, t1, p);
    let spandrel = (0.5 * (p[0] * t0[1] - p[1] * t0[0]) + g_round + g_bore).abs();
    let in_bore = |y: f64| y / 2.0 * (400.0 - y * y).sqrt() + 200.0 * (y / 20.0).asin();
    let key_in_bore = in_bore(3.0) - in_bore(-3.0) - 15.0 * 6.0;
    let volume = 10.0 * (3600.0 - 400.0 * PI + key_in_bore + 2.0 * spandrel);
    let tp = sub(t0, p);
    let area = body.area() + 4.0 * spandrel + 20.0 * (round_len - (tp[0] * tp[0] + tp[1] * tp[1]).sqrt() - bore_len);
    assert_solid(&b, volume, area);
    // "f.round_0" and "f.round_1": two rounds.
    assert_eq!(b.faces_tagged("f").len(), 2);
    assert_eq!(kinds(&b, is_cylinder), 3);
}

#[test]
fn a_fillet_as_round_as_its_corners_closes_them_with_spheres() {
    // Every corner of the outline has the fillet's own radius, so each corner's round is an eighth
    // of a sphere; picking one side takes the whole tangent rim.
    let (w, d, h, r) = (20.0, 12.0, 6.0, 2.0);
    let body = extrude(&Frame::WORLD, &rounded(w, d, r), h).unwrap();
    // ("b.end", "b.s1") and the tangent rim it reaches. yack: every edge of the top face.
    let rim = body.edges_of(body.face_at([0.0, 0.0, h]).unwrap()).unwrap();
    let b = body.fillet(&rim, r).unwrap().named("f");
    let straight = 2.0 * (w - 2.0 * r) + 2.0 * (d - 2.0 * r);
    let spandrel = r * r * (1.0 - PI / 4.0);
    // Pappus: the spandrel's centroid sits r·(10 − 3π)/(12 − 3π) in from the rim.
    let off = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let top = w * d - (4.0 - PI) * r * r;
    let volume = top * h - spandrel * straight - 2.0 * PI * (r - off) * spandrel;
    let perimeter = straight + 2.0 * PI * r;
    let area = 2.0 * top + perimeter * h - (straight * r + PI * r * r) - perimeter * r
        + PI / 2.0 * r * straight
        + 2.0 * PI * r * r;
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_sphere), 4);
    // "f.round_0" on eight faces.
    assert_eq!(b.faces_tagged("f").len(), 8);
}

#[test]
fn a_chamfer_as_wide_as_its_corner_arcs_narrows_them_to_points() {
    // Each corner arc's bevel is a cone whose apex lies on the arc's axis, where the top face keeps
    // only a sharp rectangle.
    let (w, d, h, r) = (40.0, 20.0, 10.0, 5.0);
    let body = extrude(&Frame::WORLD, &rounded(w, d, r), h).unwrap();
    // ("rr.end", "rr.s0") and the tangent rim it reaches. yack: every edge of the top face.
    let rim = body.edges_of(body.face_at([0.0, 0.0, h]).unwrap()).unwrap();
    let b = body.chamfer(&rim, r).unwrap();
    let straight = 2.0 * (w - 2.0 * r) + 2.0 * (d - 2.0 * r);
    let section = w * d - (4.0 - PI) * r * r;
    // Pappus: the bevel's triangle has its centroid r/3 in from the rim.
    let volume = section * h - r * r / 2.0 * (straight + 2.0 * PI * (r - r / 3.0));
    let area = section
        + (w - 2.0 * r) * (d - 2.0 * r)
        + (straight + 2.0 * PI * r) * (h - r)
        + 2f64.sqrt() * r * (straight + PI * r);
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_cone), 4);
}

#[test]
fn a_chamfer_of_two_distances_sets_the_first_back_on_the_face_named() {
    // A block 10 by 8 by 6; its edge along y where the top meets the side x = 10. Set back 2 on
    // the top and 1 on the side, the top keeps 8 of its 10 across and the side 5 of its 6.
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0]).unwrap();
    let edge = b.edge_at([10.0, 4.0, 6.0]).unwrap();
    let top = b.face_at([5.0, 4.0, 6.0]).unwrap();
    let side = b.face_at([10.0, 4.0, 3.0]).unwrap();
    for (first, on_top, on_side) in [(top, 2.0, 1.0), (side, 1.0, 2.0)] {
        let cut = b.chamfer_two(edge, first, 2.0, 1.0).unwrap();
        assert!(cut.check().is_empty());
        let volume = 480.0 - 0.5 * 2.0 * 1.0 * 8.0;
        assert!((cut.volume() - volume).abs() < 1e-9);
        // Where each face now ends: a point just inside the kept part is on it, one just past
        // the chamfer's edge is not.
        // yack: "on a face" is face_at finding one within tol::PICK.
        let on = |p: [f64; 3]| cut.face_at(p).is_ok();
        assert!(on([10.0 - on_top - 0.01, 4.0, 6.0]));
        assert!(!on([10.0 - on_top + 0.01, 4.0, 6.0]));
        assert!(on([10.0, 4.0, 6.0 - on_side - 0.01]));
        assert!(!on([10.0, 4.0, 6.0 - on_side + 0.01]));
    }
    // A face that is not one of the edge's is refused.
    let bottom = b.face_at([5.0, 4.0, 0.0]).unwrap();
    assert!(b.chamfer_two(edge, bottom, 2.0, 1.0).is_err());
}
