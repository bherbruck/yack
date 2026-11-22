// yack: chamfers and fillets on small swept solids against closed-form volume and area.
// Adapted from springrep@a03e8a1 tests/blends.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API; each solid is tagged with springrep's
// feature id (`named("box")`), and springrep face names "<id>.<face>" are yack (tag, Role):
// "end"/"start" are Role::End/Start, "round_n"/"bevel_n"/"corner" are Round(n)/Bevel(n)/Corner,
// and a profile side is Role::Side of its segment: springrep's rect sides v_min, u_max, v_max,
// u_min are Profile::rectangle's segments s0..s3, a circle's "side" is s0, polygon names are their
// segment's s{n}, and `rounded` labels its segments with springrep's names (Pen::label). A blend
// pair (face, face) is the edges between those faces. springrep's Profile::circle takes a
// diameter, yack's a radius; MeshOptions { linear: 0.2 } is chord 0.2, checked watertight.

//! Chamfers and fillets on small swept solids against closed-form volume and area, each result
//! checked valid and watertight, with the blend faces named by the operation.

use std::f64::consts::PI;

use yack::prelude::*;

const COARSE: f64 = 0.2;

fn at_z(z: f64) -> Frame {
    Frame::at([0.0, 0.0, z])
}

/// springrep's `block(id, w, d, h)`: the rectangle (0, 0)–(w, d) extruded h up z.
fn block_at(id: &str, w: f64, d: f64, h: f64) -> Solid {
    extrude(&at_z(0.0), &Profile::rectangle([0.0, 0.0], [w, d]).unwrap(), h).unwrap().named(id)
}

/// springrep's `disc(id, z, diameter, h)`.
fn disc(id: &str, z: f64, diameter: f64, h: f64) -> Solid {
    extrude(&at_z(z), &Profile::circle([0.0, 0.0], diameter / 2.0).unwrap(), h).unwrap().named(id)
}

/// springrep's face name `<id>.<face>` as the yack tag and role (see the header).
fn name(n: &str) -> (&str, Role) {
    let (tag, face) = n.split_once('.').unwrap();
    let side = |s: &str| Role::Side(s.into());
    let role = match face {
        "start" => Role::Start,
        "end" => Role::End,
        "corner" => Role::Corner,
        "v_min" | "side" => side("s0"),
        "u_max" => side("s1"),
        "v_max" => side("s2"),
        "u_min" => side("s3"),
        _ => {
            if let Some(k) = face.strip_prefix("round_") {
                Role::Round(k.parse().unwrap())
            } else if let Some(k) = face.strip_prefix("bevel_") {
                Role::Bevel(k.parse().unwrap())
            } else if let Some(k) = face.strip_prefix('e').filter(|k| k.parse::<u32>().is_ok()) {
                side(&format!("s{k}"))
            } else {
                side(face)
            }
        }
    };
    (tag, role)
}

/// The edges each springrep pair of named faces picks, in pair order.
fn pairs(b: &Solid, pairs: &[(&str, &str)]) -> Vec<EdgeId> {
    let mut edges = Vec::new();
    for &(a, c) in pairs {
        let ((ta, ra), (tc, rc)) = (name(a), name(c));
        let (fa, fc) = (b.faces_by(ta, ra), b.faces_by(tc, rc));
        assert!(!fa.is_empty() && !fc.is_empty(), "no face {a} or {c}");
        for &x in &fa {
            for &y in &fc {
                edges.extend(b.edges_between(x, y).unwrap());
            }
        }
    }
    edges
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

fn names(b: &Solid, n: &str) -> usize {
    let (tag, role) = name(n);
    b.faces_by(tag, role).len()
}

fn kinds(b: &Solid, keep: fn(&SurfaceKind) -> bool) -> usize {
    b.faces_where(|f| keep(&f.kind())).unwrap().len()
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

fn invalid(r: &Result<Solid>) -> bool {
    matches!(r, Err(e) if matches!(e.kind, ErrorKind::Invalid))
}

/// springrep's `fillet(body, pairs, r, id)`.
fn fillet(body: &Solid, p: &[(&str, &str)], r: f64, id: &str) -> Result<Solid> {
    body.fillet(&pairs(body, p), r).map(|b| b.named(id))
}

/// springrep's `chamfer(body, pairs, d, id)`.
fn chamfer(body: &Solid, p: &[(&str, &str)], d: f64, id: &str) -> Result<Solid> {
    body.chamfer(&pairs(body, p), d).map(|b| b.named(id))
}

#[test]
fn a_straight_convex_edge_takes_a_cylinder_fillet() {
    let b = fillet(&block_at("box", 40.0, 30.0, 20.0), &[("box.end", "box.v_min")], 5.0, "f").unwrap();
    let spandrel = 25.0 * (1.0 - PI / 4.0);
    let area = 5200.0 - 2.0 * 5.0 * 40.0 - 2.0 * spandrel + PI / 2.0 * 5.0 * 40.0;
    assert_solid(&b, 24000.0 - spandrel * 40.0, area);
    assert_eq!(b.faces().len(), 7);
    assert_eq!(kinds(&b, is_cylinder), 1);
    assert_eq!(names(&b, "f.round_0"), 1);
    assert_eq!(names(&b, "box.end"), 1);
}

#[test]
fn a_straight_concave_edge_takes_a_fillet_that_adds_material() {
    // Segments e0..e5 run from each corner to the next.
    let pts = [[0.0, 0.0], [40.0, 0.0], [40.0, 10.0], [10.0, 10.0], [10.0, 30.0], [0.0, 30.0]];
    let ell = extrude(&at_z(0.0), &Profile::polygon(&pts).unwrap(), 20.0).unwrap().named("l");
    let b = fillet(&ell, &[("l.e2", "l.e3")], 4.0, "f").unwrap();
    let spandrel = 16.0 * (1.0 - PI / 4.0);
    let area = 2.0 * (600.0 + spandrel) + 20.0 * (140.0 - 8.0 + 2.0 * PI);
    assert_solid(&b, 600.0 * 20.0 + spandrel * 20.0, area);
    assert_eq!(names(&b, "f.round_0"), 1);
    assert_eq!(names(&b, "l.e2"), 1);
}

#[test]
fn a_circular_edge_takes_a_cone_chamfer() {
    let b = chamfer(&disc("c", 0.0, 40.0, 20.0), &[("c.end", "c.side")], 2.0, "ch").unwrap();
    let volume = PI * 400.0 * 18.0 + PI * 2.0 / 3.0 * (400.0 + 360.0 + 324.0);
    let area = 400.0 * PI + 2.0 * PI * 20.0 * 18.0 + 324.0 * PI + PI * 38.0 * 2.0 * 2f64.sqrt();
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_cone), 1);
    assert_eq!(names(&b, "ch.bevel_0"), 1);
}

#[test]
fn a_concave_circular_edge_takes_a_torus_fillet() {
    let plate = extrude(&at_z(0.0), &Profile::rectangle([-30.0, -30.0], [30.0, 30.0]).unwrap(), 10.0)
        .unwrap()
        .named("plate");
    let body = plate.union(&disc("boss", 10.0, 20.0, 10.0)).unwrap();
    let b = fillet(&body, &[("plate.end", "boss.side")], 3.0, "f").unwrap();
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
    assert_eq!(names(&b, "f.round_0"), 1);
}

#[test]
fn three_fillets_at_a_corner_close_with_a_sphere() {
    let p = [("box.end", "box.u_max"), ("box.end", "box.v_max"), ("box.u_max", "box.v_max")];
    let b = fillet(&block_at("box", 20.0, 20.0, 20.0), &p, 4.0, "f").unwrap();
    let spandrel = 16.0 * (1.0 - PI / 4.0);
    let volume = 8000.0 - 3.0 * 16.0 * spandrel - (64.0 - PI * 64.0 / 6.0);
    let area = 2400.0 - 3.0 * 4.0 * 16.0 * 2.0 - 3.0 * 16.0 - 3.0 * spandrel
        + 3.0 * PI / 2.0 * 4.0 * 16.0
        + PI * 16.0 / 2.0;
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_sphere), 1);
    assert_eq!(names(&b, "f.corner"), 1);
    for i in 0..3 {
        assert_eq!(names(&b, &format!("f.round_{i}")), 1);
    }
}

#[test]
fn a_blend_wider_than_its_face_is_refused() {
    let r = chamfer(&block_at("box", 40.0, 30.0, 4.0), &[("box.end", "box.v_min")], 5.0, "ch");
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
    assert_eq!(names(&b, "f.round_0"), 1);
}

#[test]
fn a_fillet_ending_on_a_hub_it_meets_at_a_glancing_angle_dies_into_it() {
    // A hub of radius 25 and a bar out to x = 60, both 4 thick, the bar's side on the line y = −d
    // with d = √624: the side leaves the hub at x = 1, at 1/25 of a right angle from its tangent.
    // A fillet of 2 on the side's top edge ends on the hub, and leaves the hub's own edge sharp.
    let d = 624f64.sqrt();
    let bar = extrude(&at_z(0.0), &Profile::rectangle([0.0, -d], [60.0, d]).unwrap(), 4.0).unwrap().named("bar");
    let body = disc("hub", 0.0, 50.0, 4.0).union(&bar).unwrap();
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
    let body = extrude(&at_z(0.0), &profile, 4.0).unwrap().named("p");
    let x1 = (2501.0f64 - 48.0 * 48.0).sqrt() - 1.0;
    assert_dies_into(&body, 0.0, 2.0, 0.0, x1, |x| 50.0 - (2501.0 - (x + 1.0) * (x + 1.0)).sqrt());
}

#[test]
fn a_boss_fillet_wider_than_its_ledge_ends_on_the_ledge_side() {
    // A disc of radius 10 and height 4 with a boss of radius 8 and height 6 on it: the ledge is 2
    // wide and a fillet of 3 at the boss's foot runs over its rim, ending on the disc's side.
    let body = disc("base", 0.0, 20.0, 4.0).union(&disc("boss", 4.0, 16.0, 6.0)).unwrap();
    let b = fillet(&body, &[("base.end", "boss.side")], 3.0, "f").unwrap();
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
    assert_eq!(names(&b, "f.round_0"), 1);
    assert_eq!(names(&b, "base.end"), 0);
}

#[test]
fn a_concave_fillet_wider_than_its_ledge_ends_on_the_ledge_face() {
    // A ledge 10 deep (u), 4 high (v) and 30 long (z), and on it a wall 4 thick and 6 high standing
    // 2 back from the ledge's front (u = 0), running 20 of its length. A fillet of 3 where the
    // ledge's top meets the wall needs 3 of ledge and finds 2: it runs over the ledge's front edge
    // and ends on the front face, which is carried up to meet it.
    let ledge =
        extrude(&at_z(0.0), &Profile::rectangle([0.0, 0.0], [10.0, 4.0]).unwrap(), 30.0).unwrap().named("ledge");
    let wall = extrude(&at_z(0.0), &Profile::rectangle([2.0, 4.0], [6.0, 10.0]).unwrap(), 20.0).unwrap().named("wall");
    let body = ledge.union(&wall).unwrap();
    let b = fillet(&body, &[("ledge.v_max", "wall.u_min")], 3.0, "f").unwrap();
    // The arc's centre is at (−1, 7). The full spandrel is 9 − 9π/4; past u = 0 it would hold
    // ∫₀¹ (3 − √(9 − s²)) ds = 3 − √2 − 4.5·asin(1/3). The arc meets u = 0 at v = 7 − √8, having
    // turned acos(1/3) from the wall. The body's area is 1208.
    let kept = 6.0 - 9.0 * PI / 4.0 + 2f64.sqrt() + 4.5 * (1.0f64 / 3.0).asin();
    let rise = 3.0 - 8f64.sqrt();
    let area = 1208.0 + 2.0 * kept + 20.0 * rise - 40.0 - 60.0 + 60.0 * (1.0f64 / 3.0).acos();
    assert_solid(&b, 1680.0 + 20.0 * kept, area);
    assert_eq!(b.faces().len(), 11);
    assert_eq!(names(&b, "f.round_0"), 1);
    assert_eq!(names(&b, "ledge.u_min"), 1);
}

#[test]
fn a_pair_that_names_no_edge_is_invalid() {
    // yack: the pair picks no edge, and a fillet of no edges is refused.
    let r = fillet(&block_at("box", 4.0, 4.0, 4.0), &[("box.end", "box.start")], 1.0, "f");
    assert!(invalid(&r), "{:?}", r.err());
}

/// A `w × d` rectangle centred on the origin with corners of radius `r`: sides `s0..s3`
/// counter-clockwise from the bottom, each followed by its corner arc `c0..c3`.
fn rounded(w: f64, d: f64, r: f64) -> Profile {
    let (x, y) = (w / 2.0 - r, d / 2.0 - r);
    let (hw, hd) = (w / 2.0, d / 2.0);
    let outline = Loop::starting_at([-x, -hd])
        .line_to([x, -hd])
        .label("s0")
        .arc_ccw_to([hw, -y], [x, -y])
        .label("c0")
        .line_to([hw, y])
        .label("s1")
        .arc_ccw_to([x, hd], [x, y])
        .label("c1")
        .line_to([-x, hd])
        .label("s2")
        .arc_ccw_to([-hw, y], [-x, y])
        .label("c2")
        .line_to([-hw, -y])
        .label("s3")
        .arc_ccw_to([-x, -hd], [-x, -y])
        .label("c3")
        .close()
        .unwrap();
    Profile::new(outline, vec![]).unwrap()
}

#[test]
fn a_chamfer_follows_a_tangent_chain_round_a_pocket() {
    // A rounded-rectangle pocket through a plate: picking two opposite sides takes the corner arcs
    // and the other sides with them, so the whole rim is bevelled, the arcs as cones.
    let plate = extrude(&at_z(0.0), &Profile::rectangle([-30.0, -20.0], [30.0, 20.0]).unwrap(), 10.0);
    let slot = extrude(&at_z(0.0), &rounded(30.0, 12.0, 5.0), 10.0);
    let body = plate.unwrap().named("plate").difference(&slot.unwrap().named("slot")).unwrap();
    let d = 1.0;
    let b = chamfer(&body, &[("plate.end", "slot.s0"), ("plate.end", "slot.s2")], d, "ch").unwrap();
    // The band a hole's chamfer takes grows as the outline's outward offset: perimeter L + 2πw.
    let l = 2.0 * 20.0 + 2.0 * 2.0 + 2.0 * PI * 5.0;
    let hole = 30.0 * 12.0 - (4.0 - PI) * 25.0;
    let volume = (2400.0 - hole) * 10.0 - (l * d * d / 2.0 + PI * d * d * d / 3.0);
    let area = 2.0 * (2400.0 - hole) + 10.0 * (200.0 + l) - (l * d + PI * d * d) - l * d
        + 2f64.sqrt() * (l * d + PI * d * d);
    assert_solid(&b, volume, area);
    assert_eq!(kinds(&b, is_cone), 4);
    assert_eq!(names(&b, "ch.bevel_0") + names(&b, "ch.bevel_1"), 8);
}

#[test]
fn an_edge_two_pairs_pick_is_blended_once() {
    let p = [("box.end", "box.v_min"), ("box.v_min", "box.end")];
    let b = fillet(&block_at("box", 40.0, 30.0, 20.0), &p, 5.0, "f").unwrap();
    let spandrel = 25.0 * (1.0 - PI / 4.0);
    assert!((b.volume() - (24000.0 - spandrel * 40.0)).abs() <= 1e-9 * 24000.0);
    assert_eq!(names(&b, "f.round_0"), 1);
    assert_eq!(names(&b, "f.round_1"), 1);
    assert_eq!(kinds(&b, is_cylinder), 1);
}

#[test]
fn chamfers_meeting_at_corners_are_mitred() {
    let (w, d, h, c) = (20.0, 12.0, 8.0, 1.5);
    let p = [("box.end", "box.u_min"), ("box.end", "box.u_max"), ("box.end", "box.v_min"), ("box.end", "box.v_max")];
    let b = chamfer(&block_at("box", w, d, h), &p, c, "ch").unwrap();
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
    let rect = |id: &str, w: f64, d: f64, z: f64, h: f64| {
        extrude(&at_z(z), &Profile::rectangle([0.0, 0.0], [w, d]).unwrap(), h).unwrap().named(id)
    };
    let body = rect("bar", 14.0, 20.0, 0.0, 40.0).union(&rect("web", 14.0, 19.0, 40.0, 10.0)).unwrap();
    let b = chamfer(&body, &[("bar.u_max", "bar.v_max")], 4.0, "ch").unwrap();
    // The bevel takes the triangle x + y ≥ 30 off the bar's 40 height: 8 × 40 of volume, 4 of each
    // wall and 4√2 of bevel across, and 8 off the bottom. At z = 40 it takes 3.5 off the strip,
    // and leaves the web's underside over (14, 16), (14, 19), (11, 19) bare: 4.5. The body's area
    // is 3940.
    let area = 3940.0 - 320.0 + 160.0 * 2f64.sqrt() - 8.0 - 3.5 + 4.5;
    assert_solid(&b, 13860.0 - 320.0, area);
    assert_eq!(b.faces().len(), 10);
    assert_eq!(names(&b, "ch.bevel_0"), 1);
}

#[test]
fn a_chamfer_ending_on_a_slanted_face_is_trimmed_by_it() {
    // A trapezoid prism whose right end leans back; the chamfered edge between its top and its
    // end cap runs from the square left end into that slant.
    let (l, s, hh, t, d) = (30.0, 6.0, 10.0, 8.0, 1.0);
    let outline = Loop::starting_at([0.0, 0.0])
        .line_to([l, 0.0])
        .label("bottom")
        .line_to([l - s, hh])
        .label("slant")
        .line_to([0.0, hh])
        .label("top")
        .line_to([0.0, 0.0])
        .label("left")
        .close()
        .unwrap();
    let body = extrude(&at_z(0.0), &Profile::new(outline, vec![]).unwrap(), t).unwrap().named("p");
    let b = chamfer(&body, &[("p.top", "p.end")], d, "ch").unwrap();
    // Each line along x through the bevel's section ends on the slant x = l − s·y/hh.
    let tri = d * d / 2.0;
    let reach = |y: f64| l - s * y / hh;
    let volume = (l + l - s) / 2.0 * hh * t - tri * reach(hh - d / 3.0);
    let slant = (s * s + hh * hh).sqrt();
    let a0 = (l + l - s) * hh + t * (l + (l - s) + hh + slant);
    let cap_strip = d * l - s * (hh * hh - (hh - d) * (hh - d)) / (2.0 * hh);
    let area = a0 - d * (l - s) - cap_strip - tri * slant / hh - tri + d * 2f64.sqrt() * reach(hh - d / 2.0);
    assert_solid(&b, volume, area);
    assert_eq!(names(&b, "ch.bevel_0"), 1);
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
    let plate = extrude(&at_z(0.0), &Profile::rectangle([-30.0, -30.0], [30.0, 30.0]).unwrap(), 10.0);
    let bore = extrude(&at_z(0.0), &Profile::circle([0.0, 0.0], 20.0).unwrap(), 10.0);
    let key = extrude(&at_z(0.0), &Profile::rectangle([15.0, -3.0], [21.0, 3.0]).unwrap(), 10.0);
    let body = plate
        .unwrap()
        .named("plate")
        .difference(&bore.unwrap().named("bore"))
        .unwrap()
        .union(&key.unwrap().named("key"))
        .unwrap();
    let r = 1.0;
    let b = fillet(&body, &[("key.v_max", "bore.side"), ("key.v_min", "bore.side")], r, "f").unwrap();
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
    assert_eq!(names(&b, "f.round_0"), 1);
    assert_eq!(names(&b, "f.round_1"), 1);
    assert_eq!(kinds(&b, is_cylinder), 3);
}

#[test]
fn a_fillet_as_round_as_its_corners_closes_them_with_spheres() {
    // Every corner of the outline has the fillet's own radius, so each corner's round is an eighth
    // of a sphere; picking one side takes the whole tangent rim.
    let (w, d, h, r) = (20.0, 12.0, 6.0, 2.0);
    let body = extrude(&at_z(0.0), &rounded(w, d, r), h).unwrap().named("b");
    let b = fillet(&body, &[("b.end", "b.s1")], r, "f").unwrap();
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
    assert_eq!(names(&b, "f.round_0"), 8);
}

#[test]
fn a_chamfer_as_wide_as_its_corner_arcs_narrows_them_to_points() {
    // Each corner arc's bevel is a cone whose apex lies on the arc's axis, where the top face keeps
    // only a sharp rectangle.
    let (w, d, h, r) = (40.0, 20.0, 10.0, 5.0);
    let body = extrude(&at_z(0.0), &rounded(w, d, r), h).unwrap().named("rr");
    let b = chamfer(&body, &[("rr.end", "rr.s0")], r, "c").unwrap();
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
        let on = |p: [f64; 3]| cut.nearest_face(p).is_ok_and(|n| n.distance < 1e-9);
        assert!(on([10.0 - on_top - 0.01, 4.0, 6.0]));
        assert!(!on([10.0 - on_top + 0.01, 4.0, 6.0]));
        assert!(on([10.0, 4.0, 6.0 - on_side - 0.01]));
        assert!(!on([10.0, 4.0, 6.0 - on_side + 0.01]));
    }
    // A face that is not one of the edge's is refused.
    let bottom = b.face_at([5.0, 4.0, 0.0]).unwrap();
    assert!(b.chamfer_two(edge, bottom, 2.0, 1.0).is_err());
}
