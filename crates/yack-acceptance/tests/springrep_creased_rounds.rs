// yack: rounds that end on a slanted cut cross in a crease; a concave round past a short edge is trimmed by a ramp.
// Adapted from springrep@a03e8a1 tests/creased_rounds.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); edges named by face-name pairs are
// selected with `face_at` + `edges_between`; springrep's centred `Profile::rect` is written as
// `Profile::rectangle(min, max)`; the ruled two-section loft `wedge` (two rectangles of equal depth) is the
// same prism built by `extrude` of its trapezoid section along y; `unify` has no §4.3 counterpart and is
// dropped (it only preceded checks and measures); the wall-face walk reads a plane's offset from its
// face's centroid and visits each face's vertices through its edges' ends.
//! Rounds that end on a face cut at a slant: where a round on a leaning edge meets its neighbour's
//! they cross in a crease, and a concave round that runs past a short edge is trimmed by the ramp
//! beyond it.

use yack::SurfaceKind;
use yack::prelude::*;

/// A wedge ruled between two rectangles 30 deep (y from −15 to 15), each from `x0` to `x1` on the
/// plane at its `z`.
fn wedge(sections: [(f64, f64, f64); 2]) -> Solid {
    let [(za, x0a, x1a), (zb, x0b, x1b)] = sections;
    // The section in the xz plane, pushed from y = 15 to y = −15.
    let xz = Frame::new([0.0, 15.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]).unwrap();
    let section = Profile::polygon(&[[x0a, za], [x1a, za], [x1b, zb], [x0b, zb]]).unwrap();
    extrude(&xz, &section, 30.0).unwrap()
}

/// Every edge between the face through `p` and the face through `q`.
fn between(b: &Solid, p: [f64; 3], q: [f64; 3]) -> Vec<EdgeId> {
    b.edges_between(b.face_at(p).unwrap(), b.face_at(q).unwrap()).unwrap()
}

/// The 20 × 20 × 10 block with its +x top edge cut back along x + z = 15, a 135° edge on the top.
fn leaning_block() -> Solid {
    let block = extrude(&Frame::WORLD, &Profile::rectangle([-10.0, -10.0], [10.0, 10.0]).unwrap(), 10.0).unwrap();
    let lean = wedge([(5.0, 10.0, 11.0), (11.0, 4.0, 11.0)]);
    block.difference(&lean).unwrap()
}

/// A triangular prism 10 tall on the xy plane whose first side (`wall`) runs from `points[0]` to
/// `points[1]`.
fn corner_cut(points: [[f64; 2]; 3]) -> Solid {
    extrude(&Frame::WORLD, &Profile::polygon(&points).unwrap(), 10.0).unwrap()
}

/// Points on the leaning block's top (block.end), its +y side (block.v_max) and the slant
/// (lean.side).
const END: [f64; 3] = [1.0, 0.0, 10.0];
const V_MAX: [f64; 3] = [0.0, 10.0, 5.0];
const LEAN: [f64; 3] = [7.5, 0.0, 7.5];
/// A point on diag.wall, the plane y = x + 2, clear of any round at its ends.
const DIAG_WALL: [f64; 3] = [2.0, 4.0, 5.0];

#[test]
fn rounds_on_a_square_and_a_leaning_edge_meet_in_a_crease() {
    let lb = leaning_block();
    // (block.end, block.v_max), (block.end, lean.side)
    let edges = [between(&lb, END, V_MAX), between(&lb, END, LEAN)].concat();
    let b = lb.fillet(&edges, 1.5).unwrap();
    assert!(b.check().is_empty());
}

#[test]
fn rounds_on_a_diagonal_and_a_leaning_edge_meet_in_a_crease() {
    // A vertical wall along y = x + 2 cuts the block's +y corner, meeting the leaning edge at 45°.
    let diag = corner_cut([[-15.0, -13.0], [15.0, 17.0], [-15.0, 17.0]]);
    let cut = leaning_block().difference(&diag).unwrap();
    // (block.end, diag.wall), (block.end, lean.side)
    let edges = [between(&cut, END, DIAG_WALL), between(&cut, END, LEAN)].concat();
    let b = cut.fillet(&edges, 1.5).unwrap();
    assert!(b.check().is_empty());
}

#[test]
fn a_leaning_edges_round_creases_into_diagonal_rounds_at_both_ends() {
    // Walls along y = ±(x + 2) cut both of the block's +x corners, so both ends of the
    // leaning edge's round crease into a diagonal round.
    let diag = corner_cut([[-15.0, -13.0], [15.0, 17.0], [-15.0, 17.0]]);
    let diag2 = corner_cut([[15.0, -17.0], [-15.0, 13.0], [-15.0, -17.0]]);
    let b = leaning_block().difference(&diag).unwrap().difference(&diag2).unwrap();
    // (diag.wall, diag2.wall): diag2.wall is the plane y = −x − 2.
    let b = b.fillet(&between(&b, DIAG_WALL, [0.0, -2.0, 5.0]), 3.0).unwrap();
    // (block.end, diag.wall), (block.end, lean.side)
    let edges = [between(&b, END, DIAG_WALL), between(&b, END, LEAN)].concat();
    let b = b.fillet(&edges, 1.5).unwrap();
    assert!(b.check().is_empty());
}

#[test]
fn a_concave_round_past_a_short_edge_is_trimmed_by_the_ramp_beyond_it() {
    // A strut 10 × 10 × 10 (x 0–10, y ±5) against a block (x 10–15, y ±15, z 0–15), its top cut
    // back above z = x + 1, which leaves a 1 mm edge of its top at the block. The R2 rounds where
    // its walls meet the block run up to the top; past that 1 mm edge the ramp crosses their strip.
    let rect = |cx: f64, w: f64, h: f64, d: f64| {
        let profile = Profile::rectangle([cx - w / 2.0, -h / 2.0], [cx + w / 2.0, h / 2.0]).unwrap();
        extrude(&Frame::WORLD, &profile, d).unwrap()
    };
    let r = 2.0;
    let block = rect(12.5, 5.0, 30.0, 15.0);
    let strut = rect(5.0, 10.0, 10.0, 10.0);
    let ramp = wedge([(1.0, -1.0, 0.0), (10.5, -1.0, 9.5)]);
    let b = block.union(&strut).unwrap().difference(&ramp).unwrap();
    // (strut.v_min, block.u_min), (strut.v_max, block.u_min)
    let block_u_min = [10.0, 10.0, 5.0];
    let edges = [between(&b, [5.0, -5.0, 2.0], block_u_min), between(&b, [5.0, 5.0, 2.0], block_u_min)].concat();
    let b = b.fillet(&edges, r).unwrap();
    assert!(b.check().is_empty());
    // Each round's section is the R2 spandrel: at distance u from the block it is
    // L(u) = r − √(r² − (r − u)²) wide, and stands 10 high for u ≤ 1 and 11 − u (the ramp) beyond.
    // With t = r − u and S = √(r² − t²): ∫S dt = (t·S + r²·asin(t/r))/2 and
    // ∫t·S dt = −(r² − t²)^1.5/3.
    let s = |t: f64| (r * r - t * t).max(0.0).sqrt();
    let i0 = |t: f64| 0.5 * (t * s(t) + r * r * (t / r).asin());
    let i1 = |t: f64| -(r * r - t * t).powf(1.5) / 3.0;
    let flat = 10.0 * (r - (i0(r) - i0(r - 1.0)));
    let (t0, t1) = (0.0, r - 1.0);
    let sloped = (11.0 - r) * r * (t1 - t0) + r * (t1 * t1 - t0 * t0) / 2.0
        - ((11.0 - r) * (i0(t1) - i0(t0)) + (i1(t1) - i1(t0)));
    // Block 5 × 30 × 15, the strut less the ramp's 9 × 9 / 2 triangle over its 10 width, and two
    // rounds.
    let want = 2250.0 + (1000.0 - 405.0) + 2.0 * (flat + sloped);
    let got = b.volume();
    assert!((got - want).abs() < 1e-9 * want, "volume {got} for {want}");
    // Nothing of the rounds is left past the ramp: no face lies on a wall's plane beyond it.
    let walls = b.faces_where(|f| {
        matches!(f.kind(), SurfaceKind::Plane { normal } if normal.y.abs() > 1.0 - 1e-12)
            && (f.centroid().y.abs() - 5.0).abs() < 1e-9
    });
    for f in walls {
        for e in b.edges_of(f).unwrap() {
            for at in b.edge(e).unwrap().ends() {
                assert!(at.z <= at.x + 1.0 + 1e-9, "a wall face reaches {at:?}");
            }
        }
    }
}
