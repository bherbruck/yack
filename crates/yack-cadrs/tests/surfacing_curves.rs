// yack: Lofts and sweeps through curves, projected curves and faces split along curves, on sh....
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/tests/surfacing_curves.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: crate paths cadrs_kernel:: -> yack_cadrs::.
//! Lofts and sweeps through curves, projected curves and faces split along curves, on shapes
//! whose areas follow from a formula (derived next to each): a failure is the kernel's, not a
//! stale fixture's. Every result must also pass the full validity check.

use yack_cadrs::native::kernel::NativeKernel;
use yack_cadrs::native::topo::CheckLevel;
use yack_cadrs::*;
use nalgebra::{Point2, Point3, Vector3};

/// A straight line in space as a path curve.
fn line(a: [f64; 3], b: [f64; 3]) -> PathCurve {
    PathCurve::Bezier3 {
        poles: vec![Point3::from(a), Point3::from(b)],
        weights: Vec::new(),
    }
}

fn area(k: &NativeKernel, body: BodyId) -> f64 {
    k.faces(body).unwrap().iter().map(|f| f.area).sum()
}

fn valid(k: &NativeKernel, body: BodyId) {
    let issues = k.get(body).unwrap().check_with(CheckLevel::Full);
    assert!(issues.is_empty(), "{issues:?}");
}

fn loft(sections: Vec<CurveSection>, guides: Vec<Vec<PathCurve>>) -> CurveLoftSpec {
    CurveLoftSpec {
        sections,
        guides,
        start: CurveLoftEnd::default(),
        end: CurveLoftEnd::default(),
        source: 1,
    }
}

#[test]
fn a_loft_of_two_lines_is_the_ruled_rectangle() {
    // Two parallel 10 mm lines 5 apart: the rectangle between them, 10 × 5 = 50.
    let mut k = NativeKernel::new();
    let spec = loft(
        vec![
            CurveSection::Wire(vec![line([0.0, 0.0, 0.0], [10.0, 0.0, 0.0])]),
            CurveSection::Wire(vec![line([0.0, 0.0, 5.0], [10.0, 0.0, 5.0])]),
        ],
        Vec::new(),
    );
    let b = k.loft_curves(&spec).unwrap().bodies[0];
    valid(&k, b);
    assert!((area(&k, b) - 50.0).abs() < 1e-6, "{}", area(&k, b));
}

#[test]
fn two_lines_meeting_in_a_point_and_a_guide_make_the_triangle() {
    // Two profiles from (10, 0, 0) and (0, 10, 0) to the origin, a guide joining their free
    // ends: the Coons patch of three straight lines in a plane is the flat triangle, 10 · 10 / 2.
    let mut k = NativeKernel::new();
    let spec = loft(
        vec![
            CurveSection::Wire(vec![line([10.0, 0.0, 0.0], [0.0, 0.0, 0.0])]),
            CurveSection::Wire(vec![line([0.0, 10.0, 0.0], [0.0, 0.0, 0.0])]),
        ],
        vec![vec![line([10.0, 0.0, 0.0], [0.0, 10.0, 0.0])]],
    );
    let b = k.loft_curves(&spec).unwrap().bodies[0];
    valid(&k, b);
    assert!((area(&k, b) - 50.0).abs() < 1e-6, "{}", area(&k, b));
    // Every point of it lies in the plane z = 0.
    let bb = k.bounding_box(b).unwrap();
    assert!(bb.min.z.abs() < 1e-9 && bb.max.z.abs() < 1e-9, "{bb:?}");
}

#[test]
fn one_guide_turns_a_profile_drawn_the_other_way() {
    // The triangle above with its second profile drawn from the origin out: one guide gives no
    // pair of guides to order, so where it meets the profiles (the first's start, the second's
    // end) says the second runs the other way. The same flat triangle, 50.
    let mut k = NativeKernel::new();
    let spec = loft(
        vec![
            CurveSection::Wire(vec![line([10.0, 0.0, 0.0], [0.0, 0.0, 0.0])]),
            CurveSection::Wire(vec![line([0.0, 0.0, 0.0], [0.0, 10.0, 0.0])]),
        ],
        vec![vec![line([10.0, 0.0, 0.0], [0.0, 10.0, 0.0])]],
    );
    let b = k.loft_curves(&spec).unwrap().bodies[0];
    valid(&k, b);
    assert!((area(&k, b) - 50.0).abs() < 1e-6, "{}", area(&k, b));
}


/// A 10 × 10 square sheet in the plane z = 0, corner at the origin (the Fill of its sides).
fn square(k: &mut NativeKernel) -> BodyId {
    let c = [
        Point2::new(0.0, 0.0),
        Point2::new(10.0, 0.0),
        Point2::new(10.0, 10.0),
        Point2::new(0.0, 10.0),
    ];
    let spec = FillSpec {
        curves: (0..4)
            .map(|i| FillCurve::Sketch {
                plane: Plane::top(),
                curve: Curve2::Line {
                    a: c[i],
                    b: c[(i + 1) % 4],
                    source: None,
                },
            })
            .collect(),
        source: 1,
    };
    k.fill(&spec).unwrap().bodies[0]
}


#[test]
fn a_face_split_along_a_line_across_it_keeps_its_area() {
    // The square cut along x = 4 from edge to edge: pieces of 40 and 60.
    let mut k = NativeKernel::new();
    let sq = square(&mut k);
    let r = k
        .split_faces(
            sq,
            &[FaceId(0)],
            &SplitTool::Curves(vec![line([4.0, 0.0, 0.0], [4.0, 10.0, 0.0])]),
        )
        .unwrap();
    let b = r.bodies[0];
    valid(&k, b);
    let mut areas: Vec<f64> = k.faces(b).unwrap().iter().map(|f| f.area).collect();
    areas.sort_by(f64::total_cmp);
    assert_eq!(areas.len(), 2, "{areas:?}");
    assert!(
        (areas[0] - 40.0).abs() < 1e-6 && (areas[1] - 60.0).abs() < 1e-6,
        "{areas:?}"
    );
}
