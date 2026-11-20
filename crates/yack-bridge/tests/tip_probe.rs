// yack: probe for bench tangent/paraboloid-tip-a-ten-thousandth-high (WRONG: empty): does cadrs's tip,
// carried back by to_springrep, pass the classify self-check? (printed to the scratchpad)
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_cadrs::{BoolOp, DefaultKernel, Kernel};
use yack_springrep::{
    boolean,
    math::{Frame, Vec2, Vec3},
    measure,
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep::revolve,
};

#[test]
fn paraboloid_tip_comes_back() {
    let segs = vec![
        // The parabola x² = 8z as its quadratic Bézier (weight 1: the conic's apex is its middle pole).
        NamedSegment::new(
            Segment::Spline {
                curve: yack_springrep::geom::BSpline::new(
                    2,
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), Vec3::new(8.0, 8.0, 0.0)],
                )
                .unwrap(),
                reversed: false,
            },
            "s0",
        ),
        NamedSegment::new(Segment::Line { a: Vec2::new(8.0, 8.0), b: Vec2::new(0.0, 8.0) }, "s1"),
        NamedSegment::new(Segment::Line { a: Vec2::new(0.0, 8.0), b: Vec2::new(0.0, 0.0) }, "s2"),
    ];
    let meridian = Frame::new(Vec3::ZERO, -Vec3::Y, Vec3::X).unwrap();
    let par = revolve(&Profile { loops: vec![ProfileLoop { segments: segs }] }, &meridian, Vec3::ZERO, Vec3::Z, std::f64::consts::TAU, "p").unwrap();
    let blk = yack_springrep::primitive::block(&Frame::new(Vec3::new(-9.0, -9.0, -1.0), Vec3::Z, Vec3::X).unwrap(), Vec3::new(18.0, 18.0, 1.0001), "b").unwrap();
    let mut lines = vec![];
    let sr = boolean::intersection(&par, &blk);
    lines.push(format!("springrep intersection: {}", match &sr { Ok(b) => format!("volume {} solids {}", measure::volume(b), b.solid_ids().count()), Err(e) => e.to_string() }));
    let mut k = DefaultKernel::new();
    let (a, b) = (k.insert(yack_bridge::to_cadrs(&par).unwrap().0), k.insert(yack_bridge::to_cadrs(&blk).unwrap().0));
    let cut = k.boolean(BoolOp::Intersect, a, &[b]).unwrap().first().unwrap();
    lines.push(format!("cadrs intersection: volume {}", k.mass_properties(cut).unwrap().volume));
    let body = (**k.get(cut).unwrap()).clone();
    lines.push(format!("to_springrep: {}", match yack_bridge::to_springrep(&body, &[]) { Ok(b) => format!("ok volume {}", measure::volume(&b)), Err(e) => e.to_string() }));
    std::fs::write("/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/tip-probe.txt", lines.join("\n")).unwrap();
}

/// The self-check's acceptance cases: bodies cadrs builds right that must cross into springrep
/// (the paraboloid tip, 1e-4 high; the lemon, a spindle torus with poles; the rational spheroid).
#[test]
fn small_and_pointed_cadrs_bodies_cross_or_are_held() {
    use yack_cadrs::facade::profile::{Place, Seg, revolve as crevolve};
    let world = Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let mut k = DefaultKernel::new();
    // The tip, as above, cut by cadrs.
    let segs = vec![
        NamedSegment::new(
            Segment::Spline {
                curve: yack_springrep::geom::BSpline::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), Vec3::new(8.0, 8.0, 0.0)]).unwrap(),
                reversed: false,
            },
            "s0",
        ),
        NamedSegment::new(Segment::Line { a: Vec2::new(8.0, 8.0), b: Vec2::new(0.0, 8.0) }, "s1"),
        NamedSegment::new(Segment::Line { a: Vec2::new(0.0, 8.0), b: Vec2::new(0.0, 0.0) }, "s2"),
    ];
    let meridian = Frame::new(Vec3::ZERO, -Vec3::Y, Vec3::X).unwrap();
    let par = revolve(&Profile { loops: vec![ProfileLoop { segments: segs }] }, &meridian, Vec3::ZERO, Vec3::Z, std::f64::consts::TAU, "p").unwrap();
    let blk = yack_springrep::primitive::block(&Frame::new(Vec3::new(-9.0, -9.0, -1.0), Vec3::Z, Vec3::X).unwrap(), Vec3::new(18.0, 18.0, 1.0001), "b").unwrap();
    let (a, b) = (k.insert(yack_bridge::to_cadrs(&par).unwrap().0), k.insert(yack_bridge::to_cadrs(&blk).unwrap().0));
    let tip = k.boolean(BoolOp::Intersect, a, &[b]).unwrap().first().unwrap();
    // The lemon: the arc about (−3, 0) of radius 5 from (0, −4) to (0, 4), and the axis.
    let start = (-4.0f64).atan2(3.0);
    let lemon_segs = [Seg::Arc { center: [-3.0, 0.0], radius: 5.0, start, sweep: -2.0 * start }, Seg::Line { a: [0.0, 4.0], b: [0.0, -4.0] }];
    let lemon = crevolve(&mut k, &world, &[&lemon_segs], std::f64::consts::TAU).unwrap().0;
    // The spheroid: a weighted quadratic B-spline and the axis.
    let h = 0.5f64.sqrt();
    let ctrl = [[0.0, -10.0], [6.0, -10.0], [6.0, 0.0], [6.0, 10.0], [0.0, 10.0]];
    let knots = [0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
    let w = [1.0, h, 1.0, h, 1.0];
    let sph_segs = [Seg::Nurbs { degree: 2, knots: &knots, ctrl: &ctrl, weights: Some(&w) }, Seg::Line { a: [0.0, 10.0], b: [0.0, -10.0] }];
    let spheroid = crevolve(&mut k, &world, &[&sph_segs], std::f64::consts::TAU).unwrap().0;
    let mut lines = Vec::new();
    let mut oks = Vec::new();
    for (name, id, want) in [("tip", tip, 1.2566e-7), ("lemon", lemon, 57.2997), ("spheroid", spheroid, 1507.964)] {
        let body = (**k.get(id).unwrap()).clone();
        let v = k.mass_properties(id).unwrap().volume;
        let r = yack_bridge::to_springrep(&body, &[]);
        lines.push(format!("{name}: cadrs volume {v} (want ~{want}); to_springrep {}", match &r { Ok(b) => format!("ok {}", measure::volume(b)), Err(e) => e.to_string() }));
        oks.push((name, r.is_ok()));
    }
    std::fs::write("/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/accept-probe.txt", lines.join("\n")).unwrap();
    // Each crosses whole, or is refused by the self-check alone (never by a lost body), and is then
    // held in cadrs form by the lazy bridge (lead ruling: the self-check stays strict). springrep's
    // classifier misplaces points on all three (the tip is below its ray caster's resolution, B3;
    // the lemon's poles and the spheroid's tips, m3-locate-plan §2), while cadrs places them right.
    for ((name, ok), line) in oks.into_iter().zip(&lines) {
        assert!(ok || line.contains("the body did not cross intact"), "\nFAILED {name}: {line}");
    }
}
