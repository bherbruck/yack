// yack: hostile STEP files the measurer must refuse quickly, without panics or unbounded work.
#![cfg(feature = "wip")]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing, clippy::panic)]

use yack_step::measure_file;

/// One solid of one face on surface `#20`, bounded by the loop of oriented edge `#10`; `extra`
/// supplies `#10` onwards and `#20` onwards.
fn solid(extra: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n\
         #1=MANIFOLD_SOLID_BREP('',#2);\n#2=CLOSED_SHELL('',(#3));\n\
         #3=ADVANCED_FACE('',(#4),#20,.T.);\n#4=FACE_OUTER_BOUND('',#5,.T.);\n#5=EDGE_LOOP('',(#10));\n\
         #30=CARTESIAN_POINT('',(0.,0.,0.));\n#31=DIRECTION('',(0.,0.,1.));\n#32=DIRECTION('',(1.,0.,0.));\n\
         #33=AXIS2_PLACEMENT_3D('',#30,#31,#32);\n{extra}\nENDSEC;\nEND-ISO-10303-21;\n"
    )
}

/// A closed circle edge, its vertex at (1, 0, 0), whose curve is `#40`.
const CIRCLE_EDGE: &str = "#10=ORIENTED_EDGE('',*,*,#11,.T.);\n#11=EDGE_CURVE('',#12,#12,#40,.T.);\n\
     #12=VERTEX_POINT('',#13);\n#13=CARTESIAN_POINT('',(1.,0.,0.));\n";

fn refused(text: &str, why: &str) {
    match measure_file(text) {
        Err(e) => eprintln!("refused ({why}): {e}"),
        Ok(m) => panic!("{why}: measured {m:?}"),
    }
}

#[test]
fn a_degree_past_the_limit_or_not_an_integer_is_refused() {
    for degree in ["1000000000", "0", "-3", "2.5"] {
        let text = solid(&format!(
            "{CIRCLE_EDGE}#40=CIRCLE('',#33,1.);\n#20=B_SPLINE_SURFACE_WITH_KNOTS('',{degree},1,((#30,#30),(#30,#30)),\
             .UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);"
        ));
        refused(&text, &format!("degree {degree}"));
    }
}

#[test]
fn a_cycle_of_oriented_edges_or_surface_curves_is_refused() {
    let edges = solid("#10=ORIENTED_EDGE('',*,*,#14,.T.);\n#14=ORIENTED_EDGE('',*,*,#10,.F.);\n#20=PLANE('',#33);");
    refused(&edges, "oriented edges in a cycle");
    let curves = solid(
        "#10=ORIENTED_EDGE('',*,*,#11,.T.);\n#11=EDGE_CURVE('',#12,#12,#40,.T.);\n#12=VERTEX_POINT('',#13);\n\
         #13=CARTESIAN_POINT('',(1.,0.,0.));\n#40=SURFACE_CURVE('',#41,(),.CURVE_3D.);\n\
         #41=SEAM_CURVE('',#40,(),.CURVE_3D.);\n#20=PLANE('',#33);",
    );
    refused(&curves, "surface curves in a cycle");
}

#[test]
fn hostile_multiplicities_are_refused() {
    for mults in ["(4000000000000,2)", "(0,2)", "(3,2)", "(2,2,2)"] {
        let text = solid(&format!(
            "{CIRCLE_EDGE}#40=B_SPLINE_CURVE_WITH_KNOTS('',1,(#13,#13),.UNSPECIFIED.,.F.,.F.,{mults},(0.,1.),.UNSPECIFIED.);\n\
             #20=PLANE('',#33);"
        ));
        refused(&text, &format!("multiplicities {mults}"));
    }
}

#[test]
fn a_closed_spline_with_a_vanishing_period_is_refused() {
    // A closed (both ends at one point) linear spline over a domain of 1e-300, used as a full edge
    // and as the curve a surface of revolution turns: its period would cut runs into endless pieces.
    let text = solid(
        "#10=ORIENTED_EDGE('',*,*,#11,.T.);\n#11=EDGE_CURVE('',#12,#12,#50,.T.);\n#12=VERTEX_POINT('',#13);\n\
         #13=CARTESIAN_POINT('',(1.,0.,0.));\n#14=CARTESIAN_POINT('',(1.,0.,1E-300));\n\
         #50=B_SPLINE_CURVE_WITH_KNOTS('',1,(#13,#14,#13),.UNSPECIFIED.,.T.,.F.,(2,1,2),(0.,5E-301,1E-300),.UNSPECIFIED.);\n\
         #51=AXIS1_PLACEMENT('',#30,#31);\n#20=SURFACE_OF_REVOLUTION('',#50,#51);",
    );
    match measure_file(&text) {
        Err(e) => assert!(e.detail.contains("period") && e.detail.contains("too small"), "{e}"),
        Ok(m) => panic!("vanishing period: measured {m:?}"),
    }
}
