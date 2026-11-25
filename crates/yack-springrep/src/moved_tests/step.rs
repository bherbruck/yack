// yack: tests above yack-blend's STEP round trips: solids blend builds, written and read back by step.
// Adapted from springrep@a03e8a1 src/blend/shaped/corner.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the corner test's STEP round trip, moved out of yack-blend (M3 W9a); its corners read from
// codec fixtures.
//! Moved from blend/shaped/corner.rs (M3 W9a): step is above yack-blend, so the corners come from
//! codec fixtures, each proven equal to a fresh build by the one-off printer (w-h1-37).

#[cfg(test)]
mod blend_step {
    use crate::{codec, measure, topo::Brep};

    fn fixture(name: &str) -> Brep {
        let bytes: &[u8] = match name {
            "corner-1.6-2" => include_bytes!("../../tests/data/corner-1.6-2.yack"),
            "corner-0.8-5" => include_bytes!("../../tests/data/corner-0.8-5.yack"),
            "l-rounded-0" => include_bytes!("../../tests/data/l-rounded-0.yack"),
            "l-rounded-1" => include_bytes!("../../tests/data/l-rounded-1.yack"),
            "l-rounded-2" => include_bytes!("../../tests/data/l-rounded-2.yack"),
            "l-rounded-3" => include_bytes!("../../tests/data/l-rounded-3.yack"),
            "l-rounded-4" => include_bytes!("../../tests/data/l-rounded-4.yack"),
            "l-rounded-5" => include_bytes!("../../tests/data/l-rounded-5.yack"),
            "l-rounded-6" => include_bytes!("../../tests/data/l-rounded-6.yack"),
            "l-rounded-7" => include_bytes!("../../tests/data/l-rounded-7.yack"),
            _ => panic!("no fixture {name}"),
        };
        codec::from_bytes(bytes).and_then(|mut v| v.pop()).unwrap()
    }

    /// a_curvature_continuous_set_back_corner_holds_at_other_setbacks's corners, set back 2 at
    /// radius 1.6 and 5 at 0.8, written and read back: the same solid.
    #[test]
    fn a_set_back_corner_reads_back_from_a_file_as_the_same_solid() {
        for tag in ["corner-1.6-2", "corner-0.8-5"] {
            let out = fixture(tag);
            let text = crate::step::write(&out, "corner").unwrap();
            let back = crate::step::read(text.as_bytes(), "corner").unwrap();
            assert!(back.rejected.is_empty(), "{:?}", back.rejected);
            let (v, w) = (measure::volume(&out), measure::volume(&back.brep));
            assert!((v - w).abs() < 1e-7 * v, "{tag}: {v} against {w}");
        }
    }

    /// an_l_s_rounded_edges_swing_round_its_concave_edge's eight Ls, every convex edge rounded,
    /// written and read back: the swing's torus, whose pole lies on its boundary, is the same solid.
    /// Moved from edit.rs (M3 W9b).
    #[test]
    fn a_rounded_l_reads_back_from_a_file_as_the_same_solid() {
        for k in 0..8 {
            let out = fixture(&format!("l-rounded-{k}"));
            let text = crate::step::write(&out, "l").unwrap();
            let back = crate::step::read(text.as_bytes(), "l").unwrap();
            assert!(back.rejected.is_empty(), "{:?}", back.rejected);
            let (v, w) = (measure::volume(&out), measure::volume(&back.brep));
            assert!((v - w).abs() < 1e-7 * v.max(1.0), "l-rounded-{k}: {v} against {w}");
        }
    }
}
