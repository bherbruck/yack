// yack: codec's tests that build their operands with springrep modules above yack-codec.
// Adapted from springrep@a03e8a1 src/codec (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
/// Moved from codec/tests.rs (M3 W5): they encode built solids (sweep, boolean, blend), above yack-codec; they reach the codec through to_bytes/from_bytes.
#[cfg(test)]
mod codec_moved {
    use yack_blend::fillet;
    use yack_boolean::difference;
    use yack_math::kernel::Frame;
    use yack_math::kernel::Point3;
    use yack_math::kernel::Vec2;
    use yack_math::kernel::Vec3;
    use yack_measure as measure;
    use yack_profile::Profile;
    use yack_build::sweep::extrude;
    use yack_topo::Brep;

    /// The solids' bytes through the codec's public path (yack-codec keeps `Codec` crate-private).
    fn bytes<T: Solids>(x: &T) -> Vec<u8> {
        yack_codec::to_bytes(&x.solids())
    }

    fn back<T: Solids>(b: &[u8]) -> Option<T> {
        yack_codec::from_bytes(b).and_then(T::from_solids)
    }

    /// A solid, or a list of solids, as `to_bytes`/`from_bytes` carry them.
    trait Solids: Sized {
        fn solids(&self) -> Vec<Brep>;
        fn from_solids(v: Vec<Brep>) -> Option<Self>;
    }

    impl Solids for Brep {
        fn solids(&self) -> Vec<Brep> {
            vec![self.clone()]
        }
        fn from_solids(mut v: Vec<Brep>) -> Option<Brep> {
            (v.len() == 1).then(|| v.remove(0))
        }
    }

    impl Solids for Vec<Brep> {
        fn solids(&self) -> Vec<Brep> {
            self.clone()
        }
        fn from_solids(v: Vec<Brep>) -> Option<Vec<Brep>> {
            Some(v)
        }
    }

    /// `x` read back from its bytes equals it, and writes the same bytes again: every number kept to
    /// its last bit, signed zeros included.
    fn round_trips<T: Solids + PartialEq + std::fmt::Debug>(x: &T) {
        let b = bytes(x);
        let y: T = back(&b).expect("reads back");
        assert_eq!(&y, x);
        assert_eq!(bytes(&y), b);
    }

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Vec3::new(x, y, z)
    }

    /// A 20 × 20 × 10 block with a Ø6 hole through it and its top's two edges along y rounded R1.
    fn drilled_and_rounded() -> Brep {
        let block = extrude(
            &Profile::rect(Vec2::ZERO, 20.0, 20.0),
            &at_z(0.0),
            10.0,
            "block",
        )
        .unwrap();
        let hole = extrude(&Profile::circle(Vec2::ZERO, 6.0), &at_z(0.0), 10.0, "hole").unwrap();
        let drilled = difference(&block, &hole).unwrap();
        let edges = [("block.end", "block.u_min"), ("block.end", "block.u_max")];
        fillet(&drilled, &edges, 1.0, "round").unwrap()
    }

    fn at_z(z: f64) -> Frame {
        Frame::new(p(0.0, 0.0, z), p(0.0, 0.0, 1.0), p(1.0, 0.0, 0.0)).unwrap()
    }

    #[test]
    fn a_built_solid_reads_back_with_its_ids_and_names() {
        let b = drilled_and_rounded();
        let volume = measure::volume(&b);
        let back: Brep = back(&bytes(&b)).unwrap();
        assert_eq!(back, b);
        assert_eq!(bytes(&back), bytes(&b));
        // Measured again from its read-back geometry, it has the same volume to the last bit.
        assert_eq!(measure::volume(&back).to_bits(), volume.to_bits());
    }

    #[test]
    fn the_bytes_do_not_depend_on_what_was_asked_of_the_solid() {
        // Queries fill a face's memo (its volume share, its box, the trims a boolean reads); none of
        // that is written, so a solid encodes the same before and after them.
        let b = drilled_and_rounded();
        let before = bytes(&b);
        measure::volume(&b);
        measure::area(&b);
        let rays = yack_mesh::rays(&b).unwrap();
        rays.side(p(5.0, 5.0, 5.0));
        let square = Profile::rect(Vec2::new(10.0, 10.0), 4.0, 4.0);
        let post = extrude(&square, &at_z(-5.0), 20.0, "post").unwrap();
        difference(&b, &post).unwrap();
        assert_eq!(bytes(&b), before);
        let pair = vec![b.clone(), b];
        let back: Vec<Brep> = back(&bytes(&pair)).unwrap();
        assert_eq!(bytes(&back), bytes(&pair));
    }

    #[test]
    fn local_tolerances_read_back() {
        let mut b = drilled_and_rounded();
        let v = b.vertex_ids().next().unwrap();
        let e = b.edge_ids().last().unwrap();
        b.set_vertex_tolerance(v, 2e-4);
        b.set_edge_tolerance(e, 1e-4);
        round_trips(&b);
    }
}
