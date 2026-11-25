// yack: build's tests that build their operands with springrep modules above yack-build.
// Adapted from springrep@a03e8a1 src/build (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
/// Moved from build/mesh_solid.rs's tests (M3 W7): they build above yack-build.
#[cfg(test)]
mod build_mesh_solid_moved {
    use yack_boolean::intersection;
    use yack_build::build::solid_from_mesh as solid;
    use yack_math::kernel::Frame;
    use yack_math::kernel::Point3;
    use yack_math::kernel::Vec3;
    use yack_measure as measure;
    use yack_build::primitive;

    /// The tetrahedron on the origin and the three unit points of the axes scaled by `s`, every
    /// triangle listing its own three corners, as an STL does.
    fn tetrahedron(s: f64) -> (Vec<Point3>, Vec<[u32; 3]>) {
        let (o, x, y, z) = (
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(s, 0.0, 0.0),
            Point3::new(0.0, s, 0.0),
            Point3::new(0.0, 0.0, s),
        );
        let faces = [[o, y, x], [o, x, z], [o, z, y], [x, y, z]];
        let positions: Vec<Point3> = faces.iter().flatten().copied().collect();
        let triangles = (0..4).map(|k| [3 * k, 3 * k + 1, 3 * k + 2]).collect();
        (positions, triangles)
    }

    #[test]
    fn a_closed_mesh_is_the_solid_its_triangles_bound() {
        // Twelve corners at four places: four faces, six edges, four vertices, a sixth of s³.
        let (positions, triangles) = tetrahedron(6.0);
        let b = solid(&positions, &triangles).unwrap();
        assert_eq!(
            (b.face_count(), b.edge_ids().count(), b.vertex_ids().count()),
            (4, 6, 4)
        );
        assert!((measure::volume(&b) - 36.0).abs() < 1e-9);
        // It is a solid like any other: half of a 2 × 2 × 2 block at the origin lies inside it
        // up to the plane x + y + z = 6, which the block does not reach.
        let block = primitive::block(&Frame::WORLD, Vec3::new(2.0, 2.0, 2.0), "b").unwrap();
        assert!((measure::volume(&intersection(&b, &block).unwrap()) - 8.0).abs() < 1e-9);
    }
}
