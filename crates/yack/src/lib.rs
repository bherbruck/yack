// yack: the public crate; it routes to the kernel crates and never computes (PLAN §4, §6.6).
//! yack, Yet Another CAD Kernel: an exact B-rep geometry kernel in pure, safe Rust.
//!
//! Make solids with [`make`], change and query them with methods on [`Solid`], exchange them with
//! [`step`] and [`codec`]. Lengths are `f64` mm; angles are [`Angle`]s built by [`deg`] or
//! [`rad`]; every shape is placed in an explicit [`Frame`].
//!
//! ```
//! use yack::prelude::*;
//! fn build() -> Result<Solid> {
//!     let block = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0])?;
//!     let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]])?;
//!     block.difference(&sphere(&Frame::WORLD, 3.0)?.swept(&Motion::along(path, FrameLaw::Fixed))?)
//! }
//! let grooved = build()?;
//! assert!(grooved.volume() < 42.0 * 32.0 * 8.0);
//! # Ok::<(), yack::Error>(())
//! ```
//!
//! A plate with a hole through it and one top edge rounded (the README's example, kept here so
//! that it compiles and runs):
//!
//! ```
//! use yack::prelude::*;
//! let plate = block(&Frame::WORLD, [40.0, 30.0, 8.0])?;
//! let cut = plate.difference(&cylinder(&Frame::at([20.0, 15.0, -1.0]), 5.0, 10.0)?)?;
//! let part = cut.fillet(&[cut.edge_at([20.0, 0.0, 8.0])?], 1.0)?;
//! # assert!(part.check().is_empty() && part.volume() < 40.0 * 30.0 * 8.0 - 3.0 * 25.0 * 8.0);
//! # Ok::<(), yack::Error>(())
//! ```
#![deny(missing_docs)]
#![deny(clippy::float_arithmetic)]

#[cfg(feature = "bakeoff")]
pub mod bakeoff;
mod blend;
mod boolean;
pub mod codec;
mod edit;
mod engine;
mod engine_cadrs;
mod firewall;
mod input;
mod ladder;
pub mod make;
mod query;
mod select;
mod solid;
pub mod step;
mod values;

pub use input::{FrameLaw, Lead, Loop, Motion, PathPiece, Patch, Pen, Profile, Segment, Spline3, Toolpath};
pub use solid::Solid;
pub use values::{
    Closest, CurveKind, Display, DisplayFace, Drawing, EdgeId, EdgeRef, FaceId, FaceRef, Interference, Issue, Mesh, NearestFace,
    Join, Plane, Ray, Role, Side, Split, SurfaceKind, Tag, VertexId, VertexRef,
};
pub use yack_base::{Error, ErrorKind, Result, Stage};
pub use yack_math::{Aabb, Affine, Angle, Dir3, Frame, Point2, Point3, Rigid, Similarity, Vec3};

/// An angle of `d` degrees.
///
/// ```
/// use yack::prelude::*;
/// assert!((deg(180.0).radians() - std::f64::consts::PI).abs() < 1e-15);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn deg(d: f64) -> Angle {
    yack_math::deg(d)
}

/// An angle of `n` whole turns (a screw's thirty turns: `turns(30.0)`).
///
/// ```
/// use yack::prelude::*;
/// assert_eq!(turns(0.25).radians(), std::f64::consts::FRAC_PI_2);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn turns(n: f64) -> Angle {
    yack_math::turns(n)
}

/// An angle of `r` radians.
///
/// ```
/// use yack::prelude::*;
/// assert_eq!(rad(1.5).radians(), 1.5);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn rad(r: f64) -> Angle {
    yack_math::rad(r)
}

/// Everything a case program needs: `use yack::prelude::*;`.
pub mod prelude {
    pub use crate::make::{
        block, cone, cylinder, empty, extrude, loft, loft_guided, loft_through, nurbs_solid, revolve, screw, slab,
        sewn, sphere, spindle_torus, screw_tapered, sweep, sweep_mitered, sweep_path, torus, extrude_drafted, ruled,
        sweep_helix, sweep_scaled, loft_to_point, cutter, Cutter,
    };
    pub use crate::{
        Affine, Angle, Dir3, EdgeId, Error, ErrorKind, FaceId, Frame, FrameLaw, Lead, Loop, Motion, Toolpath, PathPiece, Patch, Plane, Point2,
        Point3, Profile, Ray, Result, Rigid, Role, Segment, Join, Side, Similarity, Solid, Spline3, SurfaceKind, CurveKind, Tag, Vec3, deg,
        rad, turns,
    };
}

/// The model tolerances (mm).
pub mod tol {
    /// Two points closer than this are the same point.
    pub const LINEAR: f64 = yack_tol::LINEAR;
    /// How far from an entity a selection point may be.
    pub const PICK: f64 = yack_tol::PICK;
}

/// This crate's version.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// The source revision yack was built from (`YACK_SOURCE` at build time), or `"dev"`.
pub const SOURCE: &str = match option_env!("YACK_SOURCE") {
    Some(s) => s,
    None => "dev",
};

/// The brep-conformance protocol functions yack implements, in protocol spelling (the rest refuse
/// `Unsupported`).
pub const SUPPORTS: &[&str] = &[
    "box",
    "cylinder",
    "sphere",
    "torus",
    "cone",
    "extrude",
    "revolve",
    "screw",
    "loft",
    "loft_through",
    "sweep",
    "sweep_path",
    "slab",
    "union",
    "intersection",
    "difference",
    "union_all",
    "difference_all",
    "fillet",
    "fillet_radii",
    "fillet_variable",
    "fillet_conic",
    "fillet_g2",
    "fillet_setback",
    "fillet_setback_g2",
    "fillet_faces",
    "full_round",
    "chamfer",
    "chamfer_two",
    "chamfer_distances",
    "offset",
    "offset_faces",
    "delete_faces",
    "move_faces",
    "thicken",
    "shell",
    "draft",
    "write_step",
    "sew",
    "read_step",
    "volume",
    "area",
    "classify",
    "closest",
    "normal",
    "curvature",
    "check",
    "centroid",
    "inertia",
    "distance",
    "interference",
    "betti",
    "section_area",
    "section",
    "project",
    "bounding_box",
    "split",
    "mesh",
    "display",
    "raycast",
    "transform",
    "affine",
];

#[cfg(test)]
mod tests {
    use crate::make;
    use crate::prelude::*;

    #[test]
    fn version_is_set() {
        assert!(!crate::VERSION.is_empty());
    }

    #[test]
    fn stubs_name_their_op() {
        let e = make::loft_guided(&[], &[]).unwrap_err();
        assert_eq!((e.kind, e.op), (ErrorKind::Unsupported, "loft_guided"));
    }

    #[test]
    fn solids_are_send_and_sync() {
        fn both<T: Send + Sync>() {}
        both::<Solid>();
    }

    #[test]
    fn empty_queries_are_real() {
        let s = empty();
        assert!(s.is_empty());
        assert_eq!(s.volume(), 0.0);
        assert_eq!(s.betti(), [0, 0, 0]);
        assert_eq!(s.classify(&[[0.0, 0.0, 0.0]]).unwrap(), vec![Side::Out]);
        assert!(s.face_at([0.0, 0.0, 0.0]).is_err());
        assert!(s.mesh(0.1).unwrap().triangles.is_empty());
        let bytes = crate::codec::encode(&s).unwrap();
        assert!(crate::codec::decode(&bytes).unwrap().is_empty());
    }

    #[test]
    fn profile_rules() {
        assert!(Profile::rectangle([0.0, 0.0], [2.0, 1.0]).is_ok());
        assert!(Profile::polygon(&[[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]]).is_ok());
        let cw = Loop::polygon(&[[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]]).unwrap();
        assert!(Profile::new(cw, vec![]).is_err());
        assert!(Profile::circle([0.0, 0.0], 0.0).is_err());
        let e = Loop::starting_at([0.0, 0.0]).line_to([1.0, 0.0]).arc_ccw_to([0.0, 3.0], [0.0, 0.0]).close().unwrap_err();
        assert_eq!(e.arg, Some("center"));
    }

    #[test]
    fn frames_are_checked() {
        assert!(Frame::new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]).is_err());
        let r = Rigid::rotation([0.0; 3], [0.0, 0.0, 1.0], deg(90.0)).unwrap();
        let _ = Frame::WORLD.moved(&r);
    }
}
