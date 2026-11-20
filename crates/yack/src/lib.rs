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
//!     block.difference(&sphere(&Frame::WORLD, 3.0)?.swept(&Motion::Along { path, law: FrameLaw::Fixed })?)
//! }
//! // Until the kernel lands, constructors refuse with `Unsupported`, naming the operation.
//! let e = build().unwrap_err();
//! assert_eq!((e.kind, e.op), (ErrorKind::Unsupported, "block"));
//! ```
#![deny(missing_docs)]
#![deny(clippy::float_arithmetic)]

mod input;
pub mod make;
mod solid;
mod values;

pub use input::{FrameLaw, Loop, Motion, PathPiece, Pen, Profile, Segment, Spline3};
pub use solid::Solid;
pub use values::{
    CurveKind, Display, DisplayFace, Drawing, EdgeId, EdgeRef, FaceId, FaceRef, History, Interference, Issue, Mesh,
    Plane, Ray, Side, Split, SurfaceKind, Tag, VertexId,
};
pub use yack_base::{Error, ErrorKind, Result, Stage};
pub use yack_math::{Aabb, Affine, Angle, Dir3, Frame, Point2, Point3, Rigid, Similarity, Vec3};

/// An angle of `d` degrees.
pub fn deg(d: f64) -> Angle {
    yack_math::deg(d)
}

/// An angle of `r` radians.
pub fn rad(r: f64) -> Angle {
    yack_math::rad(r)
}

/// Everything a case program needs: `use yack::prelude::*;`.
pub mod prelude {
    pub use crate::make::{
        block, cone, cylinder, empty, extrude, loft, loft_guided, loft_through, nurbs_solid, revolve, screw, slab,
        sphere, sweep, sweep_path, torus,
    };
    pub use crate::{
        Affine, Angle, Dir3, EdgeId, Error, ErrorKind, FaceId, Frame, FrameLaw, Loop, Motion, PathPiece, Plane, Point2,
        Point3, Profile, Ray, Result, Rigid, Segment, Side, Similarity, Solid, Spline3, Tag, Vec3, deg, rad,
    };
}

/// STEP exchange (AP214, exact B-rep; never faceted).
pub mod step {
    use crate::{Error, Result, Solid};

    /// The solid as an AP214 STEP file.
    pub fn write(s: &Solid) -> Result<String> {
        let _ = s;
        Err(Error::unsupported("step::write"))
    }
    /// The solid in a STEP file.
    pub fn read(text: &str) -> Result<Solid> {
        let _ = text;
        Err(Error::unsupported("step::read"))
    }
    /// The solid sewn from the faces in a STEP file, closing gaps up to `tolerance` mm.
    pub fn sew(text: &str, tolerance: f64) -> Result<Solid> {
        let _ = (text, tolerance);
        Err(Error::unsupported("step::sew"))
    }
}

/// yack's own lossless binary format.
pub mod codec {
    use crate::{Error, Result, Solid};

    const EMPTY: &[u8] = b"yack-codec 0 empty";

    /// The solid as bytes that [`decode`] reads back exactly.
    pub fn encode(s: &Solid) -> Vec<u8> {
        // M0.3: only the empty solid has an encoding yet.
        if s.is_empty() { EMPTY.to_vec() } else { Vec::new() }
    }
    /// The solid encoded in `b`.
    pub fn decode(b: &[u8]) -> Result<Solid> {
        if b == EMPTY {
            return Ok(crate::make::empty());
        }
        Err(Error::unsupported("codec::decode"))
    }
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

/// The brep-conformance protocol functions yack implements (the rest refuse `Unsupported`).
pub const SUPPORTS: &[&str] = &[];

#[cfg(test)]
mod tests {
    use crate::prelude::*;

    #[test]
    fn version_is_set() {
        assert!(!crate::VERSION.is_empty());
    }

    #[test]
    fn stubs_name_their_op() {
        let e = sphere(&Frame::WORLD, 3.0).unwrap_err();
        assert_eq!(e.kind, ErrorKind::Unsupported);
        assert_eq!(e.op, "sphere");
    }

    #[test]
    fn empty_queries_are_real() {
        let s = empty();
        assert!(s.is_empty());
        assert_eq!(s.volume(), 0.0);
        assert_eq!(s.betti(), [0, 0, 0]);
        assert_eq!(s.classify(&[[0.0, 0.0, 0.0]]), vec![Side::Out]);
        assert!(s.face_at([0.0, 0.0, 0.0]).is_err());
        assert!(s.mesh(0.1).unwrap().triangles.is_empty());
        let bytes = crate::codec::encode(&s);
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
