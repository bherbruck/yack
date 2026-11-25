// yack: Curves and surfaces, each with evaluation, derivatives, projection and parameter ranges.
// Adapted from springrep@a03e8a1 src/geom/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/mod.rs to yack-geom (M3 W3), paths and visibility only; M3 hardening dropped its indexing allow.
//! Curves and surfaces, each with evaluation, derivatives, projection and parameter ranges.

// springrep's foundation modules (D26), under the names geom's code uses.
use yack_base::kernel::{self as error, Error, Result};
use yack_math::kernel as math;
use yack_numeric as numeric;
use yack_tol as tol;

mod bspline;
mod chord;
mod curve;
mod intersection;
mod nurbs;
#[cfg(test)]
pub(crate) use nurbs::tests::octant;
mod surface;

pub use bspline::BSpline;
pub use chord::{MeshOptions, edge_params};
pub use curve::{Circle, Curve, Ellipse, Line};
pub use intersection::{IntersectionCurve, UvPair};
pub use intersection::{
    distance_to, refine_across, refine_on_contour, refine_onto_both, refine_onto_both_near,
};
pub use nurbs::Bezier;
pub use nurbs::{IsoLine, NurbsCurve, NurbsSurface, Patch};
pub use surface::{Cone, Cylinder, Extrusion, Plane, Revolution, Ruled, Sphere, Surface, Torus};
