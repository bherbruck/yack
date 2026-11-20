// yack: `yack::make`, every way to make a solid from nothing (PLAN §4.3, D3).
//
// M0.3: stubs returning `Unsupported`, except `empty`.
//! Every way to make a solid from nothing: primitives, extrusions, revolutions, lofts and sweeps.
//!
//! Each constructor places its shape in an explicit [`Frame`].
#![allow(unused_variables)]

use crate::{Angle, Error, Frame, PathPiece, Point3, Profile, Result, Solid, Spline3};

fn unsupported(op: &'static str) -> Result<Solid> {
    Err(Error::unsupported(op))
}

/// A box with one corner at the frame origin, extending `size` along the frame's x, y, z
/// (protocol `box`).
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0]);
/// # let _ = b;
/// ```
pub fn block(frame: &Frame, size: [f64; 3]) -> Result<Solid> {
    unsupported("block")
}

/// A cylinder of `radius` on the frame's z axis, from z = 0 to z = `height`.
pub fn cylinder(frame: &Frame, radius: f64, height: f64) -> Result<Solid> {
    unsupported("cylinder")
}

/// A sphere of `radius` centred at the frame origin.
pub fn sphere(frame: &Frame, radius: f64) -> Result<Solid> {
    unsupported("sphere")
}

/// A torus about the frame's z axis: tube centre at `major` from the axis, tube radius `minor`.
pub fn torus(frame: &Frame, major: f64, minor: f64) -> Result<Solid> {
    unsupported("torus")
}

/// A cone (or frustum) on the frame's z axis: radius `bottom` at z = 0, `top` at z = `height`.
pub fn cone(frame: &Frame, bottom: f64, top: f64, height: f64) -> Result<Solid> {
    unsupported("cone")
}

/// `profile`, drawn in the frame's xy plane, extruded `height` along the frame's z.
pub fn extrude(frame: &Frame, profile: &Profile, height: f64) -> Result<Solid> {
    unsupported("extrude")
}

/// `profile`, drawn in the frame's xz plane (x ≥ 0), revolved about the frame's z axis by `angle`.
pub fn revolve(frame: &Frame, profile: &Profile, angle: Angle) -> Result<Solid> {
    unsupported("revolve")
}

/// `profile` revolved about the frame's z axis by `angle` while rising `rise` mm per radian.
pub fn screw(frame: &Frame, profile: &Profile, rise: f64, angle: Angle) -> Result<Solid> {
    unsupported("screw")
}

/// A loft from `profile` at z = 0 to `top` scaled by `scale` at z = `height`.
pub fn loft(frame: &Frame, profile: &Profile, scale: f64, height: f64, top: &Profile) -> Result<Solid> {
    unsupported("loft")
}

/// A loft through `sections`, each a profile at a height along the frame's z.
pub fn loft_through(frame: &Frame, sections: &[(f64, Profile)]) -> Result<Solid> {
    unsupported("loft_through")
}

/// A loft through profiles placed in their own frames, following `guides`.
pub fn loft_guided(sections: &[(Frame, Profile)], guides: &[Spline3]) -> Result<Solid> {
    unsupported("loft_guided")
}

/// `profile` swept along a path of lines and arcs starting along the frame's z.
pub fn sweep(frame: &Frame, profile: &Profile, path: &[PathPiece]) -> Result<Solid> {
    unsupported("sweep")
}

/// `profile` swept along a spline `path`, twisting by `twist` over its length.
pub fn sweep_path(frame: &Frame, profile: &Profile, path: &Spline3, twist: Angle) -> Result<Solid> {
    unsupported("sweep_path")
}

/// A slab over the `size` rectangle with a spline top: `heights[i][j]` over knots `knots_u[i]`,
/// `knots_v[j]`.
pub fn slab(frame: &Frame, size: [f64; 2], knots_u: &[f64], knots_v: &[f64], heights: &[Vec<f64>]) -> Result<Solid> {
    unsupported("slab")
}

/// A solid bounded by closed NURBS face(s) with control net `ctrl` (rows along u).
pub fn nurbs_solid(
    frame: &Frame,
    degree: [u32; 2],
    knots: [&[f64]; 2],
    ctrl: &[Vec<Point3>],
    weights: Option<&[Vec<f64>]>,
) -> Result<Solid> {
    unsupported("nurbs_solid")
}

/// The solid with no pieces.
pub fn empty() -> Solid {
    Solid::empty()
}
