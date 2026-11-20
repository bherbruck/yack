// yack: `yack::make`, every way to make a solid from nothing (PLAN §4.3, D3), routed to springrep
// (M1.2).
//! Every way to make a solid from nothing: primitives, extrusions, revolutions, lofts and sweeps.
//!
//! Each constructor places its shape in an explicit [`Frame`]; a profile is drawn in the frame's
//! xy plane unless the function says otherwise.

use yack_springrep::{loft as sr_loft, primitive, sweep};

use crate::engine::{self, FEATURE};
use crate::{Angle, Error, Frame, PathPiece, Point3, Profile, Result, Solid, Spline3};

/// A box with one corner at the frame origin, extending `size` along the frame's x, y, z
/// (protocol `box`).
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0])?;
/// assert!((b.volume() - 42.0 * 32.0 * 8.0).abs() < 0.01);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn block(frame: &Frame, size: [f64; 3]) -> Result<Solid> {
    engine::solid("block", primitive::block(&engine::frame(frame), yack_springrep::math::Vec3::from_array(size), FEATURE))
}

/// A cylinder of `radius` on the frame's z axis, from z = 0 to z = `height`.
pub fn cylinder(frame: &Frame, radius: f64, height: f64) -> Result<Solid> {
    engine::solid("cylinder", primitive::cylinder(&engine::frame(frame), radius, height, FEATURE))
}

/// A sphere of `radius` centred at the frame origin.
pub fn sphere(frame: &Frame, radius: f64) -> Result<Solid> {
    engine::solid("sphere", primitive::sphere(&engine::frame(frame), radius, FEATURE))
}

/// A torus about the frame's z axis: tube centre at `major` from the axis, tube radius `minor`.
pub fn torus(frame: &Frame, major: f64, minor: f64) -> Result<Solid> {
    engine::solid("torus", primitive::torus(&engine::frame(frame), major, minor, FEATURE))
}

/// A cone (or frustum) on the frame's z axis: radius `bottom` at z = 0, `top` at z = `height`.
pub fn cone(frame: &Frame, bottom: f64, top: f64, height: f64) -> Result<Solid> {
    engine::solid("cone", primitive::cone(&engine::frame(frame), bottom, top, height, FEATURE))
}

/// `profile`, drawn in the frame's xy plane, extruded `height` along the frame's z.
pub fn extrude(frame: &Frame, profile: &Profile, height: f64) -> Result<Solid> {
    let op = "extrude";
    engine::solid(op, sweep::extrude(&engine::profile(op, profile, None)?, &engine::frame(frame), height, FEATURE))
}

/// `profile` revolved about the frame's z axis by `angle`. The profile is drawn in the frame's xz
/// plane: profile x along the frame's x (out from the axis, x ≥ 0), profile y along the frame's z.
pub fn revolve(frame: &Frame, profile: &Profile, angle: Angle) -> Result<Solid> {
    let op = "revolve";
    let sketch = engine::profile(op, profile, None)?;
    let axis = engine::v3(frame.z().vec());
    engine::solid(
        op,
        sweep::revolve(&sketch, &engine::meridian(frame), engine::p3(frame.origin()), axis, angle.radians(), FEATURE),
    )
}

/// `profile` (drawn as for [`revolve`]) screwed about the frame's z axis by `angle` while rising
/// `rise` mm per radian.
pub fn screw(frame: &Frame, profile: &Profile, rise: f64, angle: Angle) -> Result<Solid> {
    let op = "screw";
    let sketch = engine::profile(op, profile, None)?;
    let motion = sweep::Helix { origin: engine::p3(frame.origin()), axis: engine::v3(frame.z().vec()), rise, sign: 1.0 };
    engine::solid(op, sweep::helix(&sketch, &engine::meridian(frame), &motion, angle.radians(), FEATURE))
}

/// A loft from `profile` at z = 0 to `top` scaled by `scale` (about the frame's z axis) at
/// z = `height`.
pub fn loft(frame: &Frame, profile: &Profile, scale: f64, height: f64, top: &Profile) -> Result<Solid> {
    let op = "loft";
    let up = Frame::new(frame.point([0.0, 0.0, height]), frame.x(), frame.y(), frame.z()).map_err(|e| e.in_op(op))?;
    let sections = [
        sr_loft::Section { profile: engine::profile(op, profile, None)?, frame: engine::frame(frame) },
        sr_loft::Section { profile: engine::profile(op, top, Some(scale))?, frame: engine::frame(&up) },
    ];
    engine::solid(op, sr_loft::loft(&sections, true, FEATURE))
}

/// A loft through `sections`, each a profile at a height along the frame's z.
pub fn loft_through(frame: &Frame, sections: &[(f64, Profile)]) -> Result<Solid> {
    let op = "loft_through";
    let sections = sections
        .iter()
        .map(|(h, p)| {
            let at = Frame::new(frame.point([0.0, 0.0, *h]), frame.x(), frame.y(), frame.z()).map_err(|e| e.in_op(op))?;
            Ok(sr_loft::Section { profile: engine::profile(op, p, None)?, frame: engine::frame(&at) })
        })
        .collect::<Result<Vec<_>>>()?;
    engine::solid(op, sr_loft::loft(&sections, false, FEATURE))
}

/// A loft through profiles placed in their own frames, following `guides`.
pub fn loft_guided(sections: &[(Frame, Profile)], guides: &[Spline3]) -> Result<Solid> {
    let _ = (sections, guides);
    Err(Error::unsupported("loft_guided"))
}

/// `profile` swept along a path of lines and arcs starting along the frame's z. A positive arc
/// angle turns toward the frame's +x.
pub fn sweep(frame: &Frame, profile: &Profile, path: &[PathPiece]) -> Result<Solid> {
    let op = "sweep";
    let axis = engine::v3(frame.y().vec());
    let legs: Vec<sweep::Leg> = path
        .iter()
        .map(|p| match *p {
            PathPiece::Line { length } => sweep::Leg::Line { length },
            PathPiece::Arc { radius, angle } => sweep::Leg::Arc { radius, angle: angle.radians(), axis },
        })
        .collect();
    engine::solid(op, sweep::along(&engine::profile(op, profile, None)?, &engine::frame(frame), &legs, FEATURE))
}

/// `profile` swept along a spline `path` given in the frame's coordinates (it starts at the
/// profile), twisting by `twist` over its length.
pub fn sweep_path(frame: &Frame, profile: &Profile, path: &Spline3, twist: Angle) -> Result<Solid> {
    let op = "sweep_path";
    let (curve, domain) = engine::spline(op, path, |c| frame.point(c.to_array()))?;
    let sketch = engine::profile(op, profile, None)?;
    engine::solid(op, sweep::along_curve(&sketch, &engine::frame(frame), &curve, domain, twist.radians(), FEATURE))
}

/// A slab over the `size` rectangle with a spline top: `heights[i][j]` over knots `knots_u[i]`,
/// `knots_v[j]`.
pub fn slab(frame: &Frame, size: [f64; 2], knots_u: &[f64], knots_v: &[f64], heights: &[Vec<f64>]) -> Result<Solid> {
    engine::solid("slab", primitive::slab(&engine::frame(frame), size, (knots_u, knots_v), heights, FEATURE))
}

/// A solid bounded by closed NURBS face(s) with control net `ctrl` (rows along u).
pub fn nurbs_solid(
    frame: &Frame,
    degree: [u32; 2],
    knots: [&[f64]; 2],
    ctrl: &[Vec<Point3>],
    weights: Option<&[Vec<f64>]>,
) -> Result<Solid> {
    let _ = (frame, degree, knots, ctrl, weights);
    Err(Error::unsupported("nurbs_solid"))
}

/// The solid with no pieces.
pub fn empty() -> Solid {
    Solid::empty()
}
