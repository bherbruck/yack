// yack: `yack::make`, every way to make a solid from nothing (PLAN §4.3, D3), routed to springrep
// (M1.2).
//! Every way to make a solid from nothing: primitives, extrusions, revolutions, lofts and sweeps.
//!
//! Each constructor places its shape in an explicit [`Frame`]; a profile is drawn in the frame's
//! xy plane unless the function says otherwise.

use yack_springrep::{loft as sr_loft, primitive, sweep};

use std::collections::BTreeSet;

use crate::engine;
use crate::{engine_cadrs, firewall, ladder};
use crate::input::LeadKind;
use crate::{Angle, Error, Frame, Lead, PathPiece, Patch, Point3, Profile, Result, Solid, Spline3};

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
    firewall::run("block", || {
        ladder::two("block", &|| {
        let feature = engine::feature();
        engine::made_as(&feature, "block", engine::BLOCK_SIDES, primitive::block(&engine::frame(frame), yack_springrep::math::Vec3::from_array(size), &feature))
    }, &|| engine_cadrs::primitive("block", frame, yack_cadrs::yack_glue::Prim::Block { size }))
    })
}

/// A cylinder of `radius` on the frame's z axis, from z = 0 to z = `height`.
pub fn cylinder(frame: &Frame, radius: f64, height: f64) -> Result<Solid> {
    firewall::run("cylinder", || {
        ladder::two("cylinder", &|| {
        let feature = engine::feature();
        engine::made_as(&feature, "cylinder", &[("side", "seg:side")], primitive::cylinder(&engine::frame(frame), radius, height, &feature))
    }, &|| engine_cadrs::primitive("cylinder", frame, yack_cadrs::yack_glue::Prim::Cylinder { radius, height }))
    })
}

/// A sphere of `radius` centred at the frame origin.
pub fn sphere(frame: &Frame, radius: f64) -> Result<Solid> {
    firewall::run("sphere", || {
        ladder::two("sphere", &|| {
        let feature = engine::feature();
        engine::made(&feature, "sphere", primitive::sphere(&engine::frame(frame), radius, &feature))
    }, &|| engine_cadrs::primitive("sphere", frame, yack_cadrs::yack_glue::Prim::Sphere { radius }))
    })
}

/// A torus about the frame's z axis: tube centre at `major` from the axis, tube radius `minor`.
/// The tube must clear the axis (`minor < major`); a horn or spindle torus is refused as Invalid,
/// pointing at [`spindle_torus`].
pub fn torus(frame: &Frame, major: f64, minor: f64) -> Result<Solid> {
    if major > 0.0 && minor >= major {
        return Err(Error::invalid(
            "torus",
            "minor",
            format!("tube radius {minor} reaches the axis (major {major}); for the solid such a tube sweeps use make::spindle_torus"),
        ));
    }
    firewall::run("torus", || {
        let feature = engine::feature();
        let (c, z) = (frame.origin().to_array(), frame.z().vec().to_array());
        ladder::two(
            "torus",
            // A tube reaching the axis is invalid (the protocol's rule); springrep says so, finally.
            &|| engine::made_as(&feature, "torus", &[("tube", "skin")], primitive::torus(&engine::frame(frame), major, minor, &feature)),
            &|| engine_cadrs::make("torus", |k| yack_cadrs::yack_glue::torus(k, c, z, major, minor)),
        )
    })
}

/// The solid a tube of radius `minor` sweeps as its centre circle of radius `major` turns about the
/// frame's z axis, for a tube that reaches the axis (`minor ≥ major > 0`; a horn torus when equal):
/// every point within `minor` of that circle, the core the tube crosses itself in counted once.
/// A ring torus (`minor < major`) is [`torus`].
pub fn spindle_torus(frame: &Frame, major: f64, minor: f64) -> Result<Solid> {
    let op = "spindle_torus";
    if !(major > 0.0 && minor >= major) {
        return Err(Error::invalid(op, "minor", format!("tube radius {minor} clears the axis (major {major}): that is make::torus")));
    }
    let (c, z) = (frame.origin().to_array(), frame.z().vec().to_array());
    firewall::run(op, || {
        ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::make(op, |k| yack_cadrs::yack_glue::torus(k, c, z, major, minor)) }])
    })
}

/// A cone (or frustum) on the frame's z axis: radius `bottom` at z = 0, `top` at z = `height`.
pub fn cone(frame: &Frame, bottom: f64, top: f64, height: f64) -> Result<Solid> {
    firewall::run("cone", || {
        ladder::two("cone", &|| {
        let feature = engine::feature();
        engine::made_as(&feature, "cone", &[("bottom", "start"), ("top", "end"), ("side", "seg:side")], primitive::cone(&engine::frame(frame), bottom, top, height, &feature))
    }, &|| engine_cadrs::primitive("cone", frame, yack_cadrs::yack_glue::Prim::Cone { bottom, top, height }))
    })
}

/// `profile`, drawn in the frame's xy plane, extruded `height` (`> 0`) along the frame's z. To
/// extrude the other way, flip the frame (`Frame::new(origin, x, -y, -z)`); a negative height is
/// refused.
pub fn extrude(frame: &Frame, profile: &Profile, height: f64) -> Result<Solid> {
    if height < 0.0 {
        return Err(Error::invalid(
            "extrude",
            "height",
            format!("height {height} is negative; to extrude the other way, flip the frame: Frame::new(origin, x, -y, -z)"),
        ));
    }
    firewall::run("extrude", || {
        ladder::two(
            "extrude",
            &|| {
        let feature = engine::feature();
        let op = "extrude";
        engine::made(&feature, op, sweep::extrude(&engine::profile(op, profile, None)?, &engine::frame(frame), height, &feature))
    },
            &|| engine_cadrs::sweep_profile("extrude", frame, profile, height, false),
        )
    })
}

/// `profile` revolved about the frame's z axis by `angle`. The profile is drawn in the frame's xz
/// plane: profile x along the frame's x (out from the axis, x ≥ 0), profile y along the frame's z.
pub fn revolve(frame: &Frame, profile: &Profile, angle: Angle) -> Result<Solid> {
    firewall::run("revolve", || {
        ladder::two(
            "revolve",
            &|| {
        let feature = engine::feature();
        let op = "revolve";
        let sketch = engine::profile(op, profile, None)?;
        let axis = engine::v3(frame.z().vec());
        engine::made(&feature, 
            op,
            sweep::revolve(&sketch, &engine::meridian(frame), engine::p3(frame.origin()), axis, angle.radians(), &feature),
        )
    },
            &|| engine_cadrs::sweep_profile("revolve", frame, profile, angle.radians(), true),
        )
    })
}

/// `profile` (drawn as for [`revolve`]) screwed about the frame's z axis by `angle` while rising by
/// `lead` ([`Lead::per_turn`], [`Lead::per_radian`]; a polynomial lead is not built yet).
pub fn screw(frame: &Frame, profile: &Profile, lead: Lead, angle: Angle) -> Result<Solid> {
    let rise = match &lead.0 {
        LeadKind::PerRadian(c) => *c,
        LeadKind::PerTurn(p) => yack_math::per_radian_of(*p),
        LeadKind::Polynomial(_) => return Err(Error::new(crate::ErrorKind::Unsupported, "screw", "a screw of growing lead is not built yet")),
    };
    ladder::springrep_only("screw", || {
        let feature = engine::feature();
        let op = "screw";
        let sketch = engine::profile(op, profile, None)?;
        let motion = sweep::Helix { origin: engine::p3(frame.origin()), axis: engine::v3(frame.z().vec()), rise, sign: 1.0 };
        engine::made(&feature, op, sweep::helix(&sketch, &engine::meridian(frame), &motion, angle.radians(), &feature))
    })
}

/// As [`screw`], the profile also moving out from the axis by `taper` mm each radian turned (a
/// conical thread, a spiroid worm).
pub fn screw_tapered(frame: &Frame, profile: &Profile, lead: Lead, angle: Angle, taper: f64) -> Result<Solid> {
    let op = "screw_tapered";
    let rise = match &lead.0 {
        LeadKind::PerRadian(c) => *c,
        LeadKind::PerTurn(p) => yack_math::per_radian_of(*p),
        LeadKind::Polynomial(_) => return Err(Error::new(crate::ErrorKind::Unsupported, op, "a tapered screw of growing lead is not built yet")),
    };
    if !taper.is_finite() {
        return Err(Error::invalid(op, "taper", "not finite"));
    }
    let a = angle.radians();
    firewall::run(op, || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::screw_tapered(op, frame, profile, rise, a, taper) }]))
}

/// The solid(s) the `patches` bound once sewn edge to edge: every patch's face built, their
/// shared rims joined; an open shell is refused.
pub fn sewn(patches: &[Patch]) -> Result<Solid> {
    let op = "sewn";
    if patches.is_empty() {
        return Err(Error::invalid(op, "patches", "no patches"));
    }
    firewall::run(op, || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::sewn(op, patches) }]))
}

/// A ruled loft from `profile` at z = 0 to `top` at z = `height`. As in the protocol, `top` is
/// `profile` scaled by `scale` about the frame's z axis, sent ready-made; yack lofts to `top` as
/// given and does not scale it again.
pub fn loft(frame: &Frame, profile: &Profile, scale: f64, height: f64, top: &Profile) -> Result<Solid> {
    firewall::run("loft", || {
        ladder::two("loft", &|| {
        let _ = scale; // recorded by the protocol; `top` is already the scaled copy
        let feature = engine::feature();
        let op = "loft";
        let up = Frame::new(frame.point([0.0, 0.0, height]), frame.x(), frame.y(), frame.z()).map_err(|e| e.in_op(op))?;
        let sections = [
            sr_loft::Section { profile: engine::profile(op, profile, None)?, frame: engine::frame(frame) },
            sr_loft::Section { profile: engine::profile(op, top, None)?, frame: engine::frame(&up) },
        ];
        engine::made_as(&feature, op, &[("side", "seg:side")], sr_loft::loft(&sections, true, &feature))
    }, &|| engine_cadrs::loft("loft", frame, profile, height, top))
    })
}

/// A loft through `sections`, each a profile at a height along the frame's z.
pub fn loft_through(frame: &Frame, sections: &[(f64, Profile)]) -> Result<Solid> {
    if sections.len() < 2 {
        return Err(Error::invalid("loft_through", "sections", "a loft needs two sections or more"));
    }
    if sections.windows(2).any(|w| matches!(w, [(a, _), (b, _)] if b.partial_cmp(a) != Some(core::cmp::Ordering::Greater))) {
        return Err(Error::invalid("loft_through", "sections", "the sections' heights must rise"));
    }
    ladder::springrep_only("loft_through", || {
        let feature = engine::feature();
        let op = "loft_through";
        let counts: BTreeSet<usize> = sections.iter().map(|(_, p)| p.outer().segments().len()).collect();
        let sections = sections
            .iter()
            .map(|(h, p)| {
                let at = Frame::new(frame.point([0.0, 0.0, *h]), frame.x(), frame.y(), frame.z()).map_err(|e| e.in_op(op))?;
                Ok(sr_loft::Section { profile: engine::profile(op, p, None)?, frame: engine::frame(&at) })
            })
            .collect::<Result<Vec<_>>>()?;
        let out = engine::made_as(&feature, op, &[("side", "seg:side")], sr_loft::loft(&sections, false, &feature))?;
        if counts.len() > 1 {
            // a hard guard (brep-bench loft/square-to-round): never this wrong solid
            let _ = out;
            return Err(Error::failed(op, crate::Stage::Validate, "springrep declines: sections with different segment counts loft as twisted ruled walls"));
        }
        Ok(out)
    })
}

/// A loft through profiles placed in their own frames, following `guides`.
pub fn loft_guided(sections: &[(Frame, Profile)], guides: &[Spline3]) -> Result<Solid> {
    let op = "loft_guided";
    firewall::run(op, || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::loft_guided(op, sections, guides) }]))
}

/// `profile` swept along a path of lines and arcs starting along the frame's z. A positive arc
/// angle turns toward the frame's +x.
pub fn sweep(frame: &Frame, profile: &Profile, path: &[PathPiece]) -> Result<Solid> {
    firewall::run("sweep", || {
        ladder::two("sweep", &|| {
        let feature = engine::feature();
        let op = "sweep";
        let axis = engine::v3(frame.y().vec());
        let legs: Vec<sweep::Leg> = path
            .iter()
            .map(|p| match *p {
                PathPiece::Line { length } => sweep::Leg::Line { length },
                PathPiece::Arc { radius, angle } => sweep::Leg::Arc { radius, angle: angle.radians(), axis },
            })
            .collect();
        engine::made(&feature, op, sweep::along(&engine::profile(op, profile, None)?, &engine::frame(frame), &legs, &feature))
    }, &|| engine_cadrs::sweep("sweep", frame, profile, path))
    })
}

/// `profile` swept along a spline `path` given in the frame's coordinates (it starts at the
/// profile), twisting by `twist` over its length.
pub fn sweep_path(frame: &Frame, profile: &Profile, path: &Spline3, twist: Angle) -> Result<Solid> {
    firewall::run("sweep_path", || {
        ladder::two("sweep_path", &|| {
        let feature = engine::feature();
        let op = "sweep_path";
        let (curve, domain) = engine::spline(op, path, |c| frame.point(c.to_array()))?;
        let sketch = engine::profile(op, profile, None)?;
        engine::made(&feature, op, sweep::along_curve(&sketch, &engine::frame(frame), &curve, domain, twist.radians(), &feature))
    }, &|| engine_cadrs::sweep_path("sweep_path", frame, profile, path, twist.radians()))
    })
}

/// A slab over the `size` rectangle with a spline top: `heights[i][j]` over knots `knots_u[i]`,
/// `knots_v[j]`.
pub fn slab(frame: &Frame, size: [f64; 2], knots_u: &[f64], knots_v: &[f64], heights: &[Vec<f64>]) -> Result<Solid> {
    firewall::run("slab", || {
        ladder::two("slab", &|| {
        let feature = engine::feature();
        engine::made(&feature, "slab", primitive::slab(&engine::frame(frame), size, (knots_u, knots_v), heights, &feature))
    }, &|| engine_cadrs::slab("slab", frame, size, knots_u, knots_v, heights))
    })
}

/// A solid bounded by closed NURBS face(s) with control net `ctrl` (rows along u).
pub fn nurbs_solid(
    frame: &Frame,
    degree: [u32; 2],
    knots: [&[f64]; 2],
    ctrl: &[Vec<Point3>],
    weights: Option<&[Vec<f64>]>,
) -> Result<Solid> {
    let op = "nurbs_solid";
    firewall::run(op, || {
        ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::nurbs_solid(op, frame, degree, knots, ctrl, weights) }])
    })
}

/// The solid with no pieces.
pub fn empty() -> Solid {
    Solid::empty()
}
