// yack: `yack::make`, every way to make a solid from nothing (PLAN §4.3, D3), routed to springrep
// (M1.2).
//! Every way to make a solid from nothing: primitives, extrusions, revolutions, lofts and sweeps.
//!
//! Each constructor places its shape in an explicit [`Frame`]; a profile is drawn in the frame's
//! xy plane unless the function says otherwise.

use yack_boolean as sr_boolean;
use yack_build::loft as sr_loft;
use yack_build::primitive;
use yack_build::sweep;

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
    // api.md:246
    if !size.iter().all(|s| s.is_finite() && *s > 0.0) {
        return Err(Error::invalid("block", "size", format!("size {size:?} mm is not positive on every side")));
    }
    firewall::op("block", &[], || {
        ladder::two("block", &|| {
        let feature = engine::feature();
        engine::made_as(&feature, "block", engine::BLOCK_SIDES, primitive::block(&engine::frame(frame), yack_math::kernel::Vec3::from_array(size), &feature))
    }, &|| engine_cadrs::primitive("block", frame, yack_cadrs::yack_glue::Prim::Block { size }))
    })
}

/// A cylinder of `radius` on the frame's z axis, from z = 0 to z = `height`.
///
/// ```
/// use yack::prelude::*;
/// let c = cylinder(&Frame::WORLD, 2.0, 5.0)?;
/// assert!((c.volume() - std::f64::consts::PI * 20.0).abs() < 1e-9 * c.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn cylinder(frame: &Frame, radius: f64, height: f64) -> Result<Solid> {
    // api.md:263
    if !(radius.is_finite() && radius > 0.0) {
        return Err(Error::invalid("cylinder", "radius", format!("radius {radius} mm is not positive")));
    }
    if !(height.is_finite() && height > 0.0) {
        return Err(Error::invalid("cylinder", "height", format!("height {height} mm is not positive")));
    }
    firewall::op("cylinder", &[], || {
        ladder::two("cylinder", &|| {
        let feature = engine::feature();
        engine::made_as(&feature, "cylinder", &[("side", "seg:side")], primitive::cylinder(&engine::frame(frame), radius, height, &feature))
    }, &|| engine_cadrs::primitive("cylinder", frame, yack_cadrs::yack_glue::Prim::Cylinder { radius, height }))
    })
}

/// A sphere of `radius` centred at the frame origin.
///
/// ```
/// use yack::prelude::*;
/// let s = sphere(&Frame::WORLD, 3.0)?;
/// assert!((s.volume() - 36.0 * std::f64::consts::PI).abs() < 1e-9 * s.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sphere(frame: &Frame, radius: f64) -> Result<Solid> {
    // api.md:280
    if !(radius.is_finite() && radius > 0.0) {
        return Err(Error::invalid("sphere", "radius", format!("radius {radius} mm is not positive")));
    }
    firewall::op("sphere", &[], || {
        ladder::two("sphere", &|| {
        let feature = engine::feature();
        engine::made(&feature, "sphere", primitive::sphere(&engine::frame(frame), radius, &feature))
    }, &|| engine_cadrs::primitive("sphere", frame, yack_cadrs::yack_glue::Prim::Sphere { radius }))
    })
}

/// A torus about the frame's z axis: tube centre at `major` from the axis, tube radius `minor`.
/// The tube must clear the axis (`minor < major`); a horn or spindle torus is refused as Invalid,
/// pointing at [`spindle_torus`].
///
/// ```
/// use yack::prelude::*;
/// let t = torus(&Frame::WORLD, 5.0, 1.0)?; // 2π²·R·r²
/// let pi = std::f64::consts::PI;
/// assert!((t.volume() - 10.0 * pi * pi).abs() < 1e-9 * t.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn torus(frame: &Frame, major: f64, minor: f64) -> Result<Solid> {
    // api.md:297: major > minor > 0
    if !(minor.is_finite() && minor > 0.0) {
        return Err(Error::invalid("torus", "minor", format!("minor {minor} mm is not positive")));
    }
    if !(major.is_finite() && major > 0.0) {
        return Err(Error::invalid("torus", "major", format!("major {major} mm is not positive")));
    }
    if major > 0.0 && minor >= major {
        return Err(Error::invalid(
            "torus",
            "minor",
            format!("tube radius {minor} reaches the axis (major {major}); for the solid such a tube sweeps use make::spindle_torus"),
        ));
    }
    firewall::op("torus", &[], || {
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
///
/// ```
/// use yack::prelude::*;
/// // a tube wider than its ring: the self-overlapping spindle (cadrs builds it, or it is refused)
/// match spindle_torus(&Frame::WORLD, 1.0, 2.0) {
///     Ok(s) => assert!(s.volume() > 0.0),
///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
/// }
/// # Ok::<(), yack::Error>(())
/// ```
pub fn spindle_torus(frame: &Frame, major: f64, minor: f64) -> Result<Solid> {
    let op = "spindle_torus";
    if !(major > 0.0 && minor >= major) {
        return Err(Error::invalid(op, "minor", format!("tube radius {minor} clears the axis (major {major}): that is make::torus")));
    }
    let (c, z) = (frame.origin().to_array(), frame.z().vec().to_array());
    firewall::op(op, &[], || {
        ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::make(op, |k| yack_cadrs::yack_glue::torus(k, c, z, major, minor)) }])
    })
}

/// A cone (or frustum) on the frame's z axis: radius `bottom` at z = 0, `top` at z = `height`.
///
/// ```
/// use yack::prelude::*;
/// let c = cone(&Frame::WORLD, 2.0, 1.0, 3.0)?; // a frustum: πh/3·(R² + Rr + r²) = 7π
/// assert!((c.volume() - 7.0 * std::f64::consts::PI).abs() < 1e-9 * c.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn cone(frame: &Frame, bottom: f64, top: f64, height: f64) -> Result<Solid> {
    // api.md:315: a positive height; each end's radius at least zero, not both zero
    if !(height.is_finite() && height > 0.0) {
        return Err(Error::invalid("cone", "height", format!("height {height} mm is not positive")));
    }
    if !(bottom.is_finite() && bottom >= 0.0) {
        return Err(Error::invalid("cone", "bottom", format!("bottom radius {bottom} mm is negative")));
    }
    if !(top.is_finite() && top >= 0.0) {
        return Err(Error::invalid("cone", "top", format!("top radius {top} mm is negative")));
    }
    if bottom <= 0.0 && top <= 0.0 {
        return Err(Error::invalid("cone", "bottom", "the cone has no radius at either end"));
    }
    firewall::op("cone", &[], || {
        ladder::two("cone", &|| {
        let feature = engine::feature();
        engine::made_as(&feature, "cone", &[("bottom", "start"), ("top", "end"), ("side", "seg:side")], primitive::cone(&engine::frame(frame), bottom, top, height, &feature))
    }, &|| engine_cadrs::primitive("cone", frame, yack_cadrs::yack_glue::Prim::Cone { bottom, top, height }))
    })
}

/// `profile`, drawn in the frame's xy plane, extruded `height` (`> 0`) along the frame's z. To
/// extrude the other way, flip the frame (`Frame::new(origin, x, -y, -z)`); a negative height is
/// refused.
///
/// A negative `height` extrudes to the other side of the profile's plane, along the frame's −z
/// ("reverse direction"): the same outline, drawn as given, the solid the mirror image of the
/// positive one in that plane. (Flipping the frame instead would mirror the outline.)
///
/// ```
/// use yack::prelude::*;
/// let rect = Profile::rectangle([0.0, 0.0], [2.0, 3.0])?;
/// assert!((extrude(&Frame::WORLD, &rect, 4.0)?.volume() - 24.0).abs() < 1e-9);
/// // a negative height extrudes the same outline to the other side of its plane
/// assert!((extrude(&Frame::WORLD, &rect, -4.0)?.volume() - 24.0).abs() < 1e-9);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn extrude(frame: &Frame, profile: &Profile, height: f64) -> Result<Solid> {
    // api.md:334 (a negative height is yack's: the other side of the plane)
    if !(height.is_finite() && height != 0.0) {
        return Err(Error::invalid("extrude", "height", format!("height {height} mm is not a nonzero length")));
    }
    if height < 0.0 {
        return reversed("extrude", frame, extrude(frame, profile, height.abs()));
    }
    firewall::op("extrude", &[], || {
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
///
/// ```
/// use yack::prelude::*;
/// // the ring of a 2 × 3 rectangle 2 to 4 from the axis: π(4² − 2²)·3
/// let ring = revolve(&Frame::WORLD, &Profile::rectangle([2.0, 0.0], [4.0, 3.0])?, deg(360.0))?;
/// assert!((ring.volume() - 36.0 * std::f64::consts::PI).abs() < 1e-9 * ring.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn revolve(frame: &Frame, profile: &Profile, angle: Angle) -> Result<Solid> {
    if !(angle.radians() > 0.0 && angle.radians() <= core::f64::consts::TAU) {
        return Err(Error::invalid("revolve", "angle", format!("{}° is not above 0° and at most a full turn", angle.degrees())));
    }
    firewall::op("revolve", &[], || {
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

/// The shape of a CNC cutter (see [`cutter`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cutter {
    /// A flat end mill: a plain cylinder.
    Flat,
    /// A ball-end mill: a hemisphere of the cutter's radius at its tip.
    Ball,
    /// A bull-nose mill: a flat end whose rim is rounded by `corner_radius` (above 0, below the
    /// cutter's radius).
    BullNose {
        /// The corner round's radius (mm).
        corner_radius: f64,
    },
    /// A drill: a cone of included angle `point_angle` at its tip (118° for a twist drill, 90° for
    /// a spot drill), above 0° and below 180°.
    Drill {
        /// The point's included angle.
        point_angle: Angle,
    },
}

/// A cutter of `radius` and `length` (from its tip), its tip at the frame origin and its axis the
/// frame's z: the tool solid for [`Solid::cut_swept`](crate::Solid::cut_swept) along a
/// [`Toolpath`](crate::Toolpath). Its shank runs straight up at full radius to `length`, which must
/// reach above the tip's round or point.
///
/// ```
/// use yack::prelude::*;
/// let ball = cutter(&Frame::at([0.0, 0.0, 5.0]), Cutter::Ball, 3.0, 30.0)?;
/// let pi = std::f64::consts::PI;
/// assert!((ball.volume() - (2.0 / 3.0 * pi * 27.0 + pi * 9.0 * 27.0)).abs() < 1e-6 * ball.volume());
/// let bull = cutter(&Frame::WORLD, Cutter::BullNose { corner_radius: 1.5 }, 4.0, 25.0)?;
/// let drill = cutter(&Frame::WORLD, Cutter::Drill { point_angle: deg(90.0) }, 2.5, 40.0)?;
/// assert!(bull.volume() < pi * 16.0 * 25.0 && drill.volume() < pi * 6.25 * 40.0);
/// assert!(cutter(&Frame::WORLD, Cutter::Ball, 3.0, 2.0).is_err()); // shorter than its ball
/// # Ok::<(), yack::Error>(())
/// ```
pub fn cutter(frame: &Frame, kind: Cutter, radius: f64, length: f64) -> Result<Solid> {
    // adapter: yack's own constructor (not in api.md); a revolve of the cutter's side, or a cylinder
    let op = "cutter";
    if !(radius.is_finite() && radius > 0.0) {
        return Err(Error::invalid(op, "radius", format!("radius {radius} mm is not positive")));
    }
    let short = |above: f64| -> Result<()> {
        if length.is_finite() && length > above {
            Ok(())
        } else {
            Err(Error::invalid(op, "length", format!("length {length} mm does not reach above the tip's {above} mm")))
        }
    };
    let side = match kind {
        Cutter::Flat => {
            short(0.0)?;
            return cylinder(frame, radius, length).map_err(|e| e.in_op(op));
        }
        Cutter::Ball => {
            short(radius)?;
            crate::Loop::starting_at([0.0, 0.0]).arc_ccw_to([radius, radius], [0.0, radius]).line_to([radius, length]).line_to([0.0, length]).close()?
        }
        Cutter::BullNose { corner_radius: corner } => {
            if !(corner.is_finite() && corner > 0.0 && corner < radius) {
                return Err(Error::invalid(op, "corner", format!("corner {corner} mm is not above 0 and below the radius {radius} mm")));
            }
            short(corner)?;
            let flat = yack_math::flat_end_radius(radius, corner);
            let side = crate::Loop::starting_at([0.0, 0.0]).line_to([flat, 0.0]).arc_ccw_to([radius, corner], [flat, corner]);
            side.line_to([radius, length]).line_to([0.0, length]).close()?
        }
        Cutter::Drill { point_angle: point } => {
            if !(point.is_finite() && point > crate::deg(0.0) && point < crate::deg(180.0)) {
                return Err(Error::invalid(op, "point", format!("point {}° is not above 0° and below 180°", point.degrees())));
            }
            let tip = yack_math::drill_point_height(radius, point);
            short(tip)?;
            crate::Loop::polygon(&[[0.0, 0.0], [radius, tip], [radius, length], [0.0, length]])?
        }
    };
    revolve(frame, &Profile::new(side, Vec::new())?, crate::deg(360.0)).map_err(|e| e.in_op(op))
}

/// `profile` (drawn as for [`revolve`]) screwed about the frame's z axis by `angle` while rising by
/// `lead` ([`Lead::per_turn`], [`Lead::per_radian`]; a polynomial lead is not built yet).
///
/// ```
/// use yack::prelude::*;
/// // a 1 × 1 section 4.5 from the axis, two turns at 3 a turn: by Pappus, area × 2π × 4.5 × 2
/// let thread = screw(&Frame::WORLD, &Profile::rectangle([4.0, 0.0], [5.0, 1.0])?, Lead::per_turn(3.0), deg(720.0))?;
/// assert!((thread.volume() - 18.0 * std::f64::consts::PI).abs() < 1e-6 * thread.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn screw(frame: &Frame, profile: &Profile, lead: Lead, angle: Angle) -> Result<Solid> {
    // api.md:378
    if !(angle.is_finite() && angle.radians() > 0.0) {
        return Err(Error::invalid("screw", "angle", format!("{}° is not positive", angle.degrees())));
    }
    let rise = match &lead.0 {
        LeadKind::PerRadian(c) => *c,
        LeadKind::PerTurn(p) => yack_math::per_radian_of(*p),
        LeadKind::Polynomial(_) => return Err(Error::new(crate::ErrorKind::Unsupported, "screw", "a screw of growing lead is not built yet")),
    };
    ladder::springrep_only("screw", &[], || {
        let feature = engine::feature();
        let op = "screw";
        let sketch = engine::profile(op, profile, None)?;
        let motion = sweep::Helix { origin: engine::p3(frame.origin()), axis: engine::v3(frame.z().vec()), rise, sign: 1.0 };
        engine::made(&feature, op, sweep::helix(&sketch, &engine::meridian(frame), &motion, angle.radians(), &feature))
    })
}

/// As [`screw`], the profile also moving out from the axis by `taper` mm each radian turned (a
/// conical thread, a spiroid worm).
///
/// ```
/// use yack::prelude::*;
/// // the same section moving out 0.1 a radian as it turns: a conical thread
/// let section = Profile::rectangle([4.0, 0.0], [5.0, 1.0])?;
/// match screw_tapered(&Frame::WORLD, &section, Lead::per_turn(3.0), deg(360.0), 0.1) {
///     Ok(t) => assert!(t.volume() > 0.0),
///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
/// }
/// # Ok::<(), yack::Error>(())
/// ```
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
    firewall::op(op, &[], || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::screw_tapered(op, frame, profile, rise, a, taper) }]))
}

/// `profile`, drawn in the frame's xy plane, carried along the polyline through `corners` (with
/// `closed`, back to the first) and mitred at each corner: every run is cut by, and meets the
/// next in, the plane through the corner bisecting it. The frame must sit on `corners[0]` with
/// its z along the first run; an open path ends square to it.
///
/// ```
/// use yack::prelude::*;
/// // a 10 × 10 bar round a 60° corner, mitred: w²L = 100 × 80
/// let corners = [Point3::new(0.0, 0.0, 0.0), Point3::new(40.0, 0.0, 0.0), Point3::new(20.0, 20.0 * 3f64.sqrt(), 0.0)];
/// let bar = Profile::rectangle([-5.0, -5.0], [5.0, 5.0])?;
/// let elbow = sweep_mitered(&Frame::along(corners[0], Dir3::X)?, &bar, &corners, false)?;
/// assert!((elbow.volume() - 8000.0).abs() < 1e-6 * 8000.0);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sweep_mitered(frame: &Frame, profile: &Profile, corners: &[Point3], closed: bool) -> Result<Solid> {
    let op = "sweep_mitered";
    let (&[a, b, ..], least) = (corners, if closed { 3 } else { 2 }) else {
        return Err(Error::invalid(op, "corners", "a path needs two corners or more"));
    };
    if corners.len() < least {
        return Err(Error::invalid(op, "corners", "a closed path needs three corners or more"));
    }
    if !frame.starts_run(a, b) {
        return Err(Error::invalid(op, "frame", "frame must sit on corners[0] with z along the first run"));
    }
    firewall::op(op, &[], || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::sweep_mitered(op, frame, profile, corners, closed) }]))
}

/// The solid(s) the `patches` bound once sewn edge to edge: every patch's face built, their
/// shared rims joined; an open shell is refused.
///
/// ```
/// use yack::prelude::*;
/// // a corner tetrahedron from four flat triangles: volume 1/6
/// let tri = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| -> yack::Result<Patch> {
///     Ok(Patch::PlanarFill { boundary: vec![Spline3::bezier(&[a, b])?, Spline3::bezier(&[b, c])?, Spline3::bezier(&[c, a])?] })
/// };
/// let (o, x, y, z) = ([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
/// let t = sewn(&[tri(o, y, x)?, tri(o, x, z)?, tri(o, z, y)?, tri(x, y, z)?])?;
/// assert!((t.volume() - 1.0 / 6.0).abs() < 1e-9);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sewn(patches: &[Patch]) -> Result<Solid> {
    let op = "sewn";
    if patches.is_empty() {
        return Err(Error::invalid(op, "patches", "no patches"));
    }
    firewall::op(op, &[], || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::sewn(op, patches) }]))
}

/// A tapered post: `profile` at z = 0 joined by straight lines to its copy scaled by `scale`
/// about the frame's z axis at z = `height` (a scale of 1 is an extrusion; below 1 it narrows
/// upward). `scale` and `height` must be positive.
///
/// ```
/// use yack::prelude::*;
/// // a 6 × 6 square up 6 to half its size: a frustum, h/3·(A₁ + A₂ + √(A₁A₂)) = 2·(36 + 9 + 18)
/// let f = loft(&Frame::WORLD, &Profile::rectangle([-3.0, -3.0], [3.0, 3.0])?, 0.5, 6.0)?;
/// assert!((f.volume() - 126.0).abs() < 1e-6 * 126.0);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn loft(frame: &Frame, profile: &Profile, scale: f64, height: f64) -> Result<Solid> {
    // api.md:403
    if !(height.is_finite() && height > 0.0) {
        return Err(Error::invalid("loft", "height", format!("height {height} mm is not positive")));
    }
    if !(scale.is_finite() && scale > 0.0) {
        return Err(Error::invalid("loft", "scale", format!("scale {scale} is not positive")));
    }
    let top = &profile.scaled("loft", scale)?;
    firewall::op("loft", &[], || {
        ladder::two("loft", &|| {
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
///
/// ```
/// use yack::prelude::*;
/// // the same square at heights 0 and 5: a prism
/// let sq = Profile::rectangle([-1.0, -1.0], [1.0, 1.0])?;
/// let prism = loft_through(&Frame::WORLD, &[(0.0, sq.clone()), (5.0, sq)])?;
/// assert!((prism.volume() - 20.0).abs() < 1e-6);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn loft_through(frame: &Frame, sections: &[(f64, Profile)]) -> Result<Solid> {
    if sections.len() < 2 {
        return Err(Error::invalid("loft_through", "sections", "a loft needs two sections or more"));
    }
    if sections.windows(2).any(|w| matches!(w, [(a, _), (b, _)] if b.partial_cmp(a) != Some(core::cmp::Ordering::Greater))) {
        return Err(Error::invalid("loft_through", "sections", "the sections' heights must rise"));
    }
    ladder::springrep_only("loft_through", &[], || {
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
///
/// ```
/// use yack::prelude::*;
/// // two squares 4 apart, their corners joined by straight guides: a prism
/// let sq = Profile::rectangle([-1.0, -1.0], [1.0, 1.0])?;
/// let guides = [[1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0], [1.0, -1.0]]
///     .map(|[x, y]| Spline3::bezier(&[[x, y, 0.0], [x, y, 4.0]]))
///     .into_iter()
///     .collect::<yack::Result<Vec<_>>>()?;
/// match loft_guided(&[(Frame::WORLD, sq.clone()), (Frame::at([0.0, 0.0, 4.0]), sq)], &guides) {
///     Ok(prism) => assert!((prism.volume() - 16.0).abs() < 1e-6),
///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
/// }
/// # Ok::<(), yack::Error>(())
/// ```
pub fn loft_guided(sections: &[(Frame, Profile)], guides: &[Spline3]) -> Result<Solid> {
    let op = "loft_guided";
    firewall::op(op, &[], || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::loft_guided(op, sections, guides) }]))
}

/// `profile` swept along a path of lines and arcs starting along the frame's z. A positive arc
/// angle turns toward the frame's +x.
///
/// ```
/// use yack::prelude::*;
/// // a disk of radius 1 swept 10 straight on: π·10
/// let pipe = sweep(&Frame::WORLD, &Profile::circle([0.0, 0.0], 1.0)?, &[PathPiece::Line { length: 10.0 }])?;
/// assert!((pipe.volume() - 10.0 * std::f64::consts::PI).abs() < 1e-6 * pipe.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sweep(frame: &Frame, profile: &Profile, path: &[PathPiece]) -> Result<Solid> {
    if path.is_empty() {
        return Err(Error::invalid("sweep", "path", "no path pieces to sweep along"));
    }
    for (i, p) in path.iter().enumerate() {
        let bad = match *p {
            PathPiece::Line { length } => !(length.is_finite() && length > 0.0),
            PathPiece::Arc { radius, angle } => !(radius.is_finite() && radius > 0.0 && angle.is_finite() && angle.radians() != 0.0),
        };
        if bad {
            return Err(Error::invalid("sweep", "path", format!("path piece {i} needs a positive length or radius and a nonzero angle")));
        }
    }
    firewall::op("sweep", &[], || {
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
        engine::made(&feature, op, sweep::along(&engine::profile(op, profile, None)?, &engine::frame(frame), &legs, &feature, sr_boolean::union))
    }, &|| engine_cadrs::sweep("sweep", frame, profile, path))
    })
}

/// `profile` swept along a spline `path` given in the frame's coordinates (it starts at the
/// profile), twisting by `twist` over its length.
///
/// ```
/// use yack::prelude::*;
/// // a disk of radius 1 along a straight spline 10 long (in the frame's coordinates, along its z)
/// let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]])?;
/// let rod = sweep_path(&Frame::WORLD, &Profile::circle([0.0, 0.0], 1.0)?, &path, deg(0.0))?;
/// assert!((rod.volume() - 10.0 * std::f64::consts::PI).abs() < 1e-6 * rod.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sweep_path(frame: &Frame, profile: &Profile, path: &Spline3, twist: Angle) -> Result<Solid> {
    firewall::op("sweep_path", &[], || {
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
///
/// ```
/// use yack::prelude::*;
/// // a 4 × 3 slab whose top is flat at height 2 (a bilinear height field): a box
/// let s = slab(&Frame::WORLD, [4.0, 3.0], &[0.0, 0.0, 1.0, 1.0], &[0.0, 0.0, 1.0, 1.0], &[vec![2.0, 2.0], vec![2.0, 2.0]])?;
/// assert!(s.volume() > 0.0);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn slab(frame: &Frame, size: [f64; 2], knots_u: &[f64], knots_v: &[f64], heights: &[Vec<f64>]) -> Result<Solid> {
    // api.md:520: every height positive
    if !heights.iter().flatten().all(|h| h.is_finite() && *h > 0.0) {
        return Err(Error::invalid("slab", "heights", "every height must be positive"));
    }
    firewall::op("slab", &[], || {
        ladder::two("slab", &|| {
        let feature = engine::feature();
        engine::made(&feature, "slab", primitive::slab(&engine::frame(frame), size, (knots_u, knots_v), heights, &feature))
    }, &|| engine_cadrs::slab("slab", frame, size, knots_u, knots_v, heights))
    })
}

/// A solid bounded by closed NURBS face(s) with control net `ctrl` (rows along u).
///
/// ```
/// use yack::prelude::*;
/// // the rational quadratic NURBS sphere of radius 5, closed by itself
/// let c = [[1., 0.], [1., 1.], [0., 1.], [-1., 1.], [-1., 0.], [-1., -1.], [0., -1.], [1., -1.], [1., 0.]];
/// let m = [[0., -5.], [5., -5.], [5., 0.], [5., 5.], [0., 5.]];
/// let net: Vec<Vec<Point3>> = c.iter().map(|&[x, y]| m.iter().map(|&[r, z]| Point3::new(r * x, r * y, z)).collect()).collect();
/// let w: Vec<Vec<f64>> = (0..9).map(|i| (0..5).map(|j| 0.5f64.sqrt().powi(i % 2 + j % 2)).collect()).collect();
/// let (ku, kv) = ([0., 0., 0., 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1., 1., 1.], [0., 0., 0., 0.5, 0.5, 1., 1., 1.]);
/// let ball = nurbs_solid(&Frame::WORLD, [2, 2], [&ku, &kv], &net, Some(&w))?;
/// assert!((ball.volume() - 500.0 / 3.0 * std::f64::consts::PI).abs() < 1e-6 * ball.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn nurbs_solid(
    frame: &Frame,
    degree: [u32; 2],
    knots: [&[f64]; 2],
    ctrl: &[Vec<Point3>],
    weights: Option<&[Vec<f64>]>,
) -> Result<Solid> {
    let op = "nurbs_solid";
    firewall::op(op, &[], || {
        ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::nurbs_solid(op, frame, degree, knots, ctrl, weights) }])
    })
}

/// The solid with no pieces.
///
/// ```
/// use yack::prelude::*;
/// let nothing = empty();
/// assert!(nothing.is_empty() && nothing.volume() == 0.0);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn empty() -> Solid {
    Solid::empty()
}

/// An extrusion to the other side of the frame's plane: `positive`'s mirror image in that plane,
/// its history `positive`'s (each face the mirror of the same-numbered face).
fn reversed(op: &'static str, frame: &Frame, positive: Result<Solid>) -> Result<Solid> {
    let m = crate::Similarity::mirror(frame.origin(), frame.z()).map_err(|e| Error { op, ..e })?;
    let positive = positive?;
    firewall::run(op, || positive.transform_unrecorded(m).map(|s| s.with_moved_history(&positive).through(&positive)))
}

/// One cadrs rung behind the firewall: the constructors only cadrs builds.
fn cadrs_only(op: &'static str, run: &dyn Fn() -> Result<Solid>) -> Result<Solid> {
    firewall::op(op, &[], || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run }]))
}

/// `profile` extruded `height` along the frame's z with its sides drafted by `draft`: positive
/// leans them in as they rise (the top smaller), negative out. With `both_ways` the same again
/// below the profile's plane, the two halves meeting in a crease there.
///
/// A negative `height` extrudes to the other side of the profile's plane (along the frame's −z),
/// the draft still leaning the sides in as they leave that plane: the mirror image of the
/// positive one in it.
///
/// ```
/// use yack::prelude::*;
/// // a 10 × 10 square up 10, its sides leaning in 5°: a square frustum
/// let sq = Profile::rectangle([-5.0, -5.0], [5.0, 5.0])?;
/// let f = extrude_drafted(&Frame::WORLD, &sq, 10.0, deg(5.0), false)?;
/// assert!(f.volume() < 1000.0 && f.volume() > 600.0);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn extrude_drafted(frame: &Frame, profile: &Profile, height: f64, draft: Angle, both_ways: bool) -> Result<Solid> {
    let op = "extrude_drafted";
    if height < 0.0 {
        return reversed(op, frame, extrude_drafted(frame, profile, height.abs(), draft, both_ways));
    }
    if height.is_nan() || height == 0.0 {
        return Err(Error::invalid(op, "height", format!("height {height} is not a non-zero length")));
    }
    let d = draft.radians();
    let cadrs = || engine_cadrs::extrude_drafted(op, frame, profile, height, d, both_ways);
    // yack-build's native rung after cadrs's, so cadrs's answer stands until a bake-off promotes it;
    // its sides are `seg:side` (beside their segments' own names), as a loft's
    let native = || {
        let feature = engine::feature();
        let made = yack_build::drafted::extrude_drafted(&engine::frame(frame), &engine::profile(op, profile, None)?, height, d, both_ways, &feature);
        engine::made_as(&feature, op, &[("side", "seg:side")], made)
    };
    firewall::op(op, &[], || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &cadrs }, ladder::Rung { name: "native", run: &native }]))
}

/// The solid whose sides are straight lines from `bottom_profile` (drawn in `bottom`) to
/// `top_profile` (drawn in `top`); turning the top frame about the axis twists the rulings.
///
/// ```
/// use yack::prelude::*;
/// // circles of radius 5 at z = ±5, the top turned a quarter turn: a hyperboloid, 500π/3
/// let rim = Profile::circle([0.0, 0.0], 5.0)?;
/// let top = Frame::new([0.0, 0.0, 5.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0])?;
/// let h = ruled(&Frame::at([0.0, 0.0, -5.0]), &rim, &top, &rim)?;
/// assert!((h.volume() - 500.0 / 3.0 * std::f64::consts::PI).abs() < 1e-6 * h.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn ruled(bottom: &Frame, bottom_profile: &Profile, top: &Frame, top_profile: &Profile) -> Result<Solid> {
    let op = "ruled";
    let cadrs = || engine_cadrs::ruled(op, bottom, bottom_profile, top, top_profile);
    // yack-build's native rung first (bake-off doc/donors.md: 506 cases, one reaches ruled, V/A
    // bit-identical, every face with an origin); cadrs answers what it declines. Its sides are
    // `seg:side`, as a loft's
    let native = || {
        let feature = engine::feature();
        let (b, t) = (engine::profile(op, bottom_profile, None)?, engine::profile(op, top_profile, None)?);
        let made = yack_build::ruled::ruled(&engine::frame(bottom), &b, &engine::frame(top), &t, &feature);
        engine::made_as(&feature, op, &[("side", "seg:side")], made)
    };
    firewall::op(op, &[], || ladder::ladder(op, &[ladder::Rung { name: "native", run: &native }, ladder::Rung { name: "cadrs", run: &cadrs }]))
}

/// `profile` (in the frame's xy plane) carried along the helix about the line through
/// `axis_point` along `axis` that starts at the frame's origin, rising `lead` while turning
/// `angle`; its radius is the origin's distance from the axis. Not [`screw`], which turns a
/// profile drawn through its axis.
///
/// ```
/// use yack::prelude::*;
/// // a pipe of radius 1 on a helix of radius 10 rising 5 a turn, three turns: πr²·L
/// let start = Frame::along([10.0, 0.0, 0.0], [0.0, 20.0 * std::f64::consts::PI, 5.0])?;
/// let pipe = sweep_helix(&start, &Profile::circle([0.0, 0.0], 1.0)?, [0.0; 3], Dir3::Z, Lead::per_turn(5.0), deg(1080.0))?;
/// assert!((pipe.volume() - 594.048305033062).abs() < 1e-6 * pipe.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sweep_helix(frame: &Frame, profile: &Profile, axis_point: impl Into<Point3>, axis: impl Into<crate::Vec3>, lead: Lead, angle: Angle) -> Result<Solid> {
    let op = "sweep_helix";
    let pitch = match &lead.0 {
        LeadKind::PerTurn(p) => *p,
        LeadKind::PerRadian(c) => yack_math::per_turn_of(*c),
        LeadKind::Polynomial(_) => return Err(Error::new(crate::ErrorKind::Unsupported, op, "a helix of growing lead is not built yet")),
    };
    let (p, a) = (axis_point.into().to_array(), axis.into().to_array());
    let turns = yack_math::turns_of(angle.radians());
    cadrs_only(op, &|| engine_cadrs::sweep_helix(op, frame, profile, p, a, pitch, turns))
}

/// [`sweep`], the profile turning `twist` about the path and scaling to `end_scale` times its
/// size by the path's end, both evenly along it.
///
/// ```
/// use yack::prelude::*;
/// // a 2 × 2 square run 10 straight on, shrinking to half: a square frustum, 70/3
/// let sq = Profile::rectangle([-1.0, -1.0], [1.0, 1.0])?;
/// let f = sweep_scaled(&Frame::WORLD, &sq, &[PathPiece::Line { length: 10.0 }], deg(0.0), 0.5)?;
/// assert!((f.volume() - 70.0 / 3.0).abs() < 1e-6 * f.volume());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sweep_scaled(frame: &Frame, profile: &Profile, path: &[PathPiece], twist: Angle, end_scale: f64) -> Result<Solid> {
    let op = "sweep_scaled";
    if end_scale.is_nan() || end_scale <= 0.0 {
        return Err(Error::invalid(op, "end_scale", format!("scale {end_scale} is not positive")));
    }
    let t = twist.radians();
    cadrs_only(op, &|| engine_cadrs::sweep_scaled(op, frame, profile, path, t, end_scale))
}

/// `profile` (in the frame's xy plane) lofted to the point `apex`, its sides following
/// `guides` (each from the profile to the apex).
///
/// ```
/// use yack::prelude::*;
/// // a 2 × 2 square lofted to a point 3 above it along straight guides: a pyramid, 4
/// let sq = Profile::rectangle([-1.0, -1.0], [1.0, 1.0])?;
/// let guides = [[1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0], [1.0, -1.0]]
///     .map(|[x, y]| Spline3::bezier(&[[x, y, 0.0], [0.0, 0.0, 3.0]]))
///     .into_iter()
///     .collect::<yack::Result<Vec<_>>>()?;
/// let spire = loft_to_point(&Frame::WORLD, &sq, [0.0, 0.0, 3.0], &guides)?;
/// assert!((spire.volume() - 4.0).abs() < 1e-6);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn loft_to_point(frame: &Frame, profile: &Profile, apex: impl Into<Point3>, guides: &[Spline3]) -> Result<Solid> {
    let op = "loft_to_point";
    let apex = apex.into().to_array();
    cadrs_only(op, &|| engine_cadrs::loft_to_point(op, frame, profile, apex, guides))
}

impl Spline3 {
    /// The conical helix about the line through `axis_point` along `axis`, its radius going from
    /// `r0` to `r1` as it rises `height`, from the angle `start` (measured from `x`, square to the
    /// axis) over `turns` turns, as one clamped quintic B-spline (a loft guide or a path).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let guide = Spline3::conical_helix([0.0; 3], Dir3::Z, Dir3::X, 10.0, 5.0, 20.0, deg(0.0), 2.0)?;
    /// assert_eq!(guide.degree(), 5);
    /// # Ok::<(), yack::Error>(())
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn conical_helix(
        axis_point: impl Into<Point3>,
        axis: impl Into<crate::Vec3>,
        x: impl Into<crate::Vec3>,
        r0: f64,
        r1: f64,
        height: f64,
        start: Angle,
        turns: f64,
    ) -> Result<Spline3> {
        let op = "Spline3::conical_helix";
        let (o, a, x) = (axis_point.into().to_array(), axis.into().to_array(), x.into().to_array());
        let (knots, ctrl) = engine_cadrs::conical_helix(op, o, a, x, r0, r1, height, start.radians(), turns)?;
        Spline3::new(5, knots, ctrl.into_iter().map(Point3::from).collect())
    }
}
