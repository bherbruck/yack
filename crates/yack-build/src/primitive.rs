// yack: The five primitive solids, each placed by a frame: a block and a cylinder extruded along the
// Adapted from springrep@a03e8a1 src/primitive.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/primitive.rs to yack-build (M3 W7); paths and visibility only.
//! The five primitive solids, each placed by a frame: a block and a cylinder extruded along the
//! frame's z, a sphere, a torus and a cone revolved about it.

mod slab;

pub use slab::slab;

use std::f64::consts::{PI, TAU};

use crate::Result;
use crate::error::invalid;
use crate::math::{Frame, Vec2, Vec3};
use crate::profile::{NamedSegment, Profile, ProfileLoop, Segment};
use crate::sweep::{extrude, revolve};
use crate::topo::Brep;

fn named(segment: Segment, name: &str) -> NamedSegment {
    NamedSegment::new(segment, name)
}

fn one(segments: Vec<NamedSegment>) -> Profile {
    Profile {
        loops: vec![ProfileLoop { segments }],
    }
}

/// Revolves a meridian, drawn with x out from the frame's z axis and y along it, a whole turn.
fn turned(meridian: &Profile, frame: &Frame, feature: &str) -> Result<Brep> {
    let plane = Frame::new(frame.origin, -frame.y, frame.x)?;
    revolve(meridian, &plane, frame.origin, frame.z, TAU, feature)
}

/// The block from the frame's origin out along its axes by `size`; its faces are `start`, `end`,
/// `v_min`, `u_max`, `v_max` and `u_min`.
pub fn block(frame: &Frame, size: Vec3, feature: &str) -> Result<Brep> {
    if size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0 {
        return invalid("a block's sizes must be positive");
    }
    let base = Profile::rect(Vec2::new(size.x / 2.0, size.y / 2.0), size.x, size.y);
    extrude(&base, frame, size.z, feature)
}

/// The cylinder about the frame's z axis from its origin up by `height`; its faces are `start`,
/// `end` and `side`.
pub fn cylinder(frame: &Frame, radius: f64, height: f64, feature: &str) -> Result<Brep> {
    if radius <= 0.0 || height <= 0.0 {
        return invalid("a cylinder's radius and height must be positive");
    }
    extrude(
        &Profile::circle(Vec2::ZERO, 2.0 * radius),
        frame,
        height,
        feature,
    )
}

/// The ball about the frame's origin, its poles on the frame's z axis; its one face is `skin`.
pub fn sphere(frame: &Frame, radius: f64, feature: &str) -> Result<Brep> {
    if radius <= 0.0 {
        return invalid("a sphere's radius must be positive");
    }
    let half = one(vec![
        named(
            Segment::Arc {
                center: Vec2::ZERO,
                radius,
                start: -PI / 2.0,
                sweep: PI,
            },
            "skin",
        ),
        named(
            Segment::Line {
                a: Vec2::new(0.0, radius),
                b: Vec2::new(0.0, -radius),
            },
            "axis",
        ),
    ]);
    turned(&half, frame, feature)
}

/// The ring about the frame's z axis, its tube's centre circle `major` from the axis and the tube
/// of radius `minor`, which must be less; its one face is `tube`.
pub fn torus(frame: &Frame, major: f64, minor: f64, feature: &str) -> Result<Brep> {
    if minor <= 0.0 || major <= minor {
        return invalid("a torus's tube must be positive and clear of its axis");
    }
    let tube = one(vec![named(
        Segment::Circle {
            center: Vec2::new(major, 0.0),
            radius: minor,
            ccw: true,
        },
        "tube",
    )]);
    turned(&tube, frame, feature)
}

/// The frustum about the frame's z axis from its origin up by `height`, of radius `bottom` at the
/// origin and `top` at the far end. Either radius may be zero, for an apex, and equal radii give a
/// cylinder; its faces are `bottom`, `side` and `top`, less the one an apex has not.
pub fn cone(frame: &Frame, bottom: f64, top: f64, height: f64, feature: &str) -> Result<Brep> {
    if bottom < 0.0 || top < 0.0 || bottom + top <= 0.0 || height <= 0.0 {
        return invalid("a cone needs a positive height and radii that are not both zero");
    }
    let pts = [
        Vec2::new(0.0, 0.0),
        Vec2::new(bottom, 0.0),
        Vec2::new(top, height),
        Vec2::new(0.0, height),
    ];
    let names = ["bottom", "side", "top", "axis"];
    // A radius of zero is an apex: one point of the meridian, with no edge.
    let sides = (0..4).filter(|&i| pts[i] != pts[(i + 1) % 4]).map(|i| {
        let line = Segment::Line {
            a: pts[i],
            b: pts[(i + 1) % 4],
        };
        named(line, names[i])
    });
    turned(&one(sides.collect()), frame, feature)
}
