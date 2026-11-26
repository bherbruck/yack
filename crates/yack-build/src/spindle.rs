// yack: the spindle torus, the solid a tube that reaches its own axis sweeps: the revolve of the tube circle's arc beyond the axis, between its two apexes.
//! [`spindle_torus`]: every point within `minor` of a circle of radius `major` about the frame's
//! z axis, for a tube that crosses the axis (`minor > major`). The tube sweeps the part of space
//! near the axis twice (the "lemon"), but as a set that part lies inside the outer surface (the
//! "apple"), so the solid is the revolve of the tube circle's arc on the far side of the axis,
//! from the apex at `z = −√(minor² − major²)` round the outside to the apex at `+√(…)`, closed
//! along the axis. Its one face is `tube`, as a ring torus's is; the apexes are its two poles.

use std::f64::consts::TAU;

use crate::{
    Error, Result,
    math::{Frame, Vec2},
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep::revolve,
    tol,
    topo::Brep,
};

/// The spindle torus about `frame`'s z axis, named under `feature`. Declines (`Unsupported`) a horn
/// torus (`minor` within the tolerance of `major`: its two apexes meet in one point of the axis)
/// and a tube centred on the axis (`major` of zero: a sphere); refuses (`InvalidInput`) radii that
/// are not positive or a tube that clears the axis (`minor < major`: a ring torus).
pub fn spindle_torus(frame: &Frame, major: f64, minor: f64, feature: &str) -> Result<Brep> {
    if !(major.is_finite() && minor.is_finite()) || major < 0.0 || minor <= 0.0 {
        return Err(Error::InvalidInput("a spindle torus's radii must be positive".into()));
    }
    if major <= tol::LINEAR {
        return Err(Error::Unsupported("a tube centred on its axis is a sphere".into()));
    }
    if minor < major {
        return Err(Error::InvalidInput("a tube that clears its axis is a ring torus".into()));
    }
    let apex = (minor * minor - major * major).sqrt();
    if apex <= tol::LINEAR {
        return Err(Error::Unsupported("a horn torus, whose apexes meet in one point".into()));
    }
    // In the meridian (x out from the axis, y along it): the tube circle about (major, 0), from the
    // lower apex counter-clockwise round its far side to the upper one, then down the axis.
    let to_apex = apex.atan2(-major);
    let meridian = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                NamedSegment::new(Segment::Arc { center: Vec2::new(major, 0.0), radius: minor, start: -to_apex, sweep: 2.0 * to_apex }, "tube"),
                NamedSegment::new(Segment::Line { a: Vec2::new(0.0, apex), b: Vec2::new(0.0, -apex) }, "axis"),
            ],
        }],
    };
    let plane = Frame::new(frame.origin, -frame.y, frame.x)?;
    revolve(&meridian, &plane, frame.origin, frame.z, TAU, feature)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;
    use std::f64::consts::PI;

    /// The apple's volume and area in closed form, by Pappus over the arc's region and the arc:
    /// the region (the disc less the segment cut off by the axis) and its centroid, the arc and its.
    fn apple(r: f64, a: f64) -> (f64, f64) {
        let h = (a * a - r * r).sqrt();
        let phi = h.atan2(-r); // half the arc's angle, measured from +x
        // Arc of angle 2φ about (r, 0): length 2aφ, centroid ρ = r + a·sin φ/φ.
        let area = 2.0 * PI * (2.0 * a * phi) * (r + a * phi.sin() / phi);
        // The region: the sector of angle 2φ plus the triangle to the chord on the axis.
        let sector = a * a * phi;
        let sector_x = r + 2.0 * a * phi.sin() / (3.0 * phi);
        let tri = r * h; // centre–apex–apex: ½·2h·r, its centroid at ρ = r/3
        let region = sector + tri;
        let x = (sector * sector_x + tri * (r / 3.0)) / region;
        (2.0 * PI * x * region, area)
    }

    #[test]
    fn a_spindle_torus_is_the_apple() {
        let f = Frame::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let s = spindle_torus(&f, 3.0, 5.0, "t").unwrap();
        let (v, a) = apple(3.0, 5.0);
        let (mv, ma) = (crate::measure::volume(&s), crate::measure::area(&s));
        assert!((mv - v).abs() <= 1e-9 * v, "volume {mv} against {v}");
        assert!((ma - a).abs() <= 1e-9 * a, "area {ma} against {a}");
        assert!(s.face_ids().all(|g| s.face(g).names.iter().any(|n| n.face == "tube")));
    }

    #[test]
    fn horn_rings_and_spheres_are_not_spindles() {
        let f = Frame::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        assert!(matches!(spindle_torus(&f, 5.0, 5.0, "t"), Err(Error::Unsupported(_))));
        assert!(matches!(spindle_torus(&f, 0.0, 5.0, "t"), Err(Error::Unsupported(_))));
        assert!(matches!(spindle_torus(&f, 5.0, 3.0, "t"), Err(Error::InvalidInput(_))));
    }
}
