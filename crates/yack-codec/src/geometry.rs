// yack: [`Codec`] for the geometry: vectors, frames, curves and surfaces.
// Adapted from springrep@a03e8a1 src/codec.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of codec.rs (M3 hardening): each variant written through one tagged-value
// function; the same bytes in the same order.
//! [`Codec`] for the geometry. Analytic curves and surfaces are their fields; a spline is its
//! degree, knots, control points (and weights and period) and is rebuilt through its checked
//! constructor, so bytes that describe no valid spline read as nothing. A curve or surface is a
//! tag naming its kind, then the kind's own encoding.

use crate::wire::{Codec, Reader, Writer, put_tagged};
use yack_geom::{
    BSpline, Circle, Cone, Curve, Cylinder, Ellipse, Extrusion, IntersectionCurve, Line,
    NurbsCurve, NurbsSurface, Plane, Revolution, Ruled, Sphere, Surface, Torus,
};
use yack_math::kernel::{Aabb, Frame, Vec2, Vec3};

fields!(Vec2 { x, y });
fields!(Vec3 { x, y, z });
fields!(Frame { origin, x, y, z });
fields!(Aabb { min, max });
fields!(Line { origin, dir });
fields!(Circle { frame, radius });
fields!(Ellipse { frame, major, minor });
fields!(IntersectionCurve { approx, surfaces });
fields!(Plane { frame });
fields!(Cylinder { frame, radius });
fields!(Cone { frame, radius, half_angle });
fields!(Sphere { frame, radius });
fields!(Torus { frame, major, minor });
fields!(Extrusion { curve, dir });
fields!(Revolution { curve, origin, axis });
fields!(Ruled { a, a_range, b, b_range });

/// The only degree a periodic B-spline is built at.
const PERIODIC_DEGREE: usize = 3;

impl Codec for BSpline {
    fn put(&self, w: &mut Writer) {
        self.degree().put(w);
        self.knots().to_vec().put(w);
        self.control_points().to_vec().put(w);
        self.is_periodic().put(w);
    }
    fn get(r: &mut Reader) -> Option<BSpline> {
        let degree = usize::get(r)?;
        let knots = Vec::get(r)?;
        let ctrl = Vec::get(r)?;
        if bool::get(r)? {
            (degree == PERIODIC_DEGREE).then_some(())?;
            BSpline::periodic_cubic(knots, ctrl).ok()
        } else {
            BSpline::new(degree, knots, ctrl).ok()
        }
    }
}

impl Codec for NurbsCurve {
    fn put(&self, w: &mut Writer) {
        self.degree().put(w);
        self.knots().to_vec().put(w);
        self.control_points().to_vec().put(w);
        self.weights().map(<[f64]>::to_vec).put(w);
        self.period().put(w);
    }
    fn get(r: &mut Reader) -> Option<NurbsCurve> {
        NurbsCurve::restore(
            Codec::get(r)?,
            Codec::get(r)?,
            Codec::get(r)?,
            Codec::get(r)?,
            Codec::get(r)?,
        )
        .ok()
    }
}

impl Codec for NurbsSurface {
    fn put(&self, w: &mut Writer) {
        self.degrees().put(w);
        let (ku, kv) = self.knots();
        (ku.to_vec(), kv.to_vec()).put(w);
        self.counts().put(w);
        self.control_points().to_vec().put(w);
        self.weights().map(<[f64]>::to_vec).put(w);
        (self.u_period(), self.v_period()).put(w);
    }
    fn get(r: &mut Reader) -> Option<NurbsSurface> {
        NurbsSurface::restore(
            Codec::get(r)?,
            Codec::get(r)?,
            Codec::get(r)?,
            Codec::get(r)?,
            Codec::get(r)?,
            Codec::get(r)?,
        )
        .ok()
    }
}

impl Codec for Curve {
    fn put(&self, w: &mut Writer) {
        match self {
            Curve::Line(c) => put_tagged(0, c, w),
            Curve::Circle(c) => put_tagged(1, c, w),
            Curve::Ellipse(c) => put_tagged(2, c, w),
            Curve::BSpline(c) => put_tagged(3, c, w),
            Curve::Intersection(c) => put_tagged(4, c, w),
            Curve::Nurbs(c) => put_tagged(5, c, w),
        }
    }
    fn get(r: &mut Reader) -> Option<Curve> {
        Some(match u8::get(r)? {
            0 => Curve::Line(Codec::get(r)?),
            1 => Curve::Circle(Codec::get(r)?),
            2 => Curve::Ellipse(Codec::get(r)?),
            3 => Curve::BSpline(Codec::get(r)?),
            4 => Curve::Intersection(Codec::get(r)?),
            5 => Curve::Nurbs(Codec::get(r)?),
            _ => return None,
        })
    }
}

impl Codec for Surface {
    fn put(&self, w: &mut Writer) {
        match self {
            Surface::Plane(s) => put_tagged(0, s, w),
            Surface::Cylinder(s) => put_tagged(1, s, w),
            Surface::Cone(s) => put_tagged(2, s, w),
            Surface::Sphere(s) => put_tagged(3, s, w),
            Surface::Torus(s) => put_tagged(4, s, w),
            Surface::Extrusion(s) => put_tagged(5, s, w),
            Surface::Revolution(s) => put_tagged(6, s, w),
            Surface::Ruled(s) => put_tagged(7, s, w),
            Surface::Nurbs(s) => put_tagged(8, s, w),
        }
    }
    fn get(r: &mut Reader) -> Option<Surface> {
        Some(match u8::get(r)? {
            0 => Surface::Plane(Codec::get(r)?),
            1 => Surface::Cylinder(Codec::get(r)?),
            2 => Surface::Cone(Codec::get(r)?),
            3 => Surface::Sphere(Codec::get(r)?),
            4 => Surface::Torus(Codec::get(r)?),
            5 => Surface::Extrusion(Codec::get(r)?),
            6 => Surface::Revolution(Codec::get(r)?),
            7 => Surface::Ruled(Codec::get(r)?),
            8 => Surface::Nurbs(Codec::get(r)?),
            _ => return None,
        })
    }
}
