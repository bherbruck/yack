// yack: STEP geometry entities into kernel curves, surfaces and placements, in millimetres.
// Adapted from springrep@a03e8a1 src/step/geometry.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! STEP geometry entities into kernel curves, surfaces and placements, in millimetres.

use std::collections::{HashMap, HashSet};

use yack_p21::{Entity, File, Param};

use super::part21;
use crate::{
    Error, Result,
    geom::{
        Circle, Cone, Curve, Cylinder, Ellipse, Extrusion, Line, NurbsCurve, NurbsSurface, Plane,
        Revolution, Sphere, Surface, Torus,
    },
    math::{Frame, Point3, Transform, Vec3},
};

fn bad<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::InvalidInput(format!("STEP #{id}: {what}")))
}

fn unsupported<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::Unsupported(format!("STEP #{id}: {what}")))
}

/// The scale factors of a representation context: millimetres per length unit and radians per
/// angle unit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Units {
    pub length: f64,
    pub angle: f64,
}

impl Units {
    pub(super) const MM_RAD: Units = Units {
        length: 1.0,
        angle: 1.0,
    };
}

fn si_prefix(p: &Param) -> f64 {
    match p.enumeration() {
        Some("EXA") => 1e18,
        Some("PETA") => 1e15,
        Some("TERA") => 1e12,
        Some("GIGA") => 1e9,
        Some("MEGA") => 1e6,
        Some("KILO") => 1e3,
        Some("HECTO") => 1e2,
        Some("DECA") => 1e1,
        Some("DECI") => 1e-1,
        Some("CENTI") => 1e-2,
        Some("MILLI") => 1e-3,
        Some("MICRO") => 1e-6,
        Some("NANO") => 1e-9,
        Some("PICO") => 1e-12,
        _ => 1.0,
    }
}

/// A placement that may also scale, as a `CARTESIAN_TRANSFORMATION_OPERATOR_3D` can:
/// `x ↦ rigid(scale · x)`. The scale is applied by reading the solid in scaled units, so the
/// kernel's transforms stay rigid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Placing {
    pub rigid: Transform,
    pub scale: f64,
}

impl Placing {
    pub(super) const IDENTITY: Placing = Placing {
        rigid: Transform::IDENTITY,
        scale: 1.0,
    };

    pub(super) fn rigid(t: Transform) -> Placing {
        Placing {
            rigid: t,
            scale: 1.0,
        }
    }

    /// `self` followed by `next`.
    pub(super) fn then(&self, next: &Placing) -> Placing {
        let r = self.rigid.then(&next.rigid).r;
        Placing {
            rigid: Transform {
                r,
                t: next.rigid.vector(self.rigid.t * next.scale) + next.rigid.t,
            },
            scale: self.scale * next.scale,
        }
    }

    pub(super) fn inverse(&self) -> Placing {
        let inv = self.rigid.inverse();
        Placing {
            rigid: Transform {
                r: inv.r,
                t: inv.t / self.scale,
            },
            scale: 1.0 / self.scale,
        }
    }
}

/// Converts the geometry of one file, caching each entity's result.
pub(super) struct Geometry<'a> {
    pub file: &'a File,
    units: Units,
    points: HashMap<u64, Point3>,
    curves: HashMap<u64, Curve>,
    surfaces: HashMap<u64, Surface>,
    /// The distance within which the current context's geometry meets, in mm, where it says.
    uncertainty: Option<f64>,
    /// Instances being converted, so a reference cycle is an error rather than endless recursion.
    open: HashSet<u64>,
}

impl<'a> Geometry<'a> {
    pub(super) fn new(file: &'a File) -> Geometry<'a> {
        Geometry {
            file,
            units: Units::MM_RAD,
            points: HashMap::new(),
            curves: HashMap::new(),
            surfaces: HashMap::new(),
            uncertainty: None,
            open: HashSet::new(),
        }
    }

    /// The distance within which the current context says its geometry meets, in mm: its
    /// `distance_accuracy_value`, or the largest length uncertainty it gives.
    pub(super) fn uncertainty(&self) -> Option<f64> {
        self.uncertainty
    }

    /// Switches to the units of representation context `ctx`, clearing the caches if they change.
    pub(super) fn use_context(&mut self, ctx: Option<u64>) -> Result<()> {
        self.use_scaled(ctx, 1.0)
    }

    /// Switches to the units of `ctx` with every length times `scale`.
    pub(super) fn use_scaled(&mut self, ctx: Option<u64>, scale: f64) -> Result<()> {
        let mut units = match ctx {
            Some(c) => self.context_units(c)?,
            None => Units::MM_RAD,
        };
        units.length *= scale;
        self.uncertainty = match ctx {
            Some(c) => self.context_uncertainty(c).map(|u| u * scale),
            None => None,
        };
        if units != self.units {
            self.units = units;
            self.points.clear();
            self.curves.clear();
            self.surfaces.clear();
        }
        Ok(())
    }

    fn entity(&self, id: u64) -> Result<&'a Entity> {
        match self.file.get(id) {
            Some(e) => Ok(e),
            None => bad(id, "missing instance"),
        }
    }

    fn context_units(&self, ctx: u64) -> Result<Units> {
        let e = self.entity(ctx)?;
        let mut units = Units::MM_RAD;
        let Some(p) = e.part("GLOBAL_UNIT_ASSIGNED_CONTEXT") else {
            return Ok(units);
        };
        for u in p.first().and_then(Param::list).unwrap_or(&[]) {
            let Some(uid) = u.reference() else { continue };
            let ue = self.entity(uid)?;
            if ue.is("LENGTH_UNIT") {
                units.length = self.unit_factor(uid, 1000.0)?;
            } else if ue.is("PLANE_ANGLE_UNIT") {
                units.angle = self.unit_factor(uid, 1.0)?;
            }
        }
        Ok(units)
    }

    /// A context's length uncertainty in mm: the one named `distance_accuracy_value`, otherwise the
    /// largest given in a length unit.
    fn context_uncertainty(&self, ctx: u64) -> Option<f64> {
        let p = self
            .entity(ctx)
            .ok()?
            .part("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT")?;
        let mut named = None;
        let mut largest: Option<f64> = None;
        for u in p.first().and_then(Param::list).unwrap_or(&[]) {
            let Some(q) = u.reference().and_then(|id| self.file.get(id)) else {
                continue;
            };
            let Some(a) = q.part("UNCERTAINTY_MEASURE_WITH_UNIT") else {
                continue;
            };
            let (Some(v), Some(unit)) = (
                a.first().and_then(Param::real),
                a.get(1).and_then(Param::reference),
            ) else {
                continue;
            };
            if !self.entity(unit).is_ok_and(|e| e.is("LENGTH_UNIT")) {
                continue;
            }
            let Ok(mm) = self.unit_factor(unit, 1000.0) else {
                continue;
            };
            let d = v.abs() * mm;
            if !(d.is_finite() && d > 0.0) {
                continue;
            }
            if a.get(2)
                .and_then(Param::string)
                .is_some_and(|n| n.eq_ignore_ascii_case("distance_accuracy_value"))
            {
                named = Some(d);
            }
            largest = Some(largest.map_or(d, |l| l.max(d)));
        }
        named.or(largest)
    }

    /// The size of unit `id` in millimetres (lengths, `si_base` 1000) or radians (angles, 1).
    fn unit_factor(&self, id: u64, si_base: f64) -> Result<f64> {
        self.unit_factor_within(id, si_base, 8)
    }

    fn unit_factor_within(&self, id: u64, si_base: f64, depth: usize) -> Result<f64> {
        if depth == 0 {
            return bad(id, "conversion-based units nest too deeply");
        }
        let e = self.entity(id)?;
        if let Some(si) = e.part("SI_UNIT") {
            return Ok(si_base * si.first().map_or(1.0, si_prefix));
        }
        if let Some(cb) = e.part("CONVERSION_BASED_UNIT") {
            let Some(m) = cb.get(1).and_then(Param::reference) else {
                return bad(id, "conversion-based unit without a measure");
            };
            let me = self.entity(m)?;
            let p = &me.records()[0].params;
            let (Some(v), Some(base)) = (
                p.first().and_then(Param::real),
                p.get(1).and_then(Param::reference),
            ) else {
                return bad(m, "malformed unit measure");
            };
            return Ok(v * self.unit_factor_within(base, si_base, depth - 1)?);
        }
        bad(id, "unknown unit")
    }

    fn params(&self, id: u64, kind: &str) -> Result<&'a [Param]> {
        match self.entity(id)?.part(kind) {
            Some(p) => Ok(p),
            None => bad(id, &format!("expected {kind}")),
        }
    }

    fn reals(&self, id: u64, p: Option<&Param>) -> Result<Vec<f64>> {
        let Some(l) = p.and_then(Param::list) else {
            return bad(id, "expected a list of numbers");
        };
        l.iter()
            .map(|x| x.real().ok_or(()))
            .collect::<std::result::Result<_, _>>()
            .or_else(|_| bad(id, "expected numbers"))
    }

    /// Millimetres per length unit of the current context.
    pub(super) fn length_unit(&self) -> f64 {
        self.units.length
    }

    /// The numbers of a list parameter, as `reals` reads them.
    pub(super) fn numbers(&self, id: u64, p: Option<&Param>) -> Result<Vec<f64>> {
        self.reals(id, p)
    }

    pub(super) fn real(&self, id: u64, p: Option<&Param>) -> Result<f64> {
        match p.and_then(Param::real) {
            Some(x) => Ok(x),
            None => bad(id, "expected a number"),
        }
    }

    pub(super) fn reference(&self, id: u64, p: Option<&Param>) -> Result<u64> {
        match p.and_then(Param::reference) {
            Some(r) => Ok(r),
            None => bad(id, "expected an instance reference"),
        }
    }

    fn length(&self, id: u64, p: Option<&Param>) -> Result<f64> {
        Ok(self.real(id, p)? * self.units.length)
    }

    /// Runs `f` on instance `id` unless `id` is already being converted further up the stack.
    fn enter<T>(&mut self, id: u64, f: impl FnOnce(&mut Self) -> Result<T>) -> Result<T> {
        if !self.open.insert(id) {
            return bad(id, "refers to itself");
        }
        let out = f(self);
        self.open.remove(&id);
        out
    }

    pub(super) fn point(&mut self, id: u64) -> Result<Point3> {
        if let Some(&p) = self.points.get(&id) {
            return Ok(p);
        }
        let p = self.enter(id, |s| s.make_point(id))?;
        self.points.insert(id, p);
        Ok(p)
    }

    fn make_point(&mut self, id: u64) -> Result<Point3> {
        let e = self.entity(id)?;
        match e.part("CARTESIAN_POINT") {
            Some(a) => {
                let c = self.coords(id, a.get(1))?;
                Ok(c * self.units.length)
            }
            None => match e.part("VERTEX_POINT") {
                Some(a) => self.point(self.reference(id, a.get(1))?),
                None => bad(id, "expected a point"),
            },
        }
    }

    /// Three finite coordinates; a 2-D point or direction is an error here, never padded.
    fn coords(&self, id: u64, p: Option<&Param>) -> Result<Vec3> {
        let c = self.reals(id, p)?;
        if c.len() != 3 || c.iter().any(|x| !x.is_finite()) {
            return bad(id, "expected three finite coordinates");
        }
        Ok(Vec3::new(c[0], c[1], c[2]))
    }

    pub(super) fn direction(&self, id: u64) -> Result<Vec3> {
        let a = self.params(id, "DIRECTION")?;
        let v = self.coords(id, a.get(1))?;
        match v.unit() {
            Some(u) => Ok(u),
            None => bad(id, "zero direction"),
        }
    }

    /// A `VECTOR`'s direction and length.
    fn vector(&self, id: u64) -> Result<(Vec3, f64)> {
        let a = self.params(id, "VECTOR")?;
        let d = self.direction(self.reference(id, a.get(1))?)?;
        Ok((d, self.length(id, a.get(2))?))
    }

    pub(super) fn placement(&mut self, id: u64) -> Result<Frame> {
        let e = self.entity(id)?;
        let Some(a) = e.part("AXIS2_PLACEMENT_3D") else {
            return unsupported(id, &format!("placement {}", part21::kind(e)));
        };
        let origin = self.point(self.reference(id, a.get(1))?)?;
        let z = match a.get(2).and_then(Param::reference) {
            Some(r) => self.direction(r)?,
            None => Vec3::Z,
        };
        let x = match a.get(3).and_then(Param::reference) {
            Some(r) => self.direction(r)?,
            None if z.x.abs() < 0.9 => Vec3::X,
            None => Vec3::Y,
        };
        Frame::new(origin, z, x)
    }

    /// The motion of an `AXIS2_PLACEMENT_3D` or a `CARTESIAN_TRANSFORMATION_OPERATOR_3D`, which
    /// may also scale uniformly.
    pub(super) fn transform_of(&mut self, id: u64) -> Result<Placing> {
        let e = self.entity(id)?;
        if e.is("AXIS2_PLACEMENT_3D") {
            return Ok(Placing::rigid(self.placement(id)?.to_transform()));
        }
        if let Some(a) = e.part("CARTESIAN_TRANSFORMATION_OPERATOR_3D") {
            let dir = |s: &Self, i: usize, d: Vec3| -> Result<Vec3> {
                match a.get(i).and_then(Param::reference) {
                    Some(r) => s.direction(r),
                    None => Ok(d),
                }
            };
            let x = dir(self, 2, Vec3::X)?;
            let y = dir(self, 3, Vec3::Y)?;
            let origin = self.point(self.reference(id, a.get(4))?)?;
            let scale = a.get(5).and_then(Param::real).unwrap_or(1.0);
            if !(scale > 0.0 && scale.is_finite()) {
                return bad(id, "a transformation's scale must be positive");
            }
            let z = match a.get(6).and_then(Param::reference) {
                Some(r) => self.direction(r)?,
                None => x.cross(y),
            };
            if z.dot(x.cross(y)) < 0.0 {
                return unsupported(id, "mirroring transformation");
            }
            let f = Frame::new(origin, z, x)?;
            return Ok(Placing {
                rigid: f.to_transform(),
                scale,
            });
        }
        unsupported(id, &format!("transformation {}", part21::kind(e)))
    }

    pub(super) fn curve(&mut self, id: u64) -> Result<Curve> {
        if let Some(c) = self.curves.get(&id) {
            return Ok(c.clone());
        }
        let c = self.enter(id, |s| s.make_curve(id))?;
        self.curves.insert(id, c.clone());
        Ok(c)
    }

    fn make_curve(&mut self, id: u64) -> Result<Curve> {
        let e = self.entity(id)?;
        if e.is("B_SPLINE_CURVE_WITH_KNOTS") {
            return self.bspline_curve(id, e).map(|c| Curve::Nurbs(Box::new(c)));
        }
        let first = &e.records()[0];
        let (kind, a) = (&first.name, &first.params);
        match kind.as_str() {
            "LINE" => {
                let origin = self.point(self.reference(id, a.get(1))?)?;
                let (dir, _) = self.vector(self.reference(id, a.get(2))?)?;
                Ok(Curve::Line(Line { origin, dir }))
            }
            "CIRCLE" => Ok(Curve::Circle(Circle {
                frame: self.placement(self.reference(id, a.get(1))?)?,
                radius: self.length(id, a.get(2))?,
            })),
            "ELLIPSE" => Ok(Curve::Ellipse(Ellipse {
                frame: self.placement(self.reference(id, a.get(1))?)?,
                major: self.length(id, a.get(2))?,
                minor: self.length(id, a.get(3))?,
            })),
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" | "TRIMMED_CURVE" => {
                self.curve(self.reference(id, a.get(1))?)
            }
            other => unsupported(id, &format!("curve {other}")),
        }
    }

    fn knot_vector(
        &self,
        id: u64,
        mults: Option<&Param>,
        knots: Option<&Param>,
    ) -> Result<Vec<f64>> {
        let m = self.reals(id, mults)?;
        let k = self.reals(id, knots)?;
        if m.len() != k.len() {
            return bad(id, "knot and multiplicity counts differ");
        }
        let mut out = Vec::new();
        for (&mi, &ki) in m.iter().zip(&k) {
            if mi < 0.0 || mi.fract() != 0.0 {
                return bad(id, "bad knot multiplicity");
            }
            out.extend(std::iter::repeat_n(ki, mi as usize));
        }
        Ok(out)
    }

    fn degree(&self, id: u64, p: Option<&Param>) -> Result<usize> {
        match p.and_then(part21::integral) {
            Some(d) if d > 0 => Ok(d as usize),
            _ => bad(id, "bad B-spline degree"),
        }
    }

    fn bspline_curve(&mut self, id: u64, e: &Entity) -> Result<NurbsCurve> {
        let k = self.params(id, "B_SPLINE_CURVE_WITH_KNOTS")?;
        // A simple instance carries name, degree, points, form, closed, self-intersect, then the
        // knot attributes; a complex one splits them between B_SPLINE_CURVE and _WITH_KNOTS.
        let (head, knots): (&[Param], &[Param]) = match e.part("B_SPLINE_CURVE") {
            Some(h) => (h, k),
            None if k.len() >= 9 => (&k[1..6], &k[6..]),
            None => return bad(id, "short B_SPLINE_CURVE_WITH_KNOTS"),
        };
        let degree = self.degree(id, head.first())?;
        let mut ctrl = Vec::new();
        for p in head.get(1).and_then(Param::list).unwrap_or(&[]) {
            ctrl.push(self.point(self.reference(id, Some(p))?)?);
        }
        let knots = self.knot_vector(id, knots.first(), knots.get(1))?;
        let weights = match e.part("RATIONAL_B_SPLINE_CURVE") {
            Some(w) => Some(self.reals(id, w.first())?),
            None => None,
        };
        NurbsCurve::new(degree, knots, ctrl, weights).map_err(|err| at(id, err))
    }

    pub(super) fn surface(&mut self, id: u64) -> Result<Surface> {
        if let Some(s) = self.surfaces.get(&id) {
            return Ok(s.clone());
        }
        let s = self.enter(id, |g| g.make_surface(id))?;
        self.surfaces.insert(id, s.clone());
        Ok(s)
    }

    fn make_surface(&mut self, id: u64) -> Result<Surface> {
        let e = self.entity(id)?;
        if e.is("B_SPLINE_SURFACE_WITH_KNOTS") {
            return self
                .bspline_surface(id, e)
                .map(|s| Surface::Nurbs(Box::new(s)));
        }
        let first = &e.records()[0];
        let (kind, a) = (&first.name, &first.params);
        let frame = |s: &mut Self| -> Result<Frame> { s.placement(s.reference(id, a.get(1))?) };
        match kind.as_str() {
            "PLANE" => Ok(Surface::Plane(Plane {
                frame: frame(self)?,
            })),
            "CYLINDRICAL_SURFACE" => Ok(Surface::Cylinder(Cylinder {
                frame: frame(self)?,
                radius: self.length(id, a.get(2))?,
            })),
            "CONICAL_SURFACE" => Ok(Surface::Cone(Cone {
                frame: frame(self)?,
                radius: self.length(id, a.get(2))?,
                half_angle: self.real(id, a.get(3))? * self.units.angle,
            })),
            "SPHERICAL_SURFACE" => Ok(Surface::Sphere(Sphere {
                frame: frame(self)?,
                radius: self.length(id, a.get(2))?,
            })),
            "TOROIDAL_SURFACE" => Ok(Surface::Torus(Torus {
                frame: frame(self)?,
                major: self.length(id, a.get(2))?,
                minor: self.length(id, a.get(3))?,
            })),
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                let curve = self.curve(self.reference(id, a.get(1))?)?;
                let (dir, _) = self.vector(self.reference(id, a.get(2))?)?;
                Ok(Surface::Extrusion(Box::new(Extrusion { curve, dir })))
            }
            "SURFACE_OF_REVOLUTION" => {
                let curve = self.curve(self.reference(id, a.get(1))?)?;
                let ax = self.reference(id, a.get(2))?;
                let p = self.params(ax, "AXIS1_PLACEMENT")?;
                let origin = self.point(self.reference(ax, p.get(1))?)?;
                let axis = match p.get(2).and_then(Param::reference) {
                    Some(r) => self.direction(r)?,
                    None => Vec3::Z,
                };
                Ok(Surface::Revolution(Box::new(Revolution {
                    curve,
                    origin,
                    axis,
                })))
            }
            "RECTANGULAR_TRIMMED_SURFACE" => self.surface(self.reference(id, a.get(1))?),
            "OFFSET_SURFACE" => {
                let basis = self.surface(self.reference(id, a.get(1))?)?;
                let d = self.length(id, a.get(2))?;
                offset(id, basis, d)
            }
            other => unsupported(id, &format!("surface {other}")),
        }
    }

    fn bspline_surface(&mut self, id: u64, e: &Entity) -> Result<NurbsSurface> {
        let k = self.params(id, "B_SPLINE_SURFACE_WITH_KNOTS")?;
        let (head, knots): (&[Param], &[Param]) = match e.part("B_SPLINE_SURFACE") {
            Some(h) => (h, k),
            None if k.len() >= 13 => (&k[1..8], &k[8..]),
            None => return bad(id, "short B_SPLINE_SURFACE_WITH_KNOTS"),
        };
        let du = self.degree(id, head.first())?;
        let dv = self.degree(id, head.get(1))?;
        let rows = head.get(2).and_then(Param::list).unwrap_or(&[]);
        let nu = rows.len();
        let nv = rows
            .first()
            .and_then(Param::list)
            .map_or(0, <[Param]>::len);
        let mut ctrl = Vec::with_capacity(nu * nv);
        for r in rows {
            let r = r.list().unwrap_or(&[]);
            if r.len() != nv {
                return bad(id, "ragged control net");
            }
            for p in r {
                ctrl.push(self.point(self.reference(id, Some(p))?)?);
            }
        }
        let ku = self.knot_vector(id, knots.first(), knots.get(2))?;
        let kv = self.knot_vector(id, knots.get(1), knots.get(3))?;
        let weights = match e.part("RATIONAL_B_SPLINE_SURFACE") {
            Some(w) => {
                let mut out = Vec::with_capacity(nu * nv);
                for r in w.first().and_then(Param::list).unwrap_or(&[]) {
                    out.extend(self.reals(id, Some(r))?);
                }
                Some(out)
            }
            None => None,
        };
        NurbsSurface::new((du, dv), (ku, kv), (nu, nv), ctrl, weights).map_err(|err| at(id, err))
    }
}

/// `basis` moved by `d` along its normal, where that is a surface of the same kind; a positive
/// radius must stay positive.
fn offset(id: u64, basis: Surface, d: f64) -> Result<Surface> {
    let grown = |r: f64| -> Result<f64> {
        if r + d > 0.0 {
            Ok(r + d)
        } else {
            bad(id, "an offset surface turns inside out")
        }
    };
    Ok(match basis {
        Surface::Plane(p) => Surface::Plane(Plane {
            frame: p.frame.translated(p.frame.z * d),
        }),
        Surface::Cylinder(c) => Surface::Cylinder(Cylinder {
            radius: grown(c.radius)?,
            ..c
        }),
        Surface::Sphere(s) => Surface::Sphere(Sphere {
            radius: grown(s.radius)?,
            ..s
        }),
        Surface::Torus(t) => Surface::Torus(Torus {
            minor: grown(t.minor)?,
            ..t
        }),
        Surface::Cone(c) => {
            let (sa, ca) = c.half_angle.sin_cos();
            Surface::Cone(Cone {
                frame: c.frame.translated(c.frame.z * (-d * sa)),
                radius: c.radius + d * ca,
                half_angle: c.half_angle,
            })
        }
        other => {
            return unsupported(id, &format!("OFFSET_SURFACE of a {} surface", other.kind()));
        }
    })
}

fn at(id: u64, e: Error) -> Error {
    match e {
        Error::InvalidInput(m) => Error::InvalidInput(format!("STEP #{id}: {m}")),
        other => other,
    }
}
