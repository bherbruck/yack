// yack: a STEP file's solids read as ISO 10303-42 defines them: shells, faces, loops and edge uses.
//! From the entity graph to what the measurer needs and nothing more: each face's surface and
//! `same_sense`, each bound's edge uses in the order and direction the standard says they run, and
//! each edge's 3-D curve with the parameter range between its vertices. Pcurves are not read: the
//! standard makes them optional, and the edges' own curves are the geometry.

use std::f64::consts::PI;

use yack_base::{Error, Result, Stage};
use yack_math::{Point3, Vec3};
use yack_p21::{Entity, File, Param};

use crate::bspline;
use crate::geom::{Curve, Frame, Surface};

pub(crate) const OP: &str = "step::measure";

pub(crate) fn bad(what: impl Into<String>) -> Error {
    Error::failed(OP, Stage::Parse, what.into())
}

/// One edge as a bound uses it: its curve run from `t0` to `t1` (either way round).
#[derive(Clone, Debug)]
pub(crate) struct Use {
    pub(crate) curve: Curve,
    pub(crate) t0: f64,
    pub(crate) t1: f64,
    /// The edge's pcurves on this face's surface, as 2-D curves (z = 0) in its (u, v). They do not
    /// define the face, but they must lie on the edge: a reader may take them for its geometry.
    pub(crate) pcurves: Vec<Curve>,
}

/// A face bound: its edge uses in order, or (for a `vertex_loop`) none.
#[derive(Clone, Debug)]
pub(crate) struct Bound {
    pub(crate) uses: Vec<Use>,
}

/// A face as the measurer sees it.
#[derive(Clone, Debug)]
pub(crate) struct Face {
    /// The face's instance number.
    pub(crate) id: u64,
    pub(crate) surface: Surface,
    /// `same_sense`: the face's normal is the surface's (true) or its reverse.
    pub(crate) same_sense: bool,
    /// +1, or -1 for a face of a shell used reversed (a void's `oriented_closed_shell`).
    pub(crate) shell_sign: f64,
    pub(crate) bounds: Vec<Bound>,
}

/// Every face of every solid in the file, and the points of its vertices.
#[derive(Clone, Debug, Default)]
pub(crate) struct Model {
    pub(crate) faces: Vec<Face>,
    pub(crate) vertices: Vec<Point3>,
}

/// Reads every `manifold_solid_brep` (and `brep_with_voids`) in `file`.
pub(crate) fn model(file: &File) -> Result<Model> {
    let r = Reader { f: file, angle: angle_unit(file) };
    let mut m = Model::default();
    let mut solids = 0;
    for (&id, e) in &file.entities {
        let Some(p) = e.part("MANIFOLD_SOLID_BREP").or_else(|| e.part("BREP_WITH_VOIDS")) else { continue };
        solids += 1;
        r.shell(ref_at(p, 1, id)?, 1.0, 0, &mut m)?;
        if e.is("BREP_WITH_VOIDS") {
            for v in list_at(p, 2, id)? {
                r.shell(v.reference().ok_or_else(|| bad(format!("#{id}: a void is not a reference")))?, 1.0, 0, &mut m)?;
            }
        }
    }
    if solids == 0 {
        return Err(bad("the file holds no manifold_solid_brep"));
    }
    Ok(m)
}

/// Radians per unit of plane angle: 1, or the conversion factor of a `conversion_based_unit`
/// (π/180 for degrees).
fn angle_unit(file: &File) -> f64 {
    for e in file.entities.values() {
        if !e.is("PLANE_ANGLE_UNIT") {
            continue;
        }
        if let Some(c) = e.part("CONVERSION_BASED_UNIT") {
            let factor = c.get(1).and_then(Param::reference).and_then(|m| file.get(m)).and_then(|m| {
                m.records().iter().find_map(|r| if r.name.ends_with("MEASURE_WITH_UNIT") { r.params.first().and_then(Param::real) } else { None })
            });
            return match factor {
                Some(f) if f > 0.0 => f,
                _ => PI / 180.0,
            };
        }
        return 1.0;
    }
    1.0
}

/// How deep references may nest through oriented edges, oriented shells and surface curves before
/// the file is refused (a cycle would otherwise recurse without end).
const MAX_DEPTH: usize = 64;

fn deeper(depth: usize, id: u64) -> Result<usize> {
    if depth >= MAX_DEPTH { Err(bad(format!("#{id}: references nest deeper than {MAX_DEPTH} (a cycle?)"))) } else { Ok(depth + 1) }
}

/// A B-spline degree: an integer from 1 to `bspline::MAX_DEGREE`.
fn degree_at(p: &[Param], i: usize, id: u64) -> Result<usize> {
    match arg(p, i, id)?.int() {
        Some(d) if d >= 1 && d <= bspline::MAX_DEGREE as i64 => Ok(d as usize),
        other => Err(bad(format!("#{id}: degree {other:?} is not an integer from 1 to {}", bspline::MAX_DEGREE))),
    }
}

fn arg(p: &[Param], i: usize, id: u64) -> Result<&Param> {
    p.get(i).ok_or_else(|| bad(format!("#{id}: missing parameter {i}")))
}

fn ref_at(p: &[Param], i: usize, id: u64) -> Result<u64> {
    arg(p, i, id)?.reference().ok_or_else(|| bad(format!("#{id}: parameter {i} is not a reference")))
}

fn real_at(p: &[Param], i: usize, id: u64) -> Result<f64> {
    arg(p, i, id)?.real().ok_or_else(|| bad(format!("#{id}: parameter {i} is not a number")))
}

fn bool_at(p: &[Param], i: usize, id: u64) -> Result<bool> {
    arg(p, i, id)?.boolean().ok_or_else(|| bad(format!("#{id}: parameter {i} is not .T. or .F.")))
}

fn list_at(p: &[Param], i: usize, id: u64) -> Result<&[Param]> {
    arg(p, i, id)?.list().ok_or_else(|| bad(format!("#{id}: parameter {i} is not a list")))
}

fn reals(p: &Param, id: u64) -> Result<Vec<f64>> {
    p.list().ok_or_else(|| bad(format!("#{id}: expected a list of numbers")))?.iter().map(|x| x.real().ok_or_else(|| bad(format!("#{id}: not a number")))).collect()
}

fn ints(p: &Param, id: u64) -> Result<Vec<i64>> {
    p.list().ok_or_else(|| bad(format!("#{id}: expected a list of integers")))?.iter().map(|x| x.int().ok_or_else(|| bad(format!("#{id}: not an integer")))).collect()
}

fn unit(v: Vec3, id: u64) -> Result<Vec3> {
    let l = v.length();
    if l > 0.0 && l.is_finite() { Ok(v * (1.0 / l)) } else { Err(bad(format!("#{id}: a direction of length {l}"))) }
}

struct Reader<'a> {
    f: &'a File,
    angle: f64,
}

impl Reader<'_> {
    fn entity(&self, id: u64) -> Result<&Entity> {
        self.f.get(id).ok_or_else(|| bad(format!("#{id} is referenced but not defined")))
    }

    /// The name and parameters of simple instance `#id`.
    fn simple(&self, id: u64) -> Result<(&str, &[Param])> {
        match self.entity(id)? {
            Entity::Simple(r) => Ok((r.name.as_str(), r.params.as_slice())),
            Entity::Complex(_) => Err(bad(format!("#{id}: a complex instance where a simple one is expected"))),
        }
    }

    fn shell(&self, id: u64, sign: f64, depth: usize, m: &mut Model) -> Result<()> {
        let (name, p) = self.simple(id)?;
        match name {
            "CLOSED_SHELL" | "OPEN_SHELL" => {
                for f in list_at(p, 1, id)? {
                    let fid = f.reference().ok_or_else(|| bad(format!("#{id}: a face is not a reference")))?;
                    let face = self.face(fid, sign, m)?;
                    m.faces.push(face);
                }
                Ok(())
            }
            "ORIENTED_CLOSED_SHELL" | "ORIENTED_OPEN_SHELL" => {
                let s = if bool_at(p, 3, id)? { sign } else { -sign };
                self.shell(ref_at(p, 2, id)?, s, deeper(depth, id)?, m)
            }
            other => Err(bad(format!("#{id}: {other} is not a shell"))),
        }
    }

    fn face(&self, id: u64, shell_sign: f64, m: &mut Model) -> Result<Face> {
        let (name, p) = self.simple(id)?;
        if name != "ADVANCED_FACE" && name != "FACE_SURFACE" {
            return Err(bad(format!("#{id}: {name} is not a face with a surface")));
        }
        let surface = self.surface(ref_at(p, 2, id)?)?;
        let same_sense = bool_at(p, 3, id)?;
        let mut bounds = Vec::new();
        for b in list_at(p, 1, id)? {
            let bid = b.reference().ok_or_else(|| bad(format!("#{id}: a bound is not a reference")))?;
            bounds.push(self.bound(bid, ref_at(p, 2, id)?, m)?);
        }
        Ok(Face { id, surface, same_sense, shell_sign, bounds })
    }

    fn bound(&self, id: u64, surf: u64, m: &mut Model) -> Result<Bound> {
        let (name, p) = self.simple(id)?;
        if name != "FACE_BOUND" && name != "FACE_OUTER_BOUND" {
            return Err(bad(format!("#{id}: {name} is not a face bound")));
        }
        let forward = bool_at(p, 2, id)?;
        let lid = ref_at(p, 1, id)?;
        let (lname, lp) = self.simple(lid)?;
        let mut uses = match lname {
            "EDGE_LOOP" => {
                let mut uses = Vec::new();
                for e in list_at(lp, 1, lid)? {
                    let eid = e.reference().ok_or_else(|| bad(format!("#{lid}: an edge is not a reference")))?;
                    uses.push(self.oriented_edge(eid, true, surf, 0, m)?);
                }
                uses
            }
            "VERTEX_LOOP" => Vec::new(),
            other => return Err(bad(format!("#{lid}: {other} is not a loop yack measures"))),
        };
        if !forward {
            uses.reverse();
            for u in &mut uses {
                std::mem::swap(&mut u.t0, &mut u.t1);
            }
        }
        Ok(Bound { uses })
    }

    fn oriented_edge(&self, id: u64, forward: bool, surf: u64, depth: usize, m: &mut Model) -> Result<Use> {
        let (name, p) = self.simple(id)?;
        match name {
            "ORIENTED_EDGE" => {
                let f = if bool_at(p, 4, id)? { forward } else { !forward };
                self.oriented_edge(ref_at(p, 3, id)?, f, surf, deeper(depth, id)?, m)
            }
            "EDGE_CURVE" => {
                let (a, b) = (ref_at(p, 1, id)?, ref_at(p, 2, id)?);
                let (pa, pb) = (self.vertex(a)?, self.vertex(b)?);
                m.vertices.extend([pa, pb]);
                let cid = ref_at(p, 3, id)?;
                let curve = self.curve(cid)?;
                let pcurves = self.pcurves(cid, surf)?;
                let sense = bool_at(p, 4, id)?;
                let (ta, tb) = range(&curve, pa, pb, a == b, sense).map_err(|e| bad(format!("#{id}: {e}")))?;
                let (t0, t1) = if forward { (ta, tb) } else { (tb, ta) };
                Ok(Use { curve, t0, t1, pcurves })
            }
            other => Err(bad(format!("#{id}: {other} is not an edge"))),
        }
    }

    /// The 2-D curves of the `pcurve`s of a `surface_curve` (or `seam_curve`) `#cid` that lie on
    /// surface `#surf`; none for a bare curve.
    fn pcurves(&self, cid: u64, surf: u64) -> Result<Vec<Curve>> {
        let Ok((name, p)) = self.simple(cid) else { return Ok(Vec::new()) };
        if !matches!(name, "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE") {
            return Ok(Vec::new());
        }
        let mut out = Vec::new();
        for g in list_at(p, 2, cid)? {
            let Some(gid) = g.reference() else { continue };
            let Ok(("PCURVE", gp)) = self.simple(gid) else { continue };
            if ref_at(gp, 1, gid)? != surf {
                continue;
            }
            let rid = ref_at(gp, 2, gid)?;
            let (_, rp) = self.simple(rid)?;
            if let Some(c) = list_at(rp, 1, rid)?.first().and_then(Param::reference) {
                out.push(self.curve(c)?);
            }
        }
        Ok(out)
    }

    fn vertex(&self, id: u64) -> Result<Point3> {
        let (name, p) = self.simple(id)?;
        if name != "VERTEX_POINT" {
            return Err(bad(format!("#{id}: {name} is not a vertex_point")));
        }
        self.point(ref_at(p, 1, id)?)
    }

    fn point(&self, id: u64) -> Result<Point3> {
        let (name, p) = self.simple(id)?;
        if name != "CARTESIAN_POINT" {
            return Err(bad(format!("#{id}: {name} is not a cartesian_point")));
        }
        let c = reals(arg(p, 1, id)?, id)?;
        match c.as_slice() {
            [x, y, z] => Ok(Point3::new(*x, *y, *z)),
            [x, y] => Ok(Point3::new(*x, *y, 0.0)),
            _ => Err(bad(format!("#{id}: a point of {} coordinates", c.len()))),
        }
    }

    fn direction(&self, id: u64) -> Result<Vec3> {
        let (name, p) = self.simple(id)?;
        if name != "DIRECTION" {
            return Err(bad(format!("#{id}: {name} is not a direction")));
        }
        let c = reals(arg(p, 1, id)?, id)?;
        match c.as_slice() {
            [x, y, z] => unit(Vec3::new(*x, *y, *z), id),
            [x, y] => unit(Vec3::new(*x, *y, 0.0), id),
            _ => Err(bad(format!("#{id}: a direction of {} ratios", c.len()))),
        }
    }

    fn vector(&self, id: u64) -> Result<Vec3> {
        let (name, p) = self.simple(id)?;
        if name != "VECTOR" {
            return Err(bad(format!("#{id}: {name} is not a vector")));
        }
        Ok(self.direction(ref_at(p, 1, id)?)? * real_at(p, 2, id)?)
    }

    /// An `axis2_placement_3d` as ISO 10303-42 builds it: z the axis (default +Z), x the reference
    /// direction made perpendicular to it (default +X), y = z × x.
    fn placement(&self, id: u64) -> Result<Frame> {
        let (name, p) = self.simple(id)?;
        if name == "AXIS2_PLACEMENT_2D" {
            // In a parameter plane: x the reference direction (default +X), z out of the plane.
            let o = self.point(ref_at(p, 1, id)?)?;
            let x = match arg(p, 2, id)?.reference() {
                Some(d) => self.direction(d)?,
                None => Vec3::new(1.0, 0.0, 0.0),
            };
            let z = Vec3::new(0.0, 0.0, 1.0);
            return Ok(Frame { o, x, y: z.cross(x), z });
        }
        if name != "AXIS2_PLACEMENT_3D" {
            return Err(bad(format!("#{id}: {name} is not an axis2_placement_3d")));
        }
        let o = self.point(ref_at(p, 1, id)?)?;
        let z = match arg(p, 2, id)?.reference() {
            Some(d) => self.direction(d)?,
            None => Vec3::new(0.0, 0.0, 1.0),
        };
        let r = match arg(p, 3, id)?.reference() {
            Some(d) => self.direction(d)?,
            None => Vec3::new(1.0, 0.0, 0.0),
        };
        let mut x = r - z * r.dot(z);
        if x.length() <= REF_PARALLEL {
            // The standard's fallback: the first of +X, +Y not along the axis.
            let alt = if z.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
            x = alt - z * alt.dot(z);
        }
        let x = unit(x, id)?;
        Ok(Frame { o, x, y: z.cross(x), z })
    }

    fn curve(&self, id: u64) -> Result<Curve> {
        self.curve_in(id, 0)
    }

    fn curve_in(&self, id: u64, depth: usize) -> Result<Curve> {
        if let Entity::Complex(_) = self.entity(id)? {
            return self.complex_curve(id);
        }
        let (name, p) = self.simple(id)?;
        match name {
            "LINE" => Ok(Curve::Line { p: self.point(ref_at(p, 1, id)?)?, v: self.vector(ref_at(p, 2, id)?)? }),
            "CIRCLE" => Ok(Curve::Circle { f: self.placement(ref_at(p, 1, id)?)?, r: real_at(p, 2, id)? }),
            "ELLIPSE" => Ok(Curve::Ellipse { f: self.placement(ref_at(p, 1, id)?)?, a: real_at(p, 2, id)?, b: real_at(p, 3, id)? }),
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" => self.curve_in(ref_at(p, 1, id)?, deeper(depth, id)?),
            "B_SPLINE_CURVE_WITH_KNOTS" => {
                let degree = degree_at(p, 1, id)?;
                let ctrl = self.points(list_at(p, 2, id)?, id)?;
                let n = ctrl.len();
                self.spline_curve(id, degree, ctrl, &ints(arg(p, 6, id)?, id)?, &reals(arg(p, 7, id)?, id)?, vec![1.0; n])
            }
            other => Err(bad(format!("#{id}: {other} is not a curve yack measures"))),
        }
    }

    fn complex_curve(&self, id: u64) -> Result<Curve> {
        let e = self.entity(id)?;
        let (Some(b), Some(k)) = (e.part("B_SPLINE_CURVE"), e.part("B_SPLINE_CURVE_WITH_KNOTS")) else {
            return Err(bad(format!("#{id}: a complex curve that is not a knotted B-spline")));
        };
        let degree = degree_at(b, 0, id)?;
        let ctrl = self.points(list_at(b, 1, id)?, id)?;
        let weights = match e.part("RATIONAL_B_SPLINE_CURVE") {
            Some(r) => reals(arg(r, 0, id)?, id)?,
            None => vec![1.0; ctrl.len()],
        };
        self.spline_curve(id, degree, ctrl, &ints(arg(k, 0, id)?, id)?, &reals(arg(k, 1, id)?, id)?, weights)
    }

    fn spline_curve(&self, id: u64, degree: usize, ctrl: Vec<Point3>, mults: &[i64], knots: &[f64], weights: Vec<f64>) -> Result<Curve> {
        let basis = bspline::Basis::from_multiplicities(degree, mults, knots, ctrl.len())
            .ok_or_else(|| bad(format!("#{id}: knots that do not fit degree {degree} and {} control points", ctrl.len())))?;
        if basis.count() != ctrl.len() || weights.len() != ctrl.len() {
            return Err(bad(format!("#{id}: {} control points, {} weights for {} basis functions", ctrl.len(), weights.len(), basis.count())));
        }
        let mut c = bspline::Curve { basis, ctrl, weights, period: None };
        let (a, b) = c.basis.domain();
        let size = c.ctrl.iter().map(|p| p.to_vec().length()).fold(1.0, f64::max);
        if b > a && c.d1(a).0.distance(c.d1(b).0) <= CLOSES * size {
            if !long_enough(a, b) {
                return Err(bad(format!("#{id}: a closed spline whose period {} is too small", b - a)));
            }
            c.period = Some(b - a);
        }
        Ok(Curve::Spline(c))
    }

    fn points(&self, list: &[Param], id: u64) -> Result<Vec<Point3>> {
        list.iter().map(|q| self.point(q.reference().ok_or_else(|| bad(format!("#{id}: a control point is not a reference")))?)).collect()
    }

    fn surface(&self, id: u64) -> Result<Surface> {
        if let Entity::Complex(_) = self.entity(id)? {
            return self.complex_surface(id);
        }
        let (name, p) = self.simple(id)?;
        let f = || self.placement(ref_at(p, 1, id)?);
        match name {
            "PLANE" => Ok(Surface::Plane { f: f()? }),
            "CYLINDRICAL_SURFACE" => Ok(Surface::Cylinder { f: f()?, r: real_at(p, 2, id)? }),
            "CONICAL_SURFACE" => Ok(Surface::Cone { f: f()?, r: real_at(p, 2, id)?, tan: (real_at(p, 3, id)? * self.angle).tan() }),
            "SPHERICAL_SURFACE" => Ok(Surface::Sphere { f: f()?, r: real_at(p, 2, id)? }),
            "TOROIDAL_SURFACE" => Ok(Surface::Torus { f: f()?, rmaj: real_at(p, 2, id)?, rmin: real_at(p, 3, id)? }),
            "SURFACE_OF_LINEAR_EXTRUSION" => Ok(Surface::Extrusion { c: self.curve(ref_at(p, 1, id)?)?, d: self.vector(ref_at(p, 2, id)?)? }),
            "SURFACE_OF_REVOLUTION" => {
                let aid = ref_at(p, 2, id)?;
                let (aname, ap) = self.simple(aid)?;
                if aname != "AXIS1_PLACEMENT" {
                    return Err(bad(format!("#{aid}: {aname} is not an axis1_placement")));
                }
                let o = self.point(ref_at(ap, 1, aid)?)?;
                let axis = match arg(ap, 2, aid)?.reference() {
                    Some(d) => self.direction(d)?,
                    None => Vec3::new(0.0, 0.0, 1.0),
                };
                Ok(Surface::Revolution { c: self.curve(ref_at(p, 1, id)?)?, o, axis })
            }
            "B_SPLINE_SURFACE_WITH_KNOTS" => {
                let (du, dv) = (degree_at(p, 1, id)?, degree_at(p, 2, id)?);
                let ctrl = self.grid(list_at(p, 3, id)?, id)?;
                let weights = ctrl.iter().map(|r| vec![1.0; r.len()]).collect();
                let k = |i| arg(p, i, id);
                self.spline_surface(id, (du, dv), ctrl, weights, [&ints(k(8)?, id)?, &ints(k(9)?, id)?], [&reals(k(10)?, id)?, &reals(k(11)?, id)?])
            }
            other => Err(bad(format!("#{id}: {other} is not a surface yack measures"))),
        }
    }

    fn complex_surface(&self, id: u64) -> Result<Surface> {
        let e = self.entity(id)?;
        let (Some(b), Some(k)) = (e.part("B_SPLINE_SURFACE"), e.part("B_SPLINE_SURFACE_WITH_KNOTS")) else {
            return Err(bad(format!("#{id}: a complex surface that is not a knotted B-spline")));
        };
        let (du, dv) = (degree_at(b, 0, id)?, degree_at(b, 1, id)?);
        let ctrl = self.grid(list_at(b, 2, id)?, id)?;
        let weights = match e.part("RATIONAL_B_SPLINE_SURFACE") {
            Some(r) => list_at(r, 0, id)?.iter().map(|row| reals(row, id)).collect::<Result<Vec<_>>>()?,
            None => ctrl.iter().map(|r| vec![1.0; r.len()]).collect(),
        };
        let a = |i| arg(k, i, id);
        self.spline_surface(id, (du, dv), ctrl, weights, [&ints(a(0)?, id)?, &ints(a(1)?, id)?], [&reals(a(2)?, id)?, &reals(a(3)?, id)?])
    }

    fn grid(&self, rows: &[Param], id: u64) -> Result<Vec<Vec<Point3>>> {
        rows.iter().map(|r| self.points(r.list().ok_or_else(|| bad(format!("#{id}: a control row is not a list")))?, id)).collect()
    }

    #[allow(clippy::too_many_arguments)]
    fn spline_surface(&self, id: u64, (du, dv): (usize, usize), ctrl: Vec<Vec<Point3>>, weights: Vec<Vec<f64>>, mults: [&[i64]; 2], knots: [&[f64]; 2]) -> Result<Surface> {
        let (rows, cols) = (ctrl.len(), ctrl.first().map_or(0, Vec::len));
        let ub = bspline::Basis::from_multiplicities(du, mults[0], knots[0], rows).ok_or_else(|| bad(format!("#{id}: u knots that do not fit degree {du} and {rows} rows")))?;
        let vb = bspline::Basis::from_multiplicities(dv, mults[1], knots[1], cols).ok_or_else(|| bad(format!("#{id}: v knots that do not fit degree {dv} and {cols} columns")))?;
        let rows_ok = ctrl.len() == ub.count() && ctrl.iter().all(|r| r.len() == vb.count());
        let w_ok = weights.len() == ctrl.len() && weights.iter().zip(&ctrl).all(|(w, r)| w.len() == r.len());
        if !rows_ok || !w_ok {
            return Err(bad(format!("#{id}: the control net does not match the knots ({} × {} basis functions)", ub.count(), vb.count())));
        }
        let mut s = bspline::Surface { u: ub, v: vb, ctrl, weights, periods: [None, None] };
        let size = s.ctrl.iter().flatten().map(|p| p.to_vec().length()).fold(1.0, f64::max);
        let ((u0, u1), (v0, v1)) = s.domain();
        let samples = |i: usize, n: usize, a: f64, b: f64| a + (b - a) * i as f64 / n as f64;
        let closes_u = u1 > u0 && (0..=8).all(|i| {
            let v = samples(i, 8, v0, v1);
            s.d1(u0, v).0.distance(s.d1(u1, v).0) <= CLOSES * size
        });
        let closes_v = v1 > v0 && (0..=8).all(|i| {
            let u = samples(i, 8, u0, u1);
            s.d1(u, v0).0.distance(s.d1(u, v1).0) <= CLOSES * size
        });
        if (closes_u && !long_enough(u0, u1)) || (closes_v && !long_enough(v0, v1)) {
            return Err(bad(format!("#{id}: a closed spline surface whose period is too small")));
        }
        s.periods = [closes_u.then_some(u1 - u0), closes_v.then_some(v1 - v0)];
        Ok(Surface::Spline(s))
    }
}

/// A reference direction this close to the axis (as a unit-vector remainder) is along it.
const REF_PARALLEL: f64 = 1e-12; // tol: a direction's remainder after removing the axis

/// The shortest domain, relative to its parameters' size, a closed spline may have: a shorter
/// period would cut parameter runs into endless pieces, so such a file is refused.
const MIN_PERIOD: f64 = 1e-9; // tol: relative length of a spline's domain

fn long_enough(a: f64, b: f64) -> bool {
    b - a >= MIN_PERIOD * a.abs().max(b.abs()).max(1.0)
}

/// A spline whose ends are this close, relative to its size, closes on itself.
const CLOSES: f64 = 1e-9; // tol: relative coincidence of a spline's two ends

/// The parameters at the edge's start and end vertices, in the order the edge runs (start to end):
/// increasing along the curve where `sense` is true, decreasing where false. A closed curve runs
/// the whole period between equal vertices.
fn range(c: &Curve, a: Point3, b: Point3, same_vertex: bool, sense: bool) -> std::result::Result<(f64, f64), String> {
    let ta = c.invert(a, None);
    match c.period() {
        Some(p) => {
            let tb = c.invert(b, Some(ta));
            let ahead = (tb - ta).rem_euclid(p);
            let span = if sense { ahead } else { (p - ahead).rem_euclid(p) };
            let span = if same_vertex { p } else { span };
            Ok(if sense { (ta, ta + span) } else { (ta, ta - span) })
        }
        None => {
            let tb = c.invert(b, None);
            if (tb >= ta) == sense || ta == tb {
                Ok((ta, tb))
            } else {
                Err(format!("its vertices lie at curve parameters {ta} and {tb}, against its same_sense {sense}"))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use super::*;

    #[test]
    fn a_circle_edge_runs_the_way_its_sense_says() {
        let f = Frame { o: Point3::ORIGIN, x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 1.0, 0.0), z: Vec3::new(0.0, 0.0, 1.0) };
        let c = Curve::Circle { f, r: 1.0 };
        let (a, b) = (Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0));
        let (t0, t1) = range(&c, a, b, false, true).unwrap();
        assert!((t1 - t0 - PI / 2.0).abs() < 1e-12);
        let (t0, t1) = range(&c, a, b, false, false).unwrap();
        assert!((t0 - t1 - 1.5 * PI).abs() < 1e-12);
        let (t0, t1) = range(&c, a, a, true, false).unwrap();
        assert!((t0 - t1 - TAU).abs() < 1e-12);
    }
}
