// yack: yack profiles (plain data) as cadrs profiles, and the extrude and revolve rungs over them.
//! yack's [`Seg`]ments arrive as plain numbers; each becomes a cadrs [`Curve2`] whose `source` is
//! its index through the whole profile (outer loop, then each hole), so the faces it sweeps name
//! it ([`super::names::Naming::segments`]). A periodic spline is clamped to its own domain by knot
//! insertion (cadrs takes clamped splines): the same curve, any degree.

use nalgebra::{Point2, Point3, Unit, Vector3};

use crate::yack_glue::Made;
use crate::{
    Axis, Bspline3, BsplineSurface, Curve2, DefaultKernel, Extent, Kernel, KernelError, LoftSection, LoftSpec, Loop,
    PathCurve, Plane, Profile, Region, Result, SweepLaw, SweepSpec,
};

/// One profile segment, as yack's `Segment` (angles in radians).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Seg<'a> {
    /// The line from `a` to `b`.
    #[allow(missing_docs)]
    Line { a: [f64; 2], b: [f64; 2] },
    /// The arc about `center` from the angle `start`, turning `sweep` (counter-clockwise > 0).
    #[allow(missing_docs)]
    Arc { center: [f64; 2], radius: f64, start: f64, sweep: f64 },
    /// The rational quadratic from `a` to `b`, tangent to `a→apex` and `apex→b`, apex weight `weight`.
    #[allow(missing_docs)]
    Conic { a: [f64; 2], apex: [f64; 2], b: [f64; 2], weight: f64 },
    /// A clamped NURBS curve.
    #[allow(missing_docs)]
    Nurbs { degree: usize, knots: &'a [f64], ctrl: &'a [[f64; 2]], weights: Option<&'a [f64]> },
    /// A periodic B-spline, the whole loop on its own: `n` control points then the first `degree`
    /// again, `ctrl.len() + degree + 1` knots (the protocol's convention).
    #[allow(missing_docs)]
    Periodic { degree: usize, knots: &'a [f64], ctrl: &'a [[f64; 2]] },
}

/// A frame as yack gives it: its origin and axes (right-handed, unit).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Place {
    #[allow(missing_docs)]
    pub origin: [f64; 3],
    #[allow(missing_docs)]
    pub x: [f64; 3],
    #[allow(missing_docs)]
    pub y: [f64; 3],
    #[allow(missing_docs)]
    pub z: [f64; 3],
}

fn p2(p: [f64; 2]) -> Point2<f64> {
    Point2::new(p[0], p[1])
}

fn v3(v: [f64; 3]) -> Vector3<f64> {
    Vector3::new(v[0], v[1], v[2])
}

fn bad(why: &str) -> KernelError {
    KernelError::InvalidInput(why.into())
}

/// Inserts knot `u` once into the degree-`p` B-spline (`knots`, `ctrl`) (Boehm), in the span of
/// the last knot `<= u` (where `u` is a knot already, the points from it on keep their place).
fn insert(p: usize, knots: &mut Vec<f64>, ctrl: &mut Vec<[f64; 2]>, u: f64) -> Result<()> {
    let uncovered = || bad("a periodic spline's knots do not cover its domain");
    let k = knots[..knots.len().saturating_sub(1)].iter().rposition(|&t| t <= u).ok_or_else(uncovered)?;
    if k < p || k > ctrl.len() {
        return Err(uncovered());
    }
    let mut q = Vec::with_capacity(ctrl.len() + 1);
    q.extend_from_slice(&ctrl[..=k - p]);
    for i in k - p + 1..=k {
        let den = knots[i + p] - knots[i];
        let a = if den > 0.0 { (u - knots[i]) / den } else { 0.0 };
        let c0 = ctrl[i - 1];
        let c = match ctrl.get(i) {
            Some(&c1) => [(1.0 - a) * c0[0] + a * c1[0], (1.0 - a) * c0[1] + a * c1[1]],
            // Past the last point only where u is the knot itself (a = 0).
            None if a == 0.0 => c0,
            None => return Err(uncovered()),
        };
        q.push(c);
    }
    if k < ctrl.len() {
        q.extend_from_slice(&ctrl[k..]);
    }
    knots.insert(k + 1, u);
    *ctrl = q;
    Ok(())
}

/// A periodic B-spline (the protocol's convention: `n` control points then the first `p` again,
/// `n + 2p + 1` knots whose spacing repeats every `n`) with its points and knot spacings turned
/// round so its seam (where [`clamp_periodic`] opens it) lies at the control point farthest from
/// their centroid, the lowest such index on a tie: the same curve, its seam put where it doesn't
/// depend on where the loop's list starts. A loop's start is no part of its shape, but cadrs's
/// cut by a moving solid gives up where a seam lies on the line the cut's envelope touches
/// (brep-bench swept/spline-bar-screwed-through-rod: its list starts at the point nearest the
/// screw's axis). Knots that don't repeat are left as they are.
fn reseat_periodic(p: usize, knots: &[f64], ctrl: &[[f64; 2]]) -> (Vec<f64>, Vec<[f64; 2]>) {
    let as_given = (knots.to_vec(), ctrl.to_vec());
    if p == 0 || ctrl.len() <= 2 * p || knots.len() != ctrl.len() + p + 1 {
        return as_given;
    }
    let n = ctrl.len() - p;
    let d: Vec<f64> = knots.windows(2).map(|w| w[1] - w[0]).collect();
    let scale = d.iter().fold(0.0, |m: f64, x| m.max(x.abs()));
    if (n..d.len()).any(|i| (d[i] - d[i - n]).abs() > crate::native::tol::REL * scale) {
        return as_given;
    }
    let base = &ctrl[..n];
    let c = base.iter().fold([0.0, 0.0], |a, q| [a[0] + q[0] / n as f64, a[1] + q[1] / n as f64]);
    let dist = |q: &[f64; 2]| (q[0] - c[0]).hypot(q[1] - c[1]);
    let mut far = 0;
    for (i, q) in base.iter().enumerate() {
        if dist(q) > dist(&base[far]) {
            far = i;
        }
    }
    // The opened curve starts over the points from s on, nearest the (s + p/2)th.
    let s = (far + n - p / 2) % n;
    let ctrl: Vec<[f64; 2]> = (0..n + p).map(|i| base[(i + s) % n]).collect();
    let mut out = vec![knots[0]];
    for i in 0..d.len() {
        let last = out.last().copied().unwrap_or(knots[0]);
        out.push(last + d[(i + s) % n]);
    }
    (out, ctrl)
}

/// The periodic spline as the clamped one with the same points on its domain
/// `[knots[p], knots[n + p]]`: knots, control points.
pub fn clamp_periodic(p: usize, knots: &[f64], ctrl: &[[f64; 2]]) -> Result<(Vec<f64>, Vec<[f64; 2]>)> {
    if p == 0 || ctrl.len() <= p || knots.len() != ctrl.len() + p + 1 {
        return Err(bad("a periodic spline needs degree ≥ 1, more points than its degree and ctrl + degree + 1 knots"));
    }
    let n = ctrl.len() - p;
    let (ua, ub) = (knots[p], knots[n + p]);
    if !(ub > ua) {
        return Err(bad("a periodic spline's domain is empty"));
    }
    let (mut t, mut q) = (knots.to_vec(), ctrl.to_vec());
    let mult = |t: &[f64], u: f64| t.iter().filter(|&&x| x == u).count();
    let mut guard = 0;
    while mult(&t, ua) < p {
        insert(p, &mut t, &mut q, ua)?;
        guard += 1;
        if guard > 4 * p + 4 {
            return Err(bad("a periodic spline could not be clamped"));
        }
    }
    // u_b: its span is the last with knots[k] <= u_b < knots[k + 1]; it needs a knot past it.
    while mult(&t, ub) < p {
        insert(p, &mut t, &mut q, ub)?;
        guard += 1;
        if guard > 8 * p + 8 {
            return Err(bad("a periodic spline could not be clamped"));
        }
    }
    let a = t.iter().position(|&x| x == ua).ok_or_else(|| bad("clamping lost the domain's start"))?;
    let b = t.iter().rposition(|&x| x == ub).ok_or_else(|| bad("clamping lost the domain's end"))?;
    if a == 0 || b < p || b - p + 1 > q.len() || a - 1 > b - p {
        return Err(bad("a periodic spline could not be clamped"));
    }
    let ctrl = q[a - 1..=b - p].to_vec();
    let mut knots = Vec::with_capacity(ctrl.len() + p + 1);
    knots.push(ua);
    knots.extend_from_slice(&t[a..=b]);
    knots.push(ub);
    Ok((knots, ctrl))
}

/// Segment `s` as a cadrs curve whose `source` is `n`.
fn curve(s: &Seg, n: u64) -> Result<Curve2> {
    let source = Some(n);
    Ok(match *s {
        Seg::Line { a, b } => Curve2::Line { a: p2(a), b: p2(b), source },
        Seg::Arc { center, radius, start, sweep } => {
            Curve2::Arc { center: p2(center), radius, start_angle: start, sweep, source }
        }
        Seg::Conic { a, apex, b, weight } => Curve2::Spline {
            degree: 2,
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            poles: vec![p2(a), p2(apex), p2(b)],
            weights: vec![1.0, weight, 1.0],
            source,
        },
        Seg::Nurbs { degree, knots, ctrl, weights } => Curve2::Spline {
            degree,
            knots: knots.to_vec(),
            poles: ctrl.iter().map(|&c| p2(c)).collect(),
            weights: weights.map(<[f64]>::to_vec).unwrap_or_default(),
            source,
        },
        Seg::Periodic { degree, knots, ctrl } => {
            let (knots, ctrl) = reseat_periodic(degree, knots, ctrl);
            let (knots, ctrl) = clamp_periodic(degree, &knots, &ctrl)?;
            Curve2::Spline { degree, knots, poles: ctrl.into_iter().map(p2).collect(), weights: Vec::new(), source }
        }
    })
}

/// The cadrs profile of one region on `plane`: `loops[0]` its outline, the rest its holes, every
/// segment's `source` its index through them all.
pub fn profile(plane: Plane, loops: &[&[Seg]]) -> Result<Profile> {
    let mut n = 0u64;
    let mut out = Vec::with_capacity(loops.len());
    for l in loops {
        let curves = l
            .iter()
            .map(|s| {
                let c = curve(s, n);
                n += 1;
                c
            })
            .collect::<Result<Vec<_>>>()?;
        out.push(Loop { curves });
    }
    let mut it = out.into_iter();
    let outer = it.next().ok_or_else(|| bad("a profile with no loop"))?;
    Ok(Profile::new(plane, vec![Region { outer, holes: it.collect(), source: Some(0) }]))
}

/// How many segments each loop has (for [`super::names::Naming::loop_lens`]).
pub fn loop_lens(loops: &[&[Seg]]) -> Vec<usize> {
    loops.iter().map(|l| l.len()).collect()
}

/// The profile in `frame`'s xy plane (profile x along frame x, y along frame y) extruded
/// `height` along frame z.
pub fn extrude(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], height: f64) -> Result<Made> {
    let r = k.extrude(&profile(xy_plane(frame), loops)?, Extent::Blind(height))?;
    Ok((r.first()?, r.history))
}

/// The profile drawn in `frame`'s xz plane (profile x along frame x, out from the axis; profile
/// y along frame z) revolved about frame z by `angle` radians (counter-clockwise seen from z).
pub fn revolve(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], angle: f64) -> Result<Made> {
    // The plane's y is normal × x: with the normal −y the profile's y runs along z.
    let plane = Plane::new(Point3::from(v3(frame.origin)), v3(frame.x), -v3(frame.y));
    let z = Unit::try_new(v3(frame.z), 0.0).ok_or_else(|| bad("a revolve's axis is zero"))?;
    let axis = Axis { origin: Point3::from(v3(frame.origin)), dir: z };
    let r = k.revolve(&profile(plane, loops)?, axis, angle)?;
    Ok((r.first()?, r.history))
}

/// A point on a segment (its start), in the profile's coordinates.
fn seg_start(s: &Seg) -> Option<[f64; 2]> {
    match *s {
        Seg::Line { a, .. } | Seg::Conic { a, .. } => Some(a),
        Seg::Arc { center, radius, start, .. } => Some([center[0] + radius * start.cos(), center[1] + radius * start.sin()]),
        Seg::Nurbs { ctrl, .. } | Seg::Periodic { ctrl, .. } => ctrl.first().copied(),
    }
}

/// The profile drawn as for [`revolve`] (frame xz, x out from frame z) screwed about frame z by
/// `angle` radians, rising `rise` along z and moving `taper` out from it per radian turned (a
/// thread on a cone; `taper` 0 is a straight screw). cadrs's sweep along the spiral
/// ([`crate::Helix::conical`]) by its screw: the section stays in a plane through the axis.
pub fn screw_tapered(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], rise: f64, angle: f64, taper: f64) -> Result<Made> {
    if angle == 0.0 || !angle.is_finite() || !rise.is_finite() || !taper.is_finite() {
        return Err(bad("a screw needs a finite, nonzero turn"));
    }
    let plane = Plane::new(Point3::from(v3(frame.origin)), v3(frame.x), -v3(frame.y));
    let z = Unit::try_new(v3(frame.z), 0.0).ok_or_else(|| bad("a screw's axis is zero"))?;
    let x = Unit::try_new(v3(frame.x), 0.0).ok_or_else(|| bad("a screw's frame x is zero"))?;
    // The spiral through the profile's first point: where the path crosses the profile's plane.
    let [r0, h0] = loops.first().and_then(|l| l.first()).and_then(seg_start).ok_or_else(|| bad("a screw of no profile"))?;
    let r1 = r0 + taper * angle;
    if !(r0 > 0.0 && r1 > 0.0) {
        return Err(bad("a screw's profile must lie off its axis all the way"));
    }
    let origin = Point3::from(v3(frame.origin) + z.into_inner() * h0);
    let mut helix = crate::Helix::conical(Axis { origin, dir: z }, r0, r1, rise * angle, 0.0, angle / std::f64::consts::TAU);
    helix.x = x;
    let spec = SweepSpec { path: helix.path(), control: crate::SweepControl::Helix(helix), ..SweepSpec::solid(Vec::new()) };
    let r = k.sweep_with(&profile(plane, loops)?, &spec)?;
    Ok((r.first()?, r.history))
}

/// One patch of a solid [`sewn`] from patches.
#[derive(Clone, Copy, Debug)]
pub enum GluePatch<'a> {
    /// A Bézier patch: its control net, rows along u (any degree each way: one less than the
    /// number of rows, one less than each row's length).
    Bezier {
        #[allow(missing_docs)]
        ctrl: &'a [Vec<[f64; 3]>],
    },
    /// The flat face filling a closed chain of curves lying in one plane.
    PlanarFill {
        #[allow(missing_docs)]
        boundary: &'a [Spline<'a>],
    },
}

/// The solid the `patches` close, sewn where their edges meet (cadrs's B-spline face, fill and
/// sew). Patches that leave the shell open are refused.
pub fn sewn(k: &mut DefaultKernel, patches: &[GluePatch]) -> Result<Made> {
    let mut sheets: Vec<crate::BodyId> = Vec::with_capacity(patches.len());
    let r = (|| {
        for p in patches {
            let sheet = match *p {
                GluePatch::Bezier { ctrl } => {
                    let nv = ctrl.first().map_or(0, Vec::len);
                    if ctrl.len() < 2 || nv < 2 || ctrl.iter().any(|r| r.len() != nv) {
                        return Err(bad("a Bézier patch needs a grid of at least 2 by 2 control points"));
                    }
                    let bezier = |n: usize| [vec![0.0; n], vec![1.0; n]].concat();
                    let surface = crate::BsplineSurface {
                        degree_u: ctrl.len() - 1,
                        degree_v: nv - 1,
                        knots_u: bezier(ctrl.len()),
                        knots_v: bezier(nv),
                        poles: ctrl.iter().map(|r| r.iter().map(|&c| Point3::from(v3(c))).collect()).collect(),
                        weights: Vec::new(),
                    };
                    k.bspline_face(&surface, None)?.first()?
                }
                GluePatch::PlanarFill { boundary } => {
                    let pts: Vec<[f64; 3]> = boundary.iter().flat_map(|c| c.ctrl.iter().copied()).collect();
                    let n = crate::yack_glue::plane_normal(&pts, crate::native::tol::REL)
                        .ok_or_else(|| bad("a fill's boundary does not lie in one plane"))?;
                    let (o, n) = (v3(pts[0]), v3(n));
                    let far = pts.iter().map(|&p| v3(p) - o).max_by(|a, b| a.norm().total_cmp(&b.norm())).unwrap_or_default();
                    let x = Unit::try_new(far - n * far.dot(&n), 0.0).ok_or_else(|| bad("a fill's boundary is a point"))?.into_inner();
                    let y = n.cross(&x);
                    let plane = Plane::new(Point3::from(o), x, n);
                    let to2 = |c: [f64; 3]| {
                        let d = v3(c) - o;
                        Point2::new(d.dot(&x), d.dot(&y))
                    };
                    let curves = boundary
                        .iter()
                        .map(|c| crate::FillCurve::Sketch {
                            plane,
                            curve: Curve2::Spline {
                                degree: c.degree,
                                knots: c.knots.to_vec(),
                                poles: c.ctrl.iter().map(|&p| to2(p)).collect(),
                                weights: c.weights.map(<[f64]>::to_vec).unwrap_or_default(),
                                source: None,
                            },
                        })
                        .collect();
                    k.fill(&crate::FillSpec { curves, source: 0 })?.first()?
                }
            };
            sheets.push(sheet);
        }
        let r = k.sew_solid(&sheets, crate::native::tol::POINT)?;
        let out = r.first()?;
        if k.body_class(out)? != crate::BodyClass::Solid {
            k.release(out);
            return Err(KernelError::InvalidInput("the patches do not close a solid".into()));
        }
        Ok((out, r.history))
    })();
    for s in sheets {
        k.release(s);
    }
    r
}

/// A guide (or any space curve) as yack's `Spline3`: a clamped NURBS curve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spline<'a> {
    #[allow(missing_docs)]
    pub degree: usize,
    #[allow(missing_docs)]
    pub knots: &'a [f64],
    #[allow(missing_docs)]
    pub ctrl: &'a [[f64; 3]],
    #[allow(missing_docs)]
    pub weights: Option<&'a [f64]>,
}

/// `place`'s xy plane, its x along the frame's x (a section's or an extrude's sketch plane).
fn xy_plane(place: &Place) -> Plane {
    Plane::new(Point3::from(v3(place.origin)), v3(place.x), v3(place.z))
}

/// A solid lofted from the first section to the last along `guides`, each running from a corner
/// of the first section's outline to a corner of the last's (cadrs's loft with guides: two
/// sections, each one loop drawn in its frame's xy plane).
pub fn loft_guided(k: &mut DefaultKernel, sections: &[(Place, &[&[Seg]])], guides: &[Spline]) -> Result<Made> {
    let sections: Vec<Section> = sections.iter().map(|&(place, loops)| Section::Profile(place, loops)).collect();
    loft_guided_to(k, &sections, guides)
}

/// Whether the points lie in one plane (collinear ones do), within a relative rounding error.
fn coplanar(ctrl: &[[f64; 3]]) -> bool {
    let pts: Vec<Vector3<f64>> = ctrl.iter().map(|&c| v3(c)).collect();
    let Some(&first) = pts.first() else { return true };
    let spread = pts.iter().map(|p| (p - first).norm()).fold(0.0, f64::max);
    let eps = crate::native::tol::REL * (1.0 + spread);
    // The direction of the farthest point, then the one farthest from that line: the plane.
    let Some(&far) = pts.iter().max_by(|a, b| (*a - first).norm().total_cmp(&(*b - first).norm())) else { return true };
    let d = far - first;
    if d.norm() <= eps {
        return true;
    }
    let d = d.normalize();
    let off = |p: &Vector3<f64>| (p - first) - d * (p - first).dot(&d);
    let Some(side) = pts.iter().map(off).max_by(|a, b| a.norm().total_cmp(&b.norm())) else { return true };
    if side.norm() <= eps {
        return true;
    }
    let n = d.cross(&side.normalize());
    pts.iter().all(|p| (p - first).dot(&n).abs() <= eps)
}

/// A guided loft's section: a profile (one loop) in a frame's xy plane, or a point (the first or
/// the last section only: the loft runs to an apex there).
#[derive(Clone, Copy, Debug)]
pub enum Section<'a> {
    /// The loop `loops[0]` in the frame's xy plane.
    Profile(Place, &'a [&'a [Seg<'a>]]),
    /// A point.
    Point([f64; 3]),
}

/// [`loft_guided`] with point sections allowed at either end (the guides then run to the point).
pub fn loft_guided_to(k: &mut DefaultKernel, sections: &[Section], guides: &[Spline]) -> Result<Made> {
    let [first, last] = sections else {
        return Err(KernelError::Unsupported("a guided loft through other than two sections"));
    };
    let section = |s: &Section| -> Result<LoftSection> {
        match *s {
            Section::Point(p) => Ok(LoftSection::Point(Point3::from(v3(p)))),
            Section::Profile(place, loops) => {
                if loops.len() != 1 {
                    return Err(KernelError::Unsupported("a guided loft's section with holes"));
                }
                Ok(LoftSection::Profile(profile(xy_plane(&place), loops)?))
            }
        }
    };
    if matches!((first, last), (Section::Point(_), Section::Point(_))) {
        return Err(KernelError::InvalidInput("a loft between two points".into()));
    }
    let spec = LoftSpec::solid(vec![section(first)?, section(last)?]);
    let guides: Vec<Vec<PathCurve>> = guides
        .iter()
        .map(|g| {
            vec![PathCurve::Bspline(Bspline3 {
                degree: g.degree,
                knots: g.knots.to_vec(),
                poles: g.ctrl.iter().map(|&c| Point3::from(v3(c))).collect(),
                weights: g.weights.map(<[f64]>::to_vec).unwrap_or_default(),
            })]
        })
        .collect();
    let r = k.loft_guided(&spec, &guides)?;
    Ok((r.first()?, r.history))
}

/// The solid one closed NURBS surface bounds: control net `ctrl` (rows along u, points in
/// `frame`'s coordinates), its weights where given; its face cadrs's face on the surface's net
/// (bounded by its seams and poles), sewn into a solid.
pub fn nurbs_solid(
    k: &mut DefaultKernel,
    frame: &Place,
    degree: [usize; 2],
    knots: [&[f64]; 2],
    ctrl: &[Vec<[f64; 3]>],
    weights: Option<&[Vec<f64>]>,
) -> Result<Made> {
    let (o, x, y, z) = (v3(frame.origin), v3(frame.x), v3(frame.y), v3(frame.z));
    let world = |c: [f64; 3]| Point3::from(o + x * c[0] + y * c[1] + z * c[2]);
    let surface = BsplineSurface {
        degree_u: degree[0],
        degree_v: degree[1],
        knots_u: knots[0].to_vec(),
        knots_v: knots[1].to_vec(),
        poles: ctrl.iter().map(|row| row.iter().map(|&c| world(c)).collect()).collect(),
        weights: weights.map(<[Vec<f64>]>::to_vec).unwrap_or_default(),
    };
    let sheet = k.bspline_face(&surface, None)?.first()?;
    let mut pieces = vec![sheet];
    pieces.extend(flat_caps(k, sheet).unwrap_or_default());
    let r = k.sew_solid(&pieces, crate::native::tol::CONFUSION);
    for p in pieces {
        k.release(p);
    }
    let r = r?;
    Ok((r.first()?, r.history))
}

/// The flat faces filling each open rim of `sheet` that lies in one plane (a cone's base: cadrs's
/// runner fills it the same way), each a sheet of its own; an error for a rim it cannot fill.
fn flat_caps(k: &mut DefaultKernel, sheet: crate::BodyId) -> Result<Vec<crate::BodyId>> {
    let free: Vec<crate::EdgeInfo> = k
        .edges(sheet)?
        .into_iter()
        .filter(|e| e.faces.iter().filter(|f| f.is_some()).count() == 1 && e.length > 0.0)
        .collect();
    // The rims: chains of free edges joined end to end.
    let near = |a: &Point3<f64>, b: &Point3<f64>| (a - b).norm() <= crate::native::tol::POINT;
    let mut rim_of: Vec<usize> = (0..free.len()).collect();
    for i in 0..free.len() {
        for j in 0..i {
            let (a, b) = (&free[i], &free[j]);
            if [a.start, a.end].iter().any(|p| near(p, &b.start) || near(p, &b.end)) {
                let (ri, rj) = (rim_of[i], rim_of[j]);
                rim_of.iter_mut().filter(|r| **r == ri).for_each(|r| *r = rj);
            }
        }
    }
    let mut caps = Vec::new();
    let mut rims: Vec<usize> = rim_of.clone();
    rims.sort_unstable();
    rims.dedup();
    for rim in rims {
        let edges: Vec<&crate::EdgeInfo> = free.iter().zip(&rim_of).filter(|(_, r)| **r == rim).map(|(e, _)| e).collect();
        // Its ends, middles and the lines along its ends (a whole circle has one point twice).
        let pts: Vec<[f64; 3]> = edges
            .iter()
            .flat_map(|e| [e.start, e.mid, e.end, e.start + e.start_tangent, e.end + e.end_tangent])
            .map(|p| [p.x, p.y, p.z])
            .collect();
        if crate::yack_glue::plane_normal(&pts, crate::native::tol::REL).is_none() {
            continue;
        }
        let curves = edges.iter().map(|e| crate::FillCurve::Edge { body: sheet, edge: e.id }).collect();
        let cap = k.fill(&crate::FillSpec { curves, source: 1 })?.first()?;
        // Only a flat cap: a rim that only looked flat is left open (and the sew refuses it).
        if k.get(cap)?.faces.iter().all(|f| matches!(f.surf, crate::native::surface::Surface::Plane { .. })) {
            caps.push(cap);
        } else {
            k.release(cap);
        }
    }
    Ok(caps)
}

/// A piece of a path of lines and arcs (yack's `PathPiece`; angles in radians).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Piece {
    /// Straight on for `length`.
    #[allow(missing_docs)]
    Line { length: f64 },
    /// A circular turn of `radius` by `angle`, positive turning toward the frame's +x.
    #[allow(missing_docs)]
    Arc { radius: f64, angle: f64 },
}

/// The profile (one region: `loops[0]` and its holes) in `frame`'s xy plane swept along a path of
/// `pieces` that starts at the frame's origin along its z, in the frame's xz plane (cadrs's sweep
/// along sketch curves, mitred where pieces meet at a corner).
pub fn sweep(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], pieces: &[Piece]) -> Result<Made> {
    if pieces.is_empty() {
        return Err(bad("a sweep along no path"));
    }
    // The path's plane: its 2-D x along the frame's z (the way the path starts), its 2-D y along
    // the frame's x (where a positive arc turns): normal y × z = x, so normal = the frame's y.
    let plane = Plane::new(Point3::from(v3(frame.origin)), v3(frame.z), v3(frame.y));
    let (mut at, mut heading) = (Point2::new(0.0, 0.0), 0.0f64);
    let mut path = Vec::with_capacity(pieces.len());
    for p in pieces {
        let curve = match *p {
            Piece::Line { length } => {
                if !(length > 0.0) {
                    return Err(bad("a path line must be longer than nothing"));
                }
                let to = at + nalgebra::Vector2::new(heading.cos(), heading.sin()) * length;
                let c = Curve2::Line { a: at, b: to, source: None };
                at = to;
                c
            }
            Piece::Arc { radius, angle } => {
                if !(radius > 0.0) || angle == 0.0 {
                    return Err(bad("a path arc needs a positive radius and a turn"));
                }
                // The centre on the side it turns to; the start's angle seen from it.
                let side = angle.signum();
                let normal = nalgebra::Vector2::new(-heading.sin(), heading.cos()) * side;
                let center = at + normal * radius;
                let start_angle = heading - side * std::f64::consts::FRAC_PI_2;
                let c = Curve2::Arc { center, radius, start_angle, sweep: angle, source: None };
                let end = start_angle + angle;
                at = center + nalgebra::Vector2::new(end.cos(), end.sin()) * radius;
                heading += angle;
                c
            }
        };
        path.push(PathCurve::Sketch { plane, curve });
    }
    let spec = SweepSpec::solid(path);
    let r = k.sweep_with(&profile(xy_plane(frame), loops)?, &spec)?;
    Ok((r.first()?, r.history))
}

/// The profile in `frame`'s xy plane swept along `path` (in the frame's coordinates, starting at
/// its origin along its z), as the protocol's `sweep_path`: the profile's axes follow the path's
/// rotation-minimizing frame and turn `twist` radians more by the end, evenly in arc length
/// (cadrs's sweep law). cadrs carries a profile by the path's principal normal (Frenet) where the
/// path bends about evenly everywhere; on a planar path that is the rotation-minimizing frame,
/// on a twisted one it is not, so a non-planar path is taken only for an untwisted round
/// profile centred on it (whose sweep is the same either way), else declined.
pub fn sweep_path(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], path: &Spline, twist: f64) -> Result<Made> {
    let planar = coplanar(path.ctrl);
    let round = matches!(loops, [[Seg::Arc { center: [cx, cy], sweep, .. }]] if *cx == 0.0 && *cy == 0.0 && sweep.abs() >= std::f64::consts::TAU);
    if !planar && !(twist == 0.0 && round) {
        return Err(KernelError::Unsupported(
            "a sweep along a twisted path of a profile that is not a round one centred on it (cadrs may carry it by the path's Frenet frame)",
        ));
    }
    let (o, x, y, z) = (v3(frame.origin), v3(frame.x), v3(frame.y), v3(frame.z));
    let curve = PathCurve::Bspline(Bspline3 {
        degree: path.degree,
        knots: path.knots.to_vec(),
        poles: path.ctrl.iter().map(|&c| Point3::from(o + x * c[0] + y * c[1] + z * c[2])).collect(),
        weights: path.weights.map(<[f64]>::to_vec).unwrap_or_default(),
    });
    let spec = SweepSpec::solid(vec![curve]);
    let section = profile(xy_plane(frame), loops)?;
    let r = if twist == 0.0 {
        k.sweep_with(&section, &spec)?
    } else {
        k.sweep_with_law(&section, &spec, &SweepLaw { twist, scale: 1.0, spiral: false })?
    };
    Ok((r.first()?, r.history))
}

/// The ruled loft from the profile in `frame`'s xy plane to `top` in the parallel plane `height`
/// up its z (yack's loft: `top` arrives already scaled).
pub fn loft(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], height: f64, top: &[&[Seg]]) -> Result<Made> {
    let up = Place {
        origin: [
            frame.origin[0] + frame.z[0] * height,
            frame.origin[1] + frame.z[1] * height,
            frame.origin[2] + frame.z[2] * height,
        ],
        ..*frame
    };
    let spec = LoftSpec::through(vec![profile(xy_plane(frame), loops)?, profile(xy_plane(&up), top)?]);
    let r = k.loft_with(&spec)?;
    Ok((r.first()?, r.history))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> Place {
        Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] }
    }

    #[test]
    fn patches_sew_into_a_cube_and_an_open_set_is_refused() {
        let mut k = DefaultKernel::new();
        let c = |x: f64, y: f64, z: f64| [x, y, z];
        let quad = |a: [f64; 3], b: [f64; 3], d: [f64; 3], e: [f64; 3]| vec![vec![a, b], vec![d, e]];
        let sides = [
            quad(c(0.0, 0.0, 1.0), c(1.0, 0.0, 1.0), c(0.0, 1.0, 1.0), c(1.0, 1.0, 1.0)),
            quad(c(0.0, 0.0, 0.0), c(1.0, 0.0, 0.0), c(0.0, 0.0, 1.0), c(1.0, 0.0, 1.0)),
            quad(c(0.0, 1.0, 0.0), c(1.0, 1.0, 0.0), c(0.0, 1.0, 1.0), c(1.0, 1.0, 1.0)),
            quad(c(0.0, 0.0, 0.0), c(0.0, 1.0, 0.0), c(0.0, 0.0, 1.0), c(0.0, 1.0, 1.0)),
            quad(c(1.0, 0.0, 0.0), c(1.0, 1.0, 0.0), c(1.0, 0.0, 1.0), c(1.0, 1.0, 1.0)),
        ];
        let mut patches: Vec<GluePatch> = sides.iter().map(|q| GluePatch::Bezier { ctrl: q }).collect();
        assert!(sewn(&mut k, &patches).is_err());
        // The floor as a fill of its four edges.
        let line = [0.0, 0.0, 1.0, 1.0];
        let corners = [c(0.0, 0.0, 0.0), c(1.0, 0.0, 0.0), c(1.0, 1.0, 0.0), c(0.0, 1.0, 0.0)];
        let legs: Vec<[[f64; 3]; 2]> = (0..4).map(|i| [corners[i], corners[(i + 1) % 4]]).collect();
        let edges: Vec<Spline> = legs.iter().map(|l| Spline { degree: 1, knots: &line, ctrl: l, weights: None }).collect();
        patches.push(GluePatch::PlanarFill { boundary: &edges });
        let (b, _) = sewn(&mut k, &patches).unwrap();
        let v = k.mass_properties(b).unwrap().volume;
        assert!((v - 1.0).abs() < 1e-9, "{v}");
    }

    #[test]
    fn a_nurbs_cone_gets_its_flat_base() {
        // brep-bench geometry/nurbs-cone: a rational circle of radius 5 ruled to an apex 10 up.
        let mut k = DefaultKernel::new();
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let around = [0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
        let circle = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (-1.0, 0.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0), (1.0, 0.0)];
        let ctrl: Vec<Vec<[f64; 3]>> = circle.iter().map(|&(x, y)| vec![[5.0 * x, 5.0 * y, 0.0], [0.0, 0.0, 10.0]]).collect();
        let weights: Vec<Vec<f64>> = (0..9).map(|i| if i % 2 == 1 { vec![h, h] } else { vec![1.0, 1.0] }).collect();
        let (cone, _) = nurbs_solid(&mut k, &world(), [2, 1], [&around, &[0.0, 0.0, 1.0, 1.0]], &ctrl, Some(&weights)).unwrap();
        let v = k.mass_properties(cone).unwrap().volume;
        let want = std::f64::consts::PI * 25.0 * 10.0 / 3.0;
        assert!((v - want).abs() < 1e-6 * want, "{v} against {want}");
    }

    #[test]
    fn a_tapered_screw_sweeps_its_section_by_pappus() {
        // In-plane motion sweeps nothing: V = A (x̄ θ + taper θ² / 2).
        let mut k = DefaultKernel::new();
        let sq = [
            Seg::Line { a: [2.0, 0.0], b: [3.0, 0.0] },
            Seg::Line { a: [3.0, 0.0], b: [3.0, 0.5] },
            Seg::Line { a: [3.0, 0.5], b: [2.0, 0.5] },
            Seg::Line { a: [2.0, 0.5], b: [2.0, 0.0] },
        ];
        let tau = std::f64::consts::TAU;
        let (angle, rise, taper) = (2.0 * tau, 1.0 / tau, 0.1 / tau);
        let (b, _) = screw_tapered(&mut k, &world(), &[&sq], rise, angle, taper).unwrap();
        let want = 0.5 * (2.5 * angle + taper * angle * angle / 2.0);
        let v = k.mass_properties(b).unwrap().volume;
        assert!((v - want).abs() < 1e-6 * want, "{v} against {want}");
        let (b, _) = screw_tapered(&mut k, &world(), &[&sq], rise, angle, 0.0).unwrap();
        let v = k.mass_properties(b).unwrap().volume;
        assert!((v - 0.5 * 2.5 * angle).abs() < 1e-6 * v, "{v}");
    }

    fn de_boor(p: usize, knots: &[f64], ctrl: &[[f64; 2]], u: f64) -> [f64; 2] {
        // de Boor on the span holding u (the last for u at the domain's end).
        let n = ctrl.len();
        let k = (p..n).rev().find(|&k| knots[k] <= u && (u < knots[k + 1] || k == n - 1)).unwrap_or(p);
        let mut d: Vec<[f64; 2]> = (0..=p).map(|j| ctrl[j + k - p]).collect();
        for r in 1..=p {
            for j in (r..=p).rev() {
                let i = j + k - p;
                let a = (u - knots[i]) / (knots[i + p + 1 - r] - knots[i]);
                d[j] = [(1.0 - a) * d[j - 1][0] + a * d[j][0], (1.0 - a) * d[j - 1][1] + a * d[j][1]];
            }
        }
        d[p]
    }

    #[test]
    fn a_clamped_periodic_spline_is_the_same_curve() {
        for p in [2usize, 3, 4] {
            // A periodic loop of 6 points round the unit circle, the first p repeated.
            let base: Vec<[f64; 2]> = (0..6).map(|i| {
                let t = i as f64 * std::f64::consts::TAU / 6.0;
                [t.cos(), t.sin()]
            }).collect();
            let ctrl: Vec<[f64; 2]> = base.iter().chain(base.iter().take(p)).copied().collect();
            let knots: Vec<f64> = (0..ctrl.len() + p + 1).map(|i| i as f64 - p as f64).collect();
            let (ck, cc) = clamp_periodic(p, &knots, &ctrl).unwrap();
            assert_eq!(ck.len(), cc.len() + p + 1);
            let (ua, ub) = (knots[p], knots[6 + p]);
            for s in 0..=20 {
                let u = ua + (ub - ua) * s as f64 / 20.0;
                let a = de_boor(p, &knots, &ctrl, u.min(ub - 1e-12));
                let b = de_boor(p, &ck, &cc, u);
                assert!((a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9, "p {p} u {u}: {a:?} vs {b:?}");
            }
            // Closed: it ends where it starts.
            assert!((cc[0][0] - cc[cc.len() - 1][0]).abs() < 1e-12 && (cc[0][1] - cc[cc.len() - 1][1]).abs() < 1e-12);
        }
    }

    #[test]
    fn a_square_extrudes_and_a_rectangle_revolves() {
        let mut k = DefaultKernel::new();
        let sq = [
            Seg::Line { a: [0.0, 0.0], b: [2.0, 0.0] },
            Seg::Line { a: [2.0, 0.0], b: [2.0, 2.0] },
            Seg::Line { a: [2.0, 2.0], b: [0.0, 2.0] },
            Seg::Line { a: [0.0, 2.0], b: [0.0, 0.0] },
        ];
        let (b, h) = extrude(&mut k, &world(), &[&sq], 3.0).unwrap();
        assert!((k.mass_properties(b).unwrap().volume - 12.0).abs() < 1e-9);
        // The faces swept from segments name them through `source`.
        assert!(h.generated.iter().any(|(_, o)| matches!(o, crate::Origin::ProfileCurve { curve: 2, .. })));
        // A 1 × 2 rectangle 1 out from the axis, revolved whole: a tube of volume π (2² − 1²) 2.
        let rect = [
            Seg::Line { a: [1.0, 0.0], b: [2.0, 0.0] },
            Seg::Line { a: [2.0, 0.0], b: [2.0, 2.0] },
            Seg::Line { a: [2.0, 2.0], b: [1.0, 2.0] },
            Seg::Line { a: [1.0, 2.0], b: [1.0, 0.0] },
        ];
        let (t, _) = revolve(&mut k, &world(), &[&rect], std::f64::consts::TAU).unwrap();
        let v = k.mass_properties(t).unwrap().volume;
        assert!((v - std::f64::consts::PI * 3.0 * 2.0).abs() < 1e-9, "{v}");
        // And it stands on the frame's xy plane, rising along z.
        let c = k.mass_properties(t).unwrap().center_of_mass;
        assert!((c.z - 1.0).abs() < 1e-9, "{c:?}");
        // A periodic quartic ring extrudes too.
        let base: Vec<[f64; 2]> = (0..6).map(|i| {
            let t = i as f64 * std::f64::consts::TAU / 6.0;
            [3.0 * t.cos(), 3.0 * t.sin()]
        }).collect();
        let ctrl: Vec<[f64; 2]> = base.iter().chain(base.iter().take(4)).copied().collect();
        let knots: Vec<f64> = (0..ctrl.len() + 5).map(|i| i as f64).collect();
        let ring = [Seg::Periodic { degree: 4, knots: &knots, ctrl: &ctrl }];
        let (r, _) = extrude(&mut k, &world(), &[&ring], 1.0).unwrap();
        assert!(k.mass_properties(r).unwrap().volume > 10.0);
    }

    #[test]
    fn a_nurbs_sphere_and_a_guided_loft() {
        let mut k = DefaultKernel::new();
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let c = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (-1.0, 0.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0), (1.0, 0.0)];
        let w = [1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0];
        let m = [(0.0, -5.0), (5.0, -5.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)];
        let v = [1.0, s, 1.0, s, 1.0];
        let ctrl: Vec<Vec<[f64; 3]>> = c.iter().map(|&(x, y)| m.iter().map(|&(r, z)| [r * x, r * y, z]).collect()).collect();
        let weights: Vec<Vec<f64>> = w.iter().map(|&wi| v.iter().map(|&vj| wi * vj).collect()).collect();
        let ku = [0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
        let kv = [0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
        let (ball, _) = nurbs_solid(&mut k, &world(), [2, 2], [&ku, &kv], &ctrl, Some(&weights)).unwrap();
        let vol = k.mass_properties(ball).unwrap().volume;
        assert!((vol - 4.0 / 3.0 * std::f64::consts::PI * 125.0).abs() < 1e-6, "{vol}");
        // A 2 × 2 square at z = 0 lofted to one at z = 4 along four straight guides: a box.
        let sq = [
            Seg::Line { a: [0.0, 0.0], b: [2.0, 0.0] },
            Seg::Line { a: [2.0, 0.0], b: [2.0, 2.0] },
            Seg::Line { a: [2.0, 2.0], b: [0.0, 2.0] },
            Seg::Line { a: [0.0, 2.0], b: [0.0, 0.0] },
        ];
        let top = Place { origin: [0.0, 0.0, 4.0], ..world() };
        let corners = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]];
        let lines: Vec<[[f64; 3]; 2]> = corners.iter().map(|&[x, y]| [[x, y, 0.0], [x, y, 4.0]]).collect();
        let knots = [0.0, 0.0, 1.0, 1.0];
        let guides: Vec<Spline> = lines.iter().map(|l| Spline { degree: 1, knots: &knots, ctrl: l, weights: None }).collect();
        let loops: [&[Seg]; 1] = [&sq];
        let (b, _) = loft_guided(&mut k, &[(world(), &loops), (top, &loops)], &guides).unwrap();
        let vol = k.mass_properties(b).unwrap().volume;
        assert!((vol - 16.0).abs() < 1e-6, "{vol}");
    }

    #[test]
    fn a_guided_loft_to_a_point_is_a_pyramid() {
        let mut k = DefaultKernel::new();
        let sq = [
            Seg::Line { a: [0.0, 0.0], b: [2.0, 0.0] },
            Seg::Line { a: [2.0, 0.0], b: [2.0, 2.0] },
            Seg::Line { a: [2.0, 2.0], b: [0.0, 2.0] },
            Seg::Line { a: [0.0, 2.0], b: [0.0, 0.0] },
        ];
        let loops: [&[Seg]; 1] = [&sq];
        let apex = [1.0, 1.0, 3.0];
        let knots = [0.0, 0.0, 1.0, 1.0];
        let lines: Vec<[[f64; 3]; 2]> = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]].iter().map(|&[x, y]| [[x, y, 0.0], apex]).collect();
        let guides: Vec<Spline> = lines.iter().map(|l| Spline { degree: 1, knots: &knots, ctrl: l, weights: None }).collect();
        let (b, _) = loft_guided_to(&mut k, &[Section::Profile(world(), &loops), Section::Point(apex)], &guides).unwrap();
        let v = k.mass_properties(b).unwrap().volume;
        assert!((v - 4.0).abs() < 1e-6, "{v}");
    }

    #[test]
    fn sweeps_along_pieces_and_a_spline_and_a_loft() {
        let mut k = DefaultKernel::new();
        let disc = [Seg::Arc { center: [0.0, 0.0], radius: 1.0, start: 0.0, sweep: std::f64::consts::TAU }];
        let loops: [&[Seg]; 1] = [&disc];
        // 10 straight, then a quarter turn of radius 5: π (10 + 5·π/2) by Pappus.
        let pieces = [Piece::Line { length: 10.0 }, Piece::Arc { radius: 5.0, angle: std::f64::consts::FRAC_PI_2 }];
        let (s, _) = sweep(&mut k, &world(), &loops, &pieces).unwrap();
        let v = k.mass_properties(s).unwrap().volume;
        let want = std::f64::consts::PI * (10.0 + 5.0 * std::f64::consts::FRAC_PI_2);
        assert!((v - want).abs() < 1e-6, "{v} {want}");
        // The arc turned toward +x: the path ends at x = 5, z = 15, heading along +x (the end
        // cap in the plane x = 5, the tube 1 round the path).
        let bb = k.bounding_box(s).unwrap();
        assert!((bb.max.x - 5.0).abs() < 1e-6 && (bb.max.z - 16.0).abs() < 1e-6, "{bb:?}");
        // Along a straight spline 7 up z: π · 7.
        let ctrl = [[0.0, 0.0, 0.0], [0.0, 0.0, 7.0]];
        let knots = [0.0, 0.0, 1.0, 1.0];
        let line = Spline { degree: 1, knots: &knots, ctrl: &ctrl, weights: None };
        let (p, _) = sweep_path(&mut k, &world(), &loops, &line, 0.0).unwrap();
        assert!((k.mass_properties(p).unwrap().volume - 7.0 * std::f64::consts::PI).abs() < 1e-6);
        // A 2 × 2 square lofted to a 1 × 1 one 3 up: a frustum of volume 3 (4 + 2 + 1) / 3 = 7.
        let big = [
            Seg::Line { a: [-1.0, -1.0], b: [1.0, -1.0] },
            Seg::Line { a: [1.0, -1.0], b: [1.0, 1.0] },
            Seg::Line { a: [1.0, 1.0], b: [-1.0, 1.0] },
            Seg::Line { a: [-1.0, 1.0], b: [-1.0, -1.0] },
        ];
        let small = [
            Seg::Line { a: [-0.5, -0.5], b: [0.5, -0.5] },
            Seg::Line { a: [0.5, -0.5], b: [0.5, 0.5] },
            Seg::Line { a: [0.5, 0.5], b: [-0.5, 0.5] },
            Seg::Line { a: [-0.5, 0.5], b: [-0.5, -0.5] },
        ];
        let (l, _) = loft(&mut k, &world(), &[&big], 3.0, &[&small]).unwrap();
        assert!((k.mass_properties(l).unwrap().volume - 7.0).abs() < 1e-6);
    }

    #[test]
    fn a_twist_runs_evenly_in_arc_length() {
        let mut k = DefaultKernel::new();
        // A 4 × 0.4 bar swept 10 up z along a quadratic whose speed is far from even, a quarter
        // turn of twist: halfway along its length (z = 5) the bar has turned 45°. (Evenly in the
        // parameter it would have turned 61°, z = 5 being at t = 0.675.) A point 1.5 out at the
        // angle a is in the bar there when |a − 45°| < asin(0.2 / 1.5) = 7.7°.
        let bar = [
            Seg::Line { a: [-2.0, -0.2], b: [2.0, -0.2] },
            Seg::Line { a: [2.0, -0.2], b: [2.0, 0.2] },
            Seg::Line { a: [2.0, 0.2], b: [-2.0, 0.2] },
            Seg::Line { a: [-2.0, 0.2], b: [-2.0, -0.2] },
        ];
        let ctrl = [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 10.0]];
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let path = Spline { degree: 2, knots: &knots, ctrl: &ctrl, weights: None };
        let (s, _) = sweep_path(&mut k, &world(), &[&bar], &path, std::f64::consts::FRAC_PI_2).unwrap();
        let at = |a: f64| Point3::new(1.5 * a.to_radians().cos(), 1.5 * a.to_radians().sin(), 5.0);
        assert_eq!(k.classify(s, at(45.0), 1e-9).unwrap(), crate::PointClass::Inside);
        assert_eq!(k.classify(s, at(225.0), 1e-9).unwrap(), crate::PointClass::Inside);
        assert_eq!(k.classify(s, at(61.0), 1e-9).unwrap(), crate::PointClass::Outside);
        assert_eq!(k.classify(s, at(30.0), 1e-9).unwrap(), crate::PointClass::Outside);
        assert_eq!(k.classify(s, at(75.0), 1e-9).unwrap(), crate::PointClass::Outside);
        // The bar is untwisted volume 1.6 × 10.
        assert!((k.mass_properties(s).unwrap().volume - 16.0).abs() < 1e-3);
        // A square along a twisted cubic is declined; a disc along it is taken.
        let twisted = [[0.0, 0.0, 0.0], [0.0, 0.0, 5.0], [5.0, 0.0, 8.0], [5.0, 5.0, 12.0]];
        let k4 = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let tw = Spline { degree: 3, knots: &k4, ctrl: &twisted, weights: None };
        let small = [
            Seg::Line { a: [-0.5, -0.5], b: [0.5, -0.5] },
            Seg::Line { a: [0.5, -0.5], b: [0.5, 0.5] },
            Seg::Line { a: [0.5, 0.5], b: [-0.5, 0.5] },
            Seg::Line { a: [-0.5, 0.5], b: [-0.5, -0.5] },
        ];
        assert!(matches!(sweep_path(&mut k, &world(), &[&small], &tw, 0.0), Err(KernelError::Unsupported(_))));
        let disc = [Seg::Arc { center: [0.0, 0.0], radius: 0.5, start: 0.0, sweep: std::f64::consts::TAU }];
        assert!(sweep_path(&mut k, &world(), &[&disc], &tw, 0.0).is_ok());
    }

}
