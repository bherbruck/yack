// yack: yack profiles (plain data) as cadrs profiles, and the extrude and revolve rungs over them.
//! yack's [`Seg`]ments arrive as plain numbers; each becomes a cadrs [`Curve2`] whose `source` is
//! its index through the whole profile (outer loop, then each hole), so the faces it sweeps name
//! it ([`super::names::Naming::segments`]). A periodic spline is clamped to its own domain by knot
//! insertion (cadrs takes clamped splines): the same curve, any degree.

use nalgebra::{Matrix3, Point2, Point3, Unit, Vector3};

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
    /// The face cadrs's fill makes on a closed chain of curves, each lying in a plane of its own
    /// (a surface through all of them, flat where they all lie in one plane): a Coons-like
    /// patch on four parabolas, a cap of a rim.
    Fill {
        #[allow(missing_docs)]
        boundary: &'a [Spline<'a>],
    },
}

/// `c` as a sketch curve on a plane holding it (one through it and its chord's sideways
/// direction when it is a straight line).
pub(crate) fn spline_sketch(c: &Spline) -> Result<(Plane, Curve2)> {
    let (Some(&first), Some(&last)) = (c.ctrl.first(), c.ctrl.last()) else {
        return Err(bad("a boundary curve with no control points"));
    };
    let o = v3(first);
    let chord = v3(last) - o;
    let far = c.ctrl.iter().map(|&p| v3(p) - o).max_by(|a, b| a.norm().total_cmp(&b.norm())).unwrap_or(chord);
    let x = Unit::try_new(far, 0.0).ok_or_else(|| bad("a boundary curve is a point"))?.into_inner();
    let n = match crate::yack_glue::plane_normal(c.ctrl, crate::native::tol::REL) {
        Some(n) => v3(n),
        None => {
            // Collinear (a straight boundary): any plane through it.
            if c.ctrl.iter().any(|&p| (v3(p) - o).cross(&x).norm() > crate::native::tol::POINT) {
                return Err(bad("a boundary curve does not lie in a plane"));
            }
            let side = if x.x.abs() < 0.9 { Vector3::x() } else { Vector3::y() };
            x.cross(&side).normalize()
        }
    };
    let y = n.cross(&x);
    let plane = Plane::new(Point3::from(o), x, n);
    let to2 = |p: [f64; 3]| {
        let d = v3(p) - o;
        Point2::new(d.dot(&x), d.dot(&y))
    };
    let curve = Curve2::Spline {
        degree: c.degree,
        knots: c.knots.to_vec(),
        poles: c.ctrl.iter().map(|&p| to2(p)).collect(),
        weights: c.weights.map(<[f64]>::to_vec).unwrap_or_default(),
        source: None,
    };
    Ok((plane, curve))
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
                GluePatch::Fill { boundary } => {
                    let curves = boundary.iter().map(spline_sketch).collect::<Result<Vec<_>>>()?;
                    k.fill(&crate::FillSpec::sketched(&curves))?.first()?
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

/// The profile in `frame`'s xy plane extruded `height` along frame z, its sides drafted by
/// `angle` radians about the profile's plane (positive: leaning in as they rise, the top
/// smaller; negative: out), and with `both` the same again down along −z (a prism drafted both
/// ways from its section plane, creased there). cadrs's extrude then its draft of the sides,
/// as cadrs's runner drafts parts/star-handle (its lofts of offset sections for drafted-slot and
/// creased-bar are the same solids).
pub fn extrude_drafted(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], height: f64, angle: f64, both: bool) -> Result<Made> {
    if !(height > 0.0) || !angle.is_finite() || angle.abs() >= std::f64::consts::FRAC_PI_2 {
        return Err(bad("a drafted extrude needs a positive height and a draft under 90 degrees"));
    }
    let z = Unit::try_new(v3(frame.z), 0.0).ok_or_else(|| bad("a drafted extrude's frame z is zero"))?;
    let up = drafted_half(k, frame, loops, height, angle, z)?;
    if !both {
        return Ok(up);
    }
    let down = drafted_half(k, frame, loops, height, angle, -z);
    let r = down.and_then(|(d, _)| {
        let u = k.boolean(crate::BoolOp::Union, up.0, &[d]);
        k.release(d);
        u
    });
    k.release(up.0);
    let r = r?;
    Ok((r.first()?, r.history))
}

/// One way of [`extrude_drafted`]: along `dir` (frame z or −z).
fn drafted_half(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], height: f64, angle: f64, dir: Unit<Vector3<f64>>) -> Result<Made> {
    use crate::Pick;
    let body = k.extrude_with(&profile(xy_plane(frame), loops)?, &crate::ExtrudeSpec::blind(dir, height))?.first()?;
    let sides = k.faces_where(body, &|f| f.plane.is_none_or(|p| p.normal.dot(&dir).abs() < 0.5))?;
    if angle == 0.0 {
        return Ok((body, crate::History::default()));
    }
    let spec = crate::DraftSpec { faces: sides, angle, pull: dir, neutral: Point3::from(v3(frame.origin)), neutral_normal: None, tangent_propagation: true };
    let r = k.draft(body, &spec);
    k.release(body);
    let r = r?;
    Ok((r.first()?, r.history))
}

/// `loops` with each loop that is one whole circle's arc split into two halves at its start.
fn circles_halved<'a>(loops: &[&[Seg<'a>]]) -> Vec<Vec<Seg<'a>>> {
    loops
        .iter()
        .map(|l| match *l {
            [Seg::Arc { center, radius, start, sweep }] if sweep.abs() >= std::f64::consts::TAU => {
                let (center, radius, start, h) = (*center, *radius, *start, *sweep / 2.0);
                vec![Seg::Arc { center, radius, start, sweep: h }, Seg::Arc { center, radius, start: start + h, sweep: h }]
            }
            _ => l.to_vec(),
        })
        .collect()
}

/// The solid ruled (lofted straight) from the profile in `bottom`'s xy plane to the one in
/// `top`'s: each section in its own frame, so a top frame turned about the axis twists the
/// rulings (cadrs's loft of two sections, as cadrs's runner rules geometry/hyperboloid-ruled).
pub fn ruled(k: &mut DefaultKernel, bottom: &Place, bottom_loops: &[&[Seg]], top: &Place, top_loops: &[&[Seg]]) -> Result<Made> {
    // A whole circle drawn as one arc has no vertex to rule from: cadrs's loft then pairs its
    // two circles by where they lie, not by their frames' x. Halved at its start (each frame's
    // local angle of the arc's start), the rulings run from start to start.
    let (b, t) = (circles_halved(bottom_loops), circles_halved(top_loops));
    let (b, t): (Vec<&[Seg]>, Vec<&[Seg]>) = (b.iter().map(Vec::as_slice).collect(), t.iter().map(Vec::as_slice).collect());
    let spec = LoftSpec::through(vec![profile(xy_plane(bottom), &b)?, profile(xy_plane(top), &t)?]);
    let r = k.loft_with(&spec)?;
    Ok((r.first()?, r.history))
}

/// The solid under the patch filling the closed chain of `edges` (cadrs's fill: a Coons-like
/// surface on the boundary), down to `frame`'s xy plane: each edge lies in a plane holding
/// frame z, and its wall drops straight down from it to that floor (as cadrs's runner builds
/// fill/coons-patch-of-four-parabolas: the fill, flat walls, a flat floor, sewn).
pub fn fill_coons(k: &mut DefaultKernel, frame: &Place, edges: &[Spline]) -> Result<Made> {
    if edges.len() < 2 {
        return Err(bad("a fill needs a closed chain of two edges or more"));
    }
    let (o, z) = (v3(frame.origin), v3(frame.z));
    let mut patch_curves = Vec::with_capacity(edges.len());
    let mut walls = Vec::with_capacity(edges.len());
    let mut floor = Vec::with_capacity(edges.len());
    for e in edges {
        let (Some(&a), Some(&b)) = (e.ctrl.first(), e.ctrl.last()) else {
            return Err(bad("a fill edge with no control points"));
        };
        let (a, b) = (v3(a), v3(b));
        // Its plane: along the edge's run seen from above, and frame z.
        let run = (b - a) - z * (b - a).dot(&z);
        let x = Unit::try_new(run, 0.0).ok_or_else(|| bad("a fill edge runs straight up"))?.into_inner();
        let base = a - z * (a - o).dot(&z);
        if e.ctrl.iter().any(|&c| (v3(c) - base).dot(&x.cross(&z)).abs() > crate::native::tol::POINT) {
            return Err(bad("a fill edge does not lie in an upright plane"));
        }
        let plane = Plane::new(Point3::from(base), x, x.cross(&z));
        let to2 = |p: Vector3<f64>| Point2::new((p - base).dot(&x), (p - base).dot(&z));
        let curve = Curve2::Spline {
            degree: e.degree,
            knots: e.knots.to_vec(),
            poles: e.ctrl.iter().map(|&c| to2(v3(c))).collect(),
            weights: e.weights.map(<[f64]>::to_vec).unwrap_or_default(),
            source: None,
        };
        let (pa, pb) = (to2(a), to2(b));
        let line = |p: Point2<f64>, q: Point2<f64>| Curve2::Line { a: p, b: q, source: None };
        let (fa, fb) = (Point2::new(pa.x, 0.0), Point2::new(pb.x, 0.0));
        walls.push(vec![(plane, curve.clone()), (plane, line(pb, fb)), (plane, line(fb, fa)), (plane, line(fa, pa))]);
        patch_curves.push((plane, curve));
        floor.push(base);
    }
    let fp = Plane::new(Point3::from(o), v3(frame.x), z);
    let fx = v3(frame.x);
    let fy = z.cross(&fx);
    let f2 = |p: Vector3<f64>| Point2::new((p - o).dot(&fx), (p - o).dot(&fy));
    let n = floor.len();
    let floor_curves: Vec<(Plane, Curve2)> =
        (0..n).map(|i| (fp, Curve2::Line { a: f2(floor[i]), b: f2(floor[(i + 1) % n]), source: None })).collect();
    let mut sheets = Vec::with_capacity(n + 2);
    let r = (|| {
        sheets.push(k.fill(&crate::FillSpec::sketched(&patch_curves))?.first()?);
        for w in &walls {
            sheets.push(k.fill(&crate::FillSpec::sketched(w))?.first()?);
        }
        sheets.push(k.fill(&crate::FillSpec::sketched(&floor_curves))?.first()?);
        let r = k.sew_solid(&sheets, crate::native::tol::POINT)?;
        Ok((r.first()?, r.history))
    })();
    for sh in sheets {
        k.release(sh);
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
    let path = pieces_path(frame, pieces)?;
    let r = k.sweep_with(&profile(xy_plane(frame), loops)?, &SweepSpec::solid(path))?;
    Ok((r.first()?, r.history))
}

/// [`sweep`], the profile turning `twist` radians about the path and scaling to `scale` times
/// its size by the path's end, both evenly in the length travelled (cadrs's sweep law, as
/// cadrs's runner narrows sweep/narrowing-bend's square round its bend).
pub fn sweep_law(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], pieces: &[Piece], twist: f64, scale: f64) -> Result<Made> {
    if !(scale > 0.0) || !twist.is_finite() {
        return Err(bad("a sweep law needs a positive scale and a finite twist"));
    }
    let path = pieces_path(frame, pieces)?;
    let law = SweepLaw { twist, scale, spiral: false };
    let r = k.sweep_with_law(&profile(xy_plane(frame), loops)?, &SweepSpec::solid(path), &law)?;
    Ok((r.first()?, r.history))
}

/// The profile in `frame`'s xy plane swept along the helix about the line through
/// `axis_point` along `axis` that starts at the frame's origin, rising `pitch` a turn for
/// `turns` turns (right-handed about `axis` when positive): cadrs's exact helix path (its
/// pieces lie on the helix's cylinder), as cadrs's runner sweeps sweep/helical-pipe.
pub fn sweep_helix(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], axis_point: [f64; 3], axis: [f64; 3], pitch: f64, turns: f64) -> Result<Made> {
    let a = Unit::try_new(v3(axis), 0.0).ok_or_else(|| bad("a helix's axis is zero"))?;
    let o = v3(frame.origin);
    let foot = v3(axis_point) + a.into_inner() * (o - v3(axis_point)).dot(&a);
    let radial = o - foot;
    let x = Unit::try_new(radial, 0.0).ok_or_else(|| bad("a helix's start lies on its axis"))?;
    if turns == 0.0 || !pitch.is_finite() {
        return Err(bad("a helix needs turns and a finite pitch"));
    }
    let mut helix = crate::Helix::new(Axis { origin: Point3::from(foot), dir: a }, radial.norm(), pitch, 0.0, turns);
    helix.x = x;
    let r = k.sweep_with(&profile(xy_plane(frame), loops)?, &SweepSpec::solid(helix.path()))?;
    Ok((r.first()?, r.history))
}

/// The conical helix about the line through `origin` along `axis`, its radius going from `r0`
/// to `r1` as it rises `height`, from the angle `start` (radians, from `x`, square to the axis)
/// over `turns` turns: as cadrs draws one on a cone (sixteen quintic pieces a turn, each
/// matching the helix's points and first and second derivatives at its ends), here as one
/// clamped quintic B-spline (each joint a knot of multiplicity five: the pieces as they are):
/// knots, control points. A guide for
/// [`loft_guided`] (cadrs's runner's loft/twisted-tapering-square).
#[allow(clippy::too_many_arguments)]
pub fn conical_helix(origin: [f64; 3], axis: [f64; 3], x: [f64; 3], r0: f64, r1: f64, height: f64, start: f64, turns: f64) -> Result<(Vec<f64>, Vec<[f64; 3]>)> {
    let a = Unit::try_new(v3(axis), 0.0).ok_or_else(|| bad("a helix's axis is zero"))?;
    let xs = v3(x) - a.into_inner() * v3(x).dot(&a);
    let xs = Unit::try_new(xs, 0.0).ok_or_else(|| bad("a helix's x lies along its axis"))?;
    if turns == 0.0 || !turns.is_finite() {
        return Err(bad("a helix needs turns"));
    }
    let mut h = crate::Helix::conical(Axis { origin: Point3::from(v3(origin)), dir: a }, r0, r1, height, start, turns);
    h.x = xs;
    let n = ((turns.abs() * 16.0).ceil() as usize).max(1);
    let mut ctrl: Vec<[f64; 3]> = Vec::with_capacity(5 * n + 1);
    let mut knots = vec![0.0; 6];
    for k in 0..n {
        let (s0, s1) = (k as f64 / n as f64, (k + 1) as f64 / n as f64);
        let d = s1 - s0;
        let (p0, d0, a0) = h.at2(s0);
        let (p5, d5, a5) = h.at2(s1);
        let piece = [
            p0,
            p0 + d0 * (d / 5.0),
            p0 + d0 * (2.0 * d / 5.0) + a0 * (d * d / 20.0),
            p5 - d5 * (2.0 * d / 5.0) + a5 * (d * d / 20.0),
            p5 - d5 * (d / 5.0),
            p5,
        ];
        // Each piece's first point is the last one's end: one pole for both.
        let from = if k == 0 { 0 } else { 1 };
        ctrl.extend(piece[from..].iter().map(|p| [p.x, p.y, p.z]));
        if k + 1 < n {
            knots.extend([s1; 5]);
        }
    }
    knots.extend([1.0; 6]);
    Ok((knots, ctrl))
}

/// The path of `pieces` from `frame`'s origin along its z, in its xz plane (positive arcs
/// turning toward its x): sketch curves.
fn pieces_path(frame: &Place, pieces: &[Piece]) -> Result<Vec<PathCurve>> {
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
    Ok(path)
}

/// The profile in `frame`'s xy plane swept along the polyline through `points` (in world
/// coordinates; it starts at the first), mitred at each
/// corner: each leg runs on to the plane bisecting the corner and is cut there, the section
/// keeping its angle to the path (cadrs's sweep along sketch lines, as cadrs's runner sweeps
/// sweep/mitered-*).
pub fn sweep_mitered(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], points: &[[f64; 3]]) -> Result<Made> {
    if points.len() < 2 {
        return Err(bad("a mitred sweep needs a path of two points or more"));
    }
    mitred_run(k, frame, loops, points)
}

/// [`sweep_mitered`] round the closed polyline through `points`, back to the first and mitred
/// there too.
pub fn sweep_mitered_closed(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], points: &[[f64; 3]]) -> Result<Made> {
    if points.len() < 3 {
        return Err(bad("a closed mitred sweep needs three points or more"));
    }
    // Closed: as cadrs's runner sweeps a picture frame, two open halves from the middle of the
    // first run and of the run halfway round, united: every corner mitred alike.
    let n = points.len();
    let h = n / 2;
    let mid = |i: usize| {
        let (a, b) = (v3(points[i]), v3(points[(i + 1) % n]));
        let m = (a + b) / 2.0;
        [m.x, m.y, m.z]
    };
    let first: Vec<[f64; 3]> = std::iter::once(mid(0)).chain(points[1..=h].iter().copied()).chain([mid(h)]).collect();
    let second: Vec<[f64; 3]> = std::iter::once(mid(h)).chain(points[h + 1..].iter().copied()).chain([points[0], mid(0)]).collect();
    // The section carried to each half's start: moved along the first run to its middle, then
    // round the corners up to run h by the turn each corner makes (the least turn taking one
    // run's direction to the next, as the sweep carries it).
    let at = |f: &Place, by: Vector3<f64>| Place { origin: [f.origin[0] + by.x, f.origin[1] + by.y, f.origin[2] + by.z], ..*f };
    let start = at(frame, v3(mid(0)) - v3(points[0]));
    let mut r = Matrix3::identity();
    for i in 1..=h {
        let d0 = (v3(points[i]) - v3(points[i - 1])).normalize();
        let d1 = (v3(points[(i + 1) % n]) - v3(points[i])).normalize();
        let turn = nalgebra::Rotation3::rotation_between(&d0, &d1).ok_or_else(|| bad("a mitred sweep turns back on itself"))?;
        r = turn.into_inner() * r;
    }
    let o = v3(mid(h)) + r * (v3(start.origin) - v3(mid(0)));
    let rot = |v: [f64; 3]| {
        let w = r * v3(v);
        [w.x, w.y, w.z]
    };
    let half = Place { origin: [o.x, o.y, o.z], x: rot(start.x), y: rot(start.y), z: rot(start.z) };
    let (a, _) = mitred_run(k, &start, loops, &first)?;
    let b = mitred_run(k, &half, loops, &second);
    let r = b.and_then(|(b, _)| {
        let u = k.boolean(crate::BoolOp::Union, a, &[b]);
        k.release(b);
        u
    });
    k.release(a);
    let r = r?;
    Ok((r.first()?, r.history))
}

/// [`sweep_mitered`] along an open polyline.
fn mitred_run(k: &mut DefaultKernel, frame: &Place, loops: &[&[Seg]], points: &[[f64; 3]]) -> Result<Made> {
    let mut path = Vec::with_capacity(points.len() - 1);
    for w in points.windows(2) {
        let (a, b) = (v3(w[0]), v3(w[1]));
        if (b - a).norm() == 0.0 {
            return Err(bad("a mitred sweep's path has a leg of no length"));
        }
        // As a sketch line on a plane holding it (cadrs's runner draws its legs so).
        let d = b - a;
        let side = if d.x.abs() <= d.y.abs().max(d.z.abs()) { Vector3::x() } else { Vector3::y() };
        let plane = Plane::new(Point3::from(a), d, d.cross(&side));
        path.push(PathCurve::Sketch { plane, curve: Curve2::Line { a: Point2::new(0.0, 0.0), b: Point2::new(d.norm(), 0.0), source: None } });
    }
    let r = k.sweep_with(&profile(xy_plane(frame), loops)?, &SweepSpec::solid(path))?;
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
    let out = r.first()?;
    let made = flat_faces_as_planes(k, (out, r.history))?;
    if made.0 != out {
        k.release(out);
    }
    Ok(made)
}

/// `made` with each B-spline face that is flat (a ruled loft's side between two parallel lines)
/// made the plane it lies on, its neighbours re-trimmed to it: cadrs cuts and sweeps a body of
/// planes by its polyhedral paths exactly, a body with B-spline sides only as a mesh (a hypoid's
/// lofted tooth). Kept only where the volume is the same; the body as it was otherwise. A new
/// body leaves `out` to the caller to release.
pub(crate) fn flat_faces_as_planes(k: &mut DefaultKernel, (out, history): Made) -> Result<Made> {
    use crate::native::surface::Surface;
    let swaps: Vec<crate::FaceReplacement> = {
        let b = k.get(out)?;
        b.faces
            .iter()
            .enumerate()
            .filter_map(|(i, f)| match &f.surf {
                Surface::Nurbs(n) => match crate::native::recognize::surface(n) {
                    Some(p @ Surface::Plane { .. }) => Some(crate::FaceReplacement { face: crate::FaceId(i as u64), surface: p, domain: None }),
                    _ => None,
                },
                _ => None,
            })
            .collect()
    };
    if swaps.is_empty() {
        return Ok((out, history));
    }
    let v0 = k.mass_properties(out)?.volume;
    let Ok(r) = k.replace_faces(out, &swaps).and_then(|r| r.first()) else { return Ok((out, history)) };
    let v1 = k.mass_properties(r)?.volume;
    if (v1 - v0).abs() > crate::native::tol::REL_POINT * v0.abs().max(1.0) {
        k.release(r);
        return Ok((out, history));
    }
    Ok((r, history))
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
    fn drafted_extrudes_ruled_lofts_and_a_coons_fill() {
        let mut k = DefaultKernel::new();
        let sq = [
            Seg::Line { a: [-5.0, -5.0], b: [5.0, -5.0] },
            Seg::Line { a: [5.0, -5.0], b: [5.0, 5.0] },
            Seg::Line { a: [5.0, 5.0], b: [-5.0, 5.0] },
            Seg::Line { a: [-5.0, 5.0], b: [-5.0, -5.0] },
        ];
        // A square drafted 5° in over 5: the frustum.
        let a = 5f64.to_radians();
        let frustum = |top: f64| 5.0 / 3.0 * (100.0 + top * top + 10.0 * top);
        let (d, _) = extrude_drafted(&mut k, &world(), &[&sq], 5.0, a, false).unwrap();
        let v = k.mass_properties(d).unwrap().volume;
        assert!((v - frustum(10.0 - 10.0 * a.tan())).abs() < 1e-6 * v, "{v}");
        // Both ways: two frustums back to back.
        let (d, _) = extrude_drafted(&mut k, &world(), &[&sq], 5.0, a, true).unwrap();
        let v = k.mass_properties(d).unwrap().volume;
        assert!((v - 2.0 * frustum(10.0 - 10.0 * a.tan())).abs() < 1e-6 * v, "{v}");
        // brep-bench swept/drafted-slot-screwed-through-rod's bar: a slot growing 3° over 8, the
        // runner's loft of its two offset sections.
        let shrink = 8.0 * 3f64.to_radians().tan();
        let slot = |r: f64| {
            vec![
                Seg::Line { a: [-1.5, -r], b: [1.5, -r] },
                Seg::Arc { center: [1.5, 0.0], radius: r, start: -std::f64::consts::FRAC_PI_2, sweep: std::f64::consts::PI },
                Seg::Line { a: [1.5, r], b: [-1.5, r] },
                Seg::Arc { center: [-1.5, 0.0], radius: r, start: std::f64::consts::FRAC_PI_2, sweep: std::f64::consts::PI },
            ]
        };
        let at = |c: f64| Place { origin: [c, 0.0, 0.0], x: [0.0, 1.0, 0.0], y: [0.0, 0.0, 1.0], z: [1.0, 0.0, 0.0] };
        let (small, big) = (slot(1.0 - shrink), slot(1.0));
        let (bar, _) = extrude_drafted(&mut k, &at(4.0), &[&small], 8.0, -3f64.to_radians(), false).unwrap();
        let (lofted, _) = ruled(&mut k, &at(4.0), &[&small], &at(12.0), &[&big]).unwrap();
        let (vb, vl) = (k.mass_properties(bar).unwrap().volume, k.mass_properties(lofted).unwrap().volume);
        assert!((vb - vl).abs() < 1e-6 * vl, "{vb} against {vl}");
        // brep-bench geometry/hyperboloid-ruled: two circles of 5, 10 apart, a quarter turn
        // between their frames: 523.5987755982987 (cadrs's runner).
        let rim = [
            Seg::Arc { center: [0.0, 0.0], radius: 5.0, start: 0.0, sweep: std::f64::consts::PI },
            Seg::Arc { center: [0.0, 0.0], radius: 5.0, start: std::f64::consts::PI, sweep: std::f64::consts::PI },
        ];
        let lower = Place { origin: [0.0, 0.0, -5.0], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
        let upper = Place { origin: [0.0, 0.0, 5.0], x: [0.0, 1.0, 0.0], y: [-1.0, 0.0, 0.0], z: [0.0, 0.0, 1.0] };
        let (h, _) = ruled(&mut k, &lower, &[&rim], &upper, &[&rim]).unwrap();
        let v = k.mass_properties(h).unwrap().volume;
        assert!((v - 523.5987755982987).abs() < 1e-6 * v, "{v}");
        // Drawn as one whole arc (as the facade's Profile::circle): the same.
        let one = [Seg::Arc { center: [0.0, 0.0], radius: 5.0, start: 0.0, sweep: std::f64::consts::TAU }];
        let (h, _) = ruled(&mut k, &lower, &[&one], &upper, &[&one]).unwrap();
        let v = k.mass_properties(h).unwrap().volume;
        assert!((v - 523.5987755982987).abs() < 1e-6 * v, "{v}");
        // brep-bench fill/coons-patch-of-four-parabolas: 1000 (cadrs's runner).
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let arcs = [
            [[0.0, 0.0, 10.0], [5.0, 0.0, 14.0], [10.0, 0.0, 10.0]],
            [[10.0, 0.0, 10.0], [10.0, 5.0, 6.0], [10.0, 10.0, 10.0]],
            [[10.0, 10.0, 10.0], [5.0, 10.0, 6.0], [0.0, 10.0, 10.0]],
            [[0.0, 10.0, 10.0], [0.0, 5.0, 14.0], [0.0, 0.0, 10.0]],
        ];
        let edges: Vec<Spline> = arcs.iter().map(|c| Spline { degree: 2, knots: &knots, ctrl: c, weights: None }).collect();
        let (c, _) = fill_coons(&mut k, &world(), &edges).unwrap();
        let v = k.mass_properties(c).unwrap().volume;
        assert!((v - 1000.0).abs() < 1e-6 * v, "{v}");
    }

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Seg<'static>> {
        let c = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
        (0..4).map(|i| Seg::Line { a: c[i], b: c[(i + 1) % 4] }).collect()
    }

    #[test]
    fn a_pipe_along_a_helix() {
        // brep-bench sweep/helical-pipe: a circle of 1 along the helix of 10 rising 5 a turn,
        // three turns: cadrs's runner 594.0483049737906.
        let mut k = DefaultKernel::new();
        let t = Vector3::new(0.0, 10.0, 5.0 / std::f64::consts::TAU).normalize();
        let x = Vector3::z().cross(&t).normalize();
        let y = t.cross(&x);
        let at = Place { origin: [10.0, 0.0, 0.0], x: [x.x, x.y, x.z], y: [y.x, y.y, y.z], z: [t.x, t.y, t.z] };
        let disc = [Seg::Arc { center: [0.0, 0.0], radius: 1.0, start: 0.0, sweep: std::f64::consts::TAU }];
        let (p, _) = sweep_helix(&mut k, &at, &[&disc], [0.0; 3], [0.0, 0.0, 1.0], 5.0, 3.0).unwrap();
        let v = k.mass_properties(p).unwrap().volume;
        assert!((v - 594.0483049737906).abs() < 1e-6 * v, "{v}");
    }

    #[test]
    fn a_square_narrowing_round_a_bend() {
        // brep-bench sweep/narrowing-bend: a square narrowing from 18 to 12 round a quarter turn
        // of radius 30, less one 4 narrower from -1 to 91 degrees: cadrs's runner
        // 4900.884541078331.
        let mut k = DefaultKernel::new();
        let along = |k: &mut DefaultKernel, start: f64, end: f64, inset: f64| {
            let h = |deg: f64| 9.0 - 3.0 * deg / 90.0 - inset;
            let (s, c) = start.to_radians().sin_cos();
            let at = Place { origin: [30.0 * c, 30.0 * s, 0.0], x: [-c, -s, 0.0], y: [0.0, 0.0, 1.0], z: [-s, c, 0.0] };
            let hs = h(start);
            let sq = rect(-hs, -hs, hs, hs);
            sweep_law(k, &at, &[&sq], &[Piece::Arc { radius: 30.0, angle: (end - start).to_radians() }], 0.0, h(end) / hs).unwrap().0
        };
        let outside = along(&mut k, 0.0, 90.0, 0.0);
        let hole = along(&mut k, -1.0, 91.0, 2.0);
        let cut = k.boolean(crate::BoolOp::Subtract, outside, &[hole]).and_then(|r| r.first()).unwrap();
        let v = k.mass_properties(cut).unwrap().volume;
        assert!((v - 4900.884541078331).abs() < 1e-6 * v, "{v}");
    }

    #[test]
    fn lofts_on_conical_helices_and_to_a_point() {
        // brep-bench loft/twisted-tapering-square: a square of 30 lofted to one of 10, 40 up,
        // along four conical helices through its corners, half a turn: cadrs's runner
        // 17333.33208078546.
        let mut k = DefaultKernel::new();
        let r = 2f64.sqrt();
        let guides: Vec<(Vec<f64>, Vec<[f64; 3]>)> = [45.0f64, 135.0, 225.0, 315.0]
            .iter()
            .map(|a| conical_helix([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 15.0 * r, 5.0 * r, 40.0, a.to_radians(), 0.5).unwrap())
            .collect();
        let gs: Vec<Spline> = guides.iter().map(|(kn, c)| Spline { degree: 5, knots: kn, ctrl: c, weights: None }).collect();
        let (foot, head) = (rect(-15.0, -15.0, 15.0, 15.0), rect(-5.0, -5.0, 5.0, 5.0));
        let top = Place { origin: [0.0, 0.0, 40.0], ..world() };
        let (l, _) = loft_guided(&mut k, &[(world(), &[&foot]), (top, &[&head])], &gs).unwrap();
        let v = k.mass_properties(l).unwrap().volume;
        assert!((v - 17333.33208078546).abs() < 1e-6 * v, "{v}");
        // loft/square-spire-on-four-guides: a square of 18 to the apex (0, 0, 24) along four
        // cubic ridges: cadrs's runner 9423.771428571437.
        let apex = [0.0, 0.0, 24.0];
        let ridges: Vec<[[f64; 3]; 4]> = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
            .iter()
            .map(|&(sx, sy)| [[9.0 * sx, 9.0 * sy, 0.0], [15.0 * sx, 15.0 * sy, 8.0], [12.0 * sx, 12.0 * sy, 19.0], apex])
            .collect();
        let cubic = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let gs: Vec<Spline> = ridges.iter().map(|c| Spline { degree: 3, knots: &cubic, ctrl: c, weights: None }).collect();
        let square = rect(-9.0, -9.0, 9.0, 9.0);
        let (sp, _) = loft_guided_to(&mut k, &[Section::Profile(world(), &[&square]), Section::Point(apex)], &gs).unwrap();
        let v = k.mass_properties(sp).unwrap().volume;
        assert!((v - 9423.771428571437).abs() < 1e-6 * v, "{v}");
    }

    #[test]
    fn mitred_sweeps_along_polylines() {
        // brep-bench sweep/mitered-elbow (8000) and mitered-pipe-on-cube-edges (1130.97335529),
        // cadrs's runner's values.
        let mut k = DefaultKernel::new();
        let sq = [
            Seg::Line { a: [-5.0, -5.0], b: [5.0, -5.0] },
            Seg::Line { a: [5.0, -5.0], b: [5.0, 5.0] },
            Seg::Line { a: [5.0, 5.0], b: [-5.0, 5.0] },
            Seg::Line { a: [-5.0, 5.0], b: [-5.0, -5.0] },
        ];
        // The yz plane at the origin: x along y, y along z, the path along x.
        let yz = Place { origin: [0.0; 3], x: [0.0, 1.0, 0.0], y: [0.0, 0.0, 1.0], z: [1.0, 0.0, 0.0] };
        let (e, _) = sweep_mitered(&mut k, &yz, &[&sq], &[[0.0; 3], [40.0, 0.0, 0.0], [20.0, 20.0 * 3f64.sqrt(), 0.0]]).unwrap();
        let v = k.mass_properties(e).unwrap().volume;
        assert!((v - 8000.0).abs() < 1e-6 * v, "{v}");
        let ring = [Seg::Arc { center: [0.0, 0.0], radius: 2.0, start: 0.0, sweep: std::f64::consts::TAU }];
        let (p, _) = sweep_mitered(&mut k, &yz, &[&ring], &[[0.0; 3], [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], [30.0, 30.0, 30.0]]).unwrap();
        let v = k.mass_properties(p).unwrap().volume;
        assert!((v - 1130.9733552923235).abs() < 1e-6 * v, "{v}");
        // mitered-picture-frame (9864): the rabbeted molding round the closed rectangle
        // (±30, ±20), lying outward; its frame on the first corner, z along the first run.
        let molding: Vec<[f64; 2]> = [(0.0, 2.0), (3.0, 2.0), (3.0, 0.0), (8.0, 0.0), (8.0, 6.0), (0.0, 6.0)].iter().map(|&(s, z)| [-s, z]).collect();
        let segs: Vec<Seg> = (0..6).map(|i| Seg::Line { a: molding[i], b: molding[(i + 1) % 6] }).collect();
        let at = Place { origin: [-30.0, -20.0, 0.0], x: [0.0, 1.0, 0.0], y: [0.0, 0.0, 1.0], z: [1.0, 0.0, 0.0] };
        let corners = [[-30.0, -20.0, 0.0], [30.0, -20.0, 0.0], [30.0, 20.0, 0.0], [-30.0, 20.0, 0.0]];
        let (f, _) = sweep_mitered_closed(&mut k, &at, &[&segs], &corners).unwrap();
        let v = k.mass_properties(f).unwrap().volume;
        assert!((v - 9864.0).abs() < 1e-6 * v, "{v}");
    }

    #[test]
    fn the_coons_solid_sewn_from_fill_patches() {
        // fill/coons-patch-of-four-parabolas as make::sewn would build it: the patch on its four
        // parabolas, four walls under them and the floor, each a Fill of its boundary: 1000.
        let mut k = DefaultKernel::new();
        let q = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let l = [0.0, 0.0, 1.0, 1.0];
        let arcs = [
            [[0.0, 0.0, 10.0], [5.0, 0.0, 14.0], [10.0, 0.0, 10.0]],
            [[10.0, 0.0, 10.0], [10.0, 5.0, 6.0], [10.0, 10.0, 10.0]],
            [[10.0, 10.0, 10.0], [5.0, 10.0, 6.0], [0.0, 10.0, 10.0]],
            [[0.0, 10.0, 10.0], [0.0, 5.0, 14.0], [0.0, 0.0, 10.0]],
        ];
        let drop = |p: [f64; 3]| [p[0], p[1], 0.0];
        let lines: Vec<[[f64; 3]; 2]> = arcs.iter().flat_map(|a| [[a[2], drop(a[2])], [drop(a[2]), drop(a[0])], [drop(a[0]), a[0]]]).collect();
        let floor: Vec<[[f64; 3]; 2]> = arcs.iter().map(|a| [drop(a[0]), drop(a[2])]).collect();
        let top: Vec<Spline> = arcs.iter().map(|c| Spline { degree: 2, knots: &q, ctrl: c, weights: None }).collect();
        let walls: Vec<Vec<Spline>> = (0..4)
            .map(|i| {
                let mut w = vec![top[i]];
                w.extend(lines[3 * i..3 * i + 3].iter().map(|c| Spline { degree: 1, knots: &l, ctrl: c, weights: None }));
                w
            })
            .collect();
        let floor: Vec<Spline> = floor.iter().map(|c| Spline { degree: 1, knots: &l, ctrl: c, weights: None }).collect();
        let mut patches = vec![GluePatch::Fill { boundary: &top }];
        patches.extend(walls.iter().map(|w| GluePatch::Fill { boundary: w }));
        patches.push(GluePatch::Fill { boundary: &floor });
        let (c, _) = sewn(&mut k, &patches).unwrap();
        let v = k.mass_properties(c).unwrap().volume;
        assert!((v - 1000.0).abs() < 1e-6 * v, "{v}");
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
