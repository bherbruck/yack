// yack: A fillet along a tangent chain of edges no constant section sweeps: the surface a ball of the
// Adapted from springrep@a03e8a1 src/blend/rolling.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/blend/rolling.rs to yack-blend (M3 W9a); paths and visibility only;
// the fillet's build split out into rolling/build.rs (M3 hardening R0), paths and visibility only; the
// roll on checked access, its face-domain map a BTreeMap (looked up only), its small literals named (R1); a second section rule, square to the ball's own path, for an
// edge whose square planes cross within the ball's reach (M4 slant-cut offsets).
//! A fillet along a tangent chain of edges no constant section sweeps: the surface a ball of the
//! fillet's radius sweeps rolling along the chain in contact with the faces on both sides.
//!
//! The chain runs the way its side-0 face uses it, with one parameter `s` along it. At each `s` the
//! ball's centre lies in the plane square to the chain, at the radius from both sides, found by
//! Newton steps whose rows are the two surface normals and the chain's tangent. Where those planes
//! cross within the ball's reach (an outward offset's edge, past the curvature of the edge it grew
//! from) the sections fold back or find no centre; the chain is then rolled again with each centre
//! in the plane through the chain's point that holds both contact normals, square to the ball's own
//! path, by Newton steps on numerical rows. A side's contact is
//! the nearest point on whichever of that side's faces holds it: where a side changes face at a
//! joint, between faces that meet tangentially, the ball's contact slides from one onto the other,
//! and a foot counts only on its own face's side of the edge between them.
//!
//! An open chain ends where one other face, a plane, meets each end. Past an end the chain runs on
//! straight along its tangent there, so the ball keeps rolling between the same two surfaces, and
//! the blend is trimmed by that plane: for each `v` across it, `s*(v)` is where the true section
//! meets the plane, and the trim curve through those points is fitted like the blend.
//!
//! The blend is cut into segments at every chain vertex and wherever a contact curve crosses onto
//! the next face or over a face's seam, so each segment is smooth. Across, a segment is the exact
//! circular arc between the contacts, a rational quadratic; along, its control rows are interpolated
//! by a cubic in `s`, sampled more finely until every midway section and both contact curves lie
//! within a tenth of the linear tolerance of the true ones. Neighbouring segments share the section
//! arc between them, and a trim cuts each segment it reaches.
//!
//! The result is sewn by surgery: each face gives up the strip between the chain and its contact
//! curve, the edges and seams at the chain's vertices are cut back to where the contact curves cross
//! them, an end's plane gives up the corner the blend takes and gains its trim curve, and the
//! segments' faces fill the strip.

use std::collections::{BTreeMap, BTreeSet};

mod build;
#[cfg(test)]
mod tests;

use super::{Picked, outward, run};
pub(super) use self::build::fillet;
use crate::{
    Error, Result,
    topo::{
        domain::{Domain, Where},
        face_domain,
    },
    check,
    geom::{BSpline, Curve, NurbsCurve, NurbsSurface, Surface},
    math::{Point3, Vec2, Vec3},
    tol,
    topo::{Brep, CoedgeId, CoedgeSpec, EdgeId, FaceId, FaceName, PCurve, VertexId},
};

/// How far a sampled blend may stray from the true one, as a fraction of the linear tolerance.
const FIT: f64 = 0.1;
/// Section counts tried in turn for a segment; the last that still misses [`FIT`] refuses.
const SAMPLES: [usize; 10] = [8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096];

/// Chain parameters this close are one place on the chain.
const SAME_S: f64 = 1e-12; // tol: a chain parameter at rounding's scale, not a length

/// A sine, distance or determinant this near zero is degenerate.
const DEGENERATE: f64 = 1e-12; // tol: guards a division, not a length that decides anything

/// The least weight of a section arc's middle control point: below it the arc turns by a half turn.
const LEAST_WEIGHT: f64 = 1e-3; // tol: a weight, not a length

/// A section's Newton step this short, as a share of the tolerance, has settled.
const SETTLED: f64 = 1e-3; // tol: a share of the tolerance

/// The first secant step of an end's trim, as a share of the radius.
const FIRST_STEP: f64 = 1e-3; // tol: a share of the radius, not a length

/// A Newton step on a curve parameter this short, as a share of its size, has settled.
const SETTLED_PARAM: f64 = 1e-15; // tol: a share at rounding's scale, not a length

/// The step of a spine section's numerical rows, as a share of the radius.
const SPINE_STEP: f64 = 1e-6; // tol: a share of the radius, not a length

/// Why the edge rule refuses where its square planes find no centre.
const NO_SETTLE: &str = "whose ball's centre does not settle";

/// Why the edge rule refuses where its square planes fold the blend back on itself.
const FACING_IN: &str = "a rolling-ball fillet's face came out facing into the body";

/// The step of a curve's numerical second derivative, as a share of its range.
const DIFF_STEP: f64 = 1e-6; // tol: a share of a parameter range, not a length

/// The least parameter range a numerical derivative's step is a share of.
const LEAST_SPAN: f64 = 1e-9; // tol: a parameter range, not a length

/// A secant bracket this narrow, as a share of the parameter's size, has converged.
const CONVERGED: f64 = 1e-14; // tol: a share at rounding's scale, not a length

fn unsupported<T>(why: &str) -> Result<T> {
    Err(Error::Unsupported(format!("a rolling-ball fillet {why}")))
}

/// One cross-section: the ball's centre and its contact point and face on each side.
#[derive(Clone, Copy, Debug)]
struct Section {
    centre: Point3,
    contact: [Point3; 2],
    face: [FaceId; 2],
}

impl Section {
    /// The rational quadratic arc from contact 0 to contact 1: its middle control point and weight.
    fn arc(&self, radius: f64) -> Result<(Point3, f64)> {
        let (a, b) = (self.contact[0] - self.centre, self.contact[1] - self.centre);
        let half = 0.5 * a.cross(b).norm().atan2(a.dot(b));
        let w = half.cos();
        if w <= LEAST_WEIGHT {
            return unsupported("whose faces turn by nearly a half turn");
        }
        let Some(mid) = (a + b).unit() else {
            return unsupported("whose faces turn by a half turn");
        };
        Ok((self.centre + mid * (radius / w), w))
    }

    /// The arc's point at `v` in [0, 1].
    fn at(&self, radius: f64, v: f64) -> Result<Point3> {
        let (m, w) = self.arc(radius)?;
        let (b0, b1, b2) = ((1.0 - v) * (1.0 - v), 2.0 * v * (1.0 - v) * w, v * v);
        Ok((self.contact[0] * b0 + m * b1 + self.contact[1] * b2) / (b0 + b1 + b2))
    }

    fn arc_curve(&self, radius: f64) -> Result<NurbsCurve> {
        let (mid, w) = self.arc(radius)?;
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![self.contact[0], mid, self.contact[1]],
            Some(vec![1.0, w, 1.0]),
        )
    }
}

/// Which plane a section's centre lies in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Rule {
    /// Square to the chain at its point: the first rule.
    Edge,
    /// Through the chain's point and both contact normals, square to the ball's own path: for a
    /// chain whose square planes cross within the ball's reach.
    Spine,
}

/// One picked edge in the chain, `s` running from `s0` for `len` along it.
struct Link {
    pk: usize,
    edge: EdgeId,
    forward: bool,
    faces: [FaceId; 2],
    s0: f64,
    len: f64,
}

/// An edge at a chain vertex between the two faces a side has on either side of it, or a seam of
/// the one face it has on both, with which way is into each.
struct Junction {
    edge: EdgeId,
    at: Point3,
    /// The face before the vertex along the chain, and the face after it.
    before: FaceId,
    after: FaceId,
    seam: bool,
}

/// A free end of an open chain: the vertex, the one other face there, and that face's edges at the
/// vertex with each side's face.
struct End {
    vertex: VertexId,
    face: FaceId,
    surface: Surface,
    /// +1 or −1, so [`End::side`] is positive on the side the blend keeps.
    sign: f64,
    edges: [EdgeId; 2],
    /// `s*` at `v` = 0, 1 and the extreme values between.
    lo: f64,
    hi: f64,
}

impl End {
    /// How far `p` lies from the end face's surface, along its normal, positive on the kept side.
    fn side(&self, p: Point3) -> f64 {
        let uv = self.surface.project(p);
        let q = self.surface.eval(uv.x, uv.y);
        self.sign * (p - q).dot(self.surface.normal(uv.x, uv.y))
    }
}

/// The chain and the ball.
struct Roll<'a> {
    body: &'a Brep,
    links: Vec<Link>,
    closed: bool,
    total: f64,
    radius: f64,
    /// −1 where the ball lies inside the body (a convex chain), +1 outside it.
    side: f64,
    /// `joints[i][k]`: the junction on side `k` where link `i` ends and the next begins.
    joints: Vec<[Option<Junction>; 2]>,
    /// The trimmed domain of every face a side has, to tell whether a foot lies in the face.
    domains: BTreeMap<FaceId, Domain>,
    /// The plane each section lies in.
    rule: Rule,
}

impl Roll<'_> {
    fn wrap(&self, s: f64) -> f64 {
        if self.closed {
            s.rem_euclid(self.total)
        } else {
            s
        }
    }

    fn link_at(&self, s: f64) -> usize {
        let s = self.wrap(s);
        self.links
            .iter()
            .rposition(|l| l.s0 <= s + SAME_S)
            .unwrap_or(0)
    }

    /// The point and unit tangent of link `i`'s edge `d` along it from its start in `s`.
    /// Link `i` of the chain.
    fn link(&self, i: usize) -> Result<&Link> {
        self.links.get(i).ok_or_else(|| Error::Numerical("a rolling chain's link that is not there".into()))
    }

    /// The junction on side `k` at the end of link `i`, if there is one.
    fn joint(&self, i: usize, k: usize) -> Option<&Junction> {
        self.joints.get(i).and_then(|j| run::of_face(j, k).as_ref())
    }

    fn on_link(&self, i: usize, d: f64) -> Result<(Point3, Vec3)> {
        let l = self.link(i)?;
        let e = self.body.edge(l.edge);
        let t = if l.forward {
            e.range.0 + d
        } else {
            e.range.1 - d
        };
        let Some(curve) = e.curve.as_ref() else {
            return unsupported("on an edge with no curve");
        };
        let (p, dt) = curve.d1(t);
        let Some(along) = (if l.forward { dt } else { -dt }).unit() else {
            return unsupported("on an edge with no tangent");
        };
        Ok((p, along))
    }

    /// The chain's point and unit tangent at `s`, and the link it lies on. Past an open chain's
    /// end it runs on straight along its tangent there.
    fn chain_at(&self, s: f64) -> Result<(Point3, Vec3, usize)> {
        let s = self.wrap(s);
        let (Some(last), Some(last_link)) = (self.links.len().checked_sub(1), self.links.last()) else {
            return unsupported("along no edges");
        };
        if !self.closed && s < 0.0 {
            let (p, d) = self.on_link(0, 0.0)?;
            return Ok((p + d * s, d, 0));
        }
        if !self.closed && s > self.total {
            let (p, d) = self.on_link(last, last_link.len)?;
            return Ok((p + d * (s - self.total), d, last));
        }
        let i = self.link_at(s);
        let l = self.link(i)?;
        let (p, d) = self.on_link(i, (s - l.s0).clamp(0.0, l.len))?;
        Ok((p, d, i))
    }

    fn surface(&self, f: FaceId) -> &Surface {
        &self.body.face(f).surface
    }

    /// The nearest point to `q` on face `f`'s surface, and the outward normal there.
    fn foot_on(&self, f: FaceId, q: Point3) -> (Point3, Vec3) {
        let s = self.surface(f);
        let uv = s.project(q);
        let p = s.eval(uv.x, uv.y);
        (p, outward(self.body, f, p))
    }

    /// The junctions on side `k` at either end of link `i`.
    fn near(&self, i: usize, k: usize) -> Vec<&Junction> {
        let n = self.links.len();
        let mut out = Vec::new();
        if self.closed || i > 0 {
            out.extend(self.joint((i + n - 1) % n, k));
        }
        if self.closed || i + 1 < n {
            out.extend(self.joint(i, k));
        }
        out
    }

    /// Side `k`'s contact for a centre `q` near link `i`: the nearest foot on that side's faces
    /// about the link that lies in its face, by the face's own trimmed domain. Where none does (past
    /// an open chain's end, or where the strip runs off its face) the ball rolls on the link's own
    /// face's surface.
    fn foot(&self, i: usize, k: usize, q: Point3) -> Result<(Point3, Vec3, FaceId)> {
        let own = *run::of_face(&self.link(i)?.faces, k);
        let near = self.near(i, k);
        let (p, nrm) = self.foot_on(own, q);
        let in_face = |f: FaceId, p: Point3| {
            self.domains
                .get(&f)
                .is_some_and(|d| d.locate(p) != Where::Outside)
        };
        // A foot on the own face's surface can lie in another face only where the two surfaces meet,
        // along the curve between them: far from every such curve it is the contact.
        let far = near.iter().filter(|j| !j.seam).all(|j| {
            let x = self.body.edge(j.edge);
            match x.curve.as_ref() {
                Some(c @ (Curve::Line(_) | Curve::Circle(_))) => nearest_param(c, p, x.range)
                    .map(|t| c.eval(t).dist(p) > 4.0 * self.radius)
                    .unwrap_or(false),
                _ => false,
            }
        });
        // With no other face on this side about the link, or a foot in the link's own face, that
        // is the contact: a foot lies inside at most one face.
        if far || in_face(own, p) {
            return Ok((p, nrm, own));
        }
        let mut faces = vec![own];
        for j in near {
            for f in [j.before, j.after] {
                if !faces.contains(&f) {
                    faces.push(f);
                }
            }
        }
        let mut best: Option<(f64, Point3, Vec3, FaceId)> = None;
        for &f in &faces {
            if f == own {
                continue;
            }
            let (p, nrm) = self.foot_on(f, q);
            let inside = in_face(f, p);
            let d = p.dist(q);
            if inside && best.is_none_or(|b| d < b.0) {
                best = Some((d, p, nrm, f));
            }
        }
        Ok(match best {
            Some((_, p, nrm, f)) => (p, nrm, f),
            None => (p, nrm, own),
        })
    }

    /// The section at chain parameter `s`, starting Newton from `guess` or from the chain.
    fn section(&self, s: f64, guess: Option<Point3>) -> Result<Section> {
        if self.rule == Rule::Spine {
            return self.spine_section(s, guess);
        }
        let (e, along, i) = self.chain_at(s)?;
        let faces = self.link(i)?.faces;
        let mut c = guess.unwrap_or_else(|| {
            let n = outward(self.body, faces[0], e) + outward(self.body, faces[1], e);
            e + n * (self.side * self.radius)
        });
        for _ in 0..60 {
            let (pa, na, _) = self.foot(i, 0, c)?;
            let (pb, nb, _) = self.foot(i, 1, c)?;
            let f = [
                (c - pa).dot(na) - self.side * self.radius,
                (c - pb).dot(nb) - self.side * self.radius,
                (c - e).dot(along),
            ];
            let step = solve3([na, nb, along], f).ok_or_else(|| {
                Error::Unsupported("a rolling-ball fillet whose faces are parallel".into())
            })?;
            c -= step;
            if step.norm() <= SETTLED * tol::LINEAR {
                let (a, _, fa) = self.foot(i, 0, c)?;
                let (b, _, fb) = self.foot(i, 1, c)?;
                if a.dist(e) > 10.0 * self.radius || b.dist(e) > 10.0 * self.radius {
                    return unsupported("whose ball leaves the edge");
                }
                return Ok(Section {
                    centre: c,
                    contact: [a, b],
                    face: [fa, fb],
                });
            }
        }
        unsupported(NO_SETTLE)
    }

    /// The section at chain parameter `s` by [`Rule::Spine`]: the centre at the radius from both
    /// sides, in the plane through the chain's point that holds both contact normals. That plane
    /// turns with the feet, so the Newton rows are central differences. With no `guess` it starts
    /// where a ball in the wedge of the two tangent planes at the chain's point would sit.
    fn spine_section(&self, s: f64, guess: Option<Point3>) -> Result<Section> {
        let (e, _, i) = self.chain_at(s)?;
        let faces = self.link(i)?.faces;
        let reach = self.side * self.radius;
        let residual = |c: Point3| -> Result<Vec3> {
            let (pa, na, _) = self.foot(i, 0, c)?;
            let (pb, nb, _) = self.foot(i, 1, c)?;
            let Some(square) = na.cross(nb).unit() else {
                return unsupported("whose faces are parallel");
            };
            Ok(Vec3::new((c - pa).dot(na) - reach, (c - pb).dot(nb) - reach, (c - e).dot(square)))
        };
        let mut c = match guess {
            Some(g) => g,
            None => {
                let [f0, f1] = faces;
                let n = outward(self.body, f0, e) + outward(self.body, f1, e);
                let size = n.dot(n);
                if size <= DEGENERATE {
                    return unsupported("whose faces turn by a half turn");
                }
                // the bisector, as far out as a ball touching both tangent planes sits
                e + n * (2.0 * reach / size)
            }
        };
        let h = SPINE_STEP * self.radius;
        for _ in 0..60 {
            let f = residual(c)?;
            let [dx, dy, dz] = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| (residual(c + axis * h), residual(c - axis * h)));
            let column = |(p, m): (Result<Vec3>, Result<Vec3>)| -> Result<Vec3> { Ok((p? - m?) / (2.0 * h)) };
            let (cx, cy, cz) = (column(dx)?, column(dy)?, column(dz)?);
            let rows = [Vec3::new(cx.x, cy.x, cz.x), Vec3::new(cx.y, cy.y, cz.y), Vec3::new(cx.z, cy.z, cz.z)];
            let step = solve3(rows, [f.x, f.y, f.z]).ok_or_else(|| {
                Error::Unsupported("a rolling-ball fillet whose faces are parallel".into())
            })?;
            c -= step;
            if step.norm() <= SETTLED * tol::LINEAR {
                let (a, _, fa) = self.foot(i, 0, c)?;
                let (b, _, fb) = self.foot(i, 1, c)?;
                if a.dist(e) > 10.0 * self.radius || b.dist(e) > 10.0 * self.radius {
                    return unsupported("whose ball leaves the edge");
                }
                return Ok(Section {
                    centre: c,
                    contact: [a, b],
                    face: [fa, fb],
                });
            }
        }
        unsupported(NO_SETTLE)
    }

    /// Where, near the joint at the end of link `i`, side `k`'s contact crosses its junction.
    fn crossing(&self, i: usize, k: usize) -> Result<Option<f64>> {
        let Some(j) = self.joint(i, k) else {
            return Ok(None);
        };
        let n = self.links.len();
        let next = (i + 1) % n;
        let (here, there) = (self.link(i)?, self.link(next)?);
        let at = there.s0 + if next == 0 { self.total } else { 0.0 };
        // Up to four radii either side of the joint, in each link's own parameter.
        let speed = |l: &Link, d: f64| -> f64 {
            let e = self.body.edge(l.edge);
            let t = if l.forward {
                e.range.0 + d
            } else {
                e.range.1 - d
            };
            e.curve
                .as_ref()
                .map_or(1.0, |c| c.d1(t).1.norm())
                .max(DEGENERATE)
        };
        let back = here.len.min(4.0 * self.radius / speed(here, here.len));
        let fwd = there.len.min(4.0 * self.radius / speed(there, 0.0));
        let s = self.surface(j.before);
        let seam_u = s.project(j.at).x;
        // Zero where the contact reaches the junction: across a seam's parameter, or across the
        // curve between two faces.
        let g = |t: f64, guess: Option<Point3>| -> Result<(f64, Point3)> {
            let sec = self.section(t, guess)?;
            let d = if j.seam {
                let up = s.u_period().unwrap_or(f64::INFINITY);
                let u = s.project(*run::of_face(&sec.contact, k)).x;
                (u - seam_u + 0.5 * up).rem_euclid(up) - 0.5 * up
            } else {
                // The foot on the face before the junction, across the junction curve from it.
                let x = self.body.edge(j.edge);
                // a junction between two faces lies on a curve
                let Some(jc) = x.curve.as_ref() else {
                    return unsupported("whose junction has no curve");
                };
                let (p, _) = self.foot_on(j.before, sec.centre);
                let tq = nearest_param(jc, p, x.range)?;
                let (q, tj) = jc.d1(tq);
                let across = outward(self.body, j.before, q).cross(tj).unit_or_zero();
                (p - q).dot(across)
            };
            Ok((d, sec.centre))
        };
        let steps = 32;
        let mut guess = None;
        let mut prev: Option<(f64, f64)> = None;
        for m in 0..=steps {
            let t = at - back + (back + fwd) * m as f64 / steps as f64;
            let (d, c) = g(t, guess)?;
            guess = Some(c);
            if d.abs() <= DEGENERATE {
                return Ok(Some(t));
            }
            if let Some((tp, dp)) = prev {
                let wraps = j.seam && (d - dp).abs() > 1.0;
                if dp.signum() != d.signum() && !wraps {
                    return Ok(Some(bisect(tp, t, dp, |x| Ok(g(x, guess)?.0))?));
                }
            }
            prev = Some((t, d));
        }
        unsupported("whose contact never crosses onto the next face")
    }

    /// `s*(v)`: where the section's arc point at `v` meets end `e`'s plane, near `from`. Secant
    /// steps from `from`, each section's Newton started from the last centre; a search for a sign
    /// change and bisection where the secant strays.
    fn trim_at(&self, e: &End, v: f64, from: f64) -> Result<f64> {
        let guess = std::cell::Cell::new(None);
        let g = |s: f64| -> Result<f64> {
            let sec = self.section(s, guess.get())?;
            guess.set(Some(sec.centre));
            Ok(e.side(sec.at(self.radius, v)?))
        };
        let g0 = g(from)?;
        if g0 == 0.0 {
            return Ok(from);
        }
        let (mut a, mut ga) = (from, g0);
        let (mut b, mut gb) = (from + FIRST_STEP * self.radius, 0.0);
        gb = if b != a { g(b)? } else { gb };
        for _ in 0..40 {
            if gb == ga {
                break;
            }
            let c = b - gb * (b - a) / (gb - ga);
            if !c.is_finite() || (c - from).abs() > 20.0 * self.radius {
                break;
            }
            (a, ga) = (b, gb);
            b = c;
            gb = g(b)?;
            if gb == 0.0 || (b - a).abs() <= CONVERGED * (1.0 + b.abs()) {
                return Ok(b);
            }
        }
        let step = 0.25 * self.radius;
        for k in 1..=80 {
            for dir in [1.0, -1.0] {
                let (p, b) = (
                    from + dir * step * (k - 1) as f64,
                    from + dir * step * k as f64,
                );
                let gb = g(b)?;
                if gb.signum() != g0.signum() {
                    let gp = g(p)?;
                    return if p < b {
                        bisect(p, b, gp, g)
                    } else {
                        bisect(b, p, gb, g)
                    };
                }
            }
        }
        unsupported("whose end face never crosses it")
    }
}

/// The root of `g` between `lo` and `hi`, `g(lo)` having the sign of `glo`.
fn bisect(mut lo: f64, mut hi: f64, glo: f64, g: impl Fn(f64) -> Result<f64>) -> Result<f64> {
    let mut glo = glo;
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        let gm = g(mid)?;
        if gm == 0.0 {
            return Ok(mid);
        }
        if gm.signum() == glo.signum() {
            (lo, glo) = (mid, gm);
        } else {
            hi = mid;
        }
    }
    Ok(0.5 * (lo + hi))
}

/// `x` with `[r0; r1; r2]·x = f`, or `None` when the rows are dependent.
fn solve3(r: [Vec3; 3], f: [f64; 3]) -> Option<Vec3> {
    let det = r[0].dot(r[1].cross(r[2]));
    if det.abs() <= DEGENERATE {
        return None;
    }
    let (c0, c1, c2) = (r[1].cross(r[2]), r[2].cross(r[0]), r[0].cross(r[1]));
    Some((c0 * f[0] + c1 * f[1] + c2 * f[2]) / det)
}

/// One segment's fitted blend from `a` to `b`.
struct Segment {
    a: f64,
    b: f64,
    link: usize,
    face: [FaceId; 2],
    surface: NurbsSurface,
    contact: [BSpline; 2],
}

fn fit(roll: &Roll, a: f64, b: f64) -> Result<Segment> {
    let link = roll.link_at(0.5 * (a + b));
    let mut guess = None;
    for &n in &SAMPLES {
        let params: Vec<f64> = (0..=n).map(|i| a + (b - a) * i as f64 / n as f64).collect();
        let mut secs = Vec::with_capacity(n + 1);
        for &t in &params {
            let s = roll.section(t, guess)?;
            guess = Some(s.centre);
            secs.push(s);
        }
        guess = secs.first().map(|s| s.centre);
        // the four rows: contact 0, the weighted middle, contact 1, and the weight
        let (mut side0, mut middle, mut side1, mut weight) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for s in &secs {
            let (m, w) = s.arc(roll.radius)?;
            side0.push(s.contact[0]);
            middle.push(m * w);
            side1.push(s.contact[1]);
            weight.push(Vec3::new(w, 0.0, 0.0));
        }
        let [c0, c1, c2, c3] = [side0, middle, side1, weight].map(|r| BSpline::interpolate_at(&r, &params));
        let (c0, c1, c2, c3) = (c0?, c1?, c2?, c3?);
        let knots = c0.knots().to_vec();
        let count = c0.control_points().len();
        let mut ctrl = Vec::with_capacity(3 * count);
        let mut weights = Vec::with_capacity(3 * count);
        let rows = c0.control_points().iter().zip(c1.control_points()).zip(c2.control_points()).zip(c3.control_points());
        for (((&p0, &pm), &p1), wp) in rows {
            let w = wp.x;
            if w <= 0.0 {
                return unsupported("whose sections turn too fast to fit");
            }
            ctrl.extend([p0, pm / w, p1]);
            weights.extend([1.0, w, 1.0]);
        }
        let surface = NurbsSurface::new(
            (3, 2),
            (knots, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
            (count, 3),
            ctrl,
            Some(weights),
        )?;
        let mid = roll.section(0.5 * (a + b), secs.get(n / 2).map(|s| s.centre))?;
        let mut worst = 0.0f64;
        for (i, near) in secs.iter().take(n).enumerate() {
            let t = a + (b - a) * (i as f64 + 0.5) / n as f64;
            let s = roll.section(t, Some(near.centre))?;
            if s.face != mid.face {
                return unsupported("whose contact changes face inside a segment");
            }
            for v in [0.0, 0.25, 0.5, 0.75, 1.0] {
                worst = worst.max(surface.eval(t, v).dist(s.at(roll.radius, v)?));
            }
            for (face, v) in [(s.face[0], 0.0), (s.face[1], 1.0)] {
                let p = surface.eval(t, v);
                worst = worst.max(roll.foot_on(face, p).0.dist(p));
            }
        }
        if worst <= FIT * tol::LINEAR {
            return Ok(Segment {
                a,
                b,
                link,
                face: mid.face,
                surface,
                contact: [c0, c2],
            });
        }
    }
    unsupported("whose sections do not fit within the tolerance")
}

/// An end's trim: `s*` against `v`, and the fitted curve of the points where it meets the plane.
struct Trim {
    /// `(v, s*)` samples, `v` increasing from 0 to 1.
    samples: Vec<(f64, f64)>,
    curve: BSpline,
}

impl Trim {
    /// `s*` at `v`, read off the samples.
    fn s_at(&self, v: f64) -> f64 {
        // the first pair of samples reaching `v`, or the last pair; a trim has at least two
        let pairs = || self.samples.windows(2);
        let pair = pairs().find(|w| matches!(w, [_, (v1, _)] if v <= *v1)).or_else(|| pairs().last());
        match pair {
            Some(&[(v0, s0), (v1, s1)]) => s0 + (s1 - s0) * (v - v0) / (v1 - v0),
            _ => self.samples.first().map_or(v, |x| x.1),
        }
    }
}

fn fit_trim(roll: &Roll, e: &End, from: f64) -> Result<Trim> {
    for &n in &SAMPLES {
        let vs: Vec<f64> = (0..=n).map(|i| i as f64 / n as f64).collect();
        let mut samples = Vec::with_capacity(n + 1);
        let mut pts = Vec::with_capacity(n + 1);
        let mut last = from;
        for &v in &vs {
            let s = roll.trim_at(e, v, last)?;
            last = s;
            samples.push((v, s));
            pts.push(roll.section(s, None)?.at(roll.radius, v)?);
        }
        let curve = BSpline::interpolate_at(&pts, &vs)?;
        let mut worst = 0.0f64;
        for (i, &(_, near)) in samples.iter().take(n).enumerate() {
            let v = (i as f64 + 0.5) / n as f64;
            let s = roll.trim_at(e, v, near)?;
            let p = roll.section(s, None)?.at(roll.radius, v)?;
            worst = worst.max(curve.eval(v).dist(p));
        }
        if worst <= FIT * tol::LINEAR {
            return Ok(Trim { samples, curve });
        }
    }
    unsupported("whose trim by its end face does not fit within the tolerance")
}

/// The picked edges as one chain run the way side 0's faces use them, closed or open, or `None`.
fn chain(body: &Brep, picked: &[Picked]) -> Option<(Vec<Link>, bool)> {
    let end_of = |i: usize| picked.get(i).map(|p| body.coedge_end(p.coedges[0]));
    let start_of = |i: usize| picked.get(i).map(|p| body.coedge_start(p.coedges[0]));
    let first = (0..picked.len())
        .find(|&i| !(0..picked.len()).any(|j| end_of(j) == start_of(i)))
        .unwrap_or(0);
    let mut order = vec![first];
    let mut used = vec![false; picked.len()];
    *used.get_mut(first)? = true;
    while order.len() < picked.len() {
        let v = end_of(*order.last()?);
        let next = (0..picked.len()).find(|&j| used.get(j) == Some(&false) && start_of(j) == v)?;
        *used.get_mut(next)? = true;
        order.push(next);
    }
    let closed = end_of(*order.last()?) == start_of(*order.first()?);
    let mut s0 = 0.0;
    let mut links = Vec::with_capacity(order.len());
    for i in order {
        let pk = picked.get(i)?;
        let e = body.edge(pk.edge);
        let len = e.range.1 - e.range.0;
        links.push(Link {
            pk: i,
            edge: pk.edge,
            forward: !body.coedge(pk.coedges[0]).reversed,
            faces: pk.faces,
            s0,
            len,
        });
        s0 += len;
    }
    Some((links, closed))
}

/// An `f64` ordered for use as a key.
#[derive(Clone, Copy, Debug, PartialEq)]
struct OrderedF64(f64);

impl Eq for OrderedF64 {}

impl PartialOrd for OrderedF64 {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

impl Ord for OrderedF64 {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&o.0)
    }
}

/// The parameter of curve `c` nearest `p`: a line's or circle's in closed form (a circle's within
/// a turn of `range`), any other's by sampling `range` and refining with Newton steps.
fn nearest_param(c: &Curve, p: Point3, range: (f64, f64)) -> Result<f64> {
    match c {
        Curve::Line(l) => Ok((p - l.origin).dot(l.dir)),
        Curve::Circle(ci) => {
            let q = ci.frame.to_local(p);
            let a = q.y.atan2(q.x);
            let mid = 0.5 * (range.0 + range.1);
            Ok(a + ((mid - a) / std::f64::consts::TAU).round() * std::f64::consts::TAU)
        }
        _ => {
            let n = 64;
            let mut t = (0..=n)
                .map(|i| range.0 + (range.1 - range.0) * i as f64 / n as f64)
                .min_by(|&a, &b| c.eval(a).dist(p).total_cmp(&c.eval(b).dist(p)))
                .unwrap_or(range.0);
            for _ in 0..50 {
                let (q, d) = c.d1(t);
                let h = DIFF_STEP * (range.1 - range.0).max(LEAST_SPAN);
                let d2 = (c.d1(t + h).1 - c.d1(t - h).1) / (2.0 * h);
                let f = (q - p).dot(d);
                let df = d.dot(d) + (q - p).dot(d2);
                if df.abs() <= 1e-300 { // tol: guards a division by zero
                    break;
                }
                let step = f / df;
                t = (t - step).clamp(range.0, range.1);
                if step.abs() <= SETTLED_PARAM * (1.0 + t.abs()) {
                    break;
                }
            }
            Ok(t)
        }
    }
}
