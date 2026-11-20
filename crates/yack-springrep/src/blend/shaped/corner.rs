// yack: The set-back corner between three curvature-continuous fillets, as one smooth surface.
// Adapted from springrep@a03e8a1 src/blend/shaped/corner.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! The set-back corner between three curvature-continuous fillets, as one smooth surface.
//!
//! In the corner's own axes, the distances `u₀, u₁, u₂` in from its faces, a fillet's section is
//! `(u_i, u_k) = d·(β(t), α(t))`, `α` and `β` the weights of its legs. Let `φ(d·β(t)) = α/(α + β)`:
//! 1 on a face, falling to 0 at `d` from it, and 0 beyond. A face is where `φ(u_i) = 1` and the
//! fillet between two where `φ(u_i) + φ(u_k) = 1`, so the corner whose three fillets run on into
//! each other is the solid `φ(u₀) + φ(u₁) + φ(u₂) ≤ 1`. Set back, the corner is
//!
//! `φ(u₀) + φ(u₁) + φ(u₂) + B(u₀)·B(u₁)·B(u₂) ≤ 1`, `B(u) = (1 − u/s)⁴` within `s` and 0 beyond.
//!
//! Past the setback `s` along any edge that is the fillets and the faces exactly. Within it the
//! boundary is one surface of continuous curvature, since `φ` and `B` both reach 0 with three
//! vanishing derivatives, and it meets each fillet's end section with the fillet's curvature. The
//! added term takes material: over a face the boundary dips `d·B·B` beneath it at first order, so
//! the flat stops at the setback and not where the fillets' contact lines would cross.
//!
//! The surface is nine faces, each smooth in parameters of its own, cut where `φ` or `B` changes
//! form: a dip over each face, where both other distances are past `d`, a graph over them; a
//! strip along each fillet, where the distance along its edge is past `d`, each section the
//! points on rays from `(d, d)` toward the fillet's section, by the section's parameter; and the
//! core, where all three are within `d`, in thirds by which is least, on rays from `(d, d, d)`.
//! Each is a quintic B-spline through the surface at the Greville sites of even knots. Strips and
//! thirds are rational over the section's denominator, so a strip's outer side is the fillet's
//! section itself, and the sides two faces share are the same curve.

use std::collections::BTreeSet;
use std::f64::consts::FRAC_PI_2;

use super::{
    corner_cube, g2_section, gn_equal_area_w, gn_weights, named_made, section_cut, square_corner,
};
use crate::{
    boolean,
    error::{Error, Result},
    geom::{Curve, NurbsCurve, NurbsSurface, Surface},
    loft::rail::basis,
    math::Point3,
    numeric::solve_dense,
    step::{PartFace, Parts, parts_of, solid_of},
    tol,
    topo::{Brep, EdgeId, FaceName},
};

const DEGREE: usize = 5;

/// Spans along a strip's section, and half as many along each side of a third: the fit then
/// holds the surface within a hundred-millionth of the radius.
const SECTION_SPANS: usize = 48;

/// The corner's solid in the distances from its faces: fillets of contact distance `d` and
/// thumbweight `w`, set back `s`.
struct Blend {
    d: f64,
    s: f64,
    w: f64,
}

/// The root of `f` between `lo`, where it is negative, and `hi`, where it is positive, by false
/// position with the retained end's value halved when it stalls.
fn root(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    let (mut f_lo, mut f_hi) = (f(lo), f(hi));
    if f_lo >= 0.0 {
        return lo;
    }
    if f_hi <= 0.0 {
        return hi;
    }
    let mut last = 0;
    for _ in 0..200 {
        let x = (lo * f_hi - hi * f_lo) / (f_hi - f_lo);
        let fx = f(x);
        if fx == 0.0 {
            return x;
        }
        if fx < 0.0 {
            (lo, f_lo) = (x, fx);
            if last == -1 {
                f_hi *= 0.5;
            }
            last = -1;
        } else {
            (hi, f_hi) = (x, fx);
            if last == 1 {
                f_lo *= 0.5;
            }
            last = 1;
        }
        if hi - lo <= 4.0 * f64::EPSILON * hi.abs().max(lo.abs()) {
            break;
        }
    }
    0.5 * (lo + hi)
}

impl Blend {
    /// The section's leg weights `(α, β)` at `t`.
    fn legs(&self, t: f64) -> [f64; 2] {
        gn_weights(2, self.w, t).0
    }

    /// The section's denominator at `t`, the weight its rational form carries.
    fn den(&self, t: f64) -> f64 {
        let s = 1.0 - t;
        (s.powi(3) + t.powi(3)) * (1.0 - self.w + self.w * t) * (1.0 - self.w * t)
    }

    fn phi(&self, u: f64) -> f64 {
        if u >= self.d {
            return 0.0;
        }
        if u <= 0.0 {
            return 1.0;
        }
        // The section's parameter where its far leg's weight is u / d.
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if self.d * self.legs(mid)[1] < u {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let [a, b] = self.legs(0.5 * (lo + hi));
        a / (a + b)
    }

    fn bump(&self, u: f64) -> f64 {
        (1.0 - u / self.s).max(0.0).powi(4)
    }

    /// Negative inside the solid, positive outside, falling as any distance grows.
    fn g(&self, u: [f64; 3]) -> f64 {
        u.iter().map(|&x| self.phi(x)).sum::<f64>()
            + u.iter().map(|&x| self.bump(x)).product::<f64>()
            - 1.0
    }

    /// Where the ray from `from`, inside the solid, against `dir` leaves it, within `reach`.
    fn on_ray(&self, from: [f64; 3], dir: [f64; 3], reach: f64) -> [f64; 3] {
        let at = |r: f64| [0, 1, 2].map(|i| from[i] - r * dir[i]);
        at(root(|r| self.g(at(r)), 0.0, reach))
    }

    /// The dip over a face at `(x, y)` from the other two, both past `d`: `(x, y, depth)`.
    fn dip(&self, x: f64, y: f64) -> [f64; 3] {
        if self.bump(x) * self.bump(y) == 0.0 {
            return [x, y, 0.0];
        }
        self.on_ray([x, y, self.d], [0.0, 0.0, 1.0], self.d)
    }

    /// The strip along an edge at the section's parameter `t` and `y` along the edge, past `d`:
    /// the distances from the edge's two faces, then `y`.
    fn strip(&self, t: f64, y: f64) -> [f64; 3] {
        let [a, b] = self.legs(t);
        if self.bump(y) == 0.0 {
            return [self.d * b, self.d * a, y];
        }
        let dir = [1.0 - b, 1.0 - a, 0.0];
        self.on_ray([self.d, self.d, y], dir, self.d / dir[0].max(dir[1]))
    }

    /// The third of the core nearest a face, at the section's parameters `p` and `q` in
    /// `[½, 1]`: the distances from the other two faces, then from its own, the least.
    fn third(&self, p: f64, q: f64) -> [f64; 3] {
        let cot = |t: f64| {
            let [a, b] = self.legs(t);
            (1.0 - b) / (1.0 - a)
        };
        self.on_ray([self.d; 3], [cot(p), cot(q), 1.0], self.d)
    }
}

/// A B-spline surface in a piece's own axes.
struct Fit {
    knots: [Vec<f64>; 2],
    count: [usize; 2],
    /// `i·count[1] + j` is the `i`-th along the first parameter and the `j`-th along the second.
    ctrl: Vec<[f64; 3]>,
    weights: Option<Vec<f64>>,
}

type Weight<'a> = Option<&'a dyn Fn(f64) -> f64>;

fn even_knots((lo, hi): (f64, f64), spans: usize) -> Vec<f64> {
    let mut k = vec![lo; DEGREE + 1];
    k.extend((1..spans).map(|i| lo + (hi - lo) * i as f64 / spans as f64));
    k.extend([hi; DEGREE + 1]);
    k
}

/// The Greville sites of clamped `knots`, their ends the domain's exactly.
fn sites(knots: &[f64]) -> Vec<f64> {
    let n = knots.len() - DEGREE - 1;
    let mut at: Vec<f64> = (0..n)
        .map(|i| knots[i + 1..=i + DEGREE].iter().sum::<f64>() / DEGREE as f64)
        .collect();
    (at[0], at[n - 1]) = (knots[0], knots[n]);
    at
}

/// The quintic B-spline through `f` at the Greville sites of even knots, `spans` each way over
/// `range`; with weights, the rational one whose weight is their product, each a polynomial of
/// degree five at most.
fn fit(
    f: &dyn Fn(f64, f64) -> [f64; 3],
    weight: [Weight; 2],
    range: [(f64, f64); 2],
    spans: [usize; 2],
) -> Result<Fit> {
    let knots = [0, 1].map(|k| even_knots(range[k], spans[k]));
    let at = [sites(&knots[0]), sites(&knots[1])];
    let (nu, nv) = (at[0].len(), at[1].len());
    let through = |k: usize| -> Vec<f64> {
        at[k]
            .iter()
            .flat_map(|&t| basis(DEGREE, &knots[k], t))
            .collect()
    };
    let mats = [through(0), through(1)];
    // Each weight at the sites, and as the spline it is.
    let mut seen = [vec![1.0; nu], vec![1.0; nv]];
    let mut held = seen.clone();
    for k in 0..2 {
        if let Some(g) = weight[k] {
            seen[k] = at[k].iter().map(|&t| g(t)).collect();
            held[k] = solve_dense(mats[k].clone(), seen[k].clone(), at[k].len(), 1)?;
        }
    }
    let mut rows = vec![0.0; nu * nv * 3];
    for i in 0..nu {
        for j in 0..nv {
            let p = f(at[0][i], at[1][j]);
            for c in 0..3 {
                rows[(j * nu + i) * 3 + c] = seen[0][i] * seen[1][j] * p[c];
            }
        }
    }
    let along_v = solve_dense(mats[1].clone(), rows, nv, nu * 3)?;
    let mut rows = vec![0.0; nu * nv * 3];
    for i in 0..nu {
        for j in 0..nv {
            for c in 0..3 {
                rows[(i * nv + j) * 3 + c] = along_v[(j * nu + i) * 3 + c];
            }
        }
    }
    let both = solve_dense(mats[0].clone(), rows, nu, nv * 3)?;
    let cells = || (0..nu).flat_map(|i| (0..nv).map(move |j| (i, j)));
    let ctrl = cells()
        .map(|(i, j)| [0, 1, 2].map(|c| both[(i * nv + j) * 3 + c] / (held[0][i] * held[1][j])))
        .collect();
    let weights = weight
        .iter()
        .any(Option::is_some)
        .then(|| cells().map(|(i, j)| held[0][i] * held[1][j]).collect());
    Ok(Fit {
        knots,
        count: [nu, nv],
        ctrl,
        weights,
    })
}

impl Fit {
    fn surface(&self, place: &dyn Fn([f64; 3]) -> Point3) -> Result<Surface> {
        let s = NurbsSurface::new(
            (DEGREE, DEGREE),
            (self.knots[0].clone(), self.knots[1].clone()),
            (self.count[0], self.count[1]),
            self.ctrl.iter().map(|&p| place(p)).collect(),
            self.weights.clone(),
        )?;
        Ok(Surface::Nurbs(Box::new(s)))
    }

    /// The side where parameter `across` is at its start, or its end when `last`, running along
    /// the other parameter.
    fn side(&self, across: usize, last: bool, place: &dyn Fn([f64; 3]) -> Point3) -> Result<Curve> {
        let along = 1 - across;
        let fixed = if last { self.count[across] - 1 } else { 0 };
        let index = |k: usize| {
            if across == 0 {
                fixed * self.count[1] + k
            } else {
                k * self.count[1] + fixed
            }
        };
        let ctrl = (0..self.count[along])
            .map(|k| place(self.ctrl[index(k)]))
            .collect();
        let weights = self
            .weights
            .as_ref()
            .map(|w| (0..self.count[along]).map(|k| w[index(k)]).collect());
        let c = NurbsCurve::new(DEGREE, self.knots[along].clone(), ctrl, weights)?;
        Ok(Curve::Nurbs(Box::new(c)))
    }

    fn middle(&self) -> (f64, f64) {
        let mid = |k: &[f64]| 0.5 * (k[0] + k[k.len() - 1]);
        (mid(&self.knots[0]), mid(&self.knots[1]))
    }
}

fn gone<T>() -> Result<T> {
    Err(Error::Unsupported(
        "a set-back corner whose fillets do not end clear of each other at its setback".into(),
    ))
}

/// `s` with the three `edges` that meet at a square corner of flat faces rounded by the
/// curvature-continuous section of order 2 between the points a ball of `radius` touches, at the
/// thumbweight that takes off what the ball does, each stopped `setback` short of the corner and
/// the corner closed by the surface this module describes.
pub fn fillet_setback_g2(
    s: &Brep,
    edges: &[EdgeId],
    radius: f64,
    setback: f64,
    feature: &str,
) -> Result<Brep> {
    Ok(named_made(
        fillet_setback_g2_raw(s, edges, radius, setback)?,
        feature,
    ))
}

fn fillet_setback_g2_raw(s: &Brep, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Brep> {
    let (d, sb) = (radius, setback);
    let (at, mut a) = square_corner(s, edges, d, sb)?;
    if a[0].cross(a[1]).dot(a[2]) < 0.0 {
        a.swap(1, 2);
    }
    let w = gn_equal_area_w(2, FRAC_PI_2);
    // The cube goes first and each fillet starts inside it: what the fillets would cut from
    // each other lies within the cube, and goes with it.
    let mut cut = boolean::difference(s, &corner_cube(at, &a, sb)?)?;
    for &e in edges {
        let x = s.edge(e);
        let far = [x.start, x.end].map(|v| s.vertex(v).point);
        let far = if far[0].dist(at) > far[1].dist(at) {
            far[0]
        } else {
            far[1]
        };
        let along = (far - at)
            .unit()
            .ok_or_else(|| Error::InvalidInput("an edge of no length".into()))?;
        cut = section_cut(
            &cut,
            at + (far - at) * 0.5 + along * (0.5 * sb),
            Some(at + along * (0.5 * sb)),
            &|_| d,
            &|p, apex, q| g2_section(p, apex, q, w),
        )?;
    }
    let mut parts = parts_of(&cut)?;
    // A piece's own axes are the two after its own, then its own.
    let place = |m: usize| {
        move |l: [f64; 3]| at + a[(m + 1) % 3] * l[0] + a[(m + 2) % 3] * l[1] + a[m] * l[2]
    };
    let found = |parts: &Parts, p: Point3| {
        parts
            .points
            .iter()
            .position(|q| q.dist(p) <= 10.0 * tol::LINEAR)
            .map_or_else(gone, Ok)
    };
    let between = |parts: &Parts, v: usize, w: usize| {
        parts
            .edges
            .iter()
            .position(|&(_, p, q)| (p, q) == (v, w) || (p, q) == (w, v))
            .map_or_else(gone, Ok)
    };
    // On the cube's face square to j: where fillet j's section leaves the face after j (0) and
    // the face before it (1); and where the cube's edges come out through each face.
    let mut touch = [[0; 2]; 3];
    let mut out = [0; 3];
    let mut section = [0; 3];
    for j in 0..3 {
        touch[j] = [
            found(&parts, place(j)([0.0, d, sb]))?,
            found(&parts, place(j)([d, 0.0, sb]))?,
        ];
        out[j] = found(&parts, place(j)([sb, sb, 0.0]))?;
        section[j] = between(&parts, touch[j][0], touch[j][1])?;
    }
    // The cube's faces go: each the flat one of the two on a fillet's end section.
    let mut notch = Vec::new();
    for e in section {
        let uses = |f: &PartFace| f.loops.iter().flatten().any(|u| u.0 == e);
        let flat = |f: &PartFace| matches!(f.surface, Surface::Plane(_));
        notch.push(
            parts
                .faces
                .iter()
                .position(|f| uses(f) && flat(f))
                .map_or_else(gone, Ok)?,
        );
    }
    let shell = parts.faces[notch[0]].shell;
    notch.sort_unstable();
    notch.dedup();
    for &f in notch.iter().rev() {
        parts.faces.remove(f);
    }

    let blend = Blend { d, s: sb, w };
    let along = ((6.0 * (sb - d) / d).ceil() as usize).clamp(8, 64);
    let den = |t: f64| blend.den(t);
    let dip = fit(
        &|x, y| blend.dip(x, y),
        [None, None],
        [(d, sb), (d, sb)],
        [along, along],
    )?;
    let strip = fit(
        &|t, y| blend.strip(t, y),
        [Some(&den), None],
        [(0.0, 1.0), (d, sb)],
        [SECTION_SPANS, along],
    )?;
    let third = fit(
        &|p, q| blend.third(p, q),
        [Some(&den), Some(&den)],
        [(0.5, 1.0), (0.5, 1.0)],
        [SECTION_SPANS / 2, SECTION_SPANS / 2],
    )?;

    let mut point = |p: Point3| {
        parts.points.push(p);
        parts.points.len() - 1
    };
    // The core's middle, the middle of each strip's inner side, and each dip's inner corner.
    let centre = point(place(0)(blend.third(0.5, 0.5)));
    let mid = [0, 1, 2].map(|j| point(place(j)(blend.strip(0.5, d))));
    let low = [0, 1, 2].map(|k| point(place(k)(blend.dip(d, d))));
    let mut edge = |c: Curve, from: usize, to: usize| {
        parts.edges.push((c, from, to));
        parts.edges.len() - 1
    };
    // Each strip's sides beside the dips, the halves of its inner side, and the seam between two
    // thirds that ends at its middle.
    let (mut beside, mut inner, mut seam) = ([[0; 2]; 3], [[0; 2]; 3], [0; 3]);
    for j in 0..3 {
        let (next, prev) = ((j + 1) % 3, (j + 2) % 3);
        beside[j] = [
            edge(strip.side(0, false, &place(j))?, low[next], touch[j][0]),
            edge(strip.side(0, true, &place(j))?, low[prev], touch[j][1]),
        ];
        let c = strip.side(1, false, &place(j))?;
        inner[j] = [
            edge(c.clone(), low[next], mid[j]),
            edge(c, mid[j], low[prev]),
        ];
        seam[j] = edge(third.side(1, false, &place(prev))?, centre, mid[j]);
    }
    // The lines the cube left on each face, from the fillets' contacts to where its edge came
    // out.
    let mut lines = [[0; 2]; 3];
    for k in 0..3 {
        lines[k] = [
            between(&parts, touch[(k + 1) % 3][1], out[k])?,
            between(&parts, touch[(k + 2) % 3][0], out[k])?,
        ];
    }
    let inward = a[0] + a[1] + a[2];
    // A face on `surface` whose sides, each an edge and the vertex it starts from, run
    // counter-clockwise in its parameters.
    let mut face =
        |surface: Surface, (u, v): (f64, f64), name: String, sides: &[(usize, usize)]| {
            let (_, du, dv) = surface.d1(u, v);
            let reversed = du.cross(dv).dot(inward) > 0.0;
            let mut uses: Vec<(usize, bool)> = sides
                .iter()
                .map(|&(e, from)| (e, parts.edges[e].1 != from))
                .collect();
            if reversed {
                uses.reverse();
                for u in &mut uses {
                    u.1 = !u.1;
                }
            }
            parts.faces.push(PartFace {
                surface,
                reversed,
                loops: vec![uses],
                names: BTreeSet::from([FaceName::new("\u{1}set", name)]),
                shell,
            });
        };
    for k in 0..3 {
        let (a, b) = ((k + 1) % 3, (k + 2) % 3);
        face(
            dip.surface(&place(k))?,
            dip.middle(),
            format!("dip{k}"),
            &[
                (beside[a][1], low[k]),
                (lines[k][0], touch[a][1]),
                (lines[k][1], out[k]),
                (beside[b][0], touch[b][0]),
            ],
        );
        face(
            strip.surface(&place(k))?,
            strip.middle(),
            format!("strip{k}"),
            &[
                (inner[k][0], low[a]),
                (inner[k][1], mid[k]),
                (beside[k][1], low[b]),
                (section[k], touch[k][1]),
                (beside[k][0], touch[k][0]),
            ],
        );
        face(
            third.surface(&place(k))?,
            third.middle(),
            format!("core{k}"),
            &[
                (seam[a], centre),
                (inner[a][1], mid[a]),
                (inner[b][0], low[k]),
                (seam[b], mid[b]),
            ],
        );
    }
    solid_of(&parts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        check::check,
        math::{Frame, Vec2, Vec3},
        measure,
        profile::Profile,
        rays::Rays,
        sweep::extrude,
    };
    use std::f64::consts::PI;

    /// A cube of side 10 with the corner at (10, 10, 10) set back.
    fn corner(radius: f64, setback: f64) -> Brep {
        let b = extrude(
            &Profile::rect(Vec2::new(5.0, 5.0), 10.0, 10.0),
            &Frame::WORLD,
            10.0,
            "p",
        )
        .unwrap();
        let at = |x, y, z| measure::edge_at(&b, Point3::new(x, y, z)).unwrap().0;
        let edges = [
            at(5.0, 10.0, 10.0),
            at(10.0, 5.0, 10.0),
            at(10.0, 10.0, 5.0),
        ];
        fillet_setback_g2_raw(&b, &edges, radius, setback).unwrap()
    }

    /// The corner's faces: nine, each on the corner's surface and square to its gradient.
    fn on_its_surface(out: &Brep, d: f64, s: f64) {
        let blend = Blend {
            d,
            s,
            w: gn_equal_area_w(2, FRAC_PI_2),
        };
        let own = |f: &crate::topo::Face| f.names.iter().any(|n| n.feature == "\u{1}set");
        let faces: Vec<_> = out.face_ids().filter(|&f| own(out.face(f))).collect();
        assert_eq!(faces.len(), 9);
        for f in faces {
            let Surface::Nurbs(n) = &out.face(f).surface else {
                panic!("a corner face that is no spline");
            };
            let ((u0, u1), (v0, v1)) = n.domain();
            for i in 0..=14 {
                for j in 0..=14 {
                    let (u, v) = (
                        u0 + (u1 - u0) * i as f64 / 14.0,
                        v0 + (v1 - v0) * j as f64 / 14.0,
                    );
                    let (p, du, dv) = n.d1(u, v);
                    let x = [10.0 - p.x, 10.0 - p.y, 10.0 - p.z];
                    // g changes by 1/d per unit of distance at most.
                    let off = blend.g(x).abs() * d;
                    assert!(off < 1e-8 * d, "{off} off the corner's surface at {x:?}");
                    let h = 1e-6 * d;
                    let grad = Vec3::from_array([0, 1, 2].map(|k| {
                        let (mut hi, mut lo) = (x, x);
                        hi[k] += h;
                        lo[k] -= h;
                        -(blend.g(hi) - blend.g(lo)) / (2.0 * h)
                    }));
                    let normal = du.cross(dv).unit().unwrap();
                    let lean = normal.cross(grad.unit().unwrap()).norm();
                    assert!(lean < 1e-5, "leans {lean} from the gradient at {x:?}");
                }
            }
        }
    }

    #[test]
    fn a_curvature_continuous_set_back_corner_is_its_fillets_beyond_the_setback() {
        // A 10 cube's corner, its edges rounded with contacts 1 from each and set back 3. The
        // section takes what a ball of radius 1 does, (1 − π/4) from every unit of each edge
        // beyond the 4.5 cube at the corner; within the 3 cube the corner takes less than the
        // whole cube and more than nothing.
        let out = corner(1.0, 3.0);
        assert!(check(&out).is_empty(), "{:?}", check(&out));
        assert_eq!(measure::betti(&out), [1, 0, 0]);
        on_its_surface(&out, 1.0, 3.0);
        let frame = Frame::new(Point3::new(5.5, 5.5, 5.5), Vec3::Z, Vec3::X).unwrap();
        let k = extrude(
            &Profile::rect(Vec2::new(2.5, 2.5), 5.0, 5.0),
            &frame,
            5.0,
            "k",
        )
        .unwrap();
        let round = 1.0 - PI / 4.0;
        let outside = measure::volume(&boolean::difference(&out, &k).unwrap());
        let want = 1000.0 - 4.5f64.powi(3) - 3.0 * 5.5 * round;
        assert!(
            (outside - want).abs() < 1e-7 * want,
            "{outside} against {want}"
        );
        let v = measure::volume(&out);
        let most = 1000.0 - 21.0 * round;
        assert!(v > most - 27.0 && v < most, "{v} against {most}");
    }

    #[test]
    fn a_curvature_continuous_set_back_corner_bends_as_its_fillets_at_their_ends() {
        // Just inside a fillet's end section, at its middle, the corner bends round the edge by
        // the section's centre radius and not along it; and on a face the flat has gone where
        // the fillets' contact lines would cross, 1 in from both other faces.
        let out = corner(1.0, 3.0);
        let w = gn_equal_area_w(2, FRAC_PI_2);
        let center = super::super::gn_center_radius(2, w, FRAC_PI_2, 1.0);
        let [a, b] = gn_weights(2, w, 0.5).0;
        let got = measure::curvature(&out, Point3::new(7.0 + 3e-3, 10.0 - a, 10.0 - b)).unwrap();
        let (most, least) = (
            got[0].abs().max(got[1].abs()),
            got[0].abs().min(got[1].abs()),
        );
        assert!((most - 1.0 / center).abs() < 1e-4 / center, "{got:?}");
        assert!(least < 1e-4 / center, "{got:?}");
        let r = Rays::new(&out).unwrap();
        let hits = r
            .cast(
                Point3::new(9.0, 9.0, 10.0 - 1e-5),
                Vec3::new(0.3, 0.5, 0.8),
                (0.0, 100.0),
            )
            .unwrap();
        assert!(hits.len().is_multiple_of(2), "{} crossings", hits.len());
    }

    #[test]
    fn a_curvature_continuous_set_back_corner_holds_at_other_setbacks() {
        // Set back little more than its contacts, and far more, in a file and out of it.
        for (d, s) in [(1.6, 2.0), (0.8, 5.0)] {
            let out = corner(d, s);
            assert!(check(&out).is_empty(), "{:?}", check(&out));
            on_its_surface(&out, d, s);
            let text = crate::step::write(&out, "corner").unwrap();
            let back = crate::step::read(text.as_bytes(), "corner").unwrap();
            assert!(back.rejected.is_empty(), "{:?}", back.rejected);
            let (v, w) = (measure::volume(&out), measure::volume(&back.brep));
            assert!((v - w).abs() < 1e-7 * v, "{v} against {w}");
        }
    }
}
