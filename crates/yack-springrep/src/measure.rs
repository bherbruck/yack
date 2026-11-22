// yack: Area, volume, mass properties and tight bounding box, from the exact geometry rather than a
// Adapted from springrep@a03e8a1 src/measure.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the face-box family moved to topo/bbox.rs (M2), re-exported here.
//! Area, volume, mass properties and tight bounding box, from the exact geometry rather than a
//! mesh.
//!
//! A face integral `∬ F du dv` over the trimmed parameter domain becomes a boundary integral by
//! Green's theorem: `∮ G dv` with `G(u, v) = ∫ F(s, v) ds` from a fixed `u`, or `−∮ H du` with the
//! inner integral along `v`. The inner integral runs along the parameter in which the integrand is
//! a low-order polynomial or trigonometric polynomial, so a fixed Gauss rule is exact to rounding;
//! the boundary integral is adaptive.

mod sample;
mod tables;
mod tolerant;

pub use sample::{
    edge_length, extent, face_extent, face_samples, face_uv_bounds, solid_area, solid_bbox,
};
pub use tolerant::{volume_agrees, volume_to};
pub use crate::topo::bbox::{bbox, face_bbox, loose_bbox};
pub(crate) use crate::topo::bbox::{face_box, face_hull_box, in_face, reached_patches, stationary, uv_polygons};

use std::f64::consts::TAU;

use crate::{
    geom::{IsoLine, Surface},
    math::{Aabb, Point3, Vec2, Vec3},
    numeric::{KRONROD_7, KRONROD_15, gauss, solve2},
    topo::{
        Brep, CoedgeId, FaceId, FaceVolume, IntegralKey, LoopId, PCurve, SolidId, VertexId,
        coedge_params,
    },
};
use tables::{pcurve_table, pcurve_tables};

/// Whether a face's inner integral runs along v, and the lowest value its loops reach along that
/// parameter, where every inner integral starts; `None` for a face without loops.
fn inner_start(b: &Brep, f: FaceId) -> Option<(bool, f64)> {
    let face = b.face(f);
    let inner_v = match &face.surface {
        Surface::Extrusion(_) | Surface::Ruled(_) => true,
        // The inner rule runs once per knot span, so it runs along the parameter with fewer.
        Surface::Nurbs(n) => n.span_counts().1 < n.span_counts().0,
        _ => false,
    };
    // Any start serves, since the inner integral from one start differs from that from another by
    // a function of the outer parameter alone, which integrates to nothing round a closed loop.
    // The lowest the loops reach keeps the inner integrals short.
    let (mut lo, mut has) = (f64::INFINITY, false);
    let tables = pcurve_tables(b, f);
    for (&lp, tables) in face.loops.iter().zip(tables) {
        for (&co, table) in b.lp(lp).coedges.iter().zip(tables) {
            let uvs = if table.is_empty() {
                b.coedge_uvs(co, &coedge_params(b, co, 4))
            } else {
                table.clone()
            };
            for uv in uvs {
                lo = lo.min(if inner_v { uv.y } else { uv.x });
                has = true;
            }
        }
    }
    has.then_some((inner_v, lo))
}

/// `∫ F` along the inner parameter from `lo` to `to` at the other parameter `at`, by a Gauss rule
/// of `order` on each smooth piece no longer than a turn, `F` given the point and `su × sv`.
#[allow(clippy::too_many_arguments)]
fn inner_rule<const K: usize>(
    s: &Surface,
    iso: Option<&IsoLine<'_>>,
    inner_v: bool,
    order: usize,
    (lo, at, to): (f64, f64, f64),
    density: &dyn Fn(Point3, Vec3) -> [f64; K],
) -> [f64; K] {
    let mut cuts = vec![lo];
    let (a, z) = (lo.min(to), lo.max(to));
    let bp = if inner_v {
        s.v_breakpoints(a, z)
    } else {
        s.u_breakpoints(a, z)
    };
    if to >= lo {
        cuts.extend(bp);
    } else {
        cuts.extend(bp.into_iter().rev());
    }
    cuts.push(to);
    // Only an angle needs pieces no longer than a turn. A plane's parameters, an extrusion's or a
    // ruled surface's v and a spline's are lengths or arbitrary, and the integrand is one
    // polynomial piece between cuts however long that is: a part metres across is not thousands
    // of turns.
    let turns = match s {
        Surface::Plane(_) | Surface::Nurbs(_) => false,
        Surface::Extrusion(_) | Surface::Ruled(_) => !inner_v,
        _ => true,
    };
    let mut sum = [0.0; K];
    for w in cuts.windows(2) {
        let n_chunks = if turns {
            ((w[1] - w[0]).abs() / TAU).ceil().max(1.0) as usize
        } else {
            1
        };
        for k in 0..n_chunks {
            let x0 = w[0] + (w[1] - w[0]) * k as f64 / n_chunks as f64;
            let x1 = w[0] + (w[1] - w[0]) * (k + 1) as f64 / n_chunks as f64;
            let (m, h) = ((x0 + x1) * 0.5, (x1 - x0) * 0.5);
            for &(x, wt) in gauss(order) {
                let q = m + h * x;
                let (p, su, sv) = match iso {
                    Some(line) => line.d1(q),
                    None if inner_v => s.d1(at, q),
                    None => s.d1(q, at),
                };
                let d = density(p, su.cross(sv));
                for i in 0..K {
                    sum[i] += wt * h * d[i];
                }
            }
        }
    }
    sum
}

/// `∬ F du dv` over the face's trimmed parameter domain, through Green's theorem, each value to
/// its own tolerance in `tol`. `inner` is the integral along the inner parameter, from `lo` to
/// `to` at `at`.
fn face_sum<const K: usize>(
    b: &Brep,
    f: FaceId,
    inner_v: bool,
    inner: &dyn Fn(f64, f64) -> [f64; K],
    tol: [f64; K],
    rel: f64,
) -> [f64; K] {
    let mut total = [0.0; K];
    let tables = pcurve_tables(b, f);
    for (&lp, tables) in b.face(f).loops.iter().zip(tables) {
        for (&co, table) in b.lp(lp).coedges.iter().zip(tables) {
            let v = coedge_integral(b, co, table, inner, inner_v, tol, rel);
            for i in 0..K {
                total[i] += v[i];
            }
        }
    }
    total
}

/// The face's area and its contribution `⅓∬ (p − c)·n dA` to the volume of its solid.
fn face_integrals(b: &Brep, f: FaceId, c: Point3, scale: f64, rel: f64) -> (f64, f64) {
    let key = IntegralKey::new(c, scale, rel, b.face(f).reversed);
    if let Some((k, v)) = *b.memo(f).integrals.lock().unwrap()
        && k == key
    {
        return v;
    }
    let v = face_integrals_anew(b, f, c, scale, rel);
    *b.memo(f).integrals.lock().unwrap() = Some((key, v));
    v
}

fn face_integrals_anew(b: &Brep, f: FaceId, c: Point3, scale: f64, rel: f64) -> (f64, f64) {
    let face = b.face(f);
    let s = &face.surface;
    let sigma = if face.reversed { -1.0 } else { 1.0 };
    let Some((inner_v, lo)) = inner_start(b, f) else {
        return (0.0, 0.0);
    };
    // Along the inner parameter a plane's and an extrusion's integrands are at most linear.
    // A rough integral settles only a sign, which a shorter rule along a smooth piece cannot flip.
    let order = if matches!(s, Surface::Plane(_) | Surface::Extrusion(_)) {
        8
    } else if rel > EXACT {
        12
    } else {
        24
    };
    let density = |p: Point3, n: Vec3| [n.norm(), (p - c).dot(n) / 3.0];
    let inner = |at: f64, to: f64| -> [f64; 2] {
        let h = to - lo;
        match (s, inner_v) {
            // Along a plane's u and an extrusion's generators the normal is constant and the
            // point linear, so the inner integral has a closed form.
            (Surface::Plane(_), false) | (Surface::Extrusion(_), true) => {
                let (p, su, sv) = if inner_v { s.d1(at, lo) } else { s.d1(lo, at) };
                let n = su.cross(sv);
                let along = if inner_v { sv } else { su };
                let vol = ((p - c).dot(n) * h + along.dot(n) * h * h * 0.5) / 3.0;
                return [n.norm() * h, vol];
            }
            // Round a cylinder the normal is r·(a cos u + b sin u), a trigonometric polynomial
            // of degree one, as is the point.
            (Surface::Cylinder(cy), false) => {
                let f = &cy.frame;
                let r = cy.radius;
                let (a, b) = (f.y.cross(f.z), f.z.cross(f.x));
                let w = f.origin + f.z * at - c;
                let m = 0.5 * (lo + to);
                let (sm, cm) = m.sin_cos();
                let (s2m, c2m) = (2.0 * m).sin_cos();
                let sh2 = (0.5 * h).sin();
                let sh = h.sin();
                let int_cos = 2.0 * cm * sh2;
                let int_sin = 2.0 * sm * sh2;
                let int_cos2 = 0.5 * h + 0.5 * c2m * sh;
                let int_sin2 = 0.5 * h - 0.5 * c2m * sh;
                let int_sc = 0.5 * s2m * sh;
                let vol = r
                    * (w.dot(a) * int_cos
                        + w.dot(b) * int_sin
                        + r * (f.x.dot(a) * int_cos2
                            + (f.x.dot(b) + f.y.dot(a)) * int_sc
                            + f.y.dot(b) * int_sin2))
                    / 3.0;
                return [r * h, vol];
            }
            _ => {}
        }
        let line = match s {
            Surface::Nurbs(n) => Some(n.iso(inner_v, at)),
            _ => None,
        };
        inner_rule(s, line.as_ref(), inner_v, order, (lo, at, to), &density)
    };
    let rel = rel.max(noise_floor(c, scale));
    let tol = [rel * scale * scale, rel * scale * scale * scale];
    let [a, v] = face_sum(b, f, inner_v, &inner, tol, rel);
    (sigma * a, v)
}

/// The face's contributions to the volume, first and second moments of its solid about `c`:
/// `⅓∬ x·n`, `¼∬ xᵢ (x·n)` and `⅕∬ xᵢxⱼ (x·n) dA` with `x = p − c`, by the divergence theorem,
/// in the order 1, x, y, z, xx, yy, zz, xy, yz, zx.
fn face_moments(b: &Brep, f: FaceId, c: Point3, scale: f64) -> [f64; 10] {
    let m = face_moments_and_area(b, f, c, scale);
    std::array::from_fn(|i| m[i])
}

/// [`face_moments`] with the face's area after them. One pass gives both, so a solid measured for
/// its volume and area and then its centre integrates each of these faces once.
fn face_moments_and_area(b: &Brep, f: FaceId, c: Point3, scale: f64) -> [f64; 11] {
    let key = IntegralKey::new(c, scale, EXACT, b.face(f).reversed);
    if let Some((k, v)) = *b.memo(f).moments.lock().unwrap()
        && k == key
    {
        return v;
    }
    let v = face_moments_anew(b, f, c, scale);
    *b.memo(f).moments.lock().unwrap() = Some((key, v));
    v
}

/// The face's area and volume share about `c` to the exact tolerance: by [`face_integrals`] where
/// its inner integral has a closed form, which the moments have not, and with the moments
/// otherwise, whose pass costs the same.
fn face_exact(b: &Brep, f: FaceId, c: Point3, scale: f64) -> (f64, f64) {
    let face = b.face(f);
    if matches!(
        face.surface,
        Surface::Plane(_) | Surface::Extrusion(_) | Surface::Cylinder(_)
    ) {
        return face_integrals(b, f, c, scale, EXACT);
    }
    let m = face_moments_and_area(b, f, c, scale);
    (if face.reversed { -m[10] } else { m[10] }, m[0])
}

fn face_moments_anew(b: &Brep, f: FaceId, c: Point3, scale: f64) -> [f64; 11] {
    let s = &b.face(f).surface;
    let Some((inner_v, lo)) = inner_start(b, f) else {
        return [0.0; 11];
    };
    // Along a plane's u the integrands are polynomials of degree at most three.
    let order = if matches!(s, Surface::Plane(_) | Surface::Extrusion(_)) {
        8
    } else {
        24
    };
    let density = |p: Point3, n: Vec3| {
        let x = p - c;
        let flux = x.dot(n);
        [
            flux / 3.0,
            x.x * flux / 4.0,
            x.y * flux / 4.0,
            x.z * flux / 4.0,
            x.x * x.x * flux / 5.0,
            x.y * x.y * flux / 5.0,
            x.z * x.z * flux / 5.0,
            x.x * x.y * flux / 5.0,
            x.y * x.z * flux / 5.0,
            x.z * x.x * flux / 5.0,
            n.norm(),
        ]
    };
    let inner = |at: f64, to: f64| {
        let line = match s {
            Surface::Nurbs(n) => Some(n.iso(inner_v, at)),
            _ => None,
        };
        inner_rule(s, line.as_ref(), inner_v, order, (lo, at, to), &density)
    };
    let (s3, s4, s5) = (scale.powi(3), scale.powi(4), scale.powi(5));
    let rel = EXACT.max(noise_floor(c, scale));
    let s2 = scale * scale;
    let tol = [s3, s4, s4, s4, s5, s5, s5, s5, s5, s5, s2].map(|x| rel * x);
    face_sum(b, f, inner_v, &inner, tol, rel)
}

/// How fast the parameters move along a curve with derivative `dp` on a surface with partial
/// derivatives `su` and `sv`.
fn uv_rate(su: Vec3, sv: Vec3, dp: Vec3) -> Vec2 {
    solve2(
        su.dot(su),
        su.dot(sv),
        su.dot(sv),
        sv.dot(sv),
        su.dot(dp),
        sv.dot(dp),
    )
    .map(|(a, b)| Vec2::new(a, b))
    .unwrap_or(Vec2::ZERO)
}

/// `∮ G dv` (or `−∮ H du`) along one coedge, in its direction, each value to its own tolerance.
/// `table` is its entry of [`pcurve_tables`].
fn coedge_integral<const K: usize>(
    b: &Brep,
    co: CoedgeId,
    table: &[Vec2],
    inner: &dyn Fn(f64, f64) -> [f64; K],
    inner_v: bool,
    tol: [f64; K],
    rel: f64,
) -> [f64; K] {
    let coedge = b.coedge(co);
    let edge = b.edge(coedge.edge);
    let surface = &b.face(b.coedge_face(co)).surface;
    let (t0, t1) = edge.range;
    let (a, z) = if coedge.reversed { (t1, t0) } else { (t0, t1) };
    // Between the two entries either side of `t`, Newton starts nearer its foot than from either.
    // A straight blend stays between them, so a periodic parameter keeps their period; a cubic
    // through their rates can overshoot by one beside a pole.
    let hint = |t: f64| -> Vec2 {
        let n = table.len().saturating_sub(1).max(1);
        let x = ((t - a) / (z - a) * n as f64).clamp(0.0, n as f64);
        let k = (x.floor() as usize).min(n - 1);
        let s = x - k as f64;
        table[k] * (1.0 - s) + table[k + 1] * s
    };
    let uv_and_rate = |t: f64| -> (Vec2, Vec2) {
        match &coedge.pcurve {
            PCurve::Affine { origin, dir } => (*origin + *dir * t, *dir),
            PCurve::Fitted { .. } => coedge.pcurve.fitted_d1(t).unwrap_or_default(),
            PCurve::Projected { .. } => {
                let curve = edge.curve.as_ref().expect("projected pcurves have curves");
                let (p, dp) = curve.d1(t);
                let (uv, su, sv) = surface.project_near_d1(p, hint(t));
                (uv, uv_rate(su, sv, dp))
            }
        }
    };
    let f = |t: f64| -> [f64; K] {
        let (uv, rate) = uv_and_rate(t);
        if inner_v {
            inner(uv.x, uv.y).map(|g| -g * rate.x)
        } else {
            inner(uv.y, uv.x).map(|g| g * rate.y)
        }
    };
    let mut cuts = vec![t0];
    // A fitted intersection curve has hundreds of breakpoints. A short rule settles the smooth
    // piece between two of them, where the halving below takes far more points to find each one.
    if let Some(c) = &edge.curve {
        cuts.extend(c.breakpoints(t0, t1));
    }
    // Where a straight pcurve crosses the surface's knot lines the inner integral has a kink too:
    // along a pole, which has no curve of its own, the adaptive rule would have to find them.
    if let PCurve::Affine { origin, dir } = &coedge.pcurve {
        let (a, b) = (*origin + *dir * t0, *origin + *dir * t1);
        let knots = |lo: f64, hi: f64, u: bool| {
            if u {
                surface.u_breakpoints(lo.min(hi), lo.max(hi))
            } else {
                surface.v_breakpoints(lo.min(hi), lo.max(hi))
            }
        };
        for (u, d, o) in [(true, dir.x, origin.x), (false, dir.y, origin.y)] {
            if d.abs() <= f64::EPSILON {
                continue;
            }
            let (lo, hi) = if u { (a.x, b.x) } else { (a.y, b.y) };
            for k in knots(lo, hi, u) {
                let t = (k - o) / d;
                if t > t0 && t < t1 {
                    cuts.push(t);
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
    }
    cuts.push(t1);
    let order = if rel > EXACT { 8 } else { 16 };
    let mut sum = [0.0; K];
    for w in cuts.windows(2) {
        // Most pieces are short and smooth, and a pair of nested low rules settles them with a
        // third of the points the halving below takes to accept one. To a rough tolerance the
        // shorter pair settles most of them with half as many again.
        let short = (rel > EXACT)
            .then(|| kronrod_k(&f, w[0], w[1], tol, &KRONROD_7))
            .flatten();
        let r = match short.or_else(|| kronrod_k(&f, w[0], w[1], tol, &KRONROD_15)) {
            Some(r) => r,
            None => {
                let (whole, _) = gauss_k(&f, w[0], w[1], order);
                adapt_k(&f, (w[0], w[1]), whole, tol, order, 0)
            }
        };
        for i in 0..K {
            sum[i] += r[i];
        }
    }
    if coedge.reversed {
        sum.map(|x| -x)
    } else {
        sum
    }
}

/// How many rounding errors of a rule's own sum two rules may differ by and still agree: halving
/// cannot settle a difference that small, since each half's sum is rounded as coarsely.
const ROUNDING: f64 = 64.0 * f64::EPSILON;

/// `∫ f` over `[a, b]` by a Kronrod rule where the Gauss rule inside it agrees to `tol`: the Gauss
/// rule is then that accurate itself, and the Kronrod rule more so.
fn kronrod_k<const K: usize>(
    f: &dyn Fn(f64) -> [f64; K],
    a: f64,
    b: f64,
    tol: [f64; K],
    rule: &[(f64, f64, f64)],
) -> Option<[f64; K]> {
    let (m, h) = ((a + b) * 0.5, (b - a) * 0.5);
    let (mut kr, mut ga, mut size) = ([0.0; K], [0.0; K], [0.0; K]);
    for &(x, wk, wg) in rule {
        let (l, r) = if x == 0.0 {
            (f(m), [0.0; K])
        } else {
            (f(m - h * x), f(m + h * x))
        };
        for i in 0..K {
            kr[i] += wk * (l[i] + r[i]);
            ga[i] += wg * (l[i] + r[i]);
            size[i] += wk * (l[i].abs() + r[i].abs());
        }
    }
    let ok = (0..K).all(|i| {
        let floor = (1e-14 * kr[i].abs()).max(ROUNDING * size[i]);
        ((kr[i] - ga[i]) * h).abs() <= tol[i].max(floor * h.abs())
    });
    ok.then(|| kr.map(|x| x * h))
}

/// The rule's sum, and the sum of its terms' sizes, which its rounding error is a share of.
fn gauss_k<const K: usize>(
    f: &dyn Fn(f64) -> [f64; K],
    a: f64,
    b: f64,
    order: usize,
) -> ([f64; K], [f64; K]) {
    let (m, h) = ((a + b) * 0.5, (b - a) * 0.5);
    let (mut s, mut size) = ([0.0; K], [0.0; K]);
    for &(x, w) in gauss(order) {
        let v = f(m + h * x);
        for i in 0..K {
            s[i] += w * v[i];
            size[i] += w * v[i].abs();
        }
    }
    (s.map(|x| x * h), size.map(|x| x * h.abs()))
}

fn adapt_k<const K: usize>(
    f: &dyn Fn(f64) -> [f64; K],
    (a, b): (f64, f64),
    whole: [f64; K],
    tol: [f64; K],
    order: usize,
    depth: u32,
) -> [f64; K] {
    let m = 0.5 * (a + b);
    let ((l, ls), (r, rs)) = (gauss_k(f, a, m, order), gauss_k(f, m, b, order));
    let mut both = [0.0; K];
    for i in 0..K {
        both[i] = l[i] + r[i];
    }
    let ok = (0..K).all(|i| {
        let floor = (1e-14 * both[i].abs()).max(ROUNDING * (ls[i] + rs[i]));
        (both[i] - whole[i]).abs() <= tol[i].max(floor)
    });
    if ok || depth >= 20 {
        return both;
    }
    let half = tol.map(|t| t * 0.5);
    let x = adapt_k(f, (a, m), l, half, order, depth + 1);
    let y = adapt_k(f, (m, b), r, half, order, depth + 1);
    let mut out = [0.0; K];
    for i in 0..K {
        out[i] = x[i] + y[i];
    }
    out
}

/// A centre and size for conditioning the integrals. The vertices' box serves: it costs no
/// surface search, which matters when every solid of a large arena is measured in turn. Fewer than
/// four vertices span no volume (a ring has one), and a tolerance sized to them is unreachable.
fn reference(b: &Brep) -> (Point3, f64) {
    if b.vertex_ids().nth(3).is_none() {
        return moment_reference(b);
    }
    let mut bb = Aabb::EMPTY;
    for v in b.vertex_ids() {
        bb.add(b.vertex(v).point);
    }
    if bb.is_empty() {
        (Vec3::ZERO, 1.0)
    } else {
        (bb.center(), bb.diagonal().max(1.0))
    }
}

/// The signed area loop `lp` encloses in its face's parameter plane, `∮ u dv` integrated on its
/// coedges' exact parameter curves. The checker's sampled outline cannot settle the sign of a sliver
/// narrower than its chords' sag, and this can.
pub(crate) fn loop_area(b: &Brep, lp: LoopId) -> f64 {
    // About the loop's own corner and to its own size: about the plane's origin a sliver's area
    // is the small difference of large terms, below a tolerance sized to the whole model. A
    // billionth of its box settles a sliver's sign; tighter, the quadrature splits to its limit.
    let uvs: Vec<Vec2> = b
        .lp(lp)
        .coedges
        .iter()
        .flat_map(|&co| b.coedge_uvs(co, &coedge_params(b, co, 8)))
        .collect();
    let Some(&corner) = uvs.first() else {
        return 0.0;
    };
    let mut bb = Aabb::EMPTY;
    for uv in &uvs {
        bb.add(Point3::new(uv.x, uv.y, 0.0));
    }
    let size = bb.diagonal();
    let tol = [ROUGH * size * size];
    b.lp(lp)
        .coedges
        .iter()
        .map(|&co| {
            let table = pcurve_table(b, co);
            coedge_integral(b, co, &table, &|_, to| [to - corner.x], false, tol, EXACT)[0]
        })
        .sum()
}

pub fn face_area(b: &Brep, f: FaceId) -> f64 {
    let (c, scale) = reference(b);
    face_integrals(b, f, c, scale, EXACT).0
}

/// The area of every face in the arena.
pub fn area(b: &Brep) -> f64 {
    let (c, scale) = moment_reference(b);
    b.face_ids().map(|f| face_exact(b, f, c, scale).0).sum()
}

/// The boundary integrals' tolerance relative to the model's size: area to `EXACT·size²`, volume
/// to `EXACT·size³`.
const EXACT: f64 = 1e-14;
/// The tolerance that settles only a volume's sign.
const ROUGH: f64 = 1e-9;

/// The least relative tolerance a model `scale` across about `c` can be integrated to: every point
/// it evaluates is rounded to the last digits of its coordinates, `ε·|c|`, so far from the origin
/// [`EXACT`] is below that noise and refinement never stops.
fn noise_floor(c: Point3, scale: f64) -> f64 {
    64.0 * f64::EPSILON * c.norm() / scale
}

fn solid_volume_to(b: &Brep, s: SolidId, rel: f64) -> f64 {
    let (c, scale) = reference(b);
    b.solid_faces(s)
        .into_iter()
        .map(|f| face_integrals(b, f, c, scale, rel).1)
        .sum()
}

/// The enclosed volume: positive for an outward-oriented solid.
pub fn solid_volume(b: &Brep, s: SolidId) -> f64 {
    solid_volume_to(b, s, EXACT)
}

/// The enclosed volume where its sign is all that matters: a rough integral when that is far
/// from zero, the exact one otherwise. It costs a fraction of [`solid_volume`] on a solid with
/// many spline faces, and each face's rough share is worked out once for its geometry, so a
/// face a boolean passes through costs nothing the next time.
pub fn solid_volume_sign(b: &Brep, s: SolidId) -> f64 {
    let (c, scale) = reference(b);
    // Over a closed shell the vector areas sum to zero, so each face's share may be taken about
    // its own anchor and moved to `c` by its vector area.
    let rough: f64 = b
        .solid_faces(s)
        .into_iter()
        .map(|f| {
            let fv = face_volume(b, f);
            fv.volume + (fv.anchor - c).dot(fv.vector_area) / 3.0
        })
        .sum();
    if rough.abs() > 1e4 * ROUGH * scale * scale * scale {
        rough
    } else {
        solid_volume(b, s)
    }
}

/// The volume the closed surface of `faces` encloses, outward normals positive, where its sign is
/// all that matters: [`solid_volume_sign`] for any set of faces.
pub(crate) fn enclosed_volume_sign(b: &Brep, faces: &[FaceId]) -> f64 {
    let (c, scale) = reference(b);
    let rough: f64 = faces
        .iter()
        .map(|&f| {
            let fv = face_volume(b, f);
            fv.volume + (fv.anchor - c).dot(fv.vector_area) / 3.0
        })
        .sum();
    if rough.abs() > 1e4 * ROUGH * scale * scale * scale {
        rough
    } else {
        let rel = EXACT;
        faces
            .iter()
            .map(|&f| face_integrals(b, f, c, scale, rel).1)
            .sum()
    }
}

/// Works out face `f`'s rough volume share now, in its memo: a copy moved rigidly keeps it.
pub(crate) fn prime_face_volume(b: &Brep, f: FaceId) {
    face_volume(b, f);
}

fn face_volume(b: &Brep, f: FaceId) -> FaceVolume {
    *b.memo(f).volume.get_or_init(|| {
        let (anchor, scale) = face_reference(b, f);
        FaceVolume {
            anchor,
            volume: face_integrals(b, f, anchor, scale, ROUGH).1,
            vector_area: vector_area(b, f, anchor, scale),
        }
    })
}

/// [`reference`] for one face, from the vertices its edges end at and three points along each
/// edge: a disc's rim is one edge from a vertex back to it, and a tolerance sized to that one
/// point is unreachable however large the disc.
fn face_reference(b: &Brep, f: FaceId) -> (Point3, f64) {
    let mut bb = Aabb::EMPTY;
    for &lp in &b.face(f).loops {
        for &co in &b.lp(lp).coedges {
            let e = b.edge(b.coedge(co).edge);
            bb.add(b.vertex(e.start).point);
            bb.add(b.vertex(e.end).point);
            if let Some(c) = &e.curve {
                let (t0, t1) = e.range;
                for k in 1..4 {
                    bb.add(c.eval(t0 + (t1 - t0) * k as f64 / 4.0));
                }
            }
        }
    }
    if bb.is_empty() {
        (Vec3::ZERO, 1.0)
    } else {
        (bb.center(), bb.diagonal().max(1.0))
    }
}

/// `∬ n dA` over the face, as `½∮ (p − a) × dp` round its loops (Stokes), to the rough tolerance.
fn vector_area(b: &Brep, f: FaceId, a: Point3, scale: f64) -> Vec3 {
    let tol = ROUGH * scale * scale;
    let mut sum = Vec3::ZERO;
    for &lp in &b.face(f).loops {
        for &co in &b.lp(lp).coedges {
            let coedge = b.coedge(co);
            let edge = b.edge(coedge.edge);
            let Some(c) = &edge.curve else { continue };
            let g = |t: f64| {
                let (p, dp) = c.d1(t);
                (p - a).cross(dp) * 0.5
            };
            let (t0, t1) = edge.range;
            let mut cuts = vec![t0];
            cuts.extend(c.breakpoints(t0, t1));
            cuts.push(t1);
            let mut along = Vec3::ZERO;
            for w in cuts.windows(2) {
                along += integrate_vec(&g, w[0], w[1], tol);
            }
            sum = if coedge.reversed {
                sum - along
            } else {
                sum + along
            };
        }
    }
    sum
}

fn gauss_vec(g: &dyn Fn(f64) -> Vec3, a: f64, b: f64) -> Vec3 {
    let (m, h) = ((a + b) * 0.5, (b - a) * 0.5);
    let mut s = Vec3::ZERO;
    for &(x, w) in gauss(8) {
        s += g(m + h * x) * w;
    }
    s * h
}

/// `∫ g` over `[a, b]`, halved until two levels agree to `tol`.
fn integrate_vec(g: &dyn Fn(f64) -> Vec3, a: f64, b: f64, tol: f64) -> Vec3 {
    fn rec(g: &dyn Fn(f64) -> Vec3, a: f64, b: f64, whole: Vec3, tol: f64, depth: u32) -> Vec3 {
        let m = 0.5 * (a + b);
        let (l, r) = (gauss_vec(g, a, m), gauss_vec(g, m, b));
        let both = l + r;
        if (both - whole).norm() <= tol || depth >= 20 {
            return both;
        }
        rec(g, a, m, l, 0.5 * tol, depth + 1) + rec(g, m, b, r, 0.5 * tol, depth + 1)
    }
    rec(g, a, b, gauss_vec(g, a, b), tol, 0)
}

/// The volume of every solid in the arena.
pub fn volume(b: &Brep) -> f64 {
    let (c, scale) = moment_reference(b);
    b.solid_ids()
        .flat_map(|s| b.solid_faces(s))
        .map(|f| face_exact(b, f, c, scale).1)
        .sum()
}

/// [`area`] and [`volume`] together, integrating each face once.
pub fn area_and_volume(b: &Brep) -> (f64, f64) {
    let (c, scale) = moment_reference(b);
    let per: Vec<(f64, f64)> = b.face_ids().map(|f| face_exact(b, f, c, scale)).collect();
    let area = per.iter().map(|x| x.0).sum();
    let volume = b
        .solid_ids()
        .map(|s| {
            b.solid_faces(s)
                .into_iter()
                .map(|f| per[f.index()].1)
                .sum::<f64>()
        })
        .sum();
    (area, volume)
}

/// Volume, centre and inertia of solids at unit density, in mm: `inertia` is the tensor
/// `∭ (|r|²·I − r rᵀ) dV` about the centre, so its diagonal holds the moments about axes through
/// the centre and its off-diagonal the negated products.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    pub volume: f64,
    pub centre: Point3,
    pub inertia: [[f64; 3]; 3],
}

/// The mass properties of every solid in the arena together.
pub fn mass_properties(b: &Brep) -> MassProperties {
    let (c, scale) = moment_reference(b);
    let mut m = [0.0; 10];
    for s in b.solid_ids() {
        for f in b.solid_faces(s) {
            let fm = face_moments(b, f, c, scale);
            for i in 0..10 {
                m[i] += fm[i];
            }
        }
    }
    from_moments(m, c)
}

/// The mass properties of one solid.
pub fn solid_mass_properties(b: &Brep, s: SolidId) -> MassProperties {
    let (c, scale) = moment_reference(b);
    let mut m = [0.0; 10];
    for f in b.solid_faces(s) {
        let fm = face_moments(b, f, c, scale);
        for i in 0..10 {
            m[i] += fm[i];
        }
    }
    from_moments(m, c)
}

/// [`reference`] from the geometry's loose box rather than its vertices: a moment that symmetry
/// makes zero settles only to a tolerance in the model's size, and a torus or a ball has one
/// vertex or none to give it.
fn moment_reference(b: &Brep) -> (Point3, f64) {
    let bb = loose_bbox(b);
    if bb.is_empty() {
        (Vec3::ZERO, 1.0)
    } else {
        (bb.center(), bb.diagonal().max(1.0))
    }
}

/// Mass properties from the volume and the first and second moments about `c`, in
/// [`face_moments`]' order: the second moments move to the centre by `S − V d dᵀ`.
fn from_moments(m: [f64; 10], c: Point3) -> MassProperties {
    let v = m[0];
    let d = if v == 0.0 {
        [0.0; 3]
    } else {
        [m[1] / v, m[2] / v, m[3] / v]
    };
    let about_c = [[m[4], m[7], m[9]], [m[7], m[5], m[8]], [m[9], m[8], m[6]]];
    let mut s = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            s[i][j] = about_c[i][j] - v * d[i] * d[j];
        }
    }
    let trace = s[0][0] + s[1][1] + s[2][2];
    let mut inertia = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            inertia[i][j] = if i == j { trace } else { 0.0 } - s[i][j];
        }
    }
    MassProperties {
        volume: v,
        centre: c + Vec3::new(d[0], d[1], d[2]),
        inertia,
    }
}

/// The Betti numbers of the arena's point set: its separate pieces, the tunnels through them and
/// the voids they enclose, where pieces of a solid that touch are one piece.
///
/// The point set is its interior and its boundary complex `B`, so its Euler characteristic is
/// `χ(B) − χ(M)/2`, `M` the closed surface `B` becomes with every contact pulled apart: an edge
/// that `k` pairs of faces use becomes `k` edges, and a vertex `m` fans of faces meet at becomes
/// `m`. Each solid is one piece. Faces' outer sides joined across each edge's empty wedges bound
/// the regions of empty space; a group whose faces, with their outward normals, enclose negative
/// volume faces into the region it bounds, a void, and one enclosing positive volume is the
/// outside of the material it wraps. Tunnels are what is left: `b1 = b0 + b2 − χ`.
/// How many regions of material the fans of faces `fans` at `v`, a point where pieces touch and no
/// edge they share, bound round it: each fan's faces cut a small sphere round `v` along a circle,
/// and the circles part the sphere into regions, material and empty by turns. Two fans bound one
/// region of material when each lies on the other's material side and no fan parts them, read
/// from a point of each fan's faces `scale` from `v`.
fn material_regions(b: &Brep, v: VertexId, fans: &[Vec<FaceId>], scale: f64) -> usize {
    let p = b.vertex(v).point;
    let near: Vec<Option<Point3>> = fans.iter().map(|fs| point_near(b, fs, p, scale)).collect();
    let n = fans.len();
    // side[i][j]: whether fan j lies on fan i's material side.
    let side: Vec<Vec<Option<bool>>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| near[j].and_then(|q| material_side(b, &fans[i], v, q)))
                .collect()
        })
        .collect();
    let mut parent: Vec<usize> = (0..n).collect();
    fn root(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            i = p[i];
        }
        i
    }
    for i in 0..n {
        for j in i + 1..n {
            let mutual = side[i][j] == Some(true) && side[j][i] == Some(true);
            let parted = (0..n).any(|k| k != i && k != j && side[k][i] != side[k][j]);
            if mutual && !parted {
                let (x, y) = (root(&mut parent, i), root(&mut parent, j));
                parent[x.max(y)] = x.min(y);
            }
        }
    }
    (0..n).filter(|&i| root(&mut parent, i) == i).count()
}

/// A point inside one of the faces `faces`, about `step` from their vertex at `p`.
fn point_near(b: &Brep, faces: &[FaceId], p: Point3, step: f64) -> Option<Point3> {
    for &f in faces {
        let dom = crate::topo::face_domain(b, f);
        let uv0 = dom.surface.project(p);
        let (_, su, sv) = dom.surface.d1(uv0.x, uv0.y);
        let speed = su.norm().max(sv.norm());
        if speed <= 0.0 {
            continue;
        }
        for k in 0..16 {
            let a = TAU * (k as f64 + 0.5) / 16.0;
            let uv = uv0 + Vec2::new(a.cos(), a.sin()) * (step / speed);
            let q = dom.surface.eval(uv.x, uv.y);
            if let crate::topo::domain::Where::Inside(_) = dom.locate(q) {
                return Some(q);
            }
        }
    }
    None
}

/// Whether `q`, near vertex `v`, lies on the material side of the fan of `faces` there: behind
/// the nearest of the faces, or, nearest an edge of theirs at `v`, behind the faces either side.
fn material_side(b: &Brep, faces: &[FaceId], v: VertexId, q: Point3) -> Option<bool> {
    use crate::topo::domain::Where;
    let mut best: Option<(f64, Point3, Vec3)> = None;
    fn keep(best: &mut Option<(f64, Point3, Vec3)>, d: f64, r: Point3, n: Vec3) {
        if best.is_none_or(|x| d < x.0) {
            *best = Some((d, r, n));
        }
    }
    for &f in faces {
        let dom = crate::topo::face_domain(b, f);
        let uv = dom.surface.project(q);
        let r = dom.surface.eval(uv.x, uv.y);
        if dom.locate(r) != Where::Outside {
            keep(&mut best, r.dist(q), r, dom.outward(uv));
        }
    }
    if best.is_none() {
        let outward = |g: FaceId, r: Point3| {
            let face = b.face(g);
            let uv = face.surface.project(r);
            let n = face.surface.normal(uv.x, uv.y);
            if face.reversed { -n } else { n }
        };
        for &f in faces {
            for &lp in &b.face(f).loops {
                for &c in &b.lp(lp).coedges {
                    let id = b.coedge(c).edge;
                    let e = b.edge(id);
                    let Some(curve) = &e.curve else { continue };
                    if e.start != v && e.end != v {
                        continue;
                    }
                    let r = curve.eval(curve.project(q, e.range.0, e.range.1));
                    let n = b
                        .edge_uses(id)
                        .iter()
                        .map(|&d| b.coedge_face(d))
                        .filter(|g| faces.contains(g))
                        .fold(Vec3::ZERO, |a, g| a + outward(g, r));
                    keep(&mut best, r.dist(q), r, n);
                }
            }
        }
    }
    let (_, r, n) = best?;
    let d = (q - r).dot(n);
    (d.abs() > 1e-3 * crate::tol::LINEAR).then_some(d < 0.0)
}

pub fn betti(b: &Brep) -> [u64; 3] {
    use std::collections::{HashMap, HashSet};
    let mut verts: HashSet<VertexId> = HashSet::new();
    let (mut edges, mut pulled) = (0i64, 0i64);
    for e in b.edge_ids() {
        let edge = b.edge(e);
        let us = b.edge_uses(e);
        if us.is_empty() {
            continue;
        }
        verts.insert(edge.start);
        verts.insert(edge.end);
        if edge.is_degenerate() {
            continue;
        }
        edges += 1;
        pulled += (us.len() as i64 / 2 - 1).max(0);
    }
    let faces: i64 = b.face_ids().map(|f| 2 - b.face(f).loops.len() as i64).sum();
    let chi_b = verts.len() as i64 - edges + faces;
    // Resolving each vertex where fans meet into the manifold boundary of the interior replaces
    // it by its link, the material on a small sphere round it. Where the vertex ends an edge
    // pieces share, that is one piece of link per fan, and the edge's own term splits them.
    let on_radial: HashSet<VertexId> = b
        .radial_edges()
        .flat_map(|e| [b.edge(e).start, b.edge(e).end])
        .collect();
    let scale = 1e-3 * bbox(b).diagonal();
    let fans: i64 = crate::topo::fans_at_vertices(b)
        .into_iter()
        .filter(|(_, fs)| fs.len() > 1)
        .map(|(v, fs)| {
            let f = fs.len() as i64;
            if on_radial.contains(&v) {
                return f - 1;
            }
            let faces: Vec<Vec<FaceId>> = fs
                .iter()
                .map(|fan| fan.iter().map(|&c| b.coedge_face(c)).collect())
                .collect();
            2 * material_regions(b, v, &faces, scale) as i64 - f - 1
        })
        .sum();
    let chi_m = chi_b + fans - pulled;
    let chi = chi_b - chi_m / 2;
    // Faces' outer sides joined across the empty wedges at each edge.
    let ids: Vec<FaceId> = b.face_ids().collect();
    let at: HashMap<FaceId, usize> = ids.iter().enumerate().map(|(i, &f)| (f, i)).collect();
    let mut parent: Vec<usize> = (0..ids.len()).collect();
    fn root(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    for c in b.coedge_ids() {
        if let Some(o) = b.beside(c) {
            let (x, y) = (
                root(&mut parent, at[&b.coedge_face(c)]),
                root(&mut parent, at[&b.coedge_face(o)]),
            );
            if x != y {
                parent[x.max(y)] = x.min(y);
            }
        }
    }
    let mut groups: HashMap<usize, Vec<FaceId>> = HashMap::new();
    for (i, &f) in ids.iter().enumerate() {
        groups.entry(root(&mut parent, i)).or_default().push(f);
    }
    let voids = groups
        .values()
        .filter(|fs| enclosed_volume_sign(b, fs) < 0.0)
        .count() as u64;
    let pieces = b.solid_ids().count() as u64;
    let tunnels = pieces as i64 + voids as i64 - chi;
    [pieces, u64::try_from(tunnels).unwrap_or(0), voids]
}




/// A face's outline, for telling whether a point of its surface lies on the face.
pub(crate) struct Outline {
    /// Its loops as polygons in its surface's parameters.
    polys: Vec<Vec<Vec2>>,
    /// How far those polygons stray from its edges.
    sag: f64,
    /// Points along each edge, and how far the chords between them stray from the edge.
    rim: Vec<Vec<Point3>>,
    rim_sag: f64,
}

impl Outline {
    pub(crate) fn of(b: &Brep, f: FaceId) -> Outline {
        let mut rim = Vec::new();
        let mut rim_sag: f64 = 0.0;
        for &lp in &b.face(f).loops {
            for &co in &b.lp(lp).coedges {
                let Some(c) = &b.edge(b.coedge(co).edge).curve else {
                    continue;
                };
                let ts = coedge_params(b, co, 64);
                let pts: Vec<Point3> = ts.iter().map(|&t| c.eval(t)).collect();
                for k in 0..ts.len() - 1 {
                    let m = c.eval(0.5 * (ts[k] + ts[k + 1]));
                    rim_sag = rim_sag.max(chord_distance(m, pts[k], pts[k + 1]));
                }
                rim.push(pts);
            }
        }
        Outline {
            polys: uv_polygons(b, f),
            sag: outline_sag(b, f),
            rim,
            rim_sag,
        }
    }

    /// Where `p`, the point at `uv` of face `f`'s surface, lies on the face: its parameters
    /// there. The polygon decides, except within its chord sag of the boundary, where a polygon
    /// cuts inside a curved edge: there the nearest edge decides by its own side, and a point
    /// within `within` of an edge is on each face it bounds. The edges are asked only where `p`
    /// is that near the chords along them.
    pub(crate) fn holds(
        &self,
        b: &Brep,
        f: FaceId,
        uv: Vec2,
        p: Point3,
        within: f64,
    ) -> Option<Vec2> {
        let polygon = in_face(&b.face(f).surface, &self.polys, uv);
        let near = within.max(2.0 * self.sag) + self.rim_sag;
        let by_rim = self
            .rim
            .iter()
            .flat_map(|pts| pts.windows(2))
            .any(|w| chord_distance(p, w[0], w[1]) <= near);
        if !by_rim {
            return polygon;
        }
        match boundary_side(b, f, p) {
            Some((d, _)) if d <= within => Some(polygon.unwrap_or(uv)),
            Some((d, Some(side))) if d <= 2.0 * self.sag => side.then_some(polygon.unwrap_or(uv)),
            _ => polygon,
        }
    }
}

/// The distance from `p` to the chord from `a` to `b`.
fn chord_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    let t = if l2 > 0.0 {
        ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    p.dist(a + ab * t)
}

/// How far, in mm, the outline polygons of [`uv_polygons`] stray from face `f`'s edges: a point
/// nearer its boundary than this may lie between a chord and its arc, where a polygon's answer is
/// not the face's.
pub(crate) fn outline_sag(b: &Brep, f: FaceId) -> f64 {
    let s = &b.face(f).surface;
    let mut sag: f64 = 0.0;
    for &lp in &b.face(f).loops {
        for &co in &b.lp(lp).coedges {
            let Some(c) = &b.edge(b.coedge(co).edge).curve else {
                continue;
            };
            let ts = coedge_params(b, co, 64);
            let uv = b.coedge_uvs(co, &ts);
            for k in 0..ts.len() - 1 {
                let m = (uv[k] + uv[k + 1]) * 0.5;
                sag = sag.max(s.eval(m.x, m.y).dist(c.eval(0.5 * (ts[k] + ts[k + 1]))));
            }
        }
    }
    sag
}

/// The distance from `p`, a point on face `f`'s surface, to the face's nearest edge, and which
/// side of that edge it lies on by the edge itself: a loop has the face's material on its left
/// seen from outside, so `p` is on the face where the outward normal crossed with the coedge's
/// direction points towards it. `None` for the side where the nearest point is an edge's end,
/// whose two edges need not agree.
pub(crate) fn boundary_side(b: &Brep, f: FaceId, p: Point3) -> Option<(f64, Option<bool>)> {
    let face = b.face(f);
    let uv = face.surface.project(p);
    let n = face.surface.normal(uv.x, uv.y);
    let n = if face.reversed { -n } else { n };
    let mut best: Option<(f64, Option<bool>)> = None;
    for &lp in &face.loops {
        for &co in &b.lp(lp).coedges {
            let coedge = b.coedge(co);
            let edge = b.edge(coedge.edge);
            let Some(c) = &edge.curve else {
                continue;
            };
            let (t0, t1) = edge.range;
            let t = c.project(p, t0, t1);
            let (q, d1) = c.d1(t);
            let dist = q.dist(p);
            if best.is_some_and(|(d, _)| d <= dist) {
                continue;
            }
            let end = (t - t0).abs() <= 1e-9 * (1.0 + t0.abs())
                || (t1 - t).abs() <= 1e-9 * (1.0 + t1.abs());
            let along = if coedge.reversed { -d1 } else { d1 };
            // A seam, or a slit the face runs along and back, has the face on both sides.
            let twice = b
                .edge_uses(coedge.edge)
                .iter()
                .filter(|&&u| b.coedge_face(u) == f)
                .count()
                > 1;
            let side = (!end).then(|| twice || n.cross(along).dot(p - q) > 0.0);
            best = Some((dist, side));
        }
    }
    best
}

/// Whether `uv`, or a whole period away from it, lies inside the face.
/// The edge nearest `p`, and how far: of edges equally near, one between two faces before a seam,
/// whose two sides are one face, as where a seam's end meets a rim.
pub fn edge_at(b: &Brep, p: Point3) -> Option<(crate::topo::EdgeId, f64)> {
    let seam = |e: crate::topo::EdgeId| {
        let fs: Vec<FaceId> = b
            .coedge_ids()
            .filter(|&c| b.coedge(c).edge == e)
            .map(|c| b.coedge_face(c))
            .collect();
        fs.len() == 2 && fs[0] == fs[1]
    };
    b.edge_ids()
        .filter_map(|e| {
            let x = b.edge(e);
            let c = x.curve.as_ref()?;
            let (t0, t1) = (x.range.0.min(x.range.1), x.range.0.max(x.range.1));
            Some((e, c.eval(c.project(p, t0, t1)).dist(p)))
        })
        .min_by(|x, y| {
            let key = |z: &(crate::topo::EdgeId, f64)| (z.1 > 10.0 * crate::tol::LINEAR, seam(z.0));
            key(x).cmp(&key(y)).then(x.1.total_cmp(&y.1))
        })
}

/// The outward normal of the boundary at `p`, a point on it: every face holding `p`, within
/// `within`, gives its normal, and where they agree (a face's inside, or a smooth seam such as a
/// fillet's) that is it; at a crease, where they do not, there is none.
pub fn normal_at(b: &Brep, p: Point3, within: f64) -> Option<Vec3> {
    Outlines::new(b).normal_at(p, within)
}

/// A solid's faces, each with its outline in its surface's parameters: drawn once, to ask about
/// many points of its boundary.
pub struct Outlines<'a> {
    b: &'a Brep,
    faces: Vec<(FaceId, Outline)>,
}

impl<'a> Outlines<'a> {
    pub fn new(b: &'a Brep) -> Outlines<'a> {
        Outlines {
            b,
            faces: b.face_ids().map(|f| (f, Outline::of(b, f))).collect(),
        }
    }

    /// Where `p`, a point of the surface of the face `held` names, lies on that face.
    fn on_face(&self, (f, outline): &(FaceId, Outline), p: Point3, within: f64) -> Option<Vec2> {
        let uv = self.b.face(*f).surface.project(p);
        outline.holds(self.b, *f, uv, p, within)
    }

    /// As [`normal_at`].
    pub fn normal_at(&self, p: Point3, within: f64) -> Option<Vec3> {
        let b = self.b;
        let found: Vec<Vec3> = self
            .faces
            .iter()
            .filter_map(|held| {
                let face = b.face(held.0);
                let s = &face.surface;
                let near = s.project(p);
                if s.eval(near.x, near.y).dist(p) > within {
                    return None;
                }
                let uv = self.on_face(held, p, within)?;
                let n = s.normal(uv.x, uv.y);
                Some(if face.reversed { -n } else { n })
            })
            .collect();
        let first = *found.first()?;
        found
            .iter()
            .all(|n| (*n - first).norm() <= 1e-6)
            .then_some(first)
    }

    /// As [`curvature`].
    pub fn curvature(&self, p: Point3) -> Option<[f64; 2]> {
        let b = self.b;
        // Each surface's point nearest `p`, nearest first: the first its face holds is the
        // nearest point of the faces.
        let mut near: Vec<(f64, usize, Point3)> = self
            .faces
            .iter()
            .enumerate()
            .map(|(k, held)| {
                let s = &b.face(held.0).surface;
                let uv = s.project(p);
                let q = s.eval(uv.x, uv.y);
                (q.dist(p), k, q)
            })
            .collect();
        near.sort_by(|x, y| x.0.total_cmp(&y.0));
        let face = near.into_iter().find_map(|(d, k, q)| {
            let held = &self.faces[k];
            self.on_face(held, q, 0.0).map(|uv| (d, held.0, uv))
        });
        let edge = b
            .edge_ids()
            .filter_map(|e| {
                let edge = b.edge(e);
                let c = edge.curve.as_ref()?;
                let (t0, t1) = (
                    edge.range.0.min(edge.range.1),
                    edge.range.0.max(edge.range.1),
                );
                let q = c.eval(c.project(p, t0, t1));
                Some((q.dist(p), e, q))
            })
            .min_by(|x, y| x.0.total_cmp(&y.0));
        let (f, uv) = match (face, edge) {
            (Some((df, f, uv)), Some((de, ..))) if df <= de + crate::tol::LINEAR => (f, uv),
            (_, Some((_, e, q))) => {
                // On an edge: a crease unless its faces share their tangent plane there, as a
                // fillet and the face it meets do, where either face's curvature answers.
                let faces: Vec<FaceId> = b
                    .coedge_ids()
                    .filter(|&c| b.coedge(c).edge == e)
                    .map(|c| b.coedge_face(c))
                    .collect();
                let normal = |f: FaceId| {
                    let s = &b.face(f).surface;
                    let uv = s.project(q);
                    let n = s.normal(uv.x, uv.y);
                    (if b.face(f).reversed { -n } else { n }, uv)
                };
                let (&f0, &f1) = (faces.first()?, faces.get(1)?);
                let ((n0, uv0), (n1, _)) = (normal(f0), normal(f1));
                if n0.cross(n1).norm() > 1e-9 || n0.dot(n1) < 0.0 {
                    return None;
                }
                (f0, uv0)
            }
            (Some((_, f, uv)), None) => (f, uv),
            (None, None) => return None,
        };
        let face = b.face(f);
        principal(&face.surface, uv, face.reversed)
    }
}

/// The face nearest `p` among those whose own region, not just their surface, holds the point of
/// their surface nearest `p`; with that distance.
pub fn face_at(b: &Brep, p: Point3) -> Option<(FaceId, f64)> {
    use crate::topo::{domain::Where, face_domain};
    b.face_ids()
        .filter_map(|f| {
            let s = &b.face(f).surface;
            let uv = s.project(p);
            let foot = s.eval(uv.x, uv.y);
            // By the face's exact domain: a polygon of its outline cuts inside a curved edge, and
            // takes a point of a hole's opening for the face round it.
            let on = !matches!(face_domain(b, f).locate(foot), Where::Outside);
            on.then(|| (f, foot.dist(p)))
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))
}

/// The principal curvatures `[k1, k2]`, `k1 ≥ k2`, 1/mm, of the boundary at its point nearest
/// `p`: positive where the boundary bends away from its outward normal, as on a ball. `None` where
/// that point lies on a crease, an edge whose faces meet at an angle there.
pub fn curvature(b: &Brep, p: Point3) -> Option<[f64; 2]> {
    Outlines::new(b).curvature(p)
}

/// The principal curvatures of `s` at `uv` against its outward normal, `k1 ≥ k2`: the shape
/// operator from the first and second fundamental forms, the second derivatives by central
/// differences of the exact first ones. Where the parameters degenerate (a ball's pole), a point
/// a hair away answers for it.
fn principal(s: &Surface, uv: Vec2, reversed: bool) -> Option<[f64; 2]> {
    let at = |u: f64, v: f64| {
        let (_, su, sv) = s.d1(u, v);
        let n = su.cross(sv).unit()?;
        let n = {
            let natural = s.normal(u, v);
            let n = if n.dot(natural) < 0.0 { -n } else { n };
            if reversed { -n } else { n }
        };
        let (hu, hv) = (1e-5 * (1.0 + u.abs()), 1e-5 * (1.0 + v.abs()));
        let (_, su1, sv1) = s.d1(u + hu, v);
        let (_, su0, sv0) = s.d1(u - hu, v);
        let (_, su3, sv3) = s.d1(u, v + hv);
        let (_, su2, sv2) = s.d1(u, v - hv);
        let suu = (su1 - su0) * (0.5 / hu);
        let svv = (sv3 - sv2) * (0.5 / hv);
        let suv = ((sv1 - sv0) * (0.5 / hu) + (su3 - su2) * (0.5 / hv)) * 0.5;
        let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let det = e * g - f * f;
        if det <= 1e-24 * (e * g).max(1e-300) {
            return None;
        }
        let (l, m, nn) = (suu.dot(n), suv.dot(n), svv.dot(n));
        // Bending toward the outward normal is negative.
        let h = -(e * nn + g * l - 2.0 * f * m) / (2.0 * det);
        let k = (l * nn - m * m) / det;
        let r = (h * h - k).max(0.0).sqrt();
        Some([h + r, h - r])
    };
    at(uv.x, uv.y).or_else(|| {
        let nudge = 1e-7 * (1.0 + uv.y.abs());
        at(uv.x, uv.y + nudge).or_else(|| at(uv.x, uv.y - nudge))
    })
}














/// The face's area centroid, by Green's theorem over its parameter-plane loops, each edge taken as
/// 32 points: close, not exact, and the same for the same face.
pub fn face_centroid(b: &Brep, f: FaceId) -> Point3 {
    let face = b.face(f);
    let s = &face.surface;
    let mut polys: Vec<Vec<Vec2>> = Vec::new();
    for &lp in &face.loops {
        let mut pts = Vec::new();
        for &c in &b.lp(lp).coedges {
            let uv = b.coedge_uvs(c, &coedge_params(b, c, 32));
            pts.extend_from_slice(&uv[..uv.len() - 1]);
        }
        polys.push(pts);
    }
    let Some(u_ref) = polys.iter().flatten().next().map(|p| p.x) else {
        return Vec3::ZERO;
    };
    // ∬ g du dv = ∮ G dv, with G(u, v) = ∫ g from u_ref to u.
    let (mut area, mut moment) = (0.0, Vec3::ZERO);
    for poly in &polys {
        for i in 0..poly.len() {
            let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
            let dv = q.y - p.y;
            if dv == 0.0 {
                continue;
            }
            for &(x, w) in gauss(8) {
                let t = 0.5 * (x + 1.0);
                let (u, v) = (p.x + (q.x - p.x) * t, p.y + dv * t);
                let h = 0.5 * (u - u_ref);
                for &(y, wy) in gauss(8) {
                    let su = u_ref + h * (y + 1.0);
                    let (pt, du, dvv) = s.d1(su, v);
                    let jac = du.cross(dvv).norm();
                    let k = 0.5 * w * dv * h * wy * jac;
                    area += k;
                    moment += pt * k;
                }
            }
        }
    }
    if area == 0.0 {
        return polys[0].first().map_or(Vec3::ZERO, |uv| s.eval(uv.x, uv.y));
    }
    moment * (1.0 / area)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        math::Frame,
        profile::{NamedSegment, Profile, ProfileLoop, Segment},
        sweep::extrude,
    };

    #[test]
    fn a_face_moved_keeps_its_box_moved() {
        // An assembly measures a part against another moved before each boolean: a slid part's
        // faces keep their boxes, slid, rather than finding each spline edge's extremes again.
        let rod = extrude(
            &Profile::circle(Vec2::ZERO, 20.0),
            &Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(),
            20.0,
            "r",
        )
        .unwrap();
        let boxes: Vec<Aabb> = rod.face_ids().map(|f| face_box(&rod, f)).collect();
        let shift = Vec3::new(3.5, -120.0, 7.25);
        let mut slid = rod.clone();
        slid.transform(&crate::math::Transform::translation(shift));
        for (f, before) in slid.face_ids().zip(boxes) {
            let kept = slid.memo(f).loose_box.get().copied();
            assert_eq!(
                kept.map(|b| (b.min, b.max)),
                Some((before.min + shift, before.max + shift))
            );
        }
        // Turned, a box is found again: the turned box's own would be looser.
        let mut turned = rod.clone();
        turned.transform(&crate::math::Transform::rotation(Vec3::ZERO, Vec3::X, 0.3));
        assert!(
            turned
                .face_ids()
                .all(|f| turned.memo(f).loose_box.get().is_none())
        );
    }

    #[test]
    fn a_rods_end_just_inside_its_rim_is_flat() {
        // A rod of radius 10: its end's outline, drawn as chords, cuts up to 0.012 inside the
        // rim, and a point of the end between a chord and the rim is still a point of the end.
        let rod = extrude(
            &Profile::circle(Vec2::ZERO, 20.0),
            &Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(),
            20.0,
            "r",
        )
        .unwrap();
        for k in 0..256 {
            let a = std::f64::consts::TAU * k as f64 / 256.0;
            let p = Point3::new(9.999 * a.cos(), 9.999 * a.sin(), 20.0);
            let n = normal_at(&rod, p, 1e-6).unwrap_or_else(|| panic!("no normal at {p:?}"));
            assert!((n - Vec3::Z).norm() < 1e-12, "{n:?} at {p:?}");
            let bend = curvature(&rod, p).unwrap_or_else(|| panic!("no curvature at {p:?}"));
            assert!(
                bend[0].abs() < 1e-9 && bend[1].abs() < 1e-9,
                "{bend:?} at {p:?}"
            );
        }
    }

    #[test]
    fn a_drilled_rods_side_bends_away_its_hole_toward_and_its_rim_is_a_crease() {
        // A rod of radius 10, 20 long, drilled through along its axis at radius 4.
        let circle = |radius: f64, ccw: bool, name: &str| ProfileLoop {
            segments: vec![NamedSegment::new(
                Segment::Circle {
                    center: Vec2::ZERO,
                    radius,
                    ccw,
                },
                name,
            )],
        };
        let p = Profile {
            loops: vec![circle(10.0, true, "out"), circle(4.0, false, "in")],
        };
        let rod = extrude(
            &p,
            &Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(),
            20.0,
            "r",
        )
        .unwrap();
        let near =
            |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9;
        let side = curvature(&rod, Point3::new(11.0, 1.0, 7.0)).unwrap();
        assert!(near(side, [0.1, 0.0]), "{side:?}");
        let hole = curvature(&rod, Point3::new(0.5, 3.0, 13.0)).unwrap();
        assert!(near(hole, [0.0, -0.25]), "{hole:?}");
        let end = curvature(&rod, Point3::new(6.0, 2.0, 20.5)).unwrap();
        assert!(near(end, [0.0, 0.0]), "{end:?}");
        assert!(curvature(&rod, Point3::new(10.5, 0.0, 20.5)).is_none());
    }
}
