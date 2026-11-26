// yack: Hidden-line drawings: a solid's edges and the outlines of its curved faces seen along a
// Adapted from springrep@a03e8a1 src/draw.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/draw.rs to yack-view (M3 W9d); paths and visibility only.
//! Hidden-line drawings: a solid's edges and the outlines of its curved faces seen along a
//! direction, each cut into the stretches the viewer sees and those the solid hides.
//!
//! Visibility along a curve changes only where, seen from the viewer, it passes behind another
//! edge or outline, or meets one. So every stroke (an edge between faces that meet at an angle,
//! or an outline where the line of sight grazes a curved face, trimmed to the face) is cut where
//! its projection crosses another's, found by Newton's method from the crossing chords, and where
//! another stroke ends on it. Each piece between cuts is then seen or hidden throughout, and the
//! exact ray caster decides which at its middle, far from where the line of sight grazes anything.

use std::f64::consts::TAU;

use crate::{
    error::{Error, Result},
    geom::{Circle, Curve, Line, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    measure::{bbox, in_face, uv_polygons},
    topo::{Brep, FaceId},
};

/// A drawing's polylines, in space: the stretches seen and the stretches hidden.
#[derive(Clone, Debug, Default)]
pub struct Drawing {
    pub visible: Vec<Vec<Point3>>,
    pub hidden: Vec<Vec<Point3>>,
}

/// One curve of the drawing over `[t0, t1]`, and the faces it lies on: an edge's two, or the one
/// an outline grazes.
struct Stroke {
    curve: Curve,
    t0: f64,
    t1: f64,
    faces: Vec<FaceId>,
}

/// The drawing of `b` looking along `direction`, every chord within `tolerance` of its curve.
pub fn hidden_lines(b: &Brep, direction: Vec3, tolerance: f64) -> Result<Drawing> {
    let d = direction
        .unit()
        .ok_or_else(|| Error::InvalidInput("a view needs a direction".into()))?;
    let mut out = Drawing::default();
    if b.solid_ids().next().is_none() {
        return Ok(out);
    }
    let scale = bbox(b).diagonal().max(1.0);
    let view = Frame::new(Point3::ZERO, d, d.any_perpendicular())?;
    let flat = |p: Point3| {
        let l = view.to_local(p);
        Vec2::new(l.x, l.y)
    };
    let strokes = strokes(b, d, tolerance);

    // Each stroke's samples, close enough for the tolerance, and their projections.
    let samples: Vec<Vec<f64>> = strokes
        .iter()
        .map(|s| {
            let mut ts = Vec::new();
            sample(&|t| s.curve.eval(t), s.t0, s.t1, tolerance, 0, &mut ts);
            ts.push(s.t1);
            ts
        })
        .collect();
    let flats: Vec<Vec<Vec2>> = strokes
        .iter()
        .zip(&samples)
        .map(|(s, ts)| ts.iter().map(|&t| flat(s.curve.eval(t))).collect())
        .collect();

    // Where each stroke is cut: crossings with the others' projections, and their ends on it.
    let mut cuts: Vec<Vec<f64>> = vec![Vec::new(); strokes.len()];
    let near = 1e-9 * scale;
    for (i, ((si, ti), (fi, cut))) in strokes.iter().zip(&samples).zip(flats.iter().zip(cuts.iter_mut())).enumerate() {
        for (j, ((sj, tj), fj)) in strokes.iter().zip(&samples).zip(&flats).enumerate() {
            if i == j {
                continue;
            }
            for ((ta0, ta1), (wa, wb)) in chords(ti, fi) {
                for ((tb0, tb1), (xa, xb)) in chords(tj, fj) {
                    if let Some((sa, _)) = chords_cross(wa, wb, xa, xb) {
                        let ta = ta0 + (ta1 - ta0) * sa;
                        let tb = tb0 + (tb1 - tb0) * 0.5;
                        if let Some(t) = pin_crossing(si, sj, ta, tb, &flat) {
                            cut.push(t);
                        }
                    }
                }
            }
            for end in [sj.curve.eval(sj.t0), sj.curve.eval(sj.t1)] {
                if let Some(t) = foot(si, ti, end, &flat)
                    && flat(si.curve.eval(t)).dist(flat(end)) <= near
                {
                    cut.push(t);
                }
            }
        }
    }

    let rays = crate::mesh::rays(b)?;
    let eps = 1e-9 * scale;
    // Hidden where the ray toward the viewer leaves the solid. It starts a hair off the faces the
    // point lies on, out of the solid, so it runs beside them rather than along them.
    let hidden = |s: &Stroke, p: Point3| {
        let off = s
            .faces
            .iter()
            .map(|&f| outward(b, f, p))
            .fold(Vec3::ZERO, |a, n| a + n)
            .unit()
            .unwrap_or(Vec3::ZERO);
        rays.cast(p + (off - d) * (100.0 * eps), -d, (0.0, 1e3 * scale))
            .is_ok_and(|hits| hits.iter().any(|h| h.normal.dot(-d) > 1e-9))
    };

    for ((s, cut), stroke_samples) in strokes.iter().zip(&cuts).zip(&samples) {
        let span = s.t1 - s.t0;
        let mut ts: Vec<f64> = cut
            .iter()
            .copied()
            .filter(|&t| (t - s.t0) * span.signum() > 1e-9 * span.abs())
            .filter(|&t| (s.t1 - t) * span.signum() > 1e-9 * span.abs())
            .collect();
        ts.push(s.t0);
        ts.push(s.t1);
        ts.sort_by(|x, y| ((x - s.t0) / span).total_cmp(&((y - s.t0) / span)));
        ts.dedup_by(|x, y| (*x - *y).abs() <= 1e-12 * span.abs().max(1.0));
        // Consecutive pieces of one visibility join.
        let mut runs: Vec<(bool, f64, f64)> = Vec::new();
        for w in ts.windows(2) {
            let &[a, z] = w else { continue };
            let mid = s.curve.eval(0.5 * (a + z));
            let h = hidden(s, mid);
            match runs.last_mut() {
                Some(last) if last.0 == h => last.2 = z,
                _ => runs.push((h, a, z)),
            }
        }
        for (h, a, z) in runs {
            let mut pts: Vec<f64> = stroke_samples
                .iter()
                .copied()
                .filter(|&t| ((t - a) / span) > 1e-12 && ((z - t) / span) > 1e-12)
                .collect();
            pts.insert(0, a);
            pts.push(z);
            let line: Vec<Point3> = pts.iter().map(|&t| s.curve.eval(t)).collect();
            if h {
                out.hidden.push(line);
            } else {
                out.visible.push(line);
            }
        }
    }
    Ok(out)
}

/// A stroke's chords: each pair of neighbouring samples, with their projections.
fn chords<'a>(ts: &'a [f64], flats: &'a [Vec2]) -> impl Iterator<Item = ((f64, f64), (Vec2, Vec2))> + 'a {
    ts.windows(2).zip(flats.windows(2)).filter_map(|pair| match pair {
        (&[t0, t1], &[p0, p1]) => Some(((t0, t1), (p0, p1))),
        _ => None,
    })
}

/// The drawing's strokes: every edge but a seam or one between faces that meet smoothly, and each
/// outline trimmed to the stretches of it that lie on its face.
fn strokes(b: &Brep, d: Vec3, tolerance: f64) -> Vec<Stroke> {
    let mut out = Vec::new();
    for e in b.edge_ids() {
        let edge = b.edge(e);
        let Some(c) = &edge.curve else { continue };
        let faces: Vec<FaceId> = b.edge_uses(e).iter().map(|&co| b.coedge_face(co)).collect();
        let (t0, t1) = edge.range;
        // A seam (one face both sides), or an edge between faces that meet smoothly, draws nothing.
        if let &[f, g] = faces.as_slice()
            && (f == g || smooth(b, f, g, c.eval(0.5 * (t0 + t1))))
        {
            continue;
        }
        out.push(Stroke {
            curve: c.clone(),
            t0,
            t1,
            faces,
        });
    }
    for f in b.face_ids() {
        let s = &b.face(f).surface;
        let polys = uv_polygons(b, f);
        let on = |p: Point3| in_face(s, &polys, s.project(p)).is_some();
        for (curve, t0, t1) in outlines(b, f, d) {
            let mut ts = Vec::new();
            sample(&|t| curve.eval(t), t0, t1, tolerance, 0, &mut ts);
            ts.push(t1);
            let inside: Vec<bool> = ts.iter().map(|&t| on(curve.eval(t))).collect();
            // Each change of membership pinned by bisection.
            let pin = |mut a: f64, mut z: f64, la: bool| {
                for _ in 0..80 {
                    let m = 0.5 * (a + z);
                    if on(curve.eval(m)) == la {
                        a = m;
                    } else {
                        z = m;
                    }
                }
                0.5 * (a + z)
            };
            let mut start = match (inside.first(), ts.first()) {
                (Some(&true), Some(&t)) => Some(t),
                _ => None,
            };
            for (tw, iw) in ts.windows(2).zip(inside.windows(2)) {
                let (&[ta, tb], &[ia, ib]) = (tw, iw) else { continue };
                if ia != ib {
                    let t = pin(ta, tb, ia);
                    match start.take() {
                        Some(a) => out.push(Stroke {
                            curve: curve.clone(),
                            t0: a,
                            t1: t,
                            faces: vec![f],
                        }),
                        None => start = Some(t),
                    }
                }
            }
            if let (Some(a), Some(&end)) = (start, ts.last()) {
                out.push(Stroke {
                    curve: curve.clone(),
                    t0: a,
                    t1: end,
                    faces: vec![f],
                });
            }
        }
    }
    out.retain(|s| (s.t1 - s.t0).abs() > 0.0);
    out
}

/// Face `f`'s outward unit normal at its point nearest `p`.
fn outward(b: &Brep, f: FaceId, p: Point3) -> Vec3 {
    let s = &b.face(f).surface;
    let uv = s.project(p);
    let n = s.normal(uv.x, uv.y);
    if b.face(f).reversed { -n } else { n }
}

/// Whether faces `f` and `g` meet smoothly at `p`: their outward normals agree there.
fn smooth(b: &Brep, f: FaceId, g: FaceId, p: Point3) -> bool {
    let (n, m) = (outward(b, f, p), outward(b, g, p));
    n.cross(m).norm() <= 1e-7 && n.dot(m) > 0.0
}

/// The curves on face `f`'s surface where the line of sight along `d` grazes it, each over a
/// parameter range that covers the face: generators of a cylinder or cone, the great circle of a
/// ball. Other surfaces have none drawn.
fn outlines(b: &Brep, f: FaceId, d: Vec3) -> Vec<(Curve, f64, f64)> {
    let s = &b.face(f).surface;
    let polys = uv_polygons(b, f);
    let (vlo, vhi) = polys
        .iter()
        .flatten()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), q| {
            (lo.min(q.y), hi.max(q.y))
        });
    let mut out = Vec::new();
    match s {
        Surface::Cylinder(_) | Surface::Cone(_) => {
            // The normal is the same all along a generator: the grazing ones are the roots in u
            // of n(u)·d, which 720 samples bracket and bisection pins.
            let v = 0.5 * (vlo + vhi);
            let g = |u: f64| s.normal(u, v).dot(d);
            let n = 720;
            for i in 0..n {
                let (mut a, mut z) = (TAU * i as f64 / n as f64, TAU * (i + 1) as f64 / n as f64);
                if g(a) == 0.0 || g(a).signum() == g(z).signum() {
                    continue;
                }
                for _ in 0..80 {
                    let m = 0.5 * (a + z);
                    if g(m).signum() == g(a).signum() {
                        a = m;
                    } else {
                        z = m;
                    }
                }
                let u = 0.5 * (a + z);
                let p = s.eval(u, vlo);
                if let Some((line, len)) = Line::through(p, s.eval(u, vhi)) {
                    out.push((Curve::Line(line), 0.0, len));
                }
            }
        }
        Surface::Sphere(sp) => {
            if let Ok(frame) = Frame::new(sp.frame.origin, d, d.any_perpendicular()) {
                let circle = Circle {
                    frame,
                    radius: sp.radius,
                };
                out.push((Curve::Circle(circle), 0.0, TAU));
            }
        }
        _ => {}
    }
    out
}

/// Where chords `ab` and `cd` cross, as fractions along each, if they do.
fn chords_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Option<(f64, f64)> {
    let (r, s) = (b - a, d - c);
    let den = r.cross(s);
    if den.abs() <= 1e-300 {
        return None;
    }
    let q = c - a;
    let (t, u) = (q.cross(s) / den, q.cross(r) / den);
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some((t, u))
}

/// The parameter on `a` where its projection crosses `b`'s, by Newton's method on the two
/// parameters from `(ta, tb)`; `None` where it does not settle within both strokes.
fn pin_crossing(
    a: &Stroke,
    b: &Stroke,
    mut ta: f64,
    mut tb: f64,
    flat: &dyn Fn(Point3) -> Vec2,
) -> Option<f64> {
    let flat_d = |v: Vec3| flat(v + Point3::ZERO) - flat(Point3::ZERO);
    for _ in 0..50 {
        let (pa, da) = a.curve.d1(ta);
        let (pb, db) = b.curve.d1(tb);
        let r = flat(pa) - flat(pb);
        let (ja, jb) = (flat_d(da), -flat_d(db));
        let det = ja.cross(jb);
        if det.abs() <= 1e-300 {
            return None;
        }
        // Solve ja·x + jb·y = −r.
        let (x, y) = ((-r).cross(jb) / det, ja.cross(-r) / det);
        ta += x;
        tb += y;
        if x.abs() <= 1e-15 * (1.0 + ta.abs()) && y.abs() <= 1e-15 * (1.0 + tb.abs()) {
            break;
        }
    }
    let within = |s: &Stroke, t: f64| {
        let (lo, hi) = (s.t0.min(s.t1), s.t0.max(s.t1));
        t >= lo - 1e-12 && t <= hi + 1e-12
    };
    let gap = flat(a.curve.eval(ta)).dist(flat(b.curve.eval(tb)));
    (within(a, ta) && within(b, tb) && gap <= 1e-9 * (1.0 + flat(a.curve.eval(ta)).norm()))
        .then_some(ta)
}

/// The parameter of the point of stroke `s` nearest `p` seen along the view: the nearest sample,
/// refined by Newton's method on the projected distance.
fn foot(s: &Stroke, ts: &[f64], p: Point3, flat: &dyn Fn(Point3) -> Vec2) -> Option<f64> {
    let q = flat(p);
    let mut t = *ts.iter().min_by(|&&x, &&y| {
        flat(s.curve.eval(x))
            .dist(q)
            .total_cmp(&flat(s.curve.eval(y)).dist(q))
    })?;
    let flat_d = |v: Vec3| flat(v + Point3::ZERO) - flat(Point3::ZERO);
    let (lo, hi) = (s.t0.min(s.t1), s.t0.max(s.t1));
    for _ in 0..30 {
        let (c, d1) = s.curve.d1(t);
        let h = 1e-6 * (1.0 + t.abs());
        let d2 = (flat_d(s.curve.d1(t + h).1) - flat_d(s.curve.d1(t - h).1)) * (0.5 / h);
        let (r, g) = (flat(c) - q, flat_d(d1));
        let df = g.dot(g) + r.dot(d2);
        if df <= 0.0 {
            break;
        }
        let step = r.dot(g) / df;
        t = (t - step).clamp(lo, hi);
        if step.abs() <= 1e-15 * (1.0 + t.abs()) {
            break;
        }
    }
    Some(t)
}

/// Parameters from `t0` toward `t1`, `t1` left out, close enough that each chord's middle and
/// quarter points lie within half the tolerance of it; at least sixteen.
fn sample(
    at: &dyn Fn(f64) -> Point3,
    t0: f64,
    t1: f64,
    tolerance: f64,
    depth: u32,
    out: &mut Vec<f64>,
) {
    let (a, b) = (at(t0), at(t1));
    let off = |t: f64| {
        let p = at(t);
        match (b - a).unit() {
            Some(u) => (p - a).cross(u).norm(),
            None => p.dist(a),
        }
    };
    let worst = [0.25, 0.5, 0.75]
        .iter()
        .map(|&f| off(t0 + (t1 - t0) * f))
        .fold(0.0, f64::max);
    if depth >= 4 && (worst <= 0.5 * tolerance || depth >= 24) {
        out.push(t0);
        return;
    }
    let m = 0.5 * (t0 + t1);
    sample(at, t0, m, tolerance, depth + 1, out);
    sample(at, m, t1, tolerance, depth + 1, out);
}
