// yack: Lofts: a solid through a sequence of planar sections, each side face a B-spline surface skinned
// Adapted from springrep@a03e8a1 src/loft/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/loft/mod.rs to yack-build (M3 W7); paths and visibility only.
//! Lofts: a solid through a sequence of planar sections, each side face a B-spline surface skinned
//! across them (a plane where a ruled side is flat). Faces are `start` and `end` on the first and
//! last sections and `side` for the rest.
//!
//! Sections are matched by one rule: outer
//! loops turned to run the same way about the loft, edge `i` to edge `i` when every section has as
//! many, otherwise every section split at the polar angles of every other's vertices. Each column
//! of matched edges is brought to one degree and knot vector, and its control points are carried
//! across the sections: linearly when ruled or with two sections, otherwise by the lowest-degree
//! Bézier through the first and last sections that fits the others within the linear tolerance.

pub mod rail;
#[cfg(test)]
mod tests;

use std::f64::consts::TAU;

use rail::{Across, Hom, Rail, point};

use crate::{
    Error, Result, check,
    error::invalid,
    geom::{Curve, Line, NurbsCurve, NurbsSurface, Plane, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    profile::{Names, Profile, ProfileLoop},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceName, PCurve, VertexId},
};

/// The highest degree a smooth loft takes across its sections.
const MAX_ACROSS_DEGREE: usize = 8;

/// One section: a profile on a plane. Only its outer loop is lofted.
#[derive(Clone, Debug)]
pub struct Section {
    pub profile: Profile,
    pub frame: Frame,
}

/// A stretch of a 3-D curve, from parameter `t0` to `t1` in either order.
#[derive(Clone, Debug)]
struct Piece {
    curve: Curve,
    t0: f64,
    t1: f64,
    straight: bool,
    /// The names of the profile edge this is, or is a part of.
    names: Names,
}

impl Piece {
    fn param(&self, s: f64) -> f64 {
        self.t0 + (self.t1 - self.t0) * s
    }

    fn at(&self, s: f64) -> Point3 {
        self.curve.eval(self.param(s))
    }

    fn split(&self, s: f64) -> (Piece, Piece) {
        let tm = self.param(s);
        (
            Piece {
                t1: tm,
                ..self.clone()
            },
            Piece {
                t0: tm,
                ..self.clone()
            },
        )
    }
}

/// A section's outer loop in 3-D, running counter-clockwise about the loft direction.
struct Ring {
    pieces: Vec<Piece>,
    /// The section plane, `z` along the loft.
    frame: Frame,
    center: Point3,
}

fn centroid(lp: &ProfileLoop) -> Vec2 {
    let p = lp.polygon();
    let (mut a, mut cx, mut cy) = (0.0, 0.0, 0.0);
    for i in 0..p.len() {
        let (q, r) = (p[i], p[(i + 1) % p.len()]);
        let c = q.cross(r);
        a += c;
        cx += (q.x + r.x) * c;
        cy += (q.y + r.y) * c;
    }
    Vec2::new(cx / (3.0 * a), cy / (3.0 * a))
}

#[allow(clippy::panic)] fn outer(s: &Section) -> Result<ProfileLoop> {
    let p = s.profile.normalized()?;
    Ok(p.loops
        .into_iter()
        .next()
        .unwrap_or_else(|| panic!("a normalized profile has a loop")))
}

fn ring(s: &Section, lp: &ProfileLoop, along: Vec3) -> Result<Ring> {
    let mut lp = lp.clone();
    let c = centroid(&lp);
    let mut frame = s.frame;
    if frame.z.dot(along) < 0.0 {
        lp.reverse();
        frame = Frame::new(frame.origin, -frame.z, frame.x)?;
    }
    let pieces = lp
        .segments
        .iter()
        .map(|seg| {
            let placed = seg.segment.place(&s.frame);
            let (t0, t1) = if placed.forward {
                placed.range
            } else {
                (placed.range.1, placed.range.0)
            };
            Piece {
                straight: matches!(placed.curve, Curve::Line(_)),
                names: seg.names.clone(),
                curve: placed.curve,
                t0,
                t1,
            }
        })
        .collect();
    Ok(Ring {
        pieces,
        frame,
        center: s.frame.at(c.x, c.y),
    })
}

/// A vertex angle closer than this to another is the same angle.
const SAME_ANGLE: f64 = 1e-9;

fn angle_dist(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(TAU);
    d.min(TAU - d)
}

impl Ring {
    /// The polar angle of `p` about the section's centre, from `x0` projected into its plane.
    fn angle(&self, p: Point3, x0: Vec3) -> f64 {
        let z = self.frame.z;
        let x = (x0 - z * x0.dot(z)).unit().unwrap_or(self.frame.x);
        let y = z.cross(x);
        let d = p - self.center;
        d.dot(y).atan2(d.dot(x)).rem_euclid(TAU)
    }

    /// Splits the pieces at each of `angles` that is not already a vertex. Each piece must turn
    /// steadily forward about the centre.
    fn split_at(&mut self, angles: &[f64], x0: Vec3) -> Result<()> {
        let mut out = Vec::new();
        for piece in &self.pieces {
            let a0 = self.angle(piece.at(0.0), x0);
            let samples = 64;
            let mut prev = 0.0;
            let mut turn = Vec::with_capacity(samples + 1);
            turn.push(0.0);
            for i in 1..=samples {
                let a = self.angle(piece.at(i as f64 / samples as f64), x0);
                let mut d = (a - a0).rem_euclid(TAU);
                if i == samples && d < SAME_ANGLE {
                    d = TAU;
                }
                if d + SAME_ANGLE < prev {
                    return Err(Error::Unsupported(
                        "lofting sections with different edge counts that are not star-shaped \
                         about their centres"
                            .into(),
                    ));
                }
                prev = d;
                turn.push(d);
            }
            let total = prev;
            let mut cuts: Vec<f64> = angles
                .iter()
                .map(|&a| (a - a0).rem_euclid(TAU))
                .filter(|&d| d > SAME_ANGLE && d < total - SAME_ANGLE)
                .collect();
            cuts.sort_by(f64::total_cmp);
            let mut rest = piece.clone();
            let mut done = 0.0;
            for d in cuts {
                let i = turn.partition_point(|&t| t < d).clamp(1, samples);
                let (mut lo, mut hi) = ((i - 1) as f64 / samples as f64, i as f64 / samples as f64);
                for _ in 0..80 {
                    let mid = 0.5 * (lo + hi);
                    let dm = (self.angle(piece.at(mid), x0) - a0).rem_euclid(TAU);
                    if dm < d { lo = mid } else { hi = mid }
                }
                let s = 0.5 * (lo + hi);
                let local = (s - done) / (1.0 - done);
                let (a, b) = rest.split(local);
                out.push(a);
                rest = b;
                done = s;
            }
            out.push(rest);
        }
        self.pieces = out;
        Ok(())
    }

    fn starts(&self) -> Vec<Point3> {
        self.pieces.iter().map(|p| p.at(0.0)).collect()
    }
}

/// Matches every ring's pieces one to one, in order.
fn match_rings(rings: &mut [Ring]) -> Result<()> {
    let k = rings[0].pieces.len();
    if rings.iter().all(|r| r.pieces.len() == k) {
        // Vertices about their section's centre, read two ways, and the shift that fits either
        // best wins: along world axes (one outline drawn on planes whose axes differ, or on a
        // plane facing back down the loft), or along the section's own axes (a section carried
        // with its frame, as a helical gear's tooth space turned about the gear's axis, moves
        // further between sections than it is wide). On a tie the world reading wins, as it turns
        // the loft least.
        let offsets = |r: &Ring| -> (Vec<Vec3>, Vec<Vec3>) {
            r.starts()
                .into_iter()
                .map(|p| {
                    let d = p - r.center;
                    (d, Vec3::new(d.dot(r.frame.x), d.dot(r.frame.y), 0.0))
                })
                .unzip()
        };
        for i in 1..rings.len() {
            let (prev_w, prev_l) = offsets(&rings[i - 1]);
            let (cur_w, cur_l) = offsets(&rings[i]);
            let mut costs = Vec::with_capacity(2 * k);
            for (prev, cur) in [(&prev_w, &cur_w), (&prev_l, &cur_l)] {
                for shift in 0..k {
                    let c: f64 = (0..k)
                        .map(|j| (prev[j] - cur[(j + shift) % k]).norm())
                        .sum();
                    costs.push(c);
                }
            }
            let least = costs.iter().copied().fold(f64::INFINITY, f64::min);
            let best = costs
                .iter()
                .position(|&c| c <= least + k as f64 * tol::LINEAR)
                .map_or(0, |at| at % k);
            rings[i].pieces.rotate_left(best);
        }
        return Ok(());
    }
    let x0 = rings[0].frame.x;
    let mut all: Vec<f64> = rings
        .iter()
        .flat_map(|r| r.starts().into_iter().map(|p| r.angle(p, x0)))
        .collect();
    all.sort_by(f64::total_cmp);
    let mut angles: Vec<f64> = Vec::new();
    for a in all {
        if angles.iter().all(|&b| angle_dist(a, b) > SAME_ANGLE) {
            angles.push(a);
        }
    }
    let first = rings[0].angle(rings[0].pieces[0].at(0.0), x0);
    for r in rings.iter_mut() {
        r.split_at(&angles, x0)?;
        if r.pieces.len() != angles.len() {
            return Err(Error::Numerical(
                "loft sections did not split into matching edges".into(),
            ));
        }
        let at = r
            .starts()
            .iter()
            .position(|&p| angle_dist(r.angle(p, x0), first) <= SAME_ANGLE)
            .ok_or_else(|| Error::Numerical("loft sections lost a matching vertex".into()))?;
        r.pieces.rotate_left(at);
    }
    Ok(())
}

/// The section parameters by chord length over every control point.
fn chord_params(columns: &[Vec<Rail>], n: usize) -> Vec<f64> {
    let mut acc = vec![0.0; n];
    for s in 1..n {
        let d: f64 = columns
            .iter()
            .map(|col| {
                col[s]
                    .pw
                    .iter()
                    .zip(&col[s - 1].pw)
                    .map(|(&a, &b)| point(a).dist(point(b)))
                    .sum::<f64>()
            })
            .sum();
        acc[s] = acc[s - 1] + d;
    }
    let total = acc[n - 1];
    if total > 0.0 {
        acc.iter().map(|a| a / total).collect()
    } else {
        (0..n).map(|s| s as f64 / (n - 1) as f64).collect()
    }
}

/// How the control points run across the sections of a smooth loft: the lowest-degree Bézier
/// that passes within the linear tolerance of every section, or a cubic through them all.
fn across(columns: &[Vec<Rail>], n: usize) -> Result<Across> {
    let params = chord_params(columns, n);
    if n == 2 {
        return rail::bezier_fit(&params, 1);
    }
    for p in 2..=MAX_ACROSS_DEGREE.min(n - 1) {
        let fit = rail::bezier_fit(&params, p)?;
        let mut worst: f64 = 0.0;
        for col in columns {
            for m in 0..col[0].pw.len() {
                let data: Vec<Hom> = col.iter().map(|r| r.pw[m]).collect();
                let ctrl = fit.apply(&data);
                for (s, &v) in params.iter().enumerate() {
                    worst = worst.max(point(fit.eval(&ctrl, v)).dist(point(data[s])));
                }
            }
        }
        if worst <= tol::LINEAR {
            return Ok(fit);
        }
    }
    rail::interpolation(&params)
}

fn affine(origin: Vec2, dir: Vec2) -> PCurve {
    PCurve::Affine { origin, dir }
}

fn projected(frame: &Frame, p: Point3) -> PCurve {
    let l = frame.to_local(p);
    PCurve::Projected {
        seed: Vec2::new(l.x, l.y),
    }
}

fn coedge(edge: EdgeId, reversed: bool, pcurve: PCurve) -> CoedgeSpec {
    CoedgeSpec {
        edge,
        reversed,
        pcurve,
    }
}

/// How an edge's parameter maps onto a side face's `u` or `v`: `scale · t`.
#[derive(Clone, Copy)]
struct EdgeParam {
    id: EdgeId,
    scale: f64,
}

/// The plane through two matched straight edges, `x` along the loop and `z` outward, when they
/// are coplanar.
fn flat_side(a0: Point3, b0: Point3, a1: Point3, b1: Point3) -> Option<Frame> {
    let x = (b0 - a0).unit()?;
    let w = a1 - a0;
    let y = (w - x * w.dot(x)).unit()?;
    let z = x.cross(y);
    let off = (b1 - a0).dot(z).abs().max((b1 - a1).dot(z).abs());
    (off <= tol::LINEAR).then_some(Frame {
        origin: a0,
        x,
        y,
        z,
    })
}

/// A side face's names: `side`, and the names of the first section's edge it grows from.
fn side_names(feature: &str, edge: &Names) -> Vec<FaceName> {
    let mut names = vec![FaceName::new(feature, FaceName::SIDE)];
    names.extend(FaceName::all(
        feature,
        edge.iter().filter(|n| !n.is_empty()),
    ));
    names
}

/// Lofts through `sections` in order, ruled (straight between consecutive sections) or smooth. The
/// first section's outline is the `start` face and the last one's the `end` face. Every face
/// between is `side`, and also carries the name of the first section's edge it grows from; where
/// sections with different edge counts split that edge, each part carries its name.
#[allow(clippy::panic)] pub fn loft(sections: &[Section], ruled: bool, feature: &str) -> Result<Brep> {
    let n = sections.len();
    if n < 2 {
        return invalid("a loft needs at least two sections");
    }
    let loops: Vec<ProfileLoop> = sections.iter().map(outer).collect::<Result<_>>()?;
    let centers: Vec<Point3> = sections
        .iter()
        .zip(&loops)
        .map(|(s, l)| {
            let c = centroid(l);
            s.frame.at(c.x, c.y)
        })
        .collect();
    let Some(along) = (centers[n - 1] - centers[0]).unit() else {
        return invalid("a loft's first and last sections share a centre");
    };
    if sections
        .iter()
        .any(|s| s.frame.z.dot(along).abs() <= tol::ANGULAR)
    {
        return invalid("a loft section's plane contains the loft direction");
    }
    // Sections on parallel planes must come in order across them: one that steps back, or sits on
    // its neighbour's plane, folds the sides over and covers part of the solid twice.
    let normal = sections[0].frame.z;
    if sections
        .iter()
        .all(|s| s.frame.z.cross(normal).norm() <= tol::ANGULAR)
    {
        let offsets: Vec<f64> = sections
            .iter()
            .map(|s| (s.frame.origin - sections[0].frame.origin).dot(normal))
            .collect();
        let rises = offsets.windows(2).all(|w| w[1] - w[0] > tol::LINEAR);
        let falls = offsets.windows(2).all(|w| w[0] - w[1] > tol::LINEAR);
        if !rises && !falls {
            return invalid("a loft's parallel sections do not come in order across their planes");
        }
    }
    let mut rings: Vec<Ring> = sections
        .iter()
        .zip(&loops)
        .map(|(s, l)| ring(s, l, along))
        .collect::<Result<_>>()?;
    match_rings(&mut rings)?;
    let k = rings[0].pieces.len();

    let mut columns: Vec<Vec<Rail>> = Vec::with_capacity(k);
    for j in 0..k {
        let mut col: Vec<Rail> = rings
            .iter()
            .map(|r| {
                let p = &r.pieces[j];
                Rail::of_curve(&p.curve, p.t0, p.t1)
            })
            .collect::<Result<_>>()?;
        rail::make_compatible(&mut col);
        columns.push(col);
    }

    // The sections that bound bands of side faces, and how each band runs across its sections.
    let bands: Vec<(usize, usize, Across)> = if ruled || n == 2 {
        let lin = rail::bezier_fit(&[0.0, 1.0], 1)?;
        (0..n - 1).map(|s| (s, s + 1, lin.clone())).collect()
    } else {
        vec![(0, n - 1, across(&columns, n)?)]
    };
    let rows: Vec<usize> = if ruled || n == 2 {
        (0..n).collect()
    } else {
        vec![0, n - 1]
    };

    let mut b = Brep::new();
    let mut verts: Vec<Vec<VertexId>> = vec![Vec::new(); n];
    for &r in &rows {
        verts[r] = rings[r]
            .starts()
            .into_iter()
            .map(|p| b.add_vertex(p))
            .collect();
    }
    // Section edges, each with how its parameter maps onto its column's `u`.
    let mut sec_edges: Vec<Vec<Option<EdgeParam>>> = vec![vec![None; k]; n];
    for &r in &rows {
        for j in 0..k {
            let rl = &columns[j][r];
            let (va, vb) = (verts[r][j], verts[r][(j + 1) % k]);
            let ep = if rings[r].pieces[j].straight {
                let (line, len) = Line::through(rl.start(), rl.end()).ok_or_else(|| {
                    Error::InvalidInput("a loft section has a zero-length edge".into())
                })?;
                EdgeParam {
                    id: b.add_edge(Curve::Line(line), (0.0, len), va, vb),
                    scale: 1.0 / len,
                }
            } else {
                let c = nurbs_curve(rl.degree, rl.knots.clone(), &rl.pw)?;
                EdgeParam {
                    id: b.add_edge(Curve::Nurbs(Box::new(c)), (0.0, 1.0), va, vb),
                    scale: 1.0,
                }
            };
            sec_edges[r][j] = Some(ep);
        }
    }

    let mut faces = Vec::new();
    for (lo, hi, acr) in &bands {
        let (lo, hi) = (*lo, *hi);
        let picked: Vec<usize> = (lo..=hi).collect();
        let linear = acr.degree == 1;
        // Side edges at each vertex, running from section `lo` to section `hi`.
        let mut sides: Vec<EdgeParam> = Vec::with_capacity(k);
        for j in 0..k {
            let (va, vb) = (verts[lo][j], verts[hi][j]);
            let ep = if linear {
                let (p0, p1) = (columns[j][lo].start(), columns[j][hi].start());
                let (line, len) = Line::through(p0, p1)
                    .ok_or_else(|| Error::InvalidInput("loft sections share a vertex".into()))?;
                EdgeParam {
                    id: b.add_edge(Curve::Line(line), (0.0, len), va, vb),
                    scale: 1.0 / len,
                }
            } else {
                let data: Vec<Hom> = picked.iter().map(|&s| columns[j][s].pw[0]).collect();
                let c = nurbs_curve(acr.degree, acr.knots.clone(), &acr.apply(&data))?;
                EdgeParam {
                    id: b.add_edge(Curve::Nurbs(Box::new(c)), (0.0, 1.0), va, vb),
                    scale: 1.0,
                }
            };
            sides.push(ep);
        }
        for j in 0..k {
            let (bottom, top) = (
                sec_edges[lo][j].unwrap_or_else(|| panic!("band rows have edges")),
                sec_edges[hi][j].unwrap_or_else(|| panic!("band rows have edges")),
            );
            let (left, right) = (sides[j], sides[(j + 1) % k]);
            let (rl, rh) = (&columns[j][lo], &columns[j][hi]);
            let flat = (linear && rings[lo].pieces[j].straight && rings[hi].pieces[j].straight)
                .then(|| flat_side(rl.start(), rl.end(), rh.start(), rh.end()))
                .flatten();
            let face = if let Some(fr) = flat {
                let names = side_names(feature, &rings[0].pieces[j].names);
                let face = b.add_face(Surface::Plane(Plane { frame: fr }), false, names);
                b.add_loop(
                    face,
                    vec![
                        coedge(bottom.id, false, projected(&fr, rl.start())),
                        coedge(right.id, false, projected(&fr, rl.end())),
                        coedge(top.id, true, projected(&fr, rh.end())),
                        coedge(left.id, true, projected(&fr, rh.start())),
                    ],
                );
                face
            } else {
                let surface = skin(&columns[j], &picked, acr)?;
                let names = side_names(feature, &rings[0].pieces[j].names);
                let face = b.add_face(surface, false, names);
                b.add_loop(
                    face,
                    vec![
                        coedge(
                            bottom.id,
                            false,
                            affine(Vec2::ZERO, Vec2::new(bottom.scale, 0.0)),
                        ),
                        coedge(
                            right.id,
                            false,
                            affine(Vec2::new(1.0, 0.0), Vec2::new(0.0, right.scale)),
                        ),
                        coedge(
                            top.id,
                            true,
                            affine(Vec2::new(0.0, 1.0), Vec2::new(top.scale, 0.0)),
                        ),
                        coedge(
                            left.id,
                            true,
                            affine(Vec2::ZERO, Vec2::new(0.0, left.scale)),
                        ),
                    ],
                );
                face
            };
            faces.push(face);
        }
    }

    for (r, name, reversed) in [(0, FaceName::START, true), (n - 1, FaceName::END, false)] {
        let fr = rings[r].frame;
        let face = b.add_face(
            Surface::Plane(Plane { frame: fr }),
            reversed,
            [FaceName::new(feature, name)],
        );
        let order: Vec<usize> = if reversed {
            (0..k).rev().collect()
        } else {
            (0..k).collect()
        };
        let lp = order
            .into_iter()
            .map(|j| {
                let rl = &columns[j][r];
                let at = if reversed { rl.end() } else { rl.start() };
                let e = sec_edges[r][j].unwrap_or_else(|| panic!("cap rows have edges"));
                coedge(e.id, reversed, projected(&fr, at))
            })
            .collect();
        b.add_loop(face, lp);
        faces.push(face);
    }
    let shell = b.add_shell(faces);
    b.add_solid(vec![shell]);
    let issues = check::check(&b);
    if issues.is_empty() {
        Ok(b)
    } else {
        Err(Error::InvalidResult(
            issues.into_iter().map(|i| i.to_string()).collect(),
        ))
    }
}

fn split_hom(pw: &[Hom]) -> (Vec<Point3>, Option<Vec<f64>>) {
    let pts = pw.iter().map(|&h| point(h)).collect();
    let w: Vec<f64> = pw.iter().map(|h| h[3]).collect();
    let rational = w.iter().any(|&x| (x - 1.0).abs() > 1e-15);
    (pts, rational.then_some(w))
}

fn nurbs_curve(degree: usize, knots: Vec<f64>, pw: &[Hom]) -> Result<NurbsCurve> {
    if pw.iter().any(|h| h[3].is_nan() || h[3] <= 0.0) {
        return Err(Error::Numerical(
            "a loft edge came out with a weight that is not positive".into(),
        ));
    }
    let (pts, w) = split_hom(pw);
    NurbsCurve::new(degree, knots, pts, w)
}

/// How far a loft face's surface runs on past each of its edges, as a fraction of its parameter
/// range. Intersections that end on an edge are then traced and fitted across it, as on a swept
/// surface, rather than stopping where the surface does.
const RUN_ON: f64 = 0.1;

/// The surface through one column's rails at the `picked` sections, `u` along the rails and `v`
/// across the sections, carried on past every edge that is not a seam.
fn skin(col: &[Rail], picked: &[usize], acr: &Across) -> Result<Surface> {
    let closed = col[0].start().dist(col[0].end()) <= tol::LINEAR;
    let rails: Vec<(Vec<f64>, Vec<Hom>)> = picked
        .iter()
        .map(|&s| {
            let r = &col[s];
            if closed {
                (r.knots.clone(), r.pw.clone())
            } else {
                rail::extended(r.degree, &r.knots, &r.pw, RUN_ON)
            }
        })
        .collect();
    let ku = rails[0].0.clone();
    let nu = rails[0].1.len();
    let mut kv = Vec::new();
    let mut net: Vec<Hom> = Vec::new();
    for m in 0..nu {
        let data: Vec<Hom> = rails.iter().map(|r| r.1[m]).collect();
        let (k, ctrl) = rail::extended(acr.degree, &acr.knots, &acr.apply(&data), RUN_ON);
        kv = k;
        net.extend(ctrl);
    }
    let nv = net.len() / nu;
    if net.iter().any(|h| h[3].is_nan() || h[3] <= 0.0) {
        return Err(Error::Numerical(
            "a loft surface came out with a weight that is not positive".into(),
        ));
    }
    let (pts, w) = split_hom(&net);
    let s = NurbsSurface::new((col[0].degree, acr.degree), (ku, kv), (nu, nv), pts, w)?;
    Ok(Surface::Nurbs(Box::new(s)))
}
