// yack: faces moved onto any new surfaces, each edge traced where its faces' surfaces meet (from lib.rs, hardening T1).
// Adapted from springrep@a03e8a1 src/edit/tweak.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from edit/tweak.rs to tweak/mod.rs (M2.4, T6) and to yack-tweak (M3 W7); split out of
// lib.rs by what it does (hardening T1); checked access, the solid finished by the crate's `finished`
// (hardening T2); a rigid move pins the vertices its faces all go with (move_faces fix, w-h1).
//! Faces moved onto any new surfaces with the solid's topology kept: each vertex settles where its
//! faces' new surfaces meet, and each edge is their meeting between its moved ends.

use std::collections::BTreeMap;
use std::f64::consts::TAU;
use crate::{Error, Result, error::invalid, geom::Curve, geom::Line, geom::Surface, math::Frame, math::Point3, math::Transform, math::Vec2, math::Vec3, numeric::solve_dense, topo::Brep, topo::CoedgeSpec, topo::FaceId, topo::PCurve, topo::VertexId, unsupported, vertex_faces, clear_of_contacts, near_uv, tol, edge_faces, entry, finished};

/// The point nearest `p` on every surface of `on`, by Gauss–Newton steps along their normals:
/// the least move that puts it on all of them where they leave it free (a seam's end on two),
/// their one meeting where three or more pin it.
fn settle(p: Point3, on: &[&Surface]) -> Option<Point3> {
    let mut x = p;
    for _ in 0..60 {
        let rows: Vec<(Vec3, f64)> = on
            .iter()
            .map(|s| {
                let uv = s.project(x);
                let q = s.eval(uv.x, uv.y);
                let n = s.normal(uv.x, uv.y);
                (n, n.dot(q - x))
            })
            .collect();
        if rows.iter().all(|r| r.1.abs() <= 1e-3 * tol::LINEAR) {
            return Some(x);
        }
        // Faces tangent at the point (a round and the wall it runs into) stay tangent when both
        // move by one distance: their rows say the same, and one of them is kept.
        let mut kept: Vec<(Vec3, f64)> = Vec::new();
        for r in rows {
            if !kept.iter().any(|q| q.0.cross(r.0).norm() <= 1e-9) {
                kept.push(r);
            }
        }
        let rows = kept;
        let k = rows.len();
        let step = if k <= 3 {
            // Least norm: x moves by Jᵀy with (J Jᵀ) y = r.
            let a: Vec<f64> = rows.iter().flat_map(|ri| rows.iter().map(move |rj| ri.0.dot(rj.0))).collect();
            let y = solve_dense(a, rows.iter().map(|r| r.1).collect(), k, 1).ok()?;
            rows.iter()
                .zip(&y)
                .fold(Vec3::ZERO, |acc, (r, &yi)| acc + r.0 * yi)
        } else {
            // The least-squares point: (Σ n nᵀ) s = Σ n r.
            let mut a = [[0.0; 3]; 3];
            let mut rhs = [0.0; 3];
            for (n, r) in &rows {
                let n = n.to_array();
                for (row, &ni) in a.iter_mut().zip(&n) {
                    for (cell, &nj) in row.iter_mut().zip(&n) {
                        *cell += ni * nj;
                    }
                }
                for (hi, &ni) in rhs.iter_mut().zip(&n) {
                    *hi += ni * r;
                }
            }
            let &[sx, sy, sz] = solve_dense(a.iter().flatten().copied().collect(), rhs.to_vec(), 3, 1).ok()?.as_slice() else {
                return None;
            };
            Vec3::new(sx, sy, sz)
        };
        if !(step.x.is_finite() && step.y.is_finite() && step.z.is_finite()) {
            return None;
        }
        x += step;
    }
    None
}

/// The circle through three points, if they are not on a line.
fn circle_through(a: Point3, b: Point3, c: Point3) -> Option<crate::geom::Circle> {
    let (ab, ac) = (b - a, c - a);
    let n = ab.cross(ac);
    let n2 = n.norm2();
    if n2 <= 1e-24 {
        return None;
    }
    let centre = a + (n.cross(ab) * ac.norm2() + ac.cross(n) * ab.norm2()) * (0.5 / n2);
    let radius = centre.dist(a);
    let frame = Frame::new(centre, n, a - centre).ok()?;
    Some(crate::geom::Circle { frame, radius })
}


/// `b` with each face of `moved` on its new surface (natural normal on the side of its old one),
/// its topology kept: every vertex goes where its faces' surfaces meet, and every edge becomes the
/// meeting of its two faces' surfaces between its new vertices (a straight one stays a line where
/// that line lies on both), found in closed form where one exists and traced otherwise.
pub fn move_faces(b: &Brep, moved: &BTreeMap<FaceId, Surface>) -> Result<Brep> {
    move_faces_pinned(b, moved, &BTreeMap::new())
}

/// `b` with each face of `faces` carried by the rigid motion `motion`, as [`move_faces`] puts
/// each on its moved surface.
pub fn move_faces_by(b: &Brep, faces: &[FaceId], motion: &Transform) -> Result<Brep> {
    let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = motion.r;
    // Orthonormal columns, and no mirror.
    let columns = [Vec3::new(r00, r10, r20), Vec3::new(r01, r11, r21), Vec3::new(r02, r12, r22)];
    let square = columns.iter().enumerate().all(|(i, ci)| {
        columns
            .iter()
            .enumerate()
            .all(|(j, &cj)| (ci.dot(cj) - f64::from(u8::from(i == j))).abs() <= tol::ANGULAR)
    });
    let det = r00 * (r11 * r22 - r12 * r21) - r01 * (r10 * r22 - r12 * r20) + r02 * (r10 * r21 - r11 * r20);
    if !square || (det - 1.0).abs() > tol::ANGULAR {
        return invalid("faces move by a rigid motion: a rotation and a shift");
    }
    let moved: BTreeMap<FaceId, Surface> = faces
        .iter()
        .map(|&f| (f, b.face(f).surface.transformed(motion)))
        .collect();
    // A face the motion carries onto itself (a wall the moved faces slide along) is as good as
    // moved with them. A vertex whose faces all move with the motion so goes with it exactly:
    // settled onto its faces' new surfaces instead, it would have to find a tangency (a round
    // sliding along the wall it meets), where the settle is slow and lands off it.
    let carries = |f: FaceId| {
        moved.contains_key(&f) || {
            let s = &b.face(f).surface;
            let mut on = b.face(f).loops.iter().flat_map(|&lp| b.lp(lp).coedges.iter()).filter_map(|&c| {
                let x = b.edge(b.coedge(c).edge);
                x.curve.as_ref().map(|cv| [b.vertex(x.start).point, cv.eval(0.5 * (x.range.0 + x.range.1))])
            });
            on.all(|pts| pts.iter().all(|&q| crate::intersect::csi::signed_distance(s, motion.point(q)).abs() <= tol::LINEAR))
        }
    };
    let carried: std::collections::BTreeSet<FaceId> = b.face_ids().filter(|&f| carries(f)).collect();
    let pins: BTreeMap<VertexId, Point3> = vertex_faces(b)
        .into_iter()
        // Only where an unmoved face is carried too: a vertex of moved faces alone settles at the
        // motion's image already.
        .filter(|(_, fs)| fs.iter().any(|f| moved.contains_key(f)) && fs.iter().any(|f| !moved.contains_key(f)) && fs.iter().all(|f| carried.contains(f)))
        .map(|(v, _)| (v, motion.point(b.vertex(v).point)))
        .collect();
    moved_with(b, &moved, &pins, Some((motion, &carried)))
}

/// Where the pole at `uv` of the surface it was a pole of lies on `s`: a moved cone keeps its
/// parameters, so its apex is where its radius runs out, `v = -radius / sin a`, and not where the
/// apex of the old cone was; on any other surface a pole stays at its parameters.
fn pole_uv(s: &Surface, uv: Vec2) -> Vec2 {
    match s {
        Surface::Cone(c) if c.half_angle.sin().abs() > 1e-9 => Vec2::new(uv.x, -c.radius / c.half_angle.sin()), // tol: a cone this close to a cylinder has no apex
        _ => uv,
    }
}

/// [`move_faces`], with each vertex of `pins` put at its point where that lies on every new surface
/// at the vertex. A vertex its surfaces leave free to slide (a seam's end, on a circle) otherwise
/// moves the least distance onto them, which keeps how far along the curve it was.
pub fn move_faces_pinned(
    b: &Brep,
    moved: &BTreeMap<FaceId, Surface>,
    pins: &BTreeMap<VertexId, Point3>,
) -> Result<Brep> {
    moved_with(b, moved, pins, None)
}

/// [`move_faces_pinned`]; with `carry`, a rigid motion and the faces it carries onto their new
/// surfaces (the moved ones, and any it slides along itself), each edge of carried faces alone
/// whose ends are pinned is the old edge carried by the motion. Traced instead, its old middle
/// settled onto the new surfaces can land on the wrong side of a closed curve (a round's end arc
/// moved further than its radius), and the edge would run the long way round.
fn moved_with(
    b: &Brep,
    moved: &BTreeMap<FaceId, Surface>,
    pins: &BTreeMap<VertexId, Point3>,
    carry: Option<(&Transform, &std::collections::BTreeSet<FaceId>)>,
) -> Result<Brep> {
    clear_of_contacts(b, moved.keys())?;
    let surface_of = |f: FaceId| moved.get(&f).unwrap_or(&b.face(f).surface);
    let around = vertex_faces(b);
    let edge_faces = edge_faces(b);
    // A pole (a degenerate edge's vertex) goes to its surface's new pole exactly: its parameters
    // there are the old ones, while a point settled onto the surface sits off the pole by about √ε
    // in angle, where u is no longer free and projection gives any u at all.
    let mut poles: BTreeMap<VertexId, Point3> = BTreeMap::new();
    for c in b.coedge_ids() {
        let co = b.coedge(c);
        let edge = b.edge(co.edge);
        let f = b.coedge_face(c);
        if edge.curve.is_some() || !moved.contains_key(&f) {
            continue;
        }
        if let PCurve::Affine { origin, dir } = &co.pcurve {
            let uv = pole_uv(surface_of(f), *origin + *dir * edge.range.0);
            poles.insert(edge.start, surface_of(f).eval(uv.x, uv.y));
        }
    }
    let mut out = Brep::new();
    // By `b`'s vertex, in id order: the new vertex and whether it moved.
    let mut verts = Vec::new();
    let mut shifted = Vec::new();
    for v in b.vertex_ids() {
        let faces = around.get(&v).cloned().unwrap_or_default();
        let p = b.vertex(v).point;
        if !faces.iter().any(|f| moved.contains_key(f)) {
            verts.push(out.add_vertex(p));
            shifted.push(false);
            continue;
        }
        let on: Vec<&Surface> = faces.iter().map(|&f| surface_of(f)).collect();
        let pinned = pins.get(&v).or(poles.get(&v)).copied().filter(|&q| {
            on.iter()
                .all(|s| crate::intersect::csi::signed_distance(s, q).abs() <= 1e-2 * tol::LINEAR)
        });
        let Some(q) = pinned.or_else(|| settle(p, &on)) else {
            return unsupported("moving a corner whose faces' new surfaces do not meet near it");
        };
        shifted.push(q.dist(p) > 0.0);
        verts.push(out.add_vertex(q));
    }
    // By `b`'s edge, in id order: the new edge, whether it was laid anew, and whether it runs the
    // other way round.
    let mut edges = Vec::new();
    let mut rebuilt = Vec::new();
    let mut flipped = Vec::new();
    for e in b.edge_ids() {
        rebuilt.push(false);
        flipped.push(false);
        let x = b.edge(e);
        let (s, t) = (entry(&verts, x.start.index())?, entry(&verts, x.end.index())?);
        let mut faces = edge_faces.get(&e).cloned().unwrap_or_default();
        faces.sort();
        faces.dedup();
        let touched = faces.iter().any(|f| moved.contains_key(f))
            || entry(&shifted, x.start.index())?
            || entry(&shifted, x.end.index())?;
        let Some(old) = &x.curve else {
            edges.push(out.add_degenerate_edge(s, x.range));
            continue;
        };
        if !touched {
            edges.push(out.add_edge(old.clone(), x.range, s, t));
            continue;
        }
        if let Some((motion, carried)) = carry
            && faces.iter().all(|f| carried.contains(f))
            && pins.contains_key(&x.start)
            && pins.contains_key(&x.end)
        {
            if let Some(laid) = rebuilt.last_mut() {
                *laid = true;
            }
            edges.push(out.add_edge(old.transformed(motion), x.range, s, t));
            continue;
        }
        if let Some(laid) = rebuilt.last_mut() {
            *laid = true;
        }
        let on: Vec<&Surface> = faces.iter().map(|&f| surface_of(f)).collect();
        let lies_on = |c: &Curve, r: (f64, f64)| {
            (1..8).all(|k| {
                let p = c.eval(r.0 + (r.1 - r.0) * k as f64 / 8.0);
                on.iter()
                    .all(|sf| crate::intersect::csi::signed_distance(sf, p).abs() <= tol::LINEAR)
            })
        };
        let (a, z) = (out.vertex(s).point, out.vertex(t).point);
        if let (Curve::Line(old_line), Some((line, len))) = (old, Line::through(a, z))
            && lies_on(&Curve::Line(line.clone()), (0.0, len))
        {
            if line.dir.dot(old_line.dir) <= 0.0 {
                return Err(Error::InvalidResult(vec![
                    "a moved edge runs backwards: a face turns inside out".into(),
                ]));
            }
            edges.push(out.add_edge(Curve::Line(line), (0.0, len), s, t));
            continue;
        }
        if on.len() != 2 {
            // A seam: the new surface keeps its parameters (a frame kept, a radius grown), so the
            // seam is the same parameter line on it, here a circle through three of its points.
            let (Some(&f), true) = (faces.first(), faces.len() == 1) else {
                return unsupported("moving a curved edge of three faces");
            };
            let (from, to) = (&b.face(f).surface, surface_of(f));
            let fractions: &[f64] = if x.start == x.end {
                &[0.0, 1.0 / 3.0, 2.0 / 3.0]
            } else {
                &[0.0, 0.5, 1.0]
            };
            let &[_, second, third] = fractions
                .iter()
                .map(|&k| {
                    let q = old.eval(x.range.0 + (x.range.1 - x.range.0) * k);
                    let uv = from.project(q);
                    to.eval(uv.x, uv.y)
                })
                .collect::<Vec<Point3>>()
                .as_slice()
            else {
                return unsupported("moving a seam that is not a circle");
            };
            // Its ends are the moved vertices, which a face beside it may have carried along it.
            let [first, second, third] = if x.start == x.end { [a, second, third] } else { [a, second, z] };
            let Some(circle) = circle_through(first, second, third) else {
                return unsupported("moving a seam that is not a circle");
            };
            let c = Curve::Circle(circle);
            let lo = c.project(first, 0.0, TAU);
            let hi = if x.start == x.end {
                lo + TAU
            } else {
                let mut h = c.project(third, 0.0, TAU);
                while h <= lo {
                    h += TAU;
                }
                h
            };
            let (old_mid, old_dir) = old.d1(0.5 * (x.range.0 + x.range.1));
            let _ = old_mid;
            let mid_t = 0.5 * (lo + hi);
            if c.d1(mid_t).1.dot(old_dir) <= 0.0 && x.start != x.end {
                return unsupported("moving a seam that turns the other way");
            }
            edges.push(out.add_edge(c, (lo, hi), s, t));
            continue;
        }
        // The old edge's middle and direction, carried onto both new surfaces.
        let mid_t = 0.5 * (x.range.0 + x.range.1);
        let (old_mid, old_dir) = old.d1(mid_t);
        let Some(mid) = settle(old_mid, &on) else {
            return unsupported("moving an edge whose faces' new surfaces do not meet near it");
        };
        let mut region = old.bbox(x.range.0, x.range.1);
        region.add(a);
        region.add(z);
        region.add(mid);
        let reach = old_mid.dist(mid) + a.dist(b.vertex(x.start).point) + 1.0;
        let pad = Vec3::new(reach, reach, reach);
        let region = crate::math::Aabb {
            min: region.min - pad,
            max: region.max + pad,
        };
        let &[one, other] = on.as_slice() else {
            return unsupported("moving a curved edge of three faces");
        };
        let ssi = crate::intersect::ssi::intersect(one, other, &region, &mut || vec![mid])?;
        let crate::intersect::ssi::Ssi::Curves { curves, .. } = ssi else {
            return unsupported("moving an edge between faces that come to coincide");
        };
        let Some(best) = curves
            .iter()
            .map(|sc| {
                let tm = sc.curve.project(mid, sc.range.0, sc.range.1);
                (sc.curve.eval(tm).dist(mid), sc)
            })
            .filter(|(d, _)| *d <= 10.0 * tol::LINEAR)
            .min_by(|p, q| p.0.total_cmp(&q.0))
            .map(|(_, sc)| sc)
        else {
            return unsupported("moving an edge whose new faces meet nowhere near it");
        };
        let c = &best.curve;
        let (r0, r1) = best.range;
        let ts = c.project(a, r0, r1);
        let te = c.project(z, r0, r1);
        let tm = c.project(mid, r0, r1);
        // A closed edge's direction is its turn: moved further than its size, its old middle
        // settles anywhere round the new one, where the tangent can run either way.
        let along = if x.start == x.end {
            let (_, d1, d2) = old.d2(mid_t);
            let (_, e1, e2) = c.d2(tm);
            d1.cross(d2).dot(e1.cross(e2)) > 0.0
        } else {
            c.d1(tm).1.dot(old_dir) > 0.0
        };
        let period = c.period();
        // The stretch from the start vertex to the end one through the middle, in the curve's own
        // direction; where that runs from the end, the edge is laid the other way.
        let span = |from: f64, to: f64| -> (f64, f64) {
            match period {
                Some(p) => {
                    let mut hi = to;
                    while hi <= from + 1e-12 {
                        hi += p;
                    }
                    while hi - from > p + 1e-12 {
                        hi -= p;
                    }
                    (from, hi)
                }
                None => (from, to),
            }
        };
        let closed = x.start == x.end;
        let (lo, hi, start, end) = if along {
            let (lo, hi) = if closed {
                (ts, ts + period.unwrap_or(r1 - r0))
            } else {
                span(ts, te)
            };
            (lo, hi, s, t)
        } else {
            let (lo, hi) = if closed {
                (ts, ts + period.unwrap_or(r1 - r0))
            } else {
                span(te, ts)
            };
            if let Some(back) = flipped.last_mut() {
                *back = true;
            }
            (lo, hi, t, s)
        };
        if hi - lo <= 0.0
            || c.eval(lo).dist(out.vertex(start).point) > tol::LINEAR
            || c.eval(hi).dist(out.vertex(end).point) > tol::LINEAR
        {
            return unsupported("moving an edge whose new curve does not run between its vertices");
        }
        edges.push(out.add_edge(c.clone(), (lo, hi), start, end));
    }
    let mut faces = Vec::new();
    for f in b.face_ids() {
        let face = b.face(f);
        let surface = surface_of(f).clone();
        // A loop with anything rebuilt is re-seeded whole, so its parameters run on unbroken.
        let changed = moved.contains_key(&f)
            || face.loops.iter().any(|&lp| {
                b.lp(lp)
                    .coedges
                    .iter()
                    .any(|&c| rebuilt.get(b.coedge(c).edge.index()) == Some(&true))
            });
        let nf = out.add_face(surface.clone(), face.reversed, face.names.clone());
        for &lp in &face.loops {
            // The loop starts where it did: a moved surface keeps its parameters, so the old
            // start places a periodic u on the right side of the period.
            let mut last: Option<Vec2> = b.lp(lp).coedges.first().and_then(|&c| {
                let t = *crate::topo::coedge_params(b, c, 1).first()?;
                b.coedge_uvs(c, &[t]).first().copied()
            });
            let specs = b
                .lp(lp)
                .coedges
                .iter()
                .map(|&c| {
                    let co = b.coedge(c);
                    let edge = entry(&edges, co.edge.index())?;
                    let reversed = co.reversed != entry(&flipped, co.edge.index())?;
                    let x = out.edge(edge);
                    // A degenerate edge (a pole) keeps its own line across the parameter plane,
                    // and the next coedge carries on from its end.
                    if x.curve.is_none() {
                        let mut pcurve = co.pcurve.clone();
                        if let PCurve::Affine { origin, dir } = &mut pcurve {
                            // The pole's line moves with the pole, across the line only.
                            let at = *origin + *dir * x.range.0;
                            let shift = pole_uv(&surface, at) - at;
                            let along = dir.dot(*dir);
                            if along > 0.0 {
                                *origin = *origin + (shift - *dir * (shift.dot(*dir) / along));
                            }
                            let t = if reversed { x.range.0 } else { x.range.1 };
                            last = Some(*origin + *dir * t);
                        }
                        return Ok(CoedgeSpec { edge, reversed, pcurve });
                    }
                    let pcurve = if changed || entry(&rebuilt, co.edge.index())? {
                        let from = if reversed { x.end } else { x.start };
                        let mut seed = near_uv(&surface, out.vertex(from).point, last);
                        if let Some(cv) = &x.curve {
                            let (t0, t1) = if reversed {
                                (x.range.1, x.range.0)
                            } else {
                                x.range
                            };
                            // As the pcurve itself reads its start: at a pole, the u the edge
                            // leaves it by, not whatever projecting the pole gives.
                            let hint = last.unwrap_or(seed);
                            seed = crate::topo::pcurve::project_on(&surface, cv, t0, t1, hint);
                        }
                        let mut uv = seed;
                        if let Some(cv) = &x.curve {
                            let (t0, t1) = if reversed {
                                (x.range.1, x.range.0)
                            } else {
                                x.range
                            };
                            for k in 1..=32 {
                                let p = cv.eval(t0 + (t1 - t0) * k as f64 / 32.0);
                                uv = near_uv(&surface, p, Some(uv));
                            }
                        }
                        last = Some(uv);
                        PCurve::Projected { seed }
                    } else {
                        co.pcurve.clone()
                    };
                    Ok(CoedgeSpec {
                        edge,
                        reversed,
                        pcurve,
                    })
                })
                .collect::<Result<Vec<_>>>()?;
            out.add_loop(nf, specs);
        }
        faces.push(nf);
    }
    finished(b, out, &faces)
}
