// yack: rounds a move of the faces goes past, taken out and their edges left sharp (offset step 2, w-h1).
//! Rounds a move of the faces goes past. Moving a solid's faces in by `d` takes a convex round of
//! radius `r ≤ d` past its axis, and moving them out by `d` a concave one: the round vanishes from
//! the moved solid, whose neighbours meet in a sharp edge. The eroded (or grown) solid there is
//! the erosion (or growth) of the edge the round rounded, so the round is taken out first and its
//! two neighbours run on to the edge they met in, and then the faces move.
//!
//! Each round taken out is a ring: a torus between two coaxial surfaces of revolution (a
//! plane square to the axis, a cylinder, a cone or a ball), its two rails full circles. The edge
//! its neighbours meet in is where their meridians cross, in the half plane of the axis: a circle.

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use crate::{
    Error, Result, check,
    geom::{Circle, Curve, Surface, Torus},
    math::{Frame, Point3, Vec2, Vec3},
    near_uv, tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceId, PCurve, VertexId},
};

/// `b` with every round that `by` moves past its radius taken out (a convex one moved in, a
/// concave one out), its neighbours meeting in
/// the sharp edge it rounded, and for each face of `b` kept its face in the result; `None` where
/// no round goes. Declines (`Unsupported`) a round it does not take out exactly: one that is no
/// ring between two coaxial surfaces of revolution, whose neighbours' meridians miss each other,
/// or whose rails' vertices lie on different meridians.
pub fn drop_vanishing_rounds(b: &Brep, by: &BTreeMap<FaceId, f64>) -> Result<Option<(Brep, BTreeMap<FaceId, FaceId>)>> {
    let mut cur = b.clone();
    let mut map: BTreeMap<FaceId, FaceId> = b.face_ids().map(|f| (f, f)).collect();
    let mut dropped = false;
    loop {
        // The rounds still to go, as faces of `cur`: each face of `b` that `by` moves past its radius.
        let next = map.iter().find_map(|(&f, &g)| {
            let face = cur.face(g);
            // A ring round's tube, or a ball's cap rounding a cone's tip (convex only).
            let radius = match &face.surface {
                Surface::Torus(t) => t.minor,
                Surface::Sphere(s) if !face.reversed => s.radius,
                _ => return None,
            };
            // How far the move takes the face towards the round's axis: in for a convex round,
            // out for a concave one.
            let towards = if face.reversed { by.get(&f).copied()? } else { -by.get(&f).copied()? };
            (towards >= radius - tol::LINEAR).then_some(g)
        });
        let Some(round) = next else { break };
        let (out, kept) = match &cur.face(round).surface {
            Surface::Sphere(_) => drop_cap(&cur, round)?,
            _ => drop_round(&cur, round)?,
        };
        map = map.into_iter().filter_map(|(f, g)| Some((f, *kept.get(&g)?))).collect();
        cur = out;
        dropped = true;
    }
    Ok(dropped.then_some((cur, map)))
}

/// The meridian of a surface of revolution about the axis `(o, k)`, in the half plane's `(ρ, z)`:
/// a line through a point along a direction, or a circle about a point.
enum Meridian {
    Line(Vec2, Vec2),
    Circle(Vec2, f64),
}

/// `s`'s meridian about `(o, k)`, if it is a surface of revolution about that axis.
fn meridian(s: &Surface, o: Point3, k: Vec3) -> Option<Meridian> {
    let on_axis = |p: Point3| (p - o).cross(k).norm() <= tol::LINEAR;
    let along = |z: Vec3| z.cross(k).norm() <= tol::TANGENT;
    let height = |p: Point3| (p - o).dot(k);
    match s {
        Surface::Plane(pl) if along(pl.frame.z) => Some(Meridian::Line(Vec2::new(0.0, height(pl.frame.origin)), Vec2::new(1.0, 0.0))),
        Surface::Cylinder(c) if along(c.frame.z) && on_axis(c.frame.origin) => {
            Some(Meridian::Line(Vec2::new(c.radius, 0.0), Vec2::new(0.0, 1.0)))
        }
        Surface::Cone(c) if along(c.frame.z) && on_axis(c.frame.origin) => {
            let up = c.frame.z.dot(k).signum();
            Some(Meridian::Line(
                Vec2::new(c.radius, height(c.frame.origin)),
                Vec2::new(c.half_angle.sin(), up * c.half_angle.cos()),
            ))
        }
        Surface::Sphere(sp) if on_axis(sp.frame.origin) => Some(Meridian::Circle(Vec2::new(0.0, height(sp.frame.origin)), sp.radius)),
        _ => None,
    }
}

/// Where the meridians `a` and `c` cross, the crossing nearest `near` with `ρ > 0`.
fn crossing(a: &Meridian, c: &Meridian, near: Vec2) -> Option<Vec2> {
    let line_circle = |p: Vec2, d: Vec2, m: Vec2, r: f64| -> Vec<Vec2> {
        // |p + t·d − m|² = r², d not unit.
        let w = p - m;
        let (qa, qb, qc) = (d.dot(d), 2.0 * d.dot(w), w.dot(w) - r * r);
        let disc = qb * qb - 4.0 * qa * qc;
        if disc < 0.0 || qa <= 0.0 {
            return Vec::new();
        }
        let s = disc.sqrt();
        vec![p + d * ((-qb - s) / (2.0 * qa)), p + d * ((-qb + s) / (2.0 * qa))]
    };
    let found: Vec<Vec2> = match (a, c) {
        (Meridian::Line(p, d), Meridian::Line(q, e)) => {
            let den = d.cross(*e);
            if den.abs() <= tol::TANGENT {
                return None;
            }
            vec![*p + *d * ((*q - *p).cross(*e) / den)]
        }
        (Meridian::Line(p, d), Meridian::Circle(m, r)) | (Meridian::Circle(m, r), Meridian::Line(p, d)) => line_circle(*p, *d, *m, *r),
        (Meridian::Circle(..), Meridian::Circle(..)) => return None,
    };
    found
        .into_iter()
        .filter(|x| x.x > tol::LINEAR)
        .min_by(|x, y| (*x - near).norm().total_cmp(&(*y - near).norm()))
}

/// The outward normal of face `f` of `b` at `p`, a point on it.
fn outward(b: &Brep, f: FaceId, p: Point3) -> Vec3 {
    let s = &b.face(f).surface;
    let uv = s.project(p);
    let n = s.normal(uv.x, uv.y);
    if b.face(f).reversed { -n } else { n }
}

/// Whether the round `r` is tangent to face `g` all along their edge `e`: their outward normals
/// agree at each of a rail's sample points.
fn tangent_along(b: &Brep, r: FaceId, e: EdgeId, g: FaceId) -> bool {
    let x = b.edge(e);
    let Some(c) = &x.curve else { return false };
    (0..8).all(|i| {
        let p = c.eval(x.range.0 + (x.range.1 - x.range.0) * f64::from(i) / 8.0);
        let (n, m) = (outward(b, r, p), outward(b, g, p));
        n.cross(m).norm() <= tol::TANGENT && n.dot(m) > 0.0
    })
}

/// `p` in the half plane of the axis `(o, k)`: its distance from the axis and its height.
fn in_meridian(p: Point3, (o, k): (Point3, Vec3)) -> Vec2 {
    let q = p - o;
    let z = q.dot(k);
    Vec2::new((q - k * z).norm(), z)
}

/// Whether some edge of `b` other than the round `r`'s own comes inside the region the round cut
/// off its corner: in the half plane of the axis, the region between the rails' points `a` and
/// `c`, the corner `x` their faces meet in, and the round's arc about `spine` of radius `minor`.
fn intrudes(b: &Brep, r: FaceId, axis: (Point3, Vec3), (a, x, c): (Vec2, Vec2, Vec2), (spine, minor): (Vec2, f64)) -> bool {
    // The arc from `c` back to `a` the short way, the side the corner is on.
    let (ta, tc) = ((a - spine).y.atan2((a - spine).x), (c - spine).y.atan2((c - spine).x));
    let mut sweep = ta - tc;
    while sweep > std::f64::consts::PI {
        sweep -= TAU;
    }
    while sweep < -std::f64::consts::PI {
        sweep += TAU;
    }
    let mut poly = vec![a, x, c];
    poly.extend((1..16).map(|i| {
        let t = tc + sweep * f64::from(i) / 16.0;
        spine + Vec2::new(t.cos(), t.sin()) * minor
    }));
    let off_boundary = |q: Vec2| {
        poly.iter().zip(poly.iter().cycle().skip(1)).all(|(&u, &v)| {
            let d = v - u;
            let s = ((q - u).dot(d) / d.dot(d).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
            (u + d * s).dist(q) > tol::LINEAR
        })
    };
    let inside = |q: Vec2| {
        let mut wind = false;
        for (&u, &v) in poly.iter().zip(poly.iter().cycle().skip(1)) {
            if (u.y > q.y) != (v.y > q.y) && q.x < u.x + (q.y - u.y) / (v.y - u.y) * (v.x - u.x) {
                wind = !wind;
            }
        }
        wind && off_boundary(q)
    };
    let own: std::collections::BTreeSet<EdgeId> =
        b.face(r).loops.iter().flat_map(|&lp| b.lp(lp).coedges.iter().map(|&co| b.coedge(co).edge)).collect();
    b.edge_ids().filter(|e| !own.contains(e)).any(|e| {
        let x = b.edge(e);
        x.curve.as_ref().is_some_and(|cv| {
            (0..=16).any(|i| inside(in_meridian(cv.eval(x.range.0 + (x.range.1 - x.range.0) * f64::from(i) / 16.0), axis)))
        })
    })
}

/// The face across `e` from `f`.
fn across(b: &Brep, e: EdgeId, f: FaceId) -> Option<FaceId> {
    b.edge_uses(e).iter().map(|&c| b.coedge_face(c)).find(|&g| g != f)
}

/// `b` with the ring round `r` taken out (see [`drop_vanishing_rounds`]), and each face of `b`
/// kept mapped on to its face in the result. Only a round the move takes past its axis (a convex
/// one in, a concave one out; the caller's test), tangent to both its faces along both rails, with
/// nothing else in the corner it cut off: the move of the round is then the move of that edge.
fn drop_round(b: &Brep, r: FaceId) -> Result<(Brep, BTreeMap<FaceId, FaceId>)> {
    let no = |why: &str| Error::Unsupported(format!("taking out a round the offset goes past: {why}"));
    let Surface::Torus(Torus { frame, major, minor }) = &b.face(r).surface else {
        return Err(no("the round is no torus"));
    };
    let (o, k) = (frame.origin, frame.z);
    // The round's rails, each a full circle its neighbour shares; its seams, used by it alone.
    let mut rails: Vec<(EdgeId, FaceId)> = Vec::new();
    for &lp in &b.face(r).loops {
        for &c in &b.lp(lp).coedges {
            let e = b.coedge(c).edge;
            if let Some(g) = across(b, e, r) {
                rails.push((e, g));
            }
        }
    }
    let &[(ea, fa), (eb, fb)] = rails.as_slice() else {
        return Err(no("the round is no ring with one rail on each of two faces"));
    };
    if fa == fb || [ea, eb].iter().any(|&e| b.edge(e).start != b.edge(e).end || b.edge(e).curve.is_none()) {
        return Err(no("the round's rails are not two closed curves on two faces"));
    }
    let (Some(ma), Some(mb)) = (meridian(&b.face(fa).surface, o, k), meridian(&b.face(fb).surface, o, k)) else {
        return Err(no("a neighbour is no surface of revolution about the round's axis"));
    };
    let Some(x) = crossing(&ma, &mb, Vec2::new(*major, 0.0)).filter(|x| (*x - Vec2::new(*major, 0.0)).norm() <= 10.0 * minor) else {
        return Err(no("the neighbours' meridians do not meet by the round"));
    };
    // A true rolling-ball round: tangent to both its faces all along both rails. Then any ball of
    // the move's radius inside the solid is made of balls of the round's radius that keep out of
    // the corner it cut off, so the move past it is the sharp edge's, whatever lies near.
    if !tangent_along(b, r, ea, fa) || !tangent_along(b, r, eb, fb) {
        return Err(no("the round is not tangent to both its faces"));
    }
    let rail_at = |e: EdgeId| {
        let x = b.edge(e);
        b.edge(e).curve.as_ref().map(|c| in_meridian(c.eval(0.5 * (x.range.0 + x.range.1)), (o, k)))
    };
    let (Some(pa), Some(pb)) = (rail_at(ea), rail_at(eb)) else {
        return Err(no("a rail has no curve"));
    };
    if intrudes(b, r, (o, k), (pa, x, pb), (Vec2::new(*major, 0.0), *minor)) {
        return Err(no("another edge lies in the corner the round cut off"));
    }
    // The new edge's one vertex, on the rails' vertices' meridian.
    let (va, vb) = (b.edge(ea).start, b.edge(eb).start);
    let radial = |v: VertexId| {
        let p = b.vertex(v).point - o;
        (p - k * p.dot(k)).unit()
    };
    let (Some(ua), Some(ub)) = (radial(va), radial(vb)) else {
        return Err(no("a rail's vertex lies on the axis"));
    };
    if ua.cross(ub).norm() > tol::TANGENT || ua.dot(ub) < 0.0 {
        return Err(no("the rails' vertices lie on different meridians"));
    }
    let w = o + ua * x.x + k * x.y;
    let ring = Frame::new(o + k * x.y, k, ua)?;
    let circle = Curve::Circle(Circle { frame: ring, radius: x.x });
    // The new edge runs the way rail `ea` does: their tangents at the vertex agree.
    let tangent = |e: EdgeId| b.edge(e).curve.as_ref().map(|c| c.d1(b.edge(e).range.0).1);
    let (Some(ta), Some(tb)) = (tangent(ea), tangent(eb)) else {
        return Err(no("a rail has no curve"));
    };
    let along = circle.d1(0.0).1;
    let (flip_a, flip_b) = (along.dot(ta) < 0.0, along.dot(tb) < 0.0);
    rebuild(b, r, (ea, flip_a), (eb, flip_b), (va, vb), w, Some(circle)).map_err(|why| no(&why))
}

/// `b` with the ball's cap `r` that rounds a cone's tip taken out (see [`drop_vanishing_rounds`]),
/// the cone run on to its tip. Only a cap tangent to a coaxial cone all along its one rail, with
/// nothing else in the corner it cut off.
fn drop_cap(b: &Brep, r: FaceId) -> Result<(Brep, BTreeMap<FaceId, FaceId>)> {
    let no = |why: &str| Error::Unsupported(format!("taking out a ball's cap the offset goes past: {why}"));
    let Surface::Sphere(ball) = &b.face(r).surface else {
        return Err(no("the cap is no ball"));
    };
    let mut rails: Vec<(EdgeId, FaceId)> = Vec::new();
    for &lp in &b.face(r).loops {
        for &c in &b.lp(lp).coedges {
            let e = b.coedge(c).edge;
            if let Some(g) = across(b, e, r) {
                rails.push((e, g));
            }
        }
    }
    let &[(ea, fa)] = rails.as_slice() else {
        return Err(no("the cap is no ring with one rail on a cone"));
    };
    let Surface::Cone(cone) = &b.face(fa).surface else {
        return Err(no("the cap's neighbour is no cone"));
    };
    let k = cone.frame.z;
    let o = ball.frame.origin;
    if b.edge(ea).start != b.edge(ea).end || (o - cone.frame.origin).cross(k).norm() > tol::LINEAR || cone.half_angle.sin().abs() <= tol::TANGENT {
        return Err(no("the cap's rail is not closed, or its ball is off the cone's axis"));
    }
    if !tangent_along(b, r, ea, fa) {
        return Err(no("the cap is not tangent to its cone"));
    }
    // The cone's tip: where its radius runs out.
    let v_tip = -cone.radius / cone.half_angle.sin();
    let w = cone.frame.origin + k * (v_tip * cone.half_angle.cos());
    let axis = (o, k);
    let rail_at = {
        let x = b.edge(ea);
        b.edge(ea).curve.as_ref().map(|c| in_meridian(c.eval(0.5 * (x.range.0 + x.range.1)), axis))
    };
    let Some(pa) = rail_at else { return Err(no("the rail has no curve")) };
    let tip = in_meridian(w, axis);
    // The cap's pole, on the axis on the tip's side of the ball.
    let pole = Vec2::new(0.0, ball.radius * tip.y.signum());
    if intrudes(b, r, axis, (pa, tip, pole), (Vec2::new(0.0, 0.0), ball.radius)) {
        return Err(no("another edge lies in the tip the cap cut off"));
    }
    let va = b.edge(ea).start;
    rebuild(b, r, (ea, false), (ea, false), (va, va), w, None).map_err(|why| no(&why))
}

/// `b` without face `r`: rails `ea` and `eb` become one edge on `circle` (flipped against each
/// rail as marked), vertices `va` and `vb` the one vertex `w`, and every other edge ending at them
/// carried on along its curve to `w`. Checked; the reason where it is not a valid solid.
fn rebuild(
    b: &Brep,
    r: FaceId,
    (ea, flip_a): (EdgeId, bool),
    (eb, flip_b): (EdgeId, bool),
    (va, vb): (VertexId, VertexId),
    w: Point3,
    circle: Option<Curve>,
) -> std::result::Result<(Brep, BTreeMap<FaceId, FaceId>), String> {
    let mut out = Brep::new();
    let joined = out.add_vertex(w);
    let mut verts: BTreeMap<VertexId, VertexId> = BTreeMap::from([(va, joined), (vb, joined)]);
    let mut vertex = |out: &mut Brep, v: VertexId| *verts.entry(v).or_insert_with(|| out.add_vertex(b.vertex(v).point));
    // The ring the rails become, or, for a cap, the tip: a degenerate edge at `w`.
    let ring = match circle {
        Some(c) => out.add_edge(c, (0.0, TAU), joined, joined),
        None => out.add_degenerate_edge(joined, (0.0, TAU)),
    };
    // Every edge some kept face uses, the rails becoming the ring.
    let mut edges: BTreeMap<EdgeId, (EdgeId, bool)> = BTreeMap::from([(ea, (ring, flip_a)), (eb, (ring, flip_b))]);
    let faces: Vec<FaceId> = b.face_ids().filter(|&f| f != r).collect();
    for &f in &faces {
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                let e = b.coedge(c).edge;
                if edges.contains_key(&e) {
                    continue;
                }
                let x = b.edge(e);
                let (s, t) = (vertex(&mut out, x.start), vertex(&mut out, x.end));
                let id = match &x.curve {
                    Some(cv) => {
                        // Carried on to `w` where it ends at a rail's vertex.
                        let (mut t0, mut t1) = x.range;
                        let span = (t1 - t0).abs() + 10.0 * w.dist(b.vertex(x.start).point).max(w.dist(b.vertex(x.end).point));
                        // A closed curve's parameter is taken the turn nearest where the end was.
                        let near = |t: f64, was: f64| cv.period().map_or(t, |q| t + ((was - t) / q).round() * q);
                        if x.start == va || x.start == vb {
                            t0 = near(cv.project(w, t0 - span, t1), x.range.0);
                        }
                        if x.end == va || x.end == vb {
                            t1 = near(cv.project(w, t0, t1 + span), x.range.1);
                        }
                        if (s == joined && cv.eval(t0).dist(w) > tol::LINEAR) || (t == joined && cv.eval(t1).dist(w) > tol::LINEAR) {
                            return Err("an edge ending at a rail does not reach the new edge".into());
                        }
                        out.add_edge(cv.clone(), (t0, t1), s, t)
                    }
                    None => out.add_degenerate_edge(s, x.range),
                };
                edges.insert(e, (id, false));
            }
        }
    }
    let mut kept = BTreeMap::new();
    for &f in &faces {
        let face = b.face(f);
        let id = out.add_face(face.surface.clone(), face.reversed, face.names.clone());
        for &lp in &face.loops {
            let mut specs = Vec::new();
            for &c in &b.lp(lp).coedges {
                let co = b.coedge(c);
                let &(edge, flip) = edges.get(&co.edge).ok_or("a loop runs on an edge the rebuild lost")?;
                // The tip's degenerate edge is this coedge's alone, so it runs the coedge's way.
                let tip = (co.edge == ea || co.edge == eb) && out.edge(edge).curve.is_none();
                let reversed = !tip && co.reversed != flip;
                // Every edge keeps its parameters (one carried on to `w` along its own line); the
                // ring is projected, from where the rail it stands for began on this face.
                let pcurve = if tip {
                    // The tip: the rail's turn round the face, at the tip's parameters.
                    let ts = crate::topo::coedge_params(b, c, 2);
                    let uvs = b.coedge_uvs(c, &ts);
                    let (Some(&first), Some(&last)) = (uvs.first(), uvs.last()) else {
                        return Err("a rail's parameters are unknown".into());
                    };
                    let tip_v = match &face.surface {
                        Surface::Cone(c) if c.half_angle.sin().abs() > tol::TANGENT => -c.radius / c.half_angle.sin(),
                        s => s.project(w).y,
                    };
                    let turn = last.x - first.x;
                    if turn.abs() < 1.0 {
                        return Err("a rail that does not go round its face".into());
                    }
                    PCurve::Affine { origin: Vec2::new(first.x, tip_v), dir: Vec2::new(turn / TAU, 0.0) }
                } else if co.edge == ea || co.edge == eb {
                    let old = b.edge(co.edge);
                    let t = if co.reversed { old.range.1 } else { old.range.0 };
                    let began = match &co.pcurve {
                        PCurve::Projected { seed } => *seed,
                        other => other.uv_at(t).ok_or("a rail's parameters are unknown")?,
                    };
                    PCurve::Projected { seed: near_uv(&face.surface, w, Some(began)) }
                } else {
                    co.pcurve.clone()
                };
                specs.push(CoedgeSpec { edge, reversed, pcurve });
            }
            out.add_loop(id, specs);
        }
        kept.insert(f, id);
    }
    for sh in b.shell_ids() {
        let ids: Vec<FaceId> = b.shell(sh).faces.iter().filter_map(|f| kept.get(f).copied()).collect();
        let _ = out.add_shell(ids);
    }
    for so in b.solid_ids() {
        out.add_solid_of(b.solid(so).shells.clone(), b.solid(so).outer);
    }
    out.pair_radial();
    let issues = check::check(&out);
    if let Some(first) = issues.first() {
        return Err(format!("the solid with the round taken out is not valid ({first}, of {})", issues.len()));
    }
    Ok((out, kept))
}
