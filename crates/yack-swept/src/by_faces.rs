// yack: the swept solid as the start united with what each of its leading faces sweeps (M4.1 step 1).
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/sweep_faces.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: rewritten onto yack's crates, not ported: each leading face's sweep is a solid built from
//   its parts (yack-build's `solid_of`), united with the start by yack-boolean, and checked; so far
//   for a slide of a solid whose faces each lead, trail or run along the motion whole (no face is
//   split along its contact yet). Everything else declines.
//! What any solid passes through as it slides, turns or screws: the solid where it starts, with
//! what each of its *leading* faces sweeps.
//!
//! A point the moving solid covers at some instant is inside it where it starts, or enters it
//! later across the part of its boundary moving towards it: where the face's outward normal leads
//! the motion. A face that leads all over sweeps a solid without folding: the face where it
//! starts, the face where it ends, and the sides its edges sweep. The solid swept is the start
//! united with those solids.
//!
//! Built so far: a slide, where every face leads, trails or runs along the slide all over (planes,
//! and curved faces the slide runs along, a cylinder's side slid along its axis). A curved face
//! the slide crosses has to be split along its contact first, which this does not do yet: it
//! declines.

use yack_base::kernel::{Error, Result};
use yack_build::build::{PartFace, Parts, param_of, parts_of, solid_of};
use yack_geom::{Curve, Extrusion, Plane, Surface};
use yack_math::kernel::{Frame, Point3, Transform, Vec3};
use yack_topo::Brep;

use crate::Motion;

/// How a face stands to a slide.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stand {
    Leads,
    Trails,
    Along,
}

/// [`crate::swept`] by faces: so far a slide only.
pub(crate) fn swept(body: &Brep, motion: &Motion) -> Result<Brep> {
    match motion {
        Motion::Slide { by } => slid(body, *by),
        Motion::Turn { .. } | Motion::Screw { .. } => {
            Err(Error::Unsupported("by faces: turns and screws are not built yet".into()))
        }
    }
}

/// Everything `body` passes through as it slides by `by`.
fn slid(body: &Brep, by: Vec3) -> Result<Brep> {
    let length = by.norm();
    if length <= yack_tol::LINEAR {
        return Err(Error::InvalidInput(format!("a slide of {length:e} mm moves nothing")));
    }
    let dir = by * (1.0 / length);
    let parts = parts_of(body)?;
    let mut pieces = Vec::new();
    for (k, face) in parts.faces.iter().enumerate() {
        match stand(face, dir)? {
            Stand::Leads => pieces.push(prism(&parts, k, by, dir)?),
            Stand::Trails | Stand::Along => {}
        }
    }
    let mut all: Vec<&Brep> = vec![body];
    all.extend(pieces.iter());
    yack_boolean::union_all(&all)
}

/// How `face` stands to a slide along the unit `dir`: whole, or a decline where it would have to
/// be split along its contact first.
fn stand(face: &PartFace, dir: Vec3) -> Result<Stand> {
    let out = |n: Vec3| if face.reversed { -n } else { n };
    let along = |axis: Vec3| axis.cross(dir).norm() <= yack_tol::ANGULAR;
    match &face.surface {
        Surface::Plane(p) => {
            let s = out(p.frame.z).dot(dir);
            Ok(if s > yack_tol::ANGULAR {
                Stand::Leads
            } else if s < -yack_tol::ANGULAR {
                Stand::Trails
            } else {
                Stand::Along
            })
        }
        Surface::Cylinder(c) if along(c.frame.z) => Ok(Stand::Along),
        Surface::Extrusion(e) if along(e.dir) => Ok(Stand::Along),
        other => Err(Error::Unsupported(format!(
            "by faces: a {} face the slide crosses is not split along its contact yet",
            other.kind()
        ))),
    }
}

/// The solid face `k` of `parts` sweeps as it slides by `by` (`dir` its unit direction), the face
/// leading all over: the face where it starts (facing back), where it ends, and the side each of
/// its edges sweeps.
fn prism(parts: &Parts, k: usize, by: Vec3, dir: Vec3) -> Result<Brep> {
    let face = parts.faces.get(k).ok_or_else(|| Error::InvalidInput("no such face".into()))?;
    let moved = Transform::translation(by);
    let mut out = Parts::default();
    // Each edge the face uses, its ends and their slid copies, once each.
    let mut point_at: std::collections::BTreeMap<usize, (usize, usize)> =
        std::collections::BTreeMap::new();
    let mut edge_at: std::collections::BTreeMap<usize, (usize, usize)> =
        std::collections::BTreeMap::new();
    let mut rail_at: std::collections::BTreeMap<usize, usize> = std::collections::BTreeMap::new();
    let mut point = |out: &mut Parts, v: usize| -> Result<(usize, usize)> {
        if let Some(&p) = point_at.get(&v) {
            return Ok(p);
        }
        let p = *parts.points.get(v).ok_or_else(|| Error::InvalidInput("no such point".into()))?;
        let pair = (out.points.len(), out.points.len() + 1);
        out.points.push(p);
        out.points.push(moved.point(p));
        point_at.insert(v, pair);
        Ok(pair)
    };
    let mut start_loops = Vec::new();
    let mut end_loops = Vec::new();
    let mut sides = Vec::new();
    for lp in &face.loops {
        let (mut start_loop, mut end_loop) = (Vec::new(), Vec::new());
        for &(e, reversed) in lp {
            let (curve, a, b) =
                parts.edges.get(e).ok_or_else(|| Error::InvalidInput("no such edge".into()))?;
            let ((a0, a1), (b0, b1)) = (point(&mut out, *a)?, point(&mut out, *b)?);
            let (e0, e1) = match edge_at.get(&e) {
                Some(&pair) => pair,
                None => {
                    let pair = (out.edges.len(), out.edges.len() + 1);
                    out.edges.push((curve.clone(), a0, b0));
                    out.edges.push((curve.transformed(&moved), a1, b1));
                    edge_at.insert(e, pair);
                    pair
                }
            };
            for (v, (v0, v1)) in [(*a, (a0, a1)), (*b, (b0, b1))] {
                if let std::collections::btree_map::Entry::Vacant(slot) = rail_at.entry(v) {
                    let p = out.points.get(v0).copied().unwrap_or(Point3::ZERO);
                    let rail = Curve::Line(yack_geom::Line { origin: p, dir });
                    slot.insert(out.edges.len());
                    out.edges.push((rail, v0, v1));
                }
            }
            start_loop.push((e0, !reversed));
            end_loop.push((e1, reversed));
            sides.push((e, reversed, e0, e1, *a, *b));
        }
        // The start cap runs the other way round (it faces back), so its uses go in reverse.
        start_loop.reverse();
        start_loops.push(start_loop);
        end_loops.push(end_loop);
    }
    let names = face.names.clone();
    out.faces.push(PartFace {
        surface: face.surface.clone(),
        reversed: !face.reversed,
        loops: start_loops,
        names: names.clone(),
        shell: 0,
    });
    out.faces.push(PartFace {
        surface: face.surface.transformed(&moved),
        reversed: face.reversed,
        loops: end_loops,
        names: names.clone(),
        shell: 0,
    });
    for (e, reversed, e0, e1, a, b) in sides {
        let (curve, _, _) =
            parts.edges.get(e).ok_or_else(|| Error::InvalidInput("no such edge".into()))?;
        let (ra, rb) = (
            *rail_at.get(&a).ok_or_else(|| Error::InvalidInput("no rail".into()))?,
            *rail_at.get(&b).ok_or_else(|| Error::InvalidInput("no rail".into()))?,
        );
        // Along the loop: the edge (as the face uses it), the rail up from its end, the slid edge
        // back, the rail down to its start; the face's material lies on the loop's left.
        let (first, last) = if reversed { (rb, ra) } else { (ra, rb) };
        let loop_ = vec![(e0, reversed), (last, false), (e1, !reversed), (first, true)];
        let (surface, flip) = side_surface(
            curve,
            parts.points.get(if reversed { b } else { a }).copied(),
            reversed,
            face,
            dir,
        )?;
        out.faces.push(PartFace {
            surface,
            reversed: flip,
            loops: vec![loop_],
            names: names.clone(),
            shell: 0,
        });
    }
    solid_of(&out)
}

/// The surface an edge sweeps as it slides along the unit `dir`, and whether the side faces
/// against it: a plane for a straight edge, an extrusion otherwise. `at` is where the face's loop
/// leaves the edge, `reversed` whether the loop runs against the edge's curve, `face` the face it
/// bounds (its material on the loop's left, seen from outside).
fn side_surface(
    curve: &Curve,
    at: Option<Point3>,
    reversed: bool,
    face: &PartFace,
    dir: Vec3,
) -> Result<(Surface, bool)> {
    let at = at.ok_or_else(|| Error::InvalidInput("no such point".into()))?;
    let (_, d) = curve.d1(param_of(curve, at));
    let travel = if reversed { -d } else { d };
    // The face's outward normal where the edge leaves it, and the way into the face from the edge.
    let uv = face.surface.project(at);
    let n = face.surface.normal(uv.x, uv.y);
    let n = if face.reversed { -n } else { n };
    let inward = n.cross(travel);
    // The side's outward normal is square to the edge and the slide, away from the face's material.
    let side = travel.cross(dir);
    let out = if side.dot(inward) > 0.0 { -side } else { side };
    let Some(out) = out.unit() else {
        return Err(Error::Unsupported("by faces: an edge that runs along the slide".into()));
    };
    let surface = match curve {
        Curve::Line(l) => Surface::Plane(Plane { frame: Frame::new(l.origin, out, l.dir)? }),
        _ => Surface::Extrusion(Box::new(Extrusion { curve: curve.clone(), dir })),
    };
    let uv = surface.project(at);
    let natural = surface.normal(uv.x, uv.y);
    Ok((surface, natural.dot(out) < 0.0))
}
