// yack: Changes to a solid's faces: faces moved, tilted or taken out, a solid hollowed, grown by a
// Adapted from springrep@a03e8a1 src/edit.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: convex_edges moved in from pick.rs (M2.5, T8); then to yack-edit (M3 W9b), paths and visibility only;
// on checked access, its indexing allow gone (hardening E1); an edge's faces read from its uses, not
// a scan of every coedge (hardening E2).
//! Changes to a solid's faces: faces moved, tilted or taken out, a solid hollowed, grown by a
//! distance or stretched, and the slab a face sweeps. Each builds exact tools and combines them
//! with the booleans, blends and face moves.


// springrep's foundation modules (D26), under the names the moved code uses.
use yack_blend as blend;
use yack_boolean as boolean;
use yack_build::build;
use yack_base::kernel as error;
use yack_geom as geom;
use yack_math::kernel as math;
use yack_measure as measure;
use yack_profile as profile;
use yack_build::sweep;
use yack_tol as tol;
use yack_topo as topo;
use yack_tweak as tweak;
#[cfg(test)]
use yack_check as check;
#[cfg(test)]
use yack_build::primitive;
use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::f64::consts::{PI, TAU};

use crate::{
    blend::shaped::plane_of,
    boolean::half_space,
    error::{Error, Result},
    geom::{Curve, Surface},
    math::{Aabb, Frame, Point3, Vec2, Vec3},
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep::{extrude, revolve},
    topo::{Brep, EdgeId, FaceId, FaceName},
};

mod affine;
mod apex;
mod cone_cavity;
mod enclosed;
mod offset_in;
mod rounded_shell;
mod corner_chamfer;
mod convex_grow;
mod corner_round;
mod straight_chamfer;
mod straight_round;
mod thicken_sheet;
mod turned_chamfer;
mod variable_round;
pub mod glue;
pub mod requirement;

pub use affine::{Affine, affine};
pub use offset_in::concave_edges;
pub use rounded_shell::{shell_outward_rounded, shell_rounded};
pub use thicken_sheet::thicken_sheet;
pub use corner_chamfer::chamfer_edges;
pub use convex_grow::{grow_convex, round_all_edges};
pub use corner_round::{fillet_concave_corner, fillet_edges};
pub use straight_round::fillet_edge;
pub use turned_chamfer::chamfer_two;
pub use straight_chamfer::chamfer_two as chamfer_two_straight;
pub use variable_round::fillet_variable;
pub use crate::tweak::{move_faces, move_faces_by};

/// `b` with each of `faces` tilted by `angle` about the line where it meets the `neutral` plane, its
/// outward normal turned towards the plane's normal: positive angles narrow the part along it.
pub fn draft(b: &Brep, faces: &[FaceId], neutral: &Frame, angle: f64) -> Result<Brep> {
    merged(crate::tweak::draft(b, faces, neutral, angle)?)
}

/// `b` with each face of `by` moved that far along its outward normal, onto its offset surface.
pub fn offset_faces(b: &Brep, by: &BTreeMap<FaceId, f64>) -> Result<Brep> {
    merged(crate::tweak::offset_faces(b, by)?)
}

/// A move's solid, the faces round any that closed on the way merged; if that fails, the error the
/// direct move gave.
fn merged(m: crate::tweak::Moved) -> Result<Brep> {
    match m.merge {
        None => Ok(m.brep),
        Some(e) => boolean::unify(&m.brep).map_err(|_| e),
    }
}

/// The edges of `b` where its faces meet convexly, the solid's inside a wedge narrower than a half
/// turn. A face's material lies left of its coedge seen from outside, along `n × t`; the edge is
/// convex where the other face's outward normal leans away from that side.
pub fn convex_edges(b: &Brep) -> Result<Vec<EdgeId>> {
    Ok(offset_in::creased_edges(b, true))
}

fn named(segment: Segment, name: &str) -> NamedSegment {
    NamedSegment::new(segment, name)
}

/// Which side of a solid's faces a shell's wall lies on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wall {
    /// Inside the solid: its outside stays where it is.
    Inward,
    /// Outside the solid: its faces become the wall's inside.
    Outward,
}

/// `s` hollowed to a wall `thickness` thick, open through the faces of `open`. The cavity's faces
/// are `<feature>.inner`. An inward wall leaves every face of `s` in place with its names. An
/// outward wall's outside is new, each of its faces named `inner_f<k>` for the face `k` of `s` it
/// offsets, and each face of `s` is the wall's inside, keeping its names beside `inner`.
pub fn shell(s: &Brep, open: &[FaceId], thickness: f64, wall: Wall, feature: &str) -> Result<Brep> {
    if thickness.is_nan() || thickness <= tol::LINEAR {
        return Err(Error::InvalidInput(
            "a shell's wall must be thicker than the tolerance".into(),
        ));
    }
    let inner = FaceName::new(feature, FaceName::INNER);
    // Every face moved `by` along its outward normal, the open ones `open_by`, with the face of `s`
    // each face of the result offsets (where the move kept, or took out rounds from, the topology).
    let moved = |by: f64, open_by: f64| -> Result<(Brep, BTreeMap<FaceId, FaceId>)> {
        let by: BTreeMap<FaceId, f64> = s
            .face_ids()
            .map(|f| (f, if open.contains(&f) { open_by } else { by }))
            .filter(|&(_, d)| d != 0.0)
            .collect();
        // Rounds the move goes past (thinner than the wall) are taken out first, their neighbours
        // meeting where they rounded; a solid with none, or that cannot be taken so, moves as it is.
        if let Ok(Some((sharp, kept))) = tweak::drop_vanishing_rounds(s, &by) {
            let by_sharp: BTreeMap<FaceId, f64> = by.iter().filter_map(|(f, &d)| Some((*kept.get(f)?, d))).collect();
            if let Ok(moved) = offset_faces(&sharp, &by_sharp) {
                let mut from = BTreeMap::new();
                if same_topology(&sharp, &moved) {
                    from.extend(kept.iter().map(|(&old, &new)| (new, old)));
                } else {
                    from = offsets_of(s, &moved, &by);
                }
                return Ok((moved, from));
            }
        }
        let moved = offset_faces(s, &by)?;
        let from = if same_topology(s, &moved) { s.face_ids().map(|f| (f, f)).collect() } else { offsets_of(s, &moved, &by) };
        Ok((moved, from))
    };
    match wall {
        Wall::Inward => {
            // The cavity runs out through the open faces, so the cut leaves no skin there.
            // An open face that meets a concave edge stays where it is: moved out, its plane would cut
            // the wall that rises past the edge back from the opening.
            let open_by = if topo::local::open_at_concave_edge(s, open) { 0.0 } else { thickness };
            // Of a frustum whose cavity closes below its far face, the far face does not bound it.
            let (mut cavity, from) = match cone_cavity::closed_cavity(s, open, thickness)? {
                Some(whole) => whole,
                None => match closed_past_rounds(s, open, thickness) {
                    Some(whole) => whole,
                    None => moved(-thickness, open_by)?,
                },
            };
            // Each cavity face also names the face of `s` it offsets, where the move kept the solid's
            // topology (or took rounds out of it) and so knows which.
            for f in cavity.face_ids().collect::<Vec<_>>() {
                let mut names: BTreeSet<FaceName> = std::iter::once(inner.clone()).collect();
                if let Some(src) = from.get(&f) {
                    names.insert(FaceName::new(feature, format!("inner_f{}", src.index())));
                }
                cavity.set_face_names(f, names);
            }
            boolean::difference(s, &cavity)
        }
        Wall::Outward => {
            let (mut outer, from) = moved(thickness, 0.0)?;
            for f in outer.face_ids().collect::<Vec<_>>() {
                if !open.contains(&f) {
                    // The outside is new: each of its faces offsets a face of `s`.
                    let names = from.get(&f).map(|src| FaceName::new(feature, format!("inner_f{}", src.index()))).into_iter().collect();
                    outer.set_face_names(f, names);
                }
            }
            // The cavity is the solid itself, its open faces reaching out by the thickness twice over,
            // or not at all where an open face meets a concave edge (as for the inward wall).
            let open_by = if topo::local::open_at_concave_edge(s, open) { 0.0 } else { 2.0 * thickness };
            let (mut cavity, _) = moved(0.0, open_by)?;
            for f in cavity.face_ids().collect::<Vec<_>>() {
                let mut names = cavity.face(f).names.clone();
                names.insert(inner.clone());
                cavity.set_face_names(f, names);
            }
            boolean::difference(&outer, &cavity)
        }
    }
}

/// The points within `distance` of `s`: every face moved out that far, a hole that closes on the
/// way followed there, each convex edge rounded at `distance` (which leaves a ball's patch at each
/// convex corner), and over each end of a hole that closed, the dimple its rim leaves.
pub fn offset(s: &Brep, distance: f64, feature: &str) -> Result<Brep> {
    if distance == 0.0 {
        return Ok(s.clone());
    }
    if distance < 0.0 {
        return offset_in::offset_inward(s, -distance, feature);
    }
    let by: BTreeMap<FaceId, f64> = s.face_ids().map(|f| (f, distance)).collect();
    let moved = offset_in::offset_faces_past_rounds(s, &by)?;
    let edges = convex_edges(&moved)?;
    let kept = same_topology(s, &moved);
    let mut out = if edges.is_empty() {
        moved
    } else {
        blend::fillet_edges_mitred(&moved, &edges, distance, feature)?
    };
    // A cone's apex is a convex point that no edge meets, rounded by a ball as a corner is.
    out = apex::round_apexes(&out, distance, feature)?;
    // Each round also names the edge of `s` it rounds (`round_<k>` is the k-th of `edges`, whose
    // ids are `s`'s where the move kept its topology), and each corner patch the vertex of `s` at
    // its ball's centre.
    for f in out.face_ids().collect::<Vec<_>>() {
        let mut names = out.face(f).names.clone();
        let mut extra = Vec::new();
        for n in names.iter().filter(|n| n.feature == feature) {
            if let Some(e) = n.face.strip_prefix("round_").and_then(|k| k.parse::<usize>().ok()).and_then(|k| edges.get(k)) {
                if kept {
                    extra.push(FaceName::new(feature, format!("round_e{}", e.index())));
                }
            } else if n.face == "corner"
                && let Surface::Sphere(ball) = &out.face(f).surface
                && let Some(v) = s.vertex_ids().find(|&v| s.vertex(v).point.dist(ball.frame.origin) <= tol::LINEAR)
            {
                extra.push(FaceName::new(feature, format!("corner_v{}", v.index())));
            }
        }
        if !extra.is_empty() {
            names.extend(extra);
            out.set_face_names(f, names);
        }
    }
    for f in s.face_ids() {
        let face = s.face(f);
        let Surface::Cylinder(c) = &face.surface else {
            continue;
        };
        if !face.reversed || c.radius >= distance {
            continue;
        }
        for &lp in &face.loops {
            for &co in &s.lp(lp).coedges {
                let e = s.coedge(co).edge;
                let Some(Curve::Circle(rim)) = &s.edge(e).curve else {
                    continue;
                };
                let Some(g) = s.edge_uses(e).iter().map(|&o| s.coedge_face(o)).find(|&g| g != f) else {
                    continue;
                };
                let (_, up) = plane_of(s, g)?;
                out = boolean::difference(
                    &out,
                    &dimple(rim.frame.origin, up, c.radius, distance, feature)?,
                )?;
            }
        }
    }
    Ok(out)
}

/// The cavity of a frustum whose rounds the wall closes away and whose cavity then closes below its
/// far face: the rounds are taken out, the sharp frustum's cavity found as for a plain one, and its
/// faces mapped back to the faces of `s` they offset. `None` where any step does not apply.
fn closed_past_rounds(s: &Brep, open: &[FaceId], t: f64) -> Option<(Brep, BTreeMap<FaceId, FaceId>)> {
    let by: BTreeMap<FaceId, f64> = s.face_ids().map(|f| (f, if open.contains(&f) { t } else { -t })).collect();
    let (sharp, kept) = tweak::drop_vanishing_rounds(s, &by).ok().flatten()?;
    let open_sharp: Vec<FaceId> = open.iter().filter_map(|f| kept.get(f).copied()).collect();
    let (cavity, from) = cone_cavity::closed_cavity(&sharp, &open_sharp, t).ok().flatten()?;
    let back: BTreeMap<FaceId, FaceId> = kept.iter().map(|(&old, &new)| (new, old)).collect();
    Some((cavity, from.into_iter().filter_map(|(c, f)| Some((c, *back.get(&f)?))).collect()))
}

/// The face of `s` each face of `moved` offsets, found where the move changed the topology (a round
/// closed): a face of `moved` is the offset of the face of `s` on a surface of its kind, with its
/// outward normal, whose move by `by` puts it where the face lies.
fn offsets_of(s: &Brep, moved: &Brep, by: &BTreeMap<FaceId, f64>) -> BTreeMap<FaceId, FaceId> {
    let mut found = BTreeMap::new();
    for f in moved.face_ids() {
        let dom = crate::topo::face_domain(moved, f);
        let Some(uv) = dom.interior_point() else { continue };
        let p = dom.surface.eval(uv.x, uv.y);
        let Some(n) = topo::local::face_normal_at(moved, f, p) else { continue };
        let source = by.iter().find(|&(&g, &d)| {
            let face = s.face(g);
            if std::mem::discriminant(&face.surface) != std::mem::discriminant(&moved.face(f).surface) {
                return false;
            }
            let q = face.surface.eval(face.surface.project(p).x, face.surface.project(p).y);
            topo::local::face_normal_at(s, g, q).is_some_and(|m| m.dot(n) > 1.0 - 1e-6 && (p - (q + m * d)).norm() <= 10.0 * tol::LINEAR) // tol: a cosine, and a distance
        });
        if let Some((&g, _)) = source {
            found.insert(f, g);
        }
    }
    found
}

/// Whether `b` has `a`'s faces, edges and vertices by count: a move that kept the solid's
/// topology, and with it the ids.
fn same_topology(a: &Brep, b: &Brep) -> bool {
    a.face_ids().count() == b.face_ids().count()
        && a.edge_ids().count() == b.edge_ids().count()
        && a.vertex_ids().count() == b.vertex_ids().count()
}

/// What the offset by `d` leaves open over a hole's rim of radius `r` about `o`, opening along
/// `up`: the hole's cylinder beyond the rim, farther than `d` from it. In the meridian (ρ out from
/// the axis, z up from the rim's plane), the region above the arc of radius `d` about the rim.
fn dimple(o: Point3, up: Vec3, r: f64, d: f64, feature: &str) -> Result<Brep> {
    let q = (d * d - r * r).sqrt();
    let from = q.atan2(-r);
    let line = |a: [f64; 2], b: [f64; 2]| Segment::Line {
        a: Vec2::new(a[0], a[1]),
        b: Vec2::new(b[0], b[1]),
    };
    let meridian = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                named(
                    Segment::Arc {
                        center: Vec2::new(r, 0.0),
                        radius: d,
                        start: from,
                        sweep: PI / 2.0 - from,
                    },
                    "ring",
                ),
                named(line([r, d], [r, d + 1.0]), "wall"),
                named(line([r, d + 1.0], [0.0, d + 1.0]), "top"),
                named(line([0.0, d + 1.0], [0.0, q]), "axis"),
            ],
        }],
    };
    let f = Frame::from_axis(o, up)?;
    revolve(&meridian, &Frame::new(o, -f.y, f.x)?, o, f.z, TAU, feature)
}

/// The solid `faces` sweep moving from 0 to `thickness` along their outward normals, inward where
/// it is negative: `s` with them moved, less `s`, or `s` less `s` with them moved in.
pub fn thicken(s: &Brep, faces: &[FaceId], thickness: f64) -> Result<Brep> {
    let by: BTreeMap<FaceId, f64> = faces.iter().map(|&f| (f, thickness)).collect();
    let moved = offset_faces(s, &by)?;
    if thickness > 0.0 {
        boolean::difference(&moved, s)
    } else {
        boolean::difference(s, &moved)
    }
}

/// `s` with `faces` taken out and the gap they leave closed by their neighbours: the box round
/// them, square to the neighbouring planes or a cylinder's axis, cut to the inside of every face
/// across their edges, joined to the solid. The faces must be a recess, a hole, or a corner cut
/// away; a plane or a cylinder bulging outward must bound it.
pub fn delete_faces(s: &Brep, faces: &[FaceId]) -> Result<Brep> {
    // Each run of the faces that touch is a gap of its own: one block over several would also fill
    // what lies between them, a hole left standing between two deleted ones.
    let mut runs: Vec<Vec<FaceId>> = Vec::new();
    for &f in faces {
        let mut run = vec![f];
        let touching = |run: &[FaceId], other: &[FaceId]| {
            run.iter().any(|&a| {
                s.face(a).loops.iter().any(|&lp| {
                    s.lp(lp).coedges.iter().any(|&c| s.edge_uses(s.coedge(c).edge).iter().any(|&u| other.contains(&s.coedge_face(u))))
                })
            })
        };
        let mut k = 0;
        while let Some(other) = runs.get(k) {
            if touching(&run, other) {
                run.extend(runs.remove(k));
            } else {
                k += 1;
            }
        }
        runs.push(run);
    }
    let mut out = s.clone();
    for run in &runs {
        out = match gap_fill(s, run)? {
            Gap::Fill(block) => boolean::union(&out, &block)?,
            Gap::Cut(block) => boolean::difference(&out, &block)?,
        };
    }
    // The healed faces and the neighbours they were healed from lie on one surface: one face each.
    let snapped = snap_same_surfaces(&out).unwrap_or_else(|| out.clone());
    Ok(boolean::unify(&snapped).or_else(|_| boolean::unify(&out)).unwrap_or(out))
}

/// `b` with each curved face that lies on the surface of an earlier one put on that very surface
/// (a healed cone and the cone it heals are one surface in two frames, and faces merge only on
/// equal surfaces), its pcurves laid anew; `None` where none does or the move cannot be made. A
/// face's orientation follows its outward normal onto the surface it takes. A face with a pole keeps
/// its own.
fn snap_same_surfaces(b: &Brep) -> Option<Brep> {
    let ids: Vec<FaceId> = b.face_ids().collect();
    let curved = |f: FaceId| {
        matches!(b.face(f).surface, Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_))
            && b.face(f).loops.iter().all(|&lp| b.lp(lp).coedges.iter().all(|&c| b.edge(b.coedge(c).edge).curve.is_some()))
    };
    let mut onto: BTreeMap<FaceId, Surface> = BTreeMap::new();
    let mut reversed: BTreeMap<FaceId, bool> = BTreeMap::new();
    for (k, &fb) in ids.iter().enumerate() {
        if !curved(fb) {
            continue;
        }
        let dom = crate::topo::face_domain(b, fb);
        let samples: Vec<Point3> = dom.interior_points().into_iter().take(3).map(|uv| dom.surface.eval(uv.x, uv.y)).collect();
        let Some(&first) = samples.first() else { continue };
        let Some(outward) = topo::local::face_normal_at(b, fb, first) else { continue };
        for &fa in ids.iter().take(k) {
            let sa = onto.get(&fa).cloned().unwrap_or_else(|| b.face(fa).surface.clone());
            let sb = &b.face(fb).surface;
            if &sa == sb || std::mem::discriminant(&sa) != std::mem::discriminant(sb) || !curved(fa) {
                continue;
            }
            if samples.iter().all(|&p| (sa.eval(sa.project(p).x, sa.project(p).y) - p).norm() <= tol::LINEAR) {
                let uv = sa.project(first);
                reversed.insert(fb, sa.normal(uv.x, uv.y).dot(outward) <= 0.0);
                onto.insert(fb, sa);
                break;
            }
        }
    }
    if onto.is_empty() {
        return None;
    }
    resurfaced(b, &onto, &reversed)
}

/// `b` with the faces of `onto` on their new surfaces (the same surface in another frame, so every
/// edge stays where it is) and orientations from `reversed`, their pcurves laid anew from the edges.
fn resurfaced(b: &Brep, onto: &BTreeMap<FaceId, Surface>, reversed: &BTreeMap<FaceId, bool>) -> Option<Brep> {
    let mut out = Brep::new();
    let verts: Vec<_> = b.vertex_ids().map(|v| out.add_vertex(b.vertex(v).point)).collect();
    let mut edges = Vec::new();
    for e in b.edge_ids() {
        let x = b.edge(e);
        let (s, t) = (*verts.get(x.start.index())?, *verts.get(x.end.index())?);
        edges.push(match &x.curve {
            Some(c) => out.add_edge(c.clone(), x.range, s, t),
            None => out.add_degenerate_edge(s, x.range),
        });
    }
    // The parameters nearest `last` (a periodic one continuing from it) where `p` lies on `surface`.
    let near = |surface: &Surface, p: Point3, last: Option<Vec2>| {
        let uv = surface.project(p);
        match last {
            Some(l) => Vec2::new(uv.x + TAU * ((l.x - uv.x) / TAU).round(), uv.y),
            None => uv,
        }
    };
    let mut faces = Vec::new();
    for f in b.face_ids() {
        let face = b.face(f);
        let moved = onto.get(&f);
        let surface = moved.cloned().unwrap_or_else(|| face.surface.clone());
        let nf = out.add_face(surface.clone(), reversed.get(&f).copied().unwrap_or(face.reversed), face.names.clone());
        for &lp in &face.loops {
            let mut last: Option<Vec2> = None;
            let mut specs = Vec::new();
            for &c in &b.lp(lp).coedges {
                let co = b.coedge(c);
                let edge = *edges.get(co.edge.index())?;
                let x = out.edge(edge);
                let pcurve = match (&x.curve, moved) {
                    (Some(cv), Some(_)) => {
                        let (t0, t1) = if co.reversed { (x.range.1, x.range.0) } else { x.range };
                        let from = if co.reversed { x.end } else { x.start };
                        let start = near(&surface, out.vertex(from).point, last);
                        let seed = crate::topo::pcurve::project_on(&surface, cv, t0, t1, last.unwrap_or(start));
                        let mut uv = seed;
                        for k in 1..=32 {
                            uv = near(&surface, cv.eval(t0 + (t1 - t0) * f64::from(k) / 32.0), Some(uv));
                        }
                        last = Some(uv);
                        topo::PCurve::Projected { seed }
                    }
                    _ => co.pcurve.clone(),
                };
                specs.push(topo::CoedgeSpec { edge, reversed: co.reversed, pcurve });
            }
            out.add_loop(nf, specs);
        }
        faces.push(nf);
    }
    for sh in b.shell_ids() {
        let ids: Vec<FaceId> = b.shell(sh).faces.iter().filter_map(|f| faces.get(f.index()).copied()).collect();
        let _ = out.add_shell(ids);
    }
    for so in b.solid_ids() {
        out.add_solid_of(b.solid(so).shells.clone(), b.solid(so).outer);
    }
    out.pair_radial();
    Some(out)
}

/// What closes a gap: a block to add (a recess, a hole, a corner cut away) or one to take away (a
/// boss, a round filling an inside corner).
enum Gap {
    Fill(Brep),
    Cut(Brep),
}

/// Whether `q` lies on the material side of face `g`; `None` for a face of another kind.
fn behind(s: &Brep, g: FaceId, q: Point3) -> Option<bool> {
    match &s.face(g).surface {
        Surface::Plane(_) => {
            let (o, n) = plane_of(s, g).ok()?;
            Some((q - o).dot(n) <= 0.0)
        }
        other => enclosed::within(other, q).map(|inside| inside != enclosed::hollow(s, g)),
    }
}

/// The block that closes the gap `faces` leave, as [`delete_faces`] describes.
fn gap_fill(s: &Brep, faces: &[FaceId]) -> Result<Gap> {
    let mut near: Vec<FaceId> = Vec::new();
    let mut rims: Vec<EdgeId> = Vec::new();
    for &f in faces {
        for &lp in &s.face(f).loops {
            for &c in &s.lp(lp).coedges {
                let e = s.coedge(c).edge;
                rims.push(e);
                for g in s.edge_uses(e).iter().map(|&o| s.coedge_face(o)) {
                    if !faces.contains(&g) && !near.contains(&g) {
                        near.push(g);
                    }
                }
            }
        }
    }
    let axes: Vec<Vec3> = near
        .iter()
        .chain(faces)
        .filter_map(|&f| match &s.face(f).surface {
            Surface::Plane(pl) => Some(pl.frame.z),
            Surface::Cylinder(c) => Some(c.frame.z),
            _ => None,
        })
        .collect();
    let z = axes.first().copied().unwrap_or(Vec3::Z);
    let x = axes
        .iter()
        .copied()
        .find(|a| a.cross(z).norm() > 1e-6) // tol: axes this close to parallel are parallel
        .or_else(|| tightest_across(s, near.iter().chain(faces), z, &rims))
        .unwrap_or_else(|| z.any_perpendicular());
    let axes = Frame::new(Point3::ZERO, z, x)?;
    let mut bounds = Aabb::EMPTY;
    for &e in &rims {
        edge_bounds(s, e, &axes, &mut bounds);
    }
    // A ball bulges past the rims it meets: the block grows across the axis by its radius, the ball
    // then bounding what it holds.
    let bulge = near.iter().filter_map(|&f| match &s.face(f).surface {
        Surface::Sphere(sp) if !enclosed::hollow(s, f) => Some(sp.radius),
        // A cone bounds what lies inside it, its corner with a plane further out than the rims.
        Surface::Cone(_) if !enclosed::hollow(s, f) => Some(measure::bbox(s).diagonal()),
        _ => None,
    }).fold(0.0, f64::max);
    // A box with no thickness along an axis (a flat's rims all on its plane) grows both ways by the
    // widest ball or post round it: they bound what it holds.
    let round = near.iter().filter_map(|&f| match &s.face(f).surface {
        Surface::Sphere(sp) if !enclosed::hollow(s, f) => Some(sp.radius),
        Surface::Cylinder(c) if !enclosed::hollow(s, f) => Some(c.radius),
        Surface::Cone(_) if !enclosed::hollow(s, f) => Some(measure::bbox(s).diagonal()),
        _ => None,
    }).fold(0.0, f64::max);
    let thin = |extent: f64| if extent <= tol::LINEAR { round } else { 0.0 };
    let grow = Vec3::new(bulge + thin(bounds.max.x - bounds.min.x), bulge + thin(bounds.max.y - bounds.min.y), thin(bounds.max.z - bounds.min.z));
    let (lo, hi) = (bounds.min - grow, bounds.max + grow);
    let size = hi - lo;
    let frame = Frame::new(axes.to_world(lo), axes.z, axes.x)?;
    let rect = Profile::rect(Vec2::new(size.x / 2.0, size.y / 2.0), size.x, size.y);
    let mut fill = match turned_block(s, faces, &rims, bulge)? {
        Some(round) => round,
        None => extrude(&rect, &frame, size.z, "fill")?,
    };
    // A point just off each deleted face, on the side the gap lies: behind every neighbour, the gap is
    // a recess to fill; in front of some, a bump, closed by cutting it back to those. Faces that
    // disagree, or a neighbour of a kind not known here, leave the gap to the fill below.
    let samples: Vec<Vec<Option<bool>>> = faces
        .iter()
        .filter_map(|&f| {
            let dom = crate::topo::face_domain(s, f);
            let uv = dom.interior_point()?;
            let p = dom.surface.eval(uv.x, uv.y);
            let q = p + topo::local::face_normal_at(s, f, p)? * 1e-3; // tol: a step off the face, far above the tolerance, well inside the gap
            Some(near.iter().map(|&g| behind(s, g, q)).collect())
        })
        .collect();
    let known = samples.iter().flatten().all(Option::is_some);
    let in_front = |k: usize| samples.iter().all(|row| row.get(k) == Some(&Some(false)));
    let front: Vec<FaceId> = near.iter().enumerate().filter(|&(k, _)| in_front(k)).map(|(_, &g)| g).collect();
    let bump = samples.len() == faces.len() && samples.iter().all(|row| row.contains(&Some(false)));
    if known && bump && !front.is_empty() {
        let mut cut = fill;
        for f in front {
            let outside = match &s.face(f).surface {
                Surface::Plane(_) => {
                    let (o, n) = plane_of(s, f)?;
                    half_space(s, o, n)?
                }
                other => {
                    let body = enclosed::body(s, other)?;
                    // The void side of a post is outside it, of a bore inside it.
                    if enclosed::hollow(s, f) { body } else { boolean::difference(&cut, &body)? }
                }
            };
            cut = boolean::intersection(&cut, &outside)?;
        }
        return Ok(Gap::Cut(cut));
    }
    for f in near {
        let inside = match &s.face(f).surface {
            Surface::Plane(_) => {
                let (o, n) = plane_of(s, f)?;
                half_space(s, o, -n)?
            }
            other => {
                let body = enclosed::body(s, other)?;
                if enclosed::hollow(s, f) { boolean::difference(&fill, &body)? } else { body }
            }
        };
        fill = boolean::intersection(&fill, &inside)?;
    }
    Ok(Gap::Fill(fill))
}

/// The block round a gap that is turned about an axis: the rims all circles about one line (and
/// the seams of the deleted faces), the cylinder of the widest of them, `bulge` more where a ball
/// reaches out past them, from the first to the last of them along the axis. The box about such rims
/// has corners that stand out of the part. `None` for any other gap.
fn turned_block(s: &Brep, faces: &[FaceId], rims: &[EdgeId], bulge: f64) -> Result<Option<Brep>> {
    let mut axis: Option<(Point3, Vec3)> = None;
    let (mut lo, mut hi, mut wide) = (f64::INFINITY, f64::NEG_INFINITY, 0.0_f64);
    for &e in rims {
        let edge = s.edge(e);
        // A seam: an edge that one deleted face runs along both ways, whatever its curve.
        let uses = s.edge_uses(e);
        if uses.first().map(|&u| s.coedge_face(u)).is_some_and(|f| faces.contains(&f) && uses.iter().all(|&u| s.coedge_face(u) == f)) {
            continue;
        }
        match &edge.curve {
            None => continue,
            Some(Curve::Circle(c)) => {
                let (o, z) = *axis.get_or_insert((c.frame.origin, c.frame.z));
                let off = c.frame.origin - o;
                let about_one_line = c.frame.z.cross(z).norm() <= 1e-6 && (off - z * off.dot(z)).norm() <= 1e-6; // tol: parallel axes, centres on the first one's
                if !about_one_line {
                    return Ok(None);
                }
                let along = off.dot(z);
                lo = lo.min(along);
                hi = hi.max(along);
                wide = wide.max(c.radius);
            }
            Some(_) => return Ok(None),
        }
    }
    let Some((o, z)) = axis else { return Ok(None) };
    if hi - lo <= tol::LINEAR {
        return Ok(None);
    }
    let fr = Frame::new(o + z * lo, z, z.any_perpendicular())?;
    extrude(&Profile::circle(Vec2::ZERO, 2.0 * (wide + bulge)), &fr, hi - lo, "fill").map(Some)
}

/// Of the directions across `z` that straight edges of `faces` run in, the one whose box about
/// `rims` (square to `z`) holds the least: the solid's own directions, not an arbitrary pair, so a
/// fill block follows the part however it lies.
fn tightest_across<'a>(s: &Brep, faces: impl Iterator<Item = &'a FaceId>, z: Vec3, rims: &[EdgeId]) -> Option<Vec3> {
    let mut best: Option<(f64, Vec3)> = None;
    for &f in faces {
        for &lp in &s.face(f).loops {
            for &c in &s.lp(lp).coedges {
                let Some(Curve::Line(line)) = &s.edge(s.coedge(c).edge).curve else { continue };
                let across = line.dir - z * line.dir.dot(z);
                let Some(x) = across.unit().filter(|_| across.norm() > 1e-6) else { continue }; // tol: a direction this close to z is z
                let Ok(frame) = Frame::new(Point3::ZERO, z, x) else { continue };
                let mut bounds = Aabb::EMPTY;
                for &e in rims {
                    edge_bounds(s, e, &frame, &mut bounds);
                }
                let size = bounds.max - bounds.min;
                let volume = size.x * size.y;
                if best.is_none_or(|(least, _)| volume < least * (1.0 - 1e-9)) { // tol: equal boxes keep the first
                    best = Some((volume, x));
                }
            }
        }
    }
    best.map(|(_, x)| x)
}

/// Grows `b`, in `axes`' coordinates, to hold edge `e`: its ends, and each place between them
/// where its coordinate along an axis turns back, from the curve itself. Samples along the edge
/// stop short of such a place by the sag of a chord, which a fill block sized by them then lacks.
fn edge_bounds(s: &Brep, e: EdgeId, axes: &Frame, b: &mut Aabb) {
    let edge = s.edge(e);
    let Some(c) = &edge.curve else {
        b.add(axes.to_local(s.vertex(edge.start).point));
        return;
    };
    let (t0, t1) = (
        edge.range.0.min(edge.range.1),
        edge.range.0.max(edge.range.1),
    );
    let mut ts = vec![t0, t1];
    for axis in [axes.x, axes.y, axes.z] {
        ts.extend(c.extremes(axis, t0, t1));
    }
    for t in ts {
        b.add(axes.to_local(c.eval(t)));
    }
}

#[cfg(test)]
mod tests;
