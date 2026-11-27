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
mod thicken_sheet;
mod turned_chamfer;
mod variable_round;
pub mod glue;
pub mod requirement;

pub use affine::{Affine, affine};
pub use offset_in::concave_edges;
pub use rounded_shell::{shell_outward_rounded, shell_rounded};
pub use thicken_sheet::thicken_sheet;
pub use turned_chamfer::chamfer_two;
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
                None => moved(-thickness, open_by)?,
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
mod tests {
    use super::*;
    use crate::topo::FaceName;

    #[test]
    fn shell_cavities_and_offset_rounds_name_their_sources() {
        // A block's inward shell names each cavity face by the face it offsets; its offset names
        // each round by the edge it rounds (all 12) and each corner patch by its vertex (all 8).
        let block = crate::primitive::block(&Frame::WORLD, Vec3::new(4.0, 3.0, 2.0), "block").unwrap();
        let top = block.face_ids().find(|&f| crate::measure::face_bbox(&block, f).min.z > 1.9).unwrap();
        let b = shell(&block, &[top], 0.5, Wall::Inward, "skin").unwrap();
        let named = |b: &Brep, prefix: &str| {
            let mut ks: Vec<String> = b.face_ids().flat_map(|f| b.face(f).names.clone()).filter(|n| n.face.starts_with(prefix)).map(|n| n.face).collect();
            ks.sort();
            ks.dedup();
            ks
        };
        let inner = named(&b, "inner_f");
        assert!(inner.len() >= 5, "{inner:?}");
        let o = offset(&block, 0.5, "o").unwrap();
        let (rounds, corners) = (named(&o, "round_e"), named(&o, "corner_v"));
        assert_eq!((rounds.len(), corners.len()), (12, 8), "{rounds:?} {corners:?}");
        let edges: Vec<String> = block.edge_ids().map(|e| format!("round_e{}", e.index())).collect();
        assert!(rounds.iter().all(|r| edges.contains(r)), "{rounds:?}");
    }

    #[test]
    fn a_shells_rim_keeps_the_open_faces_name_both_ways() {
        // A 20 × 20 × 10 block open at the top, 2 thick, in and out: the ring of wall top round
        // the opening is what is left of the open face, and carries its names.
        let block = crate::primitive::block(&Frame::WORLD, Vec3::new(20.0, 20.0, 10.0), "block").unwrap();
        let top = block
            .face_ids()
            .find(|&f| crate::measure::face_bbox(&block, f).min.z > 9.9)
            .unwrap();
        let names = block.face(top).names.clone();
        for wall in [Wall::Inward, Wall::Outward] {
            let b = shell(&block, &[top], 2.0, wall, "skin").unwrap();
            let all: Vec<_> = b.face_ids().map(|f| b.face(f).names.clone()).collect();
            let rims = all.iter().filter(|n| n.is_superset(&names)).count();
            assert_eq!(rims, 1, "{wall:?}: {all:?}");
        }
    }

    #[test]
    fn an_arcs_reach_along_an_axis_is_its_own_extreme_not_its_nearest_sample() {
        use crate::profile::{NamedSegment, ProfileLoop, Segment};
        // A half disc of radius 10, extruded: its rim is a half circle. Along an axis that points
        // midway between two of 256 even samples of the rim, the rim reaches the full radius.
        let named = |segment: Segment, name: &str| NamedSegment::new(segment, name);
        let half = Profile {
            loops: vec![ProfileLoop {
                segments: vec![
                    named(
                        Segment::Arc {
                            center: Vec2::ZERO,
                            radius: 10.0,
                            start: 0.0,
                            sweep: std::f64::consts::PI,
                        },
                        "rim",
                    ),
                    named(
                        Segment::Line {
                            a: Vec2::new(-10.0, 0.0),
                            b: Vec2::new(10.0, 0.0),
                        },
                        "chord",
                    ),
                ],
            }],
        };
        let b = extrude(&half, &Frame::WORLD, 3.0, "h").unwrap();
        let rim = b
            .edge_ids()
            .find(|&e| {
                let edge = b.edge(e);
                matches!(edge.curve, Some(Curve::Circle(_)))
                    && b.vertex(edge.start).point.z.abs() < 1e-9
            })
            .unwrap();
        let towards = std::f64::consts::PI * 100.5 / 256.0;
        let x = Vec3::new(towards.cos(), towards.sin(), 0.0);
        let axes = Frame::new(Point3::ZERO, Vec3::Z, x).unwrap();
        let mut bounds = Aabb::EMPTY;
        edge_bounds(&b, rim, &axes, &mut bounds);
        assert!((bounds.max.x - 10.0).abs() < 1e-9, "{}", bounds.max.x);
    }

    /// A prism on the polygon `pts`, `h` tall, its sides named `e0`, `e1`, … from the first edge.
    fn prism(pts: &[[f64; 2]], h: f64) -> Brep {
        let pts: Vec<Vec2> = pts.iter().map(|p| Vec2::new(p[0], p[1])).collect();
        let names: Vec<String> = (0..pts.len()).map(|i| format!("e{i}")).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, h, "p").unwrap()
    }

    fn face(b: &Brep, name: &str) -> FaceId {
        let n = FaceName::new("p", name);
        b.face_ids()
            .find(|&f| b.face(f).names.contains(&n))
            .unwrap()
    }

    fn close(got: f64, want: f64) {
        assert!(
            (got - want).abs() < 1e-7 * want.max(1.0),
            "{got} against {want}"
        );
    }

    #[test]
    fn a_block_offset_is_the_block_rounded_by_the_distance() {
        // 4 × 3 × 2 offset by 0.5: V + S·d + π(a + b + c)·d² + (4/3)π·d³, Steiner's formula.
        let b = prism(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]], 2.0);
        let d = 0.5;
        let want = 24.0 + 52.0 * d + PI * 9.0 * d * d + 4.0 / 3.0 * PI * d.powi(3);
        close(measure::volume(&offset(&b, d, "o").unwrap()), want);
    }

    #[test]
    fn an_l_s_rounded_edges_swing_round_its_concave_edge() {
        // An L of arms 8 × 3, 5 tall, every convex edge rounded 1: the union of the balls of
        // radius 1 inside it. The two arm boxes rounded, less their overlap (the corner column
        // rounded, with (1 − π/4)·r² short of a square along the concave edge, and 4r³/3 of that
        // at its ends), and at each end of the concave edge what the ball keeps swinging round
        // it, (1 − π/4)·r³·(5/3 − π/2). A box rounded r all over: the inner box, its faces moved
        // out r, its edges' quarter cylinders and its corners' eighth balls.
        // Its area likewise, the swing's torus, (π/2)·r²(π/2 − 1), standing for two cylinder
        // pieces of r²(π/2 − 1), and the flat face keeping the corner's r × r square less its
        // quarter disk. The arms' widths, depths, heights and radii include draws of the
        // conformance suite's L-prisms the swing once failed on.
        let cases = [
            (8.0, 3.0, 8.0, 5.0, 1.0),
            (
                6.199379767061393,
                3.2446824245098926,
                5.944930609453334,
                2.787400445599377,
                0.983805266315506,
            ),
            (
                8.410642646708611,
                2.4606909192521025,
                9.679346455567444,
                7.230068216963447,
                0.2821417462255193,
            ),
            (
                9.564764789551234,
                3.3401708771362455,
                6.900089620242228,
                3.7297726032339753,
                0.7396202116839262,
            ),
            (
                7.342452085970791,
                2.923584026607261,
                6.1264770305629295,
                2.2167560977768446,
                0.8678845173029267,
            ),
            (
                5.52965122853034,
                2.429579012791682,
                9.404723647837528,
                5.938985170662157,
                0.6834351663091055,
            ),
            (
                6.3990454944784565,
                2.3841696916601958,
                8.181436493454777,
                4.839043994693185,
                0.7102451360532922,
            ),
            (
                6.538663752237994,
                3.1564303512067906,
                9.224051200212097,
                5.94676849583945,
                1.2821630460001838,
            ),
        ];
        for (w, t, d, h, r) in cases {
            let b = prism(&[[0.0, 0.0], [w, 0.0], [w, t], [t, t], [t, d], [0.0, d]], h);
            let out = blend::fillet_edges(&b, &convex_edges(&b).unwrap(), r, "fil").unwrap();
            assert!(
                crate::check::check(&out).is_empty(),
                "{:?}",
                crate::check::check(&out)
            );
            let rounded = |x: f64, y: f64, z: f64| {
                let (a, b, c) = (x - 2.0 * r, y - 2.0 * r, z - 2.0 * r);
                (
                    a * b * c
                        + 2.0 * r * (a * b + b * c + c * a)
                        + PI * r * r * (a + b + c)
                        + 4.0 / 3.0 * PI * r.powi(3),
                    2.0 * (a * b + b * c + c * a) + 2.0 * PI * r * (a + b + c) + 4.0 * PI * r * r,
                )
            };
            let k = 1.0 - PI / 4.0;
            let ((va, sa), (vb, sb), (vc, sc)) =
                (rounded(w, t, h), rounded(t, d, h), rounded(t, t, h));
            let core_h = h - 2.0 * r;
            let volume = va + vb - vc - k * (core_h * r * r + 4.0 / 3.0 * r.powi(3))
                + 2.0 * k * (5.0 / 3.0 - PI / 2.0) * r.powi(3);
            let area = sa + sb
                - sc
                - k * (2.0 * core_h * r + 4.0 * r * r)
                - 2.0 * ((2.0 - PI / 2.0) * (PI / 2.0 - 1.0) - k) * r * r;
            close(measure::volume(&out), volume);
            close(measure::area(&out), area);
            // Written and read back it is the same solid: springrep's moved_tests/step.rs reads
            // these Ls from fixtures (M3 W9b).
        }
    }

    #[test]
    fn an_l_prism_offset_turned_away_is_the_same_size() {
        let (w, t, h) = (9.536367478693311, 2.7154798898513985, 6.344052519590756);
        let b = prism(
            &[[0.0, 0.0], [w, 0.0], [w, t], [t, t], [t, h], [0.0, h]],
            5.222922150702244,
        );
        let d = 0.5756566176826041;
        let here = measure::volume(&offset(&b, d, "o").unwrap());
        let mut moved = b.clone();
        moved.transform(&crate::math::Transform {
            r: [
                [
                    -0.2124025946267396,
                    -0.9641070956655728,
                    -0.15931932049542274,
                ],
                [
                    0.9494953687642673,
                    -0.1650876859267214,
                    -0.26684190197674823,
                ],
                [0.2309625131726732, -0.20795086930006934, 0.9504802751589895],
            ],
            t: Vec3::new(-45.51695804202156, 41.045298753592036, -21.51869167292315),
        });
        close(measure::volume(&offset(&moved, d, "o").unwrap()), here);
    }

    #[test]
    fn a_notch_s_faces_taken_out_leave_the_block_whole() {
        let b = prism(
            &[
                [0.0, 0.0],
                [6.0, 0.0],
                [6.0, 4.0],
                [4.0, 4.0],
                [4.0, 3.0],
                [2.0, 3.0],
                [2.0, 4.0],
                [0.0, 4.0],
            ],
            1.0,
        );
        let notch = [face(&b, "e3"), face(&b, "e4"), face(&b, "e5")];
        close(measure::volume(&delete_faces(&b, &notch).unwrap()), 24.0);
    }

    #[test]
    fn a_face_thickened_is_a_slab_on_it() {
        // The 4 × 2 side of a 4 × 3 × 2 block thickened 0.5 outward, then 1 inward.
        let b = prism(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]], 2.0);
        let side = face(&b, "e0");
        close(measure::volume(&thicken(&b, &[side], 0.5).unwrap()), 4.0);
        close(measure::volume(&thicken(&b, &[side], -1.0).unwrap()), 8.0);
    }

    #[test]
    fn an_l_blocks_shell_open_at_its_step_keeps_the_wall_rising_past_it() {
        // The L of 20 × 5 with a 5 × 15 tower at x 15..20, 20 tall, open at the step face (e4, y = 5).
        // Inward, 1 thick: the cavity is 18 × 4 + 3 × 14 = 114 across, 18 high; the wall past the step
        // stays 1 thick down to the opening: 3500 − 114 · 18. Outward: the L grown by 1 everywhere but
        // at the opening, (22 × 6 + 7 × 16) · 22, less the L itself: 5368 − 3500.
        let l = prism(&[[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [15.0, 20.0], [15.0, 5.0], [0.0, 5.0]], 20.0);
        let open = face(&l, "e4");
        let inward = shell(&l, &[open], 1.0, Wall::Inward, "s").unwrap();
        close(measure::volume(&inward), 3500.0 - 114.0 * 18.0);
        let outward = shell(&l, &[open], 1.0, Wall::Outward, "s").unwrap();
        close(measure::volume(&outward), 5368.0 - 3500.0);
    }

    #[test]
    fn a_cone_shelled_open_at_its_base_is_the_cone_less_its_offset_cone() {
        // A cone of radius 6 and height 8 (sin a = 0.6, tan a = 0.75), 1 thick, open at the base: the
        // cavity is the cone offset in by 1, whose apex has come down 1 / sin a along the axis, so it
        // is a cone of height h' = 8 - 1/0.6 on the same base plane: V = (pi/3)(6^2 8 - r'^2 h').
        let c = crate::primitive::cone(&Frame::WORLD, 6.0, 0.0, 8.0, "c").unwrap();
        let base = c.face_ids().find(|&f| crate::measure::face_bbox(&c, f).max.z < 0.1).unwrap();
        let h = 8.0 - 1.0 / 0.6;
        let r = 0.75 * h;
        let b = shell(&c, &[base], 1.0, Wall::Inward, "s").unwrap();
        close(measure::volume(&b), PI / 3.0 * (36.0 * 8.0 - r * r * h));
    }

    #[test]
    fn a_cones_face_moved_either_way_moves_its_apex_along_the_axis() {
        // The cone of radius 6 and height 8, its side moved by d with the base kept: the apex moves
        // d / sin a = d / 0.6 up the axis (down for a negative d), the cone being the same angle
        // cut by the base plane: V = (pi/3) r'^2 h' with h' = 8 + d / 0.6 and r' = 0.75 h'.
        let c = crate::primitive::cone(&Frame::WORLD, 6.0, 0.0, 8.0, "c").unwrap();
        let side = c.face_ids().find(|&f| crate::measure::face_bbox(&c, f).max.z > 7.9).unwrap();
        for d in [1.0, -1.0] {
            let by: BTreeMap<FaceId, f64> = std::iter::once((side, d)).collect();
            let moved = offset_faces(&c, &by).unwrap();
            let h = 8.0 + d / 0.6;
            close(measure::volume(&moved), PI / 3.0 * (0.75 * h).powi(2) * h);
        }
    }

    #[test]
    fn an_outward_offset_of_a_cone_rounds_its_apex_with_a_ball() {
        // The 3 x 4 cone (slant 5, sin a = 0.6) offset by 1: in the meridian triangle's half plane the
        // solid is the cone 12 pi, the slant strip 19 pi, the base slab 9 pi, and the sectors round the
        // base corner (from the base normal to the slant's, 2 pi (3 t / 2 + 1.6 / 3) with
        // t = asin 0.6 + pi / 2) and the apex (2 pi 0.4 / 3): Pappus' theorem on each piece.
        let c = crate::primitive::cone(&Frame::WORLD, 3.0, 0.0, 4.0, "c").unwrap();
        let t = 0.6_f64.asin() + PI / 2.0;
        let want = 40.0 * PI + 2.0 * PI * (3.0 * t / 2.0 + 1.6 / 3.0) + 2.0 * PI * 0.4 / 3.0;
        let o = offset(&c, 1.0, "o").unwrap();
        close(measure::volume(&o), want);
        assert!(crate::check::check(&o).is_empty());
    }

    #[test]
    fn a_frustums_cavity_closing_below_its_far_face_is_the_cone_of_its_apex() {
        // The 6 to 3 frustum, 4 tall (sin a = 0.6, the whole cone 8 tall), shelled 2.7 thick: the
        // cavity cone's apex is at 8 - 2.7 / 0.6 = 3.5, below the top at 4, and its base on the
        // bottom moved up to 2.7: a cone 0.8 tall of radius 0.6. Open at the top and at both ends
        // alike (the bottom out to -2.7: a cone 3.5 tall from the plane z = 0, radius 2.625).
        let f = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 4.0, "f").unwrap();
        let face = |lo: f64, hi: f64| f.face_ids().find(|&g| {
            let b = crate::measure::face_bbox(&f, g);
            matches!(f.face(g).surface, Surface::Plane(_)) && b.min.z > lo && b.max.z < hi
        }).unwrap();
        let (bottom, top) = (face(-0.1, 0.1), face(3.9, 4.1));
        let whole = PI / 3.0 * 4.0 * (36.0 + 18.0 + 9.0);
        let sealed = shell(&f, &[top], 2.7, Wall::Inward, "s").unwrap();
        close(measure::volume(&sealed), whole - PI / 3.0 * 0.36 * 0.8);
        let through = shell(&f, &[bottom, top], 2.7, Wall::Inward, "s").unwrap();
        close(measure::volume(&through), whole - PI / 3.0 * 2.625 * 2.625 * 3.5);
    }

    #[test]
    fn deleting_the_holes_of_a_plate_fills_them_however_the_plate_lies() {
        // A 10 x 5 plate, 1 thick, with three holes: deleting their walls leaves the whole plate,
        // 50, in any pose (the fill block must not stick out of a turned plate).
        let at = |k: f64| Vec2::new(2.5 * k, 2.5);
        let holes = (1..=3).map(|k| Profile::circle(at(k as f64), 1.2 + 0.1 * k as f64).loops.remove(0));
        let plate = Profile::rect(Vec2::new(5.0, 2.5), 10.0, 5.0).with_holes(holes);
        for z in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 2.0, 3.0).unit().unwrap(), Vec3::new(-2.0, 1.0, 0.5).unit().unwrap()] {
            let pose = Frame::new(Point3::new(1.0, -2.0, 3.0), z, z.any_perpendicular()).unwrap();
            let b = extrude(&plate, &pose, 1.0, "p").unwrap();
            let walls: Vec<FaceId> = b.face_ids().filter(|&f| matches!(b.face(f).surface, Surface::Cylinder(_))).collect();
            assert_eq!(walls.len(), 3);
            let filled = delete_faces(&b, &walls).unwrap();
            close(measure::volume(&filled), 50.0);
        }
    }

    #[test]
    fn deleting_two_holes_leaves_the_one_between_them() {
        // The 10 x 5 plate with holes of diameter 1, 1.5 and 2 along its middle: deleting the first and
        // the last fills them and leaves the middle one, V = 50 - pi (0.75)^2 = 48.2327 (1 thick).
        let holes = [(2.0, 1.0), (5.0, 1.5), (8.0, 2.0)].map(|(x, d)| Profile::circle(Vec2::new(x, 2.5), d).loops.remove(0));
        let b = extrude(&Profile::rect(Vec2::new(5.0, 2.5), 10.0, 5.0).with_holes(holes), &Frame::WORLD, 1.0, "p").unwrap();
        let wall = |x: f64| b.face_ids().find(|&f| matches!(b.face(f).surface, Surface::Cylinder(_)) && (crate::measure::face_bbox(&b, f).center().x - x).abs() < 0.1).unwrap();
        let out = delete_faces(&b, &[wall(2.0), wall(8.0)]).unwrap();
        close(measure::volume(&out), 50.0 - PI * 0.75 * 0.75);
    }

    #[test]
    fn deleting_a_boss_leaves_the_plate_and_a_round_in_an_inside_corner_the_sharp_corner() {
        // A disc of radius 8 and 4 thick under a post of radius 3 and 8 tall: deleting the post's wall and
        // top leaves the disc, V = pi 8^2 4.
        let disc = crate::primitive::cylinder(&Frame::new(Point3::new(0.0, 0.0, -4.0), Vec3::Z, Vec3::X).unwrap(), 8.0, 4.0, "d").unwrap();
        let post = crate::primitive::cylinder(&Frame::WORLD, 3.0, 8.0, "p").unwrap();
        let part = boolean::union(&disc, &post).unwrap();
        let wall = part.face_ids().find(|&f| matches!(&part.face(f).surface, Surface::Cylinder(c) if c.radius < 4.0)).unwrap();
        let top = part.face_ids().find(|&f| matches!(&part.face(f).surface, Surface::Plane(_)) && crate::measure::face_bbox(&part, f).min.z > 7.9).unwrap();
        let out = delete_faces(&part, &[wall, top]).unwrap();
        close(measure::volume(&out), PI * 64.0 * 4.0);
    }

    #[test]
    fn a_shell_goes_past_a_round_thinner_than_its_wall() {
        // A puck of radius 5 and 4 tall, its top rim rounded by 0.5, shelled 1 thick open at the bottom: the
        // round is thinner than the wall and closes away, the cavity a cylinder of radius 4 and height 3.
        // The rounded puck holds pi 25 4 less what the round took off the rim (Pappus: the square less the
        // quarter disc, 0.5^2 (1 - pi/4), its centre of area 0.5 (5/6 - pi/4) / (1 - pi/4) in from the rim).
        let puck = crate::primitive::cylinder(&Frame::WORLD, 5.0, 4.0, "p").unwrap();
        let rim = puck.edge_ids().find(|&e| matches!(puck.edge(e).curve, Some(Curve::Circle(ref c)) if c.frame.origin.z > 3.9)).unwrap();
        let rounded = blend::fillet_edges(&puck, &[rim], 0.5, "r").unwrap();
        let base = rounded.face_ids().find(|&f| matches!(rounded.face(f).surface, Surface::Plane(_)) && crate::measure::face_bbox(&rounded, f).max.z < 0.1).unwrap();
        let area = 0.25 * (1.0 - PI / 4.0);
        let inward = 0.5 * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0);
        let outer = PI * 25.0 * 4.0 - 2.0 * PI * (5.0 - inward) * area;
        let cup = shell(&rounded, &[base], 1.0, Wall::Inward, "s").unwrap();
        close(measure::volume(&cup), outer - PI * 16.0 * 3.0);
    }

    #[test]
    fn a_healed_cone_is_one_face_again() {
        // A frustum 6 to 3, 6 tall, its top rim rounded by 1: deleting the round heals the cone and
        // the top, three faces again (cone, top, bottom), the frustum's 126 pi.
        let cone = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 6.0, "c").unwrap();
        let rim = cone.edge_ids().find(|&e| matches!(cone.edge(e).curve, Some(Curve::Circle(ref c)) if c.frame.origin.z > 5.9)).unwrap();
        let rounded = blend::fillet_edges(&cone, &[rim], 1.0, "r").unwrap();
        let round = rounded.face_ids().find(|&f| matches!(rounded.face(f).surface, Surface::Torus(_))).unwrap();
        let healed = delete_faces(&rounded, &[round]).unwrap();
        close(measure::volume(&healed), 126.0 * PI);
        assert_eq!(healed.face_ids().count(), 3);
        assert!(healed.face_ids().all(|f| !healed.face(f).names.is_empty()));
    }

    #[test]
    fn a_cube_edge_rounded_with_a_radius_growing_along_it() {
        // brep-bench fillet/variable-radius-edge: the 10 cube, the edge x = y = 10 rounded by circles
        // square to it, radius 1 + z / 10: 1000 - (70/3)(1 - pi/4).
        let cube = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "c").unwrap();
        let edge = cube
            .edge_ids()
            .find(|&e| {
                let (a, b) = (cube.vertex(cube.edge(e).start).point, cube.vertex(cube.edge(e).end).point);
                (a.x - 10.0).abs() < 1e-9 && (a.y - 10.0).abs() < 1e-9 && (b.x - 10.0).abs() < 1e-9 && (b.y - 10.0).abs() < 1e-9 // tol: the edge x = y = 10
            })
            .unwrap();
        let rounded = fillet_variable(&cube, edge, Point3::new(10.0, 10.0, 0.0), 1.0, 2.0, "f").unwrap();
        let want = 1000.0 - 70.0 / 3.0 * (1.0 - PI / 4.0);
        assert!((measure::volume(&rounded) - want).abs() < 1e-9 * want, "{} against {want}", measure::volume(&rounded));
        // The other way round is the mirror image.
        let flipped = fillet_variable(&cube, edge, Point3::new(10.0, 10.0, 10.0), 1.0, 2.0, "f").unwrap();
        assert!((measure::volume(&flipped) - want).abs() < 1e-9 * want);
        assert!(rounded.face_ids().any(|f| rounded.face(f).names.iter().any(|n| n.face == "round")));
    }

    #[test]
    fn every_face_of_a_shell_is_named_from_the_start() {
        // The frustum whose cavity closes below its top (the cone of its apex), a puck's rounds taken
        // out of its cavity, and a block wrapped outward: each face is named, the cavity's and the
        // outward wall's each for the face of the solid it offsets.
        let named = |b: &Brep| b.face_ids().all(|f| !b.face(f).names.is_empty());
        let frustum = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 4.0, "f").unwrap();
        let top = frustum.face_ids().find(|&g| matches!(frustum.face(g).surface, Surface::Plane(_)) && crate::measure::face_bbox(&frustum, g).min.z > 3.9).unwrap();
        let sealed = shell(&frustum, &[top], 2.7, Wall::Inward, "s").unwrap();
        assert!(named(&sealed));
        let offsets = |b: &Brep| b.face_ids().filter(|&f| b.face(f).names.iter().any(|n| n.face.starts_with("inner_f"))).count();
        assert_eq!(offsets(&sealed), 2);
        let block = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
        let lid = block.face_ids().find(|&g| crate::measure::face_bbox(&block, g).min.z > 9.9).unwrap();
        let wrapped = shell(&block, &[lid], 1.0, Wall::Outward, "s").unwrap();
        assert!(named(&wrapped));
        assert_eq!(offsets(&wrapped), 5);
    }

    #[test]
    fn a_hemispheres_rim_chamfered_by_distances_along_its_faces() {
        // A hemisphere of radius 5 on its plane z = 0, its rim chamfered 2 along the plane and 1 along the
        // sphere (a chord of 1: a turn of 2 asin(1/10)): the corner left of the chord from (3, 0) to (5 cos 0.2, 5 sin 0.2)
        // in the half plane, turned about the axis: the triangle with the plane, whose centroid is the
        // mean of its corners, and the circular segment out to the arc (Pappus on each).
        let o = Point3::ZERO;
        let f = Frame::from_axis(o, Vec3::Z).unwrap();
        let line = |a: [f64; 2], b: [f64; 2]| Segment::Line { a: Vec2::new(a[0], a[1]), b: Vec2::new(b[0], b[1]) };
        let meridian = Profile {
            loops: vec![ProfileLoop {
                segments: vec![
                    named(line([0.0, 0.0], [5.0, 0.0]), "base"),
                    named(Segment::Arc { center: Vec2::ZERO, radius: 5.0, start: 0.0, sweep: PI / 2.0 }, "dome"),
                    named(line([0.0, 5.0], [0.0, 0.0]), "axis"),
                ],
            }],
        };
        let dome = revolve(&meridian, &Frame::new(o, -f.y, f.x).unwrap(), o, f.z, TAU, "h").unwrap();
        let rim = dome.edge_ids().find(|&e| matches!(dome.edge(e).curve, Some(Curve::Circle(ref c)) if c.frame.origin.z.abs() < 1e-9 && (c.radius - 5.0).abs() < 1e-9)).unwrap();
        let base = dome.face_ids().find(|&g| matches!(dome.face(g).surface, Surface::Plane(_))).unwrap();
        let cut = chamfer_two(&dome, rim, base, 2.0, 1.0, "c").unwrap();
        let theta = 2.0 * 0.1_f64.asin();
        let (x2, y2) = (5.0 * theta.cos(), 5.0 * theta.sin());
        let triangle = 2.0 * PI * ((3.0 + 5.0 + x2) / 3.0) * y2;
        let segment_area = 12.5 * (theta - theta.sin());
        let centroid = 4.0 * 5.0 * (theta / 2.0).sin().powi(3) / (3.0 * (theta - theta.sin())) * (theta / 2.0).cos();
        let taken = triangle + 2.0 * PI * centroid * segment_area;
        close(measure::volume(&cut), 2.0 / 3.0 * PI * 125.0 - taken);
        assert!(cut.face_ids().any(|g| cut.face(g).names.iter().any(|n| n.face == "bevel_0")));
    }
}
