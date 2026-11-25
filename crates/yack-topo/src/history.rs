// yack: what an operation did to its operands' faces (PLAN §4.1 `History`), worked out from the face names.
//! A [`History`] says, for every face of an operation's result, how it came about ([`Change`]) and
//! from what ([`FaceOrigin`]); and which of the operands' faces no result face descends from.
//!
//! [`derive`] reads it off the face names every operation keeps (see [`FaceName`]): a face that
//! keeps an operand face's names continues that face, and one named by the operation's own
//! feature is new, its name saying from what. A name that more than one operand face carries is
//! settled by the surface; a face it cannot settle gets no origin, never a guess.

use std::collections::{BTreeMap, BTreeSet};

use super::{Brep, EdgeId, Face, FaceId, FaceName, VertexId};
use crate::math::Aabb;

/// How a result face came about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Change {
    /// An operand face, unchanged: the same surface, bounded by as many loops of as many edges.
    Kept,
    /// An operand face, trimmed, extended or moved.
    Modified,
    /// One of the pieces an operand face was cut into.
    Split,
    /// Several operand faces (or an operand face and a new one) made one.
    Merged,
    /// A face the operation made.
    Generated,
}

/// Which end cap of a sweep a face is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CapEnd {
    /// Where the sweep starts (on the profile's plane).
    Start,
    /// Where the sweep ends.
    End,
}

/// What a result face came from.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum FaceOrigin {
    /// Face `face` of operand `operand`.
    Operand { operand: u8, face: FaceId },
    /// A profile segment's sweep (its label), or a sweep's end cap.
    Segment { label: String, end: Option<CapEnd> },
    /// Edge `edge` of operand `operand` (a fillet or chamfer face, a shell opening's rim wall).
    Edge { operand: u8, edge: EdgeId },
    /// Vertex `vertex` of operand `operand` (a blend's corner patch).
    Vertex { operand: u8, vertex: VertexId },
    /// Face `face` of operand `operand`, offset (a shell's, an offset's or a thickening's face).
    Offset { operand: u8, face: FaceId },
    /// The `face`-th face of a file or an encoding read.
    Read { face: u32 },
}

/// What an operation did to its operands' faces.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct History {
    /// How many operands the operation took.
    pub operands: u8,
    /// One row per result face, by face id: how it came about and from what.
    pub faces: Vec<(FaceId, Change, Vec<FaceOrigin>)>,
    /// The operand faces no result face descends from, as (operand, face).
    pub deleted: Vec<(u8, FaceId)>,
}

/// A result face, the origins its new names give it, and the operand faces it continues.
type Row = (FaceId, Vec<FaceOrigin>, BTreeSet<(u8, FaceId)>);

/// The origin a name of the operation's own `feature` gives a new face, if the name says one: a
/// blend's `round_<k>` / `bevel_<k>` is the operation's `k`-th edge of operand 0 (`edges`).
fn generated(name: &FaceName, edges: &[EdgeId], operand: Option<&Brep>) -> Option<FaceOrigin> {
    let part = name.face.as_str();
    // what a shell's cavity face offsets and what an offset's round or corner patch grows from,
    // named by id in operand 0 (`inner_f<k>`, `round_e<k>`, `corner_v<k>`): only an id it has
    let id = |prefix: &str| part.strip_prefix(prefix).and_then(|k| k.parse::<u32>().ok());
    let has = |n: usize, k: u32| usize::try_from(k).is_ok_and(|k| k < n);
    if let (Some(k), Some(b)) = (id("inner_f"), operand) {
        return has(b.faces.len(), k).then(|| FaceOrigin::Offset { operand: 0, face: FaceId::from_raw(k) });
    }
    if let (Some(k), Some(b)) = (id("round_e"), operand) {
        return has(b.edges.len(), k).then(|| FaceOrigin::Edge { operand: 0, edge: EdgeId::from_raw(k) });
    }
    if let (Some(k), Some(b)) = (id("corner_v"), operand) {
        return has(b.vertices.len(), k).then(|| FaceOrigin::Vertex { operand: 0, vertex: VertexId::from_raw(k) });
    }
    let blend = part.strip_prefix("round_").or_else(|| part.strip_prefix("bevel_"));
    if let Some(k) = blend.and_then(|k| k.parse::<usize>().ok()) {
        return edges.get(k).map(|&edge| FaceOrigin::Edge { operand: 0, edge });
    }
    match part {
        "" => None,
        FaceName::START => Some(FaceOrigin::Segment { label: part.to_string(), end: Some(CapEnd::Start) }),
        FaceName::END => Some(FaceOrigin::Segment { label: part.to_string(), end: Some(CapEnd::End) }),
        // a blend corner's vertex comes from its neighbours' edges (`corners`); a cavity face's
        // and an offset round's source come with their `inner_f` / `round_e` names
        FaceName::INNER | "corner" => None,
        _ if part.starts_with("inner_f") || part.starts_with("corner_v") || part.starts_with("round_e") => None,
        _ if blend.is_some() => None,
        _ => match part.strip_prefix("face_").and_then(|n| n.parse::<u32>().ok()) {
            Some(face) => Some(FaceOrigin::Read { face }),
            None => Some(FaceOrigin::Segment { label: part.strip_prefix("seg:").unwrap_or(part).to_string(), end: None }),
        },
    }
}

/// Face `f` of `b`, if `b` has it (an id that names nothing gives no origin, never a panic).
fn face(b: &Brep, f: FaceId) -> Option<&Face> {
    b.faces.get(f.index())
}

/// How many coedges loop `l` of `b` has, if `b` has it.
fn loop_len(b: &Brep, l: super::LoopId) -> Option<usize> {
    b.loops.get(l.index()).map(|lp| lp.coedges.len())
}

/// The faces across an edge from face `f` of `b` (ids that name nothing skipped).
fn neighbours(b: &Brep, f: FaceId) -> BTreeSet<FaceId> {
    let mut out = BTreeSet::new();
    let coedges = face(b, f).into_iter().flat_map(|x| &x.loops).filter_map(|l| b.loops.get(l.index())).flat_map(|lp| &lp.coedges);
    for c in coedges {
        let Some(edge) = b.coedges.get(c.index()).map(|x| x.edge) else { continue };
        for u in b.uses.get(edge.index()).into_iter().flatten() {
            let g = b.coedges.get(u.index()).and_then(|x| b.loops.get(x.lp.index())).map(|lp| lp.face);
            out.extend(g.filter(|&g| g != f));
        }
    }
    out
}

/// Each blend corner patch of `result` (a new face named `corner` with no origin yet), given the
/// one vertex of `operand` that the edges its neighbouring blend faces come from all end at.
fn corners(rows: &mut [Row], result: &Brep, feature: &str, operand: Option<&Brep>) {
    let Some(operand) = operand else { return };
    let edge_of: BTreeMap<FaceId, EdgeId> = rows
        .iter()
        .filter_map(|(f, made, _)| {
            made.iter().find_map(|o| match o {
                FaceOrigin::Edge { operand: 0, edge } => Some((*f, *edge)),
                _ => None,
            })
        })
        .collect();
    for (f, made, from) in rows.iter_mut() {
        let is_corner = face(result, *f).is_some_and(|x| x.names.iter().any(|n| n.feature == feature && n.face == "corner"));
        if !is_corner || !made.is_empty() || !from.is_empty() {
            continue;
        }
        let mut common: Option<BTreeSet<VertexId>> = None;
        for g in neighbours(result, *f) {
            let Some(e) = edge_of.get(&g).and_then(|e| operand.edges.get(e.index())) else { continue };
            let ends: BTreeSet<VertexId> = [e.start, e.end].into_iter().collect();
            common = Some(match common {
                None => ends,
                Some(c) => c.intersection(&ends).copied().collect(),
            });
        }
        if let Some(c) = common
            && let [vertex] = c.into_iter().collect::<Vec<_>>().as_slice()
        {
            made.push(FaceOrigin::Vertex { operand: 0, vertex: *vertex });
        }
    }
}

/// The history of a solid read from a file or an encoding: every face new, the `n`-th face read.
pub fn read(result: &Brep) -> History {
    let faces = result
        .face_ids()
        .enumerate()
        .map(|(n, f)| (f, Change::Generated, u32::try_from(n).ok().map(|face| FaceOrigin::Read { face }).into_iter().collect()))
        .collect();
    History { operands: 0, faces, deleted: Vec::new() }
}

/// Of the operand faces `cands`, the ones whose box holds result face `r`'s box (within the
/// tolerance): a piece of a face lies within it, so this settles a name several operand faces
/// carry on one surface (a face split earlier, a coplanar pair) wherever their boxes differ.
fn inside(inputs: &[&Brep], result: &Brep, r: FaceId, cands: &[(u8, FaceId)]) -> Vec<(u8, FaceId)> {
    let held = crate::bbox::face_bbox(result, r);
    let e = crate::tol::LINEAR;
    let holds = |o: &Aabb| {
        (o.min.x - e <= held.min.x && o.min.y - e <= held.min.y && o.min.z - e <= held.min.z)
            && (held.max.x <= o.max.x + e && held.max.y <= o.max.y + e && held.max.z <= o.max.z + e)
    };
    cands
        .iter()
        .copied()
        .filter(|&(k, f)| inputs.get(usize::from(k)).filter(|b| face(b, f).is_some()).is_some_and(|b| holds(&crate::bbox::face_bbox(b, f))))
        .collect()
}

/// Each operand with its index, up to `u8::MAX` operands.
fn numbered<'a>(inputs: &'a [&'a Brep]) -> impl Iterator<Item = (u8, &'a Brep)> + 'a {
    inputs.iter().enumerate().filter_map(|(k, b)| Some((u8::try_from(k).ok()?, *b)))
}

/// Whether result face `r` of `result` is operand face `o` of `operand` unchanged: the same surface
/// the same way round, bounded by as many loops of as many edges each.
fn unchanged(result: &Brep, r: FaceId, operand: &Brep, o: FaceId) -> bool {
    let (Some(a), Some(b)) = (face(result, r), face(operand, o)) else { return false };
    a.surface == b.surface
        && a.reversed == b.reversed
        && a.loops.len() == b.loops.len()
        && a.loops.iter().zip(&b.loops).all(|(&x, &y)| loop_len(result, x).is_some_and(|n| Some(n) == loop_len(operand, y)))
}

/// The history of an operation that made `result` from `inputs`, whose new faces carry the names of
/// its own `feature`; a blend's `edges` are its edges of operand 0 in the order it took them (its
/// faces' names count them).
pub fn derive(inputs: &[&Brep], result: &Brep, feature: &str, edges: &[EdgeId]) -> History {
    // more operands than an origin can number: every face without an origin (the audit reports
    // it), nothing guessed
    if inputs.len() > usize::from(u8::MAX) {
        let faces = result.face_ids().map(|f| (f, Change::Generated, Vec::new())).collect();
        return History { operands: u8::MAX, faces, deleted: Vec::new() };
    }
    // every operand face carrying each name
    let mut carriers: BTreeMap<&FaceName, Vec<(u8, FaceId)>> = BTreeMap::new();
    for (k, b) in numbered(inputs) {
        for f in b.face_ids() {
            for n in face(b, f).map(|x| &x.names).into_iter().flatten() {
                carriers.entry(n).or_default().push((k, f));
            }
        }
    }
    let mut rows: Vec<Row> = Vec::new();
    for r in result.face_ids() {
        let Some(this) = face(result, r) else { continue };
        let mut from: BTreeSet<(u8, FaceId)> = BTreeSet::new();
        let mut made: Vec<FaceOrigin> = Vec::new();
        for n in &this.names {
            if n.feature == feature {
                made.extend(generated(n, edges, inputs.first().copied()));
                continue;
            }
            let Some(cands) = carriers.get(n) else { continue };
            let pick: Vec<(u8, FaceId)> = if cands.len() == 1 {
                cands.clone()
            } else {
                let same: Vec<(u8, FaceId)> = cands
                    .iter()
                    .copied()
                    .filter(|&(k, f)| inputs.get(usize::from(k)).and_then(|b| face(b, f)).is_some_and(|x| x.surface == this.surface))
                    .collect();
                if same.len() == 1 { same } else { inside(inputs, result, r, if same.is_empty() { cands } else { &same }) }
            };
            if let [one] = pick.as_slice() {
                from.insert(*one);
            }
        }
        made.sort();
        made.dedup();
        rows.push((r, made, from));
    }
    corners(&mut rows, result, feature, inputs.first().copied());
    // how many result faces each operand face reaches
    let mut reach: BTreeMap<(u8, FaceId), usize> = BTreeMap::new();
    for (_, _, from) in &rows {
        for &o in from {
            *reach.entry(o).or_default() += 1;
        }
    }
    let faces = rows
        .into_iter()
        .map(|(r, made, from)| {
            let change = match (from.len(), made.is_empty()) {
                (0, _) => Change::Generated,
                (1, true) => {
                    let (k, o) = from.iter().next().copied().unwrap_or((0, r));
                    if reach.get(&(k, o)).copied().unwrap_or(0) > 1 {
                        Change::Split
                    } else if inputs.get(usize::from(k)).is_some_and(|b| unchanged(result, r, b, o)) {
                        Change::Kept
                    } else {
                        Change::Modified
                    }
                }
                _ => Change::Merged,
            };
            let mut origins: Vec<FaceOrigin> = from.into_iter().map(|(operand, face)| FaceOrigin::Operand { operand, face }).collect();
            origins.extend(made);
            (r, change, origins)
        })
        .collect();
    let deleted = numbered(inputs)
        .flat_map(|(k, b)| b.face_ids().map(move |f| (k, f)))
        .filter(|o| !reach.contains_key(o))
        .collect();
    History { operands: u8::try_from(inputs.len()).unwrap_or(u8::MAX), faces, deleted }
}

/// [`derive`] from the face names alone, for solids that have no springrep form here (a cadrs-held
/// one): each solid as its faces' names by face index. Without surfaces a name several operand
/// faces carry gives no origin, and a face is never called kept, only modified.
pub fn derive_names(inputs: &[&[BTreeSet<FaceName>]], result: &[BTreeSet<FaceName>], feature: &str, edges: &[EdgeId]) -> History {
    let id = |i: usize| u32::try_from(i).ok().map(FaceId::from_raw);
    if inputs.len() > usize::from(u8::MAX) {
        let faces = (0..result.len()).filter_map(id).map(|f| (f, Change::Generated, Vec::new())).collect();
        return History { operands: u8::MAX, faces, deleted: Vec::new() };
    }
    let numbered = || inputs.iter().enumerate().filter_map(|(k, b)| Some((u8::try_from(k).ok()?, *b)));
    let mut carriers: BTreeMap<&FaceName, Vec<(u8, FaceId)>> = BTreeMap::new();
    for (k, faces) in numbered() {
        for (i, names) in faces.iter().enumerate() {
            let Some(f) = id(i) else { continue };
            for n in names {
                carriers.entry(n).or_default().push((k, f));
            }
        }
    }
    let mut rows: Vec<Row> = Vec::new();
    for (i, names) in result.iter().enumerate() {
        let Some(r) = id(i) else { continue };
        let mut from: BTreeSet<(u8, FaceId)> = BTreeSet::new();
        let mut made: Vec<FaceOrigin> = Vec::new();
        for n in names {
            if n.feature == feature {
                made.extend(generated(n, edges, None));
            } else if let Some([one]) = carriers.get(n).map(Vec::as_slice) {
                from.insert(*one);
            }
        }
        made.sort();
        made.dedup();
        rows.push((r, made, from));
    }
    let mut reach: BTreeMap<(u8, FaceId), usize> = BTreeMap::new();
    for (_, _, from) in &rows {
        for &o in from {
            *reach.entry(o).or_default() += 1;
        }
    }
    let faces = rows
        .into_iter()
        .map(|(r, made, from)| {
            let change = match (from.len(), made.is_empty()) {
                (0, _) => Change::Generated,
                (1, true) if from.iter().any(|o| reach.get(o).copied().unwrap_or(0) > 1) => Change::Split,
                (1, true) => Change::Modified,
                _ => Change::Merged,
            };
            let mut origins: Vec<FaceOrigin> = from.into_iter().map(|(operand, face)| FaceOrigin::Operand { operand, face }).collect();
            origins.extend(made);
            (r, change, origins)
        })
        .collect();
    let deleted = numbered()
        .flat_map(|(k, faces)| (0..faces.len()).filter_map(id).map(move |f| (k, f)))
        .filter(|o| !reach.contains_key(o))
        .collect();
    History { operands: u8::try_from(inputs.len()).unwrap_or(u8::MAX), faces, deleted }
}

/// The history of an operation that changed nothing about its one operand's faces but their
/// names (a renaming): every face kept, from the face of the same id.
pub fn identity(b: &Brep) -> History {
    let faces = b.face_ids().map(|f| (f, Change::Kept, vec![FaceOrigin::Operand { operand: 0, face: f }])).collect();
    History { operands: 1, faces, deleted: Vec::new() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{Plane, Surface};
    use crate::math::{Frame, Vec3};
    use crate::LoopId;

    /// A cube's six planes, each one face of one (empty) loop, named `feature.<side>`: enough
    /// topology for names and loops, built with topo's own builder.
    fn cube(feature: &str) -> Brep {
        let mut b = Brep::new();
        let sides = [
            ("bottom", Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0)),
            ("top", Vec3::new(0.0, 0.0, 1.0), Vec3::Z),
            ("front", Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
            ("back", Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
            ("left", Vec3::new(0.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)),
            ("right", Vec3::new(1.0, 0.0, 0.0), Vec3::X),
        ];
        for (name, at, normal) in sides {
            let x = if normal.z == 0.0 { Vec3::Z } else { Vec3::X };
            let frame = Frame::new(at, normal, x).unwrap();
            let f = b.add_face(Surface::Plane(Plane { frame }), false, [FaceName::new(feature, name)]);
            b.add_loop(f, Vec::new());
        }
        b
    }

    #[test]
    fn a_renamed_block_is_kept_face_for_face() {
        let b = cube("a");
        let h = derive(&[&b], &b, "new", &[]);
        assert_eq!(h.operands, 1);
        assert!(h.faces.iter().all(|(_, c, o)| *c == Change::Kept && o.len() == 1));
        assert!(h.deleted.is_empty());
    }

    #[test]
    fn ids_that_name_nothing_give_no_origin_and_no_panic() {
        let b = cube("a");
        let mut odd = b.clone();
        odd.faces[0].loops.push(LoopId::from_raw(9999));
        let h = derive(&[&b], &odd, "new", &[]);
        assert_eq!(h.faces.len(), 6);
        // the face with the stray loop is no longer the operand's face unchanged
        assert!(h.faces.iter().any(|(_, c, _)| *c == Change::Modified));
        let h = derive(&[&odd], &b, "new", &[]);
        assert_eq!(h.faces.len(), 6);
    }

    #[test]
    fn more_operands_than_an_origin_can_number_give_faces_without_origins() {
        let b = cube("a");
        let many: Vec<&Brep> = std::iter::repeat_n(&b, 300).collect();
        let h = derive(&many, &b, "new", &[]);
        assert_eq!(h.operands, u8::MAX);
        assert!(h.faces.iter().all(|(_, c, o)| *c == Change::Generated && o.is_empty()));
        let h = derive(&many[..255], &b, "new", &[]);
        assert_eq!(h.operands, 255);
    }

    #[test]
    fn names_alone_say_where_a_held_solid_s_faces_came_from() {
        let b = cube("a");
        let names: Vec<BTreeSet<FaceName>> = b.face_ids().map(|f| b.face(f).names.clone()).collect();
        let mut out = names.clone();
        out.push([FaceName::new("new", "round_0")].into_iter().collect());
        let edge = EdgeId::from_raw(3);
        let h = derive_names(&[&names], &out, "new", &[edge]);
        assert_eq!(h.faces.len(), 7);
        assert!(h.faces[..6].iter().all(|(_, c, o)| *c == Change::Modified && o.len() == 1));
        assert_eq!(h.faces[6].2, vec![FaceOrigin::Edge { operand: 0, edge }]);
        assert!(h.deleted.is_empty());
    }

    #[test]
    fn source_id_names_give_offset_edge_and_vertex_origins_only_for_ids_the_operand_has() {
        let b = cube("a");
        let mut out = b.clone();
        for (f, name) in [(0, "inner_f2"), (1, "round_e999"), (2, "corner_v0")] {
            out.faces[f].names = [FaceName::new("new", name)].into_iter().collect();
        }
        let h = derive(&[&b], &out, "new", &[]);
        assert_eq!(h.faces[0].2, vec![FaceOrigin::Offset { operand: 0, face: FaceId::from_raw(2) }]);
        // the cube built here has no edges: an edge id it does not have gives no origin
        assert!(h.faces[1].2.is_empty());
        assert!(h.faces[2].2.is_empty());
    }
}
