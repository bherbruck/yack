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

/// The origin a name of the operation's own `feature` gives a new face, if the name says one.
fn generated(name: &FaceName) -> Option<FaceOrigin> {
    let part = name.face.as_str();
    match part {
        "" => None,
        FaceName::START => Some(FaceOrigin::Segment { label: part.to_string(), end: Some(CapEnd::Start) }),
        FaceName::END => Some(FaceOrigin::Segment { label: part.to_string(), end: Some(CapEnd::End) }),
        // blends' and shells' faces: their origins (edges, vertices, offset faces) come with the
        // per-operation origins (M4.11 step 2)
        FaceName::INNER | "corner" => None,
        _ if part.starts_with("round_") || part.starts_with("bevel_") => None,
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
/// its own `feature`.
pub fn derive(inputs: &[&Brep], result: &Brep, feature: &str) -> History {
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
                made.extend(generated(n));
                continue;
            }
            let Some(cands) = carriers.get(n) else { continue };
            let pick: Vec<(u8, FaceId)> = if cands.len() == 1 {
                cands.clone()
            } else {
                cands
                    .iter()
                    .copied()
                    .filter(|&(k, f)| inputs.get(usize::from(k)).and_then(|b| face(b, f)).is_some_and(|x| x.surface == this.surface))
                    .collect()
            };
            if let [one] = pick.as_slice() {
                from.insert(*one);
            }
        }
        made.sort();
        made.dedup();
        rows.push((r, made, from));
    }
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

/// The history of an operation that changed nothing about its one operand's faces but their
/// names (a renaming): every face kept, from the face of the same id.
pub fn identity(b: &Brep) -> History {
    let faces = b.face_ids().map(|f| (f, Change::Kept, vec![FaceOrigin::Operand { operand: 0, face: f }])).collect();
    History { operands: 1, faces, deleted: Vec::new() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Frame, Vec3};
    use crate::primitive::block;
    use crate::topo::LoopId;

    fn cube(feature: &str) -> Brep {
        block(&Frame::new(Vec3::new(0.0, 0.0, 0.0), Vec3::Z, Vec3::X).unwrap(), Vec3::new(1.0, 1.0, 1.0), feature).unwrap()
    }

    #[test]
    fn a_renamed_block_is_kept_face_for_face() {
        let b = cube("a");
        let h = derive(&[&b], &b, "new");
        assert_eq!(h.operands, 1);
        assert!(h.faces.iter().all(|(_, c, o)| *c == Change::Kept && o.len() == 1));
        assert!(h.deleted.is_empty());
    }

    #[test]
    fn ids_that_name_nothing_give_no_origin_and_no_panic() {
        let b = cube("a");
        let mut odd = b.clone();
        odd.faces[0].loops.push(LoopId::from_raw(9999));
        let h = derive(&[&b], &odd, "new");
        assert_eq!(h.faces.len(), 6);
        // the face with the stray loop is no longer the operand's face unchanged
        assert!(h.faces.iter().any(|(_, c, _)| *c == Change::Modified));
        let h = derive(&[&odd], &b, "new");
        assert_eq!(h.faces.len(), 6);
    }

    #[test]
    fn more_operands_than_an_origin_can_number_give_faces_without_origins() {
        let b = cube("a");
        let many: Vec<&Brep> = std::iter::repeat_n(&b, 300).collect();
        let h = derive(&many, &b, "new");
        assert_eq!(h.operands, u8::MAX);
        assert!(h.faces.iter().all(|(_, c, o)| *c == Change::Generated && o.is_empty()));
        let h = derive(&many[..255], &b, "new");
        assert_eq!(h.operands, 255);
    }
}
