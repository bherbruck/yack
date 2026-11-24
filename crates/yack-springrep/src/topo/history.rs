// yack: what an operation did to its operands' faces (PLAN §4.1 `History`), worked out from the face names.
//! A [`History`] says, for every face of an operation's result, how it came about ([`Change`]) and
//! from what ([`FaceOrigin`]); and which of the operands' faces no result face descends from.
//!
//! [`derive`] reads it off the face names every operation keeps (see [`FaceName`]): a face that
//! keeps an operand face's names continues that face, and one named by the operation's own
//! feature is new, its name saying from what. A name that more than one operand face carries is
//! settled by the surface; a face it cannot settle gets no origin, never a guess.

use std::collections::{BTreeMap, BTreeSet};

use super::{Brep, EdgeId, FaceId, FaceName, VertexId};

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

/// Whether result face `r` of `result` is operand face `o` of `operand` unchanged: the same surface
/// the same way round, bounded by as many loops of as many edges each.
fn unchanged(result: &Brep, r: FaceId, operand: &Brep, o: FaceId) -> bool {
    let (a, b) = (result.face(r), operand.face(o));
    a.surface == b.surface
        && a.reversed == b.reversed
        && a.loops.len() == b.loops.len()
        && a.loops.iter().zip(&b.loops).all(|(&x, &y)| result.lp(x).coedges.len() == operand.lp(y).coedges.len())
}

/// The history of an operation that made `result` from `inputs`, whose new faces carry the names of
/// its own `feature`.
pub fn derive(inputs: &[&Brep], result: &Brep, feature: &str) -> History {
    // every operand face carrying each name
    let mut carriers: BTreeMap<&FaceName, Vec<(u8, FaceId)>> = BTreeMap::new();
    for (k, b) in (0u8..).zip(inputs) {
        for f in b.face_ids() {
            for n in &b.face(f).names {
                carriers.entry(n).or_default().push((k, f));
            }
        }
    }
    let mut rows: Vec<Row> = Vec::new();
    for r in result.face_ids() {
        let face = result.face(r);
        let mut from: BTreeSet<(u8, FaceId)> = BTreeSet::new();
        let mut made: Vec<FaceOrigin> = Vec::new();
        for n in &face.names {
            if n.feature == feature {
                made.extend(generated(n));
                continue;
            }
            let Some(cands) = carriers.get(n) else { continue };
            let pick: Vec<(u8, FaceId)> = if cands.len() == 1 {
                cands.clone()
            } else {
                cands.iter().copied().filter(|&(k, f)| inputs.get(usize::from(k)).is_some_and(|b| b.face(f).surface == face.surface)).collect()
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
    let deleted = (0u8..)
        .zip(inputs)
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
