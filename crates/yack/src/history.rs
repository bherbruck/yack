// yack: `History`, what an operation did to its operands' faces (PLAN §4.1, M4.11): carried from the engine, never computed here.
use yack_springrep::topo::history as sr;

use crate::{EdgeId, FaceId, Solid, Tag, VertexId};

/// How a face of an operation's result came about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CapEnd {
    /// Where the sweep starts (on the profile's plane).
    Start,
    /// Where the sweep ends.
    End,
}

/// What a face of an operation's result came from. `operand` counts the operation's solids in
/// the order it takes them (`self` first).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum FaceOrigin {
    /// A face of an operand.
    Operand {
        /// Which operand.
        operand: u8,
        /// Its face.
        face: FaceId,
    },
    /// A profile segment's sweep (its label), or a sweep's end cap.
    Segment {
        /// The segment's label (or the part's own name for a primitive's face).
        label: Tag,
        /// Which cap, for a cap.
        end: Option<CapEnd>,
    },
    /// An operand edge: a fillet or chamfer face, a shell opening's rim wall.
    Edge {
        /// Which operand.
        operand: u8,
        /// Its edge.
        edge: EdgeId,
    },
    /// An operand vertex: a blend's corner patch.
    Vertex {
        /// Which operand.
        operand: u8,
        /// Its vertex.
        vertex: VertexId,
    },
    /// An operand face, offset: a shell's, an offset's or a thickening's face.
    Offset {
        /// Which operand.
        operand: u8,
        /// Its face.
        face: FaceId,
    },
    /// The `face`-th face of a file or an encoding read.
    Read {
        /// Its place in the file.
        face: u32,
    },
}

/// What the operation that made a solid did to its operands' faces: for each face of the
/// solid, how it came about and from what, and which operand faces no face descends from.
///
/// ```
/// use yack::prelude::*;
/// use yack::{Change, FaceOrigin};
/// let plate = block(&Frame::WORLD, [10.0, 10.0, 2.0])?;
/// let drilled = plate.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 4.0)?)?;
/// let h = drilled.history();
/// assert_eq!(h.operands(), 2);
/// // the top was drilled through: one face, modified from the plate's top
/// let top = drilled.face_at([1.0, 1.0, 2.0])?;
/// assert_eq!(h.change(top), Some(Change::Modified));
/// assert!(matches!(h.origins(top), [FaceOrigin::Operand { operand: 0, .. }]));
/// // the plate's top went to that one face
/// let FaceOrigin::Operand { face, .. } = h.origins(top)[0] else { unreachable!() };
/// assert_eq!(h.descendants(face), vec![top]);
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Debug, Default, PartialEq)]
pub struct History {
    operands: u8,
    faces: Vec<(FaceId, Change, Vec<FaceOrigin>)>,
    deleted: Vec<FaceId>,
}

impl History {
    /// How many solids the operation took (0 for one that made a solid from nothing).
    ///
    /// (Its example is on [`History`].)
    pub fn operands(&self) -> u8 {
        self.operands
    }

    /// How face `f` of the result came about; `None` for a face the history has no row for.
    ///
    /// (Its example is on [`History`].)
    pub fn change(&self, f: FaceId) -> Option<Change> {
        self.faces.iter().find(|(id, _, _)| *id == f).map(|(_, c, _)| *c)
    }

    /// What face `f` of the result came from (empty when the history cannot say).
    ///
    /// (Its example is on [`History`].)
    pub fn origins(&self, f: FaceId) -> &[FaceOrigin] {
        self.faces.iter().find(|(id, _, _)| *id == f).map_or(&[], |(_, _, o)| o.as_slice())
    }

    /// The operand faces no face of the result descends from.
    ///
    /// (Its example is on [`History`].)
    pub fn deleted(&self) -> &[FaceId] {
        &self.deleted
    }

    /// The faces of the result that operand face `face` became (none when it was deleted).
    ///
    /// (Its example is on [`History`].)
    pub fn descendants(&self, face: FaceId) -> Vec<FaceId> {
        self.faces
            .iter()
            .filter(|(_, _, o)| o.iter().any(|x| matches!(x, FaceOrigin::Operand { face: g, .. } | FaceOrigin::Offset { face: g, .. } if *g == face)))
            .map(|(id, _, _)| *id)
            .collect()
    }

    /// Every face the history has a row for, with its change and origins, by face id.
    ///
    /// (Its example is on [`History`].)
    pub fn rows(&self) -> impl Iterator<Item = (FaceId, Change, &[FaceOrigin])> + '_ {
        self.faces.iter().map(|(f, c, o)| (*f, *c, o.as_slice()))
    }
}

/// The engine's history of `result` made from `inputs`, in the solids' own ids.
pub(crate) fn carried(h: &sr::History, result: &Solid, inputs: &[&Solid]) -> History {
    let operand = |k: u8| inputs.get(usize::from(k)).copied();
    let origin = |o: &sr::FaceOrigin| -> Option<FaceOrigin> {
        Some(match o {
            sr::FaceOrigin::Operand { operand: k, face } => FaceOrigin::Operand { operand: *k, face: operand(*k)?.face_id(*face) },
            sr::FaceOrigin::Offset { operand: k, face } => FaceOrigin::Offset { operand: *k, face: operand(*k)?.face_id(*face) },
            sr::FaceOrigin::Edge { operand: k, edge } => FaceOrigin::Edge { operand: *k, edge: operand(*k)?.edge_id(*edge) },
            sr::FaceOrigin::Vertex { operand: k, vertex } => FaceOrigin::Vertex { operand: *k, vertex: operand(*k)?.vertex_id(*vertex) },
            sr::FaceOrigin::Segment { label, end } => FaceOrigin::Segment {
                label: Tag::from(label.as_str()),
                end: end.map(|e| match e {
                    sr::CapEnd::Start => CapEnd::Start,
                    sr::CapEnd::End => CapEnd::End,
                }),
            },
            sr::FaceOrigin::Read { face } => FaceOrigin::Read { face: *face },
        })
    };
    let change = |c: sr::Change| match c {
        sr::Change::Kept => Change::Kept,
        sr::Change::Modified => Change::Modified,
        sr::Change::Split => Change::Split,
        sr::Change::Merged => Change::Merged,
        sr::Change::Generated => Change::Generated,
    };
    History {
        operands: h.operands,
        faces: h.faces.iter().map(|(f, c, o)| (result.face_id(*f), change(*c), o.iter().filter_map(origin).collect())).collect(),
        deleted: h.deleted.iter().filter_map(|(k, f)| Some(operand(*k)?.face_id(*f))).collect(),
    }
}
