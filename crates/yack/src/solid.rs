// yack: `Solid`, the immutable B-rep value (PLAN §4.2 rule 1). Its operations live in boolean.rs,
// blend.rs, edit.rs, select.rs and query.rs.
use std::sync::Arc;

use crate::engine::{self, Brep};
use crate::{EdgeId, Error, FaceId, History, Result, Tag, VertexId};

/// A solid: an immutable, checked B-rep value, cheap to clone and share between threads.
///
/// A `Solid` is always valid or empty, and may be a set of pieces that touch. Make one with the
/// functions in [`make`](crate::make); every operation returns a new solid.
///
/// ```
/// use yack::prelude::*;
/// let nothing = empty();
/// assert!(nothing.is_empty());
/// assert_eq!(nothing.volume(), 0.0);
/// assert!(nothing.faces().is_empty());
/// ```
#[derive(Clone, Debug)]
pub struct Solid(Arc<Inner>);

#[derive(Debug)]
struct Inner {
    body: Body,
    history: History,
    tag: Option<Tag>,
}

/// What a solid holds. Engines add variants as they are wired in.
#[derive(Clone, Debug)]
pub(crate) enum Body {
    Empty,
    Springrep(Box<Brep>),
}

impl Solid {
    fn new(body: Body) -> Solid {
        Solid(Arc::new(Inner { body, history: History::default(), tag: None }))
    }

    /// The solid with no pieces.
    pub(crate) fn empty() -> Solid {
        Solid::new(Body::Empty)
    }

    /// A springrep result; one with no solids in it is the empty solid.
    pub(crate) fn from_springrep(b: Brep) -> Solid {
        if b.solid_ids().next().is_none() { Solid::empty() } else { Solid::new(Body::Springrep(Box::new(b))) }
    }

    pub(crate) fn body(&self) -> &Body {
        &self.0.body
    }

    /// The springrep body, if the solid is not empty.
    pub(crate) fn brep(&self) -> Option<&Brep> {
        match &self.0.body {
            Body::Empty => None,
            Body::Springrep(b) => Some(b.as_ref()),
        }
    }

    /// The springrep body, or `Invalid` naming `op` for the empty solid.
    pub(crate) fn need(&self, op: &'static str) -> Result<&Brep> {
        self.brep().ok_or_else(|| Error::invalid(op, "self", "the solid is empty"))
    }

    /// The stamp this solid's ids carry: unique among live solids.
    pub(crate) fn stamp(&self) -> u64 {
        Arc::as_ptr(&self.0) as usize as u64
    }

    pub(crate) fn face_id(&self, f: engine_ids::Face) -> FaceId {
        FaceId::new(self.stamp(), u32::try_from(f.index()).unwrap_or(u32::MAX))
    }

    pub(crate) fn edge_id(&self, e: engine_ids::Edge) -> EdgeId {
        EdgeId::new(self.stamp(), u32::try_from(e.index()).unwrap_or(u32::MAX))
    }

    pub(crate) fn vertex_id(&self, v: engine_ids::Vertex) -> VertexId {
        VertexId::new(self.stamp(), u32::try_from(v.index()).unwrap_or(u32::MAX))
    }

    /// The engine face for `id`: `ForeignId` from another solid, `NotFound` if out of range.
    pub(crate) fn sr_face(&self, op: &'static str, arg: &'static str, id: FaceId) -> Result<engine_ids::Face> {
        if id.stamp() != self.stamp() {
            return Err(Error::foreign_id(op, arg));
        }
        let b = self.need(op)?;
        b.face_ids().nth(id.index() as usize).ok_or_else(|| Error::not_found(op, arg, format!("no {id}")))
    }

    /// The engine edge for `id`: `ForeignId` from another solid, `NotFound` if out of range.
    pub(crate) fn sr_edge(&self, op: &'static str, arg: &'static str, id: EdgeId) -> Result<engine_ids::Edge> {
        if id.stamp() != self.stamp() {
            return Err(Error::foreign_id(op, arg));
        }
        let b = self.need(op)?;
        b.edge_ids().nth(id.index() as usize).ok_or_else(|| Error::not_found(op, arg, format!("no {id}")))
    }

    pub(crate) fn sr_edges(&self, op: &'static str, arg: &'static str, ids: &[EdgeId]) -> Result<Vec<engine_ids::Edge>> {
        ids.iter().map(|&e| self.sr_edge(op, arg, e)).collect()
    }

    /// How this solid's faces came from its inputs.
    pub fn history(&self) -> &History {
        &self.0.history
    }

    /// A content digest: equal solids (same topology and geometry, bit for bit) have equal
    /// digests. The empty solid's is all zeros.
    pub fn digest(&self) -> [u8; 32] {
        match &self.0.body {
            Body::Empty => [0; 32],
            Body::Springrep(b) => engine::digest(&yack_springrep::codec::to_bytes(core::slice::from_ref(b.as_ref()))),
        }
    }

    /// Whether the solid has no pieces.
    pub fn is_empty(&self) -> bool {
        matches!(self.0.body, Body::Empty)
    }

    /// This solid with `tag` recorded as its feature name. The result is a new solid: ids from
    /// this one are foreign to it.
    pub fn named(&self, tag: impl Into<Tag>) -> Solid {
        Solid(Arc::new(Inner { body: self.0.body.clone(), history: self.0.history.clone(), tag: Some(tag.into()) }))
    }

    /// The feature name given with [`named`](Solid::named), if any.
    pub fn tag(&self) -> Option<&Tag> {
        self.0.tag.as_ref()
    }

    /// Which strategy built this solid (PLAN §3.5): `"empty"` or the engine's name.
    pub fn strategy(&self) -> &'static str {
        match self.0.body {
            Body::Empty => "empty",
            Body::Springrep(_) => "springrep",
        }
    }
}

/// The engine's id types, named once.
pub(crate) mod engine_ids {
    pub(crate) use yack_springrep::topo::{EdgeId as Edge, FaceId as Face, VertexId as Vertex};
}
