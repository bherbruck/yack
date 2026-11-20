// yack: `Solid`, the immutable B-rep value (PLAN §4.2 rule 1). Its operations live in boolean.rs,
// blend.rs, edit.rs, select.rs and query.rs.
use std::sync::{Arc, OnceLock};
use std::sync::atomic::{AtomicU64, Ordering};

use crate::engine::{self, Brep};
use crate::engine_cadrs::CadrsRef;
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
#[derive(Clone)]
pub struct Solid(Arc<Inner>);

impl core::fmt::Debug for Solid {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Solid")
            .field("strategy", &self.strategy())
            .field("faces", &self.faces().len())
            .field("tag", &self.0.tag)
            .finish_non_exhaustive()
    }
}

/// The next solid's stamp. Process-wide state, allowed in the facade only (lead, M1.5): it keeps
/// ids from one solid from ever passing on another, dropped or alive, and never affects geometry.
static NEXT_STAMP: AtomicU64 = AtomicU64::new(1);

/// A fresh process-wide number (solid stamps, per-operation feature names).
pub(crate) fn fresh_stamp() -> u64 {
    NEXT_STAMP.fetch_add(1, Ordering::Relaxed)
}

struct Inner {
    stamp: u64,
    body: Body,
    history: History,
    tag: Option<Tag>,
    /// The engine feature name on the faces the operation that made this solid created.
    feature: Option<String>,
    /// The ladder rung that built this solid (`"springrep"`, `"cadrs"`), if not its body's engine.
    strategy: Option<&'static str>,
    /// The same solid in the cadrs engine: set when cadrs built it, else converted on first use
    /// (the error kept when the bridge cannot carry it).
    cadrs: OnceLock<core::result::Result<CadrsRef, String>>,
    /// Why a guard doubts this result (springrep may have built it wrong): the ladder tries the
    /// next rung and keeps this answer only if every later rung fails.
    suspect: Option<String>,
    /// Whether this is only a faceted approximation of the solid (the ladder refuses one as an
    /// op's answer; [`Solid::is_exact`] reports it).
    approximate: bool,
}

/// What a solid holds. Engines add variants as they are wired in.
#[derive(Clone, Debug)]
pub(crate) enum Body {
    Empty,
    Springrep(Box<Brep>),
}

impl Solid {
    fn new(body: Body) -> Solid {
        Solid(Arc::new(Inner {
            stamp: fresh_stamp(),
            body,
            history: History::default(),
            tag: None,
            feature: None,
            strategy: None,
            cadrs: OnceLock::new(),
            suspect: None,
            approximate: false,
        }))
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

    /// The springrep form for a query that is not a ladder rung: `None` for the empty solid; an
    /// error when the solid has no springrep form (never an empty answer in its place).
    pub(crate) fn form(&self, op: &'static str) -> Result<Option<&Brep>> {
        let _ = op;
        Ok(self.brep())
    }

    /// The springrep body, or `Invalid` naming `op` for the empty solid.
    pub(crate) fn need(&self, op: &'static str) -> Result<&Brep> {
        self.brep().ok_or_else(|| Error::invalid(op, "self", "the solid is empty"))
    }

    /// The stamp this solid's ids carry: unique in the process.
    pub(crate) fn stamp(&self) -> u64 {
        self.0.stamp
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

    /// The engine vertex for `id`: `ForeignId` from another solid, `NotFound` if out of range.
    pub(crate) fn sr_vertex(&self, op: &'static str, arg: &'static str, id: VertexId) -> Result<engine_ids::Vertex> {
        if id.stamp() != self.stamp() {
            return Err(Error::foreign_id(op, arg));
        }
        let b = self.need(op)?;
        b.vertex_ids().nth(id.index() as usize).ok_or_else(|| Error::not_found(op, arg, format!("no {id}")))
    }

    pub(crate) fn sr_edges(&self, op: &'static str, arg: &'static str, ids: &[EdgeId]) -> Result<Vec<engine_ids::Edge>> {
        ids.iter().map(|&e| self.sr_edge(op, arg, e)).collect()
    }

    /// How this solid's faces came from its inputs.
    pub fn history(&self) -> &History {
        &self.0.history
    }

    /// A content digest of topology, geometry and face names: equal solids, bit for bit, have equal
    /// digests, and running the same program again gives the same digest. The empty solid's is all zeros.
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

    /// The same solid remembering that its new faces carry engine feature `feature`.
    /// The same solid remembering that its new faces carry `feature`. An op's provisional name
    /// ([`engine::feature`]) is replaced here by one derived from the result's own topology and
    /// geometry, so a program names its faces alike on every run, whatever ran before it.
    pub(crate) fn with_feature(self, feature: &str) -> Solid {
        match Arc::try_unwrap(self.0) {
            Ok(mut inner) => {
                let mut name = feature.to_string();
                if feature == engine::PROVISIONAL
                    && let Body::Springrep(b) = &mut inner.body
                {
                    let d = bare_digest(b);
                    name = format!("yack#{}", d.iter().take(8).map(|x| format!("{x:02x}")).collect::<String>());
                    b.rename_faces(|n| {
                        let mut n = n.clone();
                        if n.feature == engine::PROVISIONAL {
                            n.feature.clone_from(&name);
                        }
                        Some(n)
                    });
                }
                inner.feature = Some(name);
                Solid(Arc::new(inner))
            }
            Err(shared) => Solid(shared),
        }
    }

    /// The same solid marked as only a faceted approximation. No engine path returns one today
    /// (cadrs runs exact-only); kept for the ladder's refusal and its tests.
    #[cfg(test)]
    pub(crate) fn approximate(self) -> Solid {
        let i = &*self.0;
        Solid(Arc::new(Inner {
            stamp: i.stamp,
            body: i.body.clone(),
            history: i.history.clone(),
            tag: i.tag.clone(),
            feature: i.feature.clone(),
            strategy: i.strategy,
            cadrs: i.cadrs.clone(),
            suspect: i.suspect.clone(),
            approximate: true,
        }))
    }

    /// Whether the solid is exact rather than a faceted approximation. Every operation's answer
    /// is exact: an engine that finds a result only as facets declines, and when no engine does
    /// better the op refuses (`Failed`, "only a faceted approximation is available"). An imported
    /// mesh is flagged on the import alone, not on what later operations make from it (the
    /// readers do not report meshes yet, so today every solid reports true).
    pub fn is_exact(&self) -> bool {
        !self.0.approximate
    }

    /// The same solid flagged as doubted by a guard, for `why`.
    pub(crate) fn suspect(self, why: &str) -> Solid {
        match Arc::try_unwrap(self.0) {
            Ok(mut inner) => {
                inner.suspect = Some(why.to_string());
                Solid(Arc::new(inner))
            }
            Err(shared) => Solid(shared),
        }
    }

    /// The engine feature name the op that made this solid gave its new faces.
    pub(crate) fn feature_name(&self) -> Option<&str> {
        self.0.feature.as_deref()
    }

    /// Why a guard doubts this solid, if one does.
    pub(crate) fn suspected(&self) -> Option<&str> {
        self.0.suspect.as_deref()
    }

    /// The same solid, no longer doubted (the ladder keeps it as its answer).
    pub(crate) fn trusted(self) -> Solid {
        match Arc::try_unwrap(self.0) {
            Ok(mut inner) => {
                inner.suspect = None;
                Solid(Arc::new(inner))
            }
            Err(shared) => Solid(shared),
        }
    }

    /// The same solid recording that the ladder rung `name` built it.
    pub(crate) fn with_strategy(self, name: &'static str) -> Solid {
        match Arc::try_unwrap(self.0) {
            Ok(mut inner) => {
                inner.strategy = Some(name);
                Solid(Arc::new(inner))
            }
            Err(shared) => Solid(shared),
        }
    }

    /// The same solid holding its cadrs form.
    pub(crate) fn with_cadrs(self, c: CadrsRef) -> Solid {
        let _ = self.0.cadrs.set(Ok(c));
        self
    }

    /// This solid's cadrs form, converted through the bridge on first use.
    pub(crate) fn cadrs_form(&self, convert: impl FnOnce(&Brep) -> core::result::Result<CadrsRef, String>) -> core::result::Result<&CadrsRef, &str> {
        let r = self.0.cadrs.get_or_init(|| match self.brep() {
            Some(b) => convert(b),
            None => Err("the solid is empty".to_string()),
        });
        r.as_ref().map_err(String::as_str)
    }

    /// This solid with `tag` as the feature name of every face the operation that made it
    /// created (a constructor: all its faces; a fillet: its rounds; a boolean makes none). Later
    /// operations keep the name on those faces; find them with
    /// [`faces_tagged`](Solid::faces_tagged). The result is a new solid: ids from this one are
    /// foreign to it.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let r = b.fillet(&[b.edge_at([5.0, 0.0, 10.0])?], 2.0)?.named("round");
    /// let r = r.union(&block(&Frame::at([20.0, 0.0, 0.0]), [1.0, 1.0, 1.0])?)?;
    /// assert_eq!(r.faces_tagged("round").len(), 1);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn named(&self, tag: impl Into<Tag>) -> Solid {
        let tag = tag.into();
        let name = tag.as_str().to_string();
        let body = match (&self.0.body, &self.0.feature) {
            (Body::Springrep(b), Some(f)) => {
                let mut b = (**b).clone();
                b.rename_faces(|n| {
                    let mut n = n.clone();
                    if n.feature == *f {
                        n.feature.clone_from(&name);
                    }
                    Some(n)
                });
                Body::Springrep(Box::new(b))
            }
            (body, _) => body.clone(),
        };
        Solid(Arc::new(Inner {
            stamp: fresh_stamp(),
            body,
            history: self.0.history.clone(),
            tag: Some(tag),
            feature: Some(name),
            strategy: self.0.strategy,
            cadrs: self.0.cadrs.clone(),
            suspect: None,
            approximate: self.0.approximate,
        }))
    }

    /// The feature name given with [`named`](Solid::named), if any.
    pub fn tag(&self) -> Option<&Tag> {
        self.0.tag.as_ref()
    }

    /// Which ladder rung built this solid (PLAN §3.5): `"springrep"`, `"cadrs"`, or `"empty"`.
    /// The adapter and the bench runner log it per case.
    pub fn strategy(&self) -> &'static str {
        match (self.0.strategy, &self.0.body) {
            (_, Body::Empty) => "empty",
            (Some(name), _) => name,
            (None, Body::Springrep(_)) => "springrep",
        }
    }
}

/// The digest of `b`'s topology and geometry alone, its face names stripped.
fn bare_digest(b: &Brep) -> [u8; 32] {
    let mut bare = b.clone();
    bare.rename_faces(|_| None::<yack_springrep::topo::FaceName>);
    engine::digest(&yack_springrep::codec::to_bytes(core::slice::from_ref(&bare)))
}

/// The engine's id types, named once.
pub(crate) mod engine_ids {
    pub(crate) use yack_springrep::topo::{EdgeId as Edge, FaceId as Face, VertexId as Vertex};
}

