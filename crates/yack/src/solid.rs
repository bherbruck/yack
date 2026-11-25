// yack: `Solid`, the immutable B-rep value (PLAN §4.2 rule 1). Its operations live in boolean.rs,
// blend.rs, edit.rs, select.rs and query.rs.
use std::sync::{Arc, OnceLock};
use std::sync::atomic::{AtomicU64, Ordering};

use crate::engine::{self, Brep};
use crate::engine_cadrs::{CadrsRef, Held};
use crate::{EdgeId, Error, ErrorKind, FaceId, Result, Tag, VertexId};
use yack_topo::FaceName;

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
    /// What the operation that made this solid did to its operands' faces.
    history: crate::History,
}

/// What a solid holds. Engines add variants as they are wired in.
#[derive(Clone)]
pub(crate) enum Body {
    Empty,
    Springrep(Box<Brep>),
    /// A cadrs answer the bridge cannot carry to springrep, held in cadrs form (the lazy bridge):
    /// valid by cadrs's full check; measured, written and operated on by cadrs; a springrep rung
    /// or query on it declines (`Unsupported`, naming why).
    Cadrs(Box<Held>),
}




impl Solid {
    fn new(body: Body) -> Solid {
        Solid(Arc::new(Inner {
            stamp: fresh_stamp(),
            body,
            tag: None,
            feature: None,
            strategy: None,
            cadrs: OnceLock::new(),
            suspect: None,
            approximate: false,
            history: crate::History::default(),
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

    /// A cadrs-held answer (no springrep form) as a solid.
    pub(crate) fn from_held(h: Held) -> Solid {
        Solid::new(Body::Cadrs(Box::new(h)))
    }

    /// The springrep body, if the solid has one: `None` for the empty solid and for a cadrs-held
    /// one (see [`Solid::held`]); callers that mean "empty" ask [`Solid::is_empty`].
    pub(crate) fn brep(&self) -> Option<&Brep> {
        match &self.0.body {
            Body::Empty | Body::Cadrs(_) => None,
            Body::Springrep(b) => Some(b.as_ref()),
        }
    }

    /// The cadrs-held body, if this solid is one.
    pub(crate) fn held(&self) -> Option<&Held> {
        match &self.0.body {
            Body::Cadrs(h) => Some(h),
            _ => None,
        }
    }

    /// Each face's names, by face index (the cadrs and springrep forms number faces alike).
    pub(crate) fn face_names(&self) -> Vec<std::collections::BTreeSet<FaceName>> {
        match &self.0.body {
            Body::Empty => Vec::new(),
            Body::Springrep(b) => b.face_ids().map(|f| b.face(f).names.clone()).collect(),
            Body::Cadrs(h) => h.names.clone(),
        }
    }

    /// The springrep form for a query that is not a ladder rung: `None` for the empty solid; an
    /// error when the solid has no springrep form (never an empty answer in its place).
    pub(crate) fn form(&self, op: &'static str) -> Result<Option<&Brep>> {
        match &self.0.body {
            Body::Empty => Ok(None),
            _ => self.need(op).map(Some),
        }
    }

    /// The springrep body: `Invalid` naming `op` for the empty solid, `Unsupported` for a
    /// cadrs-held one (a springrep rung declines it; the cadrs rung takes it).
    pub(crate) fn need(&self, op: &'static str) -> Result<&Brep> {
        match &self.0.body {
            Body::Empty => Err(Error::invalid(op, "self", "the solid is empty")),
            Body::Springrep(b) => Ok(b.as_ref()),
            Body::Cadrs(h) => Err(Error::new(ErrorKind::Unsupported, op, format!("this solid is held in cadrs form only ({})", h.why))),
        }
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

    /// A content digest of topology, geometry and face names: equal solids, bit for bit, have equal
    /// digests, and running the same program again gives the same digest. The empty solid's is all zeros.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // the same program gives the same solid, bit for bit
    /// let a = block(&Frame::WORLD, [1.0, 2.0, 3.0])?;
    /// let b = block(&Frame::WORLD, [1.0, 2.0, 3.0])?;
    /// assert_eq!(a.digest(), b.digest());
    /// assert_ne!(a.digest(), block(&Frame::WORLD, [1.0, 2.0, 4.0])?.digest());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn digest(&self) -> [u8; 32] {
        match &self.0.body {
            Body::Empty => [0; 32],
            Body::Springrep(b) => engine::digest(&yack_codec::to_bytes(core::slice::from_ref(b.as_ref()))),
            Body::Cadrs(h) => crate::engine_cadrs::held_digest(h, true),
        }
    }

    /// Whether the solid has no pieces.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let a = block(&Frame::WORLD, [1.0, 1.0, 1.0])?;
    /// assert!(!a.is_empty());
    /// assert!(a.intersection(&block(&Frame::at([5.0, 0.0, 0.0]), [1.0, 1.0, 1.0])?)?.is_empty());
    /// # Ok::<(), yack::Error>(())
    /// ```
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
                let short = |d: [u8; 32]| format!("yack#{}", d.iter().take(8).map(|x| format!("{x:02x}")).collect::<String>());
                if feature == engine::PROVISIONAL
                    && let Body::Springrep(b) = &mut inner.body
                {
                    name = short(bare_digest(b));
                    b.rename_faces(|n| {
                        let mut n = n.clone();
                        if n.feature == engine::PROVISIONAL {
                            n.feature.clone_from(&name);
                        }
                        Some(n)
                    });
                }
                if feature == engine::PROVISIONAL
                    && let Body::Cadrs(h) = &mut inner.body
                {
                    name = short(crate::engine_cadrs::held_digest(h, false));
                    rename_held(h, engine::PROVISIONAL, &name);
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
            tag: i.tag.clone(),
            feature: i.feature.clone(),
            strategy: i.strategy,
            cadrs: i.cadrs.clone(),
            suspect: i.suspect.clone(),
            approximate: true,
            history: i.history.clone(),
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

    /// What the operation that made this solid did to its operands' faces: for each face, how it
    /// came about and from what (see [`History`](crate::History)). A solid an operation passed
    /// through unchanged (a union with the empty solid) keeps its own.
    ///
    /// (Its example is on [`History`](crate::History).)
    pub fn history(&self) -> &crate::History {
        &self.0.history
    }

    /// The same solid with the history of the operation that made it from `inputs` (`self` first
    /// where the operation is a method), read off its faces by the engine. A solid shared with
    /// another (an input passed through) is left as it is.
    pub(crate) fn with_history(self, inputs: &[&Solid]) -> Solid {
        self.with_blend_history(inputs, &[])
    }

    /// [`with_history`](Solid::with_history) for a blend of `edges` of `inputs[0]`, in the order
    /// the blend took them (its faces' names count them).
    pub(crate) fn with_blend_history(self, inputs: &[&Solid], edges: &[EdgeId]) -> Solid {
        let empty = Brep::new();
        // the empty solid is an operand with no faces; a cadrs-held one has no springrep form yet
        let forms: Option<Vec<&Brep>> = inputs.iter().map(|s| match s.body() {
            Body::Springrep(b) => Some(b.as_ref()),
            Body::Empty => Some(&empty),
            Body::Cadrs(_) => None,
        }).collect();
        let result = match self.body() {
            Body::Springrep(b) => Some(b.as_ref()),
            Body::Empty => Some(&empty),
            Body::Cadrs(_) => None,
        };
        let edges: Vec<engine_ids::Edge> = match inputs.first() {
            Some(first) => edges.iter().filter_map(|&e| first.sr_edge("history", "edges", e).ok()).collect(),
            None => Vec::new(),
        };
        let feature = self.0.feature.as_deref().unwrap_or("");
        let h = match (forms, result) {
            (Some(forms), Some(result)) => yack_topo::history::derive(&forms, result, feature, &edges),
            // a cadrs-held solid among them: from the face names alone
            _ => {
                let names: Vec<Vec<std::collections::BTreeSet<FaceName>>> = inputs.iter().map(|s| s.face_names()).collect();
                let refs: Vec<&[std::collections::BTreeSet<FaceName>]> = names.iter().map(Vec::as_slice).collect();
                yack_topo::history::derive_names(&refs, &self.face_names(), feature, &edges)
            }
        };
        let h = crate::history::carried(&h, &self, inputs);
        self.with_carried(h)
    }

    /// The same solid with the history of a map of `input` that moved every face and kept their
    /// order: face `i` from `input`'s face `i`, modified. Kept only where the faces' names show the
    /// order held (else read off the names as any operation's).
    pub(crate) fn with_moved_history(self, input: &Solid) -> Solid {
        let names = self.face_names();
        if names.is_empty() || names != input.face_names() {
            return self.with_history(&[input]);
        }
        let faces = (0..names.len())
            .filter_map(|i| u32::try_from(i).ok())
            .map(|i| {
                let from = crate::FaceOrigin::Operand { operand: 0, face: FaceId::new(input.stamp(), i) };
                (FaceId::new(self.stamp(), i), crate::Change::Modified, vec![from])
            })
            .collect();
        let h = crate::History::from_rows(1, faces, Vec::new());
        self.with_carried(h)
    }

    /// The same solid with the history of a read (a file's or an encoding's): every face new, the
    /// `n`-th face read.
    pub(crate) fn with_read_history(self) -> Solid {
        match self.body() {
            Body::Springrep(b) => {
                let h = crate::history::carried(&yack_topo::history::read(b), &self, &[]);
                self.with_carried(h)
            }
            _ => self,
        }
    }

    /// The same solid with its history taken back through `inner`'s, the solid its operation took
    /// as operand 0 on the way (see [`History::compose`](crate::History::compose)).
    pub(crate) fn through(self, inner: &Solid) -> Solid {
        let h = self.history().compose(0, inner.history());
        self.with_carried(h)
    }

    /// The same solid holding history `h` (a shared one left as it is).
    fn with_carried(self, h: crate::History) -> Solid {
        match Arc::try_unwrap(self.0) {
            Ok(mut inner) => {
                inner.history = h;
                Solid(Arc::new(inner))
            }
            Err(shared) => Solid(shared),
        }
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
        if let Body::Cadrs(h) = &self.0.body {
            return Ok(&h.cadrs);
        }
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
            (Body::Cadrs(h), Some(f)) => {
                let mut h = (**h).clone();
                rename_held(&mut h, f, &name);
                Body::Cadrs(Box::new(h))
            }
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
        let named = Solid(Arc::new(Inner {
            stamp: fresh_stamp(),
            body,
            tag: Some(tag),
            feature: Some(name),
            strategy: self.0.strategy,
            cadrs: self.0.cadrs.clone(),
            suspect: None,
            approximate: self.0.approximate,
            history: crate::History::default(),
        }));
        match self.brep() {
            Some(b) => {
                let h = crate::history::carried(&yack_topo::history::identity(b), &named, &[self]);
                named.with_carried(h)
            }
            None => named,
        }
    }

    /// The feature name given with [`named`](Solid::named), if any.
    pub fn tag(&self) -> Option<&Tag> {
        self.0.tag.as_ref()
    }

    /// The engine whose form of this solid answers its queries (volume, area, mesh, selection,
    /// STEP's read-back): `"springrep"`, `"cadrs"` (a solid held in cadrs form), or `"empty"`.
    /// Derived from the body alone, so it never disagrees with what answers.
    pub fn strategy(&self) -> &'static str {
        match &self.0.body {
            Body::Empty => "empty",
            Body::Springrep(_) => "springrep",
            Body::Cadrs(_) => "cadrs",
        }
    }

    /// The ladder rung (engine) that built this solid, which may differ from the engine that
    /// answers its queries ([`Solid::strategy`]): a cadrs result carried to springrep is answered
    /// by springrep. For the ladder's own decisions and the bake-off's attribution.
    pub(crate) fn built_by(&self) -> &'static str {
        match (self.0.strategy, &self.0.body) {
            (_, Body::Empty) => "empty",
            (Some(name), _) => name,
            (None, _) => self.strategy(),
        }
    }
}

/// `h`'s face names of feature `from` renamed to feature `to`.
fn rename_held(h: &mut Held, from: &str, to: &str) {
    for set in &mut h.names {
        *set = core::mem::take(set)
            .into_iter()
            .map(|mut n| {
                if n.feature == from {
                    n.feature = to.to_string();
                }
                n
            })
            .collect();
    }
}

/// The digest of `b`'s topology and geometry alone, its face names stripped.
fn bare_digest(b: &Brep) -> [u8; 32] {
    let mut bare = b.clone();
    bare.rename_faces(|_| None::<yack_topo::FaceName>);
    engine::digest(&yack_codec::to_bytes(core::slice::from_ref(&bare)))
}

/// The engine's id types, named once.
pub(crate) mod engine_ids {
    pub(crate) use yack_topo::EdgeId as Edge;
    pub(crate) use yack_topo::FaceId as Face;
    pub(crate) use yack_topo::VertexId as Vertex;
}

