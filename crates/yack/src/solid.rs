// yack: `Solid`, the immutable B-rep value, and every operation and query on it (PLAN §4.3).
//
// M0.3: every operation is a stub returning `Unsupported`; queries on the empty solid are real.
// Arguments are named for the docs; until the kernel is wired they are unused.
#![allow(unused_variables)]

use std::sync::Arc;

use crate::{
    Aabb, Affine, Display, Drawing, EdgeId, EdgeRef, Error, FaceId, FaceRef, History, Interference, Issue, Mesh,
    Motion, Plane, Point3, Ray, Result, Rigid, Side, Similarity, Split, Tag, Vec3, VertexId, Angle,
};

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
#[derive(Debug)]
enum Body {
    Empty,
}

fn unsupported<T>(op: &'static str) -> Result<T> {
    Err(Error::unsupported(op))
}

impl Solid {
    /// The solid with no pieces.
    pub(crate) fn empty() -> Solid {
        Solid(Arc::new(Inner { body: Body::Empty, history: History::default(), tag: None }))
    }

    /// The stamp that ids of this solid carry (none for the empty solid).
    fn stamp(&self) -> Option<u64> {
        match self.0.body {
            Body::Empty => None,
        }
    }

    fn own_face(&self, op: &'static str, arg: &'static str, id: FaceId) -> Result<()> {
        match self.stamp() {
            Some(s) if s == id.stamp() => Ok(()),
            _ => Err(Error::foreign_id(op, arg)),
        }
    }

    fn own_edge(&self, op: &'static str, arg: &'static str, id: EdgeId) -> Result<()> {
        match self.stamp() {
            Some(s) if s == id.stamp() => Ok(()),
            _ => Err(Error::foreign_id(op, arg)),
        }
    }

    // ---- booleans ----

    /// Everything in either solid.
    pub fn union(&self, other: &Solid) -> Result<Solid> {
        unsupported("union")
    }
    /// Everything in both solids.
    pub fn intersection(&self, other: &Solid) -> Result<Solid> {
        unsupported("intersection")
    }
    /// This solid less `tool`.
    pub fn difference(&self, tool: &Solid) -> Result<Solid> {
        unsupported("difference")
    }
    /// The union of all `solids`.
    pub fn union_all(solids: &[Solid]) -> Result<Solid> {
        unsupported("union_all")
    }
    /// This solid less every tool.
    pub fn difference_all(&self, tools: &[Solid]) -> Result<Solid> {
        unsupported("difference_all")
    }
    /// The parts on each side of `plane`; `front` is on the normal's side.
    pub fn split(&self, plane: &Plane) -> Result<Split> {
        unsupported("split")
    }

    // ---- blends ----

    /// Rounds `edges` with a constant `radius` (mm).
    pub fn fillet(&self, edges: &[EdgeId], radius: f64) -> Result<Solid> {
        unsupported("fillet")
    }
    /// Rounds each edge with its own radius.
    pub fn fillet_radii(&self, edges: &[(EdgeId, f64)]) -> Result<Solid> {
        unsupported("fillet_radii")
    }
    /// Rounds `edge` with a radius running linearly from `r_from` at the end nearest `from` to
    /// `r_to` at the other.
    pub fn fillet_variable(&self, edge: EdgeId, from: impl Into<Point3>, r_from: f64, r_to: f64) -> Result<Solid> {
        unsupported("fillet_variable")
    }
    /// A conic blend with `setback` along each face and shape `rho` (0.5 is parabolic).
    pub fn fillet_conic(&self, edges: &[EdgeId], setback: f64, rho: f64) -> Result<Solid> {
        unsupported("fillet_conic")
    }
    /// A curvature-continuous blend `offset` from the edge, with `center` controlling its depth.
    pub fn fillet_g2(&self, edges: &[EdgeId], offset: f64, center: f64) -> Result<Solid> {
        unsupported("fillet_g2")
    }
    /// A fillet of `radius` with corner setbacks of `setback` where edges meet.
    pub fn fillet_setback(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid> {
        unsupported("fillet_setback")
    }
    /// As [`fillet_setback`](Solid::fillet_setback) with curvature-continuous corner patches.
    pub fn fillet_setback_g2(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid> {
        unsupported("fillet_setback_g2")
    }
    /// A fillet of `radius` between faces `a` and `b`.
    pub fn fillet_faces(&self, a: FaceId, b: FaceId, radius: f64) -> Result<Solid> {
        unsupported("fillet_faces")
    }
    /// Replaces `middle` with a round tangent to `side_a` and `side_b`.
    pub fn full_round(&self, side_a: FaceId, middle: FaceId, side_b: FaceId) -> Result<Solid> {
        unsupported("full_round")
    }
    /// Bevels `edges` by `distance`: a setback from the edge along each face, measured in the
    /// plane normal to the edge (never a face offset).
    pub fn chamfer(&self, edges: &[EdgeId], distance: f64) -> Result<Solid> {
        unsupported("chamfer")
    }
    /// Bevels `edge` with setback `d1` along `face` and `d2` along the other face.
    pub fn chamfer_two(&self, edge: EdgeId, face: FaceId, d1: f64, d2: f64) -> Result<Solid> {
        unsupported("chamfer_two")
    }
    /// Bevels each edge with its own setback distance.
    pub fn chamfer_distances(&self, edges: &[(EdgeId, f64)]) -> Result<Solid> {
        unsupported("chamfer_distances")
    }

    // ---- edits (signed distances: positive grows) ----

    /// Offsets every face by `distance` (positive grows).
    pub fn offset(&self, distance: f64) -> Result<Solid> {
        unsupported("offset")
    }
    /// Offsets `faces` by `distance` (positive grows), extending their neighbours.
    pub fn offset_faces(&self, faces: &[FaceId], distance: f64) -> Result<Solid> {
        unsupported("offset_faces")
    }
    /// Removes `faces` and heals the gap from the neighbours.
    pub fn delete_faces(&self, faces: &[FaceId]) -> Result<Solid> {
        unsupported("delete_faces")
    }
    /// Moves `faces` by the rigid motion `by`, extending their neighbours.
    pub fn move_faces(&self, faces: &[FaceId], by: &Rigid) -> Result<Solid> {
        unsupported("move_faces")
    }
    /// Thickens `faces` by `thickness` (positive grows outward) into a new solid.
    pub fn thicken(&self, faces: &[FaceId], thickness: f64) -> Result<Solid> {
        unsupported("thicken")
    }
    /// Hollows the solid to a wall of `thickness`, removing the `open` faces.
    pub fn shell(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        unsupported("shell")
    }
    /// Tilts `faces` by `angle` about the neutral plane through `origin` normal to `pull`.
    pub fn draft(&self, faces: &[FaceId], origin: impl Into<Point3>, pull: impl Into<Vec3>, angle: Angle) -> Result<Solid> {
        unsupported("draft")
    }
    /// The image under a similarity (rigid, uniform scale, mirror).
    pub fn transform(&self, m: &Similarity) -> Result<Solid> {
        unsupported("transform")
    }
    /// The image under an invertible affine map (protocol `affine`).
    pub fn transform_affine(&self, m: &Affine) -> Result<Solid> {
        unsupported("transform_affine")
    }
    /// Every point the solid passes through during `motion`.
    pub fn swept(&self, motion: &Motion) -> Result<Solid> {
        unsupported("swept")
    }

    // ---- selection and topology ----

    /// Every face, in a stable order.
    pub fn faces(&self) -> Vec<FaceId> {
        match self.0.body {
            Body::Empty => Vec::new(),
        }
    }
    /// Every edge, in a stable order.
    pub fn edges(&self) -> Vec<EdgeId> {
        match self.0.body {
            Body::Empty => Vec::new(),
        }
    }
    /// Every vertex, in a stable order.
    pub fn vertices(&self) -> Vec<VertexId> {
        match self.0.body {
            Body::Empty => Vec::new(),
        }
    }
    /// The one face within `tol::PICK` of `p`. `NotFound` if none, `Ambiguous` if several.
    pub fn face_at(&self, p: impl Into<Point3>) -> Result<FaceId> {
        let p = p.into();
        match self.0.body {
            Body::Empty => Err(Error::not_found("face_at", "p", format!("no face at ({}, {}, {})", p.x, p.y, p.z))),
        }
    }
    /// The one edge within `tol::PICK` of `p`. `NotFound` if none, `Ambiguous` if several.
    pub fn edge_at(&self, p: impl Into<Point3>) -> Result<EdgeId> {
        let p = p.into();
        match self.0.body {
            Body::Empty => Err(Error::not_found("edge_at", "p", format!("no edge at ({}, {}, {})", p.x, p.y, p.z))),
        }
    }
    /// The faces for which `keep` is true.
    pub fn faces_where(&self, keep: impl Fn(&FaceRef) -> bool) -> Vec<FaceId> {
        match self.0.body {
            Body::Empty => Vec::new(),
        }
    }
    /// The edges for which `keep` is true.
    pub fn edges_where(&self, keep: impl Fn(&EdgeRef) -> bool) -> Vec<EdgeId> {
        match self.0.body {
            Body::Empty => Vec::new(),
        }
    }
    /// A view of face `id`.
    pub fn face(&self, id: FaceId) -> Result<FaceRef<'_>> {
        self.own_face("face", "id", id)?;
        unsupported("face")
    }
    /// A view of edge `id`.
    pub fn edge(&self, id: EdgeId) -> Result<EdgeRef<'_>> {
        self.own_edge("edge", "id", id)?;
        unsupported("edge")
    }
    /// The edges bounding face `f`.
    pub fn edges_of(&self, f: FaceId) -> Result<Vec<EdgeId>> {
        self.own_face("edges_of", "f", f)?;
        unsupported("edges_of")
    }
    /// The faces meeting at edge `e`.
    pub fn faces_of(&self, e: EdgeId) -> Result<Vec<FaceId>> {
        self.own_edge("faces_of", "e", e)?;
        unsupported("faces_of")
    }
    /// The edges shared by faces `a` and `b`.
    pub fn edges_between(&self, a: FaceId, b: FaceId) -> Result<Vec<EdgeId>> {
        self.own_face("edges_between", "a", a)?;
        self.own_face("edges_between", "b", b)?;
        unsupported("edges_between")
    }
    /// How this solid's faces came from its inputs.
    pub fn history(&self) -> &History {
        &self.0.history
    }
    /// A content digest: equal solids (same topology and geometry, bit for bit) have equal digests.
    pub fn digest(&self) -> [u8; 32] {
        match self.0.body {
            Body::Empty => [0; 32],
        }
    }
    /// Whether the solid has no pieces.
    pub fn is_empty(&self) -> bool {
        match self.0.body {
            Body::Empty => true,
        }
    }
    /// This solid with `tag` recorded as the feature name of every face it made.
    pub fn named(&self, tag: impl Into<Tag>) -> Solid {
        let inner = Inner { body: Body::Empty, history: self.0.history.clone(), tag: Some(tag.into()) };
        match self.0.body {
            Body::Empty => Solid(Arc::new(inner)),
        }
    }
    /// The feature name given with [`named`](Solid::named), if any.
    pub fn tag(&self) -> Option<&Tag> {
        self.0.tag.as_ref()
    }
    /// Which strategy built this solid (PLAN §3.5); `"empty"` for the empty solid.
    pub fn strategy(&self) -> &'static str {
        match self.0.body {
            Body::Empty => "empty",
        }
    }

    // ---- queries ----

    /// The volume (mm³).
    pub fn volume(&self) -> f64 {
        match self.0.body {
            Body::Empty => 0.0,
        }
    }
    /// The surface area (mm²).
    pub fn area(&self) -> f64 {
        match self.0.body {
            Body::Empty => 0.0,
        }
    }
    /// The volume centroid; `None` for the empty solid.
    pub fn centroid(&self) -> Option<Point3> {
        match self.0.body {
            Body::Empty => None,
        }
    }
    /// The inertia tensor about the centroid, unit density (mm⁵).
    pub fn inertia(&self) -> [[f64; 3]; 3] {
        match self.0.body {
            Body::Empty => [[0.0; 3]; 3],
        }
    }
    /// The bounding box; `None` for the empty solid.
    pub fn bounding_box(&self) -> Option<Aabb> {
        match self.0.body {
            Body::Empty => None,
        }
    }
    /// Betti numbers: pieces, tunnels, voids.
    pub fn betti(&self) -> [u64; 3] {
        match self.0.body {
            Body::Empty => [0; 3],
        }
    }
    /// Validity problems; empty for anything yack built.
    pub fn check(&self) -> Vec<Issue> {
        match self.0.body {
            Body::Empty => Vec::new(),
        }
    }
    /// Where each point lies: `In`, `On` (within `tol::LINEAR`) or `Out`.
    pub fn classify<P: Into<Point3> + Copy>(&self, pts: &[P]) -> Vec<Side> {
        match self.0.body {
            Body::Empty => vec![Side::Out; pts.len()],
        }
    }
    /// The closest boundary point to each point; empty for the empty solid.
    pub fn closest<P: Into<Point3> + Copy>(&self, pts: &[P]) -> Vec<Point3> {
        match self.0.body {
            Body::Empty => Vec::new(),
        }
    }
    /// The outward unit normal at each boundary point; `None` at a crease or off the boundary.
    pub fn normal_at<P: Into<Point3> + Copy>(&self, pts: &[P]) -> Vec<Option<Vec3>> {
        match self.0.body {
            Body::Empty => vec![None; pts.len()],
        }
    }
    /// The principal curvatures at each boundary point; `None` at a crease or off the boundary.
    pub fn curvature_at<P: Into<Point3> + Copy>(&self, pts: &[P]) -> Vec<Option<[f64; 2]>> {
        match self.0.body {
            Body::Empty => vec![None; pts.len()],
        }
    }
    /// The least distance between the two solids; zero if they touch or overlap, infinite if
    /// either is empty.
    pub fn distance(&self, other: &Solid) -> f64 {
        match (&self.0.body, &other.0.body) {
            (Body::Empty, _) => f64::INFINITY,
        }
    }
    /// The volume shared with `other` and the gap between them.
    pub fn interference(&self, other: &Solid) -> Result<Interference> {
        match (&self.0.body, &other.0.body) {
            (Body::Empty, _) => Ok(Interference { volume: 0.0, gap: f64::INFINITY }),
        }
    }
    /// The area of the solid's cross-section by `plane` (mm²).
    pub fn section_area(&self, plane: &Plane) -> Result<f64> {
        match self.0.body {
            Body::Empty => Ok(0.0),
        }
    }
    /// The cross-section by `plane` as closed polylines with chord error at most `chord`.
    pub fn section(&self, plane: &Plane, chord: f64) -> Result<Vec<Vec<Point3>>> {
        match self.0.body {
            Body::Empty => Ok(Vec::new()),
        }
    }
    /// The visible and hidden edges looking along `direction`, in the protocol's view axes.
    pub fn project(&self, direction: impl Into<Vec3>, chord: f64) -> Result<Drawing> {
        match self.0.body {
            Body::Empty => Ok(Drawing::default()),
        }
    }
    /// For each ray, the sorted distances at which it crosses the boundary.
    pub fn raycast(&self, rays: &[Ray]) -> Result<Vec<Vec<f64>>> {
        match self.0.body {
            Body::Empty => Ok(vec![Vec::new(); rays.len()]),
        }
    }
    /// A closed, outward, welded triangle mesh with chord error at most `chord`.
    pub fn mesh(&self, chord: f64) -> Result<Mesh> {
        match self.0.body {
            Body::Empty => Ok(Mesh::default()),
        }
    }
    /// Per-face triangles with exact normals, and crease edges as polylines.
    pub fn display(&self, chord: f64) -> Result<Display> {
        match self.0.body {
            Body::Empty => Ok(Display::default()),
        }
    }
}
