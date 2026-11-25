// yack: selection and topology walks on `Solid` (PLAN §4.2 rule 6), routed to springrep.
use std::collections::BTreeSet;

use yack_geom as geom;
use yack_measure as measure;

use crate::engine::{self, Brep};
use crate::solid::engine_ids;
use crate::{
    CurveKind, EdgeId, EdgeRef, Error, FaceId, FaceRef, Point3, Result, Solid, Role, SurfaceKind, Tag, VertexId, VertexRef,
};

/// The tags and roles a face's names give it (internal feature names left out).
fn tags_roles(names: &BTreeSet<yack_topo::FaceName>) -> (Vec<Tag>, Vec<(Tag, Role)>) {
    let tags: BTreeSet<&str> = names.iter().map(|n| n.feature.as_str()).filter(|t| !engine::is_internal_feature(t)).collect();
    let roles = names
        .iter()
        .filter(|n| !engine::is_internal_feature(&n.feature))
        .filter_map(|n| Some((Tag::from(n.feature.as_str()), engine::role(&n.face)?)))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    (tags.into_iter().map(Tag::from).collect(), roles)
}

/// A view of face `i` of a solid held in cadrs form.
fn held_face_ref<'a>(s: &Solid, h: &'a crate::engine_cadrs::Held, i: u64, op: &'static str) -> Result<FaceRef<'a>> {
    let f = crate::engine_cadrs::held_face_facts(op, h, i)?;
    let (tags, roles) = usize::try_from(i).ok().and_then(|n| h.names.get(n)).map(tags_roles).unwrap_or_default();
    Ok(FaceRef {
        id: FaceId::new(s.stamp(), u32::try_from(i).unwrap_or(u32::MAX)),
        kind: f.kind,
        normal: f.normal,
        area: std::sync::OnceLock::from(f.area),
        centroid: std::sync::OnceLock::from(f.centroid),
        bbox: std::sync::OnceLock::from(f.bbox),
        tags,
        roles,
        at: crate::values::FaceAt::Cadrs(h, i),
    })
}

/// A view of edge `i` of a solid held in cadrs form.
fn held_edge_ref<'a>(s: &Solid, h: &'a crate::engine_cadrs::Held, i: u64, op: &'static str) -> Result<EdgeRef<'a>> {
    let e = crate::engine_cadrs::held_edge_facts(op, h, i)?;
    Ok(EdgeRef {
        id: EdgeId::new(s.stamp(), u32::try_from(i).unwrap_or(u32::MAX)),
        kind: e.kind,
        ends: e.ends,
        convex: e.convex,
        length: e.length,
        midpoint: e.midpoint,
        uses: e.uses,
        at: crate::values::EdgeAt::Cadrs(h, i),
    })
}

/// A pick's answer as an id or the refusal `face_at`/`edge_at` give.
fn picked<T>(op: &'static str, what: &str, p: Point3, found: crate::engine_cadrs::Picked, id: impl Fn(u64) -> T) -> Result<T> {
    use crate::engine_cadrs::Picked as P;
    match found {
        P::One(i) => Ok(id(i)),
        P::None => Err(Error::not_found(op, "p", format!("no {what} passes within tol::PICK of ({}, {}, {})", p.x, p.y, p.z))),
        P::Ambiguous(n) => Err(Error::ambiguous(
            op,
            "p",
            format!("{n} {what}s lie within tol::PICK of ({}, {}, {}) at distances that differ by less than tol::PICK", p.x, p.y, p.z),
        )),
    }
}

fn face_ref<'a>(s: &Solid, b: &'a Brep, f: engine_ids::Face) -> FaceRef<'a> {
    let face = b.face(f);
    let outward = |z: yack_math::kernel::Vec3| if face.reversed { engine::vec(-z) } else { engine::vec(z) };
    let (kind, normal) = match &face.surface {
        geom::Surface::Plane(p) => {
            let n = outward(p.frame.z);
            (SurfaceKind::Plane { normal: n }, Some(n))
        }
        geom::Surface::Cylinder(c) => (SurfaceKind::Cylinder { axis: engine::vec(c.frame.z), radius: c.radius }, None),
        geom::Surface::Cone(_) => (SurfaceKind::Cone, None),
        geom::Surface::Sphere(sp) => (SurfaceKind::Sphere { radius: sp.radius }, None),
        geom::Surface::Torus(_) => (SurfaceKind::Torus, None),
        geom::Surface::Extrusion(_) | geom::Surface::Revolution(_) | geom::Surface::Ruled(_) => (SurfaceKind::Swept, None),
        geom::Surface::Nurbs(_) => (SurfaceKind::Spline, None),
    };
    let (tags, roles) = tags_roles(&face.names);
    FaceRef {
        id: s.face_id(f),
        kind,
        normal,
        area: std::sync::OnceLock::new(),
        centroid: std::sync::OnceLock::new(),
        bbox: std::sync::OnceLock::new(),
        tags,
        roles,
        at: crate::values::FaceAt::Springrep(b, f),
    }
}

fn edge_ref<'a>(s: &Solid, b: &'a Brep, e: engine_ids::Edge) -> Option<EdgeRef<'a>> {
    let edge = b.edge(e);
    let curve = edge.curve.as_ref()?;
    let kind = match curve {
        geom::Curve::Line(l) => CurveKind::Line { direction: engine::vec(l.dir) },
        geom::Curve::Circle(c) => CurveKind::Circle { radius: c.radius, axis: engine::vec(c.frame.z) },
        geom::Curve::Ellipse(_) => CurveKind::Ellipse,
        _ => CurveKind::Spline,
    };
    let ends = [engine::point(b.vertex(edge.start).point), engine::point(b.vertex(edge.end).point)];
    let mid = engine::point(curve.eval(yack_math::mean(edge.range.0, edge.range.1)));
    Some(EdgeRef {
        id: s.edge_id(e),
        kind,
        ends,
        convex: yack_topo::local::convexity(b, e),
        length: measure::edge_length(b, e),
        midpoint: mid,
        uses: b.edge_uses(e).len(),
        at: crate::values::EdgeAt::Springrep(b, e),
    })
}

impl Solid {
    /// Every face, in a stable order.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.faces().len(), 6);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn faces(&self) -> Vec<FaceId> {
        if let Some(h) = self.held() {
            let (n, _, _) = crate::engine_cadrs::held_counts(h);
            return (0..n).filter_map(|i| u32::try_from(i).ok()).map(|i| FaceId::new(self.stamp(), i)).collect();
        }
        self.brep().map_or_else(Vec::new, |b| b.face_ids().map(|f| self.face_id(f)).collect())
    }

    /// Every edge (poles aside), in a stable order.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.edges().len(), 12);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn edges(&self) -> Vec<EdgeId> {
        if let Some(h) = self.held() {
            let (_, edges, _) = crate::engine_cadrs::held_counts(h);
            return edges.into_iter().filter_map(|i| u32::try_from(i).ok()).map(|i| EdgeId::new(self.stamp(), i)).collect();
        }
        self.brep().map_or_else(Vec::new, |b| {
            b.edge_ids().filter(|&e| !b.edge(e).is_degenerate()).map(|e| self.edge_id(e)).collect()
        })
    }

    /// Every vertex, in a stable order.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.vertices().len(), 8);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn vertices(&self) -> Vec<VertexId> {
        if let Some(h) = self.held() {
            let (_, _, n) = crate::engine_cadrs::held_counts(h);
            return (0..n).filter_map(|i| u32::try_from(i).ok()).map(|i| VertexId::new(self.stamp(), i)).collect();
        }
        self.brep().map_or_else(Vec::new, |b| b.vertex_ids().map(|v| self.vertex_id(v)).collect())
    }

    /// The face within `tol::PICK` of `p`, nearer than any other by at least `tol::PICK`:
    /// `NotFound` if none, `Ambiguous` if two are equally near (a point on an edge touches both
    /// its faces; yack never chooses).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let top = b.face_at([5.0, 5.0, 10.0])?;
    /// assert_eq!(b.face(top)?.normal(), Some(Vec3::new(0.0, 0.0, 1.0)));
    /// // a point on an edge touches two faces: refused, never a guess
    /// assert_eq!(b.face_at([5.0, 0.0, 10.0]).unwrap_err().kind, ErrorKind::Ambiguous);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn face_at(&self, p: impl Into<Point3>) -> Result<FaceId> {
        let op = "face_at";
        let p = p.into();
        if let Some(h) = self.held() {
            let found = crate::engine_cadrs::held_face_pick(op, h, p, yack_tol::PICK)?;
            return picked(op, "face", p, found, |i| FaceId::new(self.stamp(), u32::try_from(i).unwrap_or(u32::MAX)));
        }
        let found = match self.form(op)? {
            None => yack_mesh::pick::Pick::None,
            Some(b) => yack_mesh::pick::face_within(b, engine::p3(p), yack_tol::PICK).map_err(|e| engine::error(op, e))?,
        };
        match found {
            yack_mesh::pick::Pick::One(x) => Ok(self.face_id(x)),
            yack_mesh::pick::Pick::None => Err(Error::not_found(op, "p", format!("no face passes within tol::PICK of ({}, {}, {})", p.x, p.y, p.z))),
            yack_mesh::pick::Pick::Ambiguous(n) => Err(Error::ambiguous(
                op,
                "p",
                format!("{n} faces lie within tol::PICK of ({}, {}, {}) at distances that differ by less than tol::PICK", p.x, p.y, p.z),
            )),
        }
    }

    /// The edge within `tol::PICK` of `p`, nearer than any other by at least `tol::PICK`:
    /// `NotFound` if none, `Ambiguous` if two are equally near (a vertex touches all its edges).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let e = b.edge_at([5.0, 0.0, 10.0])?;
    /// assert!((b.edge(e)?.length() - 10.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn edge_at(&self, p: impl Into<Point3>) -> Result<EdgeId> {
        let op = "edge_at";
        let p = p.into();
        if let Some(h) = self.held() {
            let found = crate::engine_cadrs::held_edge_pick(op, h, p, yack_tol::PICK)?;
            return picked(op, "edge", p, found, |i| EdgeId::new(self.stamp(), u32::try_from(i).unwrap_or(u32::MAX)));
        }
        let found = match self.form(op)? {
            None => yack_mesh::pick::Pick::None,
            Some(b) => yack_mesh::pick::edge_within(b, engine::p3(p), yack_tol::PICK).map_err(|e| engine::error(op, e))?,
        };
        match found {
            yack_mesh::pick::Pick::One(x) => Ok(self.edge_id(x)),
            yack_mesh::pick::Pick::None => Err(Error::not_found(op, "p", format!("no edge passes within tol::PICK of ({}, {}, {})", p.x, p.y, p.z))),
            yack_mesh::pick::Pick::Ambiguous(n) => Err(Error::ambiguous(
                op,
                "p",
                format!("{n} edges lie within tol::PICK of ({}, {}, {}) at distances that differ by less than tol::PICK", p.x, p.y, p.z),
            )),
        }
    }

    /// The face within `tol::PICK` of each point, in order; the first point that finds none is
    /// `NotFound`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.faces_at(&[[5.0, 5.0, 10.0], [5.0, 5.0, 0.0]])?.len(), 2);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn faces_at<P: Into<Point3> + Copy>(&self, pts: &[P]) -> Result<Vec<FaceId>> {
        pts.iter().map(|&p| self.face_at(p)).collect()
    }

    /// The edge within `tol::PICK` of each point, in order; the first point that finds none is
    /// `NotFound`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let rim = b.edges_at(&[[5.0, 0.0, 10.0], [10.0, 5.0, 10.0]])?;
    /// assert_eq!(rim.len(), 2);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn edges_at<P: Into<Point3> + Copy>(&self, pts: &[P]) -> Result<Vec<EdgeId>> {
        pts.iter().map(|&p| self.edge_at(p)).collect()
    }

    /// The faces carrying feature name `tag`, given by [`named`](Solid::named).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let tagged = b.named("box");
    /// assert_eq!(tagged.faces_tagged("box").len(), 6);
    /// assert!(tagged.faces_tagged("nothing").is_empty());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn faces_tagged(&self, tag: impl Into<Tag>) -> Vec<FaceId> {
        let tag = tag.into();
        self.named_faces(|n| n.feature == tag.as_str())
    }

    /// The faces one of whose names `pick` takes (from the names alone: a cadrs-held solid too).
    fn named_faces(&self, pick: impl Fn(&yack_topo::FaceName) -> bool) -> Vec<FaceId> {
        self.face_names()
            .iter()
            .enumerate()
            .filter(|(_, names)| names.iter().any(&pick))
            .filter_map(|(i, _)| u32::try_from(i).ok())
            .map(|i| FaceId::new(self.stamp(), i))
            .collect()
    }

    /// The faces playing `role` in the operation named `tag`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let r = b.fillet(&[b.edge_at([5.0, 0.0, 10.0])?], 2.0)?.named("r");
    /// assert_eq!(r.faces_by("r", Role::Round(0)).len(), 1);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn faces_by(&self, tag: impl Into<Tag>, role: Role) -> Vec<FaceId> {
        let tag = tag.into();
        self.named_faces(|n| n.feature == tag.as_str() && engine::role(&n.face).is_some_and(|r| r == role))
    }

    /// The one face playing `role` in the operation named `tag`: `NotFound` if none,
    /// `Ambiguous` if several (a side a boolean cut in two).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let slot = Loop::starting_at([0.0, 0.0]).line_to([10.0, 0.0]).arc_ccw_to([10.0, 4.0], [10.0, 2.0]).label("bend")
    ///     .line_to([0.0, 4.0]).arc_ccw_to([0.0, 0.0], [0.0, 2.0]).close()?;
    /// let plate = extrude(&Frame::WORLD, &Profile::new(slot, vec![])?, 3.0)?.named("plate");
    /// let bend = plate.face_by("plate", Role::Side("bend".into()))?;
    /// assert!(matches!(plate.face(bend)?.kind(), SurfaceKind::Cylinder { .. }));
    /// // the top rim is one tangent chain: its round runs all the way round, in several faces
    /// let r = plate.fillet(&[plate.edge_at([5.0, 0.0, 3.0])?], 0.5)?.named("r");
    /// assert!(!r.faces_by("r", Role::Round(0)).is_empty());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn face_by(&self, tag: impl Into<Tag>, role: Role) -> Result<FaceId> {
        let op = "face_by";
        let tag = tag.into();
        let found = self.faces_by(tag.clone(), role.clone());
        match found.as_slice() {
            [f] => Ok(*f),
            [] => Err(Error::not_found(op, "role", format!("no face plays {role:?} in `{tag}`"))),
            many => Err(Error::ambiguous(op, "role", format!("{} faces play {role:?} in `{tag}`", many.len()))),
        }
    }

    /// The faces for which `keep` is true.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let up = b.faces_where(|f| matches!(f.kind(), SurfaceKind::Plane { normal } if normal.z > 0.5))?;
    /// assert_eq!(up, vec![b.face_at([5.0, 5.0, 10.0])?]);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn faces_where(&self, keep: impl Fn(&FaceRef) -> bool) -> Result<Vec<FaceId>> {
        if let Some(h) = self.held() {
            let mut out = Vec::new();
            for id in self.faces() {
                let r = held_face_ref(self, h, u64::from(id.index()), "faces_where")?;
                if keep(&r) {
                    out.push(id);
                }
            }
            return Ok(out);
        }
        let Some(b) = self.form("faces_where")? else { return Ok(Vec::new()) };
        Ok(b.face_ids().map(|f| face_ref(self, b, f)).filter(|r| keep(r)).map(|r| r.id()).collect())
    }

    /// The edges (poles aside) for which `keep` is true.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.edges_where(|e| e.convex() == Some(true))?.len(), 12);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn edges_where(&self, keep: impl Fn(&EdgeRef) -> bool) -> Result<Vec<EdgeId>> {
        if let Some(h) = self.held() {
            let mut out = Vec::new();
            for id in self.edges() {
                let r = held_edge_ref(self, h, u64::from(id.index()), "edges_where")?;
                if keep(&r) {
                    out.push(id);
                }
            }
            return Ok(out);
        }
        let Some(b) = self.form("edges_where")? else { return Ok(Vec::new()) };
        Ok(b.edge_ids().filter_map(|e| edge_ref(self, b, e)).filter(|r| keep(r)).map(|r| r.id()).collect())
    }

    /// A view of face `id`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let f = b.face(b.face_at([5.0, 5.0, 10.0])?)?;
    /// assert!(matches!(f.kind(), SurfaceKind::Plane { .. }) && (f.area() - 100.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn face(&self, id: FaceId) -> Result<FaceRef<'_>> {
        if let Some(h) = self.held() {
            if id.stamp() != self.stamp() {
                return Err(Error::foreign_id("face", "id"));
            }
            if !self.faces().contains(&id) {
                return Err(Error::not_found("face", "id", format!("no {id}")));
            }
            return held_face_ref(self, h, u64::from(id.index()), "face");
        }
        let f = self.sr_face("face", "id", id)?;
        Ok(face_ref(self, self.need("face")?, f))
    }

    /// A view of edge `id`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let e = b.edge(b.edge_at([5.0, 0.0, 10.0])?)?;
    /// assert!(matches!(e.kind(), CurveKind::Line { .. }) && e.uses() == 2);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn edge(&self, id: EdgeId) -> Result<EdgeRef<'_>> {
        if let Some(h) = self.held() {
            if id.stamp() != self.stamp() {
                return Err(Error::foreign_id("edge", "id"));
            }
            if !self.edges().contains(&id) {
                return Err(Error::not_found("edge", "id", format!("no {id}")));
            }
            return held_edge_ref(self, h, u64::from(id.index()), "edge");
        }
        let e = self.sr_edge("edge", "id", id)?;
        let b = self.need("edge")?;
        edge_ref(self, b, e).ok_or_else(|| Error::not_found("edge", "id", format!("{id} is a pole")))
    }

    fn sr_edges_of(b: &Brep, f: engine_ids::Face) -> BTreeSet<engine_ids::Edge> {
        b.face(f).loops.iter().flat_map(|&l| b.lp(l).coedges.iter().map(|&c| b.coedge(c).edge)).collect()
    }

    /// A view of vertex `id`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let corner = b.vertices().into_iter().map(|v| b.vertex(v)).collect::<Result<Vec<_>>>()?;
    /// assert!(corner.iter().any(|v| v.point() == Point3::new(10.0, 10.0, 10.0)));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn vertex(&self, id: VertexId) -> Result<VertexRef> {
        let v = self.sr_vertex("vertex", "id", id)?;
        let b = self.need("vertex")?;
        Ok(VertexRef { id, point: engine::point(b.vertex(v).point) })
    }

    /// The edges bounding face `f`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.edges_of(b.face_at([5.0, 5.0, 10.0])?)?.len(), 4);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn edges_of(&self, f: FaceId) -> Result<Vec<EdgeId>> {
        let sf = self.sr_face("edges_of", "f", f)?;
        let b = self.need("edges_of")?;
        Ok(Solid::sr_edges_of(b, sf).into_iter().filter(|&e| !b.edge(e).is_degenerate()).map(|e| self.edge_id(e)).collect())
    }

    /// The faces meeting at edge `e`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.faces_of(b.edge_at([5.0, 0.0, 10.0])?)?.len(), 2);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn faces_of(&self, e: EdgeId) -> Result<Vec<FaceId>> {
        let se = self.sr_edge("faces_of", "e", e)?;
        let b = self.need("faces_of")?;
        let faces: BTreeSet<engine_ids::Face> = b.edge_uses(se).iter().map(|&c| b.coedge_face(c)).collect();
        Ok(faces.into_iter().map(|f| self.face_id(f)).collect())
    }

    /// The edges shared by faces `a` and `b`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let (top, front) = (b.face_at([5.0, 5.0, 10.0])?, b.face_at([5.0, 0.0, 5.0])?);
    /// assert_eq!(b.edges_between(top, front)?, vec![b.edge_at([5.0, 0.0, 10.0])?]);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn edges_between(&self, a: FaceId, b: FaceId) -> Result<Vec<EdgeId>> {
        let fa = self.sr_face("edges_between", "a", a)?;
        let fb = self.sr_face("edges_between", "b", b)?;
        let br = self.need("edges_between")?;
        let ea = Solid::sr_edges_of(br, fa);
        let eb = Solid::sr_edges_of(br, fb);
        Ok(ea.intersection(&eb).filter(|&&e| !br.edge(e).is_degenerate()).map(|&e| self.edge_id(e)).collect())
    }
}
