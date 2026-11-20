// yack: selection and topology walks on `Solid` (PLAN §4.2 rule 6), routed to springrep.
use std::collections::BTreeSet;

use yack_springrep::{geom, measure, pick};

use crate::engine::{self, Brep};
use crate::solid::engine_ids;
use crate::{
    Aabb, CurveKind, EdgeId, EdgeRef, Error, FaceId, FaceRef, Point3, Result, Solid, SurfaceKind, VertexId,
};

fn face_ref<'a>(s: &'a Solid, b: &Brep, f: engine_ids::Face) -> FaceRef<'a> {
    let face = b.face(f);
    let outward = |z: yack_springrep::math::Vec3| if face.reversed { engine::vec(-z) } else { engine::vec(z) };
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
    let bb = measure::face_bbox(b, f);
    FaceRef::new(
        s.face_id(f),
        kind,
        normal,
        measure::face_area(b, f),
        engine::point(measure::face_centroid(b, f)),
        Aabb::new(engine::point(bb.min), engine::point(bb.max)),
    )
}

fn edge_ref<'a>(s: &'a Solid, b: &Brep, e: engine_ids::Edge, convex: &BTreeSet<engine_ids::Edge>) -> Option<EdgeRef<'a>> {
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
    let convex = if convex.contains(&e) { Some(true) } else { None };
    Some(EdgeRef::new(s.edge_id(e), kind, ends, convex, measure::edge_length(b, e), mid))
}

impl Solid {
    /// Every face, in a stable order.
    pub fn faces(&self) -> Vec<FaceId> {
        self.brep().map_or_else(Vec::new, |b| b.face_ids().map(|f| self.face_id(f)).collect())
    }

    /// Every edge (poles aside), in a stable order.
    pub fn edges(&self) -> Vec<EdgeId> {
        self.brep().map_or_else(Vec::new, |b| {
            b.edge_ids().filter(|&e| !b.edge(e).is_degenerate()).map(|e| self.edge_id(e)).collect()
        })
    }

    /// Every vertex, in a stable order.
    pub fn vertices(&self) -> Vec<VertexId> {
        self.brep().map_or_else(Vec::new, |b| b.vertex_ids().map(|v| self.vertex_id(v)).collect())
    }

    /// The face within `tol::PICK` of `p`; `NotFound` if none.
    pub fn face_at(&self, p: impl Into<Point3>) -> Result<FaceId> {
        let p = p.into();
        let found = self.brep().and_then(|b| measure::face_at(b, engine::p3(p)));
        match found {
            Some((f, d)) if d <= yack_tol::PICK => Ok(self.face_id(f)),
            _ => Err(Error::not_found("face_at", "p", format!("no face passes within tol::PICK of ({}, {}, {})", p.x, p.y, p.z))),
        }
    }

    /// The edge within `tol::PICK` of `p`; `NotFound` if none.
    pub fn edge_at(&self, p: impl Into<Point3>) -> Result<EdgeId> {
        let p = p.into();
        let found = self.brep().and_then(|b| measure::edge_at(b, engine::p3(p)));
        match found {
            Some((e, d)) if d <= yack_tol::PICK => Ok(self.edge_id(e)),
            _ => Err(Error::not_found("edge_at", "p", format!("no edge passes within tol::PICK of ({}, {}, {})", p.x, p.y, p.z))),
        }
    }

    /// The face within `tol::PICK` of each point, in order; the first point that finds none is
    /// `NotFound`.
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

    /// The faces for which `keep` is true.
    pub fn faces_where(&self, keep: impl Fn(&FaceRef) -> bool) -> Vec<FaceId> {
        let Some(b) = self.brep() else { return Vec::new() };
        b.face_ids().map(|f| face_ref(self, b, f)).filter(|r| keep(r)).map(|r| r.id()).collect()
    }

    /// The edges (poles aside) for which `keep` is true.
    pub fn edges_where(&self, keep: impl Fn(&EdgeRef) -> bool) -> Vec<EdgeId> {
        let Some(b) = self.brep() else { return Vec::new() };
        let convex: BTreeSet<engine_ids::Edge> = pick::convex_edges(b).unwrap_or_default().into_iter().collect();
        b.edge_ids().filter_map(|e| edge_ref(self, b, e, &convex)).filter(|r| keep(r)).map(|r| r.id()).collect()
    }

    /// A view of face `id`.
    pub fn face(&self, id: FaceId) -> Result<FaceRef<'_>> {
        let f = self.sr_face("face", "id", id)?;
        Ok(face_ref(self, self.need("face")?, f))
    }

    /// A view of edge `id`.
    pub fn edge(&self, id: EdgeId) -> Result<EdgeRef<'_>> {
        let e = self.sr_edge("edge", "id", id)?;
        let b = self.need("edge")?;
        let convex: BTreeSet<engine_ids::Edge> = pick::convex_edges(b).unwrap_or_default().into_iter().collect();
        edge_ref(self, b, e, &convex).ok_or_else(|| Error::not_found("edge", "id", format!("{id} is a pole")))
    }

    fn sr_edges_of(b: &Brep, f: engine_ids::Face) -> BTreeSet<engine_ids::Edge> {
        b.face(f).loops.iter().flat_map(|&l| b.lp(l).coedges.iter().map(|&c| b.coedge(c).edge)).collect()
    }

    /// The edges bounding face `f`.
    pub fn edges_of(&self, f: FaceId) -> Result<Vec<EdgeId>> {
        let sf = self.sr_face("edges_of", "f", f)?;
        let b = self.need("edges_of")?;
        Ok(Solid::sr_edges_of(b, sf).into_iter().filter(|&e| !b.edge(e).is_degenerate()).map(|e| self.edge_id(e)).collect())
    }

    /// The faces meeting at edge `e`.
    pub fn faces_of(&self, e: EdgeId) -> Result<Vec<FaceId>> {
        let se = self.sr_edge("faces_of", "e", e)?;
        let b = self.need("faces_of")?;
        let faces: BTreeSet<engine_ids::Face> = b.edge_uses(se).iter().map(|&c| b.coedge_face(c)).collect();
        Ok(faces.into_iter().map(|f| self.face_id(f)).collect())
    }

    /// The edges shared by faces `a` and `b`.
    pub fn edges_between(&self, a: FaceId, b: FaceId) -> Result<Vec<EdgeId>> {
        let fa = self.sr_face("edges_between", "a", a)?;
        let fb = self.sr_face("edges_between", "b", b)?;
        let br = self.need("edges_between")?;
        let ea = Solid::sr_edges_of(br, fa);
        let eb = Solid::sr_edges_of(br, fb);
        Ok(ea.intersection(&eb).filter(|&&e| !br.edge(e).is_degenerate()).map(|&e| self.edge_id(e)).collect())
    }
}
