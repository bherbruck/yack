// yack: fillets and chamfers on `Solid`, one method per protocol function, routed to springrep
// (PLAN §4.3, D4, M1.3).
use yack_springrep::blend as sr;

use crate::engine;
use crate::{engine_cadrs, firewall, ladder};
use yack_cadrs::yack_glue as glue;
use crate::{Angle, EdgeId, FaceId, Point3, Result, Solid};

/// A hard guard (brep-bench edit/half-torus-outer-rim-round-deleted): springrep's fillet given
/// one rim of a half torus's flat face rounds both. A springrep blend with a group of blend faces
/// that comes nowhere near the given edges (judged on springrep's own result): refused (never
/// this solid), so cadrs answers.
fn over_blend(op: &'static str, input: &Solid, edges: &[u64], part: &str, size: f64, out: Solid) -> Result<Solid> {
    let near: Vec<crate::Point3> = edges
        .iter()
        .filter_map(|&e| input.edges().into_iter().find(|id| u64::from(*id) == e))
        .filter_map(|id| input.edge(id).ok().map(|r| r.midpoint()))
        .collect();
    let strays = match (out.brep(), out.feature_name()) {
        (Some(b), Some(f)) => yack_springrep::facade::api::blend_strays(b, f, part, &near.iter().map(|&p| engine::p3(p)).collect::<Vec<_>>(), yack_math::scale_length(size, 2.0)),
        _ => false,
    };
    if strays {
        return Err(crate::Error::failed(op, crate::Stage::Validate, "springrep declines: it blended faces away from the given edges' chains"));
    }
    Ok(out)
}

impl Solid {
    /// Rounds `edges` with a constant `radius` (mm). Each edge brings its whole tangent chain (the
    /// edges it runs on into smoothly, seams aside): the round runs on along it.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let edge = b.edge_at([5.0, 0.0, 10.0])?;
    /// let r = b.fillet(&[edge], 1.0)?;
    /// assert!(r.volume() < 1000.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet(&self, edges: &[EdgeId], radius: f64) -> Result<Solid> {
        if edges.is_empty() {
            return Err(crate::Error::invalid("fillet", "edges", "no edges to round"));
        }
        firewall::run("fillet", || {
            let feature = engine::feature();
            let op = "fillet";
            let ids = self.sr_edges(op, "edges", edges)?;
            let ix = crate::edit::indices(edges);
            ladder::two(
                op,
                &|| over_blend(op, self, &ix, "round_", radius, engine::made(&feature, op, sr::fillet_edges(self.need(op)?, &ids, radius, &feature))?),
                &|| engine_cadrs::blend(op, self, &ix, "round", |k, id| glue::fillet(k, id, &ix, radius)),
            )
        })
    }

    /// Rounds each edge with its own radius.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let r = b.fillet_radii(&[(b.edge_at([5.0, 0.0, 10.0])?, 1.0), (b.edge_at([5.0, 10.0, 0.0])?, 2.0)])?;
    /// let pi = std::f64::consts::PI;
    /// assert!((r.volume() - (1000.0 - (1.0 - pi / 4.0) * 10.0 * (1.0 + 4.0))).abs() < 1e-6 * 1000.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet_radii(&self, edges: &[(EdgeId, f64)]) -> Result<Solid> {
        ladder::springrep_only("fillet_radii", || {
            let feature = engine::feature();
            let op = "fillet_radii";
            let pairs = edges.iter().map(|&(e, r)| Ok((self.sr_edge(op, "edges", e)?, r))).collect::<Result<Vec<_>>>()?;
            engine::made(&feature, op, sr::fillet_radii(self.need(op)?, &pairs, &feature))
        })
    }

    /// Rounds `edge` with a radius running from `r_from` at the end nearest `from` to `r_to` at
    /// the other.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // a round growing from 1 to 2 along the top front edge
    /// match b.fillet_variable(b.edge_at([5.0, 0.0, 10.0])?, [0.0, 0.0, 10.0], 1.0, 2.0) {
    ///     Ok(r) => assert!(r.volume() < 1000.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet_variable(&self, edge: EdgeId, from: impl Into<Point3>, r_from: f64, r_to: f64) -> Result<Solid> {
        let from: Point3 = from.into();
        firewall::run("fillet_variable", || {
            let feature = engine::feature();
            let op = "fillet_variable";
            let e = self.sr_edge(op, "edge", edge)?;
            let ei = u64::from(edge);
            ladder::two(
                op,
                &|| engine::made(&feature, op, sr::fillet_variable(self.need(op)?, e, engine::p3(from), r_from, r_to, &feature)),
                &|| engine_cadrs::blend(op, self, &[ei], "round", |k, id| glue::fillet_variable(k, id, ei, from.to_array(), r_from, r_to)),
            )
        })
    }

    /// A conic blend with `setback` along each face and shape `rho` (0.5 is parabolic).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// match b.fillet_conic(&[b.edge_at([5.0, 0.0, 10.0])?], 2.0, 0.5) {
    ///     Ok(r) => assert!(r.volume() < 1000.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet_conic(&self, edges: &[EdgeId], setback: f64, rho: f64) -> Result<Solid> {
        let ix = crate::edit::indices(edges);
        firewall::run("fillet_conic", || {
            ladder::two(
                "fillet_conic",
                &|| {
            let feature = engine::feature();
            let op = "fillet_conic";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_conic(self.need(op)?, &ids, setback, rho, &feature))
        },
                &|| engine_cadrs::blend("fillet_conic", self, &ix, "round", |k, id| glue::fillet_conic(k, id, &ix, setback, rho)),
            )
        })
    }

    /// A curvature-continuous blend: it meets each face where a ball of radius `offset` (mm)
    /// rolling along the edge would touch it, sharing the face's tangent plane and curvature there,
    /// and bends with radius of curvature `center_radius` (mm) at its middle. At a square edge the
    /// blend that takes off what a round of `offset` does has `center_radius` about 1.389 ×
    /// `offset`; `center_radius` must be one the blend can take.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// match b.fillet_g2(&[b.edge_at([5.0, 0.0, 10.0])?], 2.0, 1.0) {
    ///     Ok(r) => assert!(r.volume() < 1000.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet_g2(&self, edges: &[EdgeId], offset: f64, center_radius: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_g2", || {
            let feature = engine::feature();
            let op = "fillet_g2";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_g2(self.need(op)?, &ids, offset, center_radius, &feature))
        })
    }

    /// A fillet of `radius` with corner setbacks of `setback` where edges meet.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// match b.fillet_setback(&[b.edge_at([5.0, 0.0, 10.0])?], 1.0, 2.0) {
    ///     Ok(r) => assert!(r.volume() < 1000.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet_setback(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_setback", || {
            let feature = engine::feature();
            let op = "fillet_setback";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_setback(self.need(op)?, &ids, radius, setback, &feature))
        })
    }

    /// As [`fillet_setback`](Solid::fillet_setback) with curvature-continuous corner patches.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// match b.fillet_setback_g2(&[b.edge_at([5.0, 0.0, 10.0])?], 1.0, 2.0) {
    ///     Ok(r) => assert!(r.volume() < 1000.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet_setback_g2(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_setback_g2", || {
            let feature = engine::feature();
            let op = "fillet_setback_g2";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_setback_g2(self.need(op)?, &ids, radius, setback, &feature))
        })
    }

    /// A fillet of `radius` between faces `a` and `b`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // the top and the front, rounded at 2 where they meet: (1 − π/4)·r² of section along 10
    /// let r = b.fillet_faces(b.face_at([5.0, 5.0, 10.0])?, b.face_at([5.0, 0.0, 5.0])?, 2.0)?;
    /// assert!((r.volume() - (1000.0 - (1.0 - std::f64::consts::PI / 4.0) * 40.0)).abs() < 1e-6 * 1000.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn fillet_faces(&self, a: FaceId, b: FaceId, radius: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_faces", || {
            let feature = engine::feature();
            let op = "fillet_faces";
            let (fa, fb) = (self.sr_face(op, "a", a)?, self.sr_face(op, "b", b)?);
            engine::made(&feature, op, sr::fillet_faces(self.need(op)?, fa, fb, radius, &feature))
        })
    }

    /// Replaces `middle` with a round tangent to `side_a` and `side_b`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a wall 2 thick: its top replaced by a half cylinder of radius 1
    /// let wall = block(&Frame::WORLD, [10.0, 2.0, 10.0])?;
    /// let (a, top, c) = (wall.face_at([5.0, 0.0, 5.0])?, wall.face_at([5.0, 1.0, 10.0])?, wall.face_at([5.0, 2.0, 5.0])?);
    /// match wall.full_round(a, top, c) {
    ///     Ok(r) => assert!((r.volume() - (200.0 - 10.0 * (2.0 - std::f64::consts::PI / 2.0))).abs() < 1e-6 * 200.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn full_round(&self, side_a: FaceId, middle: FaceId, side_b: FaceId) -> Result<Solid> {
        ladder::springrep_only("full_round", || {
            let feature = engine::feature();
            let op = "full_round";
            let a = self.sr_face(op, "side_a", side_a)?;
            let m = self.sr_face(op, "middle", middle)?;
            let b = self.sr_face(op, "side_b", side_b)?;
            engine::made(&feature, op, sr::full_round(self.need(op)?, a, m, b, &feature))
        })
    }

    /// Bevels `edges` by `distance`: a setback from the edge along each face, measured in the
    /// plane normal to the edge (never a face offset). Each edge brings its whole tangent chain, as
    /// for [`fillet`](Solid::fillet).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let c = b.chamfer(&[b.edge_at([5.0, 0.0, 10.0])?], 1.0)?;
    /// assert!((c.volume() - 995.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn chamfer(&self, edges: &[EdgeId], distance: f64) -> Result<Solid> {
        if edges.is_empty() {
            return Err(crate::Error::invalid("chamfer", "edges", "no edges to bevel"));
        }
        firewall::run("chamfer", || {
            let feature = engine::feature();
            let op = "chamfer";
            let ids = self.sr_edges(op, "edges", edges)?;
            let ix = crate::edit::indices(edges);
            ladder::two(
                op,
                &|| over_blend(op, self, &ix, "bevel_", distance, engine::made(&feature, op, sr::chamfer_edges(self.need(op)?, &ids, distance, &feature))?),
                &|| engine_cadrs::blend(op, self, &ix, "bevel", |k, id| glue::chamfer(k, id, &ix, distance)),
            )
        })
    }

    /// Bevels `edge` unevenly: the cut meets `face` (one of the edge's two faces) `along_face` mm
    /// back from the edge, and the edge's other face `along_other` mm back. Each setback must fit
    /// on its face.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // 1 along the top, 2 down the front: a triangle of area 1 removed along 10
    /// let c = b.chamfer_two(b.edge_at([5.0, 0.0, 10.0])?, b.face_at([5.0, 5.0, 10.0])?, 1.0, 2.0)?;
    /// assert!((c.volume() - 990.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn chamfer_two(&self, edge: EdgeId, face: FaceId, along_face: f64, along_other: f64) -> Result<Solid> {
        if !self.faces_of(edge)?.contains(&face) {
            return Err(crate::Error::invalid("chamfer_two", "face", "the face is not one of the edge's two faces"));
        }
        firewall::run("chamfer_two", || {
            let feature = engine::feature();
            let op = "chamfer_two";
            let e = self.sr_edge(op, "edge", edge)?;
            let f = self.sr_face(op, "face", face)?;
            let (ei, fi) = (u64::from(edge), u64::from(face));
            ladder::two(
                op,
                &|| engine::made(&feature, op, sr::chamfer_two_edges(self.need(op)?, e, f, along_face, along_other, &feature)),
                &|| engine_cadrs::blend(op, self, &[ei], "bevel", |k, id| glue::chamfer_two(k, id, ei, fi, along_face, along_other)),
            )
        })
    }

    /// Bevels each edge with its own setback distance.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let c = b.chamfer_distances(&[(b.edge_at([5.0, 0.0, 10.0])?, 1.0), (b.edge_at([5.0, 10.0, 0.0])?, 2.0)])?;
    /// assert!((c.volume() - 975.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn chamfer_distances(&self, edges: &[(EdgeId, f64)]) -> Result<Solid> {
        ladder::springrep_only("chamfer_distances", || {
            let feature = engine::feature();
            let op = "chamfer_distances";
            let pairs = edges.iter().map(|&(e, d)| Ok((self.sr_edge(op, "edges", e)?, d))).collect::<Result<Vec<_>>>()?;
            engine::made(&feature, op, sr::chamfer_distances(self.need(op)?, &pairs, &feature))
        })
    }

    /// Bevels `edge` with setback `distance` along `face` (one of its two faces), the bevel at
    /// `angle` to that face (cadrs only).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// match b.chamfer_angle(b.edge_at([5.0, 0.0, 10.0])?, b.face_at([5.0, 5.0, 10.0])?, 1.0, deg(45.0)) {
    ///     Ok(c) => assert!((c.volume() - 995.0).abs() < 1e-6 * 1000.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn chamfer_angle(&self, edge: EdgeId, face: FaceId, distance: f64, angle: Angle) -> Result<Solid> {
        let op = "chamfer_angle";
        if !(distance.is_finite() && distance > 0.0) {
            return Err(crate::Error::invalid(op, "distance", format!("distance {distance} is not positive")));
        }
        if !(angle.radians() > 0.0 && angle.radians() < core::f64::consts::FRAC_PI_2) {
            return Err(crate::Error::invalid(op, "angle", "the bevel's angle to the face must be between 0 and 90 degrees"));
        }
        self.sr_edge(op, "edge", edge)?;
        self.sr_face(op, "face", face)?;
        let (ei, fi, a) = (u64::from(edge), u64::from(face), angle.radians());
        firewall::run(op, || {
            ladder::ladder(
                op,
                &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::blend(op, self, &[ei], "bevel", |k, id| glue::chamfer_angle(k, id, ei, fi, distance, a)) }],
            )
        })
    }
}
