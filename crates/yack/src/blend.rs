// yack: fillets and chamfers on `Solid`, one method per protocol function, routed to springrep
// (PLAN §4.3, D4, M1.3).
use yack_springrep::blend as sr;

use crate::engine;
use crate::{engine_cadrs, firewall, ladder};
use yack_cadrs::yack_glue as glue;
use crate::{Angle, EdgeId, FaceId, Point3, Result, Solid};

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
        firewall::run("fillet", || {
            let feature = engine::feature();
            let op = "fillet";
            let ids = self.sr_edges(op, "edges", edges)?;
            let ix = crate::edit::indices(edges);
            ladder::two(
                op,
                &|| engine::made(&feature, op, sr::fillet_edges(self.need(op)?, &ids, radius, &feature)),
                &|| engine_cadrs::blend(op, self, &ix, "round", |k, id| glue::fillet(k, id, &ix, radius)),
            )
        })
    }

    /// Rounds each edge with its own radius.
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
    pub fn fillet_conic(&self, edges: &[EdgeId], setback: f64, rho: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_conic", || {
            let feature = engine::feature();
            let op = "fillet_conic";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_conic(self.need(op)?, &ids, setback, rho, &feature))
        })
    }

    /// A curvature-continuous blend `offset` from the edge, with `center` controlling its depth.
    pub fn fillet_g2(&self, edges: &[EdgeId], offset: f64, center: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_g2", || {
            let feature = engine::feature();
            let op = "fillet_g2";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_g2(self.need(op)?, &ids, offset, center, &feature))
        })
    }

    /// A fillet of `radius` with corner setbacks of `setback` where edges meet.
    pub fn fillet_setback(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_setback", || {
            let feature = engine::feature();
            let op = "fillet_setback";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_setback(self.need(op)?, &ids, radius, setback, &feature))
        })
    }

    /// As [`fillet_setback`](Solid::fillet_setback) with curvature-continuous corner patches.
    pub fn fillet_setback_g2(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_setback_g2", || {
            let feature = engine::feature();
            let op = "fillet_setback_g2";
            let ids = self.sr_edges(op, "edges", edges)?;
            engine::made(&feature, op, sr::fillet_setback_g2(self.need(op)?, &ids, radius, setback, &feature))
        })
    }

    /// A fillet of `radius` between faces `a` and `b`.
    pub fn fillet_faces(&self, a: FaceId, b: FaceId, radius: f64) -> Result<Solid> {
        ladder::springrep_only("fillet_faces", || {
            let feature = engine::feature();
            let op = "fillet_faces";
            let (fa, fb) = (self.sr_face(op, "a", a)?, self.sr_face(op, "b", b)?);
            engine::made(&feature, op, sr::fillet_faces(self.need(op)?, fa, fb, radius, &feature))
        })
    }

    /// Replaces `middle` with a round tangent to `side_a` and `side_b`.
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
    pub fn chamfer(&self, edges: &[EdgeId], distance: f64) -> Result<Solid> {
        firewall::run("chamfer", || {
            let feature = engine::feature();
            let op = "chamfer";
            let ids = self.sr_edges(op, "edges", edges)?;
            let ix = crate::edit::indices(edges);
            ladder::two(
                op,
                &|| engine::made(&feature, op, sr::chamfer_edges(self.need(op)?, &ids, distance, &feature)),
                &|| engine_cadrs::blend(op, self, &ix, "bevel", |k, id| glue::chamfer(k, id, &ix, distance)),
            )
        })
    }

    /// Bevels `edge` with setback `d1` along `face` (one of the edge's two faces) and `d2` along
    /// the other.
    pub fn chamfer_two(&self, edge: EdgeId, face: FaceId, d1: f64, d2: f64) -> Result<Solid> {
        firewall::run("chamfer_two", || {
            let feature = engine::feature();
            let op = "chamfer_two";
            let e = self.sr_edge(op, "edge", edge)?;
            let f = self.sr_face(op, "face", face)?;
            let (ei, fi) = (u64::from(edge), u64::from(face));
            ladder::two(
                op,
                &|| engine::made(&feature, op, sr::chamfer_two_edges(self.need(op)?, e, f, d1, d2, &feature)),
                &|| engine_cadrs::blend(op, self, &[ei], "bevel", |k, id| glue::chamfer_two(k, id, ei, fi, d1, d2)),
            )
        })
    }

    /// Bevels each edge with its own setback distance.
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
    pub fn chamfer_angle(&self, edge: EdgeId, face: FaceId, distance: f64, angle: Angle) -> Result<Solid> {
        let op = "chamfer_angle";
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
