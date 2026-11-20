// yack: glue for yack-bridge: a checked builder of springrep `Brep`s from outside the crate.
//! What `yack-bridge` needs that springrep keeps crate-private: building a `Brep` entity by entity
//! (with local tolerances and face names), finished by springrep's own checker.

use crate::{
    Error, Result, check,
    geom::{Curve, Surface},
    math::Point3,
    topo::{Brep, CoedgeSpec, EdgeId, FaceId, FaceName, LoopId, ShellId, SolidId, VertexId},
};

/// A `Brep` under construction. Entities get ids in the order they are added.
#[derive(Debug, Default)]
pub struct Builder {
    b: Brep,
}

impl Builder {
    /// An empty builder.
    pub fn new() -> Builder {
        Builder::default()
    }

    /// A vertex at `p` standing for the ball of radius `tol` (at most `tol::LINEAR` means exact).
    pub fn vertex(&mut self, p: Point3, tol: f64) -> VertexId {
        let v = self.b.add_vertex(p);
        self.b.set_vertex_tolerance(v, tol);
        v
    }

    /// The edge of `curve` over `range` from `start` to `end`, with local tolerance `tol`.
    pub fn edge(&mut self, curve: Curve, range: (f64, f64), start: VertexId, end: VertexId, tol: f64) -> EdgeId {
        let e = self.b.add_edge(curve, range, start, end);
        self.b.set_edge_tolerance(e, tol);
        e
    }

    /// A degenerate edge (a pole or apex) at `vertex`, its pcurves over `range`.
    pub fn degenerate_edge(&mut self, vertex: VertexId, range: (f64, f64)) -> EdgeId {
        self.b.add_degenerate_edge(vertex, range)
    }

    /// A face on `surface`, its outward normal the natural one unless `reversed`.
    pub fn face(&mut self, surface: Surface, reversed: bool, names: Vec<FaceName>) -> FaceId {
        self.b.add_face(surface, reversed, names)
    }

    /// A loop of `face`, outer first.
    pub fn add_loop(&mut self, face: FaceId, coedges: Vec<CoedgeSpec>) -> LoopId {
        self.b.add_loop(face, coedges)
    }

    /// A shell of `faces`.
    pub fn shell(&mut self, faces: Vec<FaceId>) -> ShellId {
        self.b.add_shell(faces)
    }

    /// A solid whose first `outer` shells bound it from outside, the rest voids.
    pub fn solid(&mut self, shells: Vec<ShellId>, outer: usize) -> SolidId {
        self.b.add_solid_of(shells, outer)
    }

    /// The built `Brep`, or `InvalidResult` with springrep's checker's issues.
    pub fn finish(self) -> Result<Brep> {
        let issues = check::check(&self.b);
        if issues.is_empty() {
            Ok(self.b)
        } else {
            Err(Error::InvalidResult(issues.into_iter().map(|i| i.message).collect()))
        }
    }
}
