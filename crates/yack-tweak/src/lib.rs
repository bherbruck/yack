// yack: Planar faces of a solid moved onto new planes, its topology kept: every vertex goes where the
// Adapted from springrep@a03e8a1 src/edit/tweak.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from edit/tweak.rs to tweak/mod.rs (M2.4, T6); paths only; then to yack-tweak (M3 W7), paths and visibility only;
// split by what it does, on checked access, its indexing allow gone (hardening T1-T2).
//! Planar faces of a solid moved onto new planes, its topology kept: every vertex goes where the
//! planes of its faces meet and every edge between two planes becomes the line between its new
//! vertices. Drafts and shells of solids bounded by planes are made this way.
//!
//! Only a vertex whose faces are all planes, three or more of them independent, can move, and only
//! straight edges move with it. A move that would need anything else, or that turns a face inside
//! out, is an error, never a solid with faces off their surfaces.

// springrep's foundation modules (D26), under the names the moved code uses.
use yack_base::kernel::Error;
use yack_base::kernel::Result;
use yack_check as check;
use yack_base::kernel as error;
use yack_geom as geom;
use yack_intersect as intersect;
use yack_math::kernel as math;
use yack_numeric as numeric;
use yack_tol as tol;
use yack_topo as topo;
use std::collections::{BTreeMap, BTreeSet};

use crate::{
    geom::Surface,
    math::{Frame, Point3, Vec2},
    topo::{Brep, EdgeId, FaceId, VertexId},
};

/// The faces round each vertex.
fn vertex_faces(b: &Brep) -> BTreeMap<VertexId, BTreeSet<FaceId>> {
    let mut out: BTreeMap<VertexId, BTreeSet<FaceId>> = BTreeMap::new();
    for f in b.face_ids() {
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                let e = b.edge(b.coedge(c).edge);
                out.entry(e.start).or_default().insert(f);
                out.entry(e.end).or_default().insert(f);
            }
        }
    }
    out
}

/// The faces on each edge, one entry per use.
fn edge_faces(b: &Brep) -> BTreeMap<EdgeId, Vec<FaceId>> {
    let mut out: BTreeMap<EdgeId, Vec<FaceId>> = BTreeMap::new();
    for f in b.face_ids() {
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                out.entry(b.coedge(c).edge).or_default().push(f);
            }
        }
    }
    out
}

/// The entry at `index` of a table built with one entry per entity of the solid being moved.
fn entry<T: Copy>(table: &[T], index: usize) -> Result<T> {
    table.get(index).copied().ok_or_else(|| {
        Error::InvalidResult(vec![format!("a move's table of {} has no entry {index}", table.len())])
    })
}

/// `out` given `b`'s shells (`faces[k]` standing for `b`'s face `k`) and solids, its radial order
/// paired, then checked: the solid, or the checker's issues.
fn finished(b: &Brep, mut out: Brep, faces: &[FaceId]) -> Result<Brep> {
    for sh in b.shell_ids() {
        let ids = b
            .shell(sh)
            .faces
            .iter()
            .map(|f| entry(faces, f.index()))
            .collect::<Result<Vec<_>>>()?;
        let _ = out.add_shell(ids);
    }
    for so in b.solid_ids() {
        out.add_solid_of(b.solid(so).shells.clone(), b.solid(so).outer);
    }
    out.pair_radial();
    let issues = check::check(&out);
    if issues.is_empty() {
        Ok(out)
    } else {
        Err(Error::InvalidResult(
            issues.into_iter().map(|i| i.to_string()).collect(),
        ))
    }
}

fn plane_of(s: &Surface) -> Option<Frame> {
    match s {
        Surface::Plane(p) => Some(p.frame),
        _ => None,
    }
}

fn unsupported<T>(why: &str) -> Result<T> {
    Err(Error::Unsupported(why.to_string()))
}

/// Refuses to move a face that runs up to where the solid touches itself: moving it would open
/// the contact or push one piece into the other.
fn clear_of_contacts<'a>(b: &Brep, faces: impl IntoIterator<Item = &'a FaceId>) -> Result<()> {
    if crate::check::faces_at_contacts(b, faces.into_iter().copied()).is_empty() {
        return Ok(());
    }
    unsupported("moving a face up to where the solid touches itself")
}

/// A move's result: the solid, and where faces closed on the way, the error the direct move gave.
/// The caller then merges the faces round the closed ones (`boolean::unify`, which this layer may
/// not call), and reports that error if the merge fails.
pub struct Moved {
    pub brep: Brep,
    pub merge: Option<Error>,
}

/// The direct move, or where it fails, the move followed through the faces that close on the way.
fn followed(direct: Result<Brep>, through: impl FnOnce() -> Result<(Brep, bool)>) -> Result<Moved> {
    match direct {
        Ok(brep) => Ok(Moved { brep, merge: None }),
        Err(e) => match through() {
            Ok((brep, true)) => Ok(Moved { brep, merge: Some(e) }),
            Ok((brep, false)) => Ok(Moved { brep, merge: None }),
            Err(_) => Err(e),
        },
    }
}

/// The surface parameters of `p` on `s` near `last`, moved by whole periods to continue it.
fn near_uv(s: &Surface, p: Point3, last: Option<Vec2>) -> Vec2 {
    let uv = s.project(p);
    let Some(to) = last else { return uv };
    let shift = |x: f64, t: f64, per: Option<f64>| per.map_or(x, |q| x + ((t - x) / q).round() * q);
    Vec2::new(
        shift(uv.x, to.x, s.u_period()),
        shift(uv.y, to.y, s.v_period()),
    )
}

mod draft;
mod offset;
mod planes;
mod surfaces;
mod through;

pub use draft::draft;
pub use offset::{offset_faces, offset_followed, offset_surface};
pub use surfaces::{move_faces, move_faces_by, move_faces_pinned};

#[cfg(test)]
mod tests;
