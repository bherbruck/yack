// yack: a held body carried between sessions, meshed and drawn.
//! A held body taken from one session into another with its records, and its mesh and drawing
//! (split out of `yack_glue`, unchanged).

use crate::{BodyId, DefaultKernel, Kernel, KernelError, Result};

/// A body taken out of its session with what the session keeps about it for later operations
/// ([`carry_out`], [`carry_in`]).
pub struct Carried {
    body: crate::native::topo::Body,
    colors: Vec<Option<[u8; 4]>>,
    torus: Option<std::sync::Arc<crate::native::sweep_tube::SweptTorus>>,
    pipe: Option<std::sync::Arc<crate::native::sweep_pipe::Pipe>>,
}

/// Body `id` of session `from` with its face colours and the record of a torus swept exactly or
/// of a pipe (a boolean of two swept tori is made again from their records; a pipe slid is swept
/// as one), to put in another session with [`carry_in`]. A plain copy loses the records, and the
/// same operation then runs otherwise there (brep-bench swept/tumbling-torus-crossed-*: two
/// fresh tori swept and combined failed where one torus swept twice didn't). Taken out and put
/// in as two steps, so no caller holds two sessions at once.
pub fn carry_out(from: &DefaultKernel, id: BodyId) -> Result<Carried> {
    Ok(Carried {
        body: (**from.get(id)?).clone(),
        colors: from.colors(id).to_vec(),
        torus: from.swept_tori.get(&id.0).cloned(),
        pipe: from.pipes.get(&id.0).cloned(),
    })
}

/// [`carry_out`]'s body in session `into`, under a new id there, with what came with it.
pub fn carry_in(into: &mut DefaultKernel, c: Carried) -> Result<BodyId> {
    let made = into.insert(c.body);
    if !c.colors.is_empty() {
        into.set_colors(made, c.colors);
    }
    if let Some(t) = c.torus {
        into.swept_tori.insert(made.0, t);
    }
    if let Some(p) = c.pipe {
        into.pipes.insert(made.0, p);
    }
    Ok(made)
}

/// The turn per triangle a held body's mesh allows (radians): springrep's `MeshOptions::ANGULAR`.
const MESH_TURN: f64 = 0.5;

fn meshed(k: &DefaultKernel, body: BodyId, chord: f64) -> Result<crate::TriMesh> {
    if !(chord.is_finite() && chord > 0.0) {
        return Err(KernelError::InvalidParameter(format!("a mesh's chord must be finite and above zero, not {chord}")));
    }
    k.tessellate(body, crate::Tessellation { deflection: chord, angle: MESH_TURN })
}

/// `body`'s surface as one closed, outward, welded mesh (springrep's `Solid::mesh`): every point
/// within `chord` (mm) of the exact surface, at most [`MESH_TURN`] between neighbouring normals,
/// faces sharing their boundary points (the mesher's are bit for bit the same), triangles
/// counter-clockwise seen from outside. A mesh that does not close refuses.
pub fn mesh(k: &DefaultKernel, body: BodyId, chord: f64) -> Result<(Vec<[f64; 3]>, Vec<[u32; 3]>)> {
    let m = meshed(k, body, chord)?;
    let check = crate::native::tess::check_tri_mesh(&m);
    if !check.is_watertight() {
        return Err(KernelError::OperationFailed(format!(
            "the mesh does not close ({} open and {} bad sides of {})",
            check.open, check.bad, check.edges
        )));
    }
    let mut at: std::collections::BTreeMap<[u64; 3], u32> = std::collections::BTreeMap::new();
    let mut points: Vec<[f64; 3]> = Vec::new();
    let mut index = |i: u32| -> Result<u32> {
        let p = m.positions.get(i as usize).ok_or_else(|| KernelError::OperationFailed("a triangle past the mesh's points".into()))?;
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        Ok(*at.entry(key).or_insert_with(|| {
            points.push([p.x, p.y, p.z]);
            (points.len() - 1) as u32
        }))
    };
    let mut tris = Vec::with_capacity(m.indices.len());
    for t in &m.indices {
        let [a, b, c] = [index(t[0])?, index(t[1])?, index(t[2])?];
        if a != b && b != c && c != a {
            tris.push([a, b, c]);
        }
    }
    Ok((points, tris))
}

/// A held body drawn: per face in face order its points, the exact surface's outward unit normal
/// at each and its triangles (not welded across faces); then the edges' polylines.
pub type Drawn3 = (Vec<(Vec<[f64; 3]>, Vec<[f64; 3]>, Vec<[u32; 3]>)>, Vec<Vec<[f64; 3]>>);

/// `body` drawn as springrep's `display_within(b, chord, edge_chord)`: its faces within `chord`
/// (mm) of the exact surface and its edges' polylines within `edge_chord` of their curves.
pub fn display(k: &DefaultKernel, body: BodyId, chord: f64, edge_chord: f64) -> Result<Drawn3> {
    let m = meshed(k, body, chord)?;
    let n = k.get(body)?.faces.len();
    let mut faces: Vec<(Vec<[f64; 3]>, Vec<[f64; 3]>, Vec<[u32; 3]>)> = vec![(Vec::new(), Vec::new(), Vec::new()); n];
    let mut local: Vec<std::collections::BTreeMap<u32, u32>> = vec![std::collections::BTreeMap::new(); n];
    for (t, f) in m.indices.iter().zip(&m.triangle_faces) {
        let fi = usize::try_from(f.0).ok().filter(|&i| i < n).ok_or_else(|| KernelError::OperationFailed("a triangle of no face".into()))?;
        let (Some(out), Some(map)) = (faces.get_mut(fi), local.get_mut(fi)) else { continue };
        let mut tri = [0u32; 3];
        for (slot, &i) in tri.iter_mut().zip(t) {
            let (Some(p), Some(nm)) = (m.positions.get(i as usize), m.normals.get(i as usize)) else {
                return Err(KernelError::OperationFailed("a triangle past the mesh's points".into()));
            };
            *slot = *map.entry(i).or_insert_with(|| {
                out.0.push([p.x, p.y, p.z]);
                out.1.push([nm.x, nm.y, nm.z]);
                (out.0.len() - 1) as u32
            });
        }
        out.2.push(tri);
    }
    let edges_from = if edge_chord == chord { m } else { meshed(k, body, edge_chord)? };
    let edges = edges_from.edges.iter().map(|(_, pts)| pts.iter().map(|p| [p.x, p.y, p.z]).collect()).collect();
    Ok((faces, edges))
}
