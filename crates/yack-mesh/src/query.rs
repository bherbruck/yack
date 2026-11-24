// yack: rays and distances over a solid, meshed here and handed to them (M2: they mesh nothing).
// Adapted from springrep@a03e8a1 src/rays.rs and src/distance.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the meshing `Rays::new`, `distance::closest` and `Nearest`'s constructors did, moved up
// here unchanged (one MESHES table, finest first), so rays and distance sit below the mesher; then to yack-mesh (M3 W6), paths and visibility only.
//! A solid's ray caster and nearest-point searches, each over a mesh made here: the finest of
//! [`MESHES`] the mesher manages, seeded to the solid's size.

use super::{Mesh, MeshOptions, seeding, tessellate};
use crate::{
    distance::{self, Closest, Nearest, Selection, Side},
    error::Result,
    rays::Rays,
    topo::Brep,
};

/// The meshes tried, finest first: the mesher gives up on a very fine chord over a large part.
const MESHES: [MeshOptions; 3] = [
    MeshOptions {
        linear: 0.01,
        angular: 0.2,
    },
    MeshOptions {
        linear: 0.05,
        angular: 0.5,
    },
    MeshOptions {
        linear: 0.2,
        angular: 0.8,
    },
];

/// The finest of [`MESHES`] the mesher manages for `b`, with the options it was made with.
#[allow(clippy::panic)] fn mesh_for(b: &Brep) -> Result<(Mesh, MeshOptions)> {
    let mut err = None;
    for opt in MESHES.iter().map(|o| seeding(o, b)) {
        match tessellate(b, &opt) {
            Ok(m) => return Ok((m, opt)),
            Err(e) => err = Some(e),
        }
    }
    Err(err.unwrap_or_else(|| panic!("a mesh was tried")))
}

/// The ray caster over `b`'s faces.
pub fn rays(b: &Brep) -> Result<Rays<'_>> {
    let (m, opt) = mesh_for(b)?;
    Ok(Rays::from_tessellation(b, &m.positions, &m.triangles, &m.faces, opt.linear))
}

/// The side of `b` that `sel` names, over `b`'s mesh.
fn side<'b>(b: &'b Brep, sel: &Selection) -> Result<Side<'b>> {
    let (m, opt) = mesh_for(b)?;
    Ok(Side::from_tessellation(b, sel, &m.positions, &m.triangles, &m.faces, &opt))
}

/// The closest distance between selection `sa` of `a` and `sb` of `b`, and a point of each at it.
pub fn closest(a: &Brep, sa: &Selection, b: &Brep, sb: &Selection) -> Result<Closest> {
    distance::between(&side(a, sa)?, &side(b, sb)?)
}

/// The arena's boundary alone: a point inside a solid is as far as its nearest wall.
pub fn nearest_boundary(b: &Brep) -> Result<Nearest<'_>> {
    let faces = Selection {
        faces: b.face_ids().collect(),
        ..Selection::default()
    };
    Ok(Nearest::from_side(side(b, &faces)?, side))
}

/// The faces, edges, vertices and solids `selection` names: the nearest point of one face, say,
/// wherever the rest of the arena lies.
pub fn nearest_of<'b>(b: &'b Brep, selection: &Selection) -> Result<Nearest<'b>> {
    Ok(Nearest::from_side(side(b, selection)?, side))
}

/// The arena's solids, insides counted in: a point a solid holds is its own nearest point.
pub fn nearest_solid(b: &Brep) -> Result<Nearest<'_>> {
    Ok(Nearest::from_side(side(b, &Selection::body(b))?, side))
}
