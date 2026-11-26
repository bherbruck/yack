// yack: Which faces and edges of one solid come near which of the other's, from meshes of both. A mesh
// Adapted from springrep@a03e8a1 src/boolean/near.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/near.rs to yack-boolean (M3 W8); paths and visibility only;
// hardened (M3 B4): BTree sets and maps for the hash ones (each looked up, or walked only to fill
// another set), checked access in place of indexing.
//! Which faces and edges of one solid come near which of the other's, from meshes of both. A mesh
//! lies within its deflection of the exact geometry, so two faces or edges whose pieces stay
//! farther apart than both deflections and the tolerances cannot meet, and the boolean skips their
//! intersection without tracing or projecting anything. Only the faces whose boxes reach a face of
//! the other solid are meshed: a large part touched in one corner costs that corner.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    math::{Aabb, Point3},
    math::bvh::{Bvh, segment_triangle, segments},
    mesh::{
        self, MeshOptions,
    },
    topo::{Brep, CoedgeSpec, EdgeId, FaceId, VertexId},
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

/// How far apart two meshes' pieces may lie where the exact geometry meets, beyond the reach the
/// caller asks for: a point of the exact geometry lies within the deflection of its mesh piece, so
/// two meshes' pieces lie within both deflections of where the faces or edges meet, here taken
/// twice over for safety.
fn margin(deflections: [f64; 2], reach: f64) -> f64 {
    2.0 * (deflections[0] + deflections[1]) + reach
}

/// One solid's chosen faces as triangles and chosen edges as segments, each tagged with its face's
/// or edge's index in the arena's order.
struct Pieces {
    tris: Vec<(usize, [Point3; 3])>,
    segs: Vec<(usize, [Point3; 2])>,
    /// Chosen faces the mesher left without triangles, and chosen edges without a curve: near
    /// everything.
    bare_faces: BTreeSet<usize>,
    bare_edges: BTreeSet<usize>,
    deflection: f64,
}

impl Pieces {
    fn new(b: &Brep, faces: &[bool], edges: &[bool]) -> Option<Pieces> {
        let (part, index) = faces_of(b, faces);
        let mut bare_faces: BTreeSet<usize> = index.values().copied().collect();
        let (mut tris, mut deflection) = (Vec::new(), MESHES[0].linear);
        let mut opt = MESHES[0];
        if !index.is_empty() {
            let (m, used) = MESHES
                .iter()
                .find_map(|o| mesh::tessellate(&part, o).ok().map(|m| (m, *o)))?;
            (opt, deflection) = (used, used.linear);
            let at = |k: u32| m.positions.get(k as usize).copied();
            for (f, range) in &m.faces {
                let i = *index.get(f)?;
                for &[k0, k1, k2] in m.triangles.get(range.clone())? {
                    tris.push((i, [at(k0)?, at(k1)?, at(k2)?]));
                    bare_faces.remove(&i);
                }
            }
        }
        let mut segs = Vec::new();
        let mut bare_edges = BTreeSet::new();
        for (i, e) in b.edge_ids().enumerate().filter(|&(i, _)| edges.get(i) == Some(&true)) {
            let edge = b.edge(e);
            let Some(c) = &edge.curve else {
                bare_edges.insert(i);
                continue;
            };
            let ts = mesh::edge_params(c, edge.range.0, edge.range.1, &opt);
            for w in ts.windows(2) {
                if let &[t0, t1] = w {
                    segs.push((i, [c.eval(t0), c.eval(t1)]));
                }
            }
        }
        Some(Pieces {
            tris,
            segs,
            bare_faces,
            bare_edges,
            deflection,
        })
    }
}

/// A copy of the faces of `x` that `keep` marks, with their edges and vertices, and the index in
/// `x`'s order of the face each copy came from.
fn faces_of(x: &Brep, keep: &[bool]) -> (Brep, BTreeMap<FaceId, usize>) {
    let mut out = Brep::new();
    // all three looked up only: the copy is built in `x`'s face order
    let mut verts: BTreeMap<VertexId, VertexId> = BTreeMap::new();
    let mut edges: BTreeMap<EdgeId, EdgeId> = BTreeMap::new();
    let mut index = BTreeMap::new();
    let mut faces = Vec::new();
    for (i, f) in x.face_ids().enumerate().filter(|&(i, _)| keep.get(i) == Some(&true)) {
        let face = x.face(f);
        let nf = out.add_face(face.surface.clone(), face.reversed, face.names.clone());
        for &lp in &face.loops {
            let mut specs = Vec::new();
            for &c in &x.lp(lp).coedges {
                let co = x.coedge(c);
                let e = match edges.get(&co.edge) {
                    Some(&e) => e,
                    None => {
                        let edge = x.edge(co.edge);
                        let mut v = |id: VertexId| {
                            *verts.entry(id).or_insert_with(|| {
                                let nv = out.add_vertex(x.vertex(id).point);
                                out.set_vertex_tolerance(nv, x.vertex_tolerance(id));
                                nv
                            })
                        };
                        let (v0, v1) = (v(edge.start), v(edge.end));
                        let e = match &edge.curve {
                            Some(cv) => out.add_edge(cv.clone(), edge.range, v0, v1),
                            None => out.add_degenerate_edge(v0, edge.range),
                        };
                        out.set_edge_tolerance(e, x.edge_tolerance(co.edge));
                        edges.insert(co.edge, e);
                        e
                    }
                };
                specs.push(CoedgeSpec {
                    edge: e,
                    reversed: co.reversed,
                    pcurve: co.pcurve.clone(),
                });
            }
            out.add_loop(nf, specs);
        }
        index.insert(nf, i);
        faces.push(nf);
    }
    out.add_shell(faces);
    (out, index)
}

fn tri_box(t: &[Point3; 3]) -> Aabb {
    let mut b = Aabb::EMPTY;
    for &p in t {
        b.add(p);
    }
    b
}

fn seg_box(s: &[Point3; 2]) -> Aabb {
    let mut b = Aabb::EMPTY;
    b.add(s[0]);
    b.add(s[1]);
    b
}

/// A triangle's sides, in order round it.
fn sides(&[p, q, r]: &[Point3; 3]) -> [(Point3, Point3); 3] {
    [(p, q), (q, r), (r, p)]
}

fn tri_tri(a: &[Point3; 3], b: &[Point3; 3]) -> f64 {
    let mut d = f64::INFINITY;
    for ((p, q), (r, s)) in sides(a).into_iter().zip(sides(b)) {
        d = d.min(segment_triangle(p, q, b).0);
        d = d.min(segment_triangle(r, s, a).0);
    }
    d
}

/// Which entries of `own` lie within `d` of some box of `other`.
fn reaching(own: &[Aabb], other: &Bvh, d: f64) -> Vec<bool> {
    let mut found = Vec::new();
    own.iter()
        .map(|bb| {
            other.within(bb, d, &mut found);
            !found.is_empty()
        })
        .collect()
}

/// The pairs of the two solids' faces, edges, and edges against faces, that come within reach of
/// each other, by index in each arena's order.
pub(crate) struct Near {
    faces: BTreeSet<(usize, usize)>,
    edges: BTreeSet<(usize, usize)>,
    /// `(side, edge of that side, face of the other)`.
    edge_faces: BTreeSet<(usize, usize, usize)>,
    bare_faces: [BTreeSet<usize>; 2],
    bare_edges: [BTreeSet<usize>; 2],
    /// Each solid's faces and edges near anything of the other.
    touched_faces: [BTreeSet<usize>; 2],
    touched_edges: [BTreeSet<usize>; 2],
}

impl Near {
    /// The pairs within `reach` of each other beyond the meshes' deflections, given each solid's
    /// face and edge boxes in its arena's order; `None` where a solid cannot be meshed, when every
    /// pair counts as near.
    pub(crate) fn new(
        breps: [&Brep; 2],
        face_boxes: [&[Aabb]; 2],
        edge_boxes: [&[Aabb]; 2],
        reach: f64,
    ) -> Option<Near> {
        // Faces and edges whose boxes stay farther from every face of the other solid than the
        // coarsest meshes' margin meet nothing of it, and are not meshed.
        let widest = margin([MESHES[2].linear; 2], reach);
        let [faces_a, faces_b] = face_boxes;
        let [edges_a, edges_b] = edge_boxes;
        let [of_a, of_b] = face_boxes.map(|b| Bvh::of_boxes(b.to_vec()));
        let faces = [reaching(faces_a, &of_b, widest), reaching(faces_b, &of_a, widest)];
        let edges = [reaching(edges_a, &of_b, widest), reaching(edges_b, &of_a, widest)];
        let pa = Pieces::new(breps[0], &faces[0], &edges[0])?;
        let pb = Pieces::new(breps[1], &faces[1], &edges[1])?;
        let margin = margin([pa.deflection, pb.deflection], reach);
        let [tri_bvh_a, tri_bvh_b] = [&pa, &pb].map(|p| Bvh::of_boxes(p.tris.iter().map(|t| tri_box(&t.1)).collect()));
        let seg_bvh_b = Bvh::of_boxes(pb.segs.iter().map(|s| seg_box(&s.1)).collect());
        let mut found = Vec::new();

        let mut faces = BTreeSet::new();
        for (fa, ta) in &pa.tris {
            tri_bvh_b.within(&tri_box(ta), margin, &mut found);
            for (fb, tb) in found.iter().filter_map(|&j| pb.tris.get(j)) {
                if !faces.contains(&(*fa, *fb)) && tri_tri(ta, tb) <= margin {
                    faces.insert((*fa, *fb));
                }
            }
        }
        let mut edges = BTreeSet::new();
        for &(ea, [a0, a1]) in &pa.segs {
            seg_bvh_b.within(&seg_box(&[a0, a1]), margin, &mut found);
            for &(eb, [b0, b1]) in found.iter().filter_map(|&j| pb.segs.get(j)) {
                if !edges.contains(&(ea, eb)) && segments(a0, a1, b0, b1).0 <= margin {
                    edges.insert((ea, eb));
                }
            }
        }
        // Each side's edges against the other's triangles, the first solid's edges first.
        let mut edge_faces = BTreeSet::new();
        for (side, own, other, other_tris) in [(0, &pa, &pb, &tri_bvh_b), (1, &pb, &pa, &tri_bvh_a)] {
            for &(e, [s0, s1]) in &own.segs {
                other_tris.within(&seg_box(&[s0, s1]), margin, &mut found);
                for (f, t) in found.iter().filter_map(|&j| other.tris.get(j)) {
                    let key = (side, e, *f);
                    if !edge_faces.contains(&key) && segment_triangle(s0, s1, t).0 <= margin {
                        edge_faces.insert(key);
                    }
                }
            }
        }
        let [mut touched_faces_a, mut touched_faces_b] = [pa.bare_faces.clone(), pb.bare_faces.clone()];
        let [mut touched_edges_a, mut touched_edges_b] = [pa.bare_edges.clone(), pb.bare_edges.clone()];
        for &(fa, fb) in &faces {
            touched_faces_a.insert(fa);
            touched_faces_b.insert(fb);
        }
        for &(ea, eb) in &edges {
            touched_edges_a.insert(ea);
            touched_edges_b.insert(eb);
        }
        for &(side, e, f) in &edge_faces {
            if side == 0 {
                touched_edges_a.insert(e);
                touched_faces_b.insert(f);
            } else {
                touched_edges_b.insert(e);
                touched_faces_a.insert(f);
            }
        }
        // A bare face or edge counts as near everything, so the other solid's are all touched.
        let [brep_a, brep_b] = breps;
        for (other, b, touched_faces, touched_edges) in [
            (&pb, brep_a, &mut touched_faces_a, &mut touched_edges_a),
            (&pa, brep_b, &mut touched_faces_b, &mut touched_edges_b),
        ] {
            if !other.bare_faces.is_empty() || !other.bare_edges.is_empty() {
                touched_faces.extend(0..b.face_ids().count());
                touched_edges.extend(0..b.edge_ids().count());
            }
        }
        let touched_faces = [touched_faces_a, touched_faces_b];
        let touched_edges = [touched_edges_a, touched_edges_b];
        Some(Near {
            faces,
            edges,
            edge_faces,
            bare_faces: [pa.bare_faces, pb.bare_faces],
            bare_edges: [pa.bare_edges, pb.bare_edges],
            touched_faces,
            touched_edges,
        })
    }

    /// Whether face `fa` of the first solid and face `fb` of the second may meet.
    pub(crate) fn faces(&self, fa: usize, fb: usize) -> bool {
        self.faces.contains(&(fa, fb))
            || self.bare_faces[0].contains(&fa)
            || self.bare_faces[1].contains(&fb)
    }

    /// Whether edge `ea` of the first solid and edge `eb` of the second may meet.
    pub(crate) fn edges(&self, ea: usize, eb: usize) -> bool {
        self.edges.contains(&(ea, eb))
            || self.bare_edges[0].contains(&ea)
            || self.bare_edges[1].contains(&eb)
    }

    /// Whether face `f` of solid `side` stays clear of everything of the other.
    pub(crate) fn face_clear(&self, side: usize, f: usize) -> bool {
        self.touched_faces.get(side).is_some_and(|t| !t.contains(&f))
    }

    /// Whether edge `e` of solid `side` stays clear of everything of the other.
    pub(crate) fn edge_clear(&self, side: usize, e: usize) -> bool {
        self.touched_edges.get(side).is_some_and(|t| !t.contains(&e))
    }

    /// Whether edge `e` of solid `side` may meet face `f` of the other.
    pub(crate) fn edge_face(&self, side: usize, e: usize, f: usize) -> bool {
        // a side that is neither solid's counts as near everything
        let other = 1usize.checked_sub(side);
        self.edge_faces.contains(&(side, e, f))
            || self.bare_edges.get(side).is_none_or(|b| b.contains(&e))
            || other.and_then(|o| self.bare_faces.get(o)).is_none_or(|b| b.contains(&f))
    }
}
