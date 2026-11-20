// yack: Which faces and edges of one solid come near which of the other's, from meshes of both. A mesh
// Adapted from springrep@a03e8a1 src/boolean/near.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! Which faces and edges of one solid come near which of the other's, from meshes of both. A mesh
//! lies within its deflection of the exact geometry, so two faces or edges whose pieces stay
//! farther apart than both deflections and the tolerances cannot meet, and the boolean skips their
//! intersection without tracing or projecting anything. Only the faces whose boxes reach a face of
//! the other solid are meshed: a large part touched in one corner costs that corner.

use std::collections::{HashMap, HashSet};

use crate::{
    math::{Aabb, Point3},
    mesh::{
        self, MeshOptions,
        bvh::{Bvh, segment_triangle, segments},
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
    bare_faces: HashSet<usize>,
    bare_edges: HashSet<usize>,
    deflection: f64,
}

impl Pieces {
    fn new(b: &Brep, faces: &[bool], edges: &[bool]) -> Option<Pieces> {
        let (part, index) = faces_of(b, faces);
        let mut bare_faces: HashSet<usize> = index.values().copied().collect();
        let (mut tris, mut deflection) = (Vec::new(), MESHES[0].linear);
        let mut opt = MESHES[0];
        if !index.is_empty() {
            let (m, used) = MESHES
                .iter()
                .find_map(|o| mesh::tessellate(&part, o).ok().map(|m| (m, *o)))?;
            (opt, deflection) = (used, used.linear);
            for (f, range) in &m.faces {
                let i = index[f];
                for t in &m.triangles[range.clone()] {
                    tris.push((i, t.map(|k| m.positions[k as usize])));
                    bare_faces.remove(&i);
                }
            }
        }
        let mut segs = Vec::new();
        let mut bare_edges = HashSet::new();
        for (i, e) in b.edge_ids().enumerate().filter(|&(i, _)| edges[i]) {
            let edge = b.edge(e);
            let Some(c) = &edge.curve else {
                bare_edges.insert(i);
                continue;
            };
            let ts = mesh::edge_params(c, edge.range.0, edge.range.1, &opt);
            for w in ts.windows(2) {
                segs.push((i, [c.eval(w[0]), c.eval(w[1])]));
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
fn faces_of(x: &Brep, keep: &[bool]) -> (Brep, HashMap<FaceId, usize>) {
    let mut out = Brep::new();
    let mut verts: HashMap<VertexId, VertexId> = HashMap::new();
    let mut edges: HashMap<EdgeId, EdgeId> = HashMap::new();
    let mut index = HashMap::new();
    let mut faces = Vec::new();
    for (i, f) in x.face_ids().enumerate().filter(|&(i, _)| keep[i]) {
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

fn tri_tri(a: &[Point3; 3], b: &[Point3; 3]) -> f64 {
    let mut d = f64::INFINITY;
    for k in 0..3 {
        d = d.min(segment_triangle(a[k], a[(k + 1) % 3], b).0);
        d = d.min(segment_triangle(b[k], b[(k + 1) % 3], a).0);
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
    faces: HashSet<(usize, usize)>,
    edges: HashSet<(usize, usize)>,
    /// `(side, edge of that side, face of the other)`.
    edge_faces: HashSet<(usize, usize, usize)>,
    bare_faces: [HashSet<usize>; 2],
    bare_edges: [HashSet<usize>; 2],
    /// Each solid's faces and edges near anything of the other.
    touched_faces: [HashSet<usize>; 2],
    touched_edges: [HashSet<usize>; 2],
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
        let other = face_boxes.map(|b| Bvh::of_boxes(b.to_vec()));
        let faces = [0, 1].map(|s| reaching(face_boxes[s], &other[1 - s], widest));
        let edges = [0, 1].map(|s| reaching(edge_boxes[s], &other[1 - s], widest));
        let pa = Pieces::new(breps[0], &faces[0], &edges[0])?;
        let pb = Pieces::new(breps[1], &faces[1], &edges[1])?;
        let margin = margin([pa.deflection, pb.deflection], reach);
        let ps = [&pa, &pb];
        let tri_bvh = ps.map(|p| Bvh::of_boxes(p.tris.iter().map(|t| tri_box(&t.1)).collect()));
        let seg_bvh = ps.map(|p| Bvh::of_boxes(p.segs.iter().map(|s| seg_box(&s.1)).collect()));
        let mut found = Vec::new();

        let mut faces = HashSet::new();
        for (fa, ta) in &pa.tris {
            tri_bvh[1].within(&tri_box(ta), margin, &mut found);
            for &j in &found {
                let (fb, tb) = &pb.tris[j];
                if !faces.contains(&(*fa, *fb)) && tri_tri(ta, tb) <= margin {
                    faces.insert((*fa, *fb));
                }
            }
        }
        let mut edges = HashSet::new();
        for (ea, sa) in &pa.segs {
            seg_bvh[1].within(&seg_box(sa), margin, &mut found);
            for &j in &found {
                let (eb, sb) = &pb.segs[j];
                if !edges.contains(&(*ea, *eb)) && segments(sa[0], sa[1], sb[0], sb[1]).0 <= margin
                {
                    edges.insert((*ea, *eb));
                }
            }
        }
        let mut edge_faces = HashSet::new();
        for side in 0..2 {
            let (own, other) = (ps[side], ps[1 - side]);
            for (e, s) in &own.segs {
                tri_bvh[1 - side].within(&seg_box(s), margin, &mut found);
                for &j in &found {
                    let (f, t) = &other.tris[j];
                    let key = (side, *e, *f);
                    if !edge_faces.contains(&key) && segment_triangle(s[0], s[1], t).0 <= margin {
                        edge_faces.insert(key);
                    }
                }
            }
        }
        let mut touched_faces = [pa.bare_faces.clone(), pb.bare_faces.clone()];
        let mut touched_edges = [pa.bare_edges.clone(), pb.bare_edges.clone()];
        for &(fa, fb) in &faces {
            touched_faces[0].insert(fa);
            touched_faces[1].insert(fb);
        }
        for &(ea, eb) in &edges {
            touched_edges[0].insert(ea);
            touched_edges[1].insert(eb);
        }
        for &(side, e, f) in &edge_faces {
            touched_edges[side].insert(e);
            touched_faces[1 - side].insert(f);
        }
        // A bare face or edge counts as near everything, so the other solid's are all touched.
        for side in 0..2 {
            if !ps[1 - side].bare_faces.is_empty() || !ps[1 - side].bare_edges.is_empty() {
                touched_faces[side].extend(0..breps[side].face_ids().count());
                touched_edges[side].extend(0..breps[side].edge_ids().count());
            }
        }
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
        !self.touched_faces[side].contains(&f)
    }

    /// Whether edge `e` of solid `side` stays clear of everything of the other.
    pub(crate) fn edge_clear(&self, side: usize, e: usize) -> bool {
        !self.touched_edges[side].contains(&e)
    }

    /// Whether edge `e` of solid `side` may meet face `f` of the other.
    pub(crate) fn edge_face(&self, side: usize, e: usize, f: usize) -> bool {
        self.edge_faces.contains(&(side, e, f))
            || self.bare_edges[side].contains(&e)
            || self.bare_faces[1 - side].contains(&f)
    }
}
