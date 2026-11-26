// yack: the boolean's working state: vertices, trims, faces' domains and the near-pair queries.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only.
//! The boolean's working state: vertices, trims, faces' domains and the near-pair queries.

use super::*;
use super::bounds::{cell, overlap, padded, curve_bbox, face_bbox};

#[allow(clippy::disallowed_types)] impl<'a> Work<'a> { // D26: BTree* in hardening, proven by digests
    pub(super) fn new(breps: [&'a Brep; 2]) -> Work<'a> {
        let mut w = Work {
            breps,
            pool: Vec::new(),
            vtol: Vec::new(),
            widest: tol::LINEAR,
            tolerant: Vec::new(),
            cap: tol::LINEAR,
            grid: HashMap::new(),
            vmap: [Vec::new(), Vec::new()],
            faces: Vec::new(),
            face_of: [Vec::new(), Vec::new()],
            bases: Vec::new(),
            edge_base: [Vec::new(), Vec::new()],
            coincident: Vec::new(),
            pieces: Vec::new(),
            base_pieces: Vec::new(),
            cut_faces: Vec::new(),
            canon: Vec::new(),
            hits: Vec::new(),
            on_face: BTreeSet::new(),
            inputs: 0,
            met: Vec::new(),
            touch_points: Vec::new(),
            unplaced: HashMap::new(),
            meetings: HashMap::new(),
            near: None,
        };
        for b in breps {
            let (vs, es) = b.tolerant();
            for t in vs.iter().map(|x| x.1).chain(es.iter().map(|x| x.1)) {
                w.cap = w.cap.max(t);
            }
        }
        for (side, &b) in breps.iter().enumerate() {
            w.vmap[side] = b
                .vertex_ids()
                .map(|v| w.vertex_with(b.vertex(v).point, b.vertex_tolerance(v)))
                .collect();
            for e in b.edge_ids() {
                let edge = b.edge(e);
                let curve = edge.curve.clone().map(Rc::new);
                let bbox = match &curve {
                    Some(c) => curve_bbox(c, edge.range),
                    None => {
                        let mut bb = Aabb::EMPTY;
                        bb.add(b.vertex(edge.start).point);
                        bb
                    }
                };
                w.edge_base[side].push(w.bases.len());
                w.bases.push(Base {
                    curve,
                    range: edge.range,
                    v0: Some(w.vmap[side][edge.start.index()]),
                    v1: Some(w.vmap[side][edge.end.index()]),
                    pole: b.vertex(edge.start).point,
                    closed: edge.start == edge.end,
                    bbox,
                    kind: BaseKind::Edge,
                    tol: b.edge_tolerance(e),
                });
            }
            for f in b.face_ids() {
                let (mut dom, mut bbox) = w.original_domain(side, f);
                if b.has_tolerances() {
                    let tols: Vec<Vec<f64>> = b
                        .face(f)
                        .loops
                        .iter()
                        .map(|&lp| {
                            let cs = &b.lp(lp).coedges;
                            cs.iter()
                                .map(|&c| b.edge_tolerance(b.coedge(c).edge))
                                .collect()
                        })
                        .collect();
                    dom.set_tolerances(tols);
                    bbox = padded(&bbox, dom.widest());
                }
                w.face_of[side].push(w.faces.len());
                w.faces.push(Face {
                    side,
                    id: f,
                    dom,
                    bbox,
                });
            }
        }
        w.inputs = w.pool.len();
        w
    }

    pub(super) fn vertex(&mut self, p: Point3) -> usize {
        self.vertex_with(p, tol::LINEAR)
    }

    /// The pool vertex that `p`, with tolerance `t`, coincides with, or a new one.
    pub(super) fn vertex_with(&mut self, p: Point3, t: f64) -> usize {
        if let Some(i) = self.coincident_vertex(p, t) {
            return i;
        }
        self.pool.push(p);
        self.vtol.push(t);
        self.widest = self.widest.max(t);
        let i = self.pool.len() - 1;
        if t > tol::LINEAR {
            self.tolerant.push(i);
        }
        self.grid.entry(cell(p)).or_default().push(i);
        i
    }

    /// The nearest pool vertex within the sum of its tolerance and `t` of `p` (D. J. Jackson,
    /// "Boundary representation modelling with local tolerances", Solid Modeling '95; "Jackson
    /// 1995" below).
    /// Where either is tolerant, the vertex's tolerance grows to hold `p`'s sphere, as every curve
    /// that ends at `p` must end within it, but never past the inputs' widest.
    pub(super) fn coincident_vertex(&mut self, p: Point3, t: f64) -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        let mut consider = |i: usize| {
            let d = self.pool[i].dist(p);
            if d <= tol::coincidence(t, self.vtol[i]) && best.is_none_or(|b| d < b.0) {
                best = Some((d, i));
            }
        };
        // Exact pool vertices lie within `t` of `p`; tolerant ones are searched one by one.
        let reach = (t / CELL).ceil().max(1.0) as i64;
        if (2 * reach + 1).pow(3) as usize > self.pool.len() {
            (0..self.pool.len()).for_each(&mut consider);
        } else {
            let c = cell(p);
            for dx in -reach..=reach {
                for dy in -reach..=reach {
                    for dz in -reach..=reach {
                        if let Some(ids) = self.grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                            ids.iter().for_each(|&i| consider(i));
                        }
                    }
                }
            }
            self.tolerant.iter().for_each(|&i| consider(i));
        }
        let (d, i) = best?;
        if t > tol::LINEAR || self.vtol[i] > tol::LINEAR {
            self.vtol[i] = self.vtol[i].max(d + t).min(self.cap);
            if !self.tolerant.contains(&i) {
                self.tolerant.push(i);
            }
            self.widest = self.widest.max(self.vtol[i]);
        }
        Some(i)
    }

    /// A trim for an input coedge, whole.
    pub(super) fn coedge_trim(&self, side: usize, c: CoedgeId, piece: usize) -> Trim {
        let b = self.breps[side];
        let ts = coedge_params(b, c, 16);
        let uvs = coedge_uv_at(b, c, &ts);
        self.trim_of(
            side,
            c,
            piece,
            (ts[0], ts[ts.len() - 1]),
            (uvs[0], uvs[uvs.len() - 1]),
        )
    }

    /// A trim for an input coedge from its parameter range and parameter-plane ends.
    pub(super) fn trim_of(
        &self,
        side: usize,
        c: CoedgeId,
        piece: usize,
        t: (f64, f64),
        uv: (Vec2, Vec2),
    ) -> Trim {
        let b = self.breps[side];
        let base = &self.bases[self.edge_base[side][b.coedge(c).edge.index()]];
        let v = (
            self.vmap[side][b.coedge_start(c).index()],
            self.vmap[side][b.coedge_end(c).index()],
        );
        coedge_trim_with(b, c, piece, base.curve.clone(), t, uv, v)
    }

    /// An input face's domain and box, from its memo when an earlier boolean worked them out.
    pub(super) fn original_domain(&self, side: usize, f: FaceId) -> (Domain, Aabb) {
        let b = self.breps[side];
        let face = b.face(f);
        let coedges = |lp: &crate::topo::LoopId| b.lp(*lp).coedges.iter();
        if let Some(img) = b.memo(f).slot::<FaceImages>() {
            let loops = face
                .loops
                .iter()
                .zip(&img.trims)
                .map(|(lp, ims)| {
                    coedges(lp)
                        .zip(ims)
                        .map(|(&c, im)| self.trim_of(side, c, usize::MAX, im.t, im.uv))
                        .collect()
                })
                .collect();
            let dom = Domain::from_samples(
                face.surface.clone(),
                face.reversed,
                loops,
                img.samples.clone(),
                img.boxes.clone(),
            );
            return (dom, img.bbox);
        }
        let loops: Vec<Vec<Trim>> = face
            .loops
            .iter()
            .map(|lp| {
                coedges(lp)
                    .map(|&c| self.coedge_trim(side, c, usize::MAX))
                    .collect()
            })
            .collect();
        let ends = face
            .loops
            .iter()
            .map(|lp| {
                coedges(lp)
                    .map(|&c| {
                        let (t0, t1) = b.edge(b.coedge(c).edge).range;
                        let ts = if b.coedge(c).reversed {
                            [t1, t0]
                        } else {
                            [t0, t1]
                        };
                        let uvs = coedge_uv_at(b, c, &ts);
                        (uvs[0], uvs[1])
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let trims = loops
            .iter()
            .zip(&ends)
            .map(|(l, e)| {
                l.iter()
                    .zip(e)
                    .map(|(tr, &whole)| TrimImage {
                        t: (tr.t0, tr.t1),
                        uv: (tr.uv0, tr.uv1),
                        whole,
                    })
                    .collect()
            })
            .collect();
        let dom = Domain::new(face.surface.clone(), face.reversed, loops);
        let bbox = face_bbox(&dom);
        b.memo(f).set_slot(FaceImages {
            trims,
            samples: dom.samples().to_vec(),
            boxes: dom.boxes().to_vec(),
            bbox,
        });
        (dom, bbox)
    }

    /// `e`, saying which faces it arose on: one boolean of many solids otherwise fails without a
    /// trace of where.
    pub(super) fn at_faces(&self, e: Error, faces: &[usize]) -> Error {
        let named: Vec<String> = faces
            .iter()
            .map(|&f| {
                let face = &self.faces[f];
                let c = (face.bbox.min + face.bbox.max) * 0.5;
                let b = self.breps[face.side];
                let solid = b
                    .face(face.id)
                    .shell
                    .and_then(|sh| b.shell(sh).solid)
                    .map_or(String::new(), |s| format!(" of solid {}", s.index()));
                format!(
                    "{} face {}{solid} of input {} about ({:.4}, {:.4}, {:.4})",
                    face.dom.surface.kind(),
                    face.id.index(),
                    face.side,
                    c.x,
                    c.y,
                    c.z
                )
            })
            .collect();
        let at = named.join(" and ");
        match e {
            Error::Numerical(m) => Error::Numerical(format!("{m}, on the {at}")),
            other => other,
        }
    }

    pub(super) fn other(&self, side: usize) -> &[usize] {
        &self.face_of[1 - side]
    }

    /// Step 1a: where each edge of one solid crosses a face of the other, kept to be placed once
    /// every other meeting point is known.
    pub(super) fn find_near(&mut self) {
        let faces = [0, 1].map(|s| -> Vec<Aabb> {
            self.face_of[s]
                .iter()
                .map(|&f| self.faces[f].bbox)
                .collect()
        });
        let edges = [0, 1].map(|s| -> Vec<Aabb> {
            self.edge_base[s]
                .iter()
                .map(|&e| self.bases[e].bbox)
                .collect()
        });
        self.near = near::Near::new(
            self.breps,
            [&faces[0], &faces[1]],
            [&edges[0], &edges[1]],
            2.0 * self.cap + csi::GRAZE_GAP,
        );
    }

    /// Every step of the boolean after setting up.
    pub(super) fn finish(mut self, op: Op, facts: &Facts) -> Result<Brep> {
        self.take_facts(facts)?;
        self.meet()?;
        self.split_curves();
        self.imprint_on_faces();
        self.dedup();
        let (frags, poles, intact) = self.split_faces()?;
        let mut kept = self.select(frags, &intact, op)?;
        let mut geoms = self.piece_geoms();
        geoms.extend(poles);
        self.touch_loops(&mut kept, &mut geoms);
        sew::assemble(kept, &self.pool, &self.vtol, &geoms)
    }

    /// Whether the face pairs or edge pairs whose boxes overlap are many enough that meshing both
    /// inputs to rule most of them out costs less than the exact work on all of them.
    pub(super) fn many_pairs(&self) -> bool {
        let count = |a: Vec<&Aabb>, b: Vec<&Aabb>, enough: usize| {
            let mut n = 0;
            for x in &a {
                n += b.iter().filter(|y| overlap(x, y)).count();
                if n >= enough {
                    return true;
                }
            }
            false
        };
        let faces = |side: usize| {
            self.face_of[side]
                .iter()
                .map(|&f| &self.faces[f].bbox)
                .collect()
        };
        let edges = |side: usize| {
            self.edge_base[side]
                .iter()
                .map(|&e| &self.bases[e].bbox)
                .collect()
        };
        count(faces(0), faces(1), MANY_FACE_PAIRS) || count(edges(0), edges(1), MANY_EDGE_PAIRS)
    }

    /// For each face that stays clear of the other solid, a representative of the faces joined to
    /// it through edges that stay clear too: crossing such an edge crosses nothing of the other
    /// solid, so every face of one group lies on the same side of it. `None` for the rest, and for
    /// every face where nothing is known of what is near.
    pub(super) fn clear_groups(&self) -> Vec<Option<usize>> {
        let n = self.faces.len();
        let Some(near) = &self.near else {
            return vec![None; n];
        };
        let clear: Vec<bool> = (0..n)
            .map(|fi| {
                let side = self.faces[fi].side;
                near.face_clear(side, fi - self.face_of[side][0])
            })
            .collect();
        fn root(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        let mut parent: Vec<usize> = (0..n).collect();
        for side in 0..2 {
            let b = self.breps[side];
            let mut users: Vec<Vec<usize>> = vec![Vec::new(); self.edge_base[side].len()];
            for (k, f) in b.face_ids().enumerate() {
                let fi = self.face_of[side][k];
                for &lp in &b.face(f).loops {
                    for &c in &b.lp(lp).coedges {
                        users[b.coedge(c).edge.index()].push(fi);
                    }
                }
            }
            for (e, faces) in users.iter().enumerate() {
                if !near.edge_clear(side, e) {
                    continue;
                }
                for w in faces.windows(2) {
                    if clear[w[0]] && clear[w[1]] {
                        let (x, y) = (root(&mut parent, w[0]), root(&mut parent, w[1]));
                        parent[x] = y;
                    }
                }
            }
        }
        (0..n)
            .map(|fi| clear[fi].then(|| root(&mut parent, fi)))
            .collect()
    }

    /// Whether faces `fa` and `fb`, of the first solid and the second, may meet.
    pub(super) fn faces_near(&self, fa: usize, fb: usize) -> bool {
        self.near
            .as_ref()
            .is_none_or(|n| n.faces(fa - self.face_of[0][0], fb - self.face_of[1][0]))
    }

    /// Whether edge bases `ba` and `bb`, of the first solid and the second, may meet.
    pub(super) fn edges_near(&self, ba: usize, bb: usize) -> bool {
        self.near
            .as_ref()
            .is_none_or(|n| n.edges(ba - self.edge_base[0][0], bb - self.edge_base[1][0]))
    }

    /// Whether edge base `bi` of solid `side` may meet face `fi` of the other.
    pub(super) fn edge_face_near(&self, side: usize, bi: usize, fi: usize) -> bool {
        self.near.as_ref().is_none_or(|n| {
            n.edge_face(
                side,
                bi - self.edge_base[side][0],
                fi - self.face_of[1 - side][0],
            )
        })
    }
}
