// yack: the boolean's working state: vertices, trims, faces' domains and the near-pair queries.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only; hardened (M3 B12): checked access in place of indexing (the
// working state's constructor and its trim builders return a Result), BTreeMaps for its maps.
//! The boolean's working state: vertices, trims, faces' domains and the near-pair queries.

use super::*;
use super::bounds::{cell, overlap, padded, curve_bbox, face_bbox};

/// A piece of the working state that is not there: a bug in the steps before.
fn missing(what: &str) -> Error {
    Error::Numerical(format!("boolean: no {what} at that index"))
}

/// The root of `i` in the union-find `parent`, halving the path on the way.
fn root(parent: &mut [usize], mut i: usize) -> usize {
    while let Some(&up) = parent.get(i)
        && up != i
    {
        let grand = parent.get(up).copied().unwrap_or(up);
        if let Some(slot) = parent.get_mut(i) {
            *slot = grand;
        }
        i = grand;
    }
    i
}

impl<'a> Work<'a> {
    pub(super) fn new(breps: [&'a Brep; 2]) -> Result<Work<'a>> {
        let mut w = Work {
            breps,
            pool: Vec::new(),
            vtol: Vec::new(),
            widest: tol::LINEAR,
            tolerant: Vec::new(),
            cap: tol::LINEAR,
            grid: BTreeMap::new(),
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
            unplaced: BTreeMap::new(),
            meetings: BTreeMap::new(),
            near: None,
        };
        for b in breps {
            let (vs, es) = b.tolerant();
            for t in vs.iter().map(|x| x.1).chain(es.iter().map(|x| x.1)) {
                w.cap = w.cap.max(t);
            }
        }
        for (side, &b) in breps.iter().enumerate() {
            let ids: Vec<usize> = b
                .vertex_ids()
                .map(|v| w.vertex_with(b.vertex(v).point, b.vertex_tolerance(v)))
                .collect();
            *w.vmap.get_mut(side).ok_or_else(|| missing("solid"))? = ids;
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
                let next = w.bases.len();
                w.edge_base.get_mut(side).ok_or_else(|| missing("solid"))?.push(next);
                let vmap = w.vmap.get(side).ok_or_else(|| missing("solid"))?;
                let (v0, v1) = (vmap.get(edge.start.index()).copied(), vmap.get(edge.end.index()).copied());
                if v0.is_none() || v1.is_none() {
                    return Err(missing("vertex"));
                }
                w.bases.push(Base {
                    curve,
                    range: edge.range,
                    v0,
                    v1,
                    pole: b.vertex(edge.start).point,
                    closed: edge.start == edge.end,
                    bbox,
                    kind: BaseKind::Edge,
                    tol: b.edge_tolerance(e),
                });
            }
            for f in b.face_ids() {
                let (mut dom, mut bbox) = w.original_domain(side, f)?;
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
                let next = w.faces.len();
                w.face_of.get_mut(side).ok_or_else(|| missing("solid"))?.push(next);
                w.faces.push(Face {
                    side,
                    id: f,
                    dom,
                    bbox,
                });
            }
        }
        w.inputs = w.pool.len();
        Ok(w)
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
            let (Some(q), Some(&vt)) = (self.pool.get(i), self.vtol.get(i)) else {
                return;
            };
            let d = q.dist(p);
            if d <= tol::coincidence(t, vt) && best.is_none_or(|b| d < b.0) {
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
        let vt = *self.vtol.get(i)?;
        if t > tol::LINEAR || vt > tol::LINEAR {
            let grown = vt.max(d + t).min(self.cap);
            *self.vtol.get_mut(i)? = grown;
            if !self.tolerant.contains(&i) {
                self.tolerant.push(i);
            }
            self.widest = self.widest.max(grown);
        }
        Some(i)
    }

    /// A trim for an input coedge, whole.
    pub(super) fn coedge_trim(&self, side: usize, c: CoedgeId, piece: usize) -> Result<Trim> {
        let b = *self.breps.get(side).ok_or_else(|| missing("solid"))?;
        let ts = coedge_params(b, c, 16);
        let uvs = coedge_uv_at(b, c, &ts);
        let (Some(&t0), Some(&t1), Some(&uv0), Some(&uv1)) = (ts.first(), ts.last(), uvs.first(), uvs.last()) else {
            return Err(missing("coedge sample"));
        };
        self.trim_of(side, c, piece, (t0, t1), (uv0, uv1))
    }

    /// A trim for an input coedge from its parameter range and parameter-plane ends.
    pub(super) fn trim_of(
        &self,
        side: usize,
        c: CoedgeId,
        piece: usize,
        t: (f64, f64),
        uv: (Vec2, Vec2),
    ) -> Result<Trim> {
        let b = *self.breps.get(side).ok_or_else(|| missing("solid"))?;
        let base = self
            .edge_base
            .get(side)
            .and_then(|bases| bases.get(b.coedge(c).edge.index()))
            .and_then(|&bi| self.bases.get(bi))
            .ok_or_else(|| missing("edge base"))?;
        let vmap = self.vmap.get(side).ok_or_else(|| missing("solid"))?;
        let (Some(&v0), Some(&v1)) = (vmap.get(b.coedge_start(c).index()), vmap.get(b.coedge_end(c).index())) else {
            return Err(missing("vertex"));
        };
        Ok(coedge_trim_with(b, c, piece, base.curve.clone(), t, uv, (v0, v1)))
    }

    /// An input face's domain and box, from its memo when an earlier boolean worked them out.
    pub(super) fn original_domain(&self, side: usize, f: FaceId) -> Result<(Domain, Aabb)> {
        let b = *self.breps.get(side).ok_or_else(|| missing("solid"))?;
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
                .collect::<Result<_>>()?;
            let dom = Domain::from_samples(
                face.surface.clone(),
                face.reversed,
                loops,
                img.samples.clone(),
                img.boxes.clone(),
            );
            return Ok((dom, img.bbox));
        }
        let loops: Vec<Vec<Trim>> = face
            .loops
            .iter()
            .map(|lp| {
                coedges(lp)
                    .map(|&c| self.coedge_trim(side, c, usize::MAX))
                    .collect()
            })
            .collect::<Result<_>>()?;
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
                        match coedge_uv_at(b, c, &ts).as_slice() {
                            &[uv0, uv1, ..] => Ok((uv0, uv1)),
                            _ => Err(missing("coedge sample")),
                        }
                    })
                    .collect::<Result<Vec<_>>>()
            })
            .collect::<Result<Vec<_>>>()?;
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
        Ok((dom, bbox))
    }

    /// `e`, saying which faces it arose on: one boolean of many solids otherwise fails without a
    /// trace of where.
    pub(super) fn at_faces(&self, e: Error, faces: &[usize]) -> Error {
        let named: Vec<String> = faces
            .iter()
            .filter_map(|&f| {
                let face = self.faces.get(f)?;
                let c = (face.bbox.min + face.bbox.max) * 0.5;
                let b = *self.breps.get(face.side)?;
                let solid = b
                    .face(face.id)
                    .shell
                    .and_then(|sh| b.shell(sh).solid)
                    .map_or(String::new(), |s| format!(" of solid {}", s.index()));
                Some(format!(
                    "{} face {}{solid} of input {} about ({:.4}, {:.4}, {:.4})",
                    face.dom.surface.kind(),
                    face.id.index(),
                    face.side,
                    c.x,
                    c.y,
                    c.z
                ))
            })
            .collect();
        let at = named.join(" and ");
        match e {
            Error::Numerical(m) => Error::Numerical(format!("{m}, on the {at}")),
            other => other,
        }
    }

    pub(super) fn other(&self, side: usize) -> &[usize] {
        self.face_of.get(1 - side).map_or(&[], Vec::as_slice)
    }

    /// Step 1a: where each edge of one solid crosses a face of the other, kept to be placed once
    /// every other meeting point is known.
    pub(super) fn find_near(&mut self) {
        let [faces_a, faces_b] = self.face_of.each_ref().map(|ids| -> Vec<Aabb> {
            ids.iter().filter_map(|&f| self.faces.get(f)).map(|f| f.bbox).collect()
        });
        let [edges_a, edges_b] = self.edge_base.each_ref().map(|ids| -> Vec<Aabb> {
            ids.iter().filter_map(|&e| self.bases.get(e)).map(|b| b.bbox).collect()
        });
        self.near = near::Near::new(
            self.breps,
            [&faces_a, &faces_b],
            [&edges_a, &edges_b],
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
            self.face_of.get(side).into_iter().flatten().filter_map(|&f| self.faces.get(f)).map(|f| &f.bbox).collect()
        };
        let edges = |side: usize| {
            self.edge_base.get(side).into_iter().flatten().filter_map(|&e| self.bases.get(e)).map(|b| &b.bbox).collect()
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
        let clear: Vec<bool> = self
            .faces
            .iter()
            .enumerate()
            .map(|(fi, face)| {
                let first = self.face_of.get(face.side).and_then(|ids| ids.first());
                first.is_some_and(|&f0| near.face_clear(face.side, fi - f0))
            })
            .collect();
        let is_clear = |f: usize| clear.get(f) == Some(&true);
        let mut parent: Vec<usize> = (0..n).collect();
        for (side, ((&b, own_faces), own_bases)) in self.breps.iter().zip(&self.face_of).zip(&self.edge_base).enumerate() {
            let mut users: Vec<Vec<usize>> = vec![Vec::new(); own_bases.len()];
            for (f, &fi) in b.face_ids().zip(own_faces) {
                for &lp in &b.face(f).loops {
                    for &c in &b.lp(lp).coedges {
                        if let Some(u) = users.get_mut(b.coedge(c).edge.index()) {
                            u.push(fi);
                        }
                    }
                }
            }
            for (e, faces) in users.iter().enumerate() {
                if !near.edge_clear(side, e) {
                    continue;
                }
                for w in faces.windows(2) {
                    if let &[a, c] = w
                        && is_clear(a)
                        && is_clear(c)
                    {
                        let (x, y) = (root(&mut parent, a), root(&mut parent, c));
                        if let Some(up) = parent.get_mut(x) {
                            *up = y;
                        }
                    }
                }
            }
        }
        clear.iter().enumerate().map(|(fi, &c)| c.then(|| root(&mut parent, fi))).collect()
    }

    /// Whether faces `fa` and `fb`, of the first solid and the second, may meet.
    pub(super) fn faces_near(&self, fa: usize, fb: usize) -> bool {
        self.near
            .as_ref()
            .is_none_or(|n| match (self.face_of[0].first(), self.face_of[1].first()) {
                (Some(&a0), Some(&b0)) => n.faces(fa - a0, fb - b0),
                _ => true,
            })
    }

    /// Whether edge bases `ba` and `bb`, of the first solid and the second, may meet.
    pub(super) fn edges_near(&self, ba: usize, bb: usize) -> bool {
        self.near
            .as_ref()
            .is_none_or(|n| match (self.edge_base[0].first(), self.edge_base[1].first()) {
                (Some(&a0), Some(&b0)) => n.edges(ba - a0, bb - b0),
                _ => true,
            })
    }

    /// Whether edge base `bi` of solid `side` may meet face `fi` of the other.
    pub(super) fn edge_face_near(&self, side: usize, bi: usize, fi: usize) -> bool {
        let firsts = (
            self.edge_base.get(side).and_then(|ids| ids.first()),
            self.face_of.get(1 - side).and_then(|ids| ids.first()),
        );
        self.near.as_ref().is_none_or(|n| match firsts {
            (Some(&b0), Some(&f0)) => n.edge_face(side, bi - b0, fi - f0),
            _ => true,
        })
    }
}
