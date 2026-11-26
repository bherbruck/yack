// yack: step 1c-2: face pairs' intersection curves, and every edge and curve split at its vertices.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only.
//! Step 1c-2: face pairs' intersection curves, and every edge and curve split at its vertices.

use super::*;
use super::bounds::{overlap, padded, curve_bbox, locate_piece};
use super::split::{known_touches, seeds_for};

#[allow(clippy::disallowed_types)] impl<'a> Work<'a> { // D26: BTree* in hardening, proven by digests
    /// Step 1c: every face pair's intersection curves, and coincident pairs.
    pub(super) fn intersect_faces(&mut self) -> Result<()> {
        let pairs: Vec<(usize, usize)> = self.face_of[0]
            .iter()
            .flat_map(|&fa| self.face_of[1].iter().map(move |&fb| (fa, fb)))
            .filter(|&(fa, fb)| {
                overlap(&self.faces[fa].bbox, &self.faces[fb].bbox) && self.faces_near(fa, fb)
            })
            .collect();
        let owners = self.edge_faces();
        for (fa, fb) in pairs {
            let (sa, sb) = (&self.faces[fa].dom.surface, &self.faces[fb].dom.surface);
            if same_domain_faces(&self.faces[fa].dom, &self.faces[fb].dom) {
                self.coincident.push((fa, fb));
                continue;
            }
            let (ba, bb) = (&self.faces[fa].bbox, &self.faces[fb].bbox);
            let mut region = Aabb {
                min: Vec3::new(
                    ba.min.x.max(bb.min.x),
                    ba.min.y.max(bb.min.y),
                    ba.min.z.max(bb.min.z),
                ),
                max: Vec3::new(
                    ba.max.x.min(bb.max.x),
                    ba.max.y.min(bb.max.y),
                    ba.max.z.min(bb.max.z),
                ),
            };
            region = padded(&region, 1e-3 * (ba.diagonal() + bb.diagonal()) + 1e-4);
            // Edges of one face lying on the other's surface: the faces' contact along each is that
            // edge.
            let on_edges: Vec<usize> = self
                .on_face
                .iter()
                .filter(|&&(bi, f)| {
                    let own = owners.get(&bi).map_or(&[][..], Vec::as_slice);
                    (f == fb && own.contains(&fa)) || (f == fa && own.contains(&fb))
                })
                .map(|&(bi, _)| bi)
                .collect();
            // On the edge, or where the faces are tangent along it, within the band either side
            // of it where they stay within the tolerance of each other: a trace there is noise.
            let along = |bi: usize, p: Point3| {
                let base = &self.bases[bi];
                base.curve.as_ref().is_some_and(|c| {
                    let (d, t) = csi::curve_distance(c, base.range, p);
                    d <= base.tol || d <= contact::band(sa, sb, c.eval(t), c.d1(t).1)
                })
            };
            // A tolerant edge lying on the other face is where the two faces meet; within its tube
            // they lie within its tolerance of each other, so a trace there finds only noise.
            // So is an edge lying on the other face where the two are tangent along it, within the
            // band either side where they stay within the tolerance of each other.
            let tangent_band = |b: &Base| {
                let c = b.curve.as_ref()?;
                let band = (1..8)
                    .map(|k| {
                        let t = b.range.0 + (b.range.1 - b.range.0) * k as f64 / 8.0;
                        contact::band(sa, sb, c.eval(t), c.d1(t).1)
                    })
                    .fold(f64::INFINITY, f64::min);
                (band > 0.0).then_some(band)
            };
            let tubes: Vec<(Rc<Curve>, (f64, f64), f64)> = on_edges
                .iter()
                .map(|&bi| &self.bases[bi])
                .filter_map(|b| {
                    let width = if b.tol > tol::LINEAR {
                        2.0 * b.tol
                    } else {
                        tangent_band(b)?
                    };
                    Some((b.curve.clone()?, b.range, width))
                })
                .collect();
            let mut touches: Vec<contact::Touch> = Vec::new();
            let result = {
                // The inputs' vertices, then the edges' meetings with faces, then the rest: a
                // trace starts from the first seed on its branch.
                let (inputs, found) = self.pool.split_at(self.inputs);
                let known = inputs
                    .iter()
                    .chain(self.hits.iter().map(|h| &h.p))
                    .chain(found);
                let (da, db) = (&self.faces[fa].dom, &self.faces[fb].dom);
                let pool = &self.pool;
                let touches = &mut touches;
                let mut seeds = || {
                    *touches = known_touches(pool, sa, sb, &region);
                    let mut out = seeds_for(known.clone(), da, db, &region);
                    out.retain(|&p| !touches.iter().any(|t| p.dist(t.p) <= t.reach));
                    out.retain(|&p| {
                        !tubes
                            .iter()
                            .any(|(c, r, w)| csi::curve_distance(c, *r, p).0 <= *w)
                    });
                    out
                };
                match ssi::intersect(sa, sb, &region, &mut seeds) {
                    Ok(r) => r,
                    // Faces that meet along a tolerant edge lying on both are that contact, as
                    // Jackson's face-face step leaves to edge-face coincidence; the checker still
                    // reads the result for faces that pass through each other elsewhere.
                    Err(_) if !tubes.is_empty() => Ssi::Curves {
                        curves: Vec::new(),
                        points: Vec::new(),
                    },
                    Err(e) => return Err(self.at_faces(e, &[fa, fb])),
                }
            };
            let Ssi::Curves { mut curves, points } = result else {
                self.coincident.push((fa, fb));
                continue;
            };
            // Where the surfaces touch at a known vertex and nowhere else near it, a trace that
            // stays within the touch's reach found only that vertex.
            curves.retain(|c| {
                !touches.iter().any(|t| {
                    (0..=8).all(|k| {
                        let x = c.range.0 + (c.range.1 - c.range.0) * k as f64 / 8.0;
                        c.curve.eval(x).dist(t.p) <= t.reach
                    })
                })
            });
            curves.retain(|c| {
                !on_edges.iter().any(|&bi| {
                    (0..=8).all(|k| {
                        along(
                            bi,
                            c.curve
                                .eval(c.range.0 + (c.range.1 - c.range.0) * k as f64 / 8.0),
                        )
                    })
                })
            });
            for p in points {
                if self.faces[fa].dom.locate(p) != Where::Outside
                    && self.faces[fb].dom.locate(p) != Where::Outside
                {
                    let v = self.vertex(p);
                    self.touch_points.push(v);
                }
            }
            for c in curves {
                let bbox = curve_bbox(&c.curve, c.range);
                self.bases.push(Base {
                    pole: c.curve.eval(c.range.0),
                    curve: Some(Rc::new(c.curve)),
                    range: c.range,
                    v0: None,
                    v1: None,
                    closed: c.closed,
                    bbox,
                    kind: BaseKind::Cut {
                        fa,
                        fb,
                        tangent: c.tangent,
                    },
                    tol: tol::LINEAR,
                });
            }
        }
        Ok(())
    }

    /// The vertices that lie inside a base's range, as `(parameter, vertex, distance)` in order.
    /// The pool vertices within reach of base `bi`'s box: those `points` (a hierarchy over the
    /// first `indexed` of the pool) finds there, and every vertex added since.
    pub(super) fn splits(&self, bi: usize, points: &Bvh, indexed: usize) -> Vec<(f64, usize, f64)> {
        let base = &self.bases[bi];
        let Some(curve) = &base.curve else {
            return Vec::new();
        };
        let cut = match base.kind {
            BaseKind::Cut { fa, fb, .. } => Some((fa.min(fb), fa.max(fb))),
            _ => None,
        };
        // A meeting of the cut's own two faces is on the cut, however loosely a graze placed it.
        let grazing = |vi: usize| -> f64 {
            let met = self.meetings.get(&vi).into_iter().flatten();
            met.filter(|m| Some((m.0, m.1)) == cut)
                .map(|m| m.2)
                .fold(0.0, f64::max)
        };
        let reach = if cut.is_some() { GRAZE_REACH } else { 0.0 };
        let bb = padded(&base.bbox, base.tol + self.widest + reach);
        let mut near = Vec::new();
        points.query(&bb, &mut near);
        near.extend(indexed..self.pool.len());
        near.sort_unstable();
        let mut out: Vec<(f64, usize, f64)> = Vec::new();
        for vi in near {
            let p = self.pool[vi];
            if Some(vi) == base.v0 || Some(vi) == base.v1 {
                continue;
            }
            if !(0..3).all(|k| p[k] >= bb.min[k] && p[k] <= bb.max[k]) {
                continue;
            }
            let (d, t) = csi::curve_distance(curve, base.range, p);
            if d <= tol::coincidence(base.tol, self.vtol[vi]).max(grazing(vi)) {
                out.push((t, vi, d));
            }
        }
        out.sort_by(|a, b| a.0.total_cmp(&b.0));
        out.dedup_by_key(|x| x.1);
        out
    }

    /// Step 2a: every base split at its vertices into pieces; intersection pieces outside either
    /// face are dropped.
    #[allow(clippy::panic)] pub(super) fn split_curves(&mut self) {
        self.base_pieces = vec![Vec::new(); self.bases.len()];
        let mut plans: Vec<(usize, Vec<f64>, Vec<usize>)> = Vec::new();
        let indexed = self.pool.len();
        let points = Bvh::of_boxes(self.pool.iter().map(|&p| Aabb { min: p, max: p }).collect());
        for bi in 0..self.bases.len() {
            let sp: Vec<(f64, usize)> = self
                .splits(bi, &points, indexed)
                .into_iter()
                .map(|(t, v, d)| {
                    // The piece ends where the curve passes the vertex, which must hold it.
                    if d > self.vtol[v] {
                        // A grazing meeting is known only to its reach: the vertex says so.
                        let met = self.meetings.get(&v).into_iter().flatten();
                        let reach = met.map(|m| m.2).fold(0.0, f64::max);
                        self.vtol[v] = d.min(self.cap.max(reach));
                        self.widest = self.widest.max(self.vtol[v]);
                        if !self.tolerant.contains(&v) {
                            self.tolerant.push(v);
                        }
                    }
                    (t, v)
                })
                .collect();
            let base = &self.bases[bi];
            let mut ts: Vec<f64> = Vec::new();
            let mut vs: Vec<usize> = Vec::new();
            match (base.v0, base.v1) {
                (Some(v0), Some(v1)) => {
                    ts.push(base.range.0);
                    vs.push(v0);
                    for &(t, v) in &sp {
                        ts.push(t);
                        vs.push(v);
                    }
                    ts.push(base.range.1);
                    vs.push(v1);
                }
                _ if base.closed => {
                    let period = base.range.1 - base.range.0;
                    if sp.is_empty() {
                        let t = base.range.0;
                        let mid = base
                            .curve
                            .as_ref()
                            .unwrap_or_else(|| panic!("cut has a curve"))
                            .eval(t + 0.5 * period);
                        if !self.cut_inside_both(bi, mid) {
                            continue;
                        }
                        // A closed curve that meets no vertex gets one of its own.
                        let p = base.pole;
                        let v = self.vertex(p);
                        ts = vec![t, t + period];
                        vs = vec![v, v];
                    } else {
                        for &(t, v) in &sp {
                            ts.push(t);
                            vs.push(v);
                        }
                        ts.push(sp[0].0 + period);
                        vs.push(sp[0].1);
                    }
                }
                _ => {
                    for &(t, v) in &sp {
                        ts.push(t);
                        vs.push(v);
                    }
                }
            }
            plans.push((bi, ts, vs));
        }
        // Vertices that split one curve less than the tolerance apart along it are one place on
        // it: faces lying within the tolerance of each other can put two corners there whose
        // points are just over it apart, and the piece between them would have no direction.
        let mut parent: Vec<usize> = (0..self.pool.len()).collect();
        fn root(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        let shorts: Vec<Vec<bool>> = plans
            .iter()
            .map(|(bi, ts, _)| {
                (0..ts.len().saturating_sub(1))
                    .map(|k| {
                        self.bases[*bi].curve.as_ref().is_some_and(|c| {
                            let (t0, t1) = (ts[k], ts[k + 1]);
                            let a = c.eval(t0);
                            a.dist(c.eval(t1)) <= tol::LINEAR
                                && a.dist(c.eval(0.5 * (t0 + t1))) <= tol::LINEAR
                        })
                    })
                    .collect()
            })
            .collect();
        for ((_, ts, vs), short) in plans.iter().zip(&shorts) {
            for k in 0..ts.len().saturating_sub(1) {
                if vs[k] != vs[k + 1] && short[k] {
                    let (a, b) = (root(&mut parent, vs[k]), root(&mut parent, vs[k + 1]));
                    parent[a.max(b)] = a.min(b);
                }
            }
        }
        // Each merged vertex at its members' centroid where that holds them all: two corners just
        // over the tolerance apart are each within it of their middle.
        let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
        for v in 0..parent.len() {
            let r = root(&mut parent, v);
            if r != v {
                groups.entry(r).or_insert_with(|| vec![r]).push(v);
            }
        }
        for (r, members) in &groups {
            let sum = members
                .iter()
                .fold(Vec3::ZERO, |acc, &m| acc + self.pool[m]);
            let c = sum * (1.0 / members.len() as f64);
            if members.iter().all(|&m| self.pool[m].dist(c) <= tol::LINEAR) {
                self.pool[*r] = c;
            }
        }
        for v in 0..parent.len() {
            let r = root(&mut parent, v);
            if r != v {
                let gap = self.pool[r].dist(self.pool[v]) + self.vtol[v];
                if gap > self.vtol[r] {
                    self.vtol[r] = gap.min(self.cap);
                    self.widest = self.widest.max(self.vtol[r]);
                    if self.vtol[r] > tol::LINEAR && !self.tolerant.contains(&r) {
                        self.tolerant.push(r);
                    }
                }
            }
        }
        for ((bi, ts, vs), short) in plans.into_iter().zip(shorts) {
            let vs: Vec<usize> = vs.into_iter().map(|v| root(&mut parent, v)).collect();
            for k in 0..ts.len().saturating_sub(1) {
                if vs[k] == vs[k + 1] && short[k] {
                    continue;
                }
                self.add_piece(bi, (ts[k], ts[k + 1]), vs[k], vs[k + 1]);
            }
        }
    }

    pub(super) fn cut_inside_both(&self, bi: usize, p: Point3) -> bool {
        let BaseKind::Cut { fa, fb, .. } = self.bases[bi].kind else {
            return true;
        };
        self.faces[fa].dom.locate(p) != Where::Outside
            && self.faces[fb].dom.locate(p) != Where::Outside
    }

    #[allow(clippy::panic)] pub(super) fn add_piece(&mut self, bi: usize, range: (f64, f64), v0: usize, v1: usize) {
        let base = &self.bases[bi];
        let mut inside = Vec::new();
        let rank = match base.kind {
            BaseKind::Edge => self.edge_side(bi) as u8,
            // A tangent curve cuts both faces as any other: where the result keeps both, the
            // pieces touch along it, an edge four faces use; where it keeps one, the two halves of
            // that face merge again across it.
            BaseKind::Cut { fa, fb, .. } => {
                let c = base.curve.as_ref().unwrap_or_else(|| panic!("cut has a curve"));
                let ends = [0.0, 0.5, 1.0].map(|f| c.eval(range.0 + (range.1 - range.0) * f));
                let (la, lb) = (
                    locate_piece(&self.faces[fa].dom, ends),
                    locate_piece(&self.faces[fb].dom, ends),
                );
                if la == Where::Outside || lb == Where::Outside {
                    return;
                }
                for (f, l) in [(fa, la), (fb, lb)] {
                    if matches!(l, Where::Inside(_)) {
                        inside.push(f);
                    }
                }
                2
            }
        };
        let id = self.pieces.len();
        self.pieces.push(Piece {
            base: bi,
            range,
            v0,
            v1,
            rank,
        });
        self.cut_faces.push(inside);
        self.base_pieces[bi].push(id);
    }

    /// Which input an edge base came from: the first input's edges come first.
    pub(super) fn edge_side(&self, bi: usize) -> usize {
        usize::from(self.edge_base[1].first().is_some_and(|&b| bi >= b))
    }

    pub(super) fn piece_point(&self, pi: usize, f: f64) -> Point3 {
        let p = &self.pieces[pi];
        let b = &self.bases[p.base];
        match &b.curve {
            Some(c) => c.eval(p.range.0 + (p.range.1 - p.range.0) * f),
            None => b.pole,
        }
    }

    /// Step 2b: pieces that run along each other become one, the lowest-ranked.
    #[allow(clippy::panic)] pub(super) fn dedup(&mut self) {
        let n = self.pieces.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(p: &mut [usize], i: usize) -> usize {
            let mut r = i;
            while p[r] != r {
                r = p[r];
            }
            let mut k = i;
            while p[k] != r {
                let nx = p[k];
                p[k] = r;
                k = nx;
            }
            r
        }
        let mut by_ends: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        for (i, p) in self.pieces.iter().enumerate() {
            if self.bases[p.base].curve.is_none() {
                continue;
            }
            by_ends
                .entry((p.v0.min(p.v1), p.v0.max(p.v1)))
                .or_default()
                .push(i);
        }
        for group in by_ends.values() {
            for (k, &i) in group.iter().enumerate() {
                for &j in &group[k + 1..] {
                    if self.pieces[i].base == self.pieces[j].base {
                        continue;
                    }
                    let pj = &self.pieces[j];
                    let cj = self.bases[pj.base].curve.as_ref().unwrap_or_else(|| panic!("has a curve"));
                    let within =
                        tol::coincidence(self.bases[pj.base].tol, self.bases[self.pieces[i].base].tol);
                    let same = [0.25, 0.5, 0.75].iter().all(|&f| {
                        csi::curve_distance(cj, pj.range, self.piece_point(i, f)).0 <= within
                    });
                    if same {
                        let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                        if ri != rj {
                            let key = |x: usize| (self.pieces[x].rank, x);
                            if key(ri) < key(rj) {
                                parent[rj] = ri;
                            } else {
                                parent[ri] = rj;
                            }
                        }
                    }
                }
            }
        }
        self.canon = (0..n).map(|i| find(&mut parent, i)).collect();
    }

    pub(super) fn piece_geoms(&self) -> Vec<PieceGeom> {
        self.pieces
            .iter()
            .map(|p| {
                let b = &self.bases[p.base];
                PieceGeom {
                    curve: b.curve.clone(),
                    range: p.range,
                    v0: p.v0,
                    v1: p.v1,
                    tol: b.tol,
                }
            })
            .collect()
    }
}
