// yack: step 3: each face split by the cuts inside it.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only.
//! Step 3: each face split by the cuts inside it.

use super::*;
use super::bounds::padded;

#[allow(clippy::disallowed_types)] impl<'a> Work<'a> { // D26: BTree* in hardening, proven by digests
    /// A trim along canonical piece `ci` from vertex `start`; `along` breaks the tie for a closed
    /// piece, by the direction the trim must run at the piece's middle.
    pub(super) fn canon_trim(&self, ci: usize, start: usize, along: Option<Vec3>) -> Trim {
        let p = &self.pieces[ci];
        let b = &self.bases[p.base];
        let forward = if p.v0 != p.v1 {
            p.v0 == start
        } else {
            match (along, &b.curve) {
                (Some(d), Some(c)) => c.d1(0.5 * (p.range.0 + p.range.1)).1.dot(d) >= 0.0,
                _ => true,
            }
        };
        let (t0, t1, v0, v1) = if forward {
            (p.range.0, p.range.1, p.v0, p.v1)
        } else {
            (p.range.1, p.range.0, p.v1, p.v0)
        };
        Trim {
            piece: ci,
            forward,
            curve: b.curve.clone(),
            pole: self.pool[p.v0],
            t0,
            t1,
            map: UvMap::Projected,
            uv0: Vec2::ZERO,
            uv1: Vec2::ZERO,
            v0,
            v1,
        }
    }

    /// The face's boundary, its input coedges cut into pieces.
    pub(super) fn boundary_trims(&self, fi: usize) -> Vec<Vec<Trim>> {
        let face = &self.faces[fi];
        let b = self.breps[face.side];
        let img = b.memo(face.id).slot::<FaceImages>();
        let mut loops = Vec::new();
        for (li, &lp) in b.face(face.id).loops.iter().enumerate() {
            let mut trims = Vec::new();
            for (k, &c) in b.lp(lp).coedges.iter().enumerate() {
                let co = b.coedge(c);
                let bi = self.edge_base[face.side][co.edge.index()];
                let mut ps = self.base_pieces[bi].clone();
                if co.reversed {
                    ps.reverse();
                }
                let mut ts: Vec<f64> = Vec::new();
                for &pi in &ps {
                    let r = self.pieces[pi].range;
                    let (a, z) = if co.reversed { (r.1, r.0) } else { (r.0, r.1) };
                    if ts.is_empty() {
                        ts.push(a);
                    }
                    ts.push(z);
                }
                let uvs = match &img {
                    Some(img) if ps.len() == 1 => {
                        let w = img.trims[li][k].whole;
                        vec![w.0, w.1]
                    }
                    _ => coedge_uv_at(b, c, &ts),
                };
                let whole = &face.dom.loops[li][k];
                for (k, &pi) in ps.iter().enumerate() {
                    let piece = &self.pieces[pi];
                    let ci = self.canon[pi];
                    let start = if co.reversed { piece.v1 } else { piece.v0 };
                    let mut tr = if self.bases[piece.base].curve.is_none() {
                        Trim {
                            piece: pi,
                            ..whole.clone()
                        }
                    } else {
                        let along = self.bases[piece.base].curve.as_ref().map(|c| {
                            let d = c.d1(0.5 * (piece.range.0 + piece.range.1)).1;
                            if co.reversed { -d } else { d }
                        });
                        let mut tr = self.canon_trim(ci, start, along);
                        if ci == pi
                            && let UvMap::Affine { .. } = whole.map
                        {
                            tr.map = whole.map.clone();
                        }
                        tr
                    };
                    tr.uv0 = uvs[k];
                    tr.uv1 = uvs[k + 1];
                    trims.push(tr);
                }
            }
            loops.push(trims);
        }
        loops
    }

    /// A cut across face `fi` along canonical piece `ci`, its parameter-plane image continued
    /// from its middle, placed in the face's window.
    pub(super) fn cut_trim(&self, fi: usize, ci: usize) -> Trim {
        let dom = &self.faces[fi].dom;
        let s = &dom.surface;
        let p = &self.pieces[ci];
        let mut tr = self.canon_trim(ci, p.v0, None);
        let tm = 0.5 * (tr.t0 + tr.t1);
        // A face that runs on past its seam's line (the seam cut away there) spans more than a
        // period, so only the image its loops enclose meets the boundary's nodes.
        let near = s.project(tr.point(tm));
        let mid = dom
            .inside_image(near)
            .unwrap_or_else(|| dom.shift_into(near));
        let march = |to: f64| {
            let mut uv = mid;
            let n = 16;
            for k in 1..=n {
                let t = tm + (to - tm) * k as f64 / n as f64;
                uv = tr.uv(s, t, uv);
            }
            uv
        };
        let (uv0, uv1) = (march(tr.t0), march(tr.t1));
        tr.uv0 = uv0;
        tr.uv1 = uv1;
        tr
    }

    /// Step 3: every face split into pieces.
    /// Also returns the pieces of poles that cuts end on, numbered after the other pieces, and
    /// whether each face came through whole: no cut crosses it and none of its edges is split or
    /// stands in for another, so its one piece has exactly the input face's geometry.
    #[allow(clippy::type_complexity)]
    pub(super) fn split_faces(&self) -> Result<(Vec<Vec<Vec<Vec<Trim>>>>, Vec<PieceGeom>, Vec<bool>)> {
        let mut cuts: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); self.faces.len()];
        for (pi, fs) in self.cut_faces.iter().enumerate() {
            for &f in fs {
                cuts[f].insert(self.canon[pi]);
            }
        }
        for &(fa, fb) in &self.coincident {
            for (into, from) in [(fa, fb), (fb, fa)] {
                let from_face = &self.faces[from];
                let b = self.breps[from_face.side];
                let edges: BTreeSet<EdgeId> = b
                    .face(from_face.id)
                    .loops
                    .iter()
                    .flat_map(|&lp| b.lp(lp).coedges.iter().map(|&c| b.coedge(c).edge))
                    .collect();
                for e in edges {
                    let bi = self.edge_base[from_face.side][e.index()];
                    for &pi in &self.base_pieces[bi] {
                        if self.bases[bi].curve.is_none() {
                            continue;
                        }
                        let mid = self.piece_point(pi, 0.5);
                        if let Where::Inside(_) = self.faces[into].dom.locate(mid) {
                            cuts[into].insert(self.canon[pi]);
                        }
                    }
                }
            }
        }
        let mut members: HashMap<usize, Vec<usize>> = HashMap::new();
        for (pi, &c) in self.canon.iter().enumerate() {
            members.entry(c).or_default().push(pi);
        }
        let mut out = Vec::with_capacity(self.faces.len());
        let mut poles: Vec<PieceGeom> = Vec::new();
        let mut intact = Vec::with_capacity(self.faces.len());
        for (fi, face_cuts) in cuts.iter().enumerate() {
            let mut boundary = self.boundary_trims(fi);
            let on_boundary: BTreeSet<usize> = boundary.iter().flatten().map(|t| t.piece).collect();
            let trims: Vec<Trim> = face_cuts
                .iter()
                .filter(|ci| !on_boundary.contains(ci))
                .map(|&ci| self.cut_trim(fi, ci))
                .collect();
            let next_id = self.pieces.len() + poles.len();
            let made = split_poles(&mut boundary, &trims, next_id);
            // Each of its edges whole and its own: where an edge lies along one of the other
            // input, the piece both take is one of the two, and on the other's the face runs
            // along another curve over another range, which its memo knows nothing of. A trim
            // names the piece taken, so the edge's own is looked up.
            let own = {
                let face = &self.faces[fi];
                let b = self.breps[face.side];
                b.face(face.id).loops.iter().all(|&lp| {
                    b.lp(lp).coedges.iter().all(|&c| {
                        let bi = self.edge_base[face.side][b.coedge(c).edge.index()];
                        let ps = &self.base_pieces[bi];
                        ps.len() == 1 && self.canon[ps[0]] == ps[0]
                    })
                })
            };
            let whole = trims.is_empty() && made.is_empty() && own;
            let sequence = |loops: &[Vec<Trim>]| -> Vec<Vec<usize>> {
                loops
                    .iter()
                    .map(|l| l.iter().map(|t| t.piece).collect())
                    .collect()
            };
            let before = whole.then(|| sequence(&boundary));
            poles.extend(made);
            let face = &self.faces[fi];
            let vertex_tol = |v: usize| self.vtol.get(v).copied().unwrap_or(tol::LINEAR);
            // A contact may end inside the face: a tangent curve, or an edge of the other solid
            // lying on it.
            let slit = |ci: usize| {
                members.get(&ci).is_some_and(|ps| {
                    ps.iter().any(|&pi| {
                        let bi = self.pieces[pi].base;
                        matches!(self.bases[bi].kind, BaseKind::Cut { tangent: true, .. })
                            || self.on_face.contains(&(bi, fi))
                    })
                })
            };
            let regions = arrange::split(
                &face.dom.surface,
                face.dom.reversed,
                boundary,
                trims,
                &vertex_tol,
                &slit,
            )
            .map_err(|e| self.at_faces(e, &[fi]))?;
            intact.push(match before {
                Some(b) => regions.len() == 1 && sequence(&regions[0]) == b,
                None => false,
            });
            out.push(regions);
        }
        Ok((out, poles, intact))
    }
}

/// Splits each pole of a face's boundary where a cut arrives at it, so the cut meets the pole's
/// parameter-plane image at a node. The new pieces are numbered from `next_id`.
pub(super) fn split_poles(boundary: &mut [Vec<Trim>], cuts: &[Trim], next_id: usize) -> Vec<PieceGeom> {
    let mut made = Vec::new();
    for lp in boundary.iter_mut() {
        let mut k = 0;
        while k < lp.len() {
            let tr = &lp[k];
            let UvMap::Affine { origin, dir } = tr.map else {
                k += 1;
                continue;
            };
            if tr.curve.is_some() || dir.x == 0.0 {
                k += 1;
                continue;
            }
            let v = tr.v0;
            let (lo, hi) = (tr.t0.min(tr.t1), tr.t0.max(tr.t1));
            let margin = 1e-9 * (hi - lo);
            let mut at: Vec<f64> = cuts
                .iter()
                .flat_map(|c| [(c.v0, c.uv0), (c.v1, c.uv1)])
                .filter(|&(w, _)| w == v)
                .map(|(_, uv)| (uv.x - origin.x) / dir.x)
                .filter(|&t| t > lo + margin && t < hi - margin)
                .collect();
            if at.is_empty() {
                k += 1;
                continue;
            }
            at.sort_by(f64::total_cmp);
            at.dedup_by(|a, b| (*a - *b).abs() <= margin);
            if tr.t1 < tr.t0 {
                at.reverse();
            }
            let mut ts = vec![tr.t0];
            ts.extend(at);
            ts.push(tr.t1);
            let parts: Vec<Trim> = ts
                .windows(2)
                .map(|w| {
                    let id = next_id + made.len();
                    made.push(PieceGeom {
                        curve: None,
                        range: (w[0].min(w[1]), w[0].max(w[1])),
                        v0: v,
                        v1: v,
                        tol: tol::LINEAR,
                    });
                    Trim {
                        piece: id,
                        forward: w[1] > w[0],
                        t0: w[0],
                        t1: w[1],
                        uv0: origin + dir * w[0],
                        uv1: origin + dir * w[1],
                        ..tr.clone()
                    }
                })
                .collect();
            let n = parts.len();
            lp.splice(k..k + 1, parts);
            k += n;
        }
    }
    made
}

/// The known vertices in `region` where `a` and `b` touch and share nothing else nearby.
pub(super) fn known_touches(pool: &[Point3], a: &Surface, b: &Surface, region: &Aabb) -> Vec<contact::Touch> {
    pool.iter()
        .filter(|&&p| {
            (0..3).all(|k| p[k] >= region.min[k] && p[k] <= region.max[k])
                && csi::signed_distance(a, p).abs() <= tol::LINEAR
                && csi::signed_distance(b, p).abs() <= tol::LINEAR
        })
        .filter_map(|&p| contact::touch_at(a, b, p))
        .collect()
}

/// Points on both surfaces where tracing their intersection may start: the vertices already found
/// on both, and points reached from a grid over each face.
pub(super) fn seeds_for<'p>(
    known: impl Iterator<Item = &'p Point3>,
    a: &Domain,
    b: &Domain,
    region: &Aabb,
) -> Vec<Point3> {
    let mut out: Vec<Point3> = known
        .copied()
        .filter(|&p| {
            (0..3).all(|k| p[k] >= region.min[k] && p[k] <= region.max[k])
                && csi::signed_distance(&a.surface, p).abs() <= tol::LINEAR
                && csi::signed_distance(&b.surface, p).abs() <= tol::LINEAR
        })
        .collect();
    let reach = 0.25 * region.diagonal();
    for (from, to) in [(a, b), (b, a)] {
        // A spline surface lies in its control points' box: a point farther than `reach` from
        // that box has no projection near enough to seed, so it is not projected at all.
        let hull = match &to.surface {
            Surface::Nurbs(n) => Some(padded(&n.hull_box(), reach)),
            _ => None,
        };
        let n = 8;
        for i in 0..=n {
            for j in 0..=n {
                let u = from.lo.x + (from.hi.x - from.lo.x) * (i as f64 + 0.37) / (n as f64 + 1.0);
                let v = from.lo.y + (from.hi.y - from.lo.y) * (j as f64 + 0.61) / (n as f64 + 1.0);
                let p = from.surface.eval(u, v);
                if hull
                    .as_ref()
                    .is_some_and(|h| !(0..3).all(|k| p[k] >= h.min[k] && p[k] <= h.max[k]))
                {
                    continue;
                }
                let uv = to.surface.project(p);
                let q = to.surface.eval(uv.x, uv.y);
                if p.dist(q) <= reach {
                    out.push(p.lerp(q, 0.5));
                }
            }
        }
    }
    out
}
