// yack: step 1c-2: face pairs' intersection curves, and every edge and curve split at its vertices.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only; hardened (M3 B11): checked access in place of indexing, the
// two HashMaps BTreeMaps (each walked only into a min-root union-find or disjoint updates), its
// three contract panics a skip, its small literals named.
//! Step 1c-2: face pairs' intersection curves, and every edge and curve split at its vertices.

use std::collections::BTreeMap;

use super::*;
use super::bounds::{overlap, padded, curve_bbox, locate_piece};
use super::split::{known_touches, seeds_for};

/// The region two faces' intersection is traced in: their boxes' overlap, grown by this share of
/// their diagonals and [`REGION_PAD`] mm.
const REGION_SLACK: f64 = 1e-3; // tol: a share of the boxes' size, not a feature size

/// See [`REGION_SLACK`].
const REGION_PAD: f64 = 1e-4; // tol: slack on a search box, not a tolerance

/// Where stretch `t0..t1` of `curve`, whose two ends were made one vertex `at` by another curve,
/// is held to that vertex: the end nearer it, if the stretch is no longer than `reach` (the
/// vertices' distance and tolerance) at its ends and its middle. A longer stretch is a piece of its
/// own.
fn merged_stretch(curve: &Curve, (t0, t1): (f64, f64), at: Point3, reach: f64) -> Option<f64> {
    let (from, to) = (curve.eval(t0), curve.eval(t1));
    (from.dist(to) <= reach && from.dist(curve.eval(0.5 * (t0 + t1))) <= reach)
        .then(|| if from.dist(at) <= to.dist(at) { t0 } else { t1 })
}

/// The root of `i` in the union-find `parent`, every step on the way pointed straight at it.
fn find(parent: &mut [usize], i: usize) -> usize {
    let mut root = i;
    while let Some(&up) = parent.get(root)
        && up != root
    {
        root = up;
    }
    let mut k = i;
    while let Some(slot) = parent.get_mut(k)
        && *slot != root
    {
        k = std::mem::replace(slot, root);
    }
    root
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
    /// Step 1c: every face pair's intersection curves, and coincident pairs.
    pub(super) fn intersect_faces(&mut self) -> Result<()> {
        let pairs: Vec<(usize, usize)> = self.face_of[0]
            .iter()
            .flat_map(|&fa| self.face_of[1].iter().map(move |&fb| (fa, fb)))
            .filter(|&(fa, fb)| {
                self.faces.get(fa).zip(self.faces.get(fb)).is_some_and(|(a, b)| overlap(&a.bbox, &b.bbox))
                    && self.faces_near(fa, fb)
            })
            .collect();
        let owners = self.edge_faces();
        for (fa, fb) in pairs {
            let (Some(face_a), Some(face_b)) = (self.faces.get(fa), self.faces.get(fb)) else {
                continue;
            };
            let (sa, sb) = (&face_a.dom.surface, &face_b.dom.surface);
            if same_domain_faces(&face_a.dom, &face_b.dom) {
                self.coincident.push((fa, fb));
                continue;
            }
            let (ba, bb) = (&face_a.bbox, &face_b.bbox);
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
            region = padded(&region, REGION_SLACK * (ba.diagonal() + bb.diagonal()) + REGION_PAD);
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
                self.bases.get(bi).is_some_and(|base| {
                    base.curve.as_ref().is_some_and(|c| {
                        let (d, t) = csi::curve_distance(c, base.range, p);
                        d <= base.tol || d <= contact::band(sa, sb, c.eval(t), c.d1(t).1)
                    })
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
                .filter_map(|&bi| self.bases.get(bi))
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
                let (da, db) = (&face_a.dom, &face_b.dom);
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
                let inside = self.faces.get(fa).zip(self.faces.get(fb)).is_some_and(|(a, b)| {
                    a.dom.locate(p) != Where::Outside && b.dom.locate(p) != Where::Outside
                });
                if inside {
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
        let Some(base) = self.bases.get(bi) else {
            return Vec::new();
        };
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
            let (Some(&p), Some(&vt)) = (self.pool.get(vi), self.vtol.get(vi)) else {
                continue;
            };
            if Some(vi) == base.v0 || Some(vi) == base.v1 {
                continue;
            }
            if !(0..3).all(|k| p[k] >= bb.min[k] && p[k] <= bb.max[k]) {
                continue;
            }
            let (d, t) = csi::curve_distance(curve, base.range, p);
            if d <= tol::coincidence(base.tol, vt).max(grazing(vi)) {
                out.push((t, vi, d));
            }
        }
        out.sort_by(|a, b| a.0.total_cmp(&b.0));
        out.dedup_by_key(|x| x.1);
        out
    }

    /// Step 2a: every base split at its vertices into pieces; intersection pieces outside either
    /// face are dropped.
    pub(super) fn split_curves(&mut self) {
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
                    if self.vtol.get(v).is_some_and(|&vt| d > vt) {
                        // A grazing meeting is known only to its reach: the vertex says so.
                        let met = self.meetings.get(&v).into_iter().flatten();
                        let reach = met.map(|m| m.2).fold(0.0, f64::max);
                        let held = d.min(self.cap.max(reach));
                        if let Some(vt) = self.vtol.get_mut(v) {
                            *vt = held;
                        }
                        self.widest = self.widest.max(held);
                        if !self.tolerant.contains(&v) {
                            self.tolerant.push(v);
                        }
                    }
                    (t, v)
                })
                .collect();
            let Some(base) = self.bases.get(bi) else {
                continue;
            };
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
                        // a closed base is a cut, and a cut has a curve
                        let Some(curve) = base.curve.as_ref() else {
                            continue;
                        };
                        let mid = curve.eval(t + 0.5 * period);
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
                        if let Some(&(t0, v0)) = sp.first() {
                            ts.push(t0 + period);
                            vs.push(v0);
                        }
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
        // each piece of each plan: whether it is shorter than the tolerance
        let shorts: Vec<Vec<bool>> = plans
            .iter()
            .map(|(bi, ts, _)| {
                let curve = self.bases.get(*bi).and_then(|b| b.curve.as_ref());
                ts.windows(2)
                    .map(|w| {
                        let &[t0, t1] = w else {
                            return false;
                        };
                        curve.is_some_and(|c| {
                            let a = c.eval(t0);
                            a.dist(c.eval(t1)) <= tol::LINEAR
                                && a.dist(c.eval(0.5 * (t0 + t1))) <= tol::LINEAR
                        })
                    })
                    .collect()
            })
            .collect();
        for ((_, _, vs), short) in plans.iter().zip(&shorts) {
            for (w, &is_short) in vs.windows(2).zip(short) {
                if let &[a, b] = w
                    && a != b
                    && is_short
                {
                    let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
                    if let Some(up) = parent.get_mut(ra.max(rb)) {
                        *up = ra.min(rb);
                    }
                }
            }
        }
        // Each merged vertex at its members' centroid where that holds them all: two corners just
        // over the tolerance apart are each within it of their middle.
        // classes are disjoint, so each moves its own root alone: their order reaches nothing
        let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for v in 0..parent.len() {
            let r = root(&mut parent, v);
            if r != v {
                groups.entry(r).or_insert_with(|| vec![r]).push(v);
            }
        }
        for (r, members) in &groups {
            let points: Vec<Point3> = members.iter().filter_map(|&m| self.pool.get(m).copied()).collect();
            let sum = points.iter().fold(Vec3::ZERO, |acc, &q| acc + q);
            let c = sum * (1.0 / members.len() as f64);
            if points.iter().all(|q| q.dist(c) <= tol::LINEAR)
                && let Some(at) = self.pool.get_mut(*r)
            {
                *at = c;
            }
        }
        for v in 0..parent.len() {
            let r = root(&mut parent, v);
            if r != v {
                let (Some(&pr), Some(&pv), Some(&tv), Some(&tr)) =
                    (self.pool.get(r), self.pool.get(v), self.vtol.get(v), self.vtol.get(r))
                else {
                    continue;
                };
                let gap = pr.dist(pv) + tv;
                if gap > tr {
                    // A grazing meeting is known only to its reach, as where a curve passes one;
                    // and a vertex a curve already grew keeps what it has.
                    let reach = [r, v]
                        .into_iter()
                        .filter_map(|m| self.meetings.get(&m))
                        .flatten()
                        .map(|m| m.2)
                        .fold(0.0, f64::max);
                    let held = tr.max(gap.min(self.cap.max(reach)));
                    if let Some(t) = self.vtol.get_mut(r) {
                        *t = held;
                    }
                    self.widest = self.widest.max(held);
                    if held > tol::LINEAR && !self.tolerant.contains(&r) {
                        self.tolerant.push(r);
                    }
                }
            }
        }
        for ((bi, mut ts, met), short) in plans.into_iter().zip(shorts) {
            let vs: Vec<usize> = met.iter().map(|&v| root(&mut parent, v)).collect();
            // Two vertices that another curve made one may lie over the tolerance apart along this
            // one. The piece between them, no longer than they are apart, would join the vertex to
            // itself: the curve passes the vertex once, where it comes nearest.
            let curve = self.bases.get(bi).and_then(|b| b.curve.as_ref());
            for k in 0..ts.len().saturating_sub(1) {
                let (Some(&a), Some(&b), Some(&va), Some(&vb), Some(&is_short), Some(curve)) =
                    (met.get(k), met.get(k + 1), vs.get(k), vs.get(k + 1), short.get(k), curve)
                else {
                    continue;
                };
                if a == b || va != vb || is_short {
                    continue;
                }
                let (Some(&pa), Some(&pb), Some(&ta), Some(&tb), Some(&at), Some(&t0), Some(&t1)) =
                    (self.pool.get(a), self.pool.get(b), self.vtol.get(a), self.vtol.get(b), self.pool.get(va), ts.get(k), ts.get(k + 1))
                else {
                    continue;
                };
                let reach = pa.dist(pb) + tol::between(ta, tb);
                if let Some(t) = merged_stretch(curve, (t0, t1), at, reach) {
                    for end in [k, k + 1] {
                        if let Some(slot) = ts.get_mut(end) {
                            *slot = t;
                        }
                    }
                }
            }
            for ((tw, vw), &is_short) in ts.windows(2).zip(vs.windows(2)).zip(&short) {
                let (&[t0, t1], &[v0, v1]) = (tw, vw) else {
                    continue;
                };
                if v0 == v1 && (is_short || t0 == t1) {
                    continue;
                }
                self.add_piece(bi, (t0, t1), v0, v1);
            }
        }
    }

    pub(super) fn cut_inside_both(&self, bi: usize, p: Point3) -> bool {
        let Some(&BaseKind::Cut { fa, fb, .. }) = self.bases.get(bi).map(|b| &b.kind) else {
            return true;
        };
        let inside = |f: usize| self.faces.get(f).is_some_and(|face| face.dom.locate(p) != Where::Outside);
        inside(fa) && inside(fb)
    }

    pub(super) fn add_piece(&mut self, bi: usize, range: (f64, f64), v0: usize, v1: usize) {
        let Some(base) = self.bases.get(bi) else {
            return;
        };
        let mut inside = Vec::new();
        let rank = match base.kind {
            BaseKind::Edge => self.edge_side(bi) as u8,
            // A tangent curve cuts both faces as any other: where the result keeps both, the
            // pieces touch along it, an edge four faces use; where it keeps one, the two halves of
            // that face merge again across it.
            BaseKind::Cut { fa, fb, .. } => {
                // a cut has a curve, and its faces are taken in
                let (Some(c), Some(face_a), Some(face_b)) = (base.curve.as_ref(), self.faces.get(fa), self.faces.get(fb))
                else {
                    return;
                };
                let ends = [0.0, 0.5, 1.0].map(|f| c.eval(range.0 + (range.1 - range.0) * f));
                let (la, lb) = (locate_piece(&face_a.dom, ends), locate_piece(&face_b.dom, ends));
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
        if let Some(pieces) = self.base_pieces.get_mut(bi) {
            pieces.push(id);
        }
    }

    /// Which input an edge base came from: the first input's edges come first.
    pub(super) fn edge_side(&self, bi: usize) -> usize {
        usize::from(self.edge_base[1].first().is_some_and(|&b| bi >= b))
    }

    pub(super) fn piece_point(&self, pi: usize, f: f64) -> Point3 {
        // a piece not taken in has no point: NaN, which every test of a place reads as nowhere
        let Some((p, b)) = self.pieces.get(pi).and_then(|p| Some((p, self.bases.get(p.base)?))) else {
            return Point3::new(f64::NAN, f64::NAN, f64::NAN);
        };
        match &b.curve {
            Some(c) => c.eval(p.range.0 + (p.range.1 - p.range.0) * f),
            None => b.pole,
        }
    }

    /// Step 2b: pieces that run along each other become one, the lowest-ranked.
    pub(super) fn dedup(&mut self) {
        let n = self.pieces.len();
        let mut parent: Vec<usize> = (0..n).collect();
        // walked into a union-find whose roots are each class's least (rank, index): its order
        // reaches nothing
        let mut by_ends: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (i, p) in self.pieces.iter().enumerate() {
            if self.bases.get(p.base).is_none_or(|b| b.curve.is_none()) {
                continue;
            }
            by_ends
                .entry((p.v0.min(p.v1), p.v0.max(p.v1)))
                .or_default()
                .push(i);
        }
        for group in by_ends.values() {
            for (k, &i) in group.iter().enumerate() {
                for &j in group.iter().skip(k + 1) {
                    let (Some(pi), Some(pj)) = (self.pieces.get(i), self.pieces.get(j)) else {
                        continue;
                    };
                    if pi.base == pj.base {
                        continue;
                    }
                    let (Some(bi), Some(bj)) = (self.bases.get(pi.base), self.bases.get(pj.base)) else {
                        continue;
                    };
                    // pieces of curveless bases were left out above
                    let Some(cj) = bj.curve.as_ref() else {
                        continue;
                    };
                    let within = tol::coincidence(bj.tol, bi.tol);
                    let same = [0.25, 0.5, 0.75].iter().all(|&f| {
                        csi::curve_distance(cj, pj.range, self.piece_point(i, f)).0 <= within
                    });
                    if same {
                        let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                        if ri != rj {
                            let key = |x: usize| (self.pieces.get(x).map_or(u8::MAX, |p| p.rank), x);
                            let (low, high) = if key(ri) < key(rj) { (ri, rj) } else { (rj, ri) };
                            if let Some(up) = parent.get_mut(high) {
                                *up = low;
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
                // one geometry for every piece, in order: a piece's place is its id
                let b = self.bases.get(p.base);
                PieceGeom {
                    curve: b.and_then(|b| b.curve.clone()),
                    range: p.range,
                    v0: p.v0,
                    v1: p.v1,
                    tol: b.map_or(tol::LINEAR, |b| b.tol),
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Line;

    fn along_x() -> Curve {
        Curve::Line(Line { origin: Vec3::ZERO, dir: Vec3::X })
    }

    #[test]
    fn a_stretch_between_two_merged_vertices_is_held_to_the_end_nearer_the_vertex() {
        let reach = 3.0 * tol::LINEAR;
        // Ends 2 tolerances apart, merged at a point a hair from the second end.
        let (t0, t1) = (0.0, 2.0 * tol::LINEAR);
        let at = Vec3::new(1.9 * tol::LINEAR, 0.0, 0.0);
        assert_eq!(merged_stretch(&along_x(), (t0, t1), at, reach), Some(t1));
        let at = Vec3::new(0.1 * tol::LINEAR, 0.0, 0.0);
        assert_eq!(merged_stretch(&along_x(), (t0, t1), at, reach), Some(t0));
    }

    #[test]
    fn a_stretch_longer_than_the_vertices_reach_stays_a_piece_of_its_own() {
        let reach = 3.0 * tol::LINEAR;
        let (t0, t1) = (0.0, 4.0 * tol::LINEAR);
        assert_eq!(merged_stretch(&along_x(), (t0, t1), Vec3::ZERO, reach), None);
    }
}
