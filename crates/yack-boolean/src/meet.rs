// yack: step 1a-b: edges pierce faces, edges cross and graze each other, imprints.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only.
//! Step 1a-b: edges pierce faces, edges cross and graze each other, imprints.

use super::*;
use super::bounds::{cell, overlap, padded, locate_piece};

#[allow(clippy::disallowed_types)] impl<'a> Work<'a> { // D26: BTree* in hardening, proven by digests
    pub(super) fn pierce(&mut self) {
        let mut found: Vec<Hit> = Vec::new();
        let mut on = Vec::new();
        // An edge one face uses on both sides (a seam, or a slit a contact left) bounds no
        // material: the face's surface goes on across it. Lying on the other solid's face it marks
        // no change of side there, and where the two surfaces cross at a shallow angle a seam runs
        // within the tolerance of the other face along its length without being a cut.
        let one_face: HashSet<usize> = self
            .edge_faces()
            .into_iter()
            .filter(|(_, fs)| fs.len() == 1)
            .map(|(bi, _)| bi)
            .collect();
        for side in 0..2 {
            for &bi in &self.edge_base[side] {
                let base = &self.bases[bi];
                let Some(curve) = &base.curve else { continue };
                let bounds_nothing = one_face.contains(&bi);
                for &fi in self.other(side) {
                    let face = &self.faces[fi];
                    if !overlap(&base.bbox, &face.bbox)
                        || self.on_face.contains(&(bi, fi))
                        || !self.edge_face_near(side, bi, fi)
                    {
                        continue;
                    }
                    let range = match &**curve {
                        Curve::Line(l) => {
                            match ssi::clip_line(l.origin, l.dir, &padded(&face.bbox, 1e-6)) {
                                Some((a, b)) => (a.max(base.range.0), b.min(base.range.1)),
                                None => continue,
                            }
                        }
                        _ => base.range,
                    };
                    if range.1 < range.0 {
                        continue;
                    }
                    if base.tol > tol::LINEAR
                        && lies_within(curve, base.range, &face.dom.surface, base.tol)
                    {
                        if !bounds_nothing {
                            on.push((bi, fi));
                        }
                        continue;
                    }
                    let hits = csi::curve_surface(curve, range, &face.dom.surface);
                    if hits.on_surface {
                        if !bounds_nothing {
                            on.push((bi, fi));
                        }
                        continue;
                    }
                    for t in hits.ts {
                        let p = curve.eval(t);
                        let at = face.dom.locate(p);
                        if at != Where::Outside {
                            let s = &face.dom.surface;
                            let uv = s.project(p);
                            let tangent = curve.d1(t).1.unit_or_zero();
                            let sine = tangent.dot(s.normal(uv.x, uv.y)).abs();
                            // Grazing the face along its boundary, the edge runs along a boundary
                            // edge: the vertices of that edge-edge contact are each one's ends
                            // lying on the other, and roots along the band are only rounding.
                            // Beside a pole the parameter plane puts a point a hair from the pole
                            // on the boundary though no edge is near it in space, and a seam bounds
                            // no material: a root near neither is where the edge crosses the face.
                            let seam = |c: &Rc<Curve>| {
                                face.dom
                                    .trims()
                                    .filter(|o| o.curve.as_ref().is_some_and(|x| Rc::ptr_eq(x, c)))
                                    .count()
                                    > 1
                            };
                            if at == Where::Boundary
                                && sine <= tol::TANGENT
                                && face.dom.trims().any(|tr| {
                                    tr.curve.as_ref().is_some_and(|c| {
                                        let r = (tr.t0.min(tr.t1), tr.t0.max(tr.t1));
                                        !seam(c) && csi::within(c, r, p, tol::LINEAR)
                                    })
                                })
                            {
                                continue;
                            }
                            found.push(Hit {
                                p,
                                t,
                                base: bi,
                                face: fi,
                                sine,
                            });
                        }
                    }
                }
            }
        }
        self.hits = found;
        self.on_face.extend(on);
    }

    /// Records each fact that an edge lies on a face, once it is checked at the edge's ends and
    /// quarter points.
    pub(super) fn take_facts(&mut self, facts: &Facts) -> Result<()> {
        for f in &facts.on_face {
            let (side, other) = match f.side {
                0 => (0, 1),
                1 => (1, 0),
                _ => {
                    return Err(Error::InvalidInput(
                        "a fact names an input other than 0 or 1".into(),
                    ));
                }
            };
            let (Some(&bi), Some(&fi)) = (
                self.edge_base[side].get(f.edge.index()),
                self.face_of[other].get(f.face.index()),
            ) else {
                return Err(Error::InvalidInput(
                    "a fact names an edge or face not in its input".into(),
                ));
            };
            let base = &self.bases[bi];
            let surface = &self.faces[fi].dom.surface;
            let holds = (0..=4).all(|k| {
                let p = match &base.curve {
                    Some(c) => {
                        c.eval(base.range.0 + (base.range.1 - base.range.0) * k as f64 / 4.0)
                    }
                    None => base.pole,
                };
                csi::signed_distance(surface, p).abs() <= base.tol
            });
            if !holds {
                return Err(Error::InvalidInput(format!(
                    "the fact that edge {} of input {side} lies on face {} of the other does not \
                     hold",
                    f.edge.index(),
                    f.face.index()
                )));
            }
            self.on_face.insert((bi, fi));
        }
        Ok(())
    }

    /// Each edge base's faces, by face index.
    pub(super) fn edge_faces(&self) -> HashMap<usize, Vec<usize>> {
        let mut out: HashMap<usize, Vec<usize>> = HashMap::new();
        for (f, face) in self.faces.iter().enumerate() {
            let b = self.breps[face.side];
            for &lp in &b.face(face.id).loops {
                for &c in &b.lp(lp).coedges {
                    let eb = self.edge_base[face.side][b.coedge(c).edge.index()];
                    let list = out.entry(eb).or_default();
                    if !list.contains(&f) {
                        list.push(f);
                    }
                }
            }
        }
        out
    }

    /// Step 2c: an edge lying on a face of the other solid cuts that face, as the face pair's
    /// intersection curve along it would: where the edge's own faces cross the face, and where they
    /// only touch it, a contact the result may keep both sides of.
    pub(super) fn imprint_on_faces(&mut self) {
        for &(bi, fi) in &self.on_face {
            for &pi in &self.base_pieces[bi] {
                if self.bases[bi].curve.is_none() {
                    continue;
                }
                let ends = [0.0, 0.5, 1.0].map(|f| self.piece_point(pi, f));
                if !matches!(locate_piece(&self.faces[fi].dom, ends), Where::Inside(_)) {
                    continue;
                }
                if !self.cut_faces[pi].contains(&fi) {
                    self.cut_faces[pi].push(fi);
                }
            }
        }
    }

    /// Step 1: every meeting point. Edges' meetings with faces are placed last, once the faces'
    /// intersection curves are known.
    pub(super) fn meet(&mut self) -> Result<()> {
        self.pierce();
        self.cross_edges();
        self.intersect_faces()?;
        self.place_hits();
        Ok(())
    }

    /// Places step 1a's meetings, steepest first, so where several computations find one grazing
    /// meeting the vertex is the one the best-conditioned of them found, whatever order they ran in.
    pub(super) fn place_hits(&mut self) {
        let mut hits = std::mem::take(&mut self.hits);
        hits.sort_by(|a, b| {
            b.sine
                .total_cmp(&a.sine)
                .then_with(|| a.p.x.total_cmp(&b.p.x))
                .then_with(|| a.p.y.total_cmp(&b.p.y))
                .then_with(|| a.p.z.total_cmp(&b.p.z))
                .then_with(|| (a.base, a.face).cmp(&(b.base, b.face)))
        });
        let edge_faces = self.edge_faces();
        let mut cuts: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        for (bi, base) in self.bases.iter().enumerate() {
            if let BaseKind::Cut {
                fa,
                fb,
                tangent: false,
            } = base.kind
            {
                cuts.entry((fa.min(fb), fa.max(fb))).or_default().push(bi);
            }
        }
        for h in hits {
            let (mut p, mut t, mut on) = (h.p, h.t, None);
            // The steepest crossing, then the nearest.
            let mut best = (0.0, f64::INFINITY);
            let own = edge_faces.get(&h.base).map_or(&[][..], Vec::as_slice);
            for &f in own {
                let key = (f.min(h.face), f.max(h.face));
                for &ci in cuts.get(&key).map_or(&[][..], Vec::as_slice) {
                    if let Some((q, tq, sine)) = self.crossing(ci, h, own) {
                        let d = q.dist(h.p);
                        if sine > best.0 || (sine == best.0 && d < best.1) {
                            (p, t, on, best) = (q, tq, Some(ci), (sine, d));
                        }
                    }
                }
            }
            let v = self.touch_vertex(p, t, h.base, h.face, on);
            // Two surfaces each known to the tolerance fix where they cross to the tolerance over
            // the sine between them: a grazing meeting lies that far along the graze.
            let reach = (tol::LINEAR / h.sine.max(1e-12)).min(GRAZE_REACH);
            if reach > tol::LINEAR {
                let pairs = own.iter().map(|&f| (f.min(h.face), f.max(h.face), reach));
                self.meetings.entry(v).or_default().extend(pairs);
            }
            if h.sine <= tol::TANGENT {
                self.met.push((v, h.base, h.face));
            }
        }
    }

    /// Where intersection curve `ci`, of one of the faces `own` of the hit's edge and the hit's face,
    /// crosses the edge within the band where the edge stays on the face from the hit. Where the
    /// faces nearly coincide, the edge grazes the face and the hit can lie anywhere in that band;
    /// the crossing lies on the curve the faces are cut along, and is no worse placed, since inside
    /// the edge's own face the edge crosses the curve at least as steeply as it crosses the face.
    pub(super) fn crossing(&self, ci: usize, h: Hit, own: &[usize]) -> Option<(Point3, f64, f64)> {
        let cut = &self.bases[ci];
        let edge = &self.bases[h.base];
        let (c, e) = (cut.curve.as_ref()?, edge.curve.as_ref()?);
        let BaseKind::Cut { fa, fb, .. } = cut.kind else {
            return None;
        };
        let mine = if own.contains(&fa) { fa } else { fb };
        // The edge's other face: the curve crosses it where it crosses the edge.
        let across = match own {
            [f, g] => Some(&self.faces[if *f == mine { *g } else { *f }].dom.surface),
            _ => None,
        };
        let (mut q, mut tq, sine) =
            csi::curve_curve_near(c, cut.range, e, edge.range, h.p, h.t, across)?;
        let face = &self.faces[h.face];
        // A fitted cut curve puts the crossing off the exact surfaces by its fit error; where the
        // three surfaces fix a point within the tolerance of it, that point is the meeting.
        if let Some(b) = across {
            let exact = csi::on_three([&self.faces[mine].dom.surface, b, &face.dom.surface], q)
                .filter(|x| x.dist(q) <= tol::LINEAR)
                .map(|x| (x, csi::curve_distance(e, edge.range, x)))
                .filter(|(_, (d, _))| *d <= tol::LINEAR);
            if let Some((x, (_, tx))) = exact {
                (q, tq) = (x, tx);
            }
        }
        let grazes = |k: usize| {
            let m = e.eval(h.t + (tq - h.t) * k as f64 / 16.0);
            csi::signed_distance(&face.dom.surface, m).abs() <= tol::LINEAR
        };
        // On the edge, the point is on its own faces; where the hit was, it is on the face.
        ((0..=16).all(grazes)
            && (q.dist(h.p) <= tol::LINEAR || face.dom.locate(q) != Where::Outside))
            .then_some((q, tq, sine))
    }

    /// The vertex where base `bi` meets face `fi` at `p`, its parameter `t`. Where the edge only
    /// touches the face, rounding hides the gap along a band, and the meeting found can lie
    /// anywhere in it: a vertex already on both, with the edge within the tolerance of the face
    /// all the way to it, is the same meeting, as two crossings with no excursion past the
    /// tolerance between them are one touching point. A meeting placed on intersection curve `on`
    /// is only that curve's: a vertex off it is another meeting in the same band, unless an earlier
    /// hit left it where it was found, on no curve. That one is this meeting, found from the other
    /// solid's edge, and anywhere in the band: it moves onto the curve, so the two solids' edges
    /// and the curve all pass one vertex where they cross.
    #[allow(clippy::panic)] pub(super) fn touch_vertex(
        &mut self,
        p: Point3,
        t: f64,
        bi: usize,
        fi: usize,
        on: Option<usize>,
    ) -> usize {
        let bt = self.bases[bi].tol;
        if bt > tol::LINEAR || self.widest > tol::LINEAR {
            // A meeting on a tolerant edge takes the edge's tolerance (Jackson 1995).
            if let Some(i) = self.coincident_vertex(p, bt) {
                return i;
            }
            if bt > tol::LINEAR {
                return self.vertex_with(p, bt);
            }
        }
        let base = &self.bases[bi];
        let curve = base.curve.clone().unwrap_or_else(|| panic!("pierced edges have curves"));
        let range = base.range;
        let surface = &self.faces[fi].dom.surface;
        let near = |d: f64| d.abs() <= tol::LINEAR;
        let on_cut = |q: Point3| {
            on.is_none_or(|ci| {
                let cut = &self.bases[ci];
                let c = cut.curve.as_ref().unwrap_or_else(|| panic!("cuts have curves"));
                csi::within(c, cut.range, q, tol::LINEAR)
            })
        };
        let c = cell(p);
        let mut best: Option<(f64, usize)> = None;
        let mut placed_here = false;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(ids) = self.grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) else {
                        continue;
                    };
                    for &i in ids {
                        let q = self.pool[i];
                        let d = q.dist(p);
                        if d <= tol::LINEAR {
                            return i;
                        }
                        if d > CELL || best.is_some_and(|b| b.0 <= d) {
                            continue;
                        }
                        let (off, tq) = csi::curve_distance(&curve, range, q);
                        let along = (1..8).all(|k| {
                            let m = curve.eval(t + (tq - t) * k as f64 / 8.0);
                            near(csi::signed_distance(surface, m))
                        });
                        let unplaced =
                            on.is_some() && self.unplaced.get(&i).is_some_and(|&r| d <= r);
                        if near(off)
                            && near(csi::signed_distance(surface, q))
                            && along
                            && (unplaced || on_cut(q))
                        {
                            best = Some((d, i));
                            placed_here = unplaced && !on_cut(q);
                        }
                    }
                }
            }
        }
        match best {
            Some((_, i)) => {
                if placed_here {
                    self.pool[i] = p;
                    self.grid.entry(cell(p)).or_default().push(i);
                    self.unplaced.remove(&i);
                }
                i
            }
            None => {
                let v = self.vertex(p);
                if on.is_none() {
                    self.unplaced.insert(v, GRAZE_REACH);
                }
                v
            }
        }
    }

    /// Step 1b: where edges of the two solids cross; this is what places vertices where a face's
    /// boundary crosses a coincident face's.
    pub(super) fn cross_edges(&mut self) {
        let mut found = Vec::new();
        for &ba in &self.edge_base[0] {
            let a = &self.bases[ba];
            let Some(ca) = &a.curve else { continue };
            for &bb in &self.edge_base[1] {
                let b = &self.bases[bb];
                let Some(cb) = &b.curve else { continue };
                if !overlap(&padded(&a.bbox, tol::coincidence(a.tol, b.tol)), &b.bbox)
                    || !self.edges_near(ba, bb)
                {
                    continue;
                }
                let t = a.tol.max(b.tol);
                found.extend(
                    csi::curve_curve(ca, a.range, cb, b.range)
                        .into_iter()
                        .map(|p| (p, t, ba, bb)),
                );
            }
        }
        for (p, t, ba, bb) in found {
            if t <= tol::LINEAR && self.touching_vertex(p, ba, bb).is_some() {
                continue;
            }
            // Two edges crossing at a shallow angle fix their crossing only loosely along them:
            // to the tolerance over the sine between them. Edges crossing square fix it to the
            // tolerance, and a meeting of other edges further off is another one.
            let v = self.vertex_with(p, t);
            if t <= tol::LINEAR && v >= self.inputs {
                let tangent = |b: usize| -> Option<Vec3> {
                    let base = &self.bases[b];
                    let c = base.curve.as_ref()?;
                    let (_, at) = csi::curve_distance(c, base.range, p);
                    c.d1(at).1.unit()
                };
                let sine = match (tangent(ba), tangent(bb)) {
                    (Some(x), Some(y)) => x.cross(y).norm(),
                    _ => 0.0,
                };
                let reach = (tol::LINEAR / sine.max(1e-12)).min(GRAZE_REACH);
                let r = self.unplaced.entry(v).or_insert(0.0);
                *r = r.max(reach);
            }
        }
        self.graze_edges();
    }

    /// Step 1b, after the meetings: where edges of the two solids run within [`csi::GRAZE_GAP`] of
    /// each other and cross ([`csi::grazing`]). A seam bounds no material and is left out.
    pub(super) fn graze_edges(&mut self) {
        let owners = self.edge_faces();
        let seam = |b: usize| owners.get(&b).is_some_and(|f| f.len() == 1);
        let mut parts = Vec::new();
        // Meetings the closed forms and steps missed: two splines a hair apart cross at angles of
        // 1e-7 rad, where only the gap's least finds them.
        for &ba in &self.edge_base[0] {
            let a = &self.bases[ba];
            let Some(ca) = &a.curve else { continue };
            if a.tol > tol::LINEAR || seam(ba) {
                continue;
            }
            for &bb in &self.edge_base[1] {
                let b = &self.bases[bb];
                let Some(cb) = &b.curve else { continue };
                if b.tol > tol::LINEAR
                    || seam(bb)
                    || !overlap(&padded(&a.bbox, tol::LINEAR), &b.bbox)
                    || !self.edges_near(ba, bb)
                {
                    continue;
                }
                parts.extend(csi::grazing(ca, a.range, cb, b.range));
            }
        }
        for p in parts {
            if self.pool.iter().all(|q| q.dist(p) > csi::GRAZE_GAP) {
                self.vertex(p);
            }
        }
    }

    /// A pool vertex on both edges `ba` and `bb` near their meeting `p`, the two staying within the
    /// tolerance of each other all the way from it to `p`. Where edges touch rather than cross,
    /// rounding hides the gap along a band and the meeting found can lie anywhere in it; a vertex
    /// already on both in that band is the same meeting.
    pub(super) fn touching_vertex(&self, p: Point3, ba: usize, bb: usize) -> Option<usize> {
        let near = |d: f64| d <= tol::LINEAR;
        let (a, b) = (&self.bases[ba], &self.bases[bb]);
        let (ca, cb) = (a.curve.as_ref()?, b.curve.as_ref()?);
        let (_, tp) = csi::curve_distance(ca, a.range, p);
        let c = cell(p);
        let mut best: Option<(f64, usize)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(ids) = self.grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) else {
                        continue;
                    };
                    for &i in ids {
                        let q = self.pool[i];
                        let d = q.dist(p);
                        if d > CELL || best.is_some_and(|x| x.0 <= d) {
                            continue;
                        }
                        let (off_a, tq) = csi::curve_distance(ca, a.range, q);
                        if !near(off_a) || !csi::within(cb, b.range, q, tol::LINEAR) {
                            continue;
                        }
                        let along = (1..8).all(|k| {
                            let m = ca.eval(tp + (tq - tp) * k as f64 / 8.0);
                            csi::within(cb, b.range, m, tol::LINEAR)
                        });
                        if along {
                            best = Some((d, i));
                        }
                    }
                }
            }
        }
        best.map(|x| x.1)
    }
}
