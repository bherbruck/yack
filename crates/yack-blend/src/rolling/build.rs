// yack: the rolling-ball fillet built: the ball rolled along the chain, its strip sewn into the body.
// Adapted from springrep@a03e8a1 src/blend/rolling.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/blend/rolling.rs to yack-blend (M3 W9a); paths and visibility only;
// split out of rolling.rs (M3 hardening R0), paths and visibility only.
//! The rolling-ball fillet built: the ball rolled along the chain, its strip sewn into the body.

use super::*;

/// The fillet of `radius` along the tangent chain `picked`, edge `i`'s faces named `names[i]`.
#[allow(clippy::disallowed_types)] #[allow(clippy::panic)] pub(crate) fn fillet( // D26: BTree* in hardening, proven by digests
    body: &Brep,
    picked: &[Picked],
    radius: f64,
    names: &[BTreeSet<FaceName>],
) -> Result<Brep> {
    if body.has_tolerances() {
        return unsupported("on a body with local tolerances");
    }
    if picked.iter().any(|pk| body.edge(pk.edge).curve.is_none()) {
        return unsupported("on a degenerate edge");
    }
    let Some((links, closed)) = chain(body, picked) else {
        return unsupported("on edges that do not run on from one to the next");
    };
    let Some(convex) = run::convex(body, picked[0].coedges[0]) else {
        return unsupported("between faces that meet tangentially");
    };
    if picked
        .iter()
        .any(|pk| run::convex(body, pk.coedges[0]) != Some(convex))
    {
        return unsupported("on a chain that is convex in places and concave in others");
    }
    let n = links.len();
    let total: f64 = links.iter().map(|l| l.len).sum();
    let chain_edges: BTreeSet<EdgeId> = links.iter().map(|l| l.edge).collect();
    let mut uses: HashMap<EdgeId, Vec<CoedgeId>> = HashMap::new();
    for c in body.coedge_ids() {
        uses.entry(body.coedge(c).edge).or_default().push(c);
    }
    let faces_of =
        |e: EdgeId| -> BTreeSet<FaceId> { uses[&e].iter().map(|&c| body.coedge_face(c)).collect() };
    let at_vertex = |v: VertexId| -> Vec<EdgeId> {
        body.edge_ids()
            .filter(|&e| {
                let x = body.edge(e);
                x.start == v || x.end == v
            })
            .collect()
    };
    let tangent = |link: &Link, leaving: bool| -> Vec3 {
        let e = body.edge(link.edge);
        let c = e.curve.as_ref().unwrap_or_else(|| panic!("checked above"));
        let at_end = leaving != link.forward;
        let t = if at_end { e.range.1 } else { e.range.0 };
        let d = c.d1(t).1.unit_or_zero();
        if link.forward == leaving { d } else { -d }
    };

    // The junctions at each inner chain vertex, and every other edge there refused.
    let inner = if closed { n } else { n - 1 };
    let mut joints: Vec<[Option<Junction>; 2]> = Vec::with_capacity(inner);
    let mut vertices: Vec<VertexId> = Vec::with_capacity(inner);
    for i in 0..inner {
        let (l, m) = (&links[i], &links[(i + 1) % n]);
        let v = body.coedge_end(picked[l.pk].coedges[0]);
        vertices.push(v);
        let at = body.vertex(v).point;
        let mut pair: [Option<Junction>; 2] = [None, None];
        let mut taken: BTreeSet<EdgeId> = BTreeSet::new();
        #[allow(clippy::needless_range_loop)]
        for k in 0..2 {
            let (fa, fb) = (l.faces[k], m.faces[k]);
            let found = at_vertex(v).into_iter().find(|&e| {
                !chain_edges.contains(&e)
                    && faces_of(e) == BTreeSet::from([fa, fb])
                    && (fa != fb || uses[&e].len() == 2)
            });
            let Some(e) = found else {
                if fa != fb {
                    return unsupported("where a side changes face without an edge between them");
                }
                continue;
            };
            taken.insert(e);
            let x = body.edge(e);
            let Some(c) = x.curve.as_ref() else {
                return unsupported("at a pole");
            };
            let _ = c;
            pair[k] = Some(Junction {
                edge: e,
                at,
                before: fa,
                after: fb,
                seam: fa == fb,
            });
        }
        if at_vertex(v)
            .iter()
            .any(|e| !chain_edges.contains(e) && !taken.contains(e))
        {
            return unsupported("through a vertex other faces meet");
        }
        joints.push(pair);
    }
    let roll = Roll {
        body,
        links,
        closed,
        total,
        radius,
        side: if convex { -1.0 } else { 1.0 },
        domains: picked
            .iter()
            .flat_map(|pk| pk.faces)
            .chain(
                joints
                    .iter()
                    .flatten()
                    .flatten()
                    .flat_map(|j| [j.before, j.after]),
            )
            .collect::<BTreeSet<FaceId>>()
            .into_iter()
            .map(|f| (f, face_domain(body, f)))
            .collect(),
        joints,
    };

    // An open chain's ends, each on the plane of the one other face there.
    let mut ends: Vec<End> = Vec::new();
    if !closed {
        for (at_start, li) in [(true, 0), (false, n - 1)] {
            let l = &roll.links[li];
            let v = if at_start {
                body.coedge_start(picked[l.pk].coedges[0])
            } else {
                body.coedge_end(picked[l.pk].coedges[0])
            };
            let others: BTreeSet<FaceId> = at_vertex(v)
                .iter()
                .flat_map(|&e| faces_of(e))
                .filter(|f| !l.faces.contains(f))
                .collect();
            let [f] = others.iter().copied().collect::<Vec<_>>()[..] else {
                return unsupported("ending where more than one other face meets it");
            };
            let mut found = Vec::with_capacity(2);
            for k in 0..2 {
                let Some(e) = at_vertex(v)
                    .into_iter()
                    .find(|&e| faces_of(e) == BTreeSet::from([f, l.faces[k]]))
                else {
                    return unsupported("ending where its end face meets a side at no edge");
                };
                found.push(e);
            }
            let edges = [found[0], found[1]];
            if at_vertex(v).len() != 3 {
                return unsupported("ending at a vertex where other edges meet");
            }
            // The side the chain comes from: a point a little way back along it.
            let surface = body.face(f).surface.clone();
            let back = tangent(l, at_start);
            let at = body.vertex(v).point;
            let uv = surface.project(at);
            let nrm = surface.normal(uv.x, uv.y);
            let sign = if nrm.dot(back) >= 0.0 { 1.0 } else { -1.0 };
            let s_end = if at_start { 0.0 } else { total };
            ends.push(End {
                vertex: v,
                face: f,
                surface,
                sign,
                edges,
                lo: s_end,
                hi: s_end,
            });
        }
    }
    let mut trims: Vec<Trim> = Vec::with_capacity(ends.len());
    for (k, e) in ends.iter_mut().enumerate() {
        let from = if k == 0 { 0.0 } else { total };
        let t = fit_trim(&roll, e, from)?;
        e.lo = t.samples.iter().map(|x| x.1).fold(f64::INFINITY, f64::min);
        e.hi = t
            .samples
            .iter()
            .map(|x| x.1)
            .fold(f64::NEG_INFINITY, f64::max);
        trims.push(t);
    }

    // From each end's vertex along its face's boundary, on each side, to the edge that side's
    // contact ends on: the edges passed over and the vertices between them go with the blend.
    let mut last: Vec<[(EdgeId, bool); 2]> = Vec::with_capacity(ends.len());
    let mut dropped_edges: BTreeSet<EdgeId> = BTreeSet::new();
    let mut dropped_vertices: BTreeSet<VertexId> = BTreeSet::new();
    for (ti, e) in ends.iter().enumerate() {
        let mut found = Vec::with_capacity(2);
        for k in 0..2 {
            let p = trims[ti].curve.eval(k as f64);
            let (mut edge, mut from) = (e.edges[k], e.vertex);
            let mut steps = 0;
            loop {
                let x = body.edge(edge);
                let c = x.curve.as_ref().unwrap_or_else(|| panic!("an end face's edge has a curve"));
                let starts = x.start == from;
                // A line, circle or ellipse runs on past its ends, so the first edge's contact may
                // lie past the vertex, where the blend opens the end face.
                let span = x.range.1 - x.range.0;
                let reach = match c {
                    Curve::Line(_) | Curve::Circle(_) | Curve::Ellipse(_) if steps == 0 => {
                        (x.range.0 - span, x.range.1 + span)
                    }
                    _ => x.range,
                };
                if let Ok(t) = nearest_param(c, p, reach) {
                    let inside = t >= x.range.0 - tol::LINEAR && t <= x.range.1 + tol::LINEAR;
                    // Where the end face turns past a half turn at the vertex, the blend opens it
                    // beyond the corner: its first edge runs on past the vertex to the contact.
                    let past = steps == 0 && if starts { t < x.range.0 } else { t > x.range.1 };
                    if (inside || past) && c.eval(t).dist(p) <= 10.0 * tol::LINEAR {
                        found.push((edge, starts));
                        break;
                    }
                }
                steps += 1;
                let w = if starts { x.end } else { x.start };
                let onward: Vec<EdgeId> = at_vertex(w).into_iter().filter(|&o| o != edge).collect();
                let (&[next], true) = (onward.as_slice(), steps < 6) else {
                    return unsupported("whose contact ends past more of its end face than a run");
                };
                if !faces_of(next).contains(&e.face) {
                    return unsupported("whose contact ends off its end face");
                }
                dropped_edges.insert(edge);
                dropped_vertices.insert(w);
                (edge, from) = (next, w);
            }
        }
        last.push([found[0], found[1]]);
    }

    // Segment boundaries: every chain vertex, every crossing, and an open chain's two ends past
    // its trims.
    // An open chain's ends bound segments too: past them it runs on straight, a kink in the
    // sections' turning that no one cubic follows.
    let mut cuts: Vec<f64> = roll.links.iter().map(|l| l.s0).collect();
    if !closed {
        cuts.push(total);
    }
    let mut crossing: Vec<[Option<f64>; 2]> = Vec::with_capacity(inner);
    for i in 0..inner {
        let mut c = [None, None];
        for (k, ck) in c.iter_mut().enumerate() {
            *ck = roll.crossing(i, k)?.map(|t| roll.wrap(t));
            if let Some(t) = *ck {
                cuts.push(t);
            }
        }
        crossing.push(c);
    }
    let margin = 0.25 * radius;
    let (lo, hi) = if closed {
        (0.0, total)
    } else {
        (ends[0].lo - margin, ends[1].hi + margin)
    };
    if !closed {
        cuts.retain(|&t| t > lo && t < hi);
        cuts.push(lo);
        cuts.push(hi);
    }
    cuts.sort_by(f64::total_cmp);
    let close = |a: f64, b: f64| {
        let d = if closed {
            let d = (a - b).rem_euclid(total);
            d.min(total - d)
        } else {
            (a - b).abs()
        };
        d <= 1e-9 * total.max(1.0)
    };
    let mut bounds: Vec<f64> = Vec::new();
    for t in cuts {
        if !bounds.iter().any(|&b| close(t, b)) {
            bounds.push(t);
        }
    }
    let m = if closed {
        bounds.len()
    } else {
        bounds.len() - 1
    };
    let mut segments = Vec::with_capacity(m);
    for j in 0..m {
        let a = bounds[j];
        let b = if j + 1 < bounds.len() {
            bounds[j + 1]
        } else {
            bounds[0] + total
        };
        segments.push(fit(&roll, a, b)?);
    }
    let bound_index =
        |t: f64| -> Option<usize> { (0..bounds.len()).find(|&x| close(t, bounds[x])) };

    // Where each segment is kept: `s` from `lower(v)` to `upper(v)`.
    let start_trim = (!closed).then(|| &trims[0]);
    let end_trim = (!closed).then(|| &trims[1]);
    let lower = |j: usize, v: f64| match start_trim {
        Some(t) => segments[j].a.max(t.s_at(v)),
        None => segments[j].a,
    };
    let upper = |j: usize, v: f64| match end_trim {
        Some(t) => segments[j].b.min(t.s_at(v)),
        None => segments[j].b,
    };
    let alive: Vec<bool> = (0..m)
        .map(|j| (0..=64).any(|i| upper(j, i as f64 / 64.0) > lower(j, i as f64 / 64.0) + 1e-9))
        .collect();

    // The rebuilt solid.
    let mut out = Brep::new();
    let mut gone: BTreeSet<VertexId> = vertices.iter().copied().collect();
    gone.extend(ends.iter().map(|e| e.vertex));
    gone.extend(dropped_vertices.iter().copied());
    let mut vmap: Vec<Option<VertexId>> = vec![None; body.vertex_ids().count()];
    for w in body.vertex_ids() {
        if !gone.contains(&w) {
            vmap[w.index()] = Some(out.add_vertex(body.vertex(w).point));
        }
    }

    // Vertices on each segment's corners and on the trims, shared by position.
    let mut pool: Vec<VertexId> = Vec::new();
    let mut vertex_at = |out: &mut Brep, p: Point3| -> VertexId {
        if let Some(&w) = pool
            .iter()
            .find(|&&w| out.vertex(w).point.dist(p) <= tol::LINEAR)
        {
            return w;
        }
        let w = out.add_vertex(p);
        pool.push(w);
        w
    };
    // The trims' curves, split where a segment's arc meets them.
    let mut trim_splits: Vec<BTreeSet<OrderedF64>> = vec![BTreeSet::new(); trims.len()];
    // Where each trim meets each bound's arc: (bound index, v).
    let mut meets: Vec<(usize, f64)> = Vec::new();
    for (ti, t) in trims.iter().enumerate() {
        let mut vs = vec![0.0, 1.0];
        for (bi, &b) in bounds.iter().enumerate() {
            for w in t.samples.windows(2) {
                let ((v0, s0), (v1, s1)) = (w[0], w[1]);
                if (s0 - b) * (s1 - b) < 0.0 {
                    // Exactly where the true trim meets the bound's arc, so the two share a vertex.
                    let g = |v: f64| Ok(roll.trim_at(&ends[ti], v, s0)? - b);
                    let v = bisect(v0, v1, s0 - b, g)?;
                    vs.push(v);
                    meets.push((bi, v));
                }
            }
        }
        // The trim's own ends first, so a crossing a hair from either merges into it.
        vs[2..].sort_by(f64::total_cmp);
        for v in vs {
            let p = t.curve.eval(v);
            let near = |x: &OrderedF64| t.curve.eval(x.0).dist(p) <= 10.0 * tol::LINEAR;
            if !trim_splits[ti].iter().any(near) {
                trim_splits[ti].insert(OrderedF64(v));
            }
        }
    }
    let mut trim_edges: Vec<BTreeMap<OrderedF64, EdgeId>> = vec![BTreeMap::new(); trims.len()];
    for (ti, t) in trims.iter().enumerate() {
        let vs: Vec<f64> = trim_splits[ti].iter().map(|x| x.0).collect();
        for w in vs.windows(2) {
            let a = vertex_at(&mut out, t.curve.eval(w[0]));
            let b = vertex_at(&mut out, t.curve.eval(w[1]));
            let e = out.add_edge(Curve::BSpline(t.curve.clone()), (w[0], w[1]), a, b);
            trim_edges[ti].insert(OrderedF64(w[0]), e);
        }
    }
    // Each inner bound's arc, split where a trim crosses it, kept where some segment keeps it.
    let mut arcs: Vec<Vec<(f64, f64, EdgeId)>> = vec![Vec::new(); bounds.len()];
    for (bi, &b) in bounds.iter().enumerate() {
        let sec = roll.section(b, None)?;
        let curve = sec.arc_curve(radius)?;
        let mut vs = vec![0.0, 1.0];
        let _ = b;
        for &(at, v) in &meets {
            let snapped = trim_splits
                .iter()
                .flatten()
                .map(|x| x.0)
                .find(|&x| (x - v).abs() <= 1e-6)
                .unwrap_or(v);
            if at == bi && snapped > 0.0 && snapped < 1.0 {
                vs.push(snapped);
            }
        }
        vs.sort_by(f64::total_cmp);
        for w in vs.windows(2) {
            let mid = 0.5 * (w[0] + w[1]);
            let kept_by = |j: usize| -> bool {
                alive[j] && lower(j, mid) < upper(j, mid) - 1e-9 && {
                    let (a, c) = (lower(j, mid), upper(j, mid));
                    close(a, b) || close(c, b)
                }
            };
            if !(0..m).any(kept_by) {
                continue;
            }
            let a = vertex_at(&mut out, curve.eval(w[0]));
            let c = vertex_at(&mut out, curve.eval(w[1]));
            let e = out.add_edge(Curve::Nurbs(Box::new(curve.clone())), (w[0], w[1]), a, c);
            arcs[bi].push((w[0], w[1], e));
        }
    }
    // Each kept segment's two contact pieces.
    let mut pieces: Vec<[Option<EdgeId>; 2]> = vec![[None, None]; m];
    for j in 0..m {
        if !alive[j] {
            continue;
        }
        for (k, v) in [(0, 0.0), (1, 1.0)] {
            let (a, b) = (lower(j, v), upper(j, v));
            if b <= a + 1e-9 {
                continue;
            }
            let c = Curve::BSpline(segments[j].contact[k].clone());
            let va = vertex_at(&mut out, c.eval(a));
            let vb = vertex_at(&mut out, c.eval(b));
            pieces[j][k] = Some(out.add_edge(c, (a, b), va, vb));
        }
    }

    // The body's edges: each junction cut back to where its side's contact crosses it, each end
    // face's edges with the sides cut back to the contacts' ends.
    // Per edge, the point its start is cut back to and the point its end is.
    let mut cut_at: HashMap<EdgeId, [Option<Point3>; 2]> = HashMap::new();
    let mut cut = |e: EdgeId, starts: bool, p: Point3| -> Result<()> {
        let slot = &mut cut_at.entry(e).or_default()[if starts { 0 } else { 1 }];
        if slot.replace(p).is_some() {
            return unsupported("that cuts one end of an edge back twice");
        }
        Ok(())
    };
    for i in 0..inner {
        for (k, &crossed) in crossing[i].iter().enumerate() {
            if let (Some(j), Some(t)) = (&roll.joints[i][k], crossed) {
                if bound_index(t).is_none() {
                    return unsupported("whose crossing is not a segment's end");
                }
                let p = roll.section(t, None)?.contact[k];
                cut(j.edge, body.edge(j.edge).start == vertices[i], p)?;
            }
        }
    }
    for (ti, l) in last.iter().enumerate() {
        for (k, &(edge, starts)) in l.iter().enumerate() {
            cut(edge, starts, trims[ti].curve.eval(k as f64))?;
        }
    }
    let mut emap: Vec<Option<EdgeId>> = vec![None; body.edge_ids().count()];
    for e in body.edge_ids() {
        if chain_edges.contains(&e) || dropped_edges.contains(&e) {
            continue;
        }
        let x = body.edge(e);
        let Some(c) = x.curve.clone() else {
            let Some(w) = vmap[x.start.index()] else {
                return unsupported("through a pole at a chain vertex");
            };
            emap[e.index()] = Some(out.add_degenerate_edge(w, x.range));
            continue;
        };
        let mut range = x.range;
        let (mut na, mut nb) = (vmap[x.start.index()], vmap[x.end.index()]);
        if let Some(&[at_start, at_end]) = cut_at.get(&e) {
            // An edge may run on past its end to a contact, where the blend opens its face.
            let span = x.range.1 - x.range.0;
            let reach = match c {
                Curve::Line(_) | Curve::Circle(_) | Curve::Ellipse(_) => {
                    (x.range.0 - span, x.range.1 + span)
                }
                _ => x.range,
            };
            if let Some(p) = at_start {
                range.0 = nearest_param(&c, p, reach)?;
                na = Some(vertex_at(&mut out, p));
            }
            if let Some(p) = at_end {
                range.1 = nearest_param(&c, p, reach)?;
                nb = Some(vertex_at(&mut out, p));
            }
            let off =
                |t: f64, p: Option<Point3>| p.is_some_and(|p| c.eval(t).dist(p) > tol::LINEAR);
            if off(range.0, at_start) || off(range.1, at_end) {
                return unsupported("that cuts an edge to a point off its curve");
            }
            if range.1 - range.0 <= tol::LINEAR {
                return unsupported("that cuts an edge back past its other end");
            }
        }
        let (Some(na), Some(nb)) = (na, nb) else {
            return unsupported("along an edge whose vertex another edge ends on");
        };
        emap[e.index()] = Some(out.add_edge(c, range, na, nb));
    }

    // The body's faces: each run of chain edges swapped for the contact pieces on that face, and
    // each end face's corner swapped for its trim curve.
    let mut fmap: Vec<Option<FaceId>> = vec![None; body.face_ids().count()];
    for f in body.face_ids() {
        let face = body.face(f);
        let nf = out.add_face(
            face.surface.clone(),
            face.reversed,
            face.names.iter().cloned(),
        );
        fmap[f.index()] = Some(nf);
        let end_here: Vec<usize> = (0..ends.len()).filter(|&t| ends[t].face == f).collect();
        for &lp in &face.loops {
            let coedges = &body.lp(lp).coedges;
            let is_chain = |c: CoedgeId| chain_edges.contains(&body.coedge(c).edge);
            let first = coedges.iter().position(|&c| !is_chain(c)).unwrap_or(0);
            let order: Vec<CoedgeId> = (0..coedges.len())
                .map(|i| coedges[(first + i) % coedges.len()])
                .collect();
            let mut specs: Vec<CoedgeSpec> = Vec::new();
            let mut hint: Option<Vec2> = None;
            let push_new = |specs: &mut Vec<CoedgeSpec>,
                            hint: &mut Option<Vec2>,
                            out: &Brep,
                            e: EdgeId,
                            reversed: bool| {
                let x = out.edge(e);
                let curve = x.curve.as_ref().unwrap_or_else(|| panic!("a new edge has a curve"));
                let (ta, tb) = if reversed {
                    (x.range.1, x.range.0)
                } else {
                    (x.range.0, x.range.1)
                };
                let start = curve.eval(ta);
                let seed = match *hint {
                    Some(h) => face.surface.project_near(start, h),
                    None => face.surface.project(start),
                };
                let mut uv = seed;
                for q in 1..=32 {
                    let t = ta + (tb - ta) * q as f64 / 32.0;
                    uv = face.surface.project_near(curve.eval(t), uv);
                }
                *hint = Some(uv);
                specs.push(CoedgeSpec {
                    edge: e,
                    reversed,
                    pcurve: PCurve::Projected { seed },
                });
            };
            let mut i = 0;
            while i < order.len() {
                let c = order[i];
                if dropped_edges.contains(&body.coedge(c).edge) {
                    i += 1;
                    continue;
                }
                if !is_chain(c) {
                    let co = body.coedge(c);
                    let ne = emap[co.edge.index()].unwrap_or_else(|| panic!("every other edge is copied"));
                    let x = out.edge(ne);
                    let (from, to) = if co.reversed {
                        (x.end, x.start)
                    } else {
                        (x.start, x.end)
                    };
                    let orig = body.edge(co.edge);
                    let orig_end = if co.reversed {
                        orig.range.0
                    } else {
                        orig.range.1
                    };
                    let end_uv = body.coedge_uvs(c, &[orig_end])[0];
                    let pcurve = match &co.pcurve {
                        PCurve::Projected { seed } => PCurve::Projected {
                            seed: face.surface.project_near(out.vertex(from).point, *seed),
                        },
                        other => other.clone(),
                    };
                    specs.push(CoedgeSpec {
                        edge: ne,
                        reversed: co.reversed,
                        pcurve,
                    });
                    hint = Some(face.surface.project_near(out.vertex(to).point, end_uv));
                    // At an end face's corner, its trim curve runs from this edge to the next.
                    for &t in &end_here {
                        // The walked-to edge on side k, cut where this coedge now ends.
                        let k = last[t]
                            .iter()
                            .position(|&(x, starts)| x == co.edge && starts == co.reversed);
                        let Some(k) = k else {
                            continue;
                        };
                        // From side k's contact end (v = k) to the other's.
                        let mut run: Vec<(EdgeId, bool)> =
                            trim_edges[t].values().map(|&te| (te, false)).collect();
                        if k == 1 {
                            run.reverse();
                            for r in &mut run {
                                r.1 = true;
                            }
                        }
                        for (te, rev) in run {
                            push_new(&mut specs, &mut hint, &out, te, rev);
                        }
                    }
                    i += 1;
                    continue;
                }
                let co = body.coedge(c);
                let li = roll
                    .links
                    .iter()
                    .position(|l| l.edge == co.edge)
                    .unwrap_or_else(|| panic!("a chain edge"));
                let k = (0..2)
                    .find(|&k| roll.links[li].faces[k] == f)
                    .unwrap_or_else(|| panic!("a chain edge's face is one of its sides"));
                let along = k == 0;
                let mut run = 0;
                while i + run < order.len() && is_chain(order[i + run]) {
                    run += 1;
                }
                let mut mine: Vec<usize> = (0..m)
                    .filter(|&j| pieces[j][k].is_some() && segments[j].face[k] == f)
                    .collect();
                if closed
                    && let Some(gap) = (0..mine.len()).find(|&x| {
                        let (p, q) = (mine[x], mine[(x + 1) % mine.len()]);
                        (p + 1) % m != q
                    })
                {
                    let len = mine.len();
                    mine.rotate_left((gap + 1) % len);
                }
                if mine.windows(2).any(|w| (w[0] + 1) % m != w[1]) {
                    return unsupported("whose contact leaves a face and comes back to it");
                }
                if !along {
                    mine.reverse();
                }
                for j in mine {
                    let e = pieces[j][k].unwrap_or_else(|| panic!("filtered above"));
                    push_new(&mut specs, &mut hint, &out, e, !along);
                }
                i += run;
            }
            out.add_loop(nf, specs);
        }
    }

    // The segments' faces: side 0's contact against its use there, then round by the arcs and
    // trims at its start, side 1's contact, and the arcs and trims at its end.
    let row = |v: f64| PCurve::Affine {
        origin: Vec2::new(0.0, v),
        dir: Vec2::new(1.0, 0.0),
    };
    let at_u = |u: f64| PCurve::Affine {
        origin: Vec2::new(u, 0.0),
        dir: Vec2::new(0.0, 1.0),
    };
    let mut blends = Vec::with_capacity(m);
    for (j, sg) in segments.iter().enumerate() {
        if !alive[j] {
            continue;
        }
        let pk = roll.links[sg.link].pk;
        let surface = Surface::Nurbs(Box::new(sg.surface.clone()));
        let bf = out.add_face(surface.clone(), true, names[pk].iter().cloned());
        let mut specs = Vec::new();
        let on_face = |e: EdgeId, reversed: bool, hint: Vec2, out: &Brep| -> CoedgeSpec {
            let x = out.edge(e);
            let t = if reversed { x.range.1 } else { x.range.0 };
            let p = x.curve.as_ref().unwrap_or_else(|| panic!("a curve")).eval(t);
            CoedgeSpec {
                edge: e,
                reversed,
                pcurve: PCurve::Projected {
                    seed: surface.project_near(p, hint),
                },
            }
        };
        if let Some(e) = pieces[j][0] {
            specs.push(CoedgeSpec {
                edge: e,
                reversed: true,
                pcurve: row(0.0),
            });
        }
        // The start side, v rising: the arc at `a` where kept, the start trim where it cuts.
        let side_run = |at_start: bool, specs: &mut Vec<CoedgeSpec>, out: &Brep| {
            let b_here = if at_start { sg.a } else { sg.b };
            let bi = bound_index(b_here).unwrap_or_else(|| panic!("a segment ends on a bound"));
            let mut parts: Vec<(f64, CoedgeSpec)> = Vec::new();
            for &(v0, v1, e) in &arcs[bi] {
                let mid = 0.5 * (v0 + v1);
                let edge_here = if at_start {
                    close(lower(j, mid), b_here)
                } else {
                    close(upper(j, mid), b_here)
                };
                if edge_here {
                    parts.push((
                        v0,
                        CoedgeSpec {
                            edge: e,
                            reversed: !at_start,
                            pcurve: at_u(b_here),
                        },
                    ));
                }
            }
            let ti = if at_start { 0 } else { 1 };
            if !closed {
                for (&OrderedF64(v0), &e) in &trim_edges[ti] {
                    let x = out.edge(e);
                    let mid = 0.5 * (x.range.0 + x.range.1);
                    let s = trims[ti].s_at(mid);
                    if s > sg.a + 1e-9 && s < sg.b - 1e-9 {
                        parts.push((v0, on_face(e, !at_start, Vec2::new(s, mid), out)));
                    }
                }
            }
            parts.sort_by(|x, y| x.0.total_cmp(&y.0));
            if !at_start {
                parts.reverse();
            }
            specs.extend(parts.into_iter().map(|x| x.1));
        };
        side_run(true, &mut specs, &out);
        if let Some(e) = pieces[j][1] {
            specs.push(CoedgeSpec {
                edge: e,
                reversed: false,
                pcurve: row(1.0),
            });
        }
        side_run(false, &mut specs, &out);
        out.add_loop(bf, specs);
        // Its outward side faces away from a ball inside the body, towards one outside it.
        let s = &out.face(bf).surface;
        let u = 0.5 * (lower(j, 0.5) + upper(j, 0.5));
        let p = s.eval(u, 0.5);
        let sec = roll.section(u, None)?;
        let away = (p - sec.centre) * if convex { 1.0 } else { -1.0 };
        if (-s.normal(u, 0.5)).dot(away) <= 0.0 {
            return Err(Error::Numerical(
                "a rolling-ball fillet's face came out facing into the body".into(),
            ));
        }
        blends.push(bf);
    }

    for sh in body.shell_ids() {
        let mut faces: Vec<FaceId> = body
            .shell(sh)
            .faces
            .iter()
            .map(|f| fmap[f.index()].unwrap_or_else(|| panic!("every face is copied")))
            .collect();
        if body.shell(sh).faces.contains(&picked[0].faces[0]) {
            faces.extend(&blends);
        }
        out.add_shell(faces);
    }
    for so in body.solid_ids() {
        out.add_solid_of(body.solid(so).shells.clone(), body.solid(so).outer);
    }
    out.pair_radial();
    let problems = check::check(&out);
    if !problems.is_empty() {
        return unsupported(&format!("whose result does not check: {}", problems[0]));
    }
    Ok(out)
}
