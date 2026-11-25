// yack: STEP B-rep topology into a kernel solid: vertices and edges shared across faces, each face's
// Adapted from springrep@a03e8a1 src/step/topology.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the raw builder moved to build/raw.rs (M2 T5); the file side stays here.
//! STEP B-rep topology into a kernel solid: vertices and edges shared across faces, each face's
//! loops given pcurves that close in its parameter plane, with the seams and poles the kernel's
//! representation needs (doc/design.md, "Representation").
//!
//! A solid is read in three passes: the file's topology into [`Raw`], [`crate::build::heal`] where the
//! file's geometry disagrees with itself by more than the kernel tolerance, then the arena.

use std::collections::HashMap;

use super::geometry::Geometry;
use yack_p21::Param;

use super::part21;
use crate::{
    Error, Result,
    build::{Raw, RawEdge, RawFace, Use, assemble, edge_range, heal, param_of},
    geom::{Circle, Curve, Line, Surface},
    math::{Frame, Point3},
    tol,
    topo::Brep,
};

fn bad<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::InvalidInput(format!("STEP #{id}: {what}")))
}

fn unsupported<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::Unsupported(format!("STEP #{id}: {what}")))
}

/// `raw` without the edges split in two after they were read, which no face uses any more.
fn without_split(mut raw: Raw, splits: &HashMap<usize, [usize; 2]>) -> Raw {
    if splits.is_empty() {
        return raw;
    }
    let mut index = vec![usize::MAX; raw.edges.len()];
    let mut kept = Vec::with_capacity(raw.edges.len());
    for (i, e) in raw.edges.drain(..).enumerate() {
        if !splits.contains_key(&i) {
            index[i] = kept.len();
            kept.push(e);
        }
    }
    raw.edges = kept;
    for f in &mut raw.faces {
        for (_, uses) in &mut f.bounds {
            for u in uses {
                u.edge = index[u.edge];
            }
        }
    }
    raw
}

/// `raw` with its coincident vertices made one, and then its coincident edges. A solid that
/// touches itself is written, as STEP's manifold solids require, with each side of the contact
/// given its own vertices and edges; two vertices of one valid solid are never that close, so
/// coincident ones are such a contact, and joining them gives it back.
fn join_contacts(mut raw: Raw) -> Raw {
    let n = raw.points.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn root(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| raw.points[a].x.total_cmp(&raw.points[b].x));
    let mut joined = false;
    for (k, &i) in order.iter().enumerate() {
        for &j in &order[k + 1..] {
            if raw.points[j].x - raw.points[i].x > tol::LINEAR {
                break;
            }
            if raw.points[i].dist(raw.points[j]) <= tol::LINEAR {
                let (a, b) = (root(&mut parent, i), root(&mut parent, j));
                if a != b {
                    parent[a.max(b)] = a.min(b);
                    joined = true;
                }
            }
        }
    }
    if joined {
        // The points kept, renumbered in order.
        let mut index = vec![usize::MAX; n];
        let (mut points, mut ids) = (Vec::new(), Vec::new());
        for (i, slot) in index.iter_mut().enumerate() {
            if root(&mut parent, i) == i {
                *slot = points.len();
                points.push(raw.points[i]);
                ids.push(raw.vertex_ids[i]);
            }
        }
        let at: Vec<usize> = (0..n).map(|i| index[root(&mut parent, i)]).collect();
        raw.points = points;
        raw.vertex_ids = ids;
        for e in &mut raw.edges {
            e.start = at[e.start];
            e.end = at[e.end];
        }
        for f in &mut raw.faces {
            for v in &mut f.points {
                *v = at[*v];
            }
        }
    }
    // Edges between the same vertices that lie along each other: each later one is the first,
    // its uses turned where it runs the other way.
    let mut ends: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (i, e) in raw.edges.iter().enumerate() {
        ends.entry((e.start.min(e.end), e.start.max(e.end)))
            .or_default()
            .push(i);
    }
    let m = raw.edges.len();
    let mut into: Vec<(usize, bool)> = (0..m).map(|i| (i, false)).collect();
    let mut any = false;
    for group in ends.values().filter(|g| g.len() > 1) {
        for (k, &i) in group.iter().enumerate() {
            if into[i].0 != i {
                continue;
            }
            let a = &raw.edges[i];
            let (pa, pz) = (raw.points[a.start], raw.points[a.end]);
            let Some((t0, t1)) = edge_range(&a.curve, pa, pz, a.start == a.end) else {
                continue;
            };
            let (q, d) = a.curve.d1(0.5 * (t0 + t1));
            for &j in &group[k + 1..] {
                let b = &raw.edges[j];
                if into[j].0 != j {
                    continue;
                }
                // On `b`'s curve within its range: two arcs of one circle share their ends too.
                let (pb, pe) = (raw.points[b.start], raw.points[b.end]);
                let Some((u0, u1)) = edge_range(&b.curve, pb, pe, b.start == b.end) else {
                    continue;
                };
                let mut t = param_of(&b.curve, q);
                if let Some(p) = b.curve.period() {
                    t = u0 + (t - u0).rem_euclid(p);
                }
                let (r, dr) = b.curve.d1(t);
                if (u0..=u1).contains(&t) && r.dist(q) <= tol::LINEAR {
                    into[j] = (i, d.dot(dr) < 0.0);
                    any = true;
                }
            }
        }
    }
    if !any {
        return raw;
    }
    let mut kept = vec![usize::MAX; m];
    let mut edges = Vec::new();
    for i in 0..m {
        if into[i].0 == i {
            kept[i] = edges.len();
            edges.push(raw.edges[i].clone());
        }
    }
    for (i, &(to, _)) in into.iter().enumerate() {
        if to != i {
            let faces = raw.edges[i].faces.clone();
            edges[kept[to]].faces.extend(faces);
        }
    }
    raw.edges = edges;
    for f in &mut raw.faces {
        for (_, uses) in &mut f.bounds {
            for u in uses {
                let (to, flip) = into[u.edge];
                u.edge = kept[to];
                u.reversed ^= flip;
            }
        }
    }
    raw
}

/// The vertex a use starts at.
/// A point's angle about `frame`'s axis.
fn about(frame: &Frame) -> impl Fn(Point3) -> f64 + '_ {
    move |p| {
        let l = frame.to_local(p);
        l.y.atan2(l.x)
    }
}

fn use_start(raw: &Raw, u: &Use) -> usize {
    let e = &raw.edges[u.edge];
    if u.reversed { e.end } else { e.start }
}

/// Reads one solid's topology, sharing vertices and edges between its faces.
struct Reader<'g, 'a> {
    geo: &'g mut Geometry<'a>,
    raw: Raw,
    vertices: HashMap<u64, usize>,
    /// Each edge's index and whether it runs against the STEP edge's direction.
    edges: HashMap<u64, (usize, bool)>,
    /// A `POLY_LOOP`'s corners, by their coordinates, and its sides, by their two corners.
    corners: HashMap<[u64; 3], usize>,
    sides: HashMap<(usize, usize), usize>,
    /// Edges split in two after they were read, to give a seam a vertex: each one's two halves.
    splits: HashMap<usize, [usize; 2]>,
}

/// One bound of a face as the file gives it.
enum Bound {
    /// Whether the file calls it the outer bound, and its uses in loop order.
    Edges(bool, Vec<Use>),
    /// A `VERTEX_LOOP`: the face closes on this one vertex, the apex of a cone or the pole of a
    /// sphere, or, anywhere else, holds it as a point where something touches the face.
    Vertex(usize),
}

impl<'g, 'a> Reader<'g, 'a> {
    fn params(&self, id: u64, kinds: &[&str]) -> Result<(&'a str, &'a [Param])> {
        let Some(e) = self.geo.file.get(id) else {
            return bad(id, "missing instance");
        };
        for r in e.records() {
            if kinds.contains(&r.name.as_str()) {
                return Ok((r.name.as_str(), r.params.as_slice()));
            }
        }
        unsupported(
            id,
            &format!("{} where {} was expected", part21::kind(e), kinds.join(" or ")),
        )
    }

    fn vertex(&mut self, id: u64) -> Result<usize> {
        if let Some(&v) = self.vertices.get(&id) {
            return Ok(v);
        }
        let (_, a) = self.params(id, &["VERTEX_POINT"])?;
        let p = self.geo.point(self.geo.reference(id, a.get(1))?)?;
        self.raw.points.push(p);
        self.raw.vertex_ids.push(id);
        let v = self.raw.points.len() - 1;
        self.vertices.insert(id, v);
        Ok(v)
    }

    fn edge(&mut self, id: u64) -> Result<(usize, bool)> {
        if let Some(&e) = self.edges.get(&id) {
            return Ok(e);
        }
        let (_, a) = self.params(id, &["EDGE_CURVE"])?;
        let vs = self.vertex(self.geo.reference(id, a.get(1))?)?;
        let ve = self.vertex(self.geo.reference(id, a.get(2))?)?;
        let curve = self.geo.curve(self.geo.reference(id, a.get(3))?)?;
        let flip = a.get(4).and_then(Param::boolean) == Some(false);
        let (start, end) = if flip { (ve, vs) } else { (vs, ve) };
        self.raw.edges.push(RawEdge {
            id,
            curve,
            start,
            end,
            faces: Vec::new(),
        });
        let e = (self.raw.edges.len() - 1, flip);
        self.edges.insert(id, e);
        Ok(e)
    }

    /// The vertex at a `POLY_LOOP` corner; corners at one place are one vertex.
    fn corner(&mut self, id: u64) -> Result<usize> {
        let p = self.geo.point(id)?;
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        if let Some(&v) = self.corners.get(&key) {
            return Ok(v);
        }
        self.raw.points.push(p);
        self.raw.vertex_ids.push(id);
        let v = self.raw.points.len() - 1;
        self.corners.insert(key, v);
        Ok(v)
    }

    /// The uses of a `POLY_LOOP`'s straight sides, each side shared with the face across it.
    fn poly_loop(&mut self, lp: u64, la: &[Param], forward: bool) -> Result<Vec<Use>> {
        let mut corners = Vec::new();
        for p in la.get(1).and_then(Param::list).unwrap_or(&[]) {
            corners.push(self.corner(self.geo.reference(lp, Some(p))?)?);
        }
        corners.dedup();
        while corners.len() > 1 && corners.first() == corners.last() {
            corners.pop();
        }
        if corners.len() < 3 {
            return bad(lp, "a polygon needs three distinct corners");
        }
        let mut uses = Vec::new();
        for k in 0..corners.len() {
            let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
            let edge = match self.sides.get(&(a.min(b), a.max(b))) {
                Some(&e) => e,
                None => {
                    let (pa, pb) = (self.raw.points[a], self.raw.points[b]);
                    let Some(dir) = (pb - pa).unit() else {
                        return bad(lp, "a polygon side has no length");
                    };
                    self.raw.edges.push(RawEdge {
                        id: lp,
                        curve: Curve::Line(Line { origin: pa, dir }),
                        start: a,
                        end: b,
                        faces: Vec::new(),
                    });
                    let e = self.raw.edges.len() - 1;
                    self.sides.insert((a.min(b), a.max(b)), e);
                    e
                }
            };
            let along = self.raw.edges[edge].start == a;
            uses.push(Use {
                edge,
                reversed: along != forward,
            });
        }
        if !forward {
            uses.reverse();
        }
        Ok(uses)
    }

    /// The edge uses of one `FACE_BOUND`, in the kernel's direction for the face.
    fn bound(&mut self, id: u64, flip_face: bool) -> Result<Bound> {
        let (kind, a) = self.params(id, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
        let outer = kind == "FACE_OUTER_BOUND";
        let lp = self.geo.reference(id, a.get(1))?;
        let orient = a.get(2).and_then(Param::boolean) != Some(false);
        let (lk, la) = self.params(lp, &["EDGE_LOOP", "VERTEX_LOOP", "POLY_LOOP"])?;
        match lk {
            "VERTEX_LOOP" => {
                return Ok(Bound::Vertex(
                    self.vertex(self.geo.reference(lp, la.get(1))?)?,
                ));
            }
            "POLY_LOOP" => {
                return Ok(Bound::Edges(
                    outer,
                    self.poly_loop(lp, la, orient != flip_face)?,
                ));
            }
            _ => {}
        }
        let mut uses = Vec::new();
        for oe in la.get(1).and_then(Param::list).unwrap_or(&[]) {
            let oe = self.geo.reference(lp, Some(oe))?;
            let (_, p) = self.params(oe, &["ORIENTED_EDGE"])?;
            let (edge, flip) = self.edge(self.geo.reference(oe, p.get(3))?)?;
            let along = p.get(4).and_then(Param::boolean) != Some(false);
            uses.push(Use {
                edge,
                reversed: ((along == orient) ^ flip_face) == flip,
            });
        }
        if !orient ^ flip_face {
            uses.reverse();
        }
        if uses.is_empty() {
            return bad(lp, "empty loop");
        }
        Ok(Bound::Edges(outer, self.expand(&uses)))
    }

    /// A seam from `from` to `apex` along `surface`, for a face that closes on its apex: a
    /// generator of a cone, a meridian of a sphere.
    fn seam(&mut self, id: u64, surface: &Surface, from: usize, apex: usize) -> Result<usize> {
        let (p, q) = (self.raw.points[from], self.raw.points[apex]);
        let curve = match surface {
            Surface::Cone(_) => match (q - p).unit() {
                Some(dir) => Curve::Line(Line { origin: p, dir }),
                None => return bad(id, "the vertex loop sits on its face's other bound"),
            },
            Surface::Sphere(s) => {
                let c = s.frame.origin;
                let x = p - c;
                let y = (q - c) - x.unit_or_zero() * (q - c).dot(x.unit_or_zero());
                match Frame::new(c, x.cross(y), x) {
                    Ok(frame) => Curve::Circle(Circle {
                        frame,
                        radius: s.radius,
                    }),
                    Err(_) => return bad(id, "the vertex loop is not at a pole of its sphere"),
                }
            }
            other => {
                return unsupported(id, &format!("VERTEX_LOOP bound on a {} face", other.kind()));
            }
        };
        self.raw.edges.push(RawEdge {
            id,
            curve,
            start: from,
            end: apex,
            faces: Vec::new(),
        });
        Ok(self.raw.edges.len() - 1)
    }

    /// `uses` with each use of a split edge as the uses of its halves.
    fn expand(&self, uses: &[Use]) -> Vec<Use> {
        let mut out = Vec::with_capacity(uses.len());
        for &u in uses {
            match self.splits.get(&u.edge) {
                Some(&[a, b]) => {
                    let halves = [Use { edge: a, ..u }, Use { edge: b, ..u }];
                    let ordered = if u.reversed {
                        [halves[1], halves[0]]
                    } else {
                        halves
                    };
                    out.extend(self.expand(&ordered));
                }
                None => out.push(u),
            }
        }
        out
    }

    /// Splits edge `e` at its parameter `t` into two edges meeting at a new vertex there, and
    /// gives every face read so far, and `bounds`, the halves in its place. Returns the vertex.
    fn split_edge(&mut self, e: usize, t: f64, bounds: &mut [(bool, Vec<Use>)]) -> usize {
        let edge = self.raw.edges[e].clone();
        let p = edge.curve.eval(t);
        self.raw.points.push(p);
        self.raw.vertex_ids.push(edge.id);
        let v = self.raw.points.len() - 1;
        self.raw.edges.push(RawEdge {
            end: v,
            ..edge.clone()
        });
        self.raw.edges.push(RawEdge { start: v, ..edge });
        let n = self.raw.edges.len();
        self.splits.insert(e, [n - 2, n - 1]);
        for f in 0..self.raw.faces.len() {
            for b in 0..self.raw.faces[f].bounds.len() {
                let uses = self.expand(&self.raw.faces[f].bounds[b].1);
                self.raw.faces[f].bounds[b].1 = uses;
            }
        }
        for (_, uses) in bounds.iter_mut() {
            *uses = self.expand(uses);
        }
        v
    }

    /// Where `uses` cross the angle `at`, as an edge and its parameter, `angle` giving a point's.
    fn crossing(
        &self,
        angle: &dyn Fn(Point3) -> f64,
        uses: &[Use],
        at: f64,
    ) -> Option<(usize, f64)> {
        let off = |p: Point3| {
            (angle(p) - at + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI
        };
        for u in uses {
            let e = &self.raw.edges[u.edge];
            let (ps, pe) = (self.raw.points[e.start], self.raw.points[e.end]);
            let (t0, t1) = edge_range(&e.curve, ps, pe, e.start == e.end)?;
            let n = 64;
            for k in 0..n {
                let (mut lo, mut hi) = (
                    t0 + (t1 - t0) * k as f64 / n as f64,
                    t0 + (t1 - t0) * (k + 1) as f64 / n as f64,
                );
                let (dl, dh) = (off(e.curve.eval(lo)), off(e.curve.eval(hi)));
                // A sign change away from the wrap at half a turn.
                if dl.signum() == dh.signum() || dl.abs().max(dh.abs()) > 1.0 {
                    continue;
                }
                for _ in 0..100 {
                    let mid = 0.5 * (lo + hi);
                    if off(e.curve.eval(mid)).signum() == dl.signum() {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                return Some((u.edge, 0.5 * (lo + hi)));
            }
        }
        None
    }

    /// The angle `uses` sweep about `frame`'s axis, from samples along each edge in its use's
    /// direction; `None` where an edge's span between its vertices cannot be found, or where the
    /// loop passes over the axis, as a seam through a pole does, and its angle jumps.
    fn swept(&self, frame: &Frame, uses: &[Use]) -> Option<f64> {
        self.swept_by(&about(frame), uses)
    }

    /// The angle `uses` sweep, `angle` giving a point's: [`Reader::swept`] about any measure of it.
    fn swept_by(&self, angle: &dyn Fn(Point3) -> f64, uses: &[Use]) -> Option<f64> {
        let mut total = 0.0;
        for u in uses {
            let e = &self.raw.edges[u.edge];
            let (ps, pe) = (self.raw.points[e.start], self.raw.points[e.end]);
            let (t0, t1) = edge_range(&e.curve, ps, pe, e.start == e.end)?;
            let n = 64;
            let mut ts: Vec<f64> = (0..=n)
                .map(|k| t0 + (t1 - t0) * k as f64 / n as f64)
                .collect();
            if u.reversed {
                ts.reverse();
            }
            for w in ts.windows(2) {
                let d = angle(e.curve.eval(w[1])) - angle(e.curve.eval(w[0]));
                let d = (d + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                if d.abs() > 1.0 {
                    return None;
                }
                total += d;
            }
        }
        Some(total)
    }

    /// A face of revolution (cylinder, cone, sphere, torus) with two loops that each go once round
    /// its axis, the ways opposite, and no seam: those two become one loop, the first, a seam
    /// along the surface to a vertex of the second at the same angle, the second, and the seam
    /// back. Its pcurves then close in the parameter plane, as the kernel's faces do. Other loops,
    /// holes through the band, stay. `reversed` is the face's orientation against its surface's.
    /// `None` when the face is not such a band, or no vertex of the second lies on the seam's line.
    fn seamed_band(
        &mut self,
        id: u64,
        surface: &Surface,
        reversed: bool,
        bounds: &mut Vec<(bool, Vec<Use>)>,
    ) -> Option<()> {
        let frame = match surface {
            Surface::Cylinder(c) => c.frame,
            Surface::Cone(c) => c.frame,
            Surface::Sphere(s) => s.frame,
            Surface::Torus(t) => t.frame,
            _ => return None,
        };
        let full = |w: f64| (w.abs() - std::f64::consts::TAU).abs() < 0.1;
        let mut winding = Vec::new();
        for (k, (_, uses)) in bounds.iter().enumerate() {
            let w = self.swept(&frame, uses)?;
            if full(w) {
                winding.push((k, w));
            }
        }
        let &[(ka, wa), (kb, wb)] = winding.as_slice() else {
            return None;
        };
        if wa * wb >= 0.0 {
            return None;
        }
        let (a, b) = (&bounds[ka].1, &bounds[kb].1);
        // A vertex of each loop on one line of the surface: a cylinder's lines run along its
        // axis, a cone's through its apex.
        let on_line = |p: Point3, q: Point3| {
            let (lp, lq) = (frame.to_local(p), frame.to_local(q));
            let radial = |l: Point3| (l.x * l.x + l.y * l.y).sqrt();
            let (rp, rq) = (radial(lp), radial(lq));
            if rp <= tol::LINEAR || rq <= tol::LINEAR {
                return false;
            }
            let (ux, uy) = (lp.x / rp, lp.y / rp);
            (lq.x - ux * rq).hypot(lq.y - uy * rq) <= tol::LINEAR
        };
        let aligned = (0..a.len())
            .flat_map(|i| (0..b.len()).map(move |j| (i, j)))
            .find(|&(i, j)| {
                let at = |u: &Use| self.raw.points[use_start(&self.raw, u)];
                on_line(at(&a[i]), at(&b[j]))
            });
        let (a, b, ia, ib) = match aligned {
            Some((i, j)) => (a.clone(), b.clone(), i, j),
            None => {
                // No vertex of the second loop at the first's angle: split the second's edge
                // there, in every face that uses it.
                let va = use_start(&self.raw, &a[0]);
                let la = frame.to_local(self.raw.points[va]);
                let (e, t) = self.crossing(&about(&frame), b, la.y.atan2(la.x))?;
                let v = self.split_edge(e, t, bounds);
                let (a, b) = (bounds[ka].1.clone(), bounds[kb].1.clone());
                let ia = a.iter().position(|u| use_start(&self.raw, u) == va)?;
                let ib = b.iter().position(|u| use_start(&self.raw, u) == v)?;
                (a, b, ia, ib)
            }
        };
        let (va, vb) = (use_start(&self.raw, &a[ia]), use_start(&self.raw, &b[ib]));
        let (pa, pb) = (self.raw.points[va], self.raw.points[vb]);
        // The seam's line at the vertices' angle about the axis.
        let la = frame.to_local(pa);
        let radial = (frame.x * la.x + frame.y * la.y).unit()?;
        let meridian = |centre: Point3, radius: f64, toward_v: bool| {
            // A circle whose parameter is the surface's v there, or runs against it.
            let normal = radial.cross(frame.z) * if toward_v { 1.0 } else { -1.0 };
            Frame::new(centre, normal, radial)
                .ok()
                .map(|frame| Curve::Circle(Circle { frame, radius }))
        };
        let curve = match surface {
            Surface::Cylinder(_) | Surface::Cone(_) => Curve::Line(Line {
                origin: pa,
                dir: (pb - pa).unit()?,
            }),
            // A latitude band: the meridian between the two loops, which v crosses one way.
            Surface::Sphere(s) => {
                let v = |p: Point3| {
                    frame
                        .to_local(p)
                        .z
                        .atan2(frame.to_local(p).x.hypot(frame.to_local(p).y))
                };
                meridian(s.frame.origin, s.radius, v(pb) > v(pa))?
            }
            // A ring of the tube: the first loop's material lies on its left, seen from outside,
            // which is the way v grows from it when it runs the way u grows on a face along its
            // surface's normal.
            Surface::Torus(t) => {
                let toward_v = (wa > 0.0) != reversed;
                meridian(frame.origin + radial * t.major, t.minor, toward_v)?
            }
            _ => return None,
        };
        self.raw.edges.push(RawEdge {
            id,
            curve,
            start: va,
            end: vb,
            faces: Vec::new(),
        });
        let seam = self.raw.edges.len() - 1;
        let mut out: Vec<Use> = a[ia..].iter().chain(&a[..ia]).copied().collect();
        out.push(Use {
            edge: seam,
            reversed: false,
        });
        out.extend(b[ib..].iter().chain(&b[..ib]).copied());
        out.push(Use {
            edge: seam,
            reversed: true,
        });
        bounds.remove(ka.max(kb));
        bounds.remove(ka.min(kb));
        bounds.insert(0, (true, out));
        Some(())
    }

    /// A spline face closed round one of its parameters with two loops that each go once round
    /// it, the ways opposite, and no seam: [`Reader::seamed_band`] with that parameter for the
    /// angle, and the surface's own iso curve for the seam. `None` when the face is not such a
    /// band.
    fn seamed_spline_band(
        &mut self,
        id: u64,
        surface: &Surface,
        bounds: &mut Vec<(bool, Vec<Use>)>,
    ) -> Option<()> {
        let Surface::Nurbs(n) = surface else {
            return None;
        };
        let (in_v, period) = match (surface.u_period(), surface.v_period()) {
            (None, Some(p)) => (true, p),
            (Some(p), None) => (false, p),
            _ => return None,
        };
        const TAU: f64 = std::f64::consts::TAU;
        let angle = |p: Point3| {
            let uv = surface.project(p);
            (if in_v { uv.y } else { uv.x }) / period * TAU
        };
        let full = |w: f64| (w.abs() - TAU).abs() < 0.1;
        let mut winding = Vec::new();
        for (k, (_, uses)) in bounds.iter().enumerate() {
            let w = self.swept_by(&angle, uses)?;
            if full(w) {
                winding.push((k, w));
            }
        }
        let &[(ka, wa), (kb, wb)] = winding.as_slice() else {
            return None;
        };
        if wa * wb >= 0.0 {
            return None;
        }
        // The seam runs where it is farthest round from every vertex of the two loops, the middle
        // of the widest gap between their angles, so that splitting a loop for it there leaves no
        // sliver beside a vertex a file placed a little off the line.
        let mut angles: Vec<f64> = [ka, kb]
            .iter()
            .flat_map(|&k| bounds[k].1.iter())
            .map(|u| angle(self.raw.points[use_start(&self.raw, u)]).rem_euclid(TAU))
            .collect();
        angles.sort_by(f64::total_cmp);
        let at = (0..angles.len())
            .map(|i| {
                let (lo, hi) = (angles[i], angles[(i + 1) % angles.len()]);
                let gap = (hi - lo).rem_euclid(TAU);
                let gap = if gap == 0.0 { TAU } else { gap };
                (gap, lo + 0.5 * gap)
            })
            .max_by(|x, y| x.0.total_cmp(&y.0))?
            .1;
        let seam = Curve::Nurbs(Box::new(n.iso_curve(!in_v, at / TAU * period).ok()?));
        let on_seam = |raw: &Raw, u: &Use| {
            let q = raw.points[use_start(raw, u)];
            seam.eval(param_of(&seam, q)).dist(q) <= tol::LINEAR
        };
        // A vertex of each loop on the seam, the loop's edge split there where it has none.
        let mut ends = [0usize; 2];
        for (slot, k) in [ka, kb].into_iter().enumerate() {
            ends[slot] = match bounds[k].1.iter().find(|u| on_seam(&self.raw, u)) {
                Some(u) => use_start(&self.raw, u),
                None => {
                    let uses = bounds[k].1.clone();
                    let (e, t) = self.crossing(&angle, &uses, at)?;
                    self.split_edge(e, t, bounds)
                }
            };
        }
        let [va, vb] = ends;
        let (a, b) = (bounds[ka].1.clone(), bounds[kb].1.clone());
        let ia = a.iter().position(|u| use_start(&self.raw, u) == va)?;
        let ib = b.iter().position(|u| use_start(&self.raw, u) == vb)?;
        // An edge runs the way its curve does: from whichever end comes first along the iso.
        let back = param_of(&seam, self.raw.points[va]) > param_of(&seam, self.raw.points[vb]);
        let (start, end) = if back { (vb, va) } else { (va, vb) };
        self.raw.edges.push(RawEdge {
            id,
            curve: seam,
            start,
            end,
            faces: Vec::new(),
        });
        let edge = self.raw.edges.len() - 1;
        let mut out: Vec<Use> = a[ia..].iter().chain(&a[..ia]).copied().collect();
        out.push(Use {
            edge,
            reversed: back,
        });
        out.extend(b[ib..].iter().chain(&b[..ib]).copied());
        out.push(Use {
            edge,
            reversed: !back,
        });
        bounds.remove(ka.max(kb));
        bounds.remove(ka.min(kb));
        bounds.insert(0, (true, out));
        Some(())
    }

    /// A sphere or cone face with one loop that goes once round its axis and no seam: a cap over
    /// the pole on the loop's left seen from outside, or over the cone's apex, the only side that
    /// closes. As a vertex loop there would, it gives the loop a seam up to the pole and back. A
    /// loop through a pole of a sphere's frame, such as a hemisphere's great circle, is a latitude
    /// of the frame about its circle's normal, which then becomes the surface's frame. Other loops,
    /// holes through the cap, stay. False when the face is not such a cap.
    fn capped(
        &mut self,
        id: u64,
        surface: &mut Surface,
        reversed: bool,
        bounds: &mut [(bool, Vec<Use>)],
    ) -> Result<bool> {
        let (base, sphere) = match &*surface {
            Surface::Sphere(s) => (s.frame, true),
            Surface::Cone(c) if c.half_angle.abs() > tol::ANGULAR => (c.frame, false),
            _ => return Ok(false),
        };
        let mut found = None;
        for (k, (_, uses)) in bounds.iter().enumerate() {
            // A loop through the apex, or a pole, already closes on it by its own seam.
            let at_pole = uses.iter().any(|u| {
                let l = base.to_local(self.raw.points[use_start(&self.raw, u)]);
                l.x.hypot(l.y) <= tol::LINEAR
            });
            let frame = match uses.as_slice() {
                _ if !at_pole => base,
                [u] if sphere => match &self.raw.edges[u.edge].curve {
                    Curve::Circle(c) => match Frame::new(base.origin, c.frame.z, c.frame.x) {
                        Ok(f) => f,
                        Err(_) => continue,
                    },
                    _ => continue,
                },
                _ => continue,
            };
            let Some(w) = self.swept(&frame, uses) else {
                continue;
            };
            if (w.abs() - std::f64::consts::TAU).abs() < 0.1 {
                if found.is_some() {
                    return Ok(false);
                }
                found = Some((k, frame, w));
            }
        }
        let Some((k, frame, w)) = found else {
            return Ok(false);
        };
        let uses = &bounds[k].1;
        let pole = match surface {
            Surface::Sphere(s) => {
                s.frame = frame;
                // Along the outward normal, a loop running the way u grows has +z on its left.
                let up = (w > 0.0) != reversed;
                frame.origin + frame.z * if up { s.radius } else { -s.radius }
            }
            Surface::Cone(c) => c.frame.origin - c.frame.z * (c.radius / c.half_angle.tan()),
            _ => return Ok(false),
        };
        self.raw.points.push(pole);
        self.raw.vertex_ids.push(id);
        let apex = self.raw.points.len() - 1;
        let from = use_start(&self.raw, &uses[0]);
        let seam = self.seam(id, surface, from, apex)?;
        for reversed in [false, true] {
            bounds[k].1.push(Use {
                edge: seam,
                reversed,
            });
        }
        Ok(true)
    }

    fn face(&mut self, id: u64, flip_face: bool) -> Result<usize> {
        let (_, a) = self.params(id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
        let name = a.first().and_then(Param::string).unwrap_or("").to_string();
        let mut surface = self.geo.surface(self.geo.reference(id, a.get(2))?)?;
        let same_sense = a.get(3).and_then(Param::boolean) != Some(false);
        let mut bounds = Vec::new();
        let mut apexes = Vec::new();
        for bd in a.get(1).and_then(Param::list).unwrap_or(&[]) {
            match self.bound(self.geo.reference(id, Some(bd))?, flip_face)? {
                Bound::Edges(outer, uses) => bounds.push((outer, uses)),
                Bound::Vertex(v) => apexes.push(v),
            }
        }
        let (mut apexes, points): (Vec<usize>, Vec<usize>) = apexes
            .into_iter()
            .partition(|&v| closes_at(&surface, self.raw.points[v]));
        for &v in &points {
            let p = self.raw.points[v];
            let uv = surface.project(p);
            if surface.eval(uv.x, uv.y).dist(p) > tol::LINEAR {
                return bad(id, "a VERTEX_LOOP off its face's surface");
            }
        }
        // A whole sphere, bounded only by a vertex loop at one pole: the kernel's seam, a
        // meridian from the other pole to it, used both ways.
        if let ([apex], 0, Surface::Sphere(sp)) = (apexes.as_slice(), bounds.len(), &surface) {
            let (c, p) = (sp.frame.origin, self.raw.points[*apex]);
            let other = c * 2.0 - p;
            self.raw.points.push(other);
            self.raw.vertex_ids.push(id);
            let from = self.raw.points.len() - 1;
            let x = (other - c).unit_or_zero();
            let Ok(frame) = Frame::new(c, x.cross(sp.frame.x), x) else {
                return bad(id, "the vertex loop is not at a pole of its sphere");
            };
            self.raw.edges.push(RawEdge {
                id,
                curve: Curve::Circle(Circle {
                    frame,
                    radius: sp.radius,
                }),
                start: from,
                end: *apex,
                faces: Vec::new(),
            });
            let seam = self.raw.edges.len() - 1;
            bounds.push((
                true,
                vec![
                    Use {
                        edge: seam,
                        reversed: false,
                    },
                    Use {
                        edge: seam,
                        reversed: true,
                    },
                ],
            ));
            apexes.clear();
        }
        // A face that closes on a vertex is, in the kernel, its other bound followed by a seam
        // up to the vertex and back, the pole between the seam's two uses.
        match (apexes.as_slice(), bounds.len()) {
            ([], _) => {}
            (&[apex], 1) => {
                let uses = &bounds[0].1;
                let first = uses[0];
                let e = &self.raw.edges[first.edge];
                let from = if first.reversed { e.end } else { e.start };
                let seam = self.seam(id, &surface, from, apex)?;
                bounds[0].1.push(Use {
                    edge: seam,
                    reversed: false,
                });
                bounds[0].1.push(Use {
                    edge: seam,
                    reversed: true,
                });
            }
            _ => {
                return unsupported(
                    id,
                    "a vertex loop other than one face's apex beside one other bound",
                );
            }
        }
        if bounds.is_empty() {
            return unsupported(id, "face without bounds");
        }
        let reversed = !same_sense ^ flip_face;
        if self
            .seamed_band(id, &surface, reversed, &mut bounds)
            .is_none()
            && self.seamed_spline_band(id, &surface, &mut bounds).is_none()
            && apexes.is_empty()
        {
            self.capped(id, &mut surface, reversed, &mut bounds)?;
        }
        let f = self.raw.faces.len();
        for (_, uses) in &bounds {
            for u in uses {
                self.raw.edges[u.edge].faces.push(f);
            }
        }
        self.raw.faces.push(RawFace {
            id,
            name,
            surface,
            reversed: !same_sense ^ flip_face,
            bounds,
            points,
        });
        Ok(f)
    }

    fn solid(&mut self, id: u64) -> Result<()> {
        let Some(e) = self.geo.file.get(id) else {
            return bad(id, "missing instance");
        };
        let first = &e.records()[0];
        let (kind, a) = (&first.name, &first.params);
        let mut shells: Vec<u64> = Vec::new();
        match kind.as_str() {
            "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => {
                shells.push(self.geo.reference(id, a.get(1))?)
            }
            "BREP_WITH_VOIDS" => {
                shells.push(self.geo.reference(id, a.get(1))?);
                for v in a.get(2).and_then(Param::list).unwrap_or(&[]) {
                    shells.push(self.geo.reference(id, Some(v))?);
                }
            }
            other => return unsupported(id, &format!("solid {other}")),
        }
        for mut sh in shells {
            let mut flip = false;
            if let Some(p) = self
                .geo
                .file
                .get(sh)
                .and_then(|e| e.part("ORIENTED_CLOSED_SHELL"))
            {
                flip = p.get(3).and_then(Param::boolean) == Some(false);
                sh = self.geo.reference(sh, p.get(2))?;
            }
            let (_, p) = self.params(sh, &["CLOSED_SHELL"])?;
            let mut faces = Vec::new();
            for f in p.get(1).and_then(Param::list).unwrap_or(&[]) {
                faces.push(self.face(self.geo.reference(sh, Some(f))?, flip)?);
            }
            self.raw.shells.push(faces);
        }
        Ok(())
    }
}

/// Whether a face on `s` bounded by a `VERTEX_LOOP` at `p` closes there: at a cone's apex or a
/// sphere's pole, where every parameter line meets.
fn closes_at(s: &Surface, p: Point3) -> bool {
    let near = |q: Point3| q.dist(p) <= tol::LINEAR;
    match s {
        Surface::Cone(c) => near(c.frame.origin - c.frame.z * (c.radius / c.half_angle.tan())),
        Surface::Sphere(sp) => {
            let (o, z) = (sp.frame.origin, sp.frame.z);
            near(o + z * sp.radius) || near(o - z * sp.radius)
        }
        _ => false,
    }
}

/// A solid read, its faces' STEP ids and names in order, and how far healing moved its geometry.
pub(super) type ReadSolid = (Brep, Vec<(u64, String)>, Option<heal::Healed>);

/// Builds the solid of a `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS`, healed where its geometry
/// disagrees with itself, and checked. Returns the solid, its faces' STEP ids and names in order,
/// and how far healing moved its geometry.
pub(super) fn solid(geo: &mut Geometry, id: u64) -> Result<ReadSolid> {
    if geo
        .file
        .get(id)
        .is_some_and(|e| e.part("TESSELLATED_SOLID").is_some())
    {
        return tessellated(geo, id).map(|b| {
            let names = vec![(0, String::new()); b.face_count()];
            (b, names, None)
        });
    }
    let mut rd = Reader {
        geo,
        raw: Raw::default(),
        vertices: HashMap::new(),
        edges: HashMap::new(),
        corners: HashMap::new(),
        sides: HashMap::new(),
        splits: HashMap::new(),
    };
    rd.solid(id)?;
    let file = join_contacts(without_split(rd.raw, &rd.splits));
    let mut raw = file.clone();
    let names = || file.faces.iter().map(|f| (f.id, f.name.clone())).collect();
    let within = rd.geo.uncertainty().filter(|&u| u > tol::LINEAR);
    if let Some(b) = heal::as_written(&file, within, |r, t| assemble(r, &t.ranges, Some(t)).ok()) {
        return Ok((b, names(), None));
    }
    let exact = heal::heal(&mut raw)
        .and_then(|(ranges, healed)| assemble(&raw, &ranges, None).map(|b| (b, healed)));
    // Healing past the file's stated uncertainty changed the shape: keep the file's geometry then.
    let why = match exact {
        Ok((b, healed)) if within.is_none_or(|w| healed.as_ref().is_none_or(|h| h.moved <= w)) => {
            return Ok((b, names(), healed));
        }
        other => other,
    };
    // Geometry that cannot be made to meet within the kernel tolerance is kept as the file has
    // it, its edges and vertices given the tolerances it does meet within, whatever the file
    // states.
    let tolerant = |raw: &Raw| {
        let mut raw = raw.clone();
        heal::tolerate(&mut raw, within)
            .map_err(|t| format!("kept as the file has it, {t}"))
            .and_then(|t| {
                assemble(&raw, &t.ranges, Some(&t))
                    .map_err(|e| format!("kept as the file has it, with local tolerances: {e}"))
            })
    };
    match (tolerant(&file), why) {
        (Ok(b), _) => Ok((b, names(), None)),
        (Err(_), Ok((b, healed))) => Ok((b, names(), healed)),
        (Err(t), Err(e)) => {
            // Healed where it can be, and held by tolerances where it cannot.
            let mut loose = file.clone();
            let healed = heal::heal_loosely(&mut loose);
            let l = match (tolerant(&loose), healed.as_ref(), within) {
                (Ok(_), Some(h), Some(within)) if h.moved > within => format!(
                    "healing would move {} {:.1e} mm, more than the file's {within:.1e} mm",
                    h.what, h.moved
                ),
                (Ok(b), _, _) => return Ok((b, names(), healed)),
                (Err(l), _, _) => l,
            };
            Err(Error::InvalidInput(format!(
                "{e}; {t}; healed where it can be and {l}"
            )))
        }
    }
}

/// An AP242 `TESSELLATED_SOLID`: each triangle of its `TRIANGULATED_FACE`s and
/// `COMPLEX_TRIANGULATED_FACE`s a plane face, the triangles sharing the edges between their
/// corners. The triangles run counter-clockwise seen from outside, as the schema has them.
fn tessellated(geo: &mut Geometry, id: u64) -> Result<Brep> {
    let unit = geo.length_unit();
    let Some(solid) = geo
        .file
        .get(id)
        .and_then(|e| e.part("TESSELLATED_SOLID"))
        .map(<[Param]>::to_vec)
    else {
        return bad(id, "expected a tessellated solid");
    };
    // Corners index their own face's list; one list's point is one vertex wherever it is used.
    let mut points: Vec<Point3> = Vec::new();
    let mut index: HashMap<(u64, usize), usize> = HashMap::new();
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    for item in solid.get(1).and_then(Param::list).unwrap_or(&[]) {
        let fid = geo.reference(id, Some(item))?;
        let entity = geo.file.get(fid);
        let (face, complex) = match (
            entity.and_then(|e| e.part("TRIANGULATED_FACE")),
            entity.and_then(|e| e.part("COMPLEX_TRIANGULATED_FACE")),
        ) {
            (Some(f), _) => (f.to_vec(), false),
            (None, Some(f)) => (f.to_vec(), true),
            _ => return unsupported(fid, "a tessellated face other than a triangulated one"),
        };
        let list = geo.reference(fid, face.get(1))?;
        let Some(coords) = geo
            .file
            .get(list)
            .and_then(|e| e.part("COORDINATES_LIST"))
            .map(<[Param]>::to_vec)
        else {
            return bad(list, "expected a coordinates list");
        };
        let mut xyz = Vec::new();
        for c in coords.get(2).and_then(Param::list).unwrap_or(&[]) {
            let v = geo.numbers(list, Some(c))?;
            if v.len() != 3 || v.iter().any(|x| !x.is_finite()) {
                return bad(list, "expected three finite coordinates");
            }
            xyz.push(Point3::new(v[0], v[1], v[2]) * unit);
        }
        // Triangle corners index the face's own points, which index the coordinates list.
        let own: Vec<usize> = face
            .get(5)
            .and_then(Param::list)
            .unwrap_or(&[])
            .iter()
            .map(|p| part21::integral(p).map_or(0, |i| i as usize))
            .collect();
        let corner = |k: i64| -> Option<usize> {
            let k = usize::try_from(k).ok()?.checked_sub(1)?;
            let i = if own.is_empty() {
                k
            } else {
                own.get(k)?.checked_sub(1)?
            };
            (i < xyz.len()).then_some(i)
        };
        let lists = |at: usize| -> Vec<Vec<i64>> {
            face.get(at)
                .and_then(Param::list)
                .unwrap_or(&[])
                .iter()
                .map(|l| {
                    l.list()
                        .unwrap_or(&[])
                        .iter()
                        .filter_map(part21::integral)
                        .collect()
                })
                .collect()
        };
        let tris = if complex {
            strips_and_fans(&lists(6), &lists(7))
        } else {
            lists(6)
        };
        for ks in tris {
            let idx: Option<Vec<usize>> = ks.iter().map(|&k| corner(k)).collect();
            let Some(idx) = idx.filter(|v| v.len() == 3) else {
                return bad(fid, "a triangle needs three corners in its list");
            };
            let mut corners = [0; 3];
            for (corner, &i) in corners.iter_mut().zip(&idx) {
                *corner = *index.entry((list, i)).or_insert_with(|| {
                    points.push(xyz[i]);
                    points.len() - 1
                });
            }
            triangles.push(corners);
        }
    }
    if triangles.is_empty() {
        return bad(id, "a tessellated solid with no triangles");
    }
    crate::build::from_indexed(&points, &triangles)
}

/// The triangles of a `COMPLEX_TRIANGULATED_FACE`'s strips and fans, each keeping the face's
/// turn: in a strip each point after the first two makes a triangle with the two before it, every
/// second one taken the other way round; in a fan each point after the second makes one with the
/// point before it and the first. A triangle with a corner twice, which joins two strips, has no
/// area and is left out.
fn strips_and_fans(strips: &[Vec<i64>], fans: &[Vec<i64>]) -> Vec<Vec<i64>> {
    let strip = strips.iter().flat_map(|s| {
        s.windows(3).enumerate().map(|(i, w)| match i % 2 {
            0 => vec![w[0], w[1], w[2]],
            _ => vec![w[1], w[0], w[2]],
        })
    });
    let fan = fans
        .iter()
        .flat_map(|f| f.windows(2).skip(1).map(|w| vec![f[0], w[0], w[1]]));
    strip
        .chain(fan)
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        .collect()
}

/// Every face of the shells in `file`, each with its own vertices and edges, sewn into one solid:
/// vertices within `tolerance` of each other become one, edges that then share their ends and lie
/// together become one, faces are turned so each shared edge runs opposite ways in its two faces,
/// and the solid is healed onto its faces' surfaces as a read one is.
pub(super) fn sew(geo: &mut Geometry, tolerance: f64) -> Result<Brep> {
    let shells: Vec<u64> = part21::entities(geo.file)
        .filter(|(_, e)| matches!(part21::kind(e), "OPEN_SHELL" | "CLOSED_SHELL"))
        .map(|(id, _)| id)
        .collect();
    let mut rd = Reader {
        geo,
        raw: Raw::default(),
        vertices: HashMap::new(),
        edges: HashMap::new(),
        corners: HashMap::new(),
        sides: HashMap::new(),
        splits: HashMap::new(),
    };
    let mut faces = Vec::new();
    for sh in shells {
        let (_, p) = rd.params(sh, &["OPEN_SHELL", "CLOSED_SHELL"])?;
        for f in p.get(1).and_then(Param::list).unwrap_or(&[]) {
            faces.push(rd.face(rd.geo.reference(sh, Some(f))?, false)?);
        }
    }
    if faces.is_empty() {
        return Err(Error::InvalidInput(
            "STEP: the file holds no faces to sew".into(),
        ));
    }
    let mut raw = without_split(rd.raw, &rd.splits);
    // Points within the tolerance of each other are one, at their mean.
    let n = raw.points.len();
    let mut rep: Vec<usize> = (0..n).collect();
    fn root(rep: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while rep[r] != r {
            r = rep[r];
        }
        rep[i] = r;
        r
    }
    for i in 0..n {
        for j in i + 1..n {
            if raw.points[i].dist(raw.points[j]) <= tolerance {
                let (a, b) = (root(&mut rep, i), root(&mut rep, j));
                if a != b {
                    rep[b] = a;
                }
            }
        }
    }
    let mut sum: HashMap<usize, (Point3, f64)> = HashMap::new();
    for i in 0..n {
        let r = root(&mut rep, i);
        let e = sum.entry(r).or_insert((Point3::ZERO, 0.0));
        e.0 += raw.points[i];
        e.1 += 1.0;
    }
    for i in 0..n {
        let r = root(&mut rep, i);
        let (s, k) = sum[&r];
        raw.points[i] = s * (1.0 / k);
    }
    // Edges that now run between one pair of points, and lie together along it, are one: the
    // first stands for the rest, and their uses are turned to its direction.
    let ends = |e: &RawEdge, rep: &mut [usize]| (root(rep, e.start), root(rep, e.end));
    let mut keep: Vec<usize> = (0..raw.edges.len()).collect();
    let mut turned = vec![false; raw.edges.len()];
    for i in 0..raw.edges.len() {
        if keep[i] != i {
            continue;
        }
        let (a, b) = ends(&raw.edges[i], &mut rep);
        let mid = |e: &RawEdge| {
            let (s, z) = (raw.points[e.start], raw.points[e.end]);
            let t = param_of(&e.curve, s.lerp(z, 0.5));
            e.curve.eval(t)
        };
        let mi = mid(&raw.edges[i]);
        for j in i + 1..raw.edges.len() {
            if keep[j] != j {
                continue;
            }
            let (c, d) = ends(&raw.edges[j], &mut rep);
            let same = (a, b) == (c, d);
            if !(same || (a, b) == (d, c)) || mid(&raw.edges[j]).dist(mi) > tolerance {
                continue;
            }
            keep[j] = i;
            turned[j] = !same;
        }
    }
    for f in &mut raw.faces {
        for (_, uses) in &mut f.bounds {
            for u in uses.iter_mut() {
                if keep[u.edge] != u.edge {
                    u.reversed ^= turned[u.edge];
                    u.edge = keep[u.edge];
                }
            }
        }
    }
    for e in &mut raw.edges {
        e.start = root(&mut rep, e.start);
        e.end = root(&mut rep, e.end);
    }
    // Each face's neighbours across its edges, and whether they agree: a shared edge runs one way
    // in one face and the other way in the other.
    let mut uses: HashMap<usize, Vec<(usize, bool)>> = HashMap::new();
    for (fi, f) in raw.faces.iter().enumerate() {
        for (_, us) in &f.bounds {
            for u in us {
                uses.entry(u.edge).or_default().push((fi, u.reversed));
            }
        }
    }
    let mut flip: Vec<Option<bool>> = vec![None; raw.faces.len()];
    for start in 0..raw.faces.len() {
        if flip[start].is_some() {
            continue;
        }
        flip[start] = Some(false);
        let mut stack = vec![start];
        while let Some(f) = stack.pop() {
            let mine = flip[f].unwrap_or(false);
            for (_, us) in &raw.faces[f].bounds {
                for u in us {
                    for &(g, rev) in uses.get(&u.edge).into_iter().flatten() {
                        if g == f || flip[g].is_some() {
                            continue;
                        }
                        // Agreeing faces use the edge opposite ways once both are turned as set.
                        let want = (u.reversed ^ mine) == rev;
                        flip[g] = Some(want);
                        stack.push(g);
                    }
                }
            }
        }
    }
    for (f, t) in raw.faces.iter_mut().zip(&flip) {
        if *t == Some(true) {
            f.reversed = !f.reversed;
            for (_, us) in &mut f.bounds {
                us.reverse();
                for u in us.iter_mut() {
                    u.reversed = !u.reversed;
                }
            }
        }
    }
    // Only the edges and points still used are kept, renumbered.
    let mut edge_at = vec![usize::MAX; raw.edges.len()];
    let mut edges = Vec::new();
    for (i, e) in raw.edges.drain(..).enumerate() {
        if keep[i] == i {
            edge_at[i] = edges.len();
            edges.push(e);
        }
    }
    raw.edges = edges;
    for f in &mut raw.faces {
        for (_, us) in &mut f.bounds {
            for u in us.iter_mut() {
                u.edge = edge_at[u.edge];
            }
        }
    }
    let mut point_at = vec![usize::MAX; raw.points.len()];
    let (mut points, mut ids) = (Vec::new(), Vec::new());
    for e in &mut raw.edges {
        for v in [&mut e.start, &mut e.end] {
            if point_at[*v] == usize::MAX {
                point_at[*v] = points.len();
                points.push(raw.points[*v]);
                ids.push(raw.vertex_ids[*v]);
            }
            *v = point_at[*v];
        }
    }
    for f in &mut raw.faces {
        for p in &mut f.points {
            *p = root(&mut rep, *p);
            if point_at[*p] == usize::MAX {
                point_at[*p] = points.len();
                points.push(raw.points[*p]);
                ids.push(raw.vertex_ids[*p]);
            }
            *p = point_at[*p];
        }
    }
    raw.points = points;
    raw.vertex_ids = ids;
    raw.edges.iter_mut().for_each(|e| e.faces.clear());
    for (fi, f) in raw.faces.iter().enumerate() {
        for (_, us) in &f.bounds {
            for u in us {
                raw.edges[u.edge].faces.push(fi);
            }
        }
    }
    raw.shells = vec![(0..raw.faces.len()).collect()];
    let (ranges, _) = heal::heal(&mut raw)?;
    assemble(&raw, &ranges, None)
}

