// yack: Where a ray crosses a solid's faces, from the exact geometry.
// Adapted from springrep@a03e8a1 src/rays.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: meshes nothing (M2): `Rays::from_tessellation` takes the mesh; mesh::rays makes it; then to
// yack-locate (M3 W5), paths and visibility only; hardening (w-h1): a bad mesh is an error, and
// triangles, domains and runs of hits are read with checked access.
//! Where a ray crosses a solid's faces, from the exact geometry.
//!
//! [`Rays`] meshes the arena once. A ray's candidate faces are those with a mesh triangle within
//! twice the mesh deflection of it: a face's exact surface lies within the deflection of its mesh,
//! so no crossing is missed. On each, over the stretches of the ray that pass those triangles, the
//! line is intersected with the face's surface exactly, in closed form on a plane, cylinder, cone,
//! sphere and torus. A crossing counts where it lies inside the face and its `t` in the ray's
//! range. One crossing on an edge between two faces counts once, as does a ray that touches a
//! surface without crossing it.

use std::collections::BTreeMap;

use crate::{
    intersect::csi::curve_surface,
    classify::DIRECTIONS,
    topo::{
        domain::{Domain, Where},
        face_domain,
    },
    error::{Error, Result},
    geom::{Curve, Line},
    math::{Aabb, Point3, Vec3},
    math::bvh::{Bvh, segment_triangle},
    tol,
    topo::{Brep, FaceId},
};

/// Where a ray crosses a face: `t` along its unit direction, the point, the face and the face's
/// outward normal there. `on_edge` where the point lies on the face's boundary: the ray passes
/// through an edge there, and the face next to it has a crossing at the same point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub t: f64,
    pub point: Point3,
    pub face: FaceId,
    pub normal: Vec3,
    pub on_edge: bool,
}

/// How far past the ends of its range a crossing may lie and still count, and how near the
/// surface a ray stays over a stretch it only grazes.
const ON: f64 = 0.01 * tol::LINEAR;

/// Where a point lies against an arena's solids.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Inside,
    /// On the boundary, within the linear tolerance.
    On,
    Outside,
}

/// An arena prepared for casting rays at its faces: its mesh, a box tree over it, and each face's
/// trimmed domain on its surface.
pub struct Rays<'a> {
    b: &'a Brep,
    tris: Vec<(FaceId, [Point3; 3])>,
    tree: Bvh,
    deflection: f64,
    domains: Vec<Domain>,
    /// The mesh's box: the arena lies within the deflection of it.
    reach: Aabb,
}

impl<'a> Rays<'a> {
    /// The caster over `b`'s faces, from a tessellation of them within `deflection`: the triangles
    /// `positions` and `triangles` give, `faces` naming each face's run of them; an error when a
    /// triangle names a corner past `positions`, a run passes the last triangle, or a triangle lies
    /// in no face's run.
    pub fn from_tessellation(
        b: &'a Brep,
        positions: &[Point3],
        triangles: &[[u32; 3]],
        faces: &[(FaceId, std::ops::Range<usize>)],
        deflection: f64,
    ) -> Result<Rays<'a>> {
        let tree = Bvh::new(positions, triangles)?;
        let mut face_of = vec![None; triangles.len()];
        for (f, range) in faces.iter().filter(|(_, range)| !range.is_empty()) {
            let Some(run) = face_of.get_mut(range.clone()) else {
                return Err(Error::InvalidInput(format!(
                    "a face's triangles {range:?} pass the mesh's {} triangles",
                    triangles.len()
                )));
            };
            run.fill(Some(*f));
        }
        let corner = |i: u32| {
            positions.get(i as usize).copied().ok_or_else(|| {
                Error::InvalidInput(format!(
                    "a triangle's corner index {i} is past the mesh's {} positions",
                    positions.len()
                ))
            })
        };
        let tris = triangles
            .iter()
            .zip(face_of)
            // The tree names triangles by index, so each keeps its place.
            .map(|(&[i, j, k], f)| {
                let f = f.ok_or_else(|| {
                    Error::InvalidInput("a mesh triangle lies in no face's run".into())
                })?;
                Ok((f, [corner(i)?, corner(j)?, corner(k)?]))
            })
            .collect::<Result<_>>()?;
        let mut reach = Aabb::EMPTY;
        for &q in positions {
            reach.add(q);
        }
        Ok(Rays {
            b,
            tris,
            tree,
            deflection,
            domains: b.face_ids().map(|f| face_domain(b, f)).collect(),
            reach,
        })
    }

    /// Whether a solid of the arena holds `p`, by the parity of a ray's crossings of its faces: a
    /// point in a void, or between solids, is held by none. A ray that passes through an edge,
    /// grazes a face or starts on one does not say, and the next direction is tried. `None` where
    /// none says: the point is on the boundary.
    pub fn holds(&self, p: Point3) -> Option<bool> {
        if self.tris.is_empty() {
            return Some(false);
        }
        let far = self.reach.min.dist(p).max(self.reach.max.dist(p)) + self.reach.diagonal() + 1.0;
        DIRECTIONS.iter().find_map(|d| {
            let d = Vec3::from_array(*d).unit()?;
            let hits = self.crossings(p, d, (0.0, far));
            let unclear =
                |h: &Hit| h.on_edge || h.t <= tol::LINEAR || h.normal.dot(d).abs() < tol::TANGENT;
            (!hits.iter().any(unclear)).then_some(hits.len() % 2 == 1)
        })
    }

    /// Where `p` lies against the arena's solids. A point no ray decides is one every ray leaves
    /// through a face, an edge or a touch at its start: a point of the boundary.
    pub fn side(&self, p: Point3) -> Side {
        match self.holds(p) {
            Some(true) => Side::Inside,
            Some(false) => Side::Outside,
            None => Side::On,
        }
    }

    /// Every crossing of the ray `o + t·d` with the arena's faces for `t` in `[t0, t1]`, nearest
    /// first, `t` measured along `d` made a unit.
    pub fn cast(&self, o: Point3, d: Vec3, range: (f64, f64)) -> Result<Vec<Hit>> {
        let Some(d) = d.unit() else {
            return Err(Error::InvalidInput("a ray needs a direction".into()));
        };
        // Seeds from neighbouring triangles land on one crossing, and a crossing on an edge is on
        // both its faces.
        let mut hits = self.crossings(o, d, range);
        hits.dedup_by(|later, kept| later.t - kept.t <= tol::LINEAR);
        Ok(hits)
    }

    /// Every crossing of the ray, nearest first, a crossing on an edge once for each face.
    ///
    /// The mesh says where to look, and no more: each face is searched over the stretches of the
    /// ray that pass one of its triangles, and what is found there is every meeting of the line
    /// with the face's exact surface, by the intersection the booleans use. A chord across a
    /// curved face shallower than the mesh resolves is two crossings, as any other chord is.
    fn crossings(&self, o: Point3, d: Vec3, (t0, t1): (f64, f64)) -> Vec<Hit> {
        let pad = 2.0 * self.deflection;
        let (s0, s1) = (t0 - pad, t1 + pad);
        let mut near = Vec::new();
        self.tree.along(o, d, (s0, s1), pad, &mut near);
        let (a, z) = (o + d * s0, o + d * s1);
        let mut stretches: BTreeMap<FaceId, Vec<(f64, f64)>> = BTreeMap::new();
        for &k in &near {
            // The tree names triangles by their place in `tris`.
            let kept = self.tris.get(k);
            debug_assert!(kept.is_some(), "the tree's triangle {k} is past those kept");
            let Some((f, tri)) = kept else { continue };
            if segment_triangle(a, z, tri).0 > pad {
                continue;
            }
            // The surface over this triangle lies within the deflection of it, so the ray can
            // meet it only this far past the triangle's own reach along the ray.
            let along = tri.map(|v| (v - o).dot(d));
            let lo = along.iter().copied().fold(f64::INFINITY, f64::min) - pad;
            let hi = along.iter().copied().fold(f64::NEG_INFINITY, f64::max) + pad;
            stretches
                .entry(*f)
                .or_default()
                .push((lo.max(s0), hi.min(s1)));
        }
        let line = Curve::Line(Line { origin: o, dir: d });
        let mut hits: Vec<Hit> = Vec::new();
        for (f, mut spans) in stretches {
            spans.sort_by(|x, y| x.0.total_cmp(&y.0));
            let mut merged: Vec<(f64, f64)> = Vec::new();
            for (lo, hi) in spans {
                match merged.last_mut() {
                    Some(last) if lo <= last.1 => last.1 = last.1.max(hi),
                    _ => merged.push((lo, hi)),
                }
            }
            let face = self.b.face(f);
            let s = &face.surface;
            for span in merged {
                // A ray that runs in the surface crosses it nowhere.
                let found = curve_surface(&line, span, s);
                for t in found.ts {
                    if t < t0 - ON || t > t1 + ON {
                        continue;
                    }
                    let q = o + d * t;
                    // A domain was made for every face of the arena, in order.
                    let dom = self.domains.get(f.index());
                    debug_assert!(dom.is_some(), "face {} has no domain", f.index());
                    let Some(dom) = dom else { continue };
                    let (uv, on_edge) = match dom.locate(q) {
                        Where::Outside => continue,
                        Where::Inside(uv) => (uv, false),
                        Where::Boundary => (s.project(q), true),
                    };
                    hits.push(Hit {
                        t: t.clamp(t0, t1),
                        point: q,
                        face: f,
                        normal: dom.outward(uv),
                        on_edge,
                    });
                }
            }
        }
        hits.sort_by(|x, y| x.t.total_cmp(&y.t).then(x.face.cmp(&y.face)));
        self.touches(hits, o, d)
    }

    /// `hits` with each run on one face that the ray stays on between, a ray grazing the surface,
    /// as one touch. A grazing ray stays within [`ON`] of the surface over a stretch as long as
    /// the square root of that over the curvature, so its seeds settle anywhere along it; the touch
    /// is where the surface's normal turns square to the ray, which bisection finds exactly.
    fn touches(&self, hits: Vec<Hit>, o: Point3, d: Vec3) -> Vec<Hit> {
        let mut out: Vec<Hit> = Vec::with_capacity(hits.len());
        let mut rest = hits.as_slice();
        while let Some(&h) = rest.first() {
            let s = &self.b.face(h.face).surface;
            let foot = |t: f64| {
                let p = o + d * t;
                let uv = s.project(p);
                (uv, s.eval(uv.x, uv.y).dist(p))
            };
            // The run: `h` and each next hit on its face that the ray reaches without leaving
            // the surface between.
            let len = 1 + rest
                .windows(2)
                .take_while(|pair| match pair {
                    [x, y] => y.face == h.face && foot(0.5 * (x.t + y.t)).1 <= ON,
                    _ => false,
                })
                .count();
            let (run, after) = rest.split_at(len);
            rest = after;
            let [.., last] = run else { continue };
            if run.len() == 1 {
                out.push(h);
                continue;
            }
            let square = |t: f64| {
                let (uv, _) = foot(t);
                s.normal(uv.x, uv.y).dot(d)
            };
            let (mut lo, mut hi) = (h.t, last.t);
            if square(lo).signum() != square(hi).signum() {
                for _ in 0..100 {
                    let mid = 0.5 * (lo + hi);
                    if square(mid).signum() == square(lo).signum() {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
            }
            let t = 0.5 * (lo + hi);
            let (uv, _) = foot(t);
            let n = s.normal(uv.x, uv.y);
            out.push(Hit {
                t,
                point: o + d * t,
                face: h.face,
                normal: if self.b.face(h.face).reversed { -n } else { n },
                on_edge: run.iter().any(|x| x.on_edge),
            });
        }
        out
    }
}
