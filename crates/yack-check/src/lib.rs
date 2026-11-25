// yack: The validity checker: a solid is valid when its shells are closed oriented 2-manifolds, every
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/check.rs to yack-check (M3 W6); paths and visibility only;
// M3 hardening (w-h3): face pairs found apart are kept by Brep::mark_apart / known_apart;
// M3 hardening (w-h2): its parts split out into edges, loops, solids, crossings and vertices.
//! The validity checker: a solid is valid when its shells are closed oriented 2-manifolds, every
//! edge lies on both its faces within tolerance, and each loop closes in its face's parameters.

// springrep's check indexes by position; D26's hardening pass moves it onto checked access,
// as for yack-linalg until M5.
#![allow(clippy::indexing_slicing)]

// springrep's foundation modules (D26), under the names check's code uses.
use yack_geom as geom;
use yack_intersect as intersect;
use yack_locate as locate;
use yack_math::kernel as math;
use yack_measure as measure;
use yack_tol as tol;
use yack_topo as topo;
use std::fmt;

mod crossings;
mod edges;
mod loops;
mod solids;
mod vertices;

pub use solids::{turned_shells, void_hosts};
pub use vertices::{contacts, faces_at_contacts};
pub(crate) use edges::fold;

use crate::{
    crossings::{Crossing, crossings, depth},
    loops::{stray_loop, uv_gap, within_vertex},
    math::{Point3, Vec2},
    solids::{nested, stray_voids},
    topo::{Brep, FaceId, VertexId, coedge_params, face_domain},
    vertices::{coincident_vertices, vertex_shells},
};

#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    pub message: String,
}

impl fmt::Display for Issue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

const SAMPLES: usize = 16;
/// A loop whose sampled outline encloses less than this times its perimeter squared is a sliver: a
/// circle encloses 1/4π of it, a strip a hundred times longer than it is wide 1/400.
const SLIVER_AREA: f64 = 1e-3;

/// Checks every solid of `b` as one shape: solids of one result may touch, at vertices or along
/// faces within the tolerance, as two pieces of a cut that meet at a point do; one passing into
/// another or lying inside it makes it invalid.
pub fn check(b: &Brep) -> Vec<Issue> {
    check_with(b, false, None, true)
}

/// [`check`] without the search for faces passing through each other: the structure, the geometry
/// each entity lies on, and every loop's closure and direction. An imported solid is refused for
/// these; one passing through itself is kept and reported by [`self_crossings`].
/// The checks that see nothing of a shell's facing: each edge's range and how far its curve ends
/// from its vertices, and how far each coedge lies from its face's surface. A body these refuse
/// is refused whichever way its shells face, so a reader can refuse it before working out their
/// facing, which takes a volume per shell (bench swept/ring-carried-along-a-spline).
pub fn check_distances(b: &Brep) -> Vec<Issue> {
    let mut out = Vec::new();
    let mut issue = |m: String| out.push(Issue { message: m });
    for e in b.edge_ids() {
        let edge = b.edge(e);
        if edge.range.0.partial_cmp(&edge.range.1) != Some(std::cmp::Ordering::Less) {
            issue(format!("edge {} has an empty range", e.index()));
            continue;
        }
        if let Some(c) = &edge.curve {
            for (t, v) in [(edge.range.0, edge.start), (edge.range.1, edge.end)] {
                let d = c.eval(t).dist(b.vertex(v).point);
                if d > b.vertex_tolerance(v) {
                    issue(format!("edge {} ends {d:e} mm from vertex {}", e.index(), v.index()));
                }
            }
        }
    }
    for f in b.face_ids() {
        let face = b.face(f);
        for &lp in &face.loops {
            for &c in &b.lp(lp).coedges {
                let ts = coedge_params(b, c, SAMPLES);
                let uvs = b.coedge_uvs(c, &ts);
                let e = b.coedge(c).edge;
                let edge = b.edge(e);
                let within = match edge.curve {
                    Some(_) => b.edge_tolerance(e),
                    None => b.vertex_tolerance(edge.start),
                };
                let far = ts.iter().zip(&uvs).find_map(|(t, uv)| {
                    let want = match &edge.curve {
                        Some(curve) => curve.eval(*t),
                        None => b.vertex(edge.start).point,
                    };
                    let d = face.surface.eval(uv.x, uv.y).dist(want);
                    (!crate::math::within(d, within, want)).then_some(d)
                });
                if let Some(d) = far {
                    issue(format!("coedge {} of face {} is {d:e} mm off its surface", c.index(), f.index()));
                }
            }
        }
    }
    out
}

pub fn check_structure(b: &Brep) -> Vec<Issue> {
    check_with(b, false, None, false)
}

/// Where two faces of one solid of `b` that share no edge meet, and how far they pass through each
/// other there: an imported mesh stitched with small folds passes through itself by microns.
pub fn self_crossings(b: &Brep) -> Vec<SelfCrossing> {
    crossings(b, None, true)
        .into_iter()
        .map(|x| SelfCrossing {
            a: x.a,
            b: x.b,
            at: x.at,
            depth: depth(b, x.a, x.b).max(depth(b, x.b, x.a)),
        })
        .collect()
}

/// Two faces of one solid that share no edge but meet at `at`, and how deep one passes through
/// the other in mm: zero where they only touch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SelfCrossing {
    pub a: FaceId,
    pub b: FaceId,
    pub at: Point3,
    pub depth: f64,
}

/// Checks each solid of `b` on its own, as separate parts that may touch one another: a STEP
/// assembly's placed solids, where a part seated on a board shares the board's corners within
/// the tolerance. Within any one solid it finds everything [`check`] does.
pub fn check_each(b: &Brep) -> Vec<Issue> {
    check_with(b, true, None, true)
}

/// Checks a boolean's result, whose faces were cut from the inputs `sides` gives, by face. Each
/// input is a valid solid, so two faces cut from one meet only where the input faces they were cut
/// from did, along edges and at vertices the result keeps; only pairs from different inputs are
/// searched for faces passing through each other.
pub fn check_cut(b: &Brep, sides: &[Option<usize>]) -> Vec<Issue> {
    check_with(b, false, Some(sides), true)
}

fn check_with(b: &Brep, each: bool, sides: Option<&[Option<usize>]>, crossing: bool) -> Vec<Issue> {
    let mut out = Vec::new();
    let found = std::cell::Cell::new(0usize);
    let mut issue = |m: String| {
        found.set(found.get() + 1);
        out.push(Issue { message: m });
    };

    for e in b.edge_ids() {
        let edge = b.edge(e);
        if edge.range.0.partial_cmp(&edge.range.1) != Some(std::cmp::Ordering::Less) {
            issue(format!("edge {} has an empty range", e.index()));
            continue;
        }
        if let Some(c) = &edge.curve {
            if let Some(p) = fold(c, edge.range) {
                issue(format!(
                    "edge {} turns back on itself near ({:.6}, {:.6}, {:.6})",
                    e.index(),
                    p.x,
                    p.y,
                    p.z
                ));
            }
            for (t, v) in [(edge.range.0, edge.start), (edge.range.1, edge.end)] {
                let d = c.eval(t).dist(b.vertex(v).point);
                if d > b.vertex_tolerance(v) {
                    issue(format!(
                        "edge {} ends {d:e} mm from vertex {}",
                        e.index(),
                        v.index()
                    ));
                }
            }
        }
    }

    for v in b.vertex_ids() {
        let p = b.vertex(v).point;
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            issue(format!("vertex {} is not a finite point", v.index()));
        }
    }
    // Vertices of different shells may coincide: the shells touch there.
    let solid_of = Some(vertex_shells(b));
    let apart = |v: VertexId, w: VertexId| {
        solid_of
            .as_ref()
            .is_some_and(|m| m.get(&v).is_some() && m.get(&v) != m.get(&w))
    };
    for (v, w) in coincident_vertices(b) {
        if apart(v, w) {
            continue;
        }
        issue(format!(
            "vertices {} and {} are within the tolerance, so the same point",
            v.index(),
            w.index()
        ));
    }
    for e in b.edge_ids() {
        let edge = b.edge(e);
        let Some(c) = &edge.curve else { continue };
        let ts: Vec<f64> = (0..=SAMPLES)
            .map(|i| edge.range.0 + (edge.range.1 - edge.range.0) * i as f64 / SAMPLES as f64)
            .collect();
        let len: f64 = ts.windows(2).map(|w| c.eval(w[0]).dist(c.eval(w[1]))).sum();
        if len.is_nan() || len <= tol::LINEAR {
            issue(format!(
                "edge {} is {len:e} mm long, within the tolerance",
                e.index()
            ));
        }
    }

    // Several fans of faces may meet at a vertex: pieces of the solid touch there.
    for e in b.edge_ids() {
        let u = b.edge_uses(e);
        if b.edge(e).is_degenerate() {
            if u.len() != 1 {
                issue(format!(
                    "degenerate edge {} is used {} times",
                    e.index(),
                    u.len()
                ));
            }
            continue;
        }
        // Two uses, or pairs of them where pieces touch along the edge: each coedge and its
        // partner run opposite ways and bound one sheet.
        if u.len() < 2 || !u.len().is_multiple_of(2) {
            issue(format!(
                "edge {} is used by {} coedges, not 2 or pairs of them",
                e.index(),
                u.len()
            ));
            continue;
        }
        for &c in u {
            let Some(p) = b.partner(c).filter(|&p| b.partner(p) == Some(c)) else {
                issue(format!(
                    "edge {} is used by {} coedges without pairing them",
                    e.index(),
                    u.len()
                ));
                break;
            };
            if c > p {
                continue;
            }
            if b.coedge(c).reversed == b.coedge(p).reversed {
                issue(format!(
                    "edge {} is used twice in the same direction",
                    e.index()
                ));
            }
            if b.face(b.coedge_face(c)).shell != b.face(b.coedge_face(p)).shell {
                issue(format!(
                    "edge {} joins faces of different shells",
                    e.index()
                ));
            }
        }
    }

    for f in b.face_ids() {
        let face = b.face(f);
        if face.loops.is_empty() {
            issue(format!("face {} has no loops", f.index()));
        }
        match face.shell {
            None => issue(format!("face {} is in no shell", f.index())),
            Some(s) if b.shell(s).solid.is_none() => {
                issue(format!("shell {} is in no solid", s.index()))
            }
            _ => {}
        }
        // A face's own geometry, once found sound, stays sound while the memo lives. A pole's
        // vertex is not part of that geometry, so a face with one is sampled every time.
        let memo = b.memo(f);
        let has_pole = face.loops.iter().any(|&lp| {
            b.lp(lp)
                .coedges
                .iter()
                .any(|&c| b.edge(b.coedge(c).edge).is_degenerate())
        });
        let sampled = has_pole || memo.clean.get().is_none();
        let before = found.get();
        for (li, &lp) in face.loops.iter().enumerate() {
            let cs = &b.lp(lp).coedges;
            if cs.is_empty() {
                issue(format!("face {} loop {li} is empty", f.index()));
                continue;
            }
            for (k, &c) in cs.iter().enumerate() {
                let next = cs[(k + 1) % cs.len()];
                if b.coedge_end(c) != b.coedge_start(next) {
                    issue(format!(
                        "face {} loop {li} breaks after coedge {}",
                        f.index(),
                        c.index()
                    ));
                }
            }
            if !sampled {
                continue;
            }
            let mut ends: Vec<(Vec2, Vec2)> = Vec::new();
            let mut ring: Vec<Vec2> = Vec::new();
            for &c in cs {
                let ts = coedge_params(b, c, SAMPLES);
                let uvs = b.coedge_uvs(c, &ts);
                ring.extend_from_slice(&uvs[..uvs.len() - 1]);
                let e = b.coedge(c).edge;
                let edge = b.edge(e);
                let within = match edge.curve {
                    Some(_) => b.edge_tolerance(e),
                    None => b.vertex_tolerance(edge.start),
                };
                for (t, uv) in ts.iter().zip(&uvs) {
                    let want = match &edge.curve {
                        Some(curve) => curve.eval(*t),
                        None => b.vertex(edge.start).point,
                    };
                    let d = face.surface.eval(uv.x, uv.y).dist(want);
                    if !crate::math::within(d, within, want) {
                        issue(format!(
                            "coedge {} of face {} is {d:e} mm off its surface",
                            c.index(),
                            f.index()
                        ));
                        break;
                    }
                }
                ends.push((uvs[0], uvs[uvs.len() - 1]));
            }
            for k in 0..ends.len() {
                let (end, start) = (ends[k].1, ends[(k + 1) % ends.len()].0);
                // Two coedges' images of a tolerant vertex are each within its sphere, which
                // spans more parameter where the surface moves slower than one mm per unit.
                let mut spread = 2.0 * (b.vertex_tolerance(b.coedge_end(cs[k])) - tol::LINEAR);
                if spread > 0.0 {
                    let (_, su, sv) = face.surface.d1(end.x, end.y);
                    let slowest = su.norm().min(sv.norm()).clamp(1e-12, 1.0);
                    spread *= std::f64::consts::SQRT_2 / slowest;
                }
                if uv_gap(b, f, end, start) > 10.0 * tol::LINEAR + spread
                    && !within_vertex(b, f, cs[k], end, start)
                {
                    issue(format!(
                        "face {} ({}) loop {li} does not close in parameters after coedge {}: \
                         ({:.6}, {:.6}) to ({:.6}, {:.6})",
                        f.index(),
                        face.surface.kind(),
                        cs[k].index(),
                        end.x,
                        end.y,
                        start.x,
                        start.y
                    ));
                }
            }
            let area: f64 = (0..ring.len())
                .map(|i| ring[i].cross(ring[(i + 1) % ring.len()]))
                .sum();
            // A sliver's chords sag further than it is wide, so its sampled outline can enclose
            // either sign: there the exact parameter curves decide.
            let perimeter: f64 = (0..ring.len())
                .map(|i| ring[i].dist(ring[(i + 1) % ring.len()]))
                .sum();
            let area = if area.abs() < SLIVER_AREA * perimeter * perimeter {
                crate::measure::loop_area(b, lp)
            } else {
                area
            };
            let want_positive = (li == 0) != face.reversed;
            if !b.is_vertex_loop(lp) && (area > 0.0) != want_positive {
                issue(format!("face {} loop {li} runs the wrong way", f.index()));
            }
        }
        if sampled
            && !has_pole
            && found.get() == before
            && face.loops.len() > 1
            && let Some(li) = stray_loop(b, f)
        {
            issue(format!(
                "face {} loop {li} lies outside the region its other loops bound",
                f.index()
            ));
        }
        if sampled && !has_pole && found.get() == before {
            let _ = memo.clean.set(());
        }
    }

    for s in b.solid_ids() {
        if b.solid(s).shells.is_empty() {
            issue(format!("solid {} has no shells", s.index()));
        }
    }
    if out.is_empty() {
        for s in b.solid_ids() {
            let v = measure::solid_volume_sign(b, s);
            if v.is_nan() || v <= 0.0 {
                out.push(Issue {
                    message: format!("solid {} has volume {v}", s.index()),
                });
            }
        }
    }
    if out.is_empty() {
        for (s, sh, outer) in turned_shells(b) {
            let what = if outer { "an outer sheet" } else { "a void" };
            out.push(Issue {
                message: format!(
                    "shell {} of solid {} is {what} turned inside out",
                    sh.index(),
                    s.index()
                ),
            });
        }
    }
    if out.is_empty() && crossing {
        // Faces of different shells that meet without passing into each other only touch, at
        // points or along curves. Faces on one surface meeting inside either share an area, which
        // makes the two shells one: the regularized result has that area inside it.
        let inside = |f: FaceId, p: Point3| {
            matches!(
                face_domain(b, f).locate(p),
                crate::topo::domain::Where::Inside(_)
            )
        };
        // Whether some point inside face `f` lies inside face `g` too.
        let overlaps = |f: FaceId, g: FaceId| {
            let d = face_domain(b, f);
            d.interior_point()
                .is_some_and(|uv| inside(g, d.surface.eval(uv.x, uv.y)))
        };
        let touching = |x: &Crossing| {
            b.face(x.a).shell != b.face(x.b).shell
                && depth(b, x.a, x.b).max(depth(b, x.b, x.a)) <= tol::LINEAR
                && !(crate::intersect::ssi::same_domain(&b.face(x.a).surface, &b.face(x.b).surface)
                    && (inside(x.a, x.at)
                        || inside(x.b, x.at)
                        || overlaps(x.a, x.b)
                        || overlaps(x.b, x.a)))
        };
        for x in crossings(b, sides, each)
            .into_iter()
            .filter(|x| !touching(x))
        {
            out.push(Issue {
                message: format!(
                    "faces {} and {} share no edge but meet at ({:.6}, {:.6}, {:.6}): the solid \
                     passes through itself",
                    x.a.index(),
                    x.b.index(),
                    x.at.x,
                    x.at.y,
                    x.at.z
                ),
            });
        }
    }
    if out.is_empty() {
        for (s, sh) in stray_voids(b) {
            out.push(Issue {
                message: format!(
                    "void shell {} of solid {} lies outside its outer shell",
                    sh.index(),
                    s.index()
                ),
            });
        }
    }
    if out.is_empty() && !each {
        for (s, t) in nested(b) {
            out.push(Issue {
                message: format!("solid {} lies inside solid {}", s.index(), t.index()),
            });
        }
    }
    out
}
