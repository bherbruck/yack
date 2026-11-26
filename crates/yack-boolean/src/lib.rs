// yack: Boolean union, difference and intersection of solids.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); Work's phases and the
// free helpers split out into modules (M3 hardening B0); paths and visibility only; the boolean's
// entry points on checked access, the solid copies' id maps BTreeMaps, small literals named (M3 B8).
//! Boolean union, difference and intersection of solids.
//!
//! The steps, each in its own module:
//!
//! 1. Every edge of one solid is intersected with every face of the other (`csi`), coplanar edges
//!    with each other, and every face pair's surfaces with each other (`ssi`, `march`). Each
//!    meeting point becomes a vertex, shared by every curve through it.
//! 2. Every edge and intersection curve is split at every vertex on it, and an edge lying on a face
//!    of the other solid cuts it. Pieces that run along each other are one edge, so both faces of
//!    every edge use the same one.
//! 3. Each face is split by the intersection pieces inside it, and by the boundary of any face of
//!    the other solid that lies on its surface (`arrange`).
//! 4. Each piece of face is classified inside, outside, or on the other solid (`classify`); the
//!    operation selects pieces, and `sew` joins them, merges faces on one surface, and checks the
//!    result.

// springrep's boolean code indexes by position; D26's hardening pass moves it onto checked access,
// as for yack-linalg until M5.
#![allow(clippy::indexing_slicing)]

// springrep's foundation modules (D26), under the names the moved code uses.
use yack_base::kernel::Error;
use yack_base::kernel::Result;
use yack_check as check;
use yack_base::kernel as error;
use yack_geom as geom;
use yack_intersect as intersect;
use yack_locate as locate;
use yack_math::kernel as math;
use yack_measure as measure;
use yack_mesh as mesh;
use yack_numeric as numeric;
use yack_profile as profile;
use yack_build::sweep;
use yack_tol as tol;
use yack_topo as topo;
use yack_tweak as tweak;
#[cfg(test)]
use yack_codec as codec;
#[cfg(test)]
use yack_build::loft;
#[cfg(test)]
use yack_build::primitive;
mod arrange;
mod bounds;
mod cut;
mod meet;
mod near;
mod plane;
mod same;
mod section;
mod select;
mod sew;
mod simplify;
mod snap;
mod split;
mod unify;
mod work;

pub use plane::{half_space, split};
pub use section::{SectionEdge, section, section_area, section_curves};
pub use unify::unify;

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;
use std::rc::Rc;

use self::sew::{OutFace, PieceGeom};
use self::bounds::{overlap, padded};
use crate::{
    intersect::{
        contact, csi,
        ssi::{self, Ssi},
    },
    locate::{classify, same_domain_faces},
    topo::{
        coedge_trim_with, coedge_uv_at,
        domain::{Domain, Trim, UvMap, Where},
    },
};
use crate::{
    topo::coedge_params,
    geom::{Curve, Surface},
    math::{Aabb, Frame, Point3, Vec2, Vec3},
    math::bvh::Bvh,
    topo::{
        Brep, CoedgeId, CoedgeSpec, EdgeId, FaceId, FaceName, Facts, OnFace, SolidId,
        VertexId,
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Union,
    Difference,
    Intersection,
}

pub fn union(a: &Brep, b: &Brep) -> Result<Brep> {
    boolean(a, b, Op::Union)
}

/// `a` with `b` taken away.
pub fn difference(a: &Brep, b: &Brep) -> Result<Brep> {
    boolean(a, b, Op::Difference)
}

pub fn intersection(a: &Brep, b: &Brep) -> Result<Brep> {
    boolean(a, b, Op::Intersection)
}

/// The operands sorted into groups whose members' boxes are apart from each other, each operand
/// by its place in `parts`. Solids whose boxes are apart share no point, so a group in one arena
/// is its own union, and one boolean combines the whole group with another solid.
pub fn apart_groups(parts: &[&Brep]) -> Vec<Vec<usize>> {
    let boxes: Vec<Aabb> = parts
        .iter()
        .map(|p| padded(&measure::bbox(p), tol::LINEAR))
        .collect();
    let below = |lo: Vec3, hi: Vec3| lo.to_array().iter().zip(&hi.to_array()).all(|(l, h)| l <= h);
    let meet = |a: &Aabb, b: &Aabb| below(a.min, b.max) && below(b.min, a.max);
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for (i, bx) in boxes.iter().enumerate() {
        if bx.is_empty() {
            continue;
        }
        match groups
            .iter_mut()
            .find(|g| g.iter().all(|&j| boxes.get(j).is_none_or(|bj| !meet(bx, bj))))
        {
            Some(g) => g.push(i),
            None => groups.push(vec![i]),
        }
    }
    groups
}

/// One arena holding every solid of the operands named.
fn gathered(parts: &[&Brep], group: &[usize]) -> Brep {
    let mut out = Brep::new();
    for p in group.iter().filter_map(|&i| parts.get(i)) {
        out.append(p);
    }
    out
}

/// The union of every operand. Operands apart from each other are gathered and combined in one
/// boolean, so a pattern costs as many booleans as its overlaps need, not one for each solid.
pub fn union_all(parts: &[&Brep]) -> Result<Brep> {
    let mut groups = apart_groups(parts).into_iter();
    let Some(first) = groups.next() else {
        return Ok(Brep::new());
    };
    let mut out = gathered(parts, &first);
    for g in groups {
        out = union(&out, &gathered(parts, &g))?;
    }
    Ok(out)
}

/// `target` with every tool taken away. Tools apart from each other are gathered and cut in one
/// boolean, so a drilled pattern costs as many booleans as its overlaps need, not one for each
/// hole.
pub fn difference_all(target: &Brep, tools: &[&Brep]) -> Result<Brep> {
    let mut out = target.clone();
    for g in apart_groups(tools) {
        out = difference(&out, &gathered(tools, &g))?;
    }
    Ok(out)
}

/// Combines every solid of `a` with every solid of `b`. The result passes [`crate::check`], or the
/// operation returns [`Error::InvalidResult`]. Pieces of the result that touch, along a curve or at
/// a point, are one solid: the curve an edge four faces use, the point a vertex two fans meet at.
/// A result with no material, such as a solid minus itself or the common
/// part of disjoint solids, is an empty arena: no solids, which callers must check for.
pub fn boolean(a: &Brep, b: &Brep, op: Op) -> Result<Brep> {
    boolean_with(a, b, op, &Facts::default())
}

/// [`boolean`], taking `facts` the caller's construction makes true instead of computing them. A
/// fact that does not hold is [`Error::InvalidInput`]. Facts name the inputs' own ids; where some
/// solids pass through untouched, the facts about the rest follow them into the copies combined.
pub fn boolean_with(a: &Brep, b: &Brep, op: Op, facts: &Facts) -> Result<Brep> {
    Ok(simplify::merge_edges(combined(a, b, op, facts)?))
}

/// [`boolean_with`] before redundant vertices are dissolved.
fn combined(a: &Brep, b: &Brep, op: Op, facts: &Facts) -> Result<Brep> {
    // Facts name `b`'s own faces and edges, which a snapped copy would renumber.
    if !facts.on_face.is_empty() {
        return combined_as(a, b, op, facts);
    }
    let snapped = snap::snapped(a, b, snap::Reach::Exact);
    let first = combined_as(a, snapped.as_ref().unwrap_or(b), op, facts);
    if first.is_ok() {
        return first;
    }
    // Shapes a hair off each other that the boolean could not place, retried with them on one
    // surface: the answer then differs from the exact one by at most the gap times their area.
    match snap::snapped(a, b, snap::Reach::Grazing) {
        Some(wide) => combined_as(a, &wide, op, facts).or(first),
        None => first,
    }
}

fn combined_as(a: &Brep, b: &Brep, op: Op, facts: &Facts) -> Result<Brep> {
    let (ba, bb) = (SolidBoxes::new(a), SolidBoxes::new(b));
    if let Some(out) = without_coincident(a, &ba, b, &bb, op, facts)? {
        return Ok(out);
    }
    let near = |own: &SolidBoxes, others: &SolidBoxes| -> Vec<bool> {
        (0..own.ids.len())
            .map(|i| (0..others.ids.len()).any(|j| own.meet(i, others, j)))
            .collect()
    };
    let (na, nb) = (near(&ba, &bb), near(&bb, &ba));
    if na.iter().chain(&nb).all(|&n| n) {
        return combine(a, b, op, facts);
    }
    let pick = |boxes: &SolidBoxes, n: &[bool], want: bool| -> Vec<SolidId> {
        boxes
            .ids
            .iter()
            .zip(n)
            .filter(|&(_, &k)| k == want)
            .map(|(&s, _)| s)
            .collect()
    };
    // Solids whose boxes meet nothing of the other input pass through untouched.
    let (near_a, near_b) = (pick(&ba, &na, true), pick(&bb, &nb, true));
    let mut out = if near_a.is_empty() {
        Brep::new()
    } else {
        let (sa, sb) = (copy_solids(a, &near_a), copy_solids(b, &near_b));
        combine(&sa.0, &sb.0, op, &follow(facts, Some(&sa), Some(&sb)))?
    };
    match op {
        Op::Union => {
            out.append(&solids_of(a, &pick(&ba, &na, false)));
            out.append(&solids_of(b, &pick(&bb, &nb, false)));
        }
        Op::Difference => {
            out.append(&solids_of(a, &pick(&ba, &na, false)));
        }
        Op::Intersection => {}
    }
    Ok(out)
}

/// The boolean where some solid of `a` and some solid of `b` are one solid within their tolerances
/// ([`same::same_solid`]), from the rest: such a pair is its own union and intersection, and takes
/// all of itself from a difference. `None` when no solid of `a` is one of `b`'s.
fn without_coincident(
    a: &Brep,
    ba: &SolidBoxes,
    b: &Brep,
    bb: &SolidBoxes,
    op: Op,
    facts: &Facts,
) -> Result<Option<Brep>> {
    let widest = |x: &Brep| {
        let (vs, es) = x.tolerant();
        vs.iter()
            .map(|t| t.1)
            .chain(es.iter().map(|t| t.1))
            .fold(tol::LINEAR, f64::max)
    };
    // A copy's box moves with it; this only keeps the topology comparisons to likely pairs.
    let reach = 2.0 * (widest(a) + widest(b)) + COPY_REACH;
    let close = |x: &Aabb, y: &Aabb| {
        (0..3).all(|k| (x.min[k] - y.min[k]).abs() <= reach && (x.max[k] - y.max[k]).abs() <= reach)
    };
    let mut taken = vec![false; bb.ids.len()];
    let mut matched = vec![false; ba.ids.len()];
    for (i, sa) in ba.ids.iter().enumerate() {
        let mut one: Option<Brep> = None;
        for (j, sb) in bb.ids.iter().enumerate() {
            // Boxes that close overlap, so solids whose cheap boxes miss cannot match.
            if taken.get(j) == Some(&true)
                || !ba.hulls.get(i).zip(bb.hulls.get(j)).is_some_and(|(x, y)| overlap(x, y))
                || !ba.loose(i).zip(bb.loose(j)).is_some_and(|(x, y)| close(&x, &y))
            {
                continue;
            }
            let x = one.get_or_insert_with(|| solids_of(a, &[*sa]));
            if same::same_solid(x, &solids_of(b, &[*sb])) {
                if let (Some(t), Some(m)) = (taken.get_mut(j), matched.get_mut(i)) {
                    (*t, *m) = (true, true);
                }
                break;
            }
        }
    }
    if !matched.contains(&true) {
        return Ok(None);
    }
    let pick = |boxes: &SolidBoxes, flags: &[bool], want: bool| -> Vec<SolidId> {
        boxes
            .ids
            .iter()
            .zip(flags)
            .filter(|&(_, &f)| f == want)
            .map(|(&s, _)| s)
            .collect()
    };
    let rest_a = pick(ba, &matched, false);
    let rest_b = pick(bb, &taken, false);
    Ok(Some(match op {
        Op::Union if rest_b.is_empty() => a.clone(),
        Op::Union => {
            let cb = copy_solids(b, &rest_b);
            boolean_with(a, &cb.0, op, &follow(facts, None, Some(&cb)))?
        }
        Op::Intersection => {
            let mut out = if rest_a.is_empty() || rest_b.is_empty() {
                Brep::new()
            } else {
                let (ca, cb) = (copy_solids(a, &rest_a), copy_solids(b, &rest_b));
                boolean_with(&ca.0, &cb.0, op, &follow(facts, Some(&ca), Some(&cb)))?
            };
            out.append(&solids_of(a, &pick(ba, &matched, true)));
            out
        }
        // The matched solids of `b` stay: they may overlap the rest of `a` where `a`'s own
        // solids overlap.
        Op::Difference if rest_a.is_empty() => Brep::new(),
        Op::Difference => {
            let ca = copy_solids(a, &rest_a);
            boolean_with(&ca.0, b, op, &follow(facts, Some(&ca), None))?
        }
    }))
}

/// `facts` about the inputs restated for copies of some of their solids, `None` for an input
/// passed whole. The copies number their edges and faces afresh, and a fact about a solid left out
/// has nothing left to say.
fn follow(facts: &Facts, a: Option<&Copied>, b: Option<&Copied>) -> Facts {
    let edge = |c: Option<&Copied>, e: EdgeId| c.map_or(Some(e), |c| c.1.get(&e).copied());
    let face = |c: Option<&Copied>, f: FaceId| c.map_or(Some(f), |c| c.2.get(&f).copied());
    let on_face = facts
        .on_face
        .iter()
        .filter_map(|f| {
            let (own, other) = if f.side == 0 { (a, b) } else { (b, a) };
            Some(OnFace {
                side: f.side,
                edge: edge(own, f.edge)?,
                face: face(other, f.face)?,
            })
        })
        .collect();
    Facts { on_face }
}

/// Whether `c` stays within `t` of surface `s` over `range`, at 33 points: a tolerant edge lying
/// on a face of the other solid within its own tolerance.
fn lies_within(c: &Curve, range: (f64, f64), s: &Surface, t: f64) -> bool {
    (0..=32).all(|k| {
        let p = c.eval(range.0 + (range.1 - range.0) * k as f64 / 32.0);
        csi::signed_distance(s, p).abs() <= t
    })
}

/// Each solid's box, widened by the tolerance so solids within it of each other overlap.
/// Each solid of an arena with a box that holds it, from [`measure::face_hull_box`], and its loose
/// box, worked out only where asked: the decisions read loose boxes, and solids whose hull boxes
/// miss each other have loose boxes that miss too.
struct SolidBoxes<'a> {
    x: &'a Brep,
    ids: Vec<SolidId>,
    hulls: Vec<Aabb>,
    loose: Vec<std::cell::OnceCell<Aabb>>,
}

impl<'a> SolidBoxes<'a> {
    fn new(x: &'a Brep) -> SolidBoxes<'a> {
        let ids: Vec<SolidId> = x.solid_ids().collect();
        let hulls = ids
            .iter()
            .map(|&s| {
                let mut bb = Aabb::EMPTY;
                for f in x.solid_faces(s) {
                    bb.union(&measure::face_hull_box(x, f));
                }
                padded(&bb, tol::LINEAR)
            })
            .collect();
        SolidBoxes {
            x,
            loose: vec![std::cell::OnceCell::new(); ids.len()],
            ids,
            hulls,
        }
    }

    fn loose(&self, i: usize) -> Option<Aabb> {
        let (cell, &id) = (self.loose.get(i)?, self.ids.get(i)?);
        Some(*cell.get_or_init(|| {
            let bb = if self.ids.len() == 1 {
                measure::loose_bbox(self.x)
            } else {
                measure::loose_bbox(&solids_of(self.x, &[id]))
            };
            padded(&bb, tol::LINEAR)
        }))
    }

    /// Whether solid `i` of these and solid `j` of `other` have loose boxes that meet; a solid not
    /// among them meets everything.
    fn meet(&self, i: usize, other: &SolidBoxes, j: usize) -> bool {
        let (Some(h), Some(g)) = (self.hulls.get(i), other.hulls.get(j)) else {
            return true;
        };
        overlap(h, g)
            && match (self.loose(i), other.loose(j)) {
                (Some(x), Some(y)) => overlap(&x, &y),
                _ => true,
            }
    }
}

/// Added to the two inputs' widest tolerances when copies' boxes are compared, in mm.
const COPY_REACH: f64 = 1e-6; // tol: slack on a box comparison that only shortlists pairs

/// A new arena holding copies of `solids` of `x`: one piece of a result that came apart, say, to
/// measure or cut on its own.
pub fn solids_of(x: &Brep, solids: &[SolidId]) -> Brep {
    copy_solids(x, solids).0
}

type Copied = (Brep, BTreeMap<EdgeId, EdgeId>, BTreeMap<FaceId, FaceId>);

/// [`solids_of`], with the copy's id of each edge and face copied.
fn copy_solids(x: &Brep, solids: &[SolidId]) -> Copied {
    let mut out = Brep::new();
    // all three looked up only: the copy is built in `x`'s own order
    let mut verts: BTreeMap<VertexId, VertexId> = BTreeMap::new();
    let mut edges: BTreeMap<EdgeId, EdgeId> = BTreeMap::new();
    let mut face_ids: BTreeMap<FaceId, FaceId> = BTreeMap::new();
    for &s in solids {
        let mut shells = Vec::new();
        for &sh in &x.solid(s).shells {
            let mut faces = Vec::new();
            for &f in &x.shell(sh).faces {
                let face = x.face(f);
                let nf = out.add_face(face.surface.clone(), face.reversed, face.names.clone());
                for &lp in &face.loops {
                    let mut specs = Vec::new();
                    for &c in &x.lp(lp).coedges {
                        let co = x.coedge(c);
                        let e = match edges.get(&co.edge) {
                            Some(&e) => e,
                            None => {
                                let edge = x.edge(co.edge);
                                let mut v = |id: VertexId| {
                                    *verts.entry(id).or_insert_with(|| {
                                        let nv = out.add_vertex(x.vertex(id).point);
                                        out.set_vertex_tolerance(nv, x.vertex_tolerance(id));
                                        nv
                                    })
                                };
                                let (v0, v1) = (v(edge.start), v(edge.end));
                                let e = match &edge.curve {
                                    Some(cv) => out.add_edge(cv.clone(), edge.range, v0, v1),
                                    None => out.add_degenerate_edge(v0, edge.range),
                                };
                                out.set_edge_tolerance(e, x.edge_tolerance(co.edge));
                                edges.insert(co.edge, e);
                                e
                            }
                        };
                        specs.push(CoedgeSpec {
                            edge: e,
                            reversed: co.reversed,
                            pcurve: co.pcurve.clone(),
                        });
                    }
                    out.add_loop(nf, specs);
                }
                out.share_memo(nf, x.shared_memo(f));
                face_ids.insert(f, nf);
                faces.push(nf);
            }
            shells.push(out.add_shell(faces));
        }
        out.add_solid_of(shells, x.solid(s).outer);
    }
    out.pair_radial();
    (out, edges, face_ids)
}

fn combine(a: &Brep, b: &Brep, op: Op, facts: &Facts) -> Result<Brep> {
    let mut w = Work::new([a, b])?;
    if w.many_pairs() {
        w.find_near();
    }
    w.finish(op, facts)
}

/// What a boolean reads from an input face's geometry, kept in the face's memo: each coedge's
/// whole trim (parameter range and parameter-plane ends) and its ends as one piece of the
/// boundary, the domain's samples and boxes, and the face's box.
#[derive(Debug)]
pub(crate) struct FaceImages {
    trims: Vec<Vec<TrimImage>>,
    samples: Vec<Vec<Vec<Vec2>>>,
    boxes: Vec<Vec<Aabb>>,
    bbox: Aabb,
}

impl crate::topo::MemoSlot for FaceImages {}

#[derive(Debug)]
struct TrimImage {
    t: (f64, f64),
    uv: (Vec2, Vec2),
    whole: (Vec2, Vec2),
}

/// A face of either input.
struct Face {
    side: usize,
    id: FaceId,
    dom: Domain,
    bbox: Aabb,
}

/// A curve the result's edges are cut from: an input edge, or a face pair's intersection.
struct Base {
    curve: Option<Rc<Curve>>,
    range: (f64, f64),
    v0: Option<usize>,
    v1: Option<usize>,
    pole: Point3,
    closed: bool,
    bbox: Aabb,
    kind: BaseKind,
    /// The input edge's local tolerance, or [`tol::LINEAR`] for an intersection curve.
    tol: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum BaseKind {
    Edge,
    /// The intersection of faces `fa` and `fb`.
    Cut {
        fa: usize,
        fb: usize,
        tangent: bool,
    },
}

/// A base's stretch between two adjacent vertices on it.
#[derive(Clone, Debug)]
struct Piece {
    base: usize,
    range: (f64, f64),
    v0: usize,
    v1: usize,
    /// Which of coincident pieces stands for them all: input edges before intersection curves.
    rank: u8,
}

struct Work<'a> {
    breps: [&'a Brep; 2],
    pool: Vec<Point3>,
    /// Each pool vertex's tolerance: an input vertex's own, [`tol::LINEAR`] for a meeting, and a
    /// tolerant edge's for a meeting on it, grown where it takes in another.
    vtol: Vec<f64>,
    /// The widest of `vtol`, which bounds how far apart two pool vertices can coincide.
    widest: f64,
    /// The pool vertices whose tolerance is above [`tol::LINEAR`]: few, and searched one by one.
    tolerant: Vec<usize>,
    /// The widest tolerance of the inputs, which no tolerance grows past.
    cap: f64,
    grid: BTreeMap<[i64; 3], Vec<usize>>,
    vmap: [Vec<usize>; 2],
    faces: Vec<Face>,
    face_of: [Vec<usize>; 2],
    bases: Vec<Base>,
    edge_base: [Vec<usize>; 2],
    coincident: Vec<(usize, usize)>,
    pieces: Vec<Piece>,
    base_pieces: Vec<Vec<usize>>,
    /// Faces each intersection piece crosses the inside of.
    cut_faces: Vec<Vec<usize>>,
    canon: Vec<usize>,
    /// Where each edge base meets a face of the other solid, from step 1a, before they are placed.
    hits: Vec<Hit>,
    /// Each edge base that lies in the surface of a face of the other solid, with that face.
    on_face: BTreeSet<(usize, usize)>,
    /// How many vertices of the pool are the inputs' own.
    inputs: usize,
    /// Each placed meeting where an edge runs tangent to a face: the vertex, the edge base and the
    /// face.
    met: Vec<(usize, usize, usize)>,
    /// The vertices where two faces' surfaces meet at an isolated point, as two balls touch.
    touch_points: Vec<usize>,
    /// Vertices a hit made where it was found, on no intersection curve: anywhere in the band
    /// where its edge grazes the face, as far from where they stand as the meeting fixes them.
    unplaced: BTreeMap<usize, f64>,
    /// Each vertex an edge of one face met another face at: the two faces, and how far along the
    /// graze the meeting may lie from where their intersection curve runs.
    meetings: BTreeMap<usize, Vec<(usize, usize, f64)>>,
    /// Which faces and edges of the two inputs come near each other; every pair where unknown.
    near: Option<near::Near>,
}

/// How many face pairs, or edge pairs, with overlapping boxes make [`near::Near`] worth building.
const MANY_FACE_PAIRS: usize = 256;
const MANY_EDGE_PAIRS: usize = 4096;

/// Where edge base `base` meets face `face` of the other solid: at `p`, its parameter `t`, crossing
/// at an angle whose sine is `sine`.
#[derive(Clone, Copy, Debug)]
struct Hit {
    p: Point3,
    t: f64,
    base: usize,
    face: usize,
    sine: f64,
}

/// The side of a vertex-grid cell, in mm.
const CELL: f64 = 1e-3; // tol: a bucket size for finding coincident vertices, not a tolerance

/// The furthest a grazing meeting is taken to lie from the intersection curve it belongs to.
const GRAZE_REACH: f64 = 100.0 * tol::LINEAR;

/// The sine between a piece's way in from a cut and the other face's normal below which the cut
/// does not settle the piece's side. It is a guard against rounding, not a tolerance: where the
/// surfaces cross more shallowly a ray decides.
const LOCAL_SINE: f64 = 1e-3; // tol: a sine, not a length

#[cfg(test)]
mod tests;
