// yack: the swept solid as the start united with what each of its leading faces sweeps (M4.1 step 1).
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/sweep_faces.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: rewritten onto yack's crates, not ported: each leading face's sweep is a solid built from
//   its parts (yack-build's `solid_of`), united with the start by yack-boolean, and checked; so far
//   for a slide, where a cylinder or sphere the slide crosses is split along its contact (a plane)
//   by yack-boolean's `split` first. Everything else declines.
//! What any solid passes through as it slides, turns or screws: the solid where it starts, with
//! what each of its *leading* faces sweeps.
//!
//! A point the moving solid covers at some instant is inside it where it starts, or enters it
//! later across the part of its boundary moving towards it: where the face's outward normal leads
//! the motion. A face that leads all over sweeps a solid without folding: the face where it
//! starts, the face where it ends, and the sides its edges sweep. The solid swept is the start
//! united with those solids.
//!
//! Built so far: a slide. A plane leads, trails or runs along it all over, as does a cylinder slid
//! along its axis. A cylinder the slide crosses touches it along the plane through its axis
//! square to the slide's part across the axis, and a sphere along the plane through its centre
//! square to the slide: the solid is split by that plane first, and each side's faces then lead or
//! trail all over. The sweep of the whole is the union of its pieces' sweeps, so the pieces' prisms
//! are united with the start at the end, all at once. Cones, tori and splines decline.

use std::collections::{BTreeMap, BTreeSet};
use yack_base::kernel::{Error, Result};
use yack_build::build::{PartFace, Parts, param_of, parts_of, solid_of};

use yack_geom::{Circle, Curve, Cylinder, Extrusion, Line, Plane, Surface};
use yack_math::kernel::{Frame, Point3, Transform, Vec3};
use yack_topo::{Brep, FaceName};

use crate::Motion;

/// How a face stands to a slide.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stand {
    Leads,
    Trails,
    Along,
    /// It leads on one side of a plane through `origin` square to the unit `normal` and trails on
    /// the other: on the normal's side when `leads_above`.
    Contact {
        origin: Point3,
        normal: Vec3,
        leads_above: bool,
    },
}

/// A plane a piece lies wholly on one side of, as a split left it: through `origin`, square to the
/// unit `normal`, the piece on the normal's side when `above`.
#[derive(Clone, Copy, Debug)]
struct Side {
    origin: Point3,
    normal: Vec3,
    above: bool,
}

/// How many contact planes a slide may split a solid by before it declines.
const MAX_SPLITS: usize = 6;

/// A slide: by `by`, along the unit `dir`, the faces it makes named under `feature`.
struct Slide<'a> {
    by: Vec3,
    dir: Vec3,
    feature: &'a str,
}

/// [`crate::swept`] by faces: so far a slide only.
pub(crate) fn swept(body: &Brep, motion: &Motion, feature: &str) -> Result<Brep> {
    match motion {
        Motion::Slide { by } => slid(body, *by, feature),
        Motion::Turn { .. } | Motion::Screw { .. } => {
            Err(Error::Unsupported("by faces: turns and screws are not built yet".into()))
        }
    }
}

/// Everything `body` passes through as it slides by `by`.
fn slid(body: &Brep, by: Vec3, feature: &str) -> Result<Brep> {
    let length = by.norm();
    if length <= yack_tol::LINEAR {
        return Err(Error::InvalidInput(format!("a slide of {length:e} mm moves nothing")));
    }
    let slide = Slide { by, dir: by * (1.0 / length), feature };
    let mut prisms = Vec::new();
    prisms_of(body, &slide, &[], &mut prisms)?;
    let mut all: Vec<&Brep> = vec![body];
    all.extend(prisms.iter());
    yack_boolean::union_all(&all)
}

/// Adds to `prisms` what each leading face of each solid of `body` sweeps on `slide`, splitting a solid along a curved face's contact first; `sides` are
/// the planes earlier splits left `body` on one side of.
fn prisms_of(body: &Brep, slide: &Slide, sides: &[Side], prisms: &mut Vec<Brep>) -> Result<()> {
    let solids: Vec<_> = body.solid_ids().collect();
    for &solid in &solids {
        let piece =
            if solids.len() == 1 { body.clone() } else { yack_boolean::solids_of(body, &[solid]) };
        let parts = parts_of(&piece)?;
        let mut leading = Vec::new();
        let mut cut = None;
        for (k, face) in parts.faces.iter().enumerate() {
            match settled(stand(face, slide.dir)?, sides) {
                Stand::Leads => leading.push(k),
                Stand::Trails | Stand::Along => {}
                Stand::Contact { origin, normal, .. } => {
                    cut = Some((origin, normal));
                    break;
                }
            }
        }
        if let Some((origin, normal)) = cut {
            if sides.len() >= MAX_SPLITS {
                return Err(Error::Unsupported(format!(
                    "by faces: a solid with more than {MAX_SPLITS} contacts on a slide"
                )));
            }
            let (above, below) = yack_boolean::split(&piece, origin, normal)?;
            for (half, up) in [(above, true), (below, false)] {
                let mut more = sides.to_vec();
                more.push(Side { origin, normal, above: up });
                prisms_of(&half, slide, &more, prisms)?;
            }
            continue;
        }
        for k in leading {
            prisms.push(prism(&parts, k, slide)?);
        }
    }
    Ok(())
}

/// `stand`, a contact settled where an earlier split left the piece on one side of its plane.
fn settled(stand: Stand, sides: &[Side]) -> Stand {
    let Stand::Contact { origin, normal, leads_above } = stand else {
        return stand;
    };
    for side in sides {
        let same = side.normal.cross(normal).norm() <= yack_tol::ANGULAR
            && (origin - side.origin).dot(side.normal).abs() <= yack_tol::LINEAR;
        if same {
            let above = side.above == (side.normal.dot(normal) > 0.0);
            return if above == leads_above { Stand::Leads } else { Stand::Trails };
        }
    }
    stand
}

/// How `face` stands to a slide along the unit `dir`: whole, or a decline where it would have to
/// be split along its contact first.
fn stand(face: &PartFace, dir: Vec3) -> Result<Stand> {
    let out = |n: Vec3| if face.reversed { -n } else { n };
    let along = |axis: Vec3| axis.cross(dir).norm() <= yack_tol::ANGULAR;
    match &face.surface {
        Surface::Plane(p) => {
            let s = out(p.frame.z).dot(dir);
            Ok(if s > yack_tol::ANGULAR {
                Stand::Leads
            } else if s < -yack_tol::ANGULAR {
                Stand::Trails
            } else {
                Stand::Along
            })
        }
        Surface::Cylinder(c) if along(c.frame.z) => Ok(Stand::Along),
        Surface::Extrusion(e) if along(e.dir) => Ok(Stand::Along),
        // The outward normal of a cylinder's face is ±radial: square to the slide on the plane
        // through the axis square to the slide's part across the axis.
        Surface::Cylinder(c) => {
            let across = dir - c.frame.z * c.frame.z.dot(dir);
            let normal = across
                .unit()
                .ok_or_else(|| Error::Unsupported("by faces: a degenerate cylinder".into()))?;
            Ok(Stand::Contact { origin: c.frame.origin, normal, leads_above: !face.reversed })
        }
        Surface::Sphere(sp) => {
            Ok(Stand::Contact { origin: sp.frame.origin, normal: dir, leads_above: !face.reversed })
        }
        other => Err(Error::Unsupported(format!(
            "by faces: a {} face the slide crosses is not split along its contact yet",
            other.kind()
        ))),
    }
}

/// One piece of a face's boundary, along its curve from point `start` to point `end` (indices of
/// start-and-slid pairs in [`Prism::points`]).
#[derive(Clone, Debug)]
struct Piece {
    curve: Curve,
    start: usize,
    end: usize,
}

/// What a leading face sweeps, built up: each boundary point where it starts and where it ends,
/// and each piece of boundary once, closed edges cut in two so that no side face meets itself.
struct Prism<'a> {
    parts: &'a Parts,
    moved: Transform,
    out: Parts,
    /// For each point of `parts` the face uses, its index among the pairs.
    point_at: BTreeMap<usize, usize>,
    /// Each pair's index into `out.points` where it starts (the slid copy follows it), and its rail.
    pairs: Vec<(usize, usize)>,
    /// For each edge of `parts` the face uses, its pieces in its curve's direction, and each
    /// piece's edge where it starts and where it ends in `out`.
    pieces: BTreeMap<usize, Vec<(Piece, usize, usize)>>,
}

impl Prism<'_> {
    /// The pair for `p` (a new point when `of` is `None`), with its rail along `dir`.
    fn pair(&mut self, of: Option<usize>, p: Point3, dir: Vec3) -> usize {
        if let Some(at) = of.and_then(|v| self.point_at.get(&v)) {
            return *at;
        }
        let k = self.out.points.len();
        self.out.points.push(p);
        self.out.points.push(self.moved.point(p));
        let rail = self.out.edges.len();
        self.out.edges.push((Curve::Line(Line { origin: p, dir }), k, k + 1));
        self.pairs.push((k, rail));
        let at = self.pairs.len() - 1;
        if let Some(v) = of {
            self.point_at.insert(v, at);
        }
        at
    }

    /// Pair `k`: its start point's index in `out.points` and its rail's in `out.edges`.
    fn at(&self, k: usize) -> Result<(usize, usize)> {
        self.pairs.get(k).copied().ok_or_else(|| missing("pair"))
    }

    /// Edge `e`'s pieces, made on first use: the edge, or its two halves where it is closed.
    fn pieces(&mut self, e: usize, dir: Vec3) -> Result<Vec<(Piece, usize, usize)>> {
        if let Some(p) = self.pieces.get(&e) {
            return Ok(p.clone());
        }
        let (curve, a, b) = self.parts.edges.get(e).ok_or_else(|| missing("edge"))?;
        let pa = *self.parts.points.get(*a).ok_or_else(|| missing("point"))?;
        let pb = *self.parts.points.get(*b).ok_or_else(|| missing("point"))?;
        let (ka, kb) = (self.pair(Some(*a), pa, dir), self.pair(Some(*b), pb, dir));
        let mut list = vec![Piece { curve: curve.clone(), start: ka, end: kb }];
        if a == b {
            let period = curve.period().ok_or_else(|| {
                Error::Unsupported("by faces: a closed edge on a curve with no period".into())
            })?;
            let half = curve.eval(param_of(curve, pa) + period / 2.0);
            let km = self.pair(None, half, dir);
            list = vec![
                Piece { curve: curve.clone(), start: ka, end: km },
                Piece { curve: curve.clone(), start: km, end: ka },
            ];
        }
        let mut made = Vec::new();
        for piece in list {
            let ((s0, _), (t0, _)) = (self.at(piece.start)?, self.at(piece.end)?);
            let e0 = self.out.edges.len();
            self.out.edges.push((piece.curve.clone(), s0, t0));
            self.out.edges.push((piece.curve.transformed(&self.moved), s0 + 1, t0 + 1));
            made.push((piece, e0, e0 + 1));
        }
        self.pieces.insert(e, made.clone());
        Ok(made)
    }
}

fn missing(what: &str) -> Error {
    Error::InvalidInput(format!("by faces: no such {what}"))
}

/// The solid face `k` of `parts` sweeps on `slide`, the face leading all over: the face where it
/// starts (facing back), where it ends, and the side each piece of its boundary sweeps; named
/// `start`, `end` and `side` under the slide's feature.
fn prism(parts: &Parts, k: usize, slide: &Slide) -> Result<Brep> {
    let (by, dir) = (slide.by, slide.dir);
    let face = parts.faces.get(k).ok_or_else(|| missing("face"))?;
    let mut p = Prism {
        parts,
        moved: Transform::translation(by),
        out: Parts::default(),
        point_at: BTreeMap::new(),
        pairs: Vec::new(),
        pieces: BTreeMap::new(),
    };
    // A seam the face uses both ways sweeps no side: its two sides would cancel.
    let uses = |e: usize| face.loops.iter().flatten().filter(|&&(x, _)| x == e).count();
    let (mut start_loops, mut end_loops, mut sides) = (Vec::new(), Vec::new(), Vec::new());
    for lp in &face.loops {
        let (mut start_loop, mut end_loop) = (Vec::new(), Vec::new());
        for &(e, reversed) in lp {
            let mut pieces = p.pieces(e, dir)?;
            if reversed {
                pieces.reverse();
            }
            for (piece, e0, e1) in pieces {
                start_loop.push((e0, !reversed));
                end_loop.push((e1, reversed));
                if uses(e) == 1 {
                    sides.push((piece, reversed, e0, e1));
                }
            }
        }
        // The start cap runs the other way round (it faces back), so its uses go in reverse.
        start_loop.reverse();
        start_loops.push(start_loop);
        end_loops.push(end_loop);
    }
    let named = |part: &str| BTreeSet::from([FaceName::new(slide.feature, part)]);
    let mut faces = vec![
        PartFace {
            surface: face.surface.clone(),
            reversed: !face.reversed,
            loops: start_loops,
            names: named(FaceName::START),
            shell: 0,
        },
        PartFace {
            surface: face.surface.transformed(&p.moved),
            reversed: face.reversed,
            loops: end_loops,
            names: named(FaceName::END),
            shell: 0,
        },
    ];
    for (piece, reversed, e0, e1) in sides {
        // Along the loop: the piece (as the face uses it), the rail up from its end, the slid
        // piece back, the rail down to its start; the face's material lies on the loop's left.
        let (first, last) =
            if reversed { (piece.end, piece.start) } else { (piece.start, piece.end) };
        let ((_, ra), (_, rb)) = (p.at(first)?, p.at(last)?);
        let point = |k: usize| -> Result<Point3> {
            let (at, _) = p.at(k)?;
            p.out.points.get(at).copied().ok_or_else(|| missing("point"))
        };
        let ends = (point(piece.start)?, point(piece.end)?);
        let loop_ = vec![(e0, reversed), (rb, false), (e1, !reversed), (ra, true)];
        let (surface, flip) = side_surface(&piece.curve, ends, reversed, dir)?;
        faces.push(PartFace {
            surface,
            reversed: flip,
            loops: vec![loop_],
            names: named(FaceName::SIDE),
            shell: 0,
        });
    }
    p.out.faces = faces;
    solid_of(&p.out)
}

/// The surface an edge sweeps as it slides along the unit `dir`, and whether the side faces
/// against it: a plane for a straight edge, a cylinder for a circle square to the slide, an
/// extrusion otherwise. `ends` are where the piece starts and ends along its curve, `reversed`
/// whether the leading face's loop runs against the curve. A circle's surface has its seam opposite the piece's middle,
/// so no side face crosses its own seam; other closed curves decline.
fn side_surface(
    curve: &Curve,
    ends: (Point3, Point3),
    reversed: bool,
    dir: Vec3,
) -> Result<(Surface, bool)> {
    let at = if reversed { ends.1 } else { ends.0 };
    let curve = &match curve {
        Curve::Circle(c) => {
            let (t0, t1) = (param_of(curve, ends.0), param_of(curve, ends.1));
            let t1 = if t1 <= t0 { t1 + std::f64::consts::TAU } else { t1 };
            let middle = curve.eval(0.5 * (t0 + t1));
            let frame = Frame::new(c.frame.origin, c.frame.z, c.frame.origin - middle)?;
            Curve::Circle(Circle { frame, radius: c.radius })
        }
        other if other.period().is_some() => {
            return Err(Error::Unsupported(
                "by faces: the side of a closed curve other than a circle".into(),
            ));
        }
        other => other.clone(),
    };
    let (_, d) = curve.d1(param_of(curve, at));
    let travel = if reversed { -d } else { d };
    // The side's outward normal is square to the edge and the slide, away from the face's
    // material: `travel × dir`, since the face (its normal `n`, its material on the left of
    // `travel`, inward `n × travel`) leads: (travel × dir)·(n × travel) = −|travel|²(dir·n) ≤ 0.
    // Testing against the face's inward direction instead would guess on a contact edge, where
    // the face's normal is square to the slide and the product is zero.
    let Some(out) = travel.cross(dir).unit() else {
        return Err(Error::Unsupported("by faces: an edge that runs along the slide".into()));
    };
    let surface = match curve {
        Curve::Line(l) => Surface::Plane(Plane { frame: Frame::new(l.origin, out, l.dir)? }),
        Curve::Circle(c) if c.frame.z.cross(dir).norm() <= yack_tol::ANGULAR => {
            Surface::Cylinder(Cylinder { frame: c.frame, radius: c.radius })
        }
        _ => Surface::Extrusion(Box::new(Extrusion { curve: curve.clone(), dir })),
    };
    let uv = surface.project(at);
    let natural = surface.normal(uv.x, uv.y);
    Ok((surface, natural.dot(out) < 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use yack_math::kernel::Frame;

    /// Every prism a slide makes passes the checker on its own, before any union.
    fn prisms_check(body: &Brep, by: Vec3) {
        let slide = Slide { by, dir: by.unit().unwrap(), feature: "f" };
        let mut prisms = Vec::new();
        prisms_of(body, &slide, &[], &mut prisms).unwrap();
        assert!(!prisms.is_empty());
        for (i, p) in prisms.iter().enumerate() {
            let issues = yack_check::check(p);
            let kinds: Vec<_> = p.face_ids().map(|f| p.face(f).surface.kind()).collect();
            assert!(issues.is_empty(), "prism {i} {kinds:?}: {issues:?}");
        }
    }

    #[test]
    fn a_cylinders_prisms_are_valid() {
        let c = yack_build::primitive::cylinder(&Frame::WORLD, 2.0, 10.0, "c").unwrap();
        prisms_check(&c, Vec3::new(3.0, 0.0, 4.0));
    }

    #[test]
    fn a_balls_prisms_are_valid() {
        let ball = yack_build::primitive::sphere(&Frame::WORLD, 2.0, "ball").unwrap();
        prisms_check(&ball, Vec3::new(0.0, 0.0, 5.0));
    }
}
