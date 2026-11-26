// yack: step 4: each piece of face classified and kept or dropped.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only; hardened (M3 B7): checked access in place of indexing, the
// lookup-only group-class map a BTreeMap.
//! Step 4: each piece of face classified and kept or dropped.

use std::collections::BTreeMap;

use super::*;
use super::bounds::overlap;

/// A piece of the working set that is not there: a bug in the steps before.
fn missing(what: &str) -> Error {
    Error::Numerical(format!("boolean: no {what} at that index"))
}

impl<'a> Work<'a> {
    /// Step 4: each piece of face classified and kept or dropped.
    pub(super) fn select(
        &self,
        frags: Vec<Vec<Vec<Vec<Trim>>>>,
        intact: &[bool],
        op: Op,
    ) -> Result<Vec<OutFace>> {
        let solids: [Vec<(&Domain, Aabb)>; 2] = self.face_of.each_ref().map(|ids| {
            ids.iter()
                .filter_map(|&f| self.faces.get(f))
                .map(|face| (&face.dom, face.bbox))
                .collect()
        });
        let reach = solids.each_ref().map(|faces| {
            let mut bb = Aabb::EMPTY;
            for (_, fb) in faces {
                bb.union(fb);
            }
            bb
        });
        let mut kept: Vec<OutFace> = Vec::new();
        let mut kept_doms: Vec<(usize, Domain)> = Vec::new();
        let groups = self.clear_groups();
        // looked up only: its order reaches nothing
        let mut group_class: BTreeMap<usize, Class> = BTreeMap::new();
        for (fi, pieces) in frags.into_iter().enumerate() {
            let face = self.faces.get(fi).ok_or_else(|| missing("face"))?;
            let b = *self.breps.get(face.side).ok_or_else(|| missing("solid"))?;
            let names = &b.face(face.id).names;
            let other_side = 1 - face.side;
            let other_reach = reach.get(other_side).ok_or_else(|| missing("solid"))?;
            let apart = !overlap(&face.bbox, other_reach);
            let memo = intact.get(fi) == Some(&true);
            // A whole face clear of the other solid takes its group's class once a face of the
            // group has been classified.
            let group = groups.get(fi).copied().flatten().filter(|_| pieces.len() == 1);
            let known = if apart {
                Some(Class::Out)
            } else {
                group.and_then(|g| group_class.get(&g).copied())
            };
            for loops in pieces {
                // A face apart from the other solid, or clear of it, is on one side of it whole,
                // and no contact, slit or tangent meeting the checks below look for can involve
                // it: kept or dropped without a domain.
                if let Some(class) = known {
                    let Some((flip, _)) = choose(op, face.side, class) else {
                        continue;
                    };
                    let mut out = OutFace {
                        surface: face.dom.surface.clone(),
                        reversed: face.dom.reversed,
                        names: names.clone(),
                        loops,
                        memo: memo.then(|| b.shared_memo(face.id)),
                        side: Some(face.side),
                    };
                    if flip {
                        out.flip();
                    }
                    kept.push(out);
                    continue;
                }
                let dom = Domain::new(face.dom.surface.clone(), face.dom.reversed, loops);
                let class = if apart {
                    Class::Out
                } else {
                    let other = solids.get(other_side).ok_or_else(|| missing("solid"))?;
                    let uv = dom.interior_point().ok_or_else(|| {
                        Error::Numerical("a piece of a face has no interior point".into())
                    })?;
                    // Where a piece touches the other solid without crossing it (a tangent line
                    // is not a cut), a point on the contact is on both sides at once; any other
                    // point of the piece stands for it.
                    let touching = |c: Vec2| {
                        let q = dom.surface.eval(c.x, c.y);
                        other.iter().any(|(d, bb)| {
                            (0..3).all(|k| q[k] >= bb.min[k] && q[k] <= bb.max[k])
                                && d.locate(q) != Where::Outside
                        })
                    };
                    let at = if touching(uv) {
                        dom.interior_points()
                            .into_iter()
                            .find(|&c| !touching(c))
                            .unwrap_or(uv)
                    } else {
                        uv
                    };
                    let p = dom.surface.eval(at.x, at.y);
                    self.classify(fi, &dom, at, p, other)?
                };
                if let Some(g) = group
                    && !matches!(class, Class::On { .. })
                {
                    group_class.insert(g, class);
                }
                let Some((flip, extra)) = choose(op, face.side, class) else {
                    continue;
                };
                let mut all_names: BTreeSet<FaceName> = names.clone();
                if let Some(g) = extra {
                    let gf = self.faces.get(g).ok_or_else(|| missing("face"))?;
                    let gb = self.breps.get(gf.side).ok_or_else(|| missing("solid"))?;
                    all_names.extend(gb.face(gf.id).names.iter().cloned());
                }
                let mut out = OutFace {
                    surface: dom.surface.clone(),
                    reversed: dom.reversed,
                    names: all_names,
                    loops: dom.loops.clone(),
                    memo: memo.then(|| b.shared_memo(face.id)),
                    side: Some(face.side),
                };
                if flip {
                    out.flip();
                }
                kept.push(out);
                kept_doms.push((fi, dom));
            }
        }
        Ok(kept)
    }

    /// Gives each kept face a vertex loop where the result touches it at a point inside it: where
    /// another kept face meets it at an isolated point, as two balls touch, or an edge of the
    /// result runs tangent to it, as a rim touching a plane does. The point is then a vertex both
    /// sides share, which joins them into one solid. The loops' pieces are added to `geoms`.
    pub(super) fn touch_loops(&self, kept: &mut [OutFace], geoms: &mut Vec<PieceGeom>) {
        let touch: BTreeSet<usize> = self
            .touch_points
            .iter()
            .copied()
            .chain(self.met.iter().map(|m| m.0))
            .collect();
        for v in touch {
            let Some(&p) = self.pool.get(v) else {
                continue;
            };
            let bounds = |f: &OutFace| f.loops.iter().flatten().any(|t| t.v0 == v || t.v1 == v);
            let on_boundary = kept.iter().any(bounds);
            let inside: Vec<(usize, Vec2)> = kept
                .iter()
                .enumerate()
                .filter(|(_, f)| {
                    !bounds(f) && csi::signed_distance(&f.surface, p).abs() <= tol::LINEAR
                })
                .filter_map(|(i, f)| {
                    let dom = Domain::new(f.surface.clone(), f.reversed, f.loops.clone());
                    match dom.locate(p) {
                        Where::Inside(uv) => Some((i, uv)),
                        _ => None,
                    }
                })
                .collect();
            if inside.len() + usize::from(on_boundary) < 2 {
                continue;
            }
            for (i, uv) in inside {
                let Some(face) = kept.get_mut(i) else {
                    continue;
                };
                // An uncut copy's memo describes the input face's loops, which this one outgrows.
                face.memo = None;
                geoms.push(PieceGeom {
                    curve: None,
                    range: (0.0, 1.0),
                    v0: v,
                    v1: v,
                    tol: tol::LINEAR,
                });
                face.loops.push(vec![Trim {
                    piece: geoms.len() - 1,
                    forward: true,
                    curve: None,
                    pole: p,
                    t0: 0.0,
                    t1: 1.0,
                    map: UvMap::Affine {
                        origin: uv,
                        dir: Vec2::ZERO,
                    },
                    uv0: uv,
                    uv1: uv,
                    v0: v,
                    v1: v,
                }]);
            }
        }
    }

    /// Whether a piece of face `fi` lies inside the other solid, read where one of its cuts
    /// crosses the inside of a face of that solid: the piece runs into the solid when the
    /// direction into it, square to the cut along its surface, opposes that face's outward
    /// normal. `None` when no cut crosses a face cleanly, so a ray must decide.
    pub(super) fn local_class(&self, fi: usize, dom: &Domain) -> Option<bool> {
        for (li, l) in dom.loops.iter().enumerate() {
            for (k, tr) in l.iter().enumerate() {
                let Some(piece) = self.pieces.get(tr.piece) else {
                    continue;
                };
                let Some(&BaseKind::Cut {
                    fa,
                    fb,
                    tangent: false,
                }) = self.bases.get(piece.base).map(|b| &b.kind)
                else {
                    continue;
                };
                let g = if fa == fi {
                    fb
                } else if fb == fi {
                    fa
                } else {
                    continue;
                };
                let Some(c) = &tr.curve else { continue };
                let (q, dq) = c.d1(0.5 * (tr.t0 + tr.t1));
                let Some(gd) = self.faces.get(g).map(|f| &f.dom) else {
                    continue;
                };
                let Where::Inside(uvg) = gd.locate(q) else {
                    continue;
                };
                // the middle sample of the trim's polyline
                let Some(&uv) = dom.samples().get(li).and_then(|l| l.get(k)).and_then(|pl| pl.get(pl.len() / 2)) else {
                    continue;
                };
                let travel = if tr.t1 >= tr.t0 { dq } else { -dq };
                // The loops keep the material on their left, seen from outside.
                let into = dom.outward(uv).cross(travel).unit_or_zero();
                let s = into.dot(gd.outward(uvg));
                if s.abs() >= LOCAL_SINE {
                    return Some(s < 0.0);
                }
            }
        }
        None
    }

    pub(super) fn classify(
        &self,
        fi: usize,
        dom: &Domain,
        uv: Vec2,
        p: Point3,
        other: &[(&Domain, Aabb)],
    ) -> Result<Class> {
        for &(fa, fb) in &self.coincident {
            let g = if fa == fi {
                fb
            } else if fb == fi {
                fa
            } else {
                continue;
            };
            let gd = &self.faces.get(g).ok_or_else(|| missing("face"))?.dom;
            let at = match gd.locate(p) {
                // The piece is thinner than twice the tolerance here. It was cut along every
                // stretch of g's boundary that crosses it, so its interior lies wholly on one side.
                Where::Boundary => gd.exact_side(p).map_or(Where::Outside, Where::Inside),
                w => w,
            };
            if let Where::Inside(uvg) = at {
                let same = dom.outward(uv).dot(gd.outward(uvg)) > 0.0;
                return Ok(Class::On { same, partner: g });
            }
        }
        let inside = self
            .local_class(fi, dom)
            .or_else(|| classify::touching(&dom.surface, other, p))
            .or_else(|| classify::contains(other, p));
        match inside {
            Some(true) => Ok(Class::In),
            Some(false) => Ok(Class::Out),
            None => Err(Error::Numerical(
                "could not tell whether a face piece is inside the other solid".into(),
            )),
        }
    }
}

/// Where a piece of face lies relative to the other solid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Class {
    In,
    Out,
    /// On face `partner` of the other solid, with the same outward normal or the opposite one.
    On {
        same: bool,
        partner: usize,
    },
}

/// Whether the operation keeps a piece of a face of solid `side`, whether it is turned inside out,
/// and the coincident face whose names it also takes.
pub(super) fn choose(op: Op, side: usize, class: Class) -> Option<(bool, Option<usize>)> {
    let a = side == 0;
    match (op, class) {
        (Op::Union, Class::Out) | (Op::Intersection, Class::In) => Some((false, None)),
        (
            Op::Union | Op::Intersection,
            Class::On {
                same: true,
                partner,
            },
        ) if a => Some((false, Some(partner))),
        (Op::Difference, Class::Out) if a => Some((false, None)),
        (Op::Difference, Class::On { same: false, .. }) if a => Some((false, None)),
        (Op::Difference, Class::In) if !a => Some((true, None)),
        _ => None,
    }
}
