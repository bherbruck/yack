// yack: edits on `Solid` (signed distances: positive grows), routed to springrep's `edit` through
// its facade glue (M1.4). `swept` stays a stub: springrep has no swept-solid operation.
use yack_cadrs::yack_glue as glue;
use yack_springrep::facade::{api, edit as sr};

use crate::{engine_cadrs, firewall, ladder};
use crate::solid::engine_ids;
use crate::{Affine, Angle, Error, FaceId, Join, Motion, Point3, Result, Rigid, Similarity, Solid, Vec3, engine};

/// A springrep result refused because springrep would build it wrong (M1.10 guards); the ladder
/// falls through past it.
/// A soft guard: springrep's result is doubted, so the ladder tries the next rung, and keeps
/// this answer if that fails (springrep is right on most such inputs: holes closing past their
/// radius in an offset).
fn guard(out: Solid, why: &str) -> Result<Solid> {
    Ok(out.suspect(why))
}

/// A hard guard: springrep is known to build this input wrong (each one a brep-bench case it
/// answered wrong), so its result is refused and the ladder falls through. A wrong answer is
/// never returned, even when no later rung builds one.
fn refuse(_out: Solid, op: &'static str, why: &str) -> Result<Solid> {
    Err(Error::failed(op, crate::Stage::Validate, format!("springrep declines: {why}")))
}

/// The engine indices of ids already checked to belong to the solid (the bridge keeps indices).
pub(crate) fn indices<I: Copy + Into<u64>>(ids: &[I]) -> Vec<u64> {
    ids.iter().map(|&i| i.into()).collect()
}

impl Solid {
    /// The engine faces for `ids`, each checked to belong to this solid.
    fn sr_faces(&self, op: &'static str, arg: &'static str, ids: &[FaceId]) -> Result<Vec<engine_ids::Face>> {
        ids.iter().map(|&f| self.sr_face(op, arg, f)).collect()
    }

    /// Offsets every face by `distance` (positive grows).
    pub fn offset(&self, distance: f64) -> Result<Solid> {
        let op = "offset";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        firewall::run(op, || {
            ladder::two(
                op,
                &|| {
                    let b = self.need(op)?;
                    let out = engine::made_from("offset", op, sr::offset(b, distance))?;
                    if api::radius_collapses(b, None, distance, false) {
                        return guard(out, "a round face would collapse at this distance (a hole closing, a rod vanishing)");
                    }
                    Ok(out)
                },
                &|| engine_cadrs::op(op, self, |k, id| glue::offset(k, id, distance)),
            )
        })
    }
    /// Offsets `faces` by `distance` (positive grows), extending their neighbours.
    pub fn offset_faces(&self, faces: &[FaceId], distance: f64) -> Result<Solid> {
        let op = "offset_faces";
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        firewall::run(op, || {
            ladder::two(
                op,
                &|| {
                    let out = engine::made(engine::PROVISIONAL, op, sr::offset_faces_named(self.need(op)?, &f, distance, engine::PROVISIONAL))?;
                    if api::radius_collapses(self.need(op)?, Some(&f), distance, false) {
                        return guard(out, "a round face would collapse at this distance");
                    }
                    Ok(out)
                },
                &|| engine_cadrs::op(op, self, |k, id| glue::offset_faces(k, id, &ix, distance)),
            )
        })
    }
    /// The solid with `face`'s surface replaced by `patch`, its neighbours re-trimmed to it: a
    /// Bézier or B-spline net over its whole domain, or the fill of a closed chain of curves
    /// ([`Patch::Fill`](crate::Patch::Fill)); `Invalid` for a planar fill.
    pub fn replace_face(&self, face: FaceId, patch: &crate::Patch) -> Result<Solid> {
        let op = "replace_face";
        if face.stamp() != self.stamp() {
            return Err(Error::foreign_id(op, "face"));
        }
        if !self.faces().contains(&face) {
            return Err(Error::not_found(op, "face", format!("no {face}")));
        }
        if let crate::Patch::Fill { boundary } = patch {
            let i = u64::from(face.index());
            return firewall::run(op, || {
                ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::replace_face_fill(op, self, i, boundary) }])
            });
        }
        let (degree, [ku, kv], poles, weights) = crate::input::patch_net(op, patch)?;
        let d = degree.map(|x| usize::try_from(x).unwrap_or(usize::MAX));
        let i = u64::from(face.index());
        firewall::run(op, || {
            ladder::ladder(
                op,
                &[ladder::Rung {
                    name: "cadrs",
                    run: &|| engine_cadrs::op(op, self, |k, id| glue::replace_face(k, id, i, d, &ku, &kv, &poles, weights.as_deref())),
                }],
            )
        })
    }

    /// Removes `faces` and heals the gap from the neighbours.
    pub fn delete_faces(&self, faces: &[FaceId]) -> Result<Solid> {
        let op = "delete_faces";
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        firewall::run(op, || {
            ladder::two(
                op,
                &|| {
                    let b = self.need(op)?;
                    let out = engine::made(engine::PROVISIONAL, op, sr::delete_faces_named(b, &f, engine::PROVISIONAL))?;
                    if out.brep().is_some_and(|o| api::same_shape(b, o)) {
                        return refuse(out, op, "the faces' gap was not closed: the result is the solid unchanged");
                    }
                    Ok(out)
                },
                &|| {
                    let out = engine_cadrs::op(op, self, |k, id| glue::delete_faces(k, id, &ix))?;
                    match (self.brep(), out.brep()) {
                        (Some(b), Some(o)) if api::same_shape(b, o) => {
                            Err(Error::failed(op, crate::Stage::Validate, "cadrs declines: the result is the solid unchanged"))
                        }
                        _ => Ok(out),
                    }
                },
            )
        })
    }
    /// Moves `faces` by the rigid motion `by`, extending their neighbours.
    pub fn move_faces(&self, faces: &[FaceId], by: &Rigid) -> Result<Solid> {
        let op = "move_faces";
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        firewall::run(op, || {
            ladder::two(
                op,
                &|| engine::made(engine::PROVISIONAL, op, sr::move_faces_named(self.need(op)?, &f, by.rows(), engine::PROVISIONAL)),
                &|| engine_cadrs::op(op, self, |k, id| glue::move_faces(k, id, &ix, by.rows())),
            )
        })
    }
    /// The sheet `faces` (one connected piece of this solid's boundary) thickened by `thickness`
    /// along its outward normal (negative: inward) into a new solid, the sheet alone. Where two of
    /// the faces part at a crease, `join` closes the gap; the sheet's free rims are swept square.
    pub fn thicken(&self, faces: &[FaceId], thickness: f64, join: Join) -> Result<Solid> {
        let op = "thicken";
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        let sharp = join == Join::Sharp;
        let cadrs = || engine_cadrs::op(op, self, |k, id| glue::thicken_joined(k, id, &ix, thickness, sharp));
        if sharp {
            // springrep cannot be told how to join a crease: only cadrs mitres on request
            return firewall::run(op, || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &cadrs }]));
        }
        firewall::run(op, || {
            ladder::two(
                op,
                &|| {
                    let out = engine::made(engine::PROVISIONAL, op, sr::thicken_named(self.need(op)?, &f, thickness, engine::PROVISIONAL))?;
                    if !api::rims_square(self.need(op)?, &f) {
                        return refuse(out, op, "a rim of the sheet meets its neighbour at a slant; springrep would sweep it along the neighbour, not the sheet's normal",
                        );
                    }
                    Ok(out)
                },
                &cadrs,
            )
        })
    }
    /// Hollows the solid to a wall of `thickness`, removing the `open` faces.
    pub fn shell(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell";
        let f = self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::run(op, || {
            ladder::two(
                op,
                &|| {
                    let b = self.need(op)?;
                    let out = engine::made_from("shell", op, sr::shell(b, &f, thickness))?;
                    if api::radius_collapses(b, None, thickness, true) || api::walls_cross(b, &f, thickness) || api::open_at_concave_edge(b, &f) {
                        return refuse(out, op, "the moved walls would cross (a wall thinner than twice the thickness, or a round face of radius under it)",
                        );
                    }
                    Ok(out)
                },
                &|| engine_cadrs::shell(op, self, &ix, thickness, false, false),
            )
        })
    }
    /// Hollows the solid by a wall of `thickness` grown outside it, removing the `open` faces: the
    /// solid's faces become the wall's inside.
    pub fn shell_outward(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell_outward";
        let f = self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::run(op, || {
            ladder::two(
                op,
                &|| {
                    let out = engine::made_from("shell", op, sr::shell_outward(self.need(op)?, &f, thickness))?;
                    if api::radius_collapses(self.need(op)?, None, thickness, false) || api::open_at_concave_edge(self.need(op)?, &f) {
                        return refuse(out, op, "a hole of radius under the thickness would close, or an open face meets a concave edge");
                    }
                    Ok(out)
                },
                &|| engine_cadrs::shell(op, self, &ix, thickness, true, false),
            )
        })
    }
    /// Hollows the solid to a wall of `thickness`, removing the `open` faces, its walls joined
    /// round where they part (every point within `thickness` of the solid's inside surface is
    /// wall); [`shell`](Solid::shell) joins them sharp.
    pub fn shell_rounded(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell_rounded";
        if !(thickness.is_finite() && thickness > 0.0) {
            return Err(Error::invalid(op, "thickness", format!("thickness {thickness} is not positive")));
        }
        self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::run(op, || {
            ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::shell(op, self, &ix, thickness, false, true) }])
        })
    }
    /// As [`shell_outward`](Solid::shell_outward), the outside wall joined round where it parts
    /// (every point within `thickness` of the solid).
    pub fn shell_outward_rounded(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell_outward_rounded";
        if !(thickness.is_finite() && thickness > 0.0) {
            return Err(Error::invalid(op, "thickness", format!("thickness {thickness} is not positive")));
        }
        self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::run(op, || {
            ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::shell(op, self, &ix, thickness, true, true) }])
        })
    }
    /// Tilts `faces` by `angle` about the neutral plane through `origin` normal to `pull`.
    pub fn draft(&self, faces: &[FaceId], origin: impl Into<Point3>, pull: impl Into<Vec3>, angle: Angle) -> Result<Solid> {
        let op = "draft";
        let (origin, pull) = (origin.into(), pull.into());
        if !origin.is_finite() {
            return Err(Error::invalid(op, "origin", "not finite"));
        }
        if !pull.is_finite() {
            return Err(Error::invalid(op, "pull", "not finite"));
        }
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        let (o, p, a) = (origin.to_array(), pull.to_array(), angle.radians());
        firewall::run(op, || {
            ladder::two(
                op,
                &|| engine::made(engine::PROVISIONAL, op, sr::draft_named(self.need(op)?, &f, o, p, a, engine::PROVISIONAL)),
                &|| engine_cadrs::op(op, self, |k, id| glue::draft(k, id, &ix, o, p, a)),
            )
        })
    }
    /// Every point the solid passes through during `motion` (cadrs's sweep of a solid; springrep
    /// has none). A `Fixed` path sweep is taken for a ball only, a `RotationMinimising` one along a
    /// planar path (or a ball / a body of revolution about the path's normal for `Fixed`); a `then`
    /// sequence leg by leg; legs `together` as cadrs's rigid motion ([`Solid::is_exact`] false when
    /// faceted); `Toward` is not supported yet.
    pub fn swept(&self, motion: &Motion) -> Result<Solid> {
        let op = "swept";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        firewall::run(op, || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::swept(self, motion) }]))
    }

    /// The image under a similarity (rigid, uniform scale, mirror).
    pub fn transform(&self, m: &Similarity) -> Result<Solid> {
        let op = "transform";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        let rows = m.rows();
        firewall::run(op, || {
            ladder::two(op, &|| engine::solid(op, sr::transform(self.need(op)?, rows)), &|| engine_cadrs::op(op, self, |k, id| glue::transform(k, id, rows)))
        })
    }

    /// The image under an invertible affine map (protocol `affine`).
    pub fn transform_affine(&self, m: &Affine) -> Result<Solid> {
        let op = "transform_affine";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        let rows = m.rows();
        firewall::run(op, || {
            ladder::two(op, &|| engine::solid(op, sr::affine(self.need(op)?, rows)), &|| engine_cadrs::op(op, self, |k, id| glue::transform(k, id, rows)))
        })
    }
}
