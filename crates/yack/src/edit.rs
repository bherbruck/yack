// yack: edits on `Solid` (signed distances: positive grows), routed to springrep's `edit` through
// its facade glue (M1.4). `swept` stays a stub: springrep has no swept-solid operation.
use yack_cadrs::yack_glue as glue;
use yack_edit::glue as sr;

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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // grown 1 all round: the faces moved out, the edges rounded, the corners balls
    /// let pi = std::f64::consts::PI;
    /// let grown = b.offset(1.0)?;
    /// assert!((grown.volume() - (1600.0 + 30.0 * pi + 4.0 * pi / 3.0)).abs() < 1e-6 * grown.volume());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn offset(&self, distance: f64) -> Result<Solid> {
        let op = "offset";
        if !(distance.is_finite() && distance != 0.0) {
            return Err(Error::invalid(op, "distance", format!("distance {distance} mm is not a nonzero length")));
        }
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        firewall::op(op, &[self], || {
            ladder::two(
                op,
                &|| {
                    // the rung refuses a growth that closes a hole exactly, and an inward collapse, itself
                    // (yack-edit glue::offset, OF1/OF2b); past a hole it answers, equal to cadrs
                    engine::made_from("offset", op, sr::offset(self.need(op)?, distance))
                },
                &|| engine_cadrs::op(op, self, |k, id| glue::offset(k, id, distance)),
            )
        })
    }
    /// Offsets `faces` by `distance` (positive grows), extending their neighbours.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let taller = b.offset_faces(&[b.face_at([5.0, 5.0, 10.0])?], 2.0)?;
    /// assert!((taller.volume() - 1200.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn offset_faces(&self, faces: &[FaceId], distance: f64) -> Result<Solid> {
        let op = "offset_faces";
        if faces.is_empty() {
            return Err(Error::invalid(op, "faces", "no faces to offset"));
        }
        if !(distance.is_finite() && distance != 0.0) {
            return Err(Error::invalid(op, "distance", format!("distance {distance} mm is not a nonzero length")));
        }
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        firewall::op(op, &[self], || {
            ladder::two(
                op,
                &|| {
                    let out = engine::made(engine::PROVISIONAL, op, sr::offset_faces_named(self.need(op)?, &f, distance, engine::PROVISIONAL))?;
                    if yack_topo::local::radius_collapses(self.need(op)?, Some(&f), distance, false) {
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // the top replaced by a flat Bézier patch two higher: the sides run up to it
    /// let net = vec![vec![[0.0, 0.0, 12.0].into(), [0.0, 10.0, 12.0].into()], vec![[10.0, 0.0, 12.0].into(), [10.0, 10.0, 12.0].into()]];
    /// let raised = b.replace_face(b.face_at([5.0, 5.0, 10.0])?, &Patch::Bezier { ctrl: net })?;
    /// assert!((raised.volume() - 1200.0).abs() < 1e-6 * 1200.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
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
            return firewall::op(op, &[self], || {
                ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::replace_face_fill(op, self, i, boundary) }])
            });
        }
        let (degree, [ku, kv], poles, weights) = crate::input::patch_net(op, patch)?;
        let d = degree.map(|x| usize::try_from(x).unwrap_or(usize::MAX));
        let i = u64::from(face.index());
        firewall::op(op, &[self], || {
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let drilled = b.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 12.0)?)?;
    /// let filled = drilled.delete_faces(&[drilled.face_at([3.0, 5.0, 5.0])?])?;
    /// assert!((filled.volume() - 1000.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn delete_faces(&self, faces: &[FaceId]) -> Result<Solid> {
        let op = "delete_faces";
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        firewall::op(op, &[self], || {
            ladder::two(
                op,
                &|| {
                    let b = self.need(op)?;
                    let out = engine::made(engine::PROVISIONAL, op, sr::delete_faces_named(b, &f, engine::PROVISIONAL))?;
                    if out.brep().is_some_and(|o| yack_measure::guard::same_shape(b, o)) {
                        return refuse(out, op, "the faces' gap was not closed: the result is the solid unchanged");
                    }
                    Ok(out)
                },
                &|| {
                    let out = engine_cadrs::op(op, self, |k, id| glue::delete_faces(k, id, &ix))?;
                    match (self.brep(), out.brep()) {
                        (Some(b), Some(o)) if yack_measure::guard::same_shape(b, o) => {
                            Err(Error::failed(op, crate::Stage::Validate, "cadrs declines: the result is the solid unchanged"))
                        }
                        _ => Ok(out),
                    }
                },
            )
        })
    }
    /// Moves `faces` by the rigid motion `by`, extending their neighbours.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let taller = b.move_faces(&[b.face_at([5.0, 5.0, 10.0])?], &Rigid::translation([0.0, 0.0, 2.0]))?;
    /// assert!((taller.volume() - 1200.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn move_faces(&self, faces: &[FaceId], by: &Rigid) -> Result<Solid> {
        let op = "move_faces";
        if faces.is_empty() {
            return Err(Error::invalid(op, "faces", "no faces to move"));
        }
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        firewall::op(op, &[self], || {
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let plate = b.thicken(&[b.face_at([5.0, 5.0, 10.0])?], 1.0, Join::Round)?;
    /// assert!((plate.volume() - 100.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn thicken(&self, faces: &[FaceId], thickness: f64, join: Join) -> Result<Solid> {
        let op = "thicken";
        if faces.is_empty() {
            return Err(Error::invalid(op, "faces", "no faces to thicken"));
        }
        if !(thickness.is_finite() && thickness != 0.0) {
            return Err(Error::invalid(op, "thickness", format!("thickness {thickness} mm is not a nonzero length")));
        }
        let f = self.sr_faces(op, "faces", faces)?;
        let ix = indices(faces);
        let sharp = join == Join::Sharp;
        let cadrs = || engine_cadrs::op(op, self, |k, id| glue::thicken_joined(k, id, &ix, thickness, sharp));
        if sharp {
            // springrep cannot be told how to join a crease: only cadrs mitres on request
            return firewall::op(op, &[self], || ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &cadrs }]));
        }
        firewall::op(op, &[self], || {
            ladder::two(
                op,
                &|| {
                    let out = engine::made(engine::PROVISIONAL, op, sr::thicken_named(self.need(op)?, &f, thickness, join == Join::Round, engine::PROVISIONAL))?;
                    if !yack_topo::local::rims_square(self.need(op)?, &f) {
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // open at the top, walls 1 thick inward: 1000 − 8·8·9
    /// let cup = b.shell(&[b.face_at([5.0, 5.0, 10.0])?], 1.0)?;
    /// assert!((cup.volume() - 424.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn shell(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell";
        // api.md:1051
        if !(thickness.is_finite() && thickness > 0.0) {
            return Err(Error::invalid(op, "thickness", format!("thickness {thickness} mm is not positive")));
        }
        let f = self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::op(op, &[self], || {
            ladder::two(
                op,
                &|| {
                    let b = self.need(op)?;
                    // the stated requirement (a shell's walls must not meet), checked before springrep
                    // tries to build: Invalid, so it vetoes a cadrs build too; the two checks after
                    // the build are springrep's limits, a decline
                    if yack_measure::guard::walls_cross(b, &f, thickness) {
                        return Err(Error::invalid(op, "thickness", "the walls would meet: thicker than half the part"));
                    }
                    let out = engine::made_from("shell", op, sr::shell(b, &f, thickness))?;
                    if yack_topo::local::radius_collapses(b, None, thickness, true) {
                        return refuse(out, op, "a round face of radius under the thickness would close");
                    }
                    Ok(out)
                },
                &|| engine_cadrs::shell(op, self, &ix, thickness, false, false),
            )
        })
    }
    /// Hollows the solid by a wall of `thickness` grown outside it, removing the `open` faces: the
    /// solid's faces become the wall's inside.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // open at the top, walls 1 thick outside the block: 12·12·11 − 1000
    /// let cup = b.shell_outward(&[b.face_at([5.0, 5.0, 10.0])?], 1.0)?;
    /// assert!((cup.volume() - 584.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn shell_outward(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell_outward";
        // api.md:1051
        if !(thickness.is_finite() && thickness > 0.0) {
            return Err(Error::invalid(op, "thickness", format!("thickness {thickness} mm is not positive")));
        }
        let f = self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::op(op, &[self], || {
            ladder::two(
                op,
                &|| {
                    let out = engine::made_from("shell", op, sr::shell_outward(self.need(op)?, &f, thickness))?;
                    if yack_topo::local::radius_collapses(self.need(op)?, None, thickness, false) {
                        return refuse(out, op, "a hole of radius under the thickness would close");
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// match b.shell_rounded(&[b.face_at([5.0, 5.0, 10.0])?], 1.0) {
    ///     Ok(cup) => assert!(cup.volume() > 0.0 && cup.volume() < 1000.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn shell_rounded(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell_rounded";
        if !(thickness.is_finite() && thickness > 0.0) {
            return Err(Error::invalid(op, "thickness", format!("thickness {thickness} is not positive")));
        }
        self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::op(op, &[self], || {
            ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::shell(op, self, &ix, thickness, false, true) }])
        })
    }
    /// As [`shell_outward`](Solid::shell_outward), the outside wall joined round where it parts
    /// (every point within `thickness` of the solid).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// match b.shell_outward_rounded(&[b.face_at([5.0, 5.0, 10.0])?], 1.0) {
    ///     Ok(cup) => assert!(cup.volume() > 0.0),
    ///     Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported),
    /// }
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn shell_outward_rounded(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell_outward_rounded";
        if !(thickness.is_finite() && thickness > 0.0) {
            return Err(Error::invalid(op, "thickness", format!("thickness {thickness} is not positive")));
        }
        self.sr_faces(op, "open", open)?;
        let ix = indices(open);
        firewall::op(op, &[self], || {
            ladder::ladder(op, &[ladder::Rung { name: "cadrs", run: &|| engine_cadrs::shell(op, self, &ix, thickness, true, true) }])
        })
    }
    /// Tilts `faces` by `angle` about the neutral plane through `origin` normal to `pull`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // the four sides leaned 5° about the floor's plane, pulled up: a tapered block, valid
    /// let sides = b.faces_at(&[[0.0, 5.0, 5.0], [10.0, 5.0, 5.0], [5.0, 0.0, 5.0], [5.0, 10.0, 5.0]])?;
    /// let tapered = b.draft(&sides, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(5.0))?;
    /// assert!(tapered.check().is_empty() && tapered.volume() != 1000.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn draft(&self, faces: &[FaceId], origin: impl Into<Point3>, pull: impl Into<Vec3>, angle: Angle) -> Result<Solid> {
        let op = "draft";
        if faces.is_empty() {
            return Err(Error::invalid(op, "faces", "no faces to draft"));
        }
        if !(angle.radians() > 0.0 && angle.radians() < core::f64::consts::FRAC_PI_2) {
            return Err(Error::invalid(op, "angle", format!("{}° is not between 0° and 90°", angle.degrees())));
        }
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
        firewall::op(op, &[self], || {
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
    ///
    /// Faces swept by one of the solid's faces (its first and last positions, and the surface it
    /// sweeps) keep that face's names; sides swept by its edges are new faces of this operation,
    /// with no role. A face made from pieces of several input faces keeps the lowest-numbered
    /// one's names. Where cadrs does not follow the motion's faces (a curved path, or a sweep too
    /// costly to trace), every face carries only this operation's name.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a ball of radius 1 slid 5: a capsule, π·5 + 4π/3
    /// let pi = std::f64::consts::PI;
    /// let capsule = sphere(&Frame::WORLD, 1.0)?.swept(&Motion::slide([5.0, 0.0, 0.0]))?;
    /// assert!((capsule.volume() - (5.0 * pi + 4.0 * pi / 3.0)).abs() < 1e-6 * capsule.volume());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn swept(&self, motion: &Motion) -> Result<Solid> {
        let op = "swept";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        firewall::op(op, &[self], || self.swept_unrecorded(motion))
    }

    /// [`swept`](Solid::swept) without its history, for an operation that calls it on the way.
    pub(crate) fn swept_unrecorded(&self, motion: &Motion) -> Result<Solid> {
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        // yack-swept's native rung after cadrs's, so cadrs's answer stands until a bake-off promotes it
        ladder::ladder(
            "swept",
            &[
                ladder::Rung { name: "cadrs", run: &|| engine_cadrs::swept(self, motion) },
                ladder::Rung { name: "native", run: &|| crate::engine_swept::swept(self, motion) },
            ],
        )
    }

    /// The image under a similarity (rigid, uniform scale, mirror): a [`Similarity`], or a
    /// [`Rigid`](crate::Rigid) motion as it is.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let turned = b.transform(Rigid::rotation([0.0; 3], [0.0, 0.0, 1.0], deg(90.0))?)?;
    /// assert!((turned.volume() - 1000.0).abs() < 1e-9);
    /// let mirrored = b.transform(&Similarity::mirror([0.0; 3], [1.0, 0.0, 0.0])?)?;
    /// assert!((mirrored.volume() - 1000.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn transform(&self, m: impl Into<Similarity>) -> Result<Solid> {
        let op = "transform";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        firewall::moved(op, self, || self.transform_unrecorded(m.into()))
    }

    /// [`transform`](Solid::transform) without its history, for an operation that calls it on the
    /// way.
    pub(crate) fn transform_unrecorded(&self, m: Similarity) -> Result<Solid> {
        let op = "transform";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        let rows = m.rows();
        ladder::two(op, &|| engine::solid(op, sr::transform(self.need(op)?, rows)), &|| engine_cadrs::op(op, self, |k, id| glue::transform(k, id, rows)))
    }

    /// The image under an invertible affine map (protocol `affine`).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let stretched = b.transform_affine(&Affine::from_rows([[2.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])?)?;
    /// assert!((stretched.volume() - 2000.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn transform_affine(&self, m: &Affine) -> Result<Solid> {
        let op = "transform_affine";
        if self.is_empty() {
            return Ok(Solid::empty());
        }
        let rows = m.rows();
        firewall::moved(op, self, || {
            ladder::two(op, &|| engine::solid(op, sr::affine(self.need(op)?, rows)), &|| engine_cadrs::op(op, self, |k, id| glue::transform(k, id, rows)))
        })
    }
}
