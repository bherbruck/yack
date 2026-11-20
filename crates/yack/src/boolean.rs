// yack: booleans and plane splits on `Solid`, routed to springrep (PLAN §4.3, M1.3).
use yack_springrep::boolean as sr;

use crate::engine::{self, Brep};
use crate::{engine_cadrs, firewall, ladder};
use yack_cadrs::BoolOp;
use crate::{Plane, Result, Solid, Split};

impl Solid {
    /// Everything in either solid.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let a = block(&Frame::WORLD, [2.0, 2.0, 2.0])?;
    /// let b = block(&Frame::at([1.0, 0.0, 0.0]), [2.0, 2.0, 2.0])?;
    /// assert!((a.union(&b)?.volume() - 12.0).abs() < 0.01);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn union(&self, other: &Solid) -> Result<Solid> {
        firewall::run("union", || {
            match (self.brep(), other.brep()) {
                (None, _) => Ok(other.clone()),
                (_, None) => Ok(self.clone()),
                (Some(a), Some(b)) => ladder::two(
                    "union",
                    &|| engine::solid("union", sr::union(a, b)),
                    &|| engine_cadrs::boolean("union", BoolOp::Union, self, &[other]),
                ),
            }
        })
    }

    /// Everything in both solids.
    pub fn intersection(&self, other: &Solid) -> Result<Solid> {
        firewall::run("intersection", || {
            match (self.brep(), other.brep()) {
                (Some(a), Some(b)) => ladder::two(
                    "intersection",
                    &|| {
                        let out = engine::solid("intersection", sr::intersection(a, b))?;
                        if out.is_empty() && yack_springrep::facade::api::boxes_overlap(a, b) {
                            return Err(crate::Error::failed(
                                "intersection",
                                crate::Stage::Classify,
                                "springrep declines: an empty intersection of solids whose boxes overlap (a sliver it may have lost)",
                            ));
                        }
                        Ok(out)
                    },
                    &|| engine_cadrs::boolean("intersection", BoolOp::Intersect, self, &[other]),
                ),
                _ => Ok(Solid::empty()),
            }
        })
    }

    /// This solid less `tool`.
    pub fn difference(&self, tool: &Solid) -> Result<Solid> {
        firewall::run("difference", || {
            match (self.brep(), tool.brep()) {
                (None, _) => Ok(Solid::empty()),
                (_, None) => Ok(self.clone()),
                (Some(a), Some(b)) => ladder::two(
                    "difference",
                    &|| engine::solid("difference", sr::difference(a, b)),
                    &|| engine_cadrs::boolean("difference", BoolOp::Subtract, self, &[tool]),
                ),
            }
        })
    }

    /// The union of all `solids` (empty for none).
    pub fn union_all(solids: &[Solid]) -> Result<Solid> {
        firewall::run("union_all", || {
            let parts: Vec<&Solid> = solids.iter().filter(|s| !s.is_empty()).collect();
            match parts.as_slice() {
                [] => Ok(Solid::empty()),
                [one] => Ok((*one).clone()),
                _ => {
                    let breps: Vec<&Brep> = parts.iter().filter_map(|s| s.brep()).collect();
                    let [first, rest @ ..] = parts.as_slice() else { return Ok(Solid::empty()) };
                    ladder::two(
                        "union_all",
                        &|| engine::solid("union_all", sr::union_all(&breps)),
                        &|| engine_cadrs::boolean("union_all", BoolOp::Union, first, rest),
                    )
                }
            }
        })
    }

    /// This solid less every tool.
    pub fn difference_all(&self, tools: &[Solid]) -> Result<Solid> {
        firewall::run("difference_all", || {
            let Some(a) = self.brep() else { return Ok(Solid::empty()) };
            let breps: Vec<&Brep> = tools.iter().filter_map(|s| s.brep()).collect();
            if breps.is_empty() {
                return Ok(self.clone());
            }
            let tools: Vec<&Solid> = tools.iter().filter(|s| !s.is_empty()).collect();
            ladder::two(
                "difference_all",
                &|| engine::solid("difference_all", sr::difference_all(a, &breps)),
                &|| engine_cadrs::boolean("difference_all", BoolOp::Subtract, self, &tools),
            )
        })
    }

    /// The parts on each side of `plane`; `front` is on the normal's side. Either may be empty.
    pub fn split(&self, plane: &Plane) -> Result<Split> {
        ladder::allowed("split", "springrep")?;
        firewall::run("split", || {
            let Some(b) = self.brep() else { return Ok(Split { front: Solid::empty(), back: Solid::empty() }) };
            let (front, back) = sr::split(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()))
                .map_err(|e| engine::error("split", e))?;
            let (front, back) = (Solid::from_springrep(front), Solid::from_springrep(back));
            if (front.is_empty() || back.is_empty())
                && yack_springrep::facade::api::plane_cuts(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()))
            {
                return Err(crate::Error::failed(
                    "split",
                    crate::Stage::Split,
                    "springrep lost one side of a plane that passes through the solid (a sliver it snapped away)",
                ));
            }
            Ok(Split { front, back })
        })
    }
}
