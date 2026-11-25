// yack: booleans and plane splits on `Solid`, routed to springrep (PLAN §4.3, M1.3).
use yack_boolean as sr;

use crate::engine::{self, Brep};
use crate::{engine_cadrs, firewall, ladder};
use yack_cadrs::BoolOp;
use crate::{Plane, Result, Solid, Split};

impl Solid {
    /// Everything in either solid. To join many solids in one operation, [`Solid::union_all`].
    ///
    /// ```
    /// use yack::prelude::*;
    /// let a = block(&Frame::WORLD, [2.0, 2.0, 2.0])?;
    /// let b = block(&Frame::at([1.0, 0.0, 0.0]), [2.0, 2.0, 2.0])?;
    /// assert!((a.union(&b)?.volume() - 12.0).abs() < 0.01);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn union(&self, other: &Solid) -> Result<Solid> {
        firewall::op("union", &[self, other], || {
            match (self.is_empty(), other.is_empty()) {
                (true, _) => Ok(other.clone()),
                (_, true) => Ok(self.clone()),
                _ => ladder::two(
                    "union",
                    &|| engine::solid("union", sr::union(self.need("union")?, other.need("union")?)),
                    &|| engine_cadrs::boolean("union", BoolOp::Union, self, &[other]),
                ),
            }
        })
    }

    /// Everything in both solids.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let a = block(&Frame::WORLD, [2.0, 2.0, 2.0])?;
    /// let b = block(&Frame::at([1.0, 0.0, 0.0]), [2.0, 2.0, 2.0])?;
    /// assert!((a.intersection(&b)?.volume() - 4.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn intersection(&self, other: &Solid) -> Result<Solid> {
        firewall::op("intersection", &[self, other], || {
            match (self.is_empty(), other.is_empty()) {
                (false, false) => ladder::two(
                    "intersection",
                    &|| {
                        let (a, b) = (self.need("intersection")?, other.need("intersection")?);
                        let out = engine::solid("intersection", sr::intersection(a, b))?;
                        if out.is_empty() && yack_measure::guard::boxes_overlap(a, b) {
                            return Ok(out.suspect("an empty intersection of solids whose boxes overlap (a sliver it may have lost)"));
                        }
                        Ok(out)
                    },
                    &|| engine_cadrs::boolean("intersection", BoolOp::Intersect, self, &[other]),
                ),
                _ => Ok(Solid::empty()),
            }
        })
    }

    /// This solid less `tool`. To cut many tools in one operation, [`Solid::difference_all`].
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let drilled = b.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 12.0)?)?;
    /// assert!((drilled.volume() - (1000.0 - 40.0 * std::f64::consts::PI)).abs() < 1e-9 * 1000.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn difference(&self, tool: &Solid) -> Result<Solid> {
        firewall::op("difference", &[self, tool], || self.difference_unrecorded(tool))
    }

    /// [`difference`](Solid::difference) without its history, for an operation that calls it on
    /// the way to its own answer (whose history it then gives).
    fn difference_unrecorded(&self, tool: &Solid) -> Result<Solid> {
        match (self.is_empty(), tool.is_empty()) {
            (true, _) => Ok(Solid::empty()),
            (_, true) => Ok(self.clone()),
            _ => ladder::two(
                "difference",
                &|| engine::solid("difference", sr::difference(self.need("difference")?, tool.need("difference")?)),
                &|| engine_cadrs::boolean("difference", BoolOp::Subtract, self, &[tool]),
            ),
        }
    }

    /// The union of all `solids` (empty for none).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let cubes: Vec<Solid> = (0..3).map(|i| block(&Frame::at([3.0 * f64::from(i), 0.0, 0.0]), [1.0, 1.0, 1.0])).collect::<Result<_>>()?;
    /// assert!((Solid::union_all(&cubes)?.volume() - 3.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn union_all(solids: &[Solid]) -> Result<Solid> {
        let inputs: Vec<&Solid> = solids.iter().collect();
        firewall::op("union_all", &inputs, || {
            let parts: Vec<&Solid> = solids.iter().filter(|s| !s.is_empty()).collect();
            match parts.as_slice() {
                [] => Ok(Solid::empty()),
                [one] => Ok((*one).clone()),
                _ => {
                    let [first, rest @ ..] = parts.as_slice() else { return Ok(Solid::empty()) };
                    ladder::two(
                        "union_all",
                        &|| {
                            let breps = parts.iter().map(|s| s.need("union_all")).collect::<Result<Vec<&Brep>>>()?;
                            engine::solid("union_all", sr::union_all(&breps))
                        },
                        &|| engine_cadrs::boolean("union_all", BoolOp::Union, first, rest),
                    )
                }
            }
        })
    }

    /// This solid less every tool.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let holes = [cylinder(&Frame::at([3.0, 5.0, -1.0]), 1.0, 12.0)?, cylinder(&Frame::at([7.0, 5.0, -1.0]), 1.0, 12.0)?];
    /// let plate = b.difference_all(&holes)?;
    /// assert!((plate.volume() - (1000.0 - 20.0 * std::f64::consts::PI)).abs() < 1e-9 * 1000.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn difference_all(&self, tools: &[Solid]) -> Result<Solid> {
        let inputs: Vec<&Solid> = core::iter::once(self).chain(tools).collect();
        firewall::op("difference_all", &inputs, || {
            if self.is_empty() {
                return Ok(Solid::empty());
            }
            let tools: Vec<&Solid> = tools.iter().filter(|s| !s.is_empty()).collect();
            if tools.is_empty() {
                return Ok(self.clone());
            }
            ladder::two(
                "difference_all",
                &|| {
                    let breps = tools.iter().map(|s| s.need("difference_all")).collect::<Result<Vec<&Brep>>>()?;
                    engine::solid("difference_all", sr::difference_all(self.need("difference_all")?, &breps))
                },
                &|| engine_cadrs::boolean("difference_all", BoolOp::Subtract, self, &tools),
            )
        })
    }

    /// This solid less every point `tool` passes through along `motion`: exactly
    /// `self.difference(&tool.swept(motion)?)`, computed directly where cadrs can (an exact cut
    /// where the swept tool alone would come out faceted, a long screw trimmed to the reach of
    /// this solid), else as that difference.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // a 1 mm-radius end mill 1 deep into the top, slid 6 along x: a slot of area 12 + π
    /// let mill = cylinder(&Frame::at([2.0, 5.0, 9.0]), 1.0, 3.0)?;
    /// let slotted = b.cut_swept(&mill, &Motion::slide([6.0, 0.0, 0.0]))?;
    /// assert!((slotted.volume() - (1000.0 - 12.0 - std::f64::consts::PI)).abs() < 1e-6 * 1000.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn cut_swept(&self, tool: &Solid, motion: &crate::Motion) -> Result<Solid> {
        let op = "cut_swept";
        if self.is_empty() || tool.is_empty() {
            return Ok(self.clone());
        }
        firewall::op(op, &[self, tool], || {
            ladder::ladder(
                op,
                &[
                    ladder::Rung { name: "cadrs", run: &|| engine_cadrs::cut_swept(self, tool, motion) },
                    // swept is cadrs-only, so the fallback needs cadrs too
                    ladder::Rung { name: "cadrs", run: &|| self.difference_unrecorded(&tool.swept_unrecorded(motion)?) },
                ],
            )
        })
    }

    /// The parts on each side of `plane`; `front` is on the normal's side. Either may be empty.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let halves = b.split(&Plane::new([4.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?;
    /// assert!((halves.front.volume() - 600.0).abs() < 1e-9 && (halves.back.volume() - 400.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn split(&self, plane: &Plane) -> Result<Split> {
        let op = "split";
        let Some(b) = self.brep() else { return Ok(Split { front: Solid::empty(), back: Solid::empty() }) };
        let (o, n) = (plane.origin(), plane.normal().vec());
        let springrep = || -> Result<(Split, bool)> {
            ladder::allowed(op, "springrep")?;
            let (front, back) = sr::split(b, engine::p3(o), engine::v3(n)).map_err(|e| engine::error(op, e))?;
            let (front, back) = (Solid::from_springrep(front), Solid::from_springrep(back));
            let doubted = (front.is_empty() || back.is_empty()) && yack_measure::guard::plane_cuts(b, engine::p3(o), engine::v3(n));
            Ok((Split { front, back }, doubted))
        };
        let cadrs = || -> Result<Split> {
            ladder::allowed(op, "cadrs")?;
            let (front, back) = engine_cadrs::split(self, o.to_array(), n.to_array())?;
            Ok(Split { front: front.with_strategy("cadrs"), back: back.with_strategy("cadrs") })
        };
        // A springrep split that lost a side to a sliver (brep-bench tangent/wedge-a-millionth)
        // goes to cadrs; a hard guard: it is never the answer.
        match firewall::run(op, springrep) {
            Ok((s, false)) => Ok(s),
            Ok((_, true)) => firewall::run(op, cadrs),
            Err(e) if matches!(e.kind, crate::ErrorKind::Invalid | crate::ErrorKind::Internal) => Err(e),
            Err(first) => firewall::run(op, cadrs).map_err(|second| crate::Error { detail: format!("springrep: {first}; cadrs: {second}"), ..second }),
        }
    }
}
