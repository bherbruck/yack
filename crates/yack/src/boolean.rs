// yack: booleans and plane splits on `Solid`, routed to springrep (PLAN §4.3, M1.3).
use yack_springrep::boolean as sr;

use crate::engine::{self, Brep};
use crate::firewall;
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
                (Some(a), Some(b)) => engine::solid("union", sr::union(a, b)),
            }
        })
    }

    /// Everything in both solids.
    pub fn intersection(&self, other: &Solid) -> Result<Solid> {
        firewall::run("intersection", || {
            match (self.brep(), other.brep()) {
                (Some(a), Some(b)) => engine::solid("intersection", sr::intersection(a, b)),
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
                (Some(a), Some(b)) => engine::solid("difference", sr::difference(a, b)),
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
                    engine::solid("union_all", sr::union_all(&breps))
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
            engine::solid("difference_all", sr::difference_all(a, &breps))
        })
    }

    /// The parts on each side of `plane`; `front` is on the normal's side. Either may be empty.
    pub fn split(&self, plane: &Plane) -> Result<Split> {
        firewall::run("split", || {
            let Some(b) = self.brep() else { return Ok(Split { front: Solid::empty(), back: Solid::empty() }) };
            let (front, back) = sr::split(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()))
                .map_err(|e| engine::error("split", e))?;
            Ok(Split { front: Solid::from_springrep(front), back: Solid::from_springrep(back) })
        })
    }
}
