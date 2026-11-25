// yack: a STEP solid's arena built, turned the way its volume says, and checked (from topology.rs).
//! The last step of reading a solid: build its arena, turn any shell that reads inside out, and
//! check it.
#![warn(clippy::indexing_slicing)]

use super::heal;
use super::raw::{Raw, build};
use crate::{
    Error, Result, check,
    topo::{Brep, ShellId, SolidId},
};

/// The checked arena of a solid read as `raw`, its edges over `ranges`, and its vertices and edges
/// given the local tolerances of `tolerated` where it is given.
pub fn assemble(
    raw: &Raw,
    ranges: &[(f64, f64)],
    tolerated: Option<&heal::Tolerated>,
) -> Result<Brep> {
    let mut b = build(raw, ranges, tolerated)?;
    // What no turn can mend is refused before the shells' facing is worked out (a volume each).
    let far = check::check_distances(&b);
    if !far.is_empty() {
        return Err(Error::InvalidResult(far.into_iter().map(|i| i.to_string()).collect()));
    }
    // Some systems write a void's faces facing into it and turn its shell as well, which reads
    // inside out: a shell is the way the volume it encloses says, outer positive, void negative.
    let turned = check::turned_shells(&b);
    if !turned.is_empty() {
        b = build(&turned_round(raw, &turned), ranges, tolerated)?;
    }
    let issues = check::check_structure(&b);
    if !issues.is_empty() {
        return Err(Error::InvalidResult(
            issues.into_iter().map(|i| i.to_string()).collect(),
        ));
    }
    Ok(b)
}

/// `raw` with the faces of each of `turned`'s shells facing the other way. The shells are the
/// arena's built from `raw`, so they are `raw`'s too.
fn turned_round(raw: &Raw, turned: &[(SolidId, ShellId, bool)]) -> Raw {
    let mut raw = raw.clone();
    for &(_, shell, _) in turned {
        let faces = raw.shells.get(shell.index()).cloned().unwrap_or_default();
        for f in faces {
            let Some(face) = raw.faces.get_mut(f) else { continue };
            face.reversed = !face.reversed;
            for (_, uses) in &mut face.bounds {
                uses.reverse();
                for u in uses.iter_mut() {
                    u.reversed = !u.reversed;
                }
            }
        }
    }
    raw
}
