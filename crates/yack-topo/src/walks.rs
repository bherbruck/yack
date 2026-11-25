// yack: walks of the arena: sample parameters along a coedge, and the fans of faces at each vertex.
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from check.rs to topo/walks.rs (M2.2, T9); paths only; then to yack-topo (M3 W4), paths and visibility only;
// M3 hardening: the fans are a BTreeMap (D26), walked by iterators without indexing; the dense
// parameter runs the arena's sampling shares live here.
//! Walks of the arena that the checker, measures, booleans and the STEP writer share: evenly
//! spaced parameters along a coedge, and the fans of faces round each vertex.
#![warn(clippy::indexing_slicing)]

use std::collections::{BTreeMap, BTreeSet};

use super::{Brep, CoedgeId, VertexId};

/// Edge parameters along a coedge, in its own direction.
pub fn coedge_params(b: &Brep, c: CoedgeId, n: usize) -> Vec<f64> {
    let coedge = b.coedge(c);
    let (t0, t1) = b.edge(coedge.edge).range;
    let (from, to) = if coedge.reversed { (t1, t0) } else { (t0, t1) };
    spaced(from, to, n)
}

/// `n + 1` parameters from `from` to `to`, evenly spaced, the last exactly `to`: an end an ulp
/// inside the edge would be taken for an interior point.
fn spaced(from: f64, to: f64, n: usize) -> Vec<f64> {
    (0..=n).map(|i| if i == n { to } else { from + (to - from) * i as f64 / n as f64 }).collect()
}

/// `ts` with `per - 1` evenly spaced parameters added in each gap: `per` to a gap, then the last.
/// The parameter asked for at `ts[i]` is the dense one at `i * per`.
pub(crate) fn densified(ts: &[f64], per: usize) -> Vec<f64> {
    let gaps = ts.iter().zip(ts.iter().skip(1));
    let mut dense = Vec::with_capacity(ts.len().saturating_sub(1) * per + 1);
    for (&from, &to) in gaps {
        dense.extend((0..per).map(|k| from + (to - from) * k as f64 / per as f64));
    }
    dense.extend(ts.last());
    dense
}

/// A corner of a loop at a vertex: the coedge arriving there and the next one in its loop,
/// leaving.
type Corner = (CoedgeId, CoedgeId);

/// Each vertex's fans of faces, each as the coedges arriving at the vertex round it. A degenerate
/// coedge, a pole or a vertex loop, leaves the vertex where it arrived, so its face's corners on
/// either side of it are one corner.
pub fn fans_at_vertices(b: &Brep) -> BTreeMap<VertexId, Vec<Vec<CoedgeId>>> {
    let mut corners: BTreeMap<VertexId, Vec<Corner>> = BTreeMap::new();
    for lp in b.loop_ids() {
        let coedges = &b.lp(lp).coedges;
        for (&arriving, &leaving) in coedges.iter().zip(coedges.iter().cycle().skip(1)) {
            let at = b.coedge_end(arriving);
            if b.coedge_start(leaving) == at {
                corners.entry(at).or_default().push((arriving, leaving));
            }
        }
    }
    corners.into_iter().map(|(at, corners)| (at, fans(b, at, &corners))).collect()
}

/// The fans of faces round vertex `at`, from its `corners`: each fan the coedges arriving at it,
/// one corner to the next across the edge it leaves by.
fn fans(b: &Brep, at: VertexId, corners: &[Corner]) -> Vec<Vec<CoedgeId>> {
    let by_arrival: BTreeMap<CoedgeId, usize> =
        corners.iter().enumerate().map(|(i, &(arriving, _))| (arriving, i)).collect();
    // The corner a fan's walk goes on to from the one leaving by `leaving`.
    let next = |leaving: CoedgeId| {
        let mate = if b.edge(b.coedge(leaving).edge).is_degenerate() {
            Some(leaving)
        } else {
            b.partner(leaving).filter(|&m| b.coedge_end(m) == at)
        };
        mate.and_then(|m| by_arrival.get(&m).copied())
    };
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for start in 0..corners.len() {
        if seen.contains(&start) {
            continue;
        }
        let mut fan = Vec::new();
        let mut corner = Some(start);
        while let Some(i) = corner.filter(|&i| seen.insert(i)) {
            let Some(&(arriving, leaving)) = corners.get(i) else { break };
            fan.push(arriving);
            corner = next(leaving);
        }
        out.push(fan);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_coedge_s_samples_end_exactly_at_its_edge_s_ends() {
        // Run backwards, `a + (z - a) * 1` falls an ulp short of `z` for this range, and a last
        // sample inside the edge is taken for an interior point: at a cone's apex its angle is
        // then any angle, and the loop seems not to close.
        let (t0, t1) = (-2.9474992780948828, 2.9474992780505347);
        for (a, z) in [(t1, t0), (t0, t1)] {
            for n in [1, 8, 64] {
                let ts = spaced(a, z, n);
                assert_eq!((ts[0], ts[n]), (a, z), "{n} steps from {a} to {z}");
            }
        }
    }

    #[test]
    fn densified_parameters_keep_the_ones_asked_for_at_every_per_th_place() {
        let ts = [0.0, 0.5, 2.0];
        let dense = densified(&ts, 4);
        assert_eq!(dense.len(), 9);
        assert_eq!(dense.iter().step_by(4).copied().collect::<Vec<_>>(), ts);
        assert_eq!(densified(&[], 4), Vec::<f64>::new());
        assert_eq!(densified(&[3.0], 4), vec![3.0]);
    }
}
