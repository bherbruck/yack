// yack: walks of the arena: sample parameters along a coedge, and the fans of faces at each vertex.
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from check.rs to topo/walks.rs (M2.2, T9); paths only; then to yack-topo (M3 W4), paths and visibility only.
//! Walks of the arena that the checker, measures, booleans and the STEP writer share: evenly
//! spaced parameters along a coedge, and the fans of faces round each vertex.

#[allow(clippy::disallowed_types)] use std::collections::HashMap; // D26: BTree* in hardening, proven by digests

use super::{Brep, CoedgeId, VertexId};

/// Edge parameters along a coedge, in its own direction.
pub fn coedge_params(b: &Brep, c: CoedgeId, n: usize) -> Vec<f64> {
    let co = b.coedge(c);
    let (t0, t1) = b.edge(co.edge).range;
    let (a, z) = if co.reversed { (t1, t0) } else { (t0, t1) };
    spaced(a, z, n)
}

/// `n + 1` parameters from `a` to `z`, evenly spaced, the last exactly `z`: an end an ulp inside
/// the edge would be taken for an interior point.
fn spaced(a: f64, z: f64, n: usize) -> Vec<f64> {
    (0..=n)
        .map(|i| match i {
            _ if i == n => z,
            _ => a + (z - a) * i as f64 / n as f64,
        })
        .collect()
}

/// Each vertex's fans of faces, each as the coedges arriving at the vertex round it. A degenerate
/// coedge, a pole or a vertex loop, leaves the vertex where it arrived, so its face's corners on
/// either side of it are one corner.
#[allow(clippy::disallowed_types)] pub fn fans_at_vertices(b: &Brep) -> HashMap<VertexId, Vec<Vec<CoedgeId>>> { // D26: BTree* in hardening, proven by digests
    // A corner at a vertex: the coedge arriving there and the next one in its loop, leaving.
    let mut corners: HashMap<VertexId, Vec<(CoedgeId, CoedgeId)>> = HashMap::new();
    for lp in b.loop_ids() {
        let cs = &b.lp(lp).coedges;
        for k in 0..cs.len() {
            let (a, n) = (cs[k], cs[(k + 1) % cs.len()]);
            let v = b.coedge_end(a);
            if b.coedge_start(n) == v {
                corners.entry(v).or_default().push((a, n));
            }
        }
    }
    let mut out = HashMap::new();
    for (&v, cs) in &corners {
        let by_arrival: HashMap<CoedgeId, usize> =
            cs.iter().enumerate().map(|(i, &(a, _))| (a, i)).collect();
        let mut seen = vec![false; cs.len()];
        let mut fans = Vec::new();
        for start in 0..cs.len() {
            if seen[start] {
                continue;
            }
            let mut fan = Vec::new();
            let mut i = start;
            while !seen[i] {
                seen[i] = true;
                fan.push(cs[i].0);
                let leaving = cs[i].1;
                let mate = if b.edge(b.coedge(leaving).edge).is_degenerate() {
                    Some(leaving)
                } else {
                    b.partner(leaving).filter(|&m| b.coedge_end(m) == v)
                };
                match mate.and_then(|m| by_arrival.get(&m)) {
                    Some(&j) => i = j,
                    None => break,
                }
            }
            fans.push(fan);
        }
        out.insert(v, fans);
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
}
