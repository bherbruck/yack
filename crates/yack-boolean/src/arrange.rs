// yack: Splitting a face by cuts in its parameter plane: the boundary trims and both directions of every
// Adapted from springrep@a03e8a1 src/boolean/arrange.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/arrange.rs to yack-boolean (M3 W8); paths and visibility only;
// hardened (M3 B3): the graph's half-edges one struct with checked access in place of parallel
// indexed vectors, its small literals named; the order of every walk is as before.
//! Splitting a face by cuts in its parameter plane: the boundary trims and both directions of every
//! cut form a planar graph, whose regions, traced with the face's material on the left, are the
//! pieces of the face.

use std::f64::consts::TAU;

use crate::topo::domain::{Domain, Trim, UvMap, direction_angle, polygon_area, wrap_angle};
use crate::{Error, Result, geom::Surface, math::Vec2};

/// How far apart, in mm along the surface, two parameter-plane images of one vertex may be and
/// still be one node of the graph, or twice the vertex's local tolerance where that is wider: the
/// edges of an imported vertex end anywhere in its sphere. Images of a vertex at different nodes
/// (across a seam) are a period apart.
const NODE_GAP: f64 = 1e-5; // tol: a gap between images of one vertex, in mm along the surface

/// The direction a trim leaves its start in, as an exact tangent angle and, to order trims that
/// leave tangentially, the angle of a chord a little way along; and the oriented vector to its far
/// end, which orders trims whose tangents and chords agree to the last bit (two edges of solids
/// a hair apart leaving a shared corner a hair off parallel).
#[derive(Clone, Copy, Debug)]
struct Departure {
    tangent: f64,
    chord: f64,
    far: Vec2,
}

struct Graph<'a> {
    surface: &'a Surface,
    sigma: f64,
    scale: Vec2,
}

impl Graph<'_> {
    fn oriented(&self, d: Vec2) -> Vec2 {
        Vec2::new(self.sigma * d.x * self.scale.x, d.y * self.scale.y)
    }

    /// The departure, its chord reaching `reach` mm along the trim, or the whole trim if shorter;
    /// `node` is where the trim's node is, which its far end is measured from.
    fn departure(&self, tr: &Trim, reach: f64, node: Vec2) -> Departure {
        let chord_uv = match tr.map {
            UvMap::Affine { .. } => tr.uv1,
            UvMap::Projected | UvMap::Fitted(_) => {
                tr.uv(self.surface, tr.param(fraction_at(tr, reach)), tr.uv0)
            }
        };
        let chord = direction_angle(self.oriented(chord_uv - tr.uv0)).unwrap_or(0.0);
        let tangent =
            direction_angle(self.oriented(tr.start_tangent(self.surface))).unwrap_or(chord);
        Departure {
            tangent,
            chord,
            far: self.oriented(tr.uv1 - node),
        }
    }

    /// What the trim adds to its loop's signed area, ½∫(u dv − v du) in the oriented plane: by
    /// Gauss–Legendre along the curve with the exact parameter-plane rate. Chords would sag from a
    /// curved trim by more than a sliver between two such trims is wide, and turn its sign.
    fn swept_area(&self, tr: &Trim) -> f64 {
        let chord = |a: Vec2, b: Vec2| {
            let (p, q) = (self.oriented(a), self.oriented(b));
            0.5 * (p.x * q.y - p.y * q.x)
        };
        let curved = matches!((&tr.map, &tr.curve), (UvMap::Projected, Some(_)) | (UvMap::Fitted(_), _));
        if !curved {
            return chord(tr.uv0, tr.uv1);
        }
        let mut nodes: Vec<(f64, f64)> = crate::numeric::gauss(24).to_vec();
        nodes.sort_by(|a, b| a.0.total_cmp(&b.0));
        let (m, h) = (0.5 * (tr.t0 + tr.t1), 0.5 * (tr.t1 - tr.t0));
        let mut hint = tr.uv0;
        let mut sum = 0.0;
        for (x, w) in nodes {
            let t = m + h * x;
            let uv = tr.uv(self.surface, t, hint);
            hint = uv;
            let rate = tr.rate(self.surface, uv, t);
            let (p, d) = (self.oriented(uv), self.oriented(rate));
            sum += w * (p.x * d.y - p.y * d.x);
        }
        0.5 * sum * h
    }

    fn metric(&self, a: Vec2, b: Vec2) -> f64 {
        (a.x - b.x).abs() * self.scale.x + (a.y - b.y).abs() * self.scale.y
    }
}

/// A trim's length along its curve, from a few chords.
fn trim_length(tr: &Trim) -> f64 {
    let n = 16;
    (0..n)
        .map(|k| {
            let (a, b) = (k as f64 / n as f64, (k + 1) as f64 / n as f64);
            tr.point(tr.param(a)).dist(tr.point(tr.param(b)))
        })
        .sum()
}

/// The fraction of the trim at which it is first `reach` mm from its start, or 1 if it never is.
/// Chord angles grow with chord length, so two trims bending alike are compared at exactly equal
/// distances: a fraction from the trim's length alone is off by its parameter's unevenness.
fn fraction_at(tr: &Trim, reach: f64) -> f64 {
    let start = tr.point(tr.t0);
    let away = |f: f64| tr.point(tr.param(f)).dist(start);
    let guess = (reach / trim_length(tr).max(1e-300)).min(1.0); // tol: keeps a zero length from dividing
    let mut hi = guess;
    while away(hi) < reach {
        if hi >= 1.0 {
            return 1.0;
        }
        hi = (2.0 * hi).min(1.0);
    }
    let mut lo = 0.0;
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if away(mid) < reach {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

fn signed_angle(a: f64) -> f64 {
    let w = wrap_angle(a);
    if w > TAU / 2.0 { w - TAU } else { w }
}

/// Second-order weight of a departure's bend: it orders departures whose tangents agree, and is
/// too small to reorder any whose tangents differ by more than a few 1e-7 rad.
const BEND: f64 = 1e-7; // tol: a weight on an angle, not a length

/// Tangents closer than this (radians), or than the two trims bend over their chords, are a tie.
const TANGENT_TIE: f64 = 1e-9; // tol: an angle, not a length

/// Angles this close (radians) agree to the last bits of a turn's reading.
const LAST_BITS: f64 = 1e-13; // tol: an angle at rounding's scale, not a length

/// The turn given a departure a hair either side of the arrival's own line: just short of a whole
/// turn, or just past none.
const HAIR_TURN: f64 = BEND * 1e-9; // tol: an angle below any turn read, not a length

/// Turns further apart than this (radians) are ordered by themselves alone.
const DISTINCT_TURN: f64 = 1e-6; // tol: an angle, not a length

/// A turn this far (radians) from none and from a whole turn is well away from the arrival.
const CLEAR_TURN: f64 = 1e-3; // tol: an angle, not a length

/// The clockwise turn from `back` to `c`, in (0, 2π]. Of departures along one tangent, the one
/// bending further counter-clockwise is met first.
fn clockwise_turn(back: Departure, c: Departure) -> f64 {
    let d = wrap_angle(back.tangent - c.tangent);
    // A fitted curve's tangent is good only to its fit, so tangents closer than the trims bend
    // over their chords are a tie that the chords decide.
    let eps = TANGENT_TIE.max(bend(back) + bend(c));
    if signed_angle(d).abs() > eps {
        return d - BEND * signed_angle(c.chord - c.tangent);
    }
    let d2 = signed_angle(back.chord - c.chord);
    if d2.abs() <= LAST_BITS && signed_angle(d).abs() <= LAST_BITS {
        // `c` a hair counter-clockwise of `back` is the last turn, a hair clockwise the first.
        let cross = back.far.x * c.far.y - back.far.y * c.far.x;
        return if cross > 0.0 { TAU - HAIR_TURN } else { HAIR_TURN };
    }
    if d2 > 0.0 { d2 * BEND } else { TAU + d2 * BEND }
}

/// How far a departure's chord turns from its tangent.
fn bend(x: Departure) -> f64 {
    signed_angle(x.chord - x.tangent).abs()
}

/// Which of two departures is met first turning clockwise, their turns from the arrival being
/// `ka` and `kb`. Two whose turns are closer than they bend over their chords are ordered by the
/// chords of `ab`, both taken at one reach.
fn turn_order(ka: f64, kb: f64, ab: impl Fn() -> (Departure, Departure)) -> std::cmp::Ordering {
    if (ka - kb).abs() > DISTINCT_TURN {
        return ka.total_cmp(&kb);
    }
    let (a, b) = ab();
    let same = |x: f64, y: f64| signed_angle(x - y).abs() <= LAST_BITS;
    if same(ka, kb) && same(a.tangent, b.tangent) && same(a.chord, b.chord) {
        // Of two leaving along one line to the last bit, the further counter-clockwise is met
        // first turning clockwise.
        let cross = a.far.x * b.far.y - a.far.y * b.far.x;
        return cross.total_cmp(&0.0);
    }
    let eps = bend(a) + bend(b);
    let clear = |k: f64| k > eps && k < TAU - eps;
    if (ka - kb).abs() <= eps && clear(ka) && clear(kb) {
        let by_chord = |k: f64, x: Departure| k + signed_angle(x.tangent - x.chord);
        return by_chord(ka, a).total_cmp(&by_chord(kb, b));
    }
    ka.total_cmp(&kb)
}

/// Whether `b` runs back along `a`: the same piece the other way, between the same images.
fn retraces(a: &Trim, b: &Trim, g: &Graph) -> bool {
    a.piece == b.piece
        && a.forward != b.forward
        && a.v0 == b.v1
        && a.v1 == b.v0
        && g.metric(a.uv0, b.uv1) <= NODE_GAP
        && g.metric(a.uv1, b.uv0) <= NODE_GAP
}

/// `lp` without the spikes in it: trims that run out along a piece and straight back. A boundary
/// tangent to itself at a corner (a circle meeting a line it touches) bounds a sliver no wider than
/// the tolerance between them; once a vertex of the other solid lands in that sliver, the line and
/// the circle up to it are one edge, and the loop runs out along it and back.
fn without_spikes(lp: Vec<Trim>, g: &Graph) -> Vec<Trim> {
    let mut out: Vec<Trim> = Vec::with_capacity(lp.len());
    for t in lp {
        if out.last().is_some_and(|l| retraces(l, &t, g)) {
            out.pop();
        } else {
            out.push(t);
        }
    }
    while matches!(out.as_slice(), [first, .., last] if retraces(last, first, g)) {
        out.pop();
        out.remove(0);
    }
    out
}

/// One direction of a trim in the face's graph: the trim, its other direction where it is a cut,
/// whether it still bounds a region, its nodes, and the length its chords compare at.
struct Half {
    trim: Trim,
    twin: Option<usize>,
    alive: bool,
    start: usize,
    end: usize,
    /// Chords compared reach equally far, so departures along one tangent are ordered by how they
    /// bend, not by how long they are; a straight trim's chord is the whole of it.
    length: f64,
}

fn broken(what: &str) -> Error {
    Error::Numerical(format!("face split: {what}"))
}

/// The pieces of a face with `boundary` loops cut by `cuts` (each given in one direction), each as
/// its loops, outer first. A cut `slit` names may end where nothing else does: a contact, where
/// another solid touches the face along a curve. The face's loop then runs along it and back, or
/// it is a loop of its own inside the face, both ways along it.
pub(crate) fn split(
    surface: &Surface,
    reversed: bool,
    boundary: Vec<Vec<Trim>>,
    cuts: Vec<Trim>,
    vertex_tol: &dyn Fn(usize) -> f64,
    slit: &dyn Fn(usize) -> bool,
) -> Result<Vec<Vec<Vec<Trim>>>> {
    if cuts.is_empty() {
        return Ok(vec![boundary]);
    }
    let whole = Domain::new(surface.clone(), reversed, boundary.clone());
    let g = Graph {
        surface,
        sigma: if reversed { -1.0 } else { 1.0 },
        scale: whole.scale(),
    };
    let boundary: Vec<Vec<Trim>> = boundary
        .into_iter()
        .map(|lp| without_spikes(lp, &g))
        .filter(|lp| !lp.is_empty())
        .collect();
    // The boundary's trims, then each cut's two directions, reversed first.
    let mut trims: Vec<(Trim, Option<usize>)> = boundary.into_iter().flatten().map(|tr| (tr, None)).collect();
    for c in cuts {
        let i = trims.len();
        trims.push((c.reversed(), Some(i + 1)));
        trims.push((c, Some(i)));
    }

    // Nodes: a vertex at one place in the parameter plane.
    let mut nodes: Vec<(usize, Vec2)> = Vec::new();
    let mut node_of = |v: usize, uv: Vec2| -> usize {
        if let Some(i) = nodes
            .iter()
            .position(|&(w, p)| w == v && g.metric(p, uv) <= NODE_GAP.max(2.0 * vertex_tol(v)))
        {
            return i;
        }
        nodes.push((v, uv));
        nodes.len() - 1
    };
    let mut halves: Vec<Half> = Vec::with_capacity(trims.len());
    for (trim, twin) in trims {
        let start = node_of(trim.v0, trim.uv0);
        let end = node_of(trim.v1, trim.uv1);
        let length = match trim.map {
            UvMap::Projected | UvMap::Fitted(_) => trim_length(&trim),
            UvMap::Affine { .. } => f64::INFINITY,
        };
        halves.push(Half { trim, twin, alive: true, start, end, length });
    }

    // A cut that ends where nothing else does bounds no region, unless it is a slit. Each pass
    // reads the degrees as they stood at its start; a half it drops takes its twin with it.
    loop {
        let mut degree = vec![0usize; nodes.len()];
        for h in halves.iter().filter(|h| h.alive) {
            for n in [h.start, h.end] {
                *degree.get_mut(n).ok_or_else(|| broken("a trim's node is missing"))? += 1;
            }
        }
        let two = |n: usize| degree.get(n) == Some(&2);
        let dead: Vec<(usize, usize)> = halves
            .iter()
            .enumerate()
            .filter_map(|(i, h)| {
                let t = h.twin?;
                (h.alive && !slit(h.trim.piece) && h.start != h.end && (two(h.start) || two(h.end)))
                    .then_some((i, t))
            })
            .collect();
        if dead.is_empty() {
            break;
        }
        for (i, t) in dead {
            for k in [i, t] {
                halves.get_mut(k).ok_or_else(|| broken("a cut's twin is missing"))?.alive = false;
            }
        }
    }

    let mut out_of: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for (i, h) in halves.iter().enumerate().filter(|(_, h)| h.alive) {
        out_of.get_mut(h.start).ok_or_else(|| broken("a trim's node is missing"))?.push(i);
    }
    let next = |h: usize| -> Option<usize> {
        let arrival = halves.get(h)?;
        let outs = out_of.get(arrival.end)?;
        // A boundary only turns straight back along the edge it came by at a dead end: a cut's
        // other side, or the face's own loop running back down a contact it already holds,
        // which leaves at exactly the arrival's tangent and so would read as the sharpest turn.
        let back = |o: usize| {
            Some(o) == arrival.twin
                || halves.get(o).is_some_and(|out| {
                    out.trim.piece == arrival.trim.piece
                        && out.trim.forward != arrival.trim.forward
                        && out.start == arrival.end
                        && out.end == arrival.start
                })
        };
        let others: Vec<usize> = outs.iter().copied().filter(|&o| !back(o)).collect();
        let pool = if others.is_empty() {
            outs.clone()
        } else {
            others
        };
        // Each turn from the arrival is read with chords a twentieth of the shorter of the two
        // trims, so a sliver trim elsewhere at the node cannot shorten them until rounding
        // outweighs how two trims leaving along nearly one tangent part. Turns that come out
        // close are read again with one reach, a twentieth of the shortest of the three.
        let at = nodes.get(arrival.end)?.1;
        let dep = |t: &Trim, r: f64| g.departure(t, r.min(1e300), at);
        let back_trim = arrival.trim.reversed();
        let own: Vec<(&Half, f64)> = pool
            .iter()
            .map(|&c| {
                let out = halves.get(c)?;
                let r = 0.05 * arrival.length.min(out.length);
                Some((out, clockwise_turn(dep(&back_trim, r), dep(&out.trim, r))))
            })
            .collect::<Option<_>>()?;
        let best = own.into_iter().zip(&pool).min_by(|&((a, ka), _), &((b, kb), _)| {
            if (ka - kb).abs() > DISTINCT_TURN {
                return ka.total_cmp(&kb);
            }
            // Two turning well away from the arrival are ordered between themselves; the
            // arrival's length has no say in their reach.
            let clear = |k: f64| k > CLEAR_TURN && k < TAU - CLEAR_TURN;
            let pair = 0.05 * a.length.min(b.length);
            let r = if clear(ka) && clear(kb) {
                pair
            } else {
                pair.min(0.05 * arrival.length)
            };
            let back_h = dep(&back_trim, r);
            let (la, lb) = (dep(&a.trim, r), dep(&b.trim, r));
            turn_order(
                clockwise_turn(back_h, la),
                clockwise_turn(back_h, lb),
                || (la, lb),
            )
        });
        best.map(|(_, &c)| c)
    };

    let mut seen = vec![false; halves.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for (h0, half) in halves.iter().enumerate() {
        if !half.alive || seen.get(h0).copied().unwrap_or(true) {
            continue;
        }
        let mut cyc = Vec::new();
        let mut h = h0;
        loop {
            let seen_h = seen.get_mut(h).ok_or_else(|| broken("a boundary runs to a missing trim"))?;
            if *seen_h {
                if h == h0 {
                    break;
                }
                return Err(broken("a region's boundary does not close"));
            }
            *seen_h = true;
            cyc.push(h);
            h = next(h).ok_or_else(|| broken("a boundary ends at a node with no way on"))?;
        }
        cycles.push(cyc);
    }

    let trim_of = |h: usize| halves.get(h).map(|x| &x.trim).ok_or_else(|| broken("a region names a missing trim"));
    // A loop's area from 16 chords a trim, with its error judged against 8: where that error is
    // a fair share of the area (a sliver between two curved trims, which chords sag across), the
    // exact integral decides the sign instead. Chords follow a curve a hair from a pole, where a
    // fixed rule's points would not.
    let area = |cyc: &[usize]| -> Result<f64> {
        let (mut fine, mut coarse) = (Vec::new(), Vec::new());
        for &h in cyc {
            let tr = trim_of(h)?;
            let n = match tr.map {
                UvMap::Affine { .. } => 1,
                UvMap::Projected | UvMap::Fitted(_) => 16,
            };
            let pl = tr.polyline(surface, n);
            let (_, all_but_last) = pl.split_last().ok_or_else(|| broken("a trim has no points"))?;
            for (k, &p) in all_but_last.iter().enumerate() {
                fine.push(g.oriented(p));
                if k % 2 == 0 {
                    coarse.push(g.oriented(p));
                }
            }
        }
        let a = polygon_area(&fine);
        if a.abs() > 4.0 * (a - polygon_area(&coarse)).abs() / 3.0 {
            return Ok(a);
        }
        cyc.iter().map(|&h| trim_of(h).map(|tr| g.swept_area(tr))).sum()
    };
    // A cycle running only along slits and back, each trim's twin in it too, encloses nothing: a
    // slit inside the face, a loop of its own.
    let only_slits = |cyc: &[usize]| {
        cyc.iter()
            .all(|&h| halves.get(h).and_then(|x| x.twin).is_some_and(|t| cyc.contains(&t)))
    };
    let areas: Vec<f64> = cycles
        .iter()
        .map(|c| if only_slits(c) { Ok(0.0) } else { area(c) })
        .collect::<Result<_>>()?;
    let loop_of = |cyc: &[usize]| -> Result<Vec<Trim>> { cyc.iter().map(|&h| trim_of(h).cloned()).collect() };
    // The outer regions, each its cycle and area.
    let outers: Vec<(&Vec<usize>, f64)> = cycles.iter().zip(areas.iter().copied()).filter(|&(_, a)| a > 0.0).collect();
    let outer_domains: Vec<Domain> = outers
        .iter()
        .map(|&(c, _)| Ok(Domain::new(surface.clone(), reversed, vec![loop_of(c)?])))
        .collect::<Result<_>>()?;
    let mut pieces: Vec<Vec<Vec<Trim>>> = outers.iter().map(|&(c, _)| Ok(vec![loop_of(c)?])).collect::<Result<_>>()?;
    let pieces_of = |c: &[usize]| -> Vec<usize> { c.iter().filter_map(|&h| halves.get(h)).map(|x| x.trim.piece).collect() };
    for (cyc, &area) in cycles.iter().zip(&areas) {
        if area > 0.0 {
            continue;
        }
        let first = cyc.first().map_or_else(|| Err(broken("a hole has no trims")), |&h| trim_of(h))?;
        let probe = first.mid_uv(surface);
        // A region on the other side of any of the hole's pieces cannot hold it.
        let mine = pieces_of(cyc);
        let host = outers
            .iter()
            .zip(&outer_domains)
            .enumerate()
            .filter(|(_, ((c, _), _))| !pieces_of(c).iter().any(|p| mine.contains(p)))
            // A region winding past a period holds the probe at one of its images.
            .filter(|(_, (_, d))| d.inside_image(probe).is_some())
            .min_by(|(_, ((_, a), _)), (_, ((_, b), _))| a.total_cmp(b))
            .map(|(k, _)| k);
        match host.and_then(|k| pieces.get_mut(k)) {
            Some(piece) => piece.push(loop_of(cyc)?),
            None => return Err(broken("a hole lies in no region")),
        }
    }
    Ok(pieces)
}

#[cfg(test)]
mod tests {
    use std::cmp::Ordering;
    use std::f64::consts::PI;

    use super::*;

    fn order(back: Departure, a: Departure, b: Departure) -> Ordering {
        turn_order(clockwise_turn(back, a), clockwise_turn(back, b), || (a, b))
    }

    fn dep(tangent: f64, chord: f64) -> Departure {
        Departure {
            tangent,
            chord,
            far: Vec2::ZERO,
        }
    }

    #[test]
    fn near_tangent_departures_are_ordered_by_their_chords() {
        // Arriving from below: a straight cut leaving along -x, and a fitted curve whose tangent
        // is a hair above -x but whose chord bends below it, so it is the first clockwise.
        let back = dep(1.5 * PI, 1.5 * PI);
        let line = dep(PI, PI);
        let curve = dep(PI - 3e-8, PI + 2e-3);
        assert_eq!(order(back, curve, line), Ordering::Less);
        assert_eq!(order(back, line, curve), Ordering::Greater);
        // Tangents apart by more than the bends still decide.
        let apart = dep(PI - 0.1, PI - 0.1 + 2e-3);
        assert_eq!(order(back, apart, line), Ordering::Greater);
        // A curve leaving along the arrival's own line, bending away below it, is met just after
        // it rather than a full turn later.
        let back = dep(0.0, 0.0);
        let along = dep(7e-7, -0.04);
        assert!(clockwise_turn(back, along) < 1e-6);
    }
}
