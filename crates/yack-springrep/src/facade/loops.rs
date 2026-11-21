// yack: one closed planar loop rounded at every corner, or offset, over springrep's profile regions.
//! Glue for `yack::Loop::rounded` and `Loop::offset` (w-m03-api's design): the loop as a region
//! of one loop, run through [`Region::fillet`] at each corner or [`Region::offset`], and back.
//! Lines and arcs only; the loop keeps its own direction.

use std::f64::consts::TAU;

use crate::{
    Error, Result,
    profile::{
        Names, Segment,
        region::{Geom, Loop, Region, Seg},
    },
    tol,
};

/// The closed loop `segs` (in order, the last ending where the first starts) with every corner
/// between two segments rounded by a tangent arc of `radius`, convex and concave alike. A corner
/// already tangent is left as it is. Refuses (`InvalidInput`, "the round does not fit") where an
/// arc does not fit between its neighbours, without trimming them past their ends.
pub fn round_loop(segs: &[Segment], radius: f64) -> Result<Vec<Segment>> {
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(Error::InvalidInput(format!("a round of {radius} mm is not a positive length")));
    }
    let mut region = region_of(segs)?;
    let n = segs.len();
    for i in 0..n {
        let (p, q) = (&region_geom(segs, (i + n - 1) % n)?, &region_geom(segs, i)?);
        let (out, into) = (p.tangent(1.0), q.tangent(0.0));
        if n < 2 || (out.cross(into).abs() <= tol::TANGENT && out.dot(into) > 0.0) {
            continue;
        }
        let (a, b) = (name((i + n - 1) % n), name(i));
        match region.fillet(&a, &b, radius) {
            Ok(_) => {}
            Err(e) => return Err(Error::InvalidInput(format!("the round does not fit: {e}"))),
        }
    }
    segments_of(&region)
}

/// The closed loop `segs` offset by `distance`, away from the region it bounds (its left) when
/// positive: arcs about the corners it grows round, trims at the others. Refuses where the result
/// crosses itself (the message says "crosses itself"); never a silent trim.
pub fn offset_loop(segs: &[Segment], distance: f64) -> Result<Vec<Segment>> {
    if !distance.is_finite() {
        return Err(Error::InvalidInput(format!("an offset of {distance} mm is not a length")));
    }
    let region = region_of(segs)?;
    segments_of(&region.offset(distance)?)
}

/// The name segment `i` carries through the rounding, so each corner is found by its two sides.
fn name(i: usize) -> String {
    format!("loop segment {i}")
}

fn region_geom(segs: &[Segment], i: usize) -> Result<Geom> {
    match segs.get(i) {
        Some(&Segment::Line { a, b }) => Ok(Geom::Line { a, b }),
        Some(&Segment::Arc { center, radius, start, sweep }) => {
            Ok(Geom::Arc { c: center, r: radius, a0: start, sweep })
        }
        Some(_) => Err(Error::Unsupported("a loop of segments other than lines and arcs".into())),
        None => Err(Error::InvalidInput("an empty loop".into())),
    }
}

/// The loop as a region of one loop, as given (not normalised: it keeps its direction).
fn region_of(segs: &[Segment]) -> Result<Region> {
    if segs.is_empty() {
        return Err(Error::InvalidInput("an empty loop".into()));
    }
    let mut out = Vec::with_capacity(segs.len());
    for i in 0..segs.len() {
        out.push(Seg { geom: region_geom(segs, i)?, names: Names::from([name(i)]) });
    }
    let (first, last) = (out[0].geom.start(), out[out.len() - 1].geom.end());
    if first.dist(last) > tol::LINEAR {
        return Err(Error::InvalidInput("the loop does not close".into()));
    }
    Ok(Region { loops: vec![Loop { segs: out }] })
}

fn segments_of(region: &Region) -> Result<Vec<Segment>> {
    let [lp] = region.loops.as_slice() else {
        return Err(Error::InvalidInput(format!("the loop became {} loops", region.loops.len())));
    };
    Ok(lp
        .segs
        .iter()
        .map(|s| match s.geom {
            Geom::Line { a, b } => Segment::Line { a, b },
            Geom::Arc { c, r, sweep, .. } if sweep.abs() >= TAU - tol::ANGULAR => {
                Segment::Circle { center: c, radius: r, ccw: sweep > 0.0 }
            }
            Geom::Arc { c, r, a0, sweep } => {
                Segment::Arc { center: c, radius: r, start: a0, sweep }
            }
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec2;

    fn square(s: f64) -> Vec<Segment> {
        let p = [Vec2::new(0.0, 0.0), Vec2::new(s, 0.0), Vec2::new(s, s), Vec2::new(0.0, s)];
        (0..4).map(|i| Segment::Line { a: p[i], b: p[(i + 1) % 4] }).collect()
    }

    fn area(segs: &[Segment]) -> f64 {
        // ∮ x dy, lines exactly and arcs by their chords and segments.
        segs.iter()
            .map(|s| match *s {
                Segment::Line { a, b } => 0.5 * (a.x * b.y - b.x * a.y),
                Segment::Arc { center: c, radius: r, start, sweep } => {
                    let (p, q) = (
                        Vec2::new(c.x + r * start.cos(), c.y + r * start.sin()),
                        Vec2::new(c.x + r * (start + sweep).cos(), c.y + r * (start + sweep).sin()),
                    );
                    0.5 * (p.x * q.y - q.x * p.y) + 0.5 * r * r * (sweep - sweep.sin())
                }
                _ => f64::NAN,
            })
            .sum()
    }

    #[test]
    fn a_square_rounds_at_all_four_corners() {
        let r = round_loop(&square(10.0), 2.0).unwrap();
        assert_eq!(r.len(), 8);
        let want = 100.0 - (4.0 - std::f64::consts::PI) * 4.0;
        assert!((area(&r) - want).abs() < 1e-9, "{}", area(&r));
        let e = round_loop(&square(10.0), 6.0).unwrap_err().to_string();
        assert!(e.contains("the round does not fit"), "{e}");
    }

    #[test]
    fn a_square_offsets_out_with_round_corners_and_in_with_sharp_ones() {
        let out = offset_loop(&square(10.0), 1.0).unwrap();
        let want = 100.0 + 40.0 + std::f64::consts::PI;
        assert!((area(&out) - want).abs() < 1e-9, "{}", area(&out));
        let inside = offset_loop(&square(10.0), -1.0).unwrap();
        assert!((area(&inside) - 64.0).abs() < 1e-9, "{}", area(&inside));
    }
}
