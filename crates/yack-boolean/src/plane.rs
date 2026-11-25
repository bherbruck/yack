// yack: A solid cut in two by a plane.
// Adapted from springrep@a03e8a1 src/boolean/plane.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/plane.rs to yack-boolean (M3 W8); paths and visibility only.
//! A solid cut in two by a plane.

use crate::{
    error::{Error, Result},
    math::{Frame, Point3, Vec2, Vec3},
    measure,
    profile::Profile,
    sweep::extrude,
    topo::Brep,
};

/// A box holding everything of `s` on the normal's side of the plane through `origin`, its face
/// on the plane.
pub fn half_space(s: &Brep, origin: Point3, normal: Vec3) -> Result<Brep> {
    let n = normal
        .unit()
        .ok_or_else(|| Error::InvalidInput("a plane needs a normal".into()))?;
    let b = measure::bbox(s);
    let reach = 2.0 * (b.diagonal() + b.center().dist(origin)) + 1.0;
    let frame = Frame::new(origin, n, n.any_perpendicular())?;
    extrude(
        &Profile::rect(Vec2::ZERO, 2.0 * reach, 2.0 * reach),
        &frame,
        reach,
        "half",
    )
}

/// `s` cut by the plane through `origin` square to `normal`: the part on the normal's side, then
/// the part on the other; either may be empty.
pub fn split(s: &Brep, origin: Point3, normal: Vec3) -> Result<(Brep, Brep)> {
    if s.solid_ids().next().is_none() {
        return Ok((s.clone(), s.clone()));
    }
    let h = half_space(s, origin, normal)?;
    Ok((crate::intersection(s, &h)?, crate::difference(s, &h)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::section_area;

    /// A prism on the polygon `pts`, `h` tall, its sides named `e0`, `e1`, … from the first edge.
    fn prism(pts: &[[f64; 2]], h: f64) -> Brep {
        let pts: Vec<Vec2> = pts.iter().map(|p| Vec2::new(p[0], p[1])).collect();
        let names: Vec<String> = (0..pts.len()).map(|i| format!("e{i}")).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, h, "p").unwrap()
    }

    fn close(got: f64, want: f64) {
        assert!(
            (got - want).abs() < 1e-7 * want.max(1.0),
            "{got} against {want}"
        );
    }

    #[test]
    fn a_block_split_across_keeps_each_part_and_its_cut() {
        // 4 × 3 × 2 cut at x = 1: 6 on the far side, 18 on the near, a 3 × 2 cut.
        let b = prism(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]], 2.0);
        let (o, n) = (Point3::new(1.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0));
        let (near, far) = split(&b, o, n).unwrap();
        close(measure::volume(&near), 6.0);
        close(measure::volume(&far), 18.0);
        close(section_area(&b, o, n).unwrap(), 6.0);
    }
}
