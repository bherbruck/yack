// yack: the facade's glue items, re-exported from the crates they moved to (M3 W10); dies with this crate (M3).
//! Glue for `yack::make`, selection and blends, now in the crates it wraps: picks in `yack_mesh::pick`,
//! local shape in `yack_topo::local`, measured guards in `yack_measure::guard`, and so on.

pub use yack_edit::requirement::breaks_requirement;
pub use yack_geom::periodic_through;
pub use yack_measure::guard::{boxes_overlap, face_grid, plane_cuts, same_shape, walls_cross};
pub use yack_mesh::pick::{Pick, blend_strays, edge_within, face_closest, face_within, nearest_face};
pub use yack_topo::local::{convexity, edge_point, edge_points, face_normal_at, open_at_concave_edge, radius_collapses, rims_square};

#[cfg(test)]
mod tests {
    use crate::math::{Point3, Vec3};

    #[test]
    fn the_face_nearest_a_point_just_inside_a_holes_rim_is_the_holes_wall() {
        use crate::{boolean, geom::Surface, math::Frame, primitive};
        let plate = primitive::block(&Frame::WORLD, Vec3::new(30.0, 30.0, 2.0), "p").unwrap();
        let drill = primitive::cylinder(&Frame::from_axis(Vec3::new(15.0, 15.0, -1.0), Vec3::Z).unwrap(), 10.0, 4.0, "d").unwrap();
        let b = boolean::difference(&plate, &drill).unwrap();
        for turn in [0.1f64, 1.3, 2.9, 4.4] {
            for (r, wall) in [(10.0 - 0.005, true), (10.0 + 0.005, false)] {
                let p = Point3::new(15.0 + r * turn.cos(), 15.0 + r * turn.sin(), 2.0);
                let (f, d) = super::nearest_face(&b, p).unwrap().unwrap();
                let is_wall = matches!(b.face(f).surface, Surface::Cylinder(_));
                assert_eq!(is_wall, wall, "at {turn}, radius {r}");
                assert!((d - if wall { 0.005 } else { 0.0 }).abs() < 1e-9, "at {turn}, radius {r}: {d}"); // tol: the rim's own
            }
        }
    }

    #[test]
    #[ignore = "probe: yack-acceptance springrep_one_call:124, a shell whose walls meet (w-m11)"]
    fn shell_whose_walls_meet_probe() {
        use crate::{math::Frame, primitive};
        let blk = primitive::block(&Frame::WORLD, Vec3::new(10.0, 8.0, 6.0), "b").unwrap();
        let top = blk.face_ids().find(|&f| {
            let s = &blk.face(f).surface;
            let uv = s.project(Point3::new(5.0, 4.0, 6.0));
            s.eval(uv.x, uv.y).dist(Point3::new(5.0, 4.0, 6.0)) < 1e-9 && crate::measure::face_bbox(&blk, f).min.z > 5.9
        }).unwrap();
        for t in [3.9, 3.999, 4.0, 4.001, 4.5] {
            match crate::facade::edit::shell(&blk, &[top], t) {
                Ok(b) => eprintln!("t {t}: Ok, {} faces, volume {}, walls_cross {}", b.face_ids().count(), crate::measure::volume(&b), super::walls_cross(&blk, &[top], t)),
                Err(e) => eprintln!("t {t}: Err {e:?} breaks_requirement {} walls_cross {}", super::breaks_requirement("shell", &e.to_string()), super::walls_cross(&blk, &[top], t)),
            }
        }
    }
}
