// yack: point location: a face's trimmed domain and point-in-solid classification (PLAN §3.4 T1).
//! Point location: point-in-solid classification (`classify`), and whether two faces lie on one
//! surface. A face's trimmed domain is topo's (`topo::domain`, `topo::face_domain`). Moved out of `boolean/`
//! in M2.1 (T1) so that `check`, `measure`, `mesh`, `rays`, `distance` and `topo` need not reach
//! into the boolean.


// springrep's locate indexes by position; D26's hardening pass moves it onto checked access,
// as for yack-linalg until M5.
#![allow(clippy::indexing_slicing)]

// springrep's foundation modules (D26), under the names locate's code uses.
use yack_base::kernel::Error;
use yack_base::kernel as error;
use yack_geom as geom;
use yack_intersect as intersect;
use yack_math::kernel as math;
use yack_tol as tol;
use yack_topo as topo;

pub mod distance;
pub mod rays;
pub mod classify;

use crate::{geom::Surface, intersect::ssi::same_domain, topo::domain::Domain};

fn analytic(s: &Surface) -> bool {
    matches!(
        s,
        Surface::Plane(_)
            | Surface::Cylinder(_)
            | Surface::Cone(_)
            | Surface::Sphere(_)
            | Surface::Torus(_)
    )
}

/// Whether two faces' surfaces are the same point set where the faces are. Analytic pairs are
/// decided in closed form by [`same_domain`]; any other pair (swept, ruled or spline surfaces in
/// different frames or parameterisations, or one of them against an analytic surface) by checking
/// that each face's parameter window lies on the other surface, with the same normal line, within
/// the tolerance.
pub fn same_domain_faces(a: &Domain, b: &Domain) -> bool {
    if same_domain(&a.surface, &b.surface) {
        return true;
    }
    // Two planes a hair off parallel are one plane over faces small enough that they stay within
    // the tolerance of each other: a part placed by a rounded pose sits on its seat that way, and
    // their meeting line, at the angle between them, would run anywhere along the band.
    let planes = matches!(
        (&a.surface, &b.surface),
        (Surface::Plane(_), Surface::Plane(_))
    );
    if analytic(&a.surface) && analytic(&b.surface) && !planes {
        return false;
    }
    let n = 6;
    [(a, b), (b, a)].iter().all(|(from, to)| {
        (0..=n).all(|i| {
            (0..=n).all(|j| {
                let u = from.lo.x + (from.hi.x - from.lo.x) * i as f64 / n as f64;
                let v = from.lo.y + (from.hi.y - from.lo.y) * j as f64 / n as f64;
                let p = from.surface.eval(u, v);
                let q = to.surface.project(p);
                to.surface.eval(q.x, q.y).dist(p) <= tol::LINEAR
                    && from
                        .surface
                        .normal(u, v)
                        .cross(to.surface.normal(q.x, q.y))
                        .norm()
                        <= tol::TANGENT
            })
        })
    })
}
