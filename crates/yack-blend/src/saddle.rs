// yack: A saddle: four straight edges filleted to one radius at a vertex of three planes square to each
// Adapted from springrep@a03e8a1 src/blend/saddle.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/blend/saddle.rs to yack-blend (M3 W9a); paths and visibility only.
//! A saddle: four straight edges filleted to one radius at a vertex of three planes square to each
//! other, alternating concave and convex round it. Two lines cross at the vertex in the plane of
//! two of the faces, and each turns from concave to convex there, so no ball touches all four.
//!
//! In the vertex's frame `(a, b, c)` scaled by the radius `r`, the edges run concave along `+b` and
//! `+a` and convex along `−b` and `−a`, and the body is `{c < 0, b > 0} ∪ {c > 0, a > 0}`. Each
//! fillet stops at the cube `[−r, r]³` about the vertex, where its section is square to its edge.
//! Inside the cube the filleted body is nine faces: the cube's six walls, the convex fillets
//! running on past the vertex to their ends, and the corner patch (see [`saddle_patch`]), which
//! meets each fillet G1. None of the body's faces reach inside. The cube is cut from the body and
//! that solid is joined in its place.

#[allow(clippy::disallowed_types)] use std::collections::{BTreeSet, HashMap}; // D26: BTree* in hardening, proven by digests

use super::{
    Picked,
    corner::{ToolFace, assemble},
    glue, leaving, outward, parallelepiped, run,
    saddle_patch::{self, Cylinder, Sides, concave_end, convex_end},
    shared_edges,
};
use crate::{
    Error, Result,
    boolean::{self, unify},
    topo::domain::Where,
    geom::{Circle, Curve, Line, NurbsCurve, Surface},
    math::{Frame, Point3, Vec3},
    tol,
    topo::{Brep, FaceName, VertexId},
};

/// A saddle found at a vertex.
pub(super) struct Saddle {
    origin: Point3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
    r: f64,
    /// The picked edges, by index: concave along `b`, convex along `−b`, concave along `a`,
    /// convex along `−a`.
    pub edges: [usize; 4],
}

/// The saddle the picked edges `at` make at `v`, if they make one.
#[allow(clippy::panic)] pub(super) fn find(
    body: &Brep,
    picked: &[Picked],
    at: &[usize],
    v: VertexId,
    r: f64,
) -> Option<Saddle> {
    if at.len() != 4 {
        return None;
    }
    let origin = body.vertex(v).point;
    let mut edges = Vec::with_capacity(4);
    for &i in at {
        let pk = &picked[i];
        let e = body.edge(pk.edge);
        if !matches!(e.curve, Some(Curve::Line(_)))
            || pk
                .faces
                .iter()
                .any(|&f| !matches!(body.face(f).surface, Surface::Plane(_)))
        {
            return None;
        }
        let dir = leaving(body, pk.edge, v)?.unit()?;
        let convex = run::convex(body, pk.coedges[0])?;
        let normals = pk.faces.map(|f| outward(body, f, origin));
        edges.push((i, dir, convex, normals));
    }
    let concave: Vec<_> = edges.iter().filter(|e| !e.2).collect();
    let convex: Vec<_> = edges.iter().filter(|e| e.2).collect();
    if concave.len() != 2 || convex.len() != 2 {
        return None;
    }
    let near = |p: Vec3, q: Vec3| (p - q).norm() <= tol::ANGULAR;
    let holds = |normals: &[Vec3; 2], want: [Vec3; 2]| {
        (near(normals[0], want[0]) && near(normals[1], want[1]))
            || (near(normals[0], want[1]) && near(normals[1], want[0]))
    };
    for (bi, ai) in [(0, 1), (1, 0)] {
        let (b, a) = (concave[bi].1, concave[ai].1);
        if a.dot(b).abs() > tol::ANGULAR {
            return None;
        }
        let back = |d: Vec3| convex.iter().find(|e| near(e.1, -d));
        let (Some(e2), Some(e4)) = (back(b), back(a)) else {
            continue;
        };
        // Below the convex edge along −b the body lies on the +c side of the plane both lines lie in.
        let Some(&under) =
            e2.3.iter()
                .find(|n| n.dot(a).abs() < 0.5 && n.dot(b).abs() < 0.5)
        else {
            continue;
        };
        let c = -under;
        if !(holds(&concave[bi].3, [-a, c])
            && holds(&e2.3, [-a, -c])
            && holds(&concave[ai].3, [-b, -c])
            && holds(&e4.3, [c, -b]))
        {
            continue;
        }
        let s = Saddle {
            origin,
            a,
            b,
            c,
            r,
            edges: [concave[bi].0, e2.0, concave[ai].0, e4.0],
        };
        // Each face reaches across the cube's side of it: none stops inside, where the corner is
        // built without it.
        let on = |k: usize, f: usize, pts: &[[f64; 3]]| {
            let face = picked[s.edges[k]].faces[f];
            let dom = crate::topo::face_domain(body, face);
            pts.iter().all(|&q| dom.locate(s.at(q)) != Where::Outside)
        };
        let side_of = |k: usize, n: Vec3| {
            let normals = &edges.iter().find(|e| e.0 == s.edges[k]).unwrap_or_else(|| panic!("an edge")).3;
            usize::from(!near(normals[0], n))
        };
        let x = side_of(0, -a);
        let t = side_of(0, c);
        let u = side_of(1, -c);
        let f = side_of(2, -b);
        let covered = on(0, x, &[[0.0, -1.0, 1.0], [0.0, 1.0, 1.0], [0.0, 1.0, 0.0]])
            && on(0, t, &[[-1.0, 0.0, 0.0], [-1.0, 1.0, 0.0], [0.0, 1.0, 0.0]])
            && on(1, u, &[[1.0, 0.0, 0.0], [1.0, -1.0, 0.0], [0.0, -1.0, 0.0]])
            && on(
                2,
                f,
                &[
                    [-1.0, 0.0, -1.0],
                    [1.0, 0.0, -1.0],
                    [1.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0],
                ],
            );
        return covered.then_some(s);
    }
    None
}

#[allow(clippy::disallowed_types)] impl Saddle { // D26: BTree* in hardening, proven by digests
    /// The point at `q` in the saddle's frame, in radii.
    fn at(&self, q: [f64; 3]) -> Point3 {
        self.origin + (self.a * q[0] + self.b * q[1] + self.c * q[2]) * self.r
    }

    /// The cube the fillets stop at.
    fn cube(&self) -> Result<Brep> {
        parallelepiped(
            self.at([-1.0, -1.0, -1.0]),
            [
                self.a * (2.0 * self.r),
                self.b * (2.0 * self.r),
                self.c * (2.0 * self.r),
            ],
        )
    }

    /// The fillets' axes, as cylinders: concave along `b`, convex along `−b`, concave along `a`,
    /// convex along `−a`.
    fn axes(&self) -> [Cylinder; 4] {
        let cyl = |q: [f64; 3], axis: Vec3| Cylinder {
            origin: self.at(q),
            axis,
        };
        [
            cyl([-1.0, 0.0, 1.0], self.b),
            cyl([1.0, 0.0, 1.0], self.b),
            cyl([0.0, -1.0, -1.0], self.a),
            cyl([0.0, 1.0, -1.0], self.a),
        ]
    }

    /// The filleted body inside the cube, its fillets' faces named as the edges' `names` are.
    fn solid(&self, names: [&BTreeSet<FaceName>; 4]) -> Result<Brep> {
        let axes = self.axes();
        let mut tool = Brep::new();
        let mut vs: HashMap<&str, (VertexId, Point3)> = HashMap::new();
        for (k, q) in [
            ("A", [0.0, 1.0, 1.0]),
            ("B", [1.0, -1.0, 0.0]),
            ("C", [1.0, 0.0, -1.0]),
            ("D", [-1.0, 1.0, 0.0]),
            ("w1", [-1.0, 1.0, -1.0]),
            ("w2", [1.0, 1.0, -1.0]),
            ("w3", [1.0, 1.0, 1.0]),
            ("t1", [0.0, -1.0, 1.0]),
            ("t2", [1.0, -1.0, 1.0]),
            ("b1", [-1.0, 0.0, -1.0]),
        ] {
            let p = self.at(q);
            vs.insert(k, (tool.add_vertex(p), p));
        }
        let pt = |k: &str| vs[k].1;
        let sides = Sides {
            bottom: (convex_end(&axes[1], pt("A"), pt("B"), true)?, axes[1]),
            top: (convex_end(&axes[3], pt("D"), pt("C"), false)?, axes[3]),
            left: (concave_end(&axes[0], pt("A"), pt("D"))?, axes[0]),
            right: (concave_end(&axes[2], pt("B"), pt("C"))?, axes[2]),
        };
        let patch = saddle_patch::patch(&sides)?;
        let mut edges: HashMap<(&str, &str), crate::topo::EdgeId> = HashMap::new();
        let mut add =
            |tool: &mut Brep, from: &'static str, to: &'static str, curve: Curve, range| {
                let e = tool.add_edge(curve, range, vs[from].0, vs[to].0);
                edges.insert((from, to), e);
            };
        for (from, to) in [
            ("w1", "w2"),
            ("w2", "w3"),
            ("w3", "A"),
            ("D", "w1"),
            ("t1", "t2"),
            ("t2", "w3"),
            ("A", "t1"),
            ("b1", "C"),
            ("C", "w2"),
            ("w1", "b1"),
            ("t2", "B"),
        ] {
            let (line, len) = Line::through(pt(from), pt(to))
                .ok_or_else(|| Error::Numerical("a saddle with no room".into()))?;
            add(&mut tool, from, to, Curve::Line(line), (0.0, len));
        }
        // Quarter circles: each fillet's section on the cube, about its axis.
        for (from, to, k) in [("A", "D", 0), ("t1", "B", 1), ("B", "C", 2), ("D", "b1", 3)] {
            let (p, q) = (pt(from), pt(to));
            let axis = &axes[k];
            let centre = axis.origin + axis.axis * (p - axis.origin).dot(axis.axis);
            let x = (p - centre) / self.r;
            let y = (q - centre) / self.r;
            let frame = Frame::new(centre, x.cross(y), x)?;
            let circle = Circle {
                frame,
                radius: self.r,
            };
            add(
                &mut tool,
                from,
                to,
                Curve::Circle(circle),
                (0.0, std::f64::consts::FRAC_PI_2),
            );
        }
        // The convex fillets' ends, the patch's sides along them.
        for (from, to, side) in [("A", "B", &sides.bottom), ("D", "C", &sides.top)] {
            let pts: Vec<Point3> = side
                .0
                .h
                .iter()
                .map(|h| Vec3::new(h[0], h[1], h[2]) / h[3])
                .collect();
            let w: Vec<f64> = side.0.h.iter().map(|h| h[3]).collect();
            let p = pts.len() - 1;
            let knots: Vec<f64> = std::iter::repeat_n(0.0, p + 1)
                .chain(std::iter::repeat_n(1.0, p + 1))
                .collect();
            let curve = NurbsCurve::new(p, knots, pts, Some(w))?;
            add(
                &mut tool,
                from,
                to,
                Curve::Nurbs(Box::new(curve)),
                (0.0, 1.0),
            );
        }
        let walk =
            |cycle: &[&str]| -> Vec<(crate::topo::EdgeId, bool, Option<crate::topo::PCurve>)> {
                (0..cycle.len())
                    .map(|k| {
                        let (u, v) = (cycle[k], cycle[(k + 1) % cycle.len()]);
                        match edges.get(&(u, v)) {
                            Some(&e) => (e, true, None),
                            None => (edges[&(v, u)], false, None),
                        }
                    })
                    .collect()
            };
        let plane = |at: Point3, out: Vec3| -> Result<Surface> {
            Ok(Surface::Plane(crate::geom::Plane {
                frame: Frame::from_axis(at, out)?,
            }))
        };
        let (a, b, c) = (self.a, self.b, self.c);
        let mut faces = Vec::new();
        for (cycle, out, inside) in [
            (vec!["w1", "w2", "w3", "A", "D"], b, [0.5, 1.0, -0.5]),
            (vec!["t1", "t2", "w3", "A"], c, [0.5, 0.0, 1.0]),
            (vec!["b1", "C", "w2", "w1"], -c, [0.0, 0.5, -1.0]),
            (vec!["b1", "w1", "D"], -a, [-1.0, 0.8, -0.8]),
            (vec!["B", "t2", "w3", "w2", "C"], a, [1.0, 0.0, 0.5]),
            (vec!["B", "t2", "t1"], -b, [0.8, -1.0, 0.8]),
        ] {
            let at = self.at(inside);
            faces.push(ToolFace {
                surface: plane(at, out)?,
                out,
                at,
                names: BTreeSet::new(),
                walk: walk(&cycle),
            });
        }
        // The convex fillets inside the cube: between the cube's wall, the line where the fillet
        // leaves its face, and its end.
        for (cycle, k, side, line_end, names) in [
            (vec!["t1", "B", "A"], 1, &sides.bottom, "A", names[1]),
            (vec!["D", "C", "b1"], 3, &sides.top, "C", names[3]),
        ] {
            let axis = &axes[k];
            let radial = |p: Point3| axis.offset(p);
            let q = side.0.d1(0.5).0;
            let foot = axis.origin + axis.axis * (q - axis.origin).dot(axis.axis);
            let mid =
                (radial(q).unit_or_zero() + radial(pt(line_end)).unit_or_zero()).unit_or_zero();
            let at = foot + mid * self.r;
            let frame = Frame::new(axis.origin, axis.axis, radial(pt(line_end)))?;
            faces.push(ToolFace {
                surface: Surface::Cylinder(crate::geom::Cylinder {
                    frame,
                    radius: self.r,
                }),
                out: mid,
                at,
                names: names.clone(),
                walk: walk(&cycle),
            });
        }
        // The patch, facing out as the convex fillet beside it does where they meet.
        let (q, su, sv) = patch.d1(0.5, 0.0);
        let sign = su.cross(sv).dot(axes[1].offset(q)).signum();
        let (at, su, sv) = patch.d1(0.5, 0.5);
        let out = su.cross(sv).unit_or_zero() * sign;
        let all: BTreeSet<FaceName> = names.iter().flat_map(|n| n.iter().cloned()).collect();
        faces.push(ToolFace {
            surface: Surface::Nurbs(Box::new(patch)),
            out,
            at,
            names: all,
            walk: walk(&["A", "B", "C", "D"]),
        });
        assemble(tool, faces, "a saddle's corner")
    }
}

/// `out` with the saddle `s` closed: the cube cut away and the filleted corner joined in its place.
pub(super) fn close(out: &Brep, s: &Saddle, names: [&BTreeSet<FaceName>; 4]) -> Result<Brep> {
    let cube = s.cube()?;
    let facts = shared_edges(out, &cube);
    let cut = boolean::boolean_with(out, &cube, boolean::Op::Difference, &facts)?;
    let corner = s.solid(names)?;
    let joined = match glue(&cut, &corner) {
        Some(j) => j,
        None => {
            let facts = shared_edges(&cut, &corner);
            boolean::boolean_with(&cut, &corner, boolean::Op::Union, &facts)?
        }
    };
    unify(&joined)
}

/// The filleted corner inside the cube of the saddle in the standard frame, radius `r`.
#[cfg(test)]
pub(crate) fn corner(r: f64) -> Result<Brep> {
    let s = Saddle {
        origin: Point3::ZERO,
        a: Vec3::new(1.0, 0.0, 0.0),
        b: Vec3::new(0.0, 1.0, 0.0),
        c: Vec3::new(0.0, 0.0, 1.0),
        r,
        edges: [0, 1, 2, 3],
    };
    let none = BTreeSet::new();
    s.solid([&none; 4])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_saddle_s_corner_is_one_valid_solid_that_scales_with_its_radius() {
        // Inside the cube the body held 4 r³; filleted it holds a little less, the convex fillets
        // taking more than the concave ones give back by the patch's skew.
        let one = crate::measure::volume(&corner(1.0).unwrap());
        let two = crate::measure::volume(&corner(2.0).unwrap());
        assert!((two - 8.0 * one).abs() < 1e-9, "{two} for {}", 8.0 * one);
        assert!(one > 3.9 && one < 4.0, "{one}");
    }
}
