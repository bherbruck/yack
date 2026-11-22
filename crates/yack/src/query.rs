// yack: queries on `Solid`, routed to springrep. They cannot fail on a valid solid unless they
// take a tolerance or build geometry.
use yack_springrep::{check, distance, draw, measure, mesh, overlap, rays};

use crate::{engine, engine_cadrs, firewall, ladder};
use crate::solid::Body;
use crate::{
    Aabb, Display, DisplayFace, Drawing, Closest, Interference, NearestFace, Issue, Mesh, Plane, Point3, Ray, Result, Side, Solid, Vec3,
};

fn sr_points<P: Into<Point3> + Copy>(points: &[P]) -> impl Iterator<Item = yack_springrep::math::Vec3> + '_ {
    points.iter().map(|&p| engine::p3(p.into()))
}

impl Solid {
    /// The volume (mm³).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert!((b.volume() - 1000.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn volume(&self) -> f64 {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map_or(f64::NAN, |m| m.0),
            None => self.brep().map_or(0.0, measure::volume),
        }
    }

    /// The surface area (mm²).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert!((b.area() - 600.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn area(&self) -> f64 {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map_or(f64::NAN, |m| m.1),
            None => self.brep().map_or(0.0, measure::area),
        }
    }

    /// The volume centroid; `None` for the empty solid.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.centroid(), Some(Point3::new(5.0, 5.0, 5.0)));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn centroid(&self) -> Option<Point3> {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map(|m| Point3::from(m.2)),
            None => self.brep().map(|b| engine::point(measure::mass_properties(b).centre)),
        }
    }

    /// The inertia tensor about the centroid, unit density (mm⁵).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // about the centroid, unit density: m(b² + c²)/12 on the diagonal
    /// let i = b.inertia();
    /// assert!((i[0][0] - 1000.0 * 200.0 / 12.0).abs() < 1e-6 && i[0][1].abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn inertia(&self) -> [[f64; 3]; 3] {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map_or([[f64::NAN; 3]; 3], |m| m.3),
            None => self.brep().map_or([[0.0; 3]; 3], |b| measure::mass_properties(b).inertia),
        }
    }

    /// The bounding box; `None` for the empty solid.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let bb = b.bbox().unwrap();
    /// assert_eq!((bb.min, bb.max), (Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0)));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn bbox(&self) -> Option<Aabb> {
        if let Some(h) = self.held() {
            return engine_cadrs::held_bbox(h).map(|(lo, hi)| Aabb::new(Point3::from(lo), Point3::from(hi)));
        }
        self.brep().map(|b| {
            let bb = measure::bbox(b);
            Aabb::new(engine::point(bb.min), engine::point(bb.max))
        })
    }

    /// Betti numbers: pieces, tunnels, voids.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.betti(), [1, 0, 0]);
    /// let drilled = b.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 12.0)?)?;
    /// assert_eq!(drilled.betti(), [1, 1, 0]); // one tunnel
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn betti(&self) -> [u64; 3] {
        match self.held() {
            Some(h) => engine_cadrs::held_betti(h),
            None => self.brep().map_or([0; 3], measure::betti),
        }
    }

    /// Validity problems; empty for anything yack built.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert!(b.check().is_empty());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn check(&self) -> Vec<Issue> {
        if let Some(h) = self.held() {
            return engine_cadrs::held_check(h).into_iter().map(|detail| Issue { code: "cadrs", detail }).collect();
        }
        self.brep().map_or_else(Vec::new, |b| {
            check::check(b).into_iter().map(|i| Issue { code: "springrep", detail: i.message }).collect()
        })
    }

    /// Where each point lies: `In`, `On` (within `tol::LINEAR`) or `Out`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.classify(&[[5.0, 5.0, 5.0], [10.0, 5.0, 5.0], [15.0, 5.0, 5.0]])?, vec![Side::In, Side::On, Side::Out]);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn classify<P: Into<Point3> + Copy>(&self, points: &[P]) -> Result<Vec<Side>> {
        let op = "classify";
        let Some(b) = self.form(op)? else { return Ok(vec![Side::Out; points.len()]) };
        firewall::run(op, || {
            let caster = rays::Rays::new(b).map_err(|e| engine::error(op, e))?;
            Ok(sr_points(points)
                .map(|p| match caster.side(p) {
                    rays::Side::Inside => Side::In,
                    rays::Side::On => Side::On,
                    rays::Side::Outside => Side::Out,
                })
                .collect())
        })
    }

    /// For each point, the nearest point of the solid with its inside counted, as the protocol's
    /// `closest` has it: a point the solid holds is its own nearest point; one outside gets the
    /// nearest boundary point. Empty for the empty solid.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// // a point outside goes to the nearest boundary point; one inside is its own
    /// assert_eq!(b.closest(&[[15.0, 5.0, 5.0], [5.0, 5.0, 5.0]])?, vec![Point3::new(10.0, 5.0, 5.0), Point3::new(5.0, 5.0, 5.0)]);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn closest<P: Into<Point3> + Copy>(&self, points: &[P]) -> Result<Vec<Point3>> {
        let op = "closest";
        let Some(b) = self.form(op)? else { return Ok(Vec::new()) };
        firewall::run(op, || {
            let nearest = distance::Nearest::solid(b).map_err(|e| engine::error(op, e))?;
            sr_points(points).map(|p| nearest.to(p).map(|c| engine::point(c.on_b)).map_err(|e| engine::error(op, e))).collect()
        })
    }

    /// The outward unit normal at each boundary point; `None` at a crease or off the boundary.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert_eq!(b.normal_at(&[[5.0, 5.0, 10.0]])?, vec![Some(Vec3::new(0.0, 0.0, 1.0))]);
    /// assert_eq!(b.normal_at(&[[5.0, 0.0, 10.0]])?, vec![None]); // a crease
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn normal_at<P: Into<Point3> + Copy>(&self, points: &[P]) -> Result<Vec<Option<Vec3>>> {
        let Some(b) = self.form("normal_at")? else { return Ok(vec![None; points.len()]) };
        let outlines = measure::Outlines::new(b);
        Ok(sr_points(points).map(|p| outlines.normal_at(p, yack_tol::PICK).map(engine::vec)).collect())
    }

    /// The principal curvatures at each boundary point; `None` at a crease or off the boundary.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let ball = sphere(&Frame::WORLD, 2.0)?;
    /// let k = ball.curvature_at(&[[0.0, 0.0, 2.0]])?;
    /// assert!(k[0].is_some_and(|[a, b]| (a.abs() - 0.5).abs() < 1e-9 && (b.abs() - 0.5).abs() < 1e-9));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn curvature_at<P: Into<Point3> + Copy>(&self, points: &[P]) -> Result<Vec<Option<[f64; 2]>>> {
        let Some(b) = self.form("curvature_at")? else { return Ok(vec![None; points.len()]) };
        let outlines = measure::Outlines::new(b);
        Ok(sr_points(points).map(|p| outlines.curvature(p)).collect())
    }

    /// The least distance between the two solids: zero if they touch or overlap, infinite if
    /// either is empty.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let other = block(&Frame::at([15.0, 0.0, 0.0]), [1.0, 1.0, 1.0])?;
    /// assert!((b.distance(&other)? - 5.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn distance(&self, other: &Solid) -> Result<f64> {
        let op = "distance";
        let (Some(a), Some(b)) = (self.form(op)?, other.form(op)?) else { return Ok(f64::INFINITY) };
        firewall::run(op, || {
            distance::closest(a, &distance::Selection::body(a), b, &distance::Selection::body(b))
                .map(|c| c.distance)
                .map_err(|e| engine::error(op, e))
        })
    }

    /// The face nearest `p` and its distance, at any distance (unlike [`face_at`](Solid::face_at),
    /// which picks only within `tol::PICK`). A point nearest an edge goes to the face whose own
    /// region holds the point's foot on that face's surface: just inside a hole's rim, the hole's
    /// wall. `NotFound` for the empty solid.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let near = b.nearest_face([5.0, 5.0, 12.0])?;
    /// assert_eq!(near.face, b.face_at([5.0, 5.0, 10.0])?);
    /// assert!((near.distance - 2.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn nearest_face(&self, p: impl Into<Point3>) -> Result<NearestFace> {
        let op = "nearest_face";
        let p = p.into();
        let b = self.form(op)?.ok_or_else(|| crate::Error::not_found(op, "self", "the solid is empty"))?;
        firewall::run(op, || {
            let (f, d) = yack_springrep::facade::api::nearest_face(b, engine::p3(p))
                .map_err(|e| engine::error(op, e))?
                .ok_or_else(|| crate::Error::not_found(op, "p", "the solid has no faces"))?;
            Ok(NearestFace { face: self.face_id(f), distance: d })
        })
    }

    /// The least distance between the two solids and where it is reached. `Invalid` if either
    /// solid is empty.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let other = block(&Frame::at([15.0, 0.0, 0.0]), [1.0, 1.0, 1.0])?;
    /// let c = b.closest_pair(&other)?;
    /// assert!((c.distance - 5.0).abs() < 1e-9 && c.on_self.x == 10.0 && c.on_other.x == 15.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn closest_pair(&self, other: &Solid) -> Result<Closest> {
        let op = "closest_pair";
        let (a, b) = (self.need(op)?, other.form(op)?.ok_or_else(|| crate::Error::invalid(op, "other", "the solid is empty"))?);
        firewall::run(op, || {
            distance::closest(a, &distance::Selection::body(a), b, &distance::Selection::body(b))
                .map(|c| Closest { distance: c.distance, on_self: engine::point(c.on_a), on_other: engine::point(c.on_b) })
                .map_err(|e| engine::error(op, e))
        })
    }

    /// The volume shared with `other` and the gap between them.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let other = block(&Frame::at([8.0, 0.0, 0.0]), [4.0, 10.0, 10.0])?;
    /// let i = b.interference(&other)?;
    /// assert!((i.volume - 200.0).abs() < 1e-9 && i.gap == 0.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn interference(&self, other: &Solid) -> Result<Interference> {
        let (Some(a), Some(b)) = (self.form("interference")?, other.form("interference")?) else {
            return Ok(Interference { volume: 0.0, gap: f64::INFINITY });
        };
        Ok(match overlap::between(a, b).map_err(|e| engine::error("interference", e))? {
            overlap::Meeting::Shared(v) => Interference { volume: v, gap: 0.0 },
            overlap::Meeting::Touching => Interference { volume: 0.0, gap: 0.0 },
            overlap::Meeting::Apart(d) => Interference { volume: 0.0, gap: d },
        })
    }

    /// The area of the solid's cross-section by `plane` (mm²).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert!((b.section_area(&Plane::new([0.0, 0.0, 5.0], [0.0, 0.0, 1.0])?)? - 100.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn section_area(&self, plane: &Plane) -> Result<f64> {
        let Some(b) = self.form("section_area")? else { return Ok(0.0) };
        yack_springrep::boolean::section_area(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()))
            .map_err(|e| engine::error("section_area", e))
    }

    /// The cross-section by `plane` as closed polylines (each ends on its first point), no point of
    /// a polyline farther than `tolerance` (mm) from the true section.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let loops = b.section(&Plane::new([0.0, 0.0, 5.0], [0.0, 0.0, 1.0])?, 0.01)?;
    /// assert_eq!(loops.len(), 1);
    /// assert_eq!(loops[0].first(), loops[0].last()); // each closed
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn section(&self, plane: &Plane, tolerance: f64) -> Result<Vec<Vec<Point3>>> {
        let Some(b) = self.form("section")? else { return Ok(Vec::new()) };
        let curves = yack_springrep::boolean::section(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()), tolerance)
            .map_err(|e| engine::error("section", e))?;
        Ok(curves.into_iter().map(|c| c.iter().chain(c.first()).map(|&p| engine::point(p)).collect()).collect())
    }

    /// The visible and hidden edges looking along `direction`, as polylines no farther than
    /// `tolerance` (mm) from the true edges, drawn in the view plane: `u` (across) is
    /// `direction × z` made unit (`direction × x` when `direction` runs along z) and `v` (up) is
    /// `u × direction`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let d = b.project(Dir3::Z, 0.01)?;
    /// assert!(!d.visible.is_empty());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn project(&self, direction: impl Into<Vec3>, tolerance: f64) -> Result<Drawing> {
        let op = "project";
        let d = direction.into();
        let (u, v) = yack_math::view_axes(d)?;
        if self.is_empty() {
            return Ok(Drawing::default());
        }
        let springrep = || -> Result<Drawing> {
            let drawn = draw::hidden_lines(self.need(op)?, engine::v3(d), tolerance).map_err(|e| engine::error(op, e))?;
            let flat = |lines: Vec<Vec<yack_springrep::math::Vec3>>| {
                lines.into_iter().map(|l| l.into_iter().map(|p| yack_math::view_point(engine::point(p), u, v)).collect()).collect()
            };
            Ok(Drawing { visible: flat(drawn.visible), hidden: flat(drawn.hidden) })
        };
        // springrep only: cadrs's hidden-line removal gives a whole edge one visibility, so a partly
        // hidden edge comes out wrong (w-m17's measurement); no cadrs rung, not even as a fallback
        ladder::first(op, &[("springrep", &springrep)])
    }

    /// For each ray, the sorted distances at which it crosses the boundary.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let hits = b.raycast(&[Ray::new([-1.0, 5.0, 5.0], [1.0, 0.0, 0.0])?])?;
    /// assert!((hits[0][0] - 1.0).abs() < 1e-9 && (hits[0][1] - 11.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn raycast(&self, rays: &[Ray]) -> Result<Vec<Vec<f64>>> {
        let Some(b) = self.form("raycast")? else { return Ok(vec![Vec::new(); rays.len()]) };
        let caster = rays::Rays::new(b).map_err(|e| engine::error("raycast", e))?;
        rays
            .iter()
            .map(|r| {
                let hits = caster
                    .cast(engine::p3(r.origin()), engine::v3(r.direction().vec()), (0.0, 1e9))
                    .map_err(|e| engine::error("raycast", e))?;
                Ok(hits.iter().map(|h| h.t).collect())
            })
            .collect()
    }

    /// A closed, outward, welded triangle mesh, no triangle farther than `tolerance` (mm) from the
    /// boundary. A fine tolerance on a large curved model can need more points than a face may
    /// have (Unsupported).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let m = b.mesh(0.01)?;
    /// assert!(m.is_watertight() && (m.volume() - 1000.0).abs() < 1e-6);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn mesh(&self, tolerance: f64) -> Result<Mesh> {
        if let Some(h) = self.held() {
            let (positions, triangles) = engine_cadrs::held_mesh("mesh", h, tolerance)?;
            return Ok(Mesh { positions, triangles });
        }
        let Some(b) = self.form("mesh")? else { return Ok(Mesh::default()) };
        let m = mesh::tessellate(b, &mesh::MeshOptions::within(tolerance)).map_err(|e| engine::error("mesh", e))?;
        Ok(Mesh { positions: m.positions.into_iter().map(engine::point).collect(), triangles: m.triangles })
    }

    /// Per-face triangles with exact normals, and crease edges as polylines, none farther than
    /// `tolerance` (mm) from the boundary.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let d = b.display(0.01)?;
    /// assert_eq!(d.faces.len(), 6);
    /// assert!(d.faces.iter().all(|f| f.normals.len() == f.positions.len()));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn display(&self, tolerance: f64) -> Result<Display> {
        let b = match self.body() {
            Body::Empty => return Ok(Display::default()),
            Body::Cadrs(h) => {
                let (faces, edges) = engine_cadrs::held_display("display", h, tolerance)?;
                return Ok(Display {
                    faces: faces.into_iter().map(|(positions, normals, triangles)| DisplayFace { positions, normals, triangles }).collect(),
                    edges,
                });
            }
            Body::Springrep(_) => self.need("display")?,
        };
        let d = mesh::display::display_within(b, tolerance, tolerance).map_err(|e| engine::error("display", e))?;
        let wide = |p: [f32; 3]| p.map(f64::from);
        Ok(Display {
            faces: d
                .faces
                .into_iter()
                .map(|f| DisplayFace {
                    positions: f.positions.into_iter().map(|p| Point3::from(wide(p))).collect(),
                    normals: f.normals.into_iter().map(|n| Vec3::from(wide(n))).collect(),
                    triangles: f.triangles,
                })
                .collect(),
            edges: d.edges.iter().map(|e| e.iter().map(|&p| engine::point(p)).collect()).collect(),
        })
    }
}
