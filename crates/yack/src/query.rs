// yack: queries on `Solid`, routed to springrep. They cannot fail on a valid solid unless they
// take a tolerance or build geometry.
use yack_springrep::{check, distance, draw, measure, mesh, overlap, rays};

use crate::{engine, engine_cadrs, firewall, ladder};
use crate::solid::Body;
use crate::{
    Aabb, Display, DisplayFace, Drawing, Interference, Issue, Mesh, Plane, Point3, Ray, Result, Side, Solid, Vec3,
};

fn pts<P: Into<Point3> + Copy>(pts: &[P]) -> impl Iterator<Item = yack_springrep::math::Vec3> + '_ {
    pts.iter().map(|&p| engine::p3(p.into()))
}

impl Solid {
    /// The volume (mm³).
    pub fn volume(&self) -> f64 {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map_or(f64::NAN, |m| m.0),
            None => self.brep().map_or(0.0, measure::volume),
        }
    }

    /// The surface area (mm²).
    pub fn area(&self) -> f64 {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map_or(f64::NAN, |m| m.1),
            None => self.brep().map_or(0.0, measure::area),
        }
    }

    /// The volume centroid; `None` for the empty solid.
    pub fn centroid(&self) -> Option<Point3> {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map(|m| Point3::from(m.2)),
            None => self.brep().map(|b| engine::point(measure::mass_properties(b).centre)),
        }
    }

    /// The inertia tensor about the centroid, unit density (mm⁵).
    pub fn inertia(&self) -> [[f64; 3]; 3] {
        match self.held() {
            Some(h) => engine_cadrs::held_mass(h).map_or([[f64::NAN; 3]; 3], |m| m.3),
            None => self.brep().map_or([[0.0; 3]; 3], |b| measure::mass_properties(b).inertia),
        }
    }

    /// The bounding box; `None` for the empty solid.
    pub fn bounding_box(&self) -> Option<Aabb> {
        if let Some(h) = self.held() {
            return engine_cadrs::held_bbox(h).map(|(lo, hi)| Aabb::new(Point3::from(lo), Point3::from(hi)));
        }
        self.brep().map(|b| {
            let bb = measure::bbox(b);
            Aabb::new(engine::point(bb.min), engine::point(bb.max))
        })
    }

    /// Betti numbers: pieces, tunnels, voids.
    pub fn betti(&self) -> [u64; 3] {
        match self.held() {
            Some(h) => engine_cadrs::held_betti(h),
            None => self.brep().map_or([0; 3], measure::betti),
        }
    }

    /// Validity problems; empty for anything yack built.
    pub fn check(&self) -> Vec<Issue> {
        if let Some(h) = self.held() {
            return engine_cadrs::held_check(h).into_iter().map(|detail| Issue { code: "cadrs", detail }).collect();
        }
        self.brep().map_or_else(Vec::new, |b| {
            check::check(b).into_iter().map(|i| Issue { code: "springrep", detail: i.message }).collect()
        })
    }

    /// Where each point lies: `In`, `On` (within `tol::LINEAR`) or `Out`.
    pub fn classify<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Result<Vec<Side>> {
        let op = "classify";
        let Some(b) = self.form(op)? else { return Ok(vec![Side::Out; pts_.len()]) };
        firewall::run(op, || {
            let caster = rays::Rays::new(b).map_err(|e| engine::error(op, e))?;
            Ok(pts(pts_)
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
    pub fn closest<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Result<Vec<Point3>> {
        let op = "closest";
        let Some(b) = self.form(op)? else { return Ok(Vec::new()) };
        firewall::run(op, || {
            let nearest = distance::Nearest::solid(b).map_err(|e| engine::error(op, e))?;
            pts(pts_).map(|p| nearest.to(p).map(|c| engine::point(c.on_b)).map_err(|e| engine::error(op, e))).collect()
        })
    }

    /// The outward unit normal at each boundary point; `None` at a crease or off the boundary.
    pub fn normal_at<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Result<Vec<Option<Vec3>>> {
        let Some(b) = self.form("normal_at")? else { return Ok(vec![None; pts_.len()]) };
        let outlines = measure::Outlines::new(b);
        Ok(pts(pts_).map(|p| outlines.normal_at(p, yack_tol::PICK).map(engine::vec)).collect())
    }

    /// The principal curvatures at each boundary point; `None` at a crease or off the boundary.
    pub fn curvature_at<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Result<Vec<Option<[f64; 2]>>> {
        let Some(b) = self.form("curvature_at")? else { return Ok(vec![None; pts_.len()]) };
        let outlines = measure::Outlines::new(b);
        Ok(pts(pts_).map(|p| outlines.curvature(p)).collect())
    }

    /// The least distance between the two solids: zero if they touch or overlap, infinite if
    /// either is empty.
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
    /// surface it is nearer; `NotFound` for the empty solid.
    pub fn nearest_face(&self, p: impl Into<Point3>) -> Result<(crate::FaceId, f64)> {
        let op = "nearest_face";
        let p = p.into();
        let b = self.form(op)?.ok_or_else(|| crate::Error::not_found(op, "self", "the solid is empty"))?;
        firewall::run(op, || {
            let (f, d) = yack_springrep::facade::api::nearest_face(b, engine::p3(p))
                .map_err(|e| engine::error(op, e))?
                .ok_or_else(|| crate::Error::not_found(op, "p", "the solid has no faces"))?;
            Ok((self.face_id(f), d))
        })
    }

    /// The least distance between the two solids and where it is reached: `(distance, on self,
    /// on other)`. `Invalid` if either solid is empty.
    pub fn closest_pair(&self, other: &Solid) -> Result<(f64, Point3, Point3)> {
        let op = "closest_pair";
        let (a, b) = (self.need(op)?, other.form(op)?.ok_or_else(|| crate::Error::invalid(op, "other", "the solid is empty"))?);
        firewall::run(op, || {
            distance::closest(a, &distance::Selection::body(a), b, &distance::Selection::body(b))
                .map(|c| (c.distance, engine::point(c.on_a), engine::point(c.on_b)))
                .map_err(|e| engine::error(op, e))
        })
    }

    /// The volume shared with `other` and the gap between them.
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
    pub fn section_area(&self, plane: &Plane) -> Result<f64> {
        let Some(b) = self.form("section_area")? else { return Ok(0.0) };
        yack_springrep::boolean::section_area(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()))
            .map_err(|e| engine::error("section_area", e))
    }

    /// The cross-section by `plane` as closed polylines (each ends on its first point) with chord
    /// error at most `chord`.
    pub fn section(&self, plane: &Plane, chord: f64) -> Result<Vec<Vec<Point3>>> {
        let Some(b) = self.form("section")? else { return Ok(Vec::new()) };
        let curves = yack_springrep::boolean::section(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()), chord)
            .map_err(|e| engine::error("section", e))?;
        Ok(curves.into_iter().map(|c| c.iter().chain(c.first()).map(|&p| engine::point(p)).collect()).collect())
    }

    /// The visible and hidden edges looking along `direction`, in the protocol's view axes.
    pub fn project(&self, direction: impl Into<Vec3>, chord: f64) -> Result<Drawing> {
        let op = "project";
        let d = direction.into();
        let (u, v) = yack_math::view_axes(d)?;
        if self.is_empty() {
            return Ok(Drawing::default());
        }
        let springrep = || -> Result<Drawing> {
            let drawn = draw::hidden_lines(self.need(op)?, engine::v3(d), chord).map_err(|e| engine::error(op, e))?;
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
    pub fn raycast(&self, rays_: &[Ray]) -> Result<Vec<Vec<f64>>> {
        let Some(b) = self.form("raycast")? else { return Ok(vec![Vec::new(); rays_.len()]) };
        let caster = rays::Rays::new(b).map_err(|e| engine::error("raycast", e))?;
        rays_
            .iter()
            .map(|r| {
                let hits = caster
                    .cast(engine::p3(r.origin()), engine::v3(r.direction().vec()), (0.0, 1e9))
                    .map_err(|e| engine::error("raycast", e))?;
                Ok(hits.iter().map(|h| h.t).collect())
            })
            .collect()
    }

    /// A closed, outward, welded triangle mesh with chord error at most `chord`.
    pub fn mesh(&self, chord: f64) -> Result<Mesh> {
        let Some(b) = self.form("mesh")? else { return Ok(Mesh::default()) };
        let m = mesh::tessellate(b, &mesh::MeshOptions::within(chord)).map_err(|e| engine::error("mesh", e))?;
        Ok(Mesh { positions: m.positions.into_iter().map(engine::point).collect(), triangles: m.triangles })
    }

    /// Per-face triangles with exact normals, and crease edges as polylines.
    pub fn display(&self, chord: f64) -> Result<Display> {
        let b = match self.body() {
            Body::Empty => return Ok(Display::default()),
            _ => self.need("display")?,
        };
        let d = mesh::display::display_within(b, chord, chord).map_err(|e| engine::error("display", e))?;
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
