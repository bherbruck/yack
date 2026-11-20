// yack: queries on `Solid`, routed to springrep. They cannot fail on a valid solid unless they
// take a tolerance or build geometry.
use yack_springrep::{check, distance, draw, measure, mesh, overlap, rays};

use crate::engine;
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
        self.brep().map_or(0.0, measure::volume)
    }

    /// The surface area (mm²).
    pub fn area(&self) -> f64 {
        self.brep().map_or(0.0, measure::area)
    }

    /// The volume centroid; `None` for the empty solid.
    pub fn centroid(&self) -> Option<Point3> {
        self.brep().map(|b| engine::point(measure::mass_properties(b).centre))
    }

    /// The inertia tensor about the centroid, unit density (mm⁵).
    pub fn inertia(&self) -> [[f64; 3]; 3] {
        self.brep().map_or([[0.0; 3]; 3], |b| measure::mass_properties(b).inertia)
    }

    /// The bounding box; `None` for the empty solid.
    pub fn bounding_box(&self) -> Option<Aabb> {
        self.brep().map(|b| {
            let bb = measure::bbox(b);
            Aabb::new(engine::point(bb.min), engine::point(bb.max))
        })
    }

    /// Betti numbers: pieces, tunnels, voids.
    pub fn betti(&self) -> [u64; 3] {
        self.brep().map_or([0; 3], measure::betti)
    }

    /// Validity problems; empty for anything yack built.
    pub fn check(&self) -> Vec<Issue> {
        self.brep().map_or_else(Vec::new, |b| {
            check::check(b).into_iter().map(|i| Issue { code: "springrep", detail: i.message }).collect()
        })
    }

    /// Where each point lies: `In`, `On` (within `tol::LINEAR`) or `Out`. Empty if the solid's
    /// ray caster could not be built (never for a valid solid).
    pub fn classify<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Vec<Side> {
        let Some(b) = self.brep() else { return vec![Side::Out; pts_.len()] };
        let Ok(caster) = rays::Rays::new(b) else { return Vec::new() };
        pts(pts_)
            .map(|p| match caster.side(p) {
                rays::Side::Inside => Side::In,
                rays::Side::On => Side::On,
                rays::Side::Outside => Side::Out,
            })
            .collect()
    }

    /// The closest boundary point to each point; empty for the empty solid, or if a search fails.
    pub fn closest<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Vec<Point3> {
        let Some(b) = self.brep() else { return Vec::new() };
        let Ok(nearest) = distance::Nearest::solid(b) else { return Vec::new() };
        pts(pts_).map(|p| nearest.to(p).map(|c| engine::point(c.on_b))).collect::<core::result::Result<_, _>>().unwrap_or_default()
    }

    /// The outward unit normal at each boundary point; `None` at a crease or off the boundary.
    pub fn normal_at<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Vec<Option<Vec3>> {
        let Some(b) = self.brep() else { return vec![None; pts_.len()] };
        let outlines = measure::Outlines::new(b);
        pts(pts_).map(|p| outlines.normal_at(p, yack_tol::PICK).map(engine::vec)).collect()
    }

    /// The principal curvatures at each boundary point; `None` at a crease or off the boundary.
    pub fn curvature_at<P: Into<Point3> + Copy>(&self, pts_: &[P]) -> Vec<Option<[f64; 2]>> {
        let Some(b) = self.brep() else { return vec![None; pts_.len()] };
        let outlines = measure::Outlines::new(b);
        pts(pts_).map(|p| outlines.curvature(p)).collect()
    }

    /// The least distance between the two solids: zero if they touch or overlap, infinite if
    /// either is empty, NaN if the search fails.
    pub fn distance(&self, other: &Solid) -> f64 {
        match (self.brep(), other.brep()) {
            (Some(a), Some(b)) => {
                distance::closest(a, &distance::Selection::body(a), b, &distance::Selection::body(b))
                    .map_or(f64::NAN, |c| c.distance)
            }
            _ => f64::INFINITY,
        }
    }

    /// The volume shared with `other` and the gap between them.
    pub fn interference(&self, other: &Solid) -> Result<Interference> {
        let (Some(a), Some(b)) = (self.brep(), other.brep()) else {
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
        let Some(b) = self.brep() else { return Ok(0.0) };
        measure::section_area(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()))
            .map_err(|e| engine::error("section_area", e))
    }

    /// The cross-section by `plane` as closed polylines (each ends on its first point) with chord
    /// error at most `chord`.
    pub fn section(&self, plane: &Plane, chord: f64) -> Result<Vec<Vec<Point3>>> {
        let Some(b) = self.brep() else { return Ok(Vec::new()) };
        let curves = measure::section(b, engine::p3(plane.origin()), engine::v3(plane.normal().vec()), chord)
            .map_err(|e| engine::error("section", e))?;
        Ok(curves.into_iter().map(|c| c.iter().chain(c.first()).map(|&p| engine::point(p)).collect()).collect())
    }

    /// The visible and hidden edges looking along `direction`, in the protocol's view axes.
    pub fn project(&self, direction: impl Into<Vec3>, chord: f64) -> Result<Drawing> {
        let d = direction.into();
        let (u, v) = yack_math::view_axes(d)?;
        let Some(b) = self.brep() else { return Ok(Drawing::default()) };
        let drawn = draw::hidden_lines(b, engine::v3(d), chord).map_err(|e| engine::error("project", e))?;
        let flat = |lines: Vec<Vec<yack_springrep::math::Vec3>>| {
            lines.into_iter().map(|l| l.into_iter().map(|p| yack_math::view_point(engine::point(p), u, v)).collect()).collect()
        };
        Ok(Drawing { visible: flat(drawn.visible), hidden: flat(drawn.hidden) })
    }

    /// For each ray, the sorted distances at which it crosses the boundary.
    pub fn raycast(&self, rays_: &[Ray]) -> Result<Vec<Vec<f64>>> {
        let Some(b) = self.brep() else { return Ok(vec![Vec::new(); rays_.len()]) };
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
        let Some(b) = self.brep() else { return Ok(Mesh::default()) };
        let m = mesh::tessellate(b, &mesh::MeshOptions::within(chord)).map_err(|e| engine::error("mesh", e))?;
        Ok(Mesh { positions: m.positions.into_iter().map(engine::point).collect(), triangles: m.triangles })
    }

    /// Per-face triangles with exact normals, and crease edges as polylines.
    pub fn display(&self, chord: f64) -> Result<Display> {
        let b = match self.body() {
            Body::Empty => return Ok(Display::default()),
            Body::Springrep(b) => b,
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
