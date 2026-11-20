// yack: small public value types: planes, rays, ids, tags and query results.
use core::fmt;

use yack_springrep::facade::api;

use crate::engine::{self, Brep};
use crate::solid::engine_ids;
use crate::{Aabb, Dir3, Point2, Point3, Result, Solid, Vec3};

/// An oriented plane: a point on it and a unit normal.
///
/// ```
/// use yack::prelude::*;
/// let p = Plane::new([0.0, 0.0, 5.0], [0.0, 0.0, 2.0]).unwrap();
/// assert_eq!(p.normal(), Dir3::Z);
/// assert!(Plane::new([0.0; 3], [0.0; 3]).is_err());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    origin: Point3,
    normal: Dir3,
}

impl Plane {
    /// The plane through `origin` with normal `normal` (made unit). Refuses a zero normal.
    pub fn new(origin: impl Into<Point3>, normal: impl Into<Vec3>) -> Result<Plane> {
        let origin = origin.into();
        if !origin.is_finite() {
            return Err(crate::Error::invalid("Plane::new", "origin", "not finite"));
        }
        let normal = Dir3::new(normal).map_err(|e| e.in_op("Plane::new"))?;
        Ok(Plane { origin, normal })
    }
    /// A point on the plane.
    pub fn origin(&self) -> Point3 {
        self.origin
    }
    /// The unit normal; the plane's front side.
    pub fn normal(&self) -> Dir3 {
        self.normal
    }
}

/// A half-line: an origin and a unit direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    origin: Point3,
    direction: Dir3,
}

impl Ray {
    /// The ray from `origin` along `direction` (made unit). Refuses a zero direction.
    pub fn new(origin: impl Into<Point3>, direction: impl Into<Vec3>) -> Result<Ray> {
        let origin = origin.into();
        if !origin.is_finite() {
            return Err(crate::Error::invalid("Ray::new", "origin", "not finite"));
        }
        let direction = Dir3::new(direction).map_err(|e| e.in_op("Ray::new"))?;
        Ok(Ray { origin, direction })
    }
    /// The start point.
    pub fn origin(&self) -> Point3 {
        self.origin
    }
    /// The unit direction.
    pub fn direction(&self) -> Dir3 {
        self.direction
    }
}

/// A feature name, attached to faces with [`Solid::named`] or to profile segments with
/// [`Pen::label`](crate::Pen::label).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Tag(String);

impl Tag {
    /// The name.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for Tag {
    fn from(s: &str) -> Tag {
        Tag(s.to_string())
    }
}

impl From<String> for Tag {
    fn from(s: String) -> Tag {
        Tag(s)
    }
}

impl fmt::Display for Tag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// Where a point lies relative to a solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Side {
    /// Strictly inside.
    In,
    /// On the boundary, within `tol::LINEAR`.
    On,
    /// Strictly outside.
    Out,
}

macro_rules! id {
    ($name:ident, $letter:literal, $what:literal) => {
        #[doc = concat!("A ", $what, " of one particular solid. Passing it to another solid gives `ForeignId`.")]
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name {
            stamp: u64,
            index: u32,
        }

        impl $name {
            pub(crate) fn new(stamp: u64, index: u32) -> $name {
                $name { stamp, index }
            }
            /// The stamp of the solid this id belongs to.
            pub(crate) fn stamp(self) -> u64 {
                self.stamp
            }
            /// The index within its solid.
            pub fn index(self) -> u32 {
                self.index
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}{}", $letter, self.index)
            }
        }
    };
}

id!(FaceId, "face ", "face");
id!(EdgeId, "edge ", "edge");
id!(VertexId, "vertex ", "vertex");

/// The kind of surface a face lies on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SurfaceKind {
    /// A plane with this outward normal.
    Plane {
        /// Outward unit normal.
        normal: Vec3,
    },
    /// A circular cylinder.
    Cylinder {
        /// Unit axis.
        axis: Vec3,
        /// Radius (mm).
        radius: f64,
    },
    /// A circular cone.
    Cone,
    /// A sphere.
    Sphere {
        /// Radius (mm).
        radius: f64,
    },
    /// A torus.
    Torus,
    /// A swept surface (extrusion, revolution, ruled).
    Swept,
    /// A spline surface.
    Spline,
}

/// The kind of curve an edge lies on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CurveKind {
    /// A straight line.
    Line {
        /// Unit direction, from the edge's first end to its second.
        direction: Vec3,
    },
    /// A circle.
    Circle {
        /// Radius (mm).
        radius: f64,
        /// Unit axis.
        axis: Vec3,
    },
    /// An ellipse.
    Ellipse,
    /// A spline or intersection curve.
    Spline,
}

/// What part a face plays in the operation that made it. Every engine fills the same roles; each
/// variant says which operations produce it. A face can play a role in several operations (a face
/// two operations' faces merged into has one from each); a face an operation made with no
/// meaningful part (a boolean's trimmed copy keeps its original role instead) has none.
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [10.0, 10.0, 4.0])?.named("box");
/// let s = b.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 6.0)?)?;
/// let top = s.face_by("box", Role::End)?; // still box's end cap after the drilling
/// assert_eq!(s.face(top)?.normal().map(|n| n.z > 0.99), Some(true));
/// assert_eq!(s.faces_by("box", Role::Side("x_min".into())).len(), 1);
/// # Ok::<(), yack::Error>(())
/// ```
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Role {
    /// The cap a sweep starts from: the profile's own face of `extrude`, `sweep`, `sweep_path`,
    /// a partial `revolve` or `screw`; a loft's first section; the z = 0 face of `block`,
    /// `cylinder` and `cone`.
    Start,
    /// The cap it ends on: the far face of the same operations; the top of `block`, `cylinder` and
    /// `cone` (none at a cone's apex).
    End,
    /// A side swept from one profile segment (`extrude`, `revolve`, `screw`, `sweep`,
    /// `sweep_path`): its label from [`Pen::label`](crate::Pen::label), or `s{n}` for the n-th
    /// segment counted through the profile's loops, outer first. A `block`'s sides are `x_min`,
    /// `x_max`, `y_min`, `y_max` along its frame; a `cylinder`'s, `cone`'s or loft's sides are
    /// `side`.
    Side(Tag),
    /// The one face of a `sphere` or a `torus`.
    Skin,
    /// The round a fillet (`fillet`, `fillet_radii`, `fillet_variable`, `fillet_conic`,
    /// `fillet_g2`, `fillet_setback*`) made along the call's n-th edge.
    Round(u32),
    /// The bevel a chamfer (`chamfer`, `chamfer_two`, `chamfer_distances`) made along the call's
    /// n-th edge.
    Bevel(u32),
    /// A corner patch where a blend's rounds or bevels meet.
    Corner,
    /// A wall of the cavity a `shell` made.
    Inner,
}

/// A read-only view of one face, from [`Solid::face`] or [`Solid::faces_where`].
#[derive(Clone, Debug)]
pub struct FaceRef<'a> {
    pub(crate) id: FaceId,
    pub(crate) kind: SurfaceKind,
    pub(crate) normal: Option<Vec3>,
    pub(crate) area: f64,
    pub(crate) centroid: Point3,
    pub(crate) bbox: Aabb,
    pub(crate) tags: Vec<Tag>,
    pub(crate) roles: Vec<(Tag, Role)>,
    pub(crate) brep: &'a Brep,
    pub(crate) face: engine_ids::Face,
}

impl FaceRef<'_> {
    /// The face's id.
    pub fn id(&self) -> FaceId {
        self.id
    }
    /// The kind of surface.
    pub fn kind(&self) -> SurfaceKind {
        self.kind
    }
    /// The outward normal if the face is planar.
    pub fn normal(&self) -> Option<Vec3> {
        self.normal
    }
    /// The area (mm²).
    pub fn area(&self) -> f64 {
        self.area
    }
    /// The area centroid (it may lie off the face).
    pub fn centroid(&self) -> Point3 {
        self.centroid
    }
    /// The bounding box.
    pub fn bbox(&self) -> Aabb {
        self.bbox
    }
    /// The feature names the face carries ([`Solid::named`]), sorted, without repeats: a face two
    /// operations' faces merged into carries both.
    pub fn tags(&self) -> &[Tag] {
        &self.tags
    }
    /// The face's role in each named operation it came from, as `(tag, role)`, sorted.
    pub fn roles(&self) -> &[(Tag, Role)] {
        &self.roles
    }
    /// The face's role in the operation named `tag`, if it plays one there.
    pub fn role(&self, tag: impl Into<Tag>) -> Option<Role> {
        let tag = tag.into();
        self.roles.iter().find(|(t, _)| *t == tag).map(|(_, r)| r.clone())
    }
    /// The outward unit normal of the face's own surface at the point of it nearest `p` (defined
    /// at creases too, unlike [`Solid::normal_at`]); `None` where the surface is singular.
    pub fn normal_at(&self, p: impl Into<Point3>) -> Option<Vec3> {
        api::face_normal_at(self.brep, self.face, engine::p3(p.into())).map(engine::vec)
    }
    /// An even `nu` × `nv` grid over the face's surface parameter box, rows along u. The box spans
    /// the face's outline, so on a trimmed face some points lie off the face.
    pub fn sample(&self, nu: usize, nv: usize) -> Vec<Vec<Point3>> {
        api::face_grid(self.brep, self.face, nu, nv).into_iter().map(|r| r.into_iter().map(engine::point).collect()).collect()
    }
}

/// A read-only view of one edge, from [`Solid::edge`] or [`Solid::edges_where`].
#[derive(Clone, Debug)]
pub struct EdgeRef<'a> {
    pub(crate) id: EdgeId,
    pub(crate) kind: CurveKind,
    pub(crate) ends: [Point3; 2],
    pub(crate) convex: Option<bool>,
    pub(crate) length: f64,
    pub(crate) midpoint: Point3,
    pub(crate) uses: usize,
    pub(crate) brep: &'a Brep,
    pub(crate) edge: engine_ids::Edge,
}

impl EdgeRef<'_> {
    /// The edge's id.
    pub fn id(&self) -> EdgeId {
        self.id
    }
    /// The kind of curve.
    pub fn kind(&self) -> CurveKind {
        self.kind
    }
    /// The two end points (equal for a closed edge).
    pub fn ends(&self) -> [Point3; 2] {
        self.ends
    }
    /// Whether the solid is convex (`Some(true)`) or concave (`Some(false)`) across the edge at
    /// its midpoint; `None` where its faces meet tangentially, or more than two faces use it.
    pub fn convex(&self) -> Option<bool> {
        self.convex
    }
    /// The length (mm).
    pub fn length(&self) -> f64 {
        self.length
    }
    /// The point halfway along the edge.
    pub fn midpoint(&self) -> Point3 {
        self.midpoint
    }
    /// How many face uses the edge has: 2 on a manifold edge, 4 where two pieces touch along it.
    /// A face that runs along both sides of the edge counts twice.
    pub fn uses(&self) -> usize {
        self.uses
    }
    /// `n` points spaced evenly in the edge's parameter, ends included.
    pub fn points(&self, n: usize) -> Vec<Point3> {
        api::edge_points(self.brep, self.edge, n).into_iter().map(engine::point).collect()
    }
}

/// A read-only view of one vertex, from [`Solid::vertex`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VertexRef {
    pub(crate) id: VertexId,
    pub(crate) point: Point3,
}

impl VertexRef {
    /// The vertex's id.
    pub fn id(&self) -> VertexId {
        self.id
    }
    /// Where it is.
    pub fn point(&self) -> Point3 {
        self.point
    }
}

/// How the faces of a solid came from its inputs.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct History {
    _private: (),
}

impl History {
    /// Whether no face has a recorded origin.
    pub fn is_empty(&self) -> bool {
        true
    }
}

/// One validity problem found by [`Solid::check`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Issue {
    /// A short stable code, e.g. `"open-shell"`.
    pub code: &'static str,
    /// What is wrong, with ids and numbers.
    pub detail: String,
}

/// The two sides of a [`Solid::split`].
#[derive(Clone, Debug)]
pub struct Split {
    /// The part on the plane normal's side.
    pub front: Solid,
    /// The part on the other side.
    pub back: Solid,
}

/// How two solids meet: shared volume and the gap between them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Interference {
    /// The volume both hold (mm³).
    pub volume: f64,
    /// The distance between them; zero where they touch or overlap.
    pub gap: f64,
}

/// A view along a direction: visible and hidden edges as 2-D polylines, in the protocol's view
/// axes (`u` across, `v` up).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Drawing {
    /// Polylines the viewer sees.
    pub visible: Vec<Vec<Point2>>,
    /// Polylines the solid hides.
    pub hidden: Vec<Vec<Point2>>,
}

/// A closed, outward, welded triangle mesh.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    /// Vertex positions.
    pub positions: Vec<Point3>,
    /// Triangles as three indices into `positions`, counter-clockwise seen from outside.
    pub triangles: Vec<[u32; 3]>,
}

impl Mesh {
    /// The volume the mesh encloses (mm³), assuming it is closed and outward.
    pub fn volume(&self) -> f64 {
        yack_math::mesh_volume(&self.positions, &self.triangles)
    }

    /// Whether every directed triangle edge is matched by exactly one opposite one: the mesh is
    /// closed and consistently oriented.
    pub fn is_watertight(&self) -> bool {
        let mut count: std::collections::BTreeMap<(u32, u32), i64> = std::collections::BTreeMap::new();
        for &[a, b, c] in &self.triangles {
            for (p, q) in [(a, b), (b, c), (c, a)] {
                *count.entry((p, q)).or_default() += 1;
            }
        }
        count.iter().all(|(&(p, q), &n)| n == 1 && count.get(&(q, p)) == Some(&1))
    }
}

/// One face of a [`Display`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DisplayFace {
    /// Vertex positions.
    pub positions: Vec<Point3>,
    /// The exact surface's outward unit normal at each position.
    pub normals: Vec<Vec3>,
    /// Triangles over `positions`.
    pub triangles: Vec<[u32; 3]>,
}

/// A solid as a viewer draws it: each face meshed on its own, plus crease edges as polylines.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Display {
    /// The faces, in [`Solid::faces`] order.
    pub faces: Vec<DisplayFace>,
    /// The edge polylines.
    pub edges: Vec<Vec<Point3>>,
}
