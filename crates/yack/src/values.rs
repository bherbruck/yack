// yack: small public value types: planes, rays, ids, tags and query results.
use core::fmt;
use core::marker::PhantomData;

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
            #[allow(dead_code)] // VertexId has no lookup yet
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

/// A read-only view of one face, from [`Solid::face`] or [`Solid::faces_where`].
#[derive(Clone, Copy, Debug)]
pub struct FaceRef<'a> {
    id: FaceId,
    kind: SurfaceKind,
    normal: Option<Vec3>,
    area: f64,
    centroid: Point3,
    bbox: Aabb,
    solid: PhantomData<&'a Solid>,
}

impl FaceRef<'_> {
    pub(crate) fn new(
        id: FaceId,
        kind: SurfaceKind,
        normal: Option<Vec3>,
        area: f64,
        centroid: Point3,
        bbox: Aabb,
    ) -> FaceRef<'static> {
        FaceRef { id, kind, normal, area, centroid, bbox, solid: PhantomData }
    }
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
}

/// A read-only view of one edge, from [`Solid::edge`] or [`Solid::edges_where`].
#[derive(Clone, Copy, Debug)]
pub struct EdgeRef<'a> {
    id: EdgeId,
    kind: CurveKind,
    ends: [Point3; 2],
    convex: Option<bool>,
    length: f64,
    midpoint: Point3,
    solid: PhantomData<&'a Solid>,
}

impl EdgeRef<'_> {
    pub(crate) fn new(
        id: EdgeId,
        kind: CurveKind,
        ends: [Point3; 2],
        convex: Option<bool>,
        length: f64,
        midpoint: Point3,
    ) -> EdgeRef<'static> {
        EdgeRef { id, kind, ends, convex, length, midpoint, solid: PhantomData }
    }
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
    /// `Some(true)` where the solid is convex across the edge; `None` otherwise (concave, tangent
    /// or mixed; telling these apart is still to come).
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
