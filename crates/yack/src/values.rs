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
///
/// ```
/// use yack::prelude::*;
/// let p = Plane::new([1.0, 2.0, 3.0], [0.0, 0.0, 1.0])?;
/// assert_eq!((p.origin(), p.normal()), (Point3::new(1.0, 2.0, 3.0), Dir3::Z));
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    origin: Point3,
    normal: Dir3,
}

impl Plane {
    /// The plane through `origin` with normal `normal` (made unit). Refuses a zero normal.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let p = Plane::new([0.0, 0.0, 5.0], [0.0, 0.0, 2.0])?; // the normal made unit
    /// assert_eq!((p.origin(), p.normal().vec()), (Point3::new(0.0, 0.0, 5.0), Vec3::new(0.0, 0.0, 1.0)));
    /// assert!(Plane::new([0.0; 3], [0.0; 3]).is_err());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn new(origin: impl Into<Point3>, normal: impl Into<Vec3>) -> Result<Plane> {
        let origin = origin.into();
        if !origin.is_finite() {
            return Err(crate::Error::invalid("Plane::new", "origin", "not finite"));
        }
        let normal = Dir3::new(normal).map_err(|e| e.in_op("Plane::new"))?;
        Ok(Plane { origin, normal })
    }
    /// A point on the plane.
    ///
    /// (Its example is on [`Plane`].)
    pub fn origin(&self) -> Point3 {
        self.origin
    }
    /// The unit normal; the plane's front side.
    ///
    /// (Its example is on [`Plane`].)
    pub fn normal(&self) -> Dir3 {
        self.normal
    }
}

/// A half-line: an origin and a unit direction.
///
/// ```
/// use yack::prelude::*;
/// let r = Ray::new([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
/// assert_eq!((r.origin(), r.direction()), (Point3::new(1.0, 0.0, 0.0), Dir3::Y));
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    origin: Point3,
    direction: Dir3,
}

impl Ray {
    /// The ray from `origin` along `direction` (made unit). Refuses a zero direction.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let r = Ray::new([0.0; 3], [3.0, 0.0, 0.0])?; // the direction made unit
    /// assert_eq!((r.origin(), r.direction().vec()), (Point3::ORIGIN, Vec3::new(1.0, 0.0, 0.0)));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn new(origin: impl Into<Point3>, direction: impl Into<Vec3>) -> Result<Ray> {
        let origin = origin.into();
        if !origin.is_finite() {
            return Err(crate::Error::invalid("Ray::new", "origin", "not finite"));
        }
        let direction = Dir3::new(direction).map_err(|e| e.in_op("Ray::new"))?;
        Ok(Ray { origin, direction })
    }
    /// The start point.
    ///
    /// (Its example is on [`Ray`].)
    pub fn origin(&self) -> Point3 {
        self.origin
    }
    /// The unit direction.
    ///
    /// (Its example is on [`Ray`].)
    pub fn direction(&self) -> Dir3 {
        self.direction
    }
}

/// A name. One type serves two purposes:
///
/// - a **feature name**: [`Solid::named`] gives it to the faces the operation that made the solid
///   created; [`Solid::tag`] reads it back, [`Solid::faces_tagged`] finds those faces, and
///   [`Solid::faces_by`] the ones playing a [`Role`] in that feature;
/// - a **segment label**: [`Pen::label`](crate::Pen::label) gives it to a profile segment, and the
///   side that segment sweeps plays [`Role::Side`] with that label.
///
/// ```
/// use yack::prelude::*;
/// assert_eq!(Tag::from("boss").as_str(), "boss");
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Tag(String);

impl Tag {
    /// The name.
    ///
    /// (Its example is on [`Tag`].)
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

/// How a thickened sheet closes where its faces part at a crease.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Join {
    /// The moved faces extended until they meet (a mitre), adding no round.
    Sharp,
    /// A cylinder of the thickness about each crease, a ball about each corner.
    Round,
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
            ///
            /// ```
            /// use yack::prelude::*;
            /// let b = block(&Frame::WORLD, [1.0, 1.0, 1.0])?;
            /// let indices: Vec<u32> = b.faces().iter().map(|f| f.index()).collect();
            /// assert_eq!(indices, (0..6).collect::<Vec<u32>>());
            /// # Ok::<(), yack::Error>(())
            /// ```
            pub fn index(self) -> u32 {
                self.index
            }
        }

        impl From<$name> for u64 {
            fn from(id: $name) -> u64 {
                u64::from(id.index)
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
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
/// let f = b.face(b.face_at([5.0, 5.0, 10.0])?)?.clone();
/// assert_eq!(f.id(), b.face_at([5.0, 5.0, 10.0])?);
/// assert!(matches!(f.kind(), SurfaceKind::Plane { .. }) && f.normal() == Some(Vec3::new(0.0, 0.0, 1.0)));
/// assert!((f.area() - 100.0).abs() < 1e-9 && (f.centroid() - Point3::new(5.0, 5.0, 10.0)).length() < 1e-9);
/// assert_eq!((f.bbox().min.z, f.bbox().max.z), (10.0, 10.0));
/// let named = b.named("box");
/// let g = named.face(named.face_at([5.0, 5.0, 10.0])?)?;
/// assert_eq!(g.tags(), &[Tag::from("box")]);
/// assert!(g.roles().iter().all(|(t, _)| t.as_str() == "box") && g.role("other").is_none());
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone)]
pub struct FaceRef<'a> {
    pub(crate) id: FaceId,
    pub(crate) kind: SurfaceKind,
    pub(crate) normal: Option<Vec3>,
    /// Measured on first use (exact, as before): a view read only for its kind costs no quadrature.
    pub(crate) area: std::sync::OnceLock<f64>,
    pub(crate) centroid: std::sync::OnceLock<Point3>,
    pub(crate) bbox: std::sync::OnceLock<Aabb>,
    pub(crate) tags: Vec<Tag>,
    pub(crate) roles: Vec<(Tag, Role)>,
    pub(crate) at: FaceAt<'a>,
}

impl core::fmt::Debug for FaceRef<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FaceRef")
            .field("id", &self.id)
            .field("kind", &self.kind)
            .field("normal", &self.normal)
            .field("area", &self.area())
            .field("centroid", &self.centroid())
            .field("bbox", &self.bbox())
            .field("tags", &self.tags)
            .field("roles", &self.roles)
            .finish_non_exhaustive()
    }
}

impl core::fmt::Debug for EdgeRef<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EdgeRef")
            .field("id", &self.id)
            .field("kind", &self.kind)
            .field("ends", &self.ends)
            .field("convex", &self.convex)
            .field("length", &self.length)
            .field("midpoint", &self.midpoint)
            .field("uses", &self.uses)
            .finish_non_exhaustive()
    }
}

// Views are shared across threads as freely as before the lazy facts (auto traits are API).
const _: () = {
    const fn send_sync<T: Send + Sync>() {}
    send_sync::<FaceRef<'static>>();
    send_sync::<EdgeRef<'static>>();
};

/// Which engine's face a [`FaceRef`] reads its geometry from.
#[derive(Clone, Copy)]
pub(crate) enum FaceAt<'a> {
    Springrep(&'a Brep, engine_ids::Face),
    /// A face of a solid held in cadrs form, by cadrs face index.
    Cadrs(&'a crate::engine_cadrs::Held, u64),
}


/// Which engine's edge an [`EdgeRef`] reads its geometry from.
#[derive(Clone, Copy)]
pub(crate) enum EdgeAt<'a> {
    Springrep(&'a Brep, engine_ids::Edge),
    /// An edge of a solid held in cadrs form, by cadrs edge index.
    Cadrs(&'a crate::engine_cadrs::Held, u64),
}



impl FaceRef<'_> {
    /// The face's id.
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn id(&self) -> FaceId {
        self.id
    }
    /// The kind of surface.
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn kind(&self) -> SurfaceKind {
        self.kind
    }
    /// The outward normal if the face is planar.
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn normal(&self) -> Option<Vec3> {
        self.normal
    }
    /// The area (mm²).
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn area(&self) -> f64 {
        *self.area.get_or_init(|| match self.at {
            FaceAt::Springrep(b, f) => yack_springrep::measure::face_area(b, f),
            FaceAt::Cadrs(h, f) => crate::engine_cadrs::held_face_facts("FaceRef::area", h, f).map_or(f64::NAN, |x| x.area),
        })
    }
    /// The area centroid (it may lie off the face).
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn centroid(&self) -> Point3 {
        *self.centroid.get_or_init(|| match self.at {
            FaceAt::Springrep(b, f) => engine::point(yack_springrep::measure::face_centroid(b, f)),
            FaceAt::Cadrs(h, f) => crate::engine_cadrs::held_face_facts("FaceRef::centroid", h, f)
                .map_or(Point3::new(f64::NAN, f64::NAN, f64::NAN), |x| x.centroid),
        })
    }
    /// The bounding box.
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn bbox(&self) -> Aabb {
        *self.bbox.get_or_init(|| match self.at {
            FaceAt::Springrep(b, f) => {
                let bb = yack_springrep::measure::face_bbox(b, f);
                Aabb::new(engine::point(bb.min), engine::point(bb.max))
            }
            FaceAt::Cadrs(h, f) => crate::engine_cadrs::held_face_facts("FaceRef::bbox", h, f).map_or(
                Aabb::new(Point3::new(f64::NAN, f64::NAN, f64::NAN), Point3::new(f64::NAN, f64::NAN, f64::NAN)),
                |x| x.bbox,
            ),
        })
    }
    /// The feature names the face carries ([`Solid::named`]), sorted, without repeats: a face two
    /// operations' faces merged into carries both.
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn tags(&self) -> &[Tag] {
        &self.tags
    }
    /// The face's role in each named operation it came from, as `(tag, role)`, sorted.
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn roles(&self) -> &[(Tag, Role)] {
        &self.roles
    }
    /// The face's role in the operation named `tag`, if it plays one there.
    ///
    /// (Its example is on [`FaceRef`].)
    pub fn role(&self, tag: impl Into<Tag>) -> Option<Role> {
        let tag = tag.into();
        self.roles.iter().find(|(t, _)| *t == tag).map(|(_, r)| r.clone())
    }
    /// The outward unit normal of the face's own surface at the point of it nearest `p` (defined
    /// at creases too, unlike [`Solid::normal_at`]); `None` where the surface is singular.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let top = b.face(b.face_at([5.0, 5.0, 10.0])?)?;
    /// assert_eq!(top.normal_at([5.0, 5.0, 10.0]), Some(Vec3::new(0.0, 0.0, 1.0)));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn normal_at(&self, p: impl Into<Point3>) -> Option<Vec3> {
        match self.at {
            FaceAt::Springrep(b, f) => api::face_normal_at(b, f, engine::p3(p.into())).map(engine::vec),
            FaceAt::Cadrs(h, f) => crate::engine_cadrs::held_face_normal_at(h, f, p.into()),
        }
    }
    /// The nearest point of this face alone to `p` (the rest of the solid ignored).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let top = b.face(b.face_at([5.0, 5.0, 10.0])?)?;
    /// assert!((top.closest([5.0, 5.0, 20.0])? - Point3::new(5.0, 5.0, 10.0)).length() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn closest(&self, p: impl Into<Point3>) -> Result<Point3> {
        match self.at {
            FaceAt::Springrep(b, f) => {
                api::face_closest(b, f, engine::p3(p.into())).map(engine::point).map_err(|e| engine::error("FaceRef::closest", e))
            }
            FaceAt::Cadrs(h, f) => crate::engine_cadrs::held_face_closest(h, f, p.into()),
        }
    }
    /// An even `nu` × `nv` grid over the face's surface parameter box, rows along u. The box spans
    /// the face's outline, so on a trimmed face some points lie off the face.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let grid = b.face(b.face_at([5.0, 5.0, 10.0])?)?.sample(3, 3);
    /// assert!(grid.len() == 3 && grid.iter().all(|row| row.len() == 3 && row.iter().all(|p| p.z == 10.0)));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn sample(&self, nu: usize, nv: usize) -> Vec<Vec<Point3>> {
        match self.at {
            FaceAt::Springrep(b, f) => api::face_grid(b, f, nu, nv).into_iter().map(|r| r.into_iter().map(engine::point).collect()).collect(),
            FaceAt::Cadrs(h, f) => crate::engine_cadrs::held_face_grid(h, f, nu, nv),
        }
    }
}

/// A read-only view of one edge, from [`Solid::edge`] or [`Solid::edges_where`].
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
/// let e = b.edge(b.edge_at([5.0, 0.0, 10.0])?)?;
/// assert_eq!(e.id(), b.edge_at([5.0, 0.0, 10.0])?);
/// assert!(matches!(e.kind(), CurveKind::Line { .. }) && e.convex() == Some(true) && e.uses() == 2);
/// assert!((e.length() - 10.0).abs() < 1e-9 && e.midpoint() == Point3::new(5.0, 0.0, 10.0));
/// let [t0, t1] = e.range();
/// assert!(t0 < t1 && e.ends().iter().all(|p| p.y == 0.0 && p.z == 10.0));
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone)]
pub struct EdgeRef<'a> {
    pub(crate) id: EdgeId,
    pub(crate) kind: CurveKind,
    pub(crate) ends: [Point3; 2],
    pub(crate) convex: Option<bool>,
    pub(crate) length: f64,
    pub(crate) midpoint: Point3,
    pub(crate) uses: usize,
    pub(crate) at: EdgeAt<'a>,
}

impl EdgeRef<'_> {
    /// The edge's id.
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn id(&self) -> EdgeId {
        self.id
    }
    /// The kind of curve.
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn kind(&self) -> CurveKind {
        self.kind
    }
    /// The two end points (equal for a closed edge).
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn ends(&self) -> [Point3; 2] {
        self.ends
    }
    /// Whether the solid is convex (`Some(true)`) or concave (`Some(false)`) across the edge at
    /// its midpoint; `None` where its faces meet tangentially, or more than two faces use it.
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn convex(&self) -> Option<bool> {
        self.convex
    }
    /// The length (mm).
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn length(&self) -> f64 {
        self.length
    }
    /// The point halfway along the edge.
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn midpoint(&self) -> Point3 {
        self.midpoint
    }
    /// How many face uses the edge has: 2 on a manifold edge, 4 where two pieces touch along it.
    /// A face that runs along both sides of the edge counts twice.
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn uses(&self) -> usize {
        self.uses
    }
    /// The edge's curve parameter range `[t0, t1]`, `t0 < t1`, running from `ends()[0]` to
    /// `ends()[1]`.
    ///
    /// (Its example is on [`EdgeRef`].)
    pub fn range(&self) -> [f64; 2] {
        match self.at {
            EdgeAt::Springrep(b, e) => {
                let (t0, t1) = b.edge(e).range;
                [t0, t1]
            }
            EdgeAt::Cadrs(h, e) => crate::engine_cadrs::held_edge_range(h, e),
        }
    }
    /// The point of the edge's curve at parameter `t` (any `t`; within [`range`](EdgeRef::range)
    /// it lies on the edge).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let e = b.edge(b.edge_at([5.0, 0.0, 10.0])?)?;
    /// let [t0, t1] = e.range();
    /// let near = |a: Point3, b: Point3| (a - b).length() < 1e-9;
    /// assert!(near(e.point(t0), e.ends()[0]) && near(e.point(t1), e.ends()[1]));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn point(&self, t: f64) -> Point3 {
        match self.at {
            EdgeAt::Springrep(b, e) => api::edge_point(b, e, t).map_or(self.ends[0], engine::point),
            EdgeAt::Cadrs(h, e) => crate::engine_cadrs::held_edge_point(h, e, t).unwrap_or(self.ends[0]),
        }
    }
    /// `n` points spaced evenly in the edge's parameter, ends included.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// let e = b.edge(b.edge_at([5.0, 0.0, 10.0])?)?;
    /// let pts = e.points(3);
    /// assert!(pts.len() == 3 && (pts[1] - e.midpoint()).length() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn points(&self, n: usize) -> Vec<Point3> {
        match self.at {
            EdgeAt::Springrep(b, e) => api::edge_points(b, e, n).into_iter().map(engine::point).collect(),
            EdgeAt::Cadrs(h, e) => crate::engine_cadrs::held_edge_points(h, e, n),
        }
    }
}

/// A read-only view of one vertex, from [`Solid::vertex`].
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
/// let v = b.vertex(b.vertices()[0])?;
/// assert_eq!(v.id(), b.vertices()[0]);
/// assert!(b.classify(&[v.point()])?[0] == Side::On);
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VertexRef {
    pub(crate) id: VertexId,
    pub(crate) point: Point3,
}

impl VertexRef {
    /// The vertex's id.
    ///
    /// (Its example is on [`VertexRef`].)
    pub fn id(&self) -> VertexId {
        self.id
    }
    /// Where it is.
    ///
    /// (Its example is on [`VertexRef`].)
    pub fn point(&self) -> Point3 {
        self.point
    }
}

/// Where two solids come nearest, from [`Solid::closest_pair`] (its example is there).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Closest {
    /// The least distance between them (mm); zero where they touch or overlap.
    pub distance: f64,
    /// The point of the solid asked that is nearest the other.
    pub on_self: Point3,
    /// The point of the other solid that is nearest it.
    pub on_other: Point3,
}

/// The face of a solid nearest a point, from [`Solid::nearest_face`] (its example is there).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NearestFace {
    /// The face.
    pub face: FaceId,
    /// Its distance from the point (mm).
    pub distance: f64,
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

/// A view along a direction: visible and hidden edges as 2-D polylines in the view plane's
/// `u` (across) and `v` (up) axes, as [`Solid::project`] defines them.
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert!((b.mesh(0.01)?.volume() - 1000.0).abs() < 1e-6);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn volume(&self) -> f64 {
        yack_math::mesh_volume(&self.positions, &self.triangles)
    }

    /// Whether every directed triangle edge is matched by as many opposite ones (counting each
    /// use): the mesh is closed and consistently oriented. An edge where a solid touches itself,
    /// used by four triangles in two opposite pairs, is closed.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    /// assert!(b.mesh(0.01)?.is_watertight());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn is_watertight(&self) -> bool {
        let mut count: std::collections::BTreeMap<(u32, u32), i64> = std::collections::BTreeMap::new();
        for &[a, b, c] in &self.triangles {
            for (p, q) in [(a, b), (b, c), (c, a)] {
                *count.entry((p, q)).or_default() += 1;
            }
        }
        count.iter().all(|(&(p, q), &n)| count.get(&(q, p)) == Some(&n))
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
