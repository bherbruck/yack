// yack: Drawing-view projection (P3C.2): hidden-line removal of bodies seen along a view dire....
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/projection.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: serde derives and attributes stripped.
//! Drawing-view projection (P3C.2): hidden-line removal of bodies seen along a view direction.
//!
//! [`crate::Kernel::project`] projects bodies orthographically onto a view plane and sorts the
//! projected edges into visible and hidden, and into sharp edges, smooth (tangent) edges and
//! outlines (silhouettes), like the lines of a drawing view. Everything here is plain data (no
//! backend types), so a projection can be cached, sent between threads and drawn anywhere.
//!
//! # 2D frame
//! A [`ViewFrame`] is an eye direction `dir` (from the eye into the scene) and a sheet-right
//! direction `x` (made perpendicular to `dir`). 2D points are in model units (mm) relative to
//! `origin`: `x` along [`ViewFrame::x`], `y` along [`ViewFrame::up`] = `x × dir`. A front view
//! (looking along +Y, `x` = +X) has 2D y = +Z; a top view (`dir` = −Z, `x` = +X) has 2D y = +Y.
//!
//! # Source topology
//! Each projected edge keeps where it came from ([`ProjSource`]): the index of the body in the
//! list passed to `project`, the body's [`EdgeId`] for sharp and smooth edges, and the
//! [`FaceId`] for the outline of a cylinder. Drawing annotations (P3C.3) attach to model
//! topology through these (the caller adds the persistent names, see [`ProjSource::edge_name`]).
//! The backend's HLR reports no identity, so [`attach_sources`] finds it geometrically: a
//! projected piece belongs to the 3D edge whose projection runs through it; where several 3D
//! edges project onto the same line (a box's front and back edges seen from the side), a
//! visible piece takes the one nearest the eye and a hidden piece the farthest.

use nalgebra::{Point2, Point3, Vector3};

use crate::naming::{EdgeName, FaceName};
use crate::{EdgeId, FaceId};

/// Where a view looks from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewFrame {
    /// The model point at 2D (0, 0).
    pub origin: Point3<f64>,
    /// Unit direction of sight, from the eye into the scene.
    pub dir: Vector3<f64>,
    /// Unit direction of 2D +x (perpendicular to `dir`).
    pub x: Vector3<f64>,
}

impl ViewFrame {
    /// A frame looking along `dir` with 2D x along `x` (made perpendicular and unit length).
    pub fn new(dir: Vector3<f64>, x: Vector3<f64>) -> Self {
        let dir = dir.normalize();
        let x = (x - dir * x.dot(&dir)).normalize();
        Self {
            origin: Point3::origin(),
            dir,
            x,
        }
    }

    /// Unit direction of 2D +y: `x × dir`.
    pub fn up(&self) -> Vector3<f64> {
        self.x.cross(&self.dir)
    }

    /// A model point in the view's 2D frame.
    pub fn to_2d(&self, p: &Point3<f64>) -> Point2<f64> {
        let d = p - self.origin;
        Point2::new(d.dot(&self.x), d.dot(&self.up()))
    }

    /// How far a point is along the direction of sight (larger is farther from the eye).
    pub fn depth(&self, p: &Point3<f64>) -> f64 {
        (p - self.origin).dot(&self.dir)
    }

    /// A model direction in the view's 2D frame (not normalized).
    pub fn dir_2d(&self, v: &Vector3<f64>) -> nalgebra::Vector2<f64> {
        nalgebra::Vector2::new(v.dot(&self.x), v.dot(&self.up()))
    }
}

/// How to project.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProjectOptions {
    /// Chordal deviation of the edge polylines (mm).
    pub tolerance: f64,
    /// Compute hidden edges too.
    pub hidden: bool,
}

impl Default for ProjectOptions {
    fn default() -> Self {
        Self {
            tolerance: 0.01,
            hidden: true,
        }
    }
}

/// Seen or hidden behind material.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProjVisibility {
    /// In front of everything.
    Visible,
    /// Behind some face.
    Hidden,
}

/// What kind of line a projected edge is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProjClass {
    /// An edge where two faces meet at an angle.
    Sharp,
    /// An edge between tangent faces (a fillet's boundary): a drawing's tangent edge.
    Smooth,
    /// The silhouette of a curved face (the sides of a cylinder seen side-on).
    Outline,
}

/// The exact 2D curve of a projected edge, where it is a line or a circular arc.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(missing_docs, reason = "the fields are described with each variant")]
pub enum ProjCurve {
    /// A line from `start` to `end`.
    Line {
        start: Point2<f64>,
        end: Point2<f64>,
    },
    /// An arc of `center` and `radius` from `start` through `mid` to `end` (a whole circle when
    /// `full`).
    Arc {
        center: Point2<f64>,
        radius: f64,
        start: Point2<f64>,
        mid: Point2<f64>,
        end: Point2<f64>,
        full: bool,
    },
    /// Anything else (an ellipse, a spline): use the polyline.
    Polyline,
}

/// The model topology a projected edge came from.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ProjSource {
    /// Index of the body in the list given to [`crate::Kernel::project`].
    pub body: usize,
    /// The 3D edge (sharp and smooth edges).
    pub edge: Option<EdgeId>,
    /// The face (outlines of cylinders).
    pub face: Option<FaceId>,
    /// Persistent names, filled in by the caller that knows the body's names (`cadrs_core`).
    pub edge_name: Option<EdgeName>,
    /// The face's persistent name, as `edge_name`.
    pub face_name: Option<FaceName>,
}

/// One projected edge (or a piece of one: HLR splits edges where they go behind something).
#[derive(Debug, Clone, PartialEq)]
pub struct ProjEdge {
    /// Seen or hidden.
    pub visibility: ProjVisibility,
    /// Sharp, smooth or outline.
    pub class: ProjClass,
    /// The exact 2D curve, where it is a line or an arc.
    pub curve: ProjCurve,
    /// The edge as a polyline within the tolerance (at least two points).
    pub points: Vec<Point2<f64>>,
    /// The model edge or face it came from.
    pub source: Option<ProjSource>,
}

impl ProjEdge {
    /// Length of the polyline (exact for lines).
    pub fn length(&self) -> f64 {
        self.points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
    }

}

/// The projected edges of a view.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Projection {
    /// The edges, visible and hidden.
    pub edges: Vec<ProjEdge>,
}

impl Projection {
    /// 2D bounding box `(min, max)` of every edge, or `None` when there are none.
    pub fn bounds(&self) -> Option<(Point2<f64>, Point2<f64>)> {
        let mut it = self.edges.iter().flat_map(|e| e.points.iter());
        let first = *it.next()?;
        Some(it.fold((first, first), |(lo, hi), p| {
            (
                Point2::new(lo.x.min(p.x), lo.y.min(p.y)),
                Point2::new(hi.x.max(p.x), hi.y.max(p.y)),
            )
        }))
    }

    /// The edges of one visibility and class.
    pub fn of(
        &self,
        visibility: ProjVisibility,
        class: ProjClass,
    ) -> impl Iterator<Item = &ProjEdge> {
        self.edges
            .iter()
            .filter(move |e| e.visibility == visibility && e.class == class)
    }
}








#[cfg(test)]
mod tests {
    use super::*;


    #[test]
    fn frames_follow_the_hlr_convention() {
        // Front: looking along +Y with x = +X, 2D y is +Z.
        let f = ViewFrame::new(Vector3::y(), Vector3::x());
        assert!((f.up() - Vector3::z()).norm() < 1e-12);
        assert_eq!(f.to_2d(&Point3::new(3.0, 7.0, 5.0)), Point2::new(3.0, 5.0));
        // Top: looking down (-Z) with x = +X, 2D y is +Y.
        let t = ViewFrame::new(-Vector3::z(), Vector3::x());
        assert!((t.up() - Vector3::y()).norm() < 1e-12);
    }

}
