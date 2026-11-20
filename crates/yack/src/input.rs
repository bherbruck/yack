// yack: 2-D profiles, paths, splines and motions: the inputs of the make:: constructors.
use yack_math::planar::{self, LoopShape, Piece};

use crate::{Angle, Error, Point2, Point3, Result, Tag, Vec3};

/// One piece of a planar [`Loop`], in the profile's own (x, y) plane.
#[derive(Clone, Debug, PartialEq)]
pub enum Segment {
    /// A straight line from `a` to `b`.
    Line {
        /// Start.
        a: Point2,
        /// End.
        b: Point2,
    },
    /// A circular arc. `sweep > 0` turns counter-clockwise.
    Arc {
        /// Centre.
        center: Point2,
        /// Radius (mm), `> 0`.
        radius: f64,
        /// Angle of the start point from +x.
        start: Angle,
        /// Signed sweep from the start.
        sweep: Angle,
    },
    /// A conic (rational quadratic) from `a` to `b`, tangent to `a→apex` and `apex→b`.
    Conic {
        /// Start.
        a: Point2,
        /// Where the end tangents meet.
        apex: Point2,
        /// End.
        b: Point2,
        /// The apex weight, `> 0`: `< 1` ellipse, `1` parabola, `> 1` hyperbola.
        weight: f64,
    },
    /// A curvature-continuous corner blend of the given order from `a` to `b` about `apex`.
    Blend {
        /// Start.
        a: Point2,
        /// The corner being blended.
        apex: Point2,
        /// End.
        b: Point2,
        /// Continuity order.
        order: u32,
        /// Shape weight.
        weight: f64,
    },
    /// A clamped NURBS curve.
    Nurbs {
        /// Degree, `≥ 1`.
        degree: u32,
        /// Knot vector, `ctrl.len() + degree + 1` non-decreasing values, clamped at both ends.
        knots: Vec<f64>,
        /// Control points.
        ctrl: Vec<Point2>,
        /// Optional positive weights, one per control point.
        weights: Option<Vec<f64>>,
    },
    /// A periodic B-spline that forms the whole loop on its own.
    PeriodicSpline {
        /// Degree, `≥ 1`.
        degree: u32,
        /// Knot vector, non-decreasing.
        knots: Vec<f64>,
        /// Control points (not repeated).
        ctrl: Vec<Point2>,
    },
}

/// A closed planar loop of [`Segment`]s, each starting where the previous one ends.
///
/// ```
/// use yack::prelude::*;
/// let square = Loop::polygon(&[[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]).unwrap();
/// assert!(square.is_ccw());
/// let slot = Loop::starting_at([0.0, 0.0]).line_to([10.0, 0.0]).arc_to([10.0, 4.0], [10.0, 2.0])
///     .line_to([0.0, 4.0]).arc_to([0.0, 0.0], [0.0, 2.0]).close().unwrap();
/// assert_eq!(slot.segments().len(), 4);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    segments: Vec<Segment>,
    labels: Vec<Option<Tag>>,
    shape: LoopShape,
}

/// A loop being drawn: start with [`Loop::starting_at`], end with [`Pen::close`].
///
/// Errors are kept until `close`, which reports the first.
#[derive(Clone, Debug)]
#[must_use = "a pen draws nothing until `close()`"]
pub struct Pen {
    start: Point2,
    at: Point2,
    segments: Vec<Segment>,
    labels: Vec<Option<Tag>>,
    error: Option<Error>,
}

/// A planar region: one counter-clockwise outer [`Loop`] and clockwise holes inside it.
///
/// ```
/// use yack::prelude::*;
/// let washer = Profile::new(
///     Loop::new(vec![Segment::Arc { center: [0.0, 0.0].into(), radius: 5.0, start: deg(0.0), sweep: deg(360.0) }])?,
///     vec![Loop::new(vec![Segment::Arc { center: [0.0, 0.0].into(), radius: 2.0, start: deg(0.0), sweep: deg(-360.0) }])?],
/// )?;
/// assert_eq!(washer.holes().len(), 1);
/// assert!(Profile::rectangle([0.0, 0.0], [0.0, 1.0]).is_err());
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Profile {
    outer: Loop,
    holes: Vec<Loop>,
}

fn knots_ok(op: &'static str, degree: u32, knots: &[f64], n: usize, clamped: bool) -> Result<()> {
    let d = usize::try_from(degree).unwrap_or(usize::MAX);
    if degree == 0 || n <= d {
        return Err(Error::invalid(op, "degree", format!("degree {degree} needs more than {degree} control points, got {n}")));
    }
    if !knots.iter().all(|k| k.is_finite()) || knots.windows(2).any(|w| matches!(w, [a, b] if b < a)) {
        return Err(Error::invalid(op, "knots", "the knots must be finite and non-decreasing"));
    }
    if clamped {
        let want = n.saturating_add(d).saturating_add(1);
        if knots.len() != want {
            return Err(Error::invalid(op, "knots", format!("{} knots given, {want} needed", knots.len())));
        }
        let same = |run: &[f64]| run.windows(2).all(|w| matches!(w, [a, b] if a >= b));
        let head = knots.get(..=d).unwrap_or(&[]);
        let tail = knots.get(knots.len().saturating_sub(d + 1)..).unwrap_or(&[]);
        if !same(head) || !same(tail) {
            return Err(Error::new(crate::ErrorKind::Unsupported, op, "only clamped knot vectors are supported"));
        }
    }
    Ok(())
}

fn piece(op: &'static str, s: &Segment, alone: bool) -> Result<Piece> {
    Ok(match s {
        Segment::Line { a, b } => Piece::Line { a: *a, b: *b },
        Segment::Arc { center, radius, start, sweep } => {
            Piece::Arc { center: *center, radius: *radius, start: *start, sweep: *sweep }
        }
        Segment::Conic { a, apex, b, weight } => Piece::Conic { a: *a, apex: *apex, b: *b, weight: *weight },
        Segment::Blend { a, apex, b, order, weight } => {
            if *order == 0 || !weight.is_finite() || *weight <= 0.0 {
                return Err(Error::invalid(op, "segments", "a blend needs order ≥ 1 and a positive weight"));
            }
            Piece::Hull(vec![*a, *apex, *b])
        }
        Segment::Nurbs { degree, knots, ctrl, weights } => {
            knots_ok(op, *degree, knots, ctrl.len(), true)?;
            if let Some(w) = weights
                && (w.len() != ctrl.len() || !w.iter().all(|w| w.is_finite() && *w > 0.0))
            {
                return Err(Error::invalid(op, "segments", "NURBS weights must be positive, one per control point"));
            }
            Piece::Hull(ctrl.clone())
        }
        Segment::PeriodicSpline { degree, knots, ctrl } => {
            if !alone {
                return Err(Error::invalid(op, "segments", "a periodic spline must be the loop's only segment"));
            }
            knots_ok(op, *degree, knots, ctrl.len(), false)?;
            let mut hull = ctrl.clone();
            hull.extend(ctrl.first().copied());
            Piece::Hull(hull)
        }
    })
}

impl Loop {
    /// A loop from segments. Refuses gaps between consecutive segments, zero-length pieces, zero
    /// area, and pieces that cross or touch, naming the segments.
    pub fn new(segments: Vec<Segment>) -> Result<Loop> {
        let labels = vec![None; segments.len()];
        Loop::labelled("Loop::new", segments, labels)
    }

    fn labelled(op: &'static str, segments: Vec<Segment>, labels: Vec<Option<Tag>>) -> Result<Loop> {
        let alone = segments.len() == 1;
        let pieces = segments.iter().map(|s| piece(op, s, alone)).collect::<Result<Vec<_>>>()?;
        let shape = planar::check_loop(op, &pieces)?;
        Ok(Loop { segments, labels, shape })
    }

    /// A polygon through `corners`, in the order given (closed back to the first).
    pub fn polygon(corners: &[[f64; 2]]) -> Result<Loop> {
        if corners.len() < 3 {
            return Err(Error::invalid("Loop::polygon", "corners", format!("{} corners given, 3 needed", corners.len())));
        }
        let pts: Vec<Point2> = corners.iter().map(|&c| Point2::from(c)).collect();
        let segments = pts.iter().zip(pts.iter().cycle().skip(1)).map(|(&a, &b)| Segment::Line { a, b }).collect();
        let labels = vec![None; corners.len()];
        Loop::labelled("Loop::polygon", segments, labels)
    }

    /// Start drawing a loop at `p`.
    pub fn starting_at(p: impl Into<Point2>) -> Pen {
        let p = p.into();
        Pen { start: p, at: p, segments: Vec::new(), labels: Vec::new(), error: None }
    }

    /// The segments.
    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }
    /// Each segment's label, if one was given with [`Pen::label`].
    pub fn labels(&self) -> &[Option<Tag>] {
        &self.labels
    }
    /// Whether the loop runs counter-clockwise.
    pub fn is_ccw(&self) -> bool {
        self.shape.area > 0.0
    }
}

impl Pen {
    fn push(mut self, s: Segment, end: Point2) -> Pen {
        self.segments.push(s);
        self.labels.push(None);
        self.at = end;
        self
    }

    /// A line from the current point to `p`.
    pub fn line_to(self, p: impl Into<Point2>) -> Pen {
        let b = p.into();
        let a = self.at;
        self.push(Segment::Line { a, b }, b)
    }

    /// A counter-clockwise arc from the current point to `end` about `center`. Both ends must
    /// lie at the same distance from `center`.
    pub fn arc_to(mut self, end: impl Into<Point2>, center: impl Into<Point2>) -> Pen {
        let (end, center) = (end.into(), center.into());
        match planar::arc_to("Pen::arc_to", self.at, end, center) {
            Ok((radius, start, sweep)) => self.push(Segment::Arc { center, radius, start, sweep }, end),
            Err(e) => {
                self.error.get_or_insert(e);
                self
            }
        }
    }

    /// A conic from the current point to `end`, tangent to the lines through `apex`, with apex
    /// weight `weight`.
    pub fn conic_to(self, end: impl Into<Point2>, apex: impl Into<Point2>, weight: f64) -> Pen {
        let (b, apex) = (end.into(), apex.into());
        let a = self.at;
        self.push(Segment::Conic { a, apex, b, weight }, b)
    }

    /// Labels the last segment drawn; the faces it sweeps carry the label in history.
    pub fn label(mut self, l: impl Into<Tag>) -> Pen {
        match self.labels.last_mut() {
            Some(slot) => *slot = Some(l.into()),
            None => {
                self.error.get_or_insert(Error::invalid("Pen::label", "l", "no segment drawn yet to label"));
            }
        }
        self
    }

    /// Closes the loop with a line back to the start (unless already there) and validates it.
    pub fn close(self) -> Result<Loop> {
        if let Some(e) = self.error {
            return Err(e);
        }
        let mut pen = self;
        if !planar::coincident(pen.at, pen.start) {
            let start = pen.start;
            pen = pen.line_to(start);
        }
        Loop::labelled("Pen::close", pen.segments, pen.labels)
    }
}

impl Profile {
    /// A profile from an outer loop and holes. The outer loop must run counter-clockwise and the
    /// holes clockwise, strictly inside it and apart from each other; anything else is refused,
    /// never repaired.
    pub fn new(outer: Loop, holes: Vec<Loop>) -> Result<Profile> {
        let shapes: Vec<LoopShape> = holes.iter().map(|h| h.shape.clone()).collect();
        planar::check_profile("Profile::new", &outer.shape, &shapes)?;
        Ok(Profile { outer, holes })
    }

    /// The rectangle with corners `min` and `max` (`min < max` on both axes).
    pub fn rectangle(min: impl Into<Point2>, max: impl Into<Point2>) -> Result<Profile> {
        let (lo, hi) = (min.into(), max.into());
        if !(lo.x < hi.x && lo.y < hi.y) {
            return Err(Error::invalid(
                "Profile::rectangle",
                "max",
                format!("max ({}, {}) is not above and right of min ({}, {})", hi.x, hi.y, lo.x, lo.y),
            ));
        }
        Profile::new(Loop::polygon(&[[lo.x, lo.y], [hi.x, lo.y], [hi.x, hi.y], [lo.x, hi.y]])?, Vec::new())
    }

    /// The disc of `radius` about `center`.
    pub fn circle(center: impl Into<Point2>, radius: f64) -> Result<Profile> {
        let arc = Segment::Arc {
            center: center.into(),
            radius,
            start: yack_math::rad(0.0),
            sweep: yack_math::rad(core::f64::consts::TAU),
        };
        let outer = Loop::labelled("Profile::circle", vec![arc], vec![None])?;
        Profile::new(outer, Vec::new())
    }

    /// The polygon through `corners`, in either order (it is made counter-clockwise).
    pub fn polygon(corners: &[[f64; 2]]) -> Result<Profile> {
        let mut l = Loop::polygon(corners)?;
        if !l.is_ccw() {
            let reversed: Vec<[f64; 2]> = corners.iter().rev().copied().collect();
            l = Loop::polygon(&reversed)?;
        }
        Profile::new(l, Vec::new())
    }

    /// The outer loop.
    pub fn outer(&self) -> &Loop {
        &self.outer
    }
    /// The holes.
    pub fn holes(&self) -> &[Loop] {
        &self.holes
    }
}

/// One piece of a planar sweep path, starting along the profile frame's z axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathPiece {
    /// Straight on for `length` mm.
    Line {
        /// Length (mm), `> 0`.
        length: f64,
    },
    /// A circular turn of `radius` by `angle` (positive turns toward the frame's +x).
    Arc {
        /// Radius (mm), `> 0`.
        radius: f64,
        /// Turn angle.
        angle: Angle,
    },
}

/// A clamped B-spline curve in space.
///
/// ```
/// use yack::prelude::*;
/// let s = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]]).unwrap();
/// assert_eq!(s.degree(), 3);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Spline3 {
    degree: u32,
    knots: Vec<f64>,
    ctrl: Vec<Point3>,
}

impl Spline3 {
    /// A clamped B-spline of `degree` with `ctrl.len() + degree + 1` non-decreasing knots.
    pub fn new(degree: u32, knots: Vec<f64>, ctrl: Vec<Point3>) -> Result<Spline3> {
        knots_ok("Spline3::new", degree, &knots, ctrl.len(), true)?;
        if !ctrl.iter().all(|p| p.is_finite()) {
            return Err(Error::invalid("Spline3::new", "ctrl", "a control point is not finite"));
        }
        Ok(Spline3 { degree, knots, ctrl })
    }

    /// The Bézier curve with these control points (degree = points − 1).
    pub fn bezier(ctrl: &[[f64; 3]]) -> Result<Spline3> {
        let n = ctrl.len();
        let degree = u32::try_from(n.saturating_sub(1)).unwrap_or(0);
        let mut knots = vec![0.0; n];
        knots.extend(core::iter::repeat_n(1.0, n));
        let pts = ctrl.iter().map(|&p| Point3::from(p)).collect();
        Spline3::new(degree, knots, pts).map_err(|e| e.in_op("Spline3::bezier"))
    }

    /// The degree.
    pub fn degree(&self) -> u32 {
        self.degree
    }
    /// The knots.
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }
    /// The control points.
    pub fn ctrl(&self) -> &[Point3] {
        &self.ctrl
    }
}

/// How a profile's frame turns as it moves along a path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FrameLaw {
    /// Rotation-minimising (no twist about the tangent).
    RotationMinimising,
    /// The frame keeps its orientation; it only translates.
    Fixed,
    /// The frame's x axis keeps facing this direction.
    Toward(Vec3),
}

/// A motion of a solid through space, for [`Solid::swept`](crate::Solid::swept).
#[derive(Clone, Debug, PartialEq)]
pub enum Motion {
    /// A straight translation by `by`.
    Slide {
        /// The displacement.
        by: Vec3,
    },
    /// A rotation about an axis.
    Turn {
        /// A point on the axis.
        axis_point: Point3,
        /// The axis direction.
        axis: Vec3,
        /// The angle turned (counter-clockwise looking down `axis`).
        angle: Angle,
    },
    /// A screw: rotation about an axis while rising along it.
    Screw {
        /// A point on the axis.
        axis_point: Point3,
        /// The axis direction.
        axis: Vec3,
        /// The angle turned.
        angle: Angle,
        /// Rise along the axis per full turn (mm).
        rise_per_turn: f64,
    },
    /// Along a spline path, with the frame following `law`.
    Along {
        /// The path, starting at the solid's current position.
        path: Spline3,
        /// How the solid turns along the path.
        law: FrameLaw,
    },
    /// These motions one after another.
    Then(Vec<Motion>),
}
