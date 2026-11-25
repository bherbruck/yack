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
    /// A periodic B-spline that forms the whole loop on its own, in the protocol's convention:
    /// `n` control points followed by the first `degree` of them again, and
    /// `ctrl.len() + degree + 1` knots `u₋ₚ … u_{n+p}` with `u_{i+n} = u_i + period`. Only cubics
    /// are built yet. [`Loop::through`] makes one through given points.
    PeriodicSpline {
        /// Degree, `≥ 1`.
        degree: u32,
        /// Knot vector, non-decreasing.
        knots: Vec<f64>,
        /// Control points, the first `degree` repeated at the end.
        ctrl: Vec<Point2>,
    },
}

impl Segment {
    /// This segment scaled by `s` about the plane's origin: its points and lengths (angles,
    /// weights and knots kept), multiplied as the protocol's client scales a loft's top.
    fn scaled(&self, s: f64) -> Segment {
        let pt = |p: &Point2| yack_math::scale2(*p, s);
        match self {
            Segment::Line { a, b } => Segment::Line { a: pt(a), b: pt(b) },
            Segment::Arc { center, radius, start, sweep } => {
                Segment::Arc { center: pt(center), radius: yack_math::scale_length(*radius, s), start: *start, sweep: *sweep }
            }
            Segment::Conic { a, apex, b, weight } => Segment::Conic { a: pt(a), apex: pt(apex), b: pt(b), weight: *weight },
            Segment::Blend { a, apex, b, order, weight } => {
                Segment::Blend { a: pt(a), apex: pt(apex), b: pt(b), order: *order, weight: *weight }
            }
            Segment::Nurbs { degree, knots, ctrl, weights } => {
                Segment::Nurbs { degree: *degree, knots: knots.clone(), ctrl: ctrl.iter().map(pt).collect(), weights: weights.clone() }
            }
            Segment::PeriodicSpline { degree, knots, ctrl } => {
                Segment::PeriodicSpline { degree: *degree, knots: knots.clone(), ctrl: ctrl.iter().map(pt).collect() }
            }
        }
    }
}

/// A closed planar loop of [`Segment`]s, each starting where the previous one ends.
///
/// ```
/// use yack::prelude::*;
/// let square = Loop::polygon(&[[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]).unwrap();
/// assert!(square.is_ccw());
/// let slot = Loop::starting_at([0.0, 0.0]).line_to([10.0, 0.0]).arc_ccw_to([10.0, 4.0], [10.0, 2.0])
///     .line_to([0.0, 4.0]).arc_ccw_to([0.0, 0.0], [0.0, 2.0]).close().unwrap();
/// // a clockwise (concave) bite out of the top edge
/// let bitten = Loop::starting_at([0.0, 0.0]).line_to([10.0, 0.0]).line_to([10.0, 6.0]).line_to([7.0, 6.0])
///     .arc_cw_to([3.0, 6.0], [5.0, 6.0]).line_to([0.0, 6.0]).close().unwrap();
/// assert!(bitten.is_ccw());
/// assert_eq!(slot.segments().len(), 4);
/// ```
///
/// ```
/// use yack::prelude::*;
/// let l = Loop::starting_at([0.0, 0.0]).line_to([4.0, 0.0]).label("floor").line_to([0.0, 3.0]).close()?;
/// assert_eq!(l.segments().len(), 3);
/// assert_eq!(l.labels(), &[Some(Tag::from("floor")), None, None]);
/// assert!(l.is_ccw());
/// # Ok::<(), yack::Error>(())
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
///
/// ```
/// use yack::prelude::*;
/// let outer = Loop::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])?;
/// let hole = Loop::polygon(&[[4.0, 4.0], [4.0, 6.0], [6.0, 6.0], [6.0, 4.0]])?;
/// let p = Profile::new(outer, vec![hole])?;
/// assert_eq!((p.outer().segments().len(), p.holes().len()), (4, 1));
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
    /// This loop with every corner between two segments filleted at `radius`, convex and concave
    /// alike (corners already tangent kept). Refused where a round does not fit between its
    /// neighbours; a neighbour is never trimmed past its end.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let sq = Loop::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])?;
    /// assert_eq!(sq.rounded(2.0)?.segments().len(), 8); // four sides, four corner arcs
    /// assert!(sq.rounded(6.0).is_err()); // a round that does not fit is refused
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn rounded(&self, radius: f64) -> Result<Loop> {
        let op = "Loop::rounded";
        let segs = crate::engine::loop_segments(op, self)?;
        let out = yack_profile::loops::round_loop(&segs, radius).map_err(|e| crate::engine::error(op, e))?;
        Loop::new(crate::engine::segments_back(op, out)?)
    }

    /// This loop offset by `distance`: outward (away from the region it bounds when it runs
    /// counter-clockwise) for a positive distance, inward for a negative one; corners it grows
    /// round become arcs about them (a tool's path round a part). An offset that would cross
    /// itself is refused, never trimmed.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let sq = Loop::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])?;
    /// let out = Profile::new(sq.offset(1.0)?, vec![])?; // its corners grown round: 140 + π
    /// let plate = extrude(&Frame::WORLD, &out, 1.0)?;
    /// assert!((plate.volume() - (140.0 + std::f64::consts::PI)).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn offset(&self, distance: f64) -> Result<Loop> {
        let op = "Loop::offset";
        let segs = crate::engine::loop_segments(op, self)?;
        let out = yack_profile::loops::offset_loop(&segs, distance).map_err(|e| crate::engine::error(op, e))?;
        Loop::new(crate::engine::segments_back(op, out)?)
    }

    /// The closed loop round a chain of circles, taken in order counter-clockwise, each joined to
    /// the next by their common tangent: a positive radius is a tip (the loop wraps round the
    /// circle's outside), a negative one a fillet (it curves in round the circle's inside). See
    /// [`Profile::tangent_chain`] for the profile it bounds.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // two tips joined by their outer tangents: a stadium
    /// let slot = Loop::tangent_chain(&[([0.0, 0.0].into(), 1.0), ([5.0, 0.0].into(), 1.0)])?;
    /// assert_eq!(slot.segments().len(), 4);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn tangent_chain(circles: &[(Point2, f64)]) -> Result<Loop> {
        let op = "Loop::tangent_chain";
        let ends = yack_math::planar::tangent_chain(op, circles)?;
        let Some(&(_, start)) = ends.first() else { return Err(Error::invalid(op, "circles", "no circles")) };
        let mut pen = Loop::starting_at(start);
        for ((c, r), (arrive, leave)) in circles.iter().zip(&ends).skip(1).chain(circles.iter().zip(&ends).take(1)) {
            pen = pen.line_to(*arrive);
            pen = if *r > 0.0 { pen.arc_ccw_to(*leave, *c) } else { pen.arc_cw_to(*leave, *c) };
        }
        pen.close()
    }

    /// A loop from segments. Refuses gaps between consecutive segments, zero-length pieces, zero
    /// area, and pieces that cross or touch, naming the segments.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let tri = Loop::new(vec![
    ///     Segment::Line { a: [0.0, 0.0].into(), b: [1.0, 0.0].into() },
    ///     Segment::Line { a: [1.0, 0.0].into(), b: [0.0, 1.0].into() },
    ///     Segment::Line { a: [0.0, 1.0].into(), b: [0.0, 0.0].into() },
    /// ])?;
    /// assert!(tri.is_ccw());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn new(segments: Vec<Segment>) -> Result<Loop> {
        let labels = vec![None; segments.len()];
        Loop::labelled("Loop::new", segments, labels)
    }

    /// This loop scaled by `s` about the plane's origin, labels kept.
    pub(crate) fn scaled(&self, op: &'static str, s: f64) -> Result<Loop> {
        Loop::labelled(op, self.segments.iter().map(|g| g.scaled(s)).collect(), self.labels.clone())
    }

    fn labelled(op: &'static str, segments: Vec<Segment>, labels: Vec<Option<Tag>>) -> Result<Loop> {
        let alone = segments.len() == 1;
        let pieces = segments.iter().map(|s| piece(op, s, alone)).collect::<Result<Vec<_>>>()?;
        let shape = planar::check_loop(op, &pieces)?;
        Ok(Loop { segments, labels, shape })
    }

    /// A polygon through `corners`, in the order given (closed back to the first).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let l = Loop::polygon(&[[0.0, 0.0], [4.0, 0.0], [4.0, 1.0], [1.0, 1.0], [1.0, 3.0], [0.0, 3.0]])?;
    /// assert_eq!(l.segments().len(), 6);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn polygon(corners: &[[f64; 2]]) -> Result<Loop> {
        if corners.len() < 3 {
            return Err(Error::invalid("Loop::polygon", "corners", format!("{} corners given, 3 needed", corners.len())));
        }
        let pts: Vec<Point2> = corners.iter().map(|&c| Point2::from(c)).collect();
        let segments = pts.iter().zip(pts.iter().cycle().skip(1)).map(|(&a, &b)| Segment::Line { a, b }).collect();
        let labels = vec![None; corners.len()];
        Loop::labelled("Loop::polygon", segments, labels)
    }

    /// The closed curvature-continuous cubic through `points` in the order given (chord-length
    /// spacing), as one [`Segment::PeriodicSpline`].
    ///
    /// ```
    /// use yack::prelude::*;
    /// let blob = Loop::through(&[[0.0, 0.0], [4.0, -1.0], [6.0, 2.0], [3.0, 5.0], [-1.0, 3.0]])?;
    /// assert!(blob.is_ccw());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn through(points: &[[f64; 2]]) -> Result<Loop> {
        let op = "Loop::through";
        let (knots, ctrl) =
            yack_geom::periodic_through(points).map_err(|e| crate::engine::error(op, e))?;
        let ctrl = ctrl.into_iter().map(Point2::from).collect();
        Loop::labelled(op, vec![Segment::PeriodicSpline { degree: 3, knots, ctrl }], vec![None])
    }

    /// The closed uniform B-spline of `degree` with control points `ctrl` (it passes near them,
    /// not through them; for a curve through points, [`Loop::through`]).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let cam = Loop::closed_spline(3, &[[3.5, 0.0], [4.5, -1.5], [8.0, -1.5], [12.0, -1.0], [12.0, 1.0], [8.0, 1.5], [4.5, 1.5]])?;
    /// assert_eq!(cam.segments().len(), 1);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn closed_spline(degree: u32, ctrl: &[[f64; 2]]) -> Result<Loop> {
        let op = "Loop::closed_spline";
        let pts: Vec<Point2> = ctrl.iter().map(|&c| Point2::from(c)).collect();
        let (knots, ctrl) = yack_math::planar::closed_spline(degree, &pts);
        Loop::labelled(op, vec![Segment::PeriodicSpline { degree, knots, ctrl }], vec![None])
    }

    /// Start drawing a loop at `p`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let slot = Loop::starting_at([0.0, 0.0]).line_to([10.0, 0.0]).arc_ccw_to([10.0, 4.0], [10.0, 2.0])
    ///     .line_to([0.0, 4.0]).arc_ccw_to([0.0, 0.0], [0.0, 2.0]).close()?;
    /// assert_eq!(slot.segments().len(), 4);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn starting_at(p: impl Into<Point2>) -> Pen {
        let p = p.into();
        Pen { start: p, at: p, segments: Vec::new(), labels: Vec::new(), error: None }
    }

    /// The segments.
    ///
    /// (Its example is on [`Loop`].)
    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }
    /// Each segment's label, if one was given with [`Pen::label`].
    ///
    /// (Its example is on [`Loop`].)
    pub fn labels(&self) -> &[Option<Tag>] {
        &self.labels
    }
    /// Whether the loop runs counter-clockwise.
    ///
    /// (Its example is on [`Loop`].)
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let tri = Loop::starting_at([0.0, 0.0]).line_to([1.0, 0.0]).line_to([0.0, 1.0]).close()?;
    /// assert_eq!(tri.segments().len(), 3); // close adds the last side
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn line_to(self, p: impl Into<Point2>) -> Pen {
        let b = p.into();
        let a = self.at;
        self.push(Segment::Line { a, b }, b)
    }

    fn arc(mut self, op: &'static str, end: Point2, center: Point2, ccw: bool) -> Pen {
        match planar::arc_to(op, self.at, end, center, ccw) {
            Ok((radius, start, sweep)) => self.push(Segment::Arc { center, radius, start, sweep }, end),
            Err(e) => {
                self.error.get_or_insert(e);
                self
            }
        }
    }

    /// A counter-clockwise arc from the current point to `end` about `center`. Both ends must lie
    /// at the same distance from `center` (within `tol::LINEAR`); `end` equal to the current point
    /// draws a full circle.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let half = Loop::starting_at([1.0, 0.0]).arc_ccw_to([-1.0, 0.0], [0.0, 0.0]).close()?;
    /// assert!(half.is_ccw());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn arc_ccw_to(self, end: impl Into<Point2>, center: impl Into<Point2>) -> Pen {
        self.arc("Pen::arc_ccw_to", end.into(), center.into(), true)
    }

    /// A clockwise arc from the current point to `end` about `center`, as
    /// [`arc_ccw_to`](Pen::arc_ccw_to) otherwise.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a notch curving into a square's top side
    /// let l = Loop::starting_at([0.0, 0.0]).line_to([4.0, 0.0]).line_to([4.0, 4.0]).line_to([3.0, 4.0])
    ///     .arc_cw_to([1.0, 4.0], [2.0, 4.0]).line_to([0.0, 4.0]).close()?;
    /// assert_eq!(l.segments().len(), 6);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn arc_cw_to(self, end: impl Into<Point2>, center: impl Into<Point2>) -> Pen {
        self.arc("Pen::arc_cw_to", end.into(), center.into(), false)
    }

    /// A conic from the current point to `end`, tangent to the lines through `apex`, with apex
    /// weight `weight`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a parabola (weight 1) from its vertex to (8, 8), its end tangents meeting at (4, 0)
    /// let side = Loop::starting_at([0.0, 0.0]).conic_to([8.0, 8.0], [4.0, 0.0], 1.0).line_to([0.0, 8.0]).close()?;
    /// assert_eq!(side.segments().len(), 3);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn conic_to(self, end: impl Into<Point2>, apex: impl Into<Point2>, weight: f64) -> Pen {
        let (b, apex) = (end.into(), apex.into());
        let a = self.at;
        self.push(Segment::Conic { a, apex, b, weight }, b)
    }

    /// Labels the last segment drawn; the faces it sweeps carry the label in history.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let l = Loop::starting_at([0.0, 0.0]).line_to([4.0, 0.0]).label("floor").line_to([0.0, 3.0]).close()?;
    /// assert_eq!(l.labels()[0], Some(Tag::from("floor")));
    /// # Ok::<(), yack::Error>(())
    /// ```
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// // an open pen closes with a line back to its start; a gapless one needs none
    /// let l = Loop::starting_at([0.0, 0.0]).line_to([2.0, 0.0]).line_to([2.0, 2.0]).close()?;
    /// assert_eq!(l.segments().len(), 3);
    /// # Ok::<(), yack::Error>(())
    /// ```
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
    /// The outline round a chain of circles, taken in order counter-clockwise, each joined to the
    /// next by their common tangent. A positive radius is a tip: the outline wraps round that
    /// circle's outside, bulging out (a star's point). A negative radius is a fillet: the outline
    /// runs round the inside of the circle, curving in between two tips (a star's notch). So
    /// tips at the four compass points with fillets on the diagonals between them make a
    /// four-pointed star whose sides are straight lines tangent to both.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let star = Profile::tangent_chain(&[([30.0, 0.0].into(), 4.0), ([10.0, 10.0].into(), -3.0), ([0.0, 30.0].into(), 4.0),
    ///     ([-10.0, 10.0].into(), -3.0), ([-30.0, 0.0].into(), 4.0), ([-10.0, -10.0].into(), -3.0), ([0.0, -30.0].into(), 4.0),
    ///     ([10.0, -10.0].into(), -3.0)])?;
    /// assert_eq!(star.outer().segments().len(), 16);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn tangent_chain(circles: &[(Point2, f64)]) -> Result<Profile> {
        Profile::new(Loop::tangent_chain(circles)?, Vec::new())
    }

    /// A profile from an outer loop and holes. The outer loop must run counter-clockwise and the
    /// holes clockwise, strictly inside it and apart from each other; anything else is refused,
    /// never repaired.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let outer = Loop::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])?;
    /// let hole = Loop::polygon(&[[4.0, 4.0], [4.0, 6.0], [6.0, 6.0], [6.0, 4.0]])?; // clockwise
    /// let washer = Profile::new(outer, vec![hole])?;
    /// assert!((extrude(&Frame::WORLD, &washer, 1.0)?.volume() - 96.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn new(outer: Loop, holes: Vec<Loop>) -> Result<Profile> {
        let shapes: Vec<LoopShape> = holes.iter().map(|h| h.shape.clone()).collect();
        planar::check_profile("Profile::new", &outer.shape, &shapes)?;
        Ok(Profile { outer, holes })
    }

    /// This profile scaled by `s > 0` about the plane's origin (a scaled copy of a valid profile
    /// is valid, so the profile check is not run again).
    pub(crate) fn scaled(&self, op: &'static str, s: f64) -> Result<Profile> {
        let holes = self.holes.iter().map(|h| h.scaled(op, s)).collect::<Result<Vec<_>>>()?;
        Ok(Profile { outer: self.outer.scaled(op, s)?, holes })
    }

    /// The rectangle with corners `min` and `max` (`min < max` on both axes).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let r = Profile::rectangle([0.0, 0.0], [3.0, 2.0])?;
    /// assert!((extrude(&Frame::WORLD, &r, 1.0)?.volume() - 6.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
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
    ///
    /// ```
    /// use yack::prelude::*;
    /// let c = Profile::circle([0.0, 0.0], 2.0)?;
    /// assert!((extrude(&Frame::WORLD, &c, 1.0)?.volume() - 4.0 * std::f64::consts::PI).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
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

    /// The axis-aligned ellipse about `center` with semi-axes `a` along x and `b` along y (exact:
    /// four quarter conics).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let e = Profile::ellipse([0.0, 0.0], 3.0, 2.0)?;
    /// assert_eq!(e.outer().segments().len(), 4);
    /// assert!(Profile::ellipse([0.0, 0.0], 3.0, 0.0).is_err());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn ellipse(center: impl Into<Point2>, a: f64, b: f64) -> Result<Profile> {
        let op = "Profile::ellipse";
        let center = center.into();
        for (arg, v) in [("a", a), ("b", b)] {
            if !(v.is_finite() && v > yack_tol::LINEAR) {
                return Err(Error::invalid(op, arg, format!("semi-axis {v} is not positive")));
            }
        }
        let segments = planar::ellipse_quarters(center, a, b)
            .into_iter()
            .map(|(a, apex, b)| Segment::Conic { a, apex, b, weight: core::f64::consts::FRAC_1_SQRT_2 })
            .collect();
        Profile::new(Loop::labelled(op, segments, vec![None; 4])?, Vec::new())
    }

    /// The polygon through `corners`, in either order (it is made counter-clockwise).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let tri = Profile::polygon(&[[0.0, 0.0], [4.0, 0.0], [0.0, 3.0]])?;
    /// assert!((extrude(&Frame::WORLD, &tri, 1.0)?.volume() - 6.0).abs() < 1e-9);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn polygon(corners: &[[f64; 2]]) -> Result<Profile> {
        let mut l = Loop::polygon(corners)?;
        if !l.is_ccw() {
            let reversed: Vec<[f64; 2]> = corners.iter().rev().copied().collect();
            l = Loop::polygon(&reversed)?;
        }
        Profile::new(l, Vec::new())
    }

    /// The outer loop.
    ///
    /// (Its example is on [`Profile`].)
    pub fn outer(&self) -> &Loop {
        &self.outer
    }
    /// The holes.
    ///
    /// (Its example is on [`Profile`].)
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
///
/// ```
/// use yack::prelude::*;
/// let s = Spline3::bezier(&[[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])?;
/// assert_eq!(s.degree(), 2);
/// assert_eq!(s.knots(), &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
/// assert_eq!(s.ctrl()[1], Point3::new(1.0, 1.0, 0.0));
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Spline3 {
    degree: u32,
    knots: Vec<f64>,
    ctrl: Vec<Point3>,
}

impl Spline3 {
    /// A clamped B-spline of `degree` with `ctrl.len() + degree + 1` non-decreasing knots.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a clamped cubic through four control points: knots 0 0 0 0 1 1 1 1
    /// let pts = [[0.0, 0.0, 0.0], [1.0, 2.0, 0.0], [3.0, 2.0, 0.0], [4.0, 0.0, 0.0]].map(Point3::from).to_vec();
    /// let s = Spline3::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], pts)?;
    /// assert_eq!(s.degree(), 3);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn new(degree: u32, knots: Vec<f64>, ctrl: Vec<Point3>) -> Result<Spline3> {
        knots_ok("Spline3::new", degree, &knots, ctrl.len(), true)?;
        if !ctrl.iter().all(|p| p.is_finite()) {
            return Err(Error::invalid("Spline3::new", "ctrl", "a control point is not finite"));
        }
        Ok(Spline3 { degree, knots, ctrl })
    }

    /// The Bézier curve with these control points (degree = points − 1).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let line = Spline3::bezier(&[[0.0, 0.0, 0.0], [5.0, 0.0, 0.0]])?;
    /// assert_eq!(line.degree(), 1);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn bezier(ctrl: &[[f64; 3]]) -> Result<Spline3> {
        let n = ctrl.len();
        let degree = u32::try_from(n.saturating_sub(1)).unwrap_or(0);
        let mut knots = vec![0.0; n];
        knots.extend(core::iter::repeat_n(1.0, n));
        let pts = ctrl.iter().map(|&p| Point3::from(p)).collect();
        Spline3::new(degree, knots, pts).map_err(|e| e.in_op("Spline3::bezier"))
    }

    /// The degree.
    ///
    /// (Its example is on [`Spline3`].)
    pub fn degree(&self) -> u32 {
        self.degree
    }
    /// The knots.
    ///
    /// (Its example is on [`Spline3`].)
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }
    /// The control points.
    ///
    /// (Its example is on [`Spline3`].)
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

/// How a screw rises along its axis as it turns. Built only by its constructors, which name their
/// unit; one screw concept for [`Motion::screw`], [`make::screw`](crate::make::screw) and
/// [`make::screw_tapered`](crate::make::screw_tapered).
///
/// ```
/// use yack::prelude::*;
/// let thread = Lead::per_turn(1.5);          // 1.5 mm a turn
/// let same = Lead::per_radian(1.5 / (2.0 * std::f64::consts::PI));
/// let growing = Lead::polynomial(vec![0.0, 0.2, 0.01]); // rise = 0.2 θ + 0.01 θ², θ in radians
/// # let _ = (thread, same, growing);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Lead(pub(crate) LeadKind);

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum LeadKind {
    PerTurn(f64),
    PerRadian(f64),
    Polynomial(Vec<f64>),
}

impl Lead {
    /// A constant rise of `mm` along the axis each whole turn.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a thread rising 3 mm each turn
    /// let thread = screw(&Frame::WORLD, &Profile::rectangle([4.0, 0.0], [5.0, 1.0])?, Lead::per_turn(3.0), deg(360.0))?;
    /// assert!(thread.volume() > 0.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn per_turn(mm: f64) -> Lead {
        Lead(LeadKind::PerTurn(mm))
    }
    /// A constant rise of `mm` along the axis each radian turned.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // 0.5 mm of rise for each radian turned (π mm a whole turn)
    /// let ball = sphere(&Frame::at([5.0, 0.0, 0.0]), 1.0)?;
    /// let coil = ball.swept(&Motion::screw([0.0; 3], Dir3::Z, deg(0.0), deg(360.0), Lead::per_radian(0.5)))?;
    /// assert!(coil.volume() > ball.volume());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn per_radian(mm: f64) -> Lead {
        Lead(LeadKind::PerRadian(mm))
    }
    /// A rise that is a polynomial in the angle turned: `rise = Σ cₖ θᵏ`, θ in radians measured from
    /// the motion's start (`from`), `coefficients[k] = cₖ`. A lead that grows as a thread climbs.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a lead growing as it turns: rise = 0.5θ + 0.1θ² from the start angle
    /// let growing = Lead::polynomial(vec![0.0, 0.5, 0.1]);
    /// let _ = Motion::screw([0.0; 3], Dir3::Z, deg(0.0), deg(180.0), growing);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn polynomial(coefficients: Vec<f64>) -> Lead {
        Lead(LeadKind::Polynomial(coefficients))
    }
}

/// A motion of a solid through space, for [`Solid::swept`](crate::Solid::swept) and
/// [`Solid::cut_swept`](crate::Solid::cut_swept). Built only by its constructors.
///
/// Every axis and direction takes a [`Dir3`](crate::Dir3), a [`Vec3`](crate::Vec3) or an array
/// as it is (`Dir3::Z`, never `Dir3::Z.into()`); a zero axis is refused when the motion is used.
///
/// ```
/// use yack::prelude::*;
/// let spin = Motion::turn([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(0.0), deg(90.0));
/// let tilt = Motion::turn(Point3::ORIGIN, Dir3::X, deg(0.0), deg(30.0));
/// let rack = Motion::together(vec![Motion::slide([10.0, 0.0, 0.0]), Motion::turn([0.0; 3], [0.0, 0.0, 1.0], deg(0.0), deg(45.0))]);
/// let screw = Motion::screw([0.0; 3], [0.0, 0.0, 1.0], deg(0.0), deg(720.0), Lead::per_turn(2.0));
/// # let _ = (spin, tilt, rack, screw);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Motion(pub(crate) MotionKind);

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum MotionKind {
    Slide { by: Vec3 },
    Turn { axis_point: Point3, axis: Vec3, from: Angle, to: Angle },
    Screw { axis_point: Point3, axis: Vec3, from: Angle, to: Angle, lead: Lead },
    Along { path: Spline3, law: FrameLaw },
    Then(Vec<Motion>),
    Together(Vec<Motion>),
}

impl Motion {
    /// A straight slide by `by`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let capsule = sphere(&Frame::WORLD, 1.0)?.swept(&Motion::slide([4.0, 0.0, 0.0]))?;
    /// assert!(capsule.volume() > sphere(&Frame::WORLD, 1.0)?.volume());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn slide(by: impl Into<Vec3>) -> Motion {
        Motion(MotionKind::Slide { by: by.into() })
    }
    /// A turn about the line through `axis_point` along `axis`, from the angle `from` to `to`
    /// (counter-clockwise looking down `axis`). The solid is as given at angle 0.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a quarter turn about z of a bar off the axis sweeps a quarter ring
    /// let bar = block(&Frame::at([2.0, -0.5, 0.0]), [1.0, 1.0, 1.0])?;
    /// let arc = bar.swept(&Motion::turn([0.0; 3], Dir3::Z, deg(0.0), deg(90.0)))?;
    /// assert!(arc.volume() > bar.volume());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn turn(axis_point: impl Into<Point3>, axis: impl Into<Vec3>, from: Angle, to: Angle) -> Motion {
        Motion(MotionKind::Turn { axis_point: axis_point.into(), axis: axis.into(), from, to })
    }
    /// A turn as [`turn`](Motion::turn) while rising along the axis by `lead`. The solid is as
    /// given at angle 0 (a constant lead rises from there; a polynomial one from `from`).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let ball = sphere(&Frame::at([5.0, 0.0, 0.0]), 1.0)?;
    /// let coil = ball.swept(&Motion::screw([0.0; 3], Dir3::Z, deg(0.0), deg(360.0), Lead::per_turn(4.0)))?;
    /// assert!(coil.volume() > ball.volume());
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn screw(axis_point: impl Into<Point3>, axis: impl Into<Vec3>, from: Angle, to: Angle, lead: Lead) -> Motion {
        Motion(MotionKind::Screw { axis_point: axis_point.into(), axis: axis.into(), from, to, lead })
    }
    /// Along a spline path that starts at the solid's position, its frame following `law`.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 10.0, 0.0]])?;
    /// let groove = sphere(&Frame::WORLD, 1.0)?.swept(&Motion::along(path, FrameLaw::Fixed))?;
    /// assert!(groove.volume() > 0.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn along(path: Spline3, law: FrameLaw) -> Motion {
        Motion(MotionKind::Along { path, law })
    }
    /// These motions one after another, each starting where the last left the solid.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // one leg after another; a then of thens is one flat sequence
    /// let l = Motion::then(vec![Motion::slide([5.0, 0.0, 0.0]), Motion::slide([0.0, 5.0, 0.0])]);
    /// let nested = Motion::then(vec![Motion::then(vec![Motion::slide([5.0, 0.0, 0.0])]), Motion::slide([0.0, 5.0, 0.0])]);
    /// assert_eq!(l, nested);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn then(legs: Vec<Motion>) -> Motion {
        // a sequence of sequences is one sequence: an along_loop or through leg joins in place
        let flat = legs
            .into_iter()
            .flat_map(|m| match m.0 {
                MotionKind::Then(inner) => inner,
                kind => vec![Motion(kind)],
            })
            .collect();
        Motion(MotionKind::Then(flat))
    }
    /// A tool carried once round `path`, drawn in `frame`'s xy plane, from the loop's start:
    /// a slide along each line and a turn about each arc's centre (CAM's contour pass).
    /// `Unsupported` where the loop has a conic or spline segment.
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a ball carried once round a 10 × 4 stadium in the xy plane
    /// let track = Loop::starting_at([0.0, 0.0]).line_to([10.0, 0.0]).arc_ccw_to([10.0, 4.0], [10.0, 2.0])
    ///     .line_to([0.0, 4.0]).arc_ccw_to([0.0, 0.0], [0.0, 2.0]).close()?;
    /// let ring = sphere(&Frame::WORLD, 0.5)?.swept(&Motion::along_loop(&Frame::WORLD, &track)?)?;
    /// assert!(ring.volume() > 0.0);
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn along_loop(frame: &crate::Frame, path: &Loop) -> Result<Motion> {
        let op = "Motion::along_loop";
        let pieces = path.segments().iter().map(|s| piece(op, s, path.segments().len() == 1)).collect::<Result<Vec<_>>>()?;
        let legs = frame.loop_legs(&pieces)?;
        Ok(Motion::then(
            legs.into_iter()
                .map(|l| match l {
                    yack_math::LoopLeg::Slide(by) => Motion::slide(by),
                    yack_math::LoopLeg::Turn { at, axis, by } => Motion::turn(at, axis, crate::rad(0.0), by),
                })
                .collect(),
        ))
    }
    /// A toolpath through the absolute points `points`, in order: the solid starts where it is
    /// drawn, at `points[0]`, and slides straight to each next point in turn (a [`then`] of
    /// slides). `Invalid` for fewer than two points.
    ///
    /// [`then`]: Motion::then
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a toolpath through absolute points: the slides between them
    /// let pts = [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [5.0, 3.0, 0.0]].map(Point3::from);
    /// assert_eq!(Motion::through(&pts)?, Motion::then(vec![Motion::slide([5.0, 0.0, 0.0]), Motion::slide([0.0, 3.0, 0.0])]));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn through(points: &[Point3]) -> Result<Motion> {
        if points.len() < 2 {
            return Err(Error::invalid("Motion::through", "points", "a toolpath needs two points or more"));
        }
        Ok(Motion::then(yack_math::steps(points).into_iter().map(Motion::slide).collect()))
    }
    /// These motions all at once: one parameter `t` runs 0 → 1, each leg is at fraction `t` of
    /// itself, applied in order at each instant (a rack's slide and its pinion's turn). Such a
    /// sweep comes out as a valid closed solid that may be faceted: [`Solid::is_exact`] is false
    /// then (never claimed exact).
    ///
    /// ```
    /// use yack::prelude::*;
    /// // a rack's slide and its pinion's turn, run at once
    /// let rolling = Motion::together(vec![Motion::slide([10.0, 0.0, 0.0]), Motion::turn([0.0; 3], Dir3::Z, deg(0.0), deg(30.0))]);
    /// let _ = rolling;
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn together(legs: Vec<Motion>) -> Motion {
        Motion(MotionKind::Together(legs))
    }
}

/// A surface patch for [`make::sewn`](crate::make::sewn).
#[derive(Clone, Debug, PartialEq)]
pub enum Patch {
    /// A Bézier patch on the control net `ctrl` (rows along u; degree rows − 1 by columns − 1).
    Bezier {
        /// The control net.
        ctrl: Vec<Vec<Point3>>,
    },
    /// The flat face filling a closed chain of curves in one plane, laid end to end (usually other
    /// patches' rims).
    PlanarFill {
        /// The boundary curves.
        boundary: Vec<Spline3>,
    },
    /// The face cadrs's fill makes on a closed chain of curves laid end to end, each in a plane of
    /// its own: flat when they share one, a Coons-like blend of them otherwise.
    Fill {
        /// The boundary curves.
        boundary: Vec<Spline3>,
    },
    /// A clamped B-spline (NURBS with `weights`) patch: `ctrl` rows along u, `degree` and `knots`
    /// for u then v; each knot vector holds its row or column count + degree + 1 knots.
    Bspline {
        /// Degree along u, then v.
        degree: [u32; 2],
        /// Knots along u, then v.
        knots: [Vec<f64>; 2],
        /// The control net.
        ctrl: Vec<Vec<Point3>>,
        /// One weight per control point (rational), or none.
        weights: Option<Vec<Vec<f64>>>,
    },
}

impl Patch {
    /// This patch moved by `m` (a [`Rigid`](crate::Rigid) motion or a [`Similarity`](crate::Similarity)).
    /// A mirror also turns the patch over (its net's rows, or its boundary chain, run the other
    /// way), so a mirrored half of a part still faces out.
    ///
    /// ```
    /// use yack::prelude::*;
    /// let net = vec![vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)], vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)]];
    /// let half = Patch::Bezier { ctrl: net };
    /// let other = half.transformed(Similarity::mirror([0.0; 3], [0.0, 1.0, 0.0])?)?;
    /// assert!(matches!(other, Patch::Bezier { ref ctrl } if ctrl[0][0] == Point3::new(0.0, -1.0, 0.0)));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn transformed(&self, m: impl Into<crate::Similarity>) -> Result<Patch> {
        let m: crate::Similarity = m.into();
        let flip = m.is_mirror();
        let net = |ctrl: &[Vec<Point3>]| -> Vec<Vec<Point3>> {
            ctrl.iter()
                .map(|row| {
                    let mut r: Vec<Point3> = row.iter().map(|&p| m.point(p)).collect();
                    if flip {
                        r.reverse();
                    }
                    r
                })
                .collect()
        };
        let chain = |boundary: &[Spline3]| -> Result<Vec<Spline3>> {
            let mut out = boundary
                .iter()
                .map(|c| {
                    let mut ctrl: Vec<Point3> = c.ctrl().iter().map(|&p| m.point(p)).collect();
                    let knots = if flip {
                        ctrl.reverse();
                        yack_math::planar::knots_reversed(c.knots())
                    } else {
                        c.knots().to_vec()
                    };
                    Spline3::new(c.degree(), knots, ctrl)
                })
                .collect::<Result<Vec<_>>>()?;
            if flip {
                out.reverse();
            }
            Ok(out)
        };
        Ok(match self {
            Patch::Bezier { ctrl } => Patch::Bezier { ctrl: net(ctrl) },
            Patch::PlanarFill { boundary } => Patch::PlanarFill { boundary: chain(boundary)? },
            Patch::Fill { boundary } => Patch::Fill { boundary: chain(boundary)? },
            Patch::Bspline { degree, knots, ctrl, weights } => {
                let [ku, kv] = knots;
                let kv = if flip { yack_math::planar::knots_reversed(kv) } else { kv.clone() };
                let weights = weights.as_ref().map(|w| {
                    w.iter()
                        .map(|row| {
                            let mut r = row.clone();
                            if flip {
                                r.reverse();
                            }
                            r
                        })
                        .collect()
                });
                Patch::Bspline { degree: *degree, knots: [ku.clone(), kv], ctrl: net(ctrl), weights }
            }
        })
    }
}

/// A tool's path drawn move by move from where the tool is, as a CAM program reads: absolute
/// moves (`line_to`), relative ones (`slide`), turns, and contour passes round a [`Loop`]. It
/// keeps the tool's position, so each move needs only where it goes; [`Toolpath::motion`] gives
/// the [`Motion`] for [`Solid::cut_swept`](crate::Solid::cut_swept) or [`Solid::swept`](crate::Solid::swept).
///
/// ```
/// use yack::prelude::*;
/// // down 2 into a 10 mm plate, across 20, round a half turn, back up
/// let path = Toolpath::starting_at([0.0, 0.0, 5.0])
///     .line_to([0.0, 0.0, -2.0])
///     .line_to([20.0, 0.0, -2.0])
///     .turn([20.0, 5.0, -2.0], [0.0, 0.0, 1.0], deg(180.0))?
///     .slide([0.0, 0.0, 7.0]);
/// let end = path.at(); // round the half turn to (20, 10), then up 7
/// assert!((end.x - 20.0).abs() < 1e-9 && (end.y - 10.0).abs() < 1e-9 && (end.z - 5.0).abs() < 1e-9);
/// let mill = cylinder(&Frame::at([0.0, 0.0, 5.0]), 1.0, 10.0)?;
/// let cut = block(&Frame::at([-5.0, -5.0, -10.0]), [40.0, 25.0, 10.0])?.cut_swept(&mill, &path.motion())?;
/// assert!(cut.volume() < 40.0 * 25.0 * 10.0);
/// # Ok::<(), yack::Error>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct Toolpath {
    at: Point3,
    legs: Vec<Motion>,
}

impl Toolpath {
    /// A toolpath with the tool at `p` (where the tool solid is drawn).
    ///
    /// (Its example is on [`Toolpath`].)
    pub fn starting_at(p: impl Into<Point3>) -> Toolpath {
        Toolpath { at: p.into(), legs: Vec::new() }
    }

    /// Where the tool is after the moves so far.
    ///
    /// (Its example is on [`Toolpath`].)
    pub fn at(&self) -> Point3 {
        self.at
    }

    /// A straight move to `p` (nothing when the tool is already there).
    ///
    /// (Its example is on [`Toolpath`].)
    pub fn line_to(mut self, p: impl Into<Point3>) -> Toolpath {
        let p = p.into();
        if let Some(by) = yack_math::slide_to(self.at, p) {
            self.legs.push(Motion::slide(by));
            self.at = p;
        }
        self
    }

    /// A straight move by `by`.
    ///
    /// (Its example is on [`Toolpath`].)
    pub fn slide(mut self, by: impl Into<Vec3>) -> Toolpath {
        let by = by.into();
        self.at = crate::Rigid::translation(by).point(self.at);
        self.legs.push(Motion::slide(by));
        self
    }

    /// A turn of the tool by `by` about the line through `axis_point` along `axis` (an arc of the
    /// tool's path round that line; the tool turns with it, as [`Motion::turn`]).
    ///
    /// (Its example is on [`Toolpath`].)
    pub fn turn(mut self, axis_point: impl Into<Point3>, axis: impl Into<Vec3>, by: Angle) -> Result<Toolpath> {
        let (q, a) = (axis_point.into(), axis.into());
        self.at = yack_math::turned_about(self.at, q, a, by).map_err(|e| e.in_op("Toolpath::turn"))?;
        self.legs.push(Motion::turn(q, a, crate::rad(0.0), by));
        Ok(self)
    }

    /// A contour pass once round `path` drawn in `frame`'s xy plane: a move to the loop's start,
    /// then round it as [`Motion::along_loop`] (ending back at the start).
    ///
    /// ```
    /// use yack::prelude::*;
    /// let square = Loop::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])?.rounded(2.0)?;
    /// let path = Toolpath::starting_at([5.0, 0.0, 5.0]).line_to([5.0, 0.0, -1.0]).along_loop(&Frame::at([0.0, 0.0, -1.0]), &square)?;
    /// assert!(path.motion() != Motion::then(vec![]));
    /// # Ok::<(), yack::Error>(())
    /// ```
    pub fn along_loop(self, frame: &crate::Frame, path: &Loop) -> Result<Toolpath> {
        let start = path.shape.polyline.first().copied().unwrap_or_else(|| Point2::new(0.0, 0.0));
        let mut s = self.line_to(frame.point([start.x, start.y, 0.0]));
        s.legs.push(Motion::along_loop(frame, path)?);
        Ok(s)
    }

    /// The toolpath as one motion (the moves in order).
    ///
    /// (Its example is on [`Toolpath`].)
    pub fn motion(&self) -> Motion {
        Motion::then(self.legs.clone())
    }
}

/// A patch's net as cadrs takes it: degree (u, v), knots (u, v), poles, weights; `Invalid` for a
/// malformed net, and for a planar fill (not a net).
#[allow(clippy::type_complexity)]
pub(crate) fn patch_net(op: &'static str, p: &Patch) -> Result<([u32; 2], [Vec<f64>; 2], Vec<Vec<[f64; 3]>>, Option<Vec<Vec<f64>>>)> {
    let rows = |ctrl: &[Vec<Point3>]| -> Result<(usize, usize)> {
        let n = ctrl.first().map_or(0, Vec::len);
        if ctrl.len() < 2 || n < 2 || ctrl.iter().any(|r| r.len() != n) {
            return Err(Error::invalid(op, "patch", "a net needs two rows or more, each of one length, two points or more"));
        }
        Ok((ctrl.len(), n))
    };
    let poles = |ctrl: &[Vec<Point3>]| ctrl.iter().map(|r| r.iter().map(|q| q.to_array()).collect()).collect();
    match p {
        Patch::Bezier { ctrl } => {
            let (m, n) = rows(ctrl)?;
            let du = u32::try_from(m - 1).unwrap_or(u32::MAX);
            let dv = u32::try_from(n - 1).unwrap_or(u32::MAX);
            let clamped = |k: usize| core::iter::repeat_n(0.0, k).chain(core::iter::repeat_n(1.0, k)).collect::<Vec<f64>>();
            Ok(([du, dv], [clamped(m), clamped(n)], poles(ctrl), None))
        }
        Patch::Bspline { degree, knots, ctrl, weights } => {
            let (m, n) = rows(ctrl)?;
            knots_ok(op, degree[0], &knots[0], m, true)?;
            knots_ok(op, degree[1], &knots[1], n, true)?;
            if let Some(w) = weights
                && (w.len() != m || w.iter().any(|r| r.len() != n || r.iter().any(|&x| x.is_nan() || x <= 0.0)))
            {
                return Err(Error::invalid(op, "weights", "one positive weight per control point"));
            }
            Ok((*degree, knots.clone(), poles(ctrl), weights.clone()))
        }
        Patch::PlanarFill { .. } | Patch::Fill { .. } => Err(Error::invalid(op, "patch", "a fill is not a surface net")),
    }
}
