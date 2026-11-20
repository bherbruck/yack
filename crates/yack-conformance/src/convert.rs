// yack: the protocol's forms as yack's values, and yack's errors as refusals (contract rules 1, 2).
// Adapted from brep-conformance-springrep@92ee43f src/main.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep's types replaced by the yack public API; one error map `r`.

use brep_protocol::{Refusal, model, path::PathCurve, profile};
use yack::{
    EdgeId, Error, ErrorKind, FaceId, Frame, Loop, PathPiece, Point3, Profile, Segment, Solid,
    Spline3, rad,
};

/// yack's error as the protocol's refusal (PLAN §5.1). A checker rejection or a caught panic is
/// `Internal`, and the suite must see it as the crash it is.
pub fn r(e: Error) -> Refusal {
    let reason = e.to_string();
    match e.kind {
        ErrorKind::Invalid | ErrorKind::NotFound | ErrorKind::ForeignId => Refusal::invalid(reason),
        ErrorKind::Ambiguous | ErrorKind::Unsupported => Refusal::unsupported(reason),
        ErrorKind::Failed | ErrorKind::Budget => Refusal::failed(reason),
        ErrorKind::Internal => std::panic::resume_unwind(Box::new(reason)),
        _ => Refusal::failed(reason),
    }
}

/// The suite's frame as yack's; yack refuses a frame that is not orthonormal and right-handed.
pub fn frame(f: &model::Frame) -> Result<Frame, Refusal> {
    Frame::new(f.origin, f.x, f.y, f.z).map_err(r)
}

/// Points as yack's.
fn points(ps: &[model::Vec3]) -> Vec<Point3> {
    ps.iter().map(|&p| Point3::from(p)).collect()
}

/// A count of the protocol's as yack's `u32`.
fn count(n: usize, what: &str) -> Result<u32, Refusal> {
    u32::try_from(n).map_err(|_| Refusal::invalid(format!("{what} {n} is out of range")))
}

fn segment(s: &profile::Seg) -> Result<Segment, Refusal> {
    Ok(match s {
        profile::Seg::Line { a, b } => Segment::Line { a: (*a).into(), b: (*b).into() },
        profile::Seg::Arc { center, radius, start, sweep } => Segment::Arc {
            center: (*center).into(),
            radius: *radius,
            start: rad(*start),
            sweep: rad(*sweep),
        },
        profile::Seg::Conic { a, apex, b, w } => {
            Segment::Conic { a: (*a).into(), apex: (*apex).into(), b: (*b).into(), weight: *w }
        }
        profile::Seg::Gn { a, apex, b, order, w } => Segment::Blend {
            a: (*a).into(),
            apex: (*apex).into(),
            b: (*b).into(),
            order: *order,
            weight: *w,
        },
        profile::Seg::Spline(sp) => Segment::PeriodicSpline {
            degree: count(sp.degree, "spline degree")?,
            knots: sp.knots.clone(),
            ctrl: sp.ctrl.iter().map(|&c| c.into()).collect(),
        },
        _ => return Err(Refusal::unsupported("a profile segment this protocol version adds")),
    })
}

fn sketch_loop(l: &[profile::Seg]) -> Result<Loop, Refusal> {
    Loop::new(l.iter().map(segment).collect::<Result<_, _>>()?).map_err(r)
}

/// The suite's profile as yack's: its first loop the outline, the rest holes.
pub fn sketch(p: &profile::Profile) -> Result<Profile, Refusal> {
    let Some((outer, holes)) = p.loops.split_first() else {
        return Err(Refusal::invalid("a profile with no loops"));
    };
    let holes = holes.iter().map(|h| sketch_loop(h)).collect::<Result<_, _>>()?;
    Profile::new(sketch_loop(outer)?, holes).map_err(r)
}

/// A sweep's path of lines and bends as yack's.
pub fn path(pieces: &[model::PathSeg]) -> Vec<PathPiece> {
    let piece = |p: &model::PathSeg| match *p {
        model::PathSeg::Line { length } => PathPiece::Line { length },
        model::PathSeg::Arc { radius, angle: a } => PathPiece::Arc { radius, angle: rad(a) },
    };
    pieces.iter().map(piece).collect()
}

/// A path curve, in the frame's coordinates, as yack's spline.
pub fn spline(c: &PathCurve) -> Result<Spline3, Refusal> {
    Spline3::new(count(c.degree, "path degree")?, c.knots.clone(), points(&c.ctrl)).map_err(r)
}

/// The edge each point lies on, by yack's own pick at `tol::PICK`.
pub fn edges(s: &Solid, at: &[model::Vec3]) -> Result<Vec<EdgeId>, Refusal> {
    at.iter().map(|&p| s.edge_at(p).map_err(r)).collect()
}

/// The face each point lies on, by yack's own pick at `tol::PICK`.
pub fn faces(s: &Solid, at: &[model::Vec3]) -> Result<Vec<FaceId>, Refusal> {
    at.iter().map(|&p| s.face_at(p).map_err(r)).collect()
}

/// Two lists the protocol sends side by side, as pairs; lists of unequal length name no request.
pub fn pairs<A>(a: Vec<A>, b: &[f64], what: &str) -> Result<Vec<(A, f64)>, Refusal> {
    if a.len() != b.len() {
        return Err(Refusal::invalid(format!("{} edges but {} {what}", a.len(), b.len())));
    }
    Ok(a.into_iter().zip(b.iter().copied()).collect())
}
