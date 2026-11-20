// yack: routing to the temporary springrep engine (PLAN §3.2, M1.2): type conversions and error
// mapping only. No arithmetic here (gate 13); anything computed goes through yack-math.
use yack_springrep as sr;

use sr::math::{Vec2 as SrVec2, Vec3 as SrVec3};

use crate::{Error, ErrorKind, Frame, Point2, Point3, Profile, Result, Segment, Solid, Spline3, Stage, Vec3};

pub(crate) use sr::topo::Brep;

/// The feature name an operation builds its new faces under; [`made`] records it on the result
/// (made deterministic there) so `Solid::named` can find exactly those faces.
pub(crate) fn feature() -> String {
    PROVISIONAL.to_string()
}

/// The provisional feature name an op builds under; `Solid::with_feature` replaces it with one
/// derived from the result's content.
pub(crate) const PROVISIONAL: &str = "yack#new";

pub(crate) fn v3(v: Vec3) -> SrVec3 {
    SrVec3::new(v.x, v.y, v.z)
}

pub(crate) fn p3(p: Point3) -> SrVec3 {
    SrVec3::new(p.x, p.y, p.z)
}

fn p2(p: Point2) -> SrVec2 {
    SrVec2::new(p.x, p.y)
}

pub(crate) fn point(v: SrVec3) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}

pub(crate) fn vec(v: SrVec3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

/// The same frame in springrep's terms.
/// Built as springrep's own adapter builds it (`Frame::new(origin, z, x)`: z made unit, x squared
/// against it, y = z × x), so a near-degenerate case reads the frame to the last bit alike.
pub(crate) fn frame(f: &Frame) -> sr::math::Frame {
    let copied = sr::math::Frame { origin: p3(f.origin()), x: v3(f.x().vec()), y: v3(f.y().vec()), z: v3(f.z().vec()) };
    sr::math::Frame::new(copied.origin, copied.z, copied.x).unwrap_or(copied)
}

/// The frame's xz plane as a sketch frame (x along the frame's x, sketch y along the frame's z),
/// so a profile drawn in it revolves about the frame's z.
pub(crate) fn meridian(f: &Frame) -> sr::math::Frame {
    let copied = sr::math::Frame { origin: p3(f.origin()), x: v3(f.x().vec()), y: v3(f.z().vec()), z: v3(-f.y().vec()) };
    sr::math::Frame::new(copied.origin, copied.z, copied.x).unwrap_or(copied)
}

/// springrep's refusal as yack's error, from `op`.
pub(crate) fn error(op: &'static str, e: sr::Error) -> Error {
    let detail = e.to_string();
    match e {
        sr::Error::InvalidInput(_) => Error { kind: ErrorKind::Invalid, op, arg: None, stage: Some(Stage::Validate), detail },
        sr::Error::Unsupported(_) => Error::new(ErrorKind::Unsupported, op, detail),
        sr::Error::Numerical(_) => Error { kind: ErrorKind::Failed, op, arg: None, stage: None, detail },
        sr::Error::InvalidResult(_) => Error::failed(op, Stage::Check, detail),
    }
}

/// The prefix of a face name that says the face was swept from a profile segment of that name.
const SEGMENT: &str = "seg:";

/// springrep's block faces renamed to yack's frame-axis side names.
pub(crate) const BLOCK_SIDES: &[(&str, &str)] =
    &[("u_min", "seg:x_min"), ("u_max", "seg:x_max"), ("v_min", "seg:y_min"), ("v_max", "seg:y_max")];

/// A springrep result whose new faces carry `feature`, with that feature's face names renamed by
/// `names` (engine names to yack's role vocabulary, see [`role`]).
pub(crate) fn made_as(feature: &str, op: &'static str, names: &[(&str, &str)], r: sr::Result<Brep>) -> Result<Solid> {
    made(
        feature,
        op,
        r.map(|mut b| {
            b.rename_faces(|n| {
                let mut n = n.clone();
                if n.feature == feature
                    && let Some((_, to)) = names.iter().find(|(from, _)| *from == n.face)
                {
                    n.face = (*to).to_string();
                }
                Some(n)
            });
            b
        }),
    )
}

/// The role a face name says its face plays, in yack's vocabulary; `None` for a name with no role.
pub(crate) fn role(face: &str) -> Option<crate::Role> {
    use crate::Role;
    if let Some(label) = face.strip_prefix(SEGMENT) {
        return Some(Role::Side(label.into()));
    }
    let index = |rest: &str| rest.parse::<u32>().ok();
    match face {
        "start" => Some(Role::Start),
        "end" => Some(Role::End),
        "skin" => Some(Role::Skin),
        "inner" => Some(Role::Inner),
        _ if face.starts_with("corner") => Some(Role::Corner),
        _ => {
            if let Some(n) = face.strip_prefix("round_").and_then(index) {
                Some(Role::Round(n))
            } else {
                face.strip_prefix("bevel_").and_then(index).map(Role::Bevel)
            }
        }
    }
}

/// Whether a face feature name is one yack made up for an unnamed operation.
pub(crate) fn is_internal_feature(name: &str) -> bool {
    name.starts_with("yack#")
}

/// A springrep result whose new faces carry `feature`, as a solid that remembers it.
pub(crate) fn made(feature: &str, op: &'static str, r: sr::Result<Brep>) -> Result<Solid> {
    r.map(|b| Solid::from_springrep(b).with_feature(feature)).map_err(|e| error(op, e))
}

/// A springrep result as a solid.
pub(crate) fn solid(op: &'static str, r: sr::Result<Brep>) -> Result<Solid> {
    r.map(Solid::from_springrep).map_err(|e| error(op, e))
}

fn unsupported(op: &'static str, what: &str) -> Error {
    Error::new(ErrorKind::Unsupported, op, format!("{what} is not supported by the springrep engine yet"))
}

/// A planar point carried into springrep, scaled by `scale` if given.
fn at(p: Point2, scale: Option<f64>) -> SrVec2 {
    p2(scale.map_or(p, |s| yack_math::scale2(p, s)))
}

fn segment(op: &'static str, s: &Segment, scale: Option<f64>) -> Result<sr::profile::Segment> {
    use sr::profile::Segment as S;
    let len = |l: f64| scale.map_or(l, |s| yack_math::scale_length(l, s));
    let ctrl3 = |ctrl: &[Point2]| -> Vec<SrVec3> {
        ctrl.iter().map(|&c| at(c, scale)).map(|c| SrVec3::new(c.x, c.y, 0.0)).collect()
    };
    Ok(match s {
        Segment::Line { a, b } => S::Line { a: at(*a, scale), b: at(*b, scale) },
        Segment::Arc { center, radius, sweep, .. } if sweep.is_full_turn() => {
            S::Circle { center: at(*center, scale), radius: len(*radius), ccw: sweep.radians() > 0.0 }
        }
        Segment::Arc { center, radius, start, sweep } => {
            S::Arc { center: at(*center, scale), radius: len(*radius), start: start.radians(), sweep: sweep.radians() }
        }
        // An ellipse's arc below weight 1; a parabola (1) or a hyperbola (above) as springrep's
        // rational Bézier, exact for any weight.
        Segment::Conic { a, apex, b, weight } if *weight >= 1.0 => {
            S::Bezier { ctrl: vec![at(*a, scale), at(*apex, scale), at(*b, scale)], weights: vec![1.0, *weight, 1.0] }
        }
        Segment::Conic { a, apex, b, weight } => {
            S::conic(at(*a, scale), at(*apex, scale), at(*b, scale), *weight).map_err(|e| error(op, e))?
        }
        Segment::Blend { .. } => return Err(unsupported(op, "a blend segment")),
        Segment::Nurbs { weights: Some(_), .. } => return Err(unsupported(op, "a rational NURBS segment")),
        Segment::Nurbs { degree, knots, ctrl, weights: None } => {
            let degree = usize::try_from(*degree).unwrap_or(usize::MAX);
            let curve = sr::geom::BSpline::new(degree, knots.clone(), ctrl3(ctrl)).map_err(|e| error(op, e))?;
            S::Spline { curve, reversed: false }
        }
        Segment::PeriodicSpline { degree: 3, knots, ctrl } => {
            let curve = sr::geom::BSpline::periodic_cubic(knots.clone(), ctrl3(ctrl)).map_err(|e| error(op, e))?;
            S::Spline { curve, reversed: false }
        }
        Segment::PeriodicSpline { .. } => return Err(unsupported(op, "a periodic spline of degree other than 3")),
    })
}

/// A yack profile as springrep's, each segment named by its label or by its place in the profile
/// (numbered through every loop, so a hole's faces never share an outline's names).
pub(crate) fn profile(op: &'static str, p: &Profile, scale: Option<f64>) -> Result<sr::profile::Profile> {
    let mut n = 0usize;
    let mut loops = Vec::new();
    for l in core::iter::once(p.outer()).chain(p.holes()) {
        let mut segments = Vec::new();
        for (s, label) in l.segments().iter().zip(l.labels()) {
            let name = match label {
                Some(t) => format!("{SEGMENT}{}", t.as_str()),
                None => format!("{SEGMENT}s{n}"),
            };
            n = n.saturating_add(1);
            segments.push(sr::profile::NamedSegment::new(segment(op, s, scale)?, name));
        }
        loops.push(sr::profile::ProfileLoop { segments });
    }
    Ok(sr::profile::Profile { loops })
}

/// A yack spline as a springrep curve, its control points carried through `place`, and the
/// curve's parameter domain.
pub(crate) fn spline(op: &'static str, s: &Spline3, place: impl Fn(Point3) -> Point3) -> Result<(sr::geom::Curve, (f64, f64))> {
    let degree = usize::try_from(s.degree()).unwrap_or(usize::MAX);
    let ctrl = s.ctrl().iter().map(|&c| p3(place(c))).collect();
    let b = sr::geom::BSpline::new(degree, s.knots().to_vec(), ctrl).map_err(|e| error(op, e))?;
    let domain = b.domain();
    Ok((sr::geom::Curve::BSpline(b), domain))
}

/// A 32-byte digest of `bytes`: four FNV-1a lanes with different seeds.
pub(crate) fn digest(bytes: &[u8]) -> [u8; 32] {
    let mut lanes: [u64; 4] = [0xcbf2_9ce4_8422_2325, 0x8422_2325_cbf2_9ce4, 0x9e37_79b9_7f4a_7c15, 0x7f4a_7c15_9e37_79b9];
    for (i, b) in bytes.iter().enumerate() {
        if let Some(lane) = lanes.get_mut(i % 4) {
            *lane = (*lane ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3);
        }
    }
    let mut out = [0u8; 32];
    for (chunk, lane) in out.chunks_mut(8).zip(lanes) {
        chunk.copy_from_slice(&lane.to_le_bytes());
    }
    out
}
