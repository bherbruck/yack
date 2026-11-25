// yack: the facade's route to yack-swept's own rungs (M4.1): routes, never computes.
use crate::engine;
use crate::input::{LeadKind, MotionKind};
use crate::{Error, ErrorKind, Motion, Result, Solid};

/// `s` swept along `m` by yack-swept: a slide, a turn or a screw. A motion it does not build
/// yet (along a path, one after another, several at once) is a decline.
pub(crate) fn swept(s: &Solid, m: &Motion) -> Result<Solid> {
    let op = "swept";
    let motion = match &m.0 {
        MotionKind::Slide { by } => yack_swept::Motion::Slide { by: engine::v3(*by) },
        MotionKind::Turn { axis_point, axis, from, to } => yack_swept::Motion::Turn {
            axis_point: engine::p3(*axis_point),
            axis: engine::v3(*axis),
            from: from.radians(),
            to: to.radians(),
        },
        MotionKind::Screw { axis_point, axis, from, to, lead } => yack_swept::Motion::Screw {
            axis_point: engine::p3(*axis_point),
            axis: engine::v3(*axis),
            from: from.radians(),
            to: to.radians(),
            rise: match &lead.0 {
                LeadKind::PerTurn(x) => yack_swept::Rise::PerTurn(*x),
                LeadKind::PerRadian(x) => yack_swept::Rise::PerRadian(*x),
                LeadKind::Polynomial(c) => yack_swept::Rise::Polynomial(c.clone()),
            },
        },
        MotionKind::Along { .. } | MotionKind::Then(_) | MotionKind::Together(_) => {
            return Err(Error::new(
                ErrorKind::Unsupported,
                op,
                "this motion is not built natively yet",
            ));
        }
    };
    // faces the motion makes carry this op's feature (`end`, `side`; `start` if one survives)
    let feature = engine::feature();
    engine::made(&feature, op, yack_swept::swept(s.need(op)?, &motion, &feature))
}
