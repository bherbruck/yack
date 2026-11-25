// yack: the facade's route to yack-blend's own rungs (M4.8): routes, never computes.
use crate::engine;
use crate::{Angle, EdgeId, FaceId, Result, Solid};

/// `s` with `edge` bevelled `distance` along `face` at `angle` to it, by yack-blend's own
/// strategies (a rim between coaxial faces of revolution today); a decline elsewhere.
pub(crate) fn chamfer_angle(s: &Solid, edge: EdgeId, face: FaceId, distance: f64, angle: Angle) -> Result<Solid> {
    let op = "chamfer_angle";
    let (e, f) = (s.sr_edge(op, "edge", edge)?, s.sr_face(op, "face", face)?);
    // the bevel face carries this op's feature (`bevel`)
    let feature = engine::feature();
    engine::made(&feature, op, yack_blend::chamfer_angle(s.need(op)?, e, f, distance, angle.radians(), &feature))
}
