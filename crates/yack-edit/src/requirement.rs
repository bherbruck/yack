// yack: which refusals break a stated requirement (moved from springrep's facade/api.rs, M3 W10).
//! Which of an operation's refusals say the caller broke a stated requirement (a size that must be
//! positive, a blend that must fit), rather than that the kernel could not do it.

/// Whether springrep's refusal `detail` from yack op `op` says the input breaks a stated
/// requirement of the operation (yack's §4.3 docs, which follow brep-conformance doc/api.md);
/// any other springrep refusal is a limit of springrep, not of the request. Each entry names the
/// requirement it stands for.
pub fn breaks_requirement(op: &str, detail: &str) -> bool {
    const BLENDS: &[&str] = &[
        "fillet", "fillet_radii", "fillet_variable", "fillet_conic", "fillet_g2", "fillet_setback", "fillet_setback_g2",
        "fillet_faces", "full_round", "chamfer", "chamfer_two", "chamfer_distances",
    ];
    let is = |ops: &[&str]| ops.contains(&op);
    // sizes, radii, heights and blend sizes must be positive (block, cylinder, sphere, cone, every blend)
    detail.contains("must be positive")
        // every number given must be finite (every op; not a computed spline's knots)
        || detail.ends_with("must be finite")
        // a slab's heights must all lie above its floor
        || (op == "slab" && detail.contains("above its floor"))
        // a sweep's path needs a leg
        || (op == "sweep" && detail.contains("needs a leg"))
        // a radius or distance must fit the faces it lies on (every fillet and chamfer)
        || (is(BLENDS)
            && (detail.contains("takes a whole face")
                || detail.contains("runs past the end of an edge")
                || detail.contains("does not fit on its faces")
                || detail.contains("the faces are parallel")
                // a blend needs an edge to work on
                || detail.contains("no edges to")))
        // a cone needs a positive height and a radius at one end at least
        || (op == "cone" && detail.contains("needs a positive height"))
        // an extrusion's height must be positive
        || (op == "extrude" && detail.contains("must be nonzero"))
        // loft_through's heights must rise
        || (op == "loft_through" && detail.contains("do not come in order"))
        // a revolved or screwed profile must keep to one side of the axis
        || (is(&["revolve", "screw"]) && detail.contains("axis crosses the profile"))
        // a screw's turns must stay apart
        || (op == "screw" && detail.contains("coils meet"))
        // a torus needs major > minor > 0
        || (op == "torus" && detail.contains("clear of its axis"))
        // a shell's walls must not meet; a moved face may not pass the face it meets
        || (is(&["shell", "offset_faces", "move_faces"]) && detail.contains("turns inside out"))
}

#[cfg(test)]
mod tests {
    use super::breaks_requirement;

    #[test]
    fn a_blend_with_no_edges_breaks_a_requirement() {
        assert!(breaks_requirement("fillet", "no edges to round"));
        assert!(breaks_requirement("chamfer", "no edges to chamfer"));
        assert!(!breaks_requirement("shell", "no edges to round"));
        assert!(breaks_requirement("extrude", "extrusion distance must be finite"));
        assert!(breaks_requirement("slab", "a slab's heights must all lie above its floor"));
        assert!(breaks_requirement("sweep", "a sweep's path needs a leg"));
        assert!(!breaks_requirement("loft", "B-spline knots must be finite and not decrease"));
    }
}
