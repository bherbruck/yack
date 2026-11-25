// yack: shells turned inside out, voids outside their solid, and solids inside one another.
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/check.rs to yack-check (M3 W6); split out of lib.rs into its
// own module (M3 hardening); paths and visibility only.
//! Shells turned inside out, voids outside their solid, and solids inside one another.

use crate::{
    crossings::{in_box, padded_box},
    locate::classify,
    math::Aabb,
    measure,
    topo::{Brep, ShellId, SolidId, domain::Domain, face_domain},
};

/// Void shells, with their solids, that a point inside one of their faces finds outside the
/// solid's outer shell.
pub(crate) fn stray_voids(b: &Brep) -> Vec<(SolidId, ShellId)> {
    let mut out = Vec::new();
    for s in b.solid_ids() {
        let shells = &b.solid(s).shells;
        let n_outer = b.solid(s).outer;
        if shells.len() <= n_outer {
            continue;
        }
        let outer: Vec<(Domain, Aabb)> = shells[..n_outer]
            .iter()
            .flat_map(|&sh| b.shell(sh).faces.iter())
            .map(|&f| (face_domain(b, f), padded_box(measure::face_box(b, f))))
            .collect();
        let faces: Vec<(&Domain, Aabb)> = outer.iter().map(|(d, bb)| (d, *bb)).collect();
        for &sh in &shells[n_outer..] {
            let probe = b.shell(sh).faces.iter().find_map(|&f| {
                let d = face_domain(b, f);
                d.interior_point().map(|uv| d.surface.eval(uv.x, uv.y))
            });
            if probe.is_some_and(|p| classify::contains(&faces, p) == Some(false)) {
                out.push((s, sh));
            }
        }
    }
    out
}

/// The shells, with their solids and whether each is an outer sheet, whose faces enclose volume of
/// the wrong sign: an outer sheet's is positive, a void's negative.
pub fn turned_shells(b: &Brep) -> Vec<(SolidId, ShellId, bool)> {
    let mut out = Vec::new();
    for s in b.solid_ids() {
        let solid = b.solid(s);
        for (k, &sh) in solid.shells.iter().enumerate() {
            let outer = k < solid.outer;
            let v = measure::enclosed_volume_sign(b, &b.shell(sh).faces);
            if (v > 0.0) != outer {
                out.push((s, sh, outer));
            }
        }
    }
    out
}

/// For each void shell of solid `s`, the outer sheet it lies inside, as an index into its outer
/// shells: found by a point inside one of its faces, the first sheet where that is not found.
pub fn void_hosts(b: &Brep, s: SolidId) -> Vec<usize> {
    let shells = &b.solid(s).shells;
    let n_outer = b.solid(s).outer;
    let sheets: Vec<Vec<(Domain, Aabb)>> = shells[..n_outer]
        .iter()
        .map(|&sh| {
            b.shell(sh)
                .faces
                .iter()
                .map(|&f| (face_domain(b, f), padded_box(measure::face_box(b, f))))
                .collect()
        })
        .collect();
    shells[n_outer..]
        .iter()
        .map(|&sh| {
            let probe = b.shell(sh).faces.iter().find_map(|&f| {
                let d = face_domain(b, f);
                d.interior_point().map(|uv| d.surface.eval(uv.x, uv.y))
            });
            let inside = |k: &usize| {
                let faces: Vec<(&Domain, Aabb)> =
                    sheets[*k].iter().map(|(d, bb)| (d, *bb)).collect();
                probe.is_some_and(|p| classify::contains(&faces, p) == Some(true))
            };
            (0..n_outer).find(inside).unwrap_or(0)
        })
        .collect()
}

/// Pairs of solids whose faces do not meet but one of which lies inside the other, found by a
/// point inside a face of the first.
pub(crate) fn nested(b: &Brep) -> Vec<(SolidId, SolidId)> {
    let solids: Vec<SolidId> = b.solid_ids().collect();
    if solids.len() < 2 {
        return Vec::new();
    }
    let parts: Vec<Vec<(Domain, Aabb)>> = solids
        .iter()
        .map(|&s| {
            b.solid_faces(s)
                .into_iter()
                .map(|f| (face_domain(b, f), padded_box(measure::face_box(b, f))))
                .collect()
        })
        .collect();
    let boxes: Vec<Aabb> = parts
        .iter()
        .map(|fs| {
            fs.iter().fold(Aabb::EMPTY, |mut acc, (_, bb)| {
                acc.union(bb);
                acc
            })
        })
        .collect();
    let mut out = Vec::new();
    for (i, inner) in parts.iter().enumerate() {
        let Some(probe) =
            inner.iter().find_map(|(d, _)| d.interior_point().map(|uv| d.surface.eval(uv.x, uv.y)))
        else {
            continue;
        };
        for (j, outer) in parts.iter().enumerate() {
            if i == j || !in_box(probe, &boxes[j]) {
                continue;
            }
            let faces: Vec<(&Domain, Aabb)> = outer.iter().map(|(d, bb)| (d, *bb)).collect();
            if classify::contains(&faces, probe) == Some(true) {
                out.push((solids[i], solids[j]));
            }
        }
    }
    out
}
