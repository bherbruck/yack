// yack: cadrs result faces named in yack's role vocabulary (seg:…, start, end, round_n, bevel_n, corner, inner).
//! cadrs says where each result face came from ([`History`]); yack names faces by role. This turns
//! one into the other, so the facade only copies names: no geometry in `crates/yack`.
//!
//! Per result face, [`face_roles`] gives what the face is called: the input faces whose names it
//! keeps ([`FaceRole::Keep`]) and the role parts yack's `feature.part` names carry
//! ([`FaceRole::Part`]). A face with neither carries just the feature (as springrep's unnamed
//! faces do).

use crate::native::topo::Body;
use crate::{BodyId, DefaultKernel, History, InputFace, Origin, Result};

/// What one result face is called, in part.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FaceRole {
    /// It continues this input face: it keeps that face's names.
    Keep(InputFace),
    /// It carries `feature.<part>`, `part` in yack's role vocabulary (`seg:x`, `start`, `end`,
    /// `round_3`, `bevel_0`, `corner`, `inner`).
    Part(String),
}

/// The kind of faces a blend makes along its edges.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Blend {
    /// Not a blend.
    #[default]
    None,
    /// A fillet: `round_<n>`.
    Round,
    /// A chamfer: `bevel_<n>`.
    Bevel,
}

/// The side of the solid a shell's wall lies on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wall {
    /// Inside: the input's faces stay with their names; the cavity's faces are `inner`.
    Inward,
    /// Outside: the input's faces become the cavity's, keeping their names beside `inner`; the
    /// new outside carries no name.
    Outward,
}

/// How to name an operation's result, beyond its history.
#[derive(Clone, Debug, Default)]
pub struct Naming<'a> {
    /// The profile's segment names in yack's order (outer loop, then each hole), each already a
    /// whole part (`seg:<label>`, or `seg:s<n>` for an unlabelled one); for a sweep of a profile.
    pub segments: &'a [String],
    /// How many segments each loop of that profile has, in the same order.
    pub loop_lens: &'a [usize],
    /// The blend's edges, in the call's order (`round_<n>`/`bevel_<n>` is the n-th).
    pub blend_edges: &'a [u64],
    /// Which blend, if any.
    pub blend: Blend,
    /// For a shell: its wall's side.
    pub shell: Option<Wall>,
}

/// The name part of profile curve `curve` (its `source`, or [`crate::unsourced_curve`] of its
/// place): its segment's name, else `seg:s<n>` by its place in the profile; `None` if it is
/// neither.
fn segment_part(curve: u64, how: &Naming) -> Option<String> {
    let index = match usize::try_from(curve) {
        Ok(i) if i < how.segments.len() => Some(i),
        _ => {
            // An unsourced curve: index `i` of loop `lp`, numbered through every loop.
            let back = u64::MAX - curve;
            let (lp, i) = (usize::try_from(back >> 32).ok()?, usize::try_from(back & 0xffff_ffff).ok()?);
            if lp >= how.loop_lens.len() || i >= how.loop_lens[lp] {
                None
            } else {
                Some(how.loop_lens[..lp].iter().sum::<usize>() + i)
            }
        }
    }?;
    Some(how.segments.get(index).cloned().unwrap_or_else(|| format!("seg:s{index}")))
}

/// The part a generated face's origin gives it, if any.
fn origin_part(o: &Origin, how: &Naming) -> Option<String> {
    match *o {
        Origin::ProfileCurve { curve, .. } => segment_part(curve, how),
        Origin::StartCap { .. } => Some("start".into()),
        Origin::EndCap { .. } => Some("end".into()),
        Origin::FromEdge { edge, .. } => {
            let n = how.blend_edges.iter().position(|&e| e == edge.0)?;
            match how.blend {
                Blend::Round => Some(format!("round_{n}")),
                Blend::Bevel => Some(format!("bevel_{n}")),
                Blend::None => None,
            }
        }
        Origin::FromVertex { .. } => Some("corner".into()),
        Origin::PathSegment { .. } => None,
    }
}

/// Whether result face `f` of `out` lies on the very surface of input face `from` of `input`
/// (the input face left in place, not a moved copy of it).
fn in_place(out: &Body, f: usize, input: Option<&Body>, from: &InputFace) -> bool {
    let (Some(input), Ok(i)) = (input, usize::try_from(from.face.0)) else {
        return false;
    };
    match (out.faces.get(f), input.faces.get(i)) {
        (Some(a), Some(b)) => a.surf == b.surf,
        _ => false,
    }
}

/// What each face of `result` (by index) is called, from `history` and `how`. `input` is the
/// operation's input body, needed to tell a shell's walls apart.
pub fn face_roles(
    k: &DefaultKernel,
    input: Option<BodyId>,
    result: BodyId,
    history: &History,
    how: &Naming,
) -> Result<Vec<Vec<FaceRole>>> {
    let out = k.get(result)?;
    let inp = match input {
        Some(id) => Some(k.get(id)?),
        None => None,
    };
    Ok(roles(out, inp.map(|b| &**b), history, how))
}

/// [`face_roles`] on the bodies themselves.
pub fn roles(out: &Body, input: Option<&Body>, history: &History, how: &Naming) -> Vec<Vec<FaceRole>> {
    let n = out.faces.len();
    let mut roles: Vec<Vec<FaceRole>> = vec![Vec::new(); n];
    // Faces that stay on an input face's own surface (a shell's untouched faces).
    let mut kept_in_place = vec![false; n];
    let mut has_history = vec![false; n];
    for (f, from) in &history.modified {
        let Some(i) = usize::try_from(f.0).ok().filter(|&i| i < n) else { continue };
        has_history[i] = true;
        let stays = in_place(out, i, input, from);
        kept_in_place[i] |= stays;
        match how.shell {
            // A moved copy of an input face is a cavity wall.
            Some(Wall::Inward) if !stays => push(&mut roles[i], FaceRole::Part("inner".into())),
            // The new outside of an outward wall carries no name.
            Some(Wall::Outward) if !stays => {}
            _ => push(&mut roles[i], FaceRole::Keep(*from)),
        }
    }
    for (f, o) in &history.generated {
        let Some(i) = usize::try_from(f.0).ok().filter(|&i| i < n) else { continue };
        has_history[i] = true;
        match how.shell {
            // A round in a shell's cavity is part of the cavity.
            Some(Wall::Inward) => push(&mut roles[i], FaceRole::Part("inner".into())),
            Some(Wall::Outward) => {}
            None => {
                if let Some(p) = origin_part(o, how) {
                    push(&mut roles[i], FaceRole::Part(p));
                }
            }
        }
    }
    for i in 0..n {
        match how.shell {
            Some(Wall::Outward) if kept_in_place[i] => push(&mut roles[i], FaceRole::Part("inner".into())),
            // A face the shell made with no history at all is a cavity wall too.
            Some(Wall::Inward) if !has_history[i] => push(&mut roles[i], FaceRole::Part("inner".into())),
            _ => {}
        }
    }
    roles
}

fn push(v: &mut Vec<FaceRole>, r: FaceRole) {
    if !v.contains(&r) {
        v.push(r);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EdgeId, FaceId, VertexId};

    fn how<'a>(segments: &'a [String], loop_lens: &'a [usize], edges: &'a [u64], blend: Blend) -> Naming<'a> {
        Naming { segments, loop_lens, blend_edges: edges, blend, shell: None }
    }

    #[test]
    fn profile_curves_name_their_segments() {
        let segs = vec!["seg:top".to_string(), "seg:s1".to_string(), "seg:s2".to_string()];
        let h = how(&segs, &[2, 1], &[], Blend::None);
        assert_eq!(segment_part(0, &h).as_deref(), Some("seg:top"));
        // Unsourced: loop 1, index 0 is the third segment.
        assert_eq!(segment_part(crate::unsourced_curve(1, 0), &h).as_deref(), Some("seg:s2"));
        assert_eq!(segment_part(crate::unsourced_curve(0, 1), &h).as_deref(), Some("seg:s1"));
        assert_eq!(segment_part(crate::unsourced_curve(3, 0), &h), None);
    }

    #[test]
    fn origins_give_yack_parts() {
        let h = how(&[], &[], &[7, 9], Blend::Bevel);
        let body = BodyId(1);
        assert_eq!(origin_part(&Origin::StartCap { region: 0 }, &h).as_deref(), Some("start"));
        assert_eq!(origin_part(&Origin::EndCap { region: 0 }, &h).as_deref(), Some("end"));
        assert_eq!(origin_part(&Origin::FromEdge { body, edge: EdgeId(9) }, &h).as_deref(), Some("bevel_1"));
        assert_eq!(origin_part(&Origin::FromEdge { body, edge: EdgeId(8) }, &h), None);
        assert_eq!(origin_part(&Origin::FromVertex { body, vertex: VertexId(0) }, &h).as_deref(), Some("corner"));
        assert_eq!(origin_part(&Origin::PathSegment { segment: 2 }, &h), None);
    }

    #[test]
    fn an_inward_shell_names_its_cavity_inner() {
        use crate::{Kernel, ShellSpec};
        let mut k = DefaultKernel::new();
        let b = k
            .primitive(&crate::Primitive::Box {
                min: nalgebra::Point3::new(0.0, 0.0, 0.0),
                max: nalgebra::Point3::new(10.0, 10.0, 10.0),
            })
            .and_then(|r| r.first())
            .unwrap();
        let spec = ShellSpec { remove: vec![FaceId(0)], thickness: 1.0, outward: false, hollow: false, sharp: false };
        let r = k.shell_with(b, &spec).unwrap();
        let out = r.first().unwrap();
        let how = Naming { shell: Some(Wall::Inward), ..Naming::default() };
        let roles = face_roles(&k, Some(b), out, &r.history, &how).unwrap();
        let inner = roles.iter().filter(|r| r.contains(&FaceRole::Part("inner".into()))).count();
        let kept = roles.iter().filter(|r| r.iter().any(|x| matches!(x, FaceRole::Keep(_)))).count();
        // An open box: 5 outside faces and the rim left of the open face (on its plane) kept with
        // their names, as springrep's shell keeps them; 5 cavity walls.
        assert_eq!((kept, inner), (6, 5), "{roles:?}");
    }
}
