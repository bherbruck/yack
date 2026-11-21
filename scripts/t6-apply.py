# M2.4 (T6): edit/tweak.rs + edit/tweak/{through,tests}.rs move to src/tweak/{mod,through,tests}.rs, a
# top-level module (it becomes yack-tweak, layer 7). Paths only; edit keeps re-exporting the four ops.
# Usage: python3 t6-apply.py <yack-springrep dir>
import os, sys

src = os.path.join(sys.argv[1], "src")
p = lambda *a: os.path.join(src, *a)


def edit(f, pairs):
    s = open(p(f)).read()
    for a, b in pairs:
        if s.count(a) != 1:
            raise SystemExit(f"{f}: expected one {a[:70]!r}, found {s.count(a)}")
        s = s.replace(a, b)
    open(p(f), "w").write(s)


def move(a, b):
    s = open(p(a)).read()
    s = s.replace("// Changes: none (verbatim); this header added.",
                  f"// Changes: moved from {a} to {b} (M2.4, T6); paths only.", 1)
    os.makedirs(os.path.dirname(p(b)), exist_ok=True)
    open(p(b), "w").write(s)
    os.remove(p(a))


move("edit/tweak.rs", "tweak/mod.rs")
move("edit/tweak/through.rs", "tweak/through.rs")
move("edit/tweak/tests.rs", "tweak/tests.rs")
os.rmdir(p("edit/tweak"))

edit("edit.rs", [
    ("pub(crate) mod tweak;\n", ""),
    ("pub use tweak::{draft, move_faces, move_faces_by, offset_faces};",
     """pub use crate::tweak::{move_faces, move_faces_by};

/// `b` with each of `faces` tilted by `angle` about the line where it meets the `neutral` plane, its
/// outward normal turned towards the plane's normal: positive angles narrow the part along it.
pub fn draft(b: &Brep, faces: &[FaceId], neutral: &Frame, angle: f64) -> Result<Brep> {
    merged(crate::tweak::draft(b, faces, neutral, angle)?)
}

/// `b` with each face of `by` moved that far along its outward normal, onto its offset surface.
pub fn offset_faces(b: &Brep, by: &HashMap<FaceId, f64>) -> Result<Brep> {
    merged(crate::tweak::offset_faces(b, by)?)
}

/// A move's solid, the faces round any that closed on the way merged; if that fails, the error the
/// direct move gave.
fn merged(m: crate::tweak::Moved) -> Result<Brep> {
    match m.merge {
        None => Ok(m.brep),
        Some(e) => boolean::unify(&m.brep).map_err(|_| e),
    }
}"""),
])
s = open(p("edit.rs")).read()
n = s.count("tweak::offset_faces(s, &by)")
s = s.replace("        tweak::offset_faces(s, &by)", "        offset_faces(s, &by)").replace("= tweak::offset_faces(s, &by)?", "= offset_faces(s, &by)?")
open(p("edit.rs"), "w").write(s)
if n != 3:
    raise SystemExit(f"edit.rs: expected 3 tweak::offset_faces calls, found {n}")

edit("boolean/snap.rs", [("    edit::tweak,\n", "    tweak,\n")])

# Ruling (b): tweak may not call boolean::unify (boolean depends on tweak). `through` returns whether faces
# closed on the way; tweak's draft/offset_faces hand that up as `Moved`; edit (which may use boolean) merges.
edit("tweak/through.rs", [
    ("    Result,\n    boolean::unify,\n", "    Result,\n"),
    ("/// `b` with the faces of `paths` moved to the ends of their paths, its topology changed where a\n/// face closes on the way.\npub(crate) fn through(b: &Brep, paths: &HashMap<FaceId, Path>) -> Result<Brep> {",
     "/// `b` with the faces of `paths` moved to the ends of their paths, its topology changed where a\n/// face closes on the way, and whether one did: the faces round it are then for the caller to merge\n/// (`boolean::unify`, which this layer may not call).\npub(crate) fn through(b: &Brep, paths: &HashMap<FaceId, Path>) -> Result<(Brep, bool)> {"),
    ("            Ok(out) if changed => return unify(&out),\n            Ok(out) => return Ok(out),\n",
     "            Ok(out) => return Ok((out, changed)),\n"),
])
edit("tweak/mod.rs", [
    ("/// `b` with each of `faces` tilted by `angle` about the line where it meets the `neutral` plane, its\n/// outward normal turned towards the plane's normal: positive angles narrow the part along it.\npub fn draft(b: &Brep, faces: &[FaceId], neutral: &Frame, angle: f64) -> Result<Brep> {",
     """/// A move's result: the solid, and where faces closed on the way, the error the direct move gave.
/// The caller then merges the faces round the closed ones (`boolean::unify`, which this layer may
/// not call), and reports that error if the merge fails.
pub(crate) struct Moved {
    pub brep: Brep,
    pub merge: Option<Error>,
}

/// The direct move, or where it fails, the move followed through the faces that close on the way.
fn followed(direct: Result<Brep>, through: impl FnOnce() -> Result<(Brep, bool)>) -> Result<Moved> {
    match direct {
        Ok(brep) => Ok(Moved { brep, merge: None }),
        Err(e) => match through() {
            Ok((brep, true)) => Ok(Moved { brep, merge: Some(e) }),
            Ok((brep, false)) => Ok(Moved { brep, merge: None }),
            Err(_) => Err(e),
        },
    }
}

/// `b` with each of `faces` tilted by `angle` about the line where it meets the `neutral` plane, its
/// outward normal turned towards the plane's normal: positive angles narrow the part along it.
pub(crate) fn draft(b: &Brep, faces: &[FaceId], neutral: &Frame, angle: f64) -> Result<Moved> {"""),
    ("    direct.or_else(|e| through::through(b, &paths).map_err(|_| e))\n}",
     "    followed(direct, || through::through(b, &paths))\n}"),
    ("/// `b` with each face of `by` moved that far along its outward normal, onto its offset surface.\npub fn offset_faces(b: &Brep, by: &HashMap<FaceId, f64>) -> Result<Brep> {",
     "/// `b` with each face of `by` moved that far along its outward normal, onto its offset surface.\npub(crate) fn offset_faces(b: &Brep, by: &HashMap<FaceId, f64>) -> Result<Moved> {"),
    ("    offset_kept(b, by).or_else(|e| {\n        let mut paths", "    followed(offset_kept(b, by), || {\n        let mut paths"),
    ("        through::through(b, &paths).map_err(|_| e)\n    })\n}", "        through::through(b, &paths)\n    })\n}"),
])
edit("tweak/tests.rs", [("use super::*;\n", "use super::*;\nuse crate::edit::{draft, offset_faces};\n")])
edit("lib.rs", [("pub mod tol;\npub mod topo;\n", "pub mod tol;\npub mod topo;\npub(crate) mod tweak;\n")])
print("ok")
