# M2.2 step 2 (T4b): face memos are no longer encoded; a decoded arena starts with fresh memos.
# Usage: python3 m22b.py <yack-springrep dir>
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


def cut_between(f, start, end):
    s = open(p(f)).read()
    i = s.index(start)
    k = s.index(end, i)
    open(p(f), "w").write(s[:i] + s[k:])


# topo/codec.rs: drop put_memo/get_memo and the memo stream.
cut_between("topo/codec.rs", "/// What a face's memo has worked out that holds in any process", "impl Codec for Brep {")
edit("topo/codec.rs", [
    ("// Changes: none (verbatim); this header added.",
     "// Changes: face memos are no longer encoded (M2.2 T4b): a decoded arena starts with fresh memos,\n// so the bytes, and the digest, no longer depend on what was asked of a solid before."),
    ("use super::memo::{FaceMemo, Memos};", "use super::memo::Memos;"),
    ("            memos,\n            tolerances,\n            partners,\n            beside,\n            uses: _,\n        } = self;",
     "            memos: _,\n            tolerances,\n            partners,\n            beside,\n            uses: _,\n        } = self;"),
    ("        for m in &memos.0 {\n            put_memo(m, w);\n        }\n", ""),
    ("        let memos = (0..faces.len())\n            .map(|_| get_memo(r))\n            .collect::<Option<Vec<_>>>()?;\n",
     "        // Memos are what a process has worked out about the faces: they start empty.\n        let memos = (0..faces.len()).map(|_| Arc::default()).collect();\n"),
])

# codec.rs: the Writer and Reader no longer carry memos.
edit("codec.rs", [
    ("// Changes: none (verbatim); this header added.",
     "// Changes: the Writer and Reader no longer carry face memos (M2.2 T4b)."),
    ("use std::collections::{BTreeMap, BTreeSet, HashMap};\nuse std::sync::Arc;\n", "use std::collections::{BTreeMap, BTreeSet};\n"),
    ("    topo::{Brep, FaceMemo},\n", "    topo::Brep,\n"),
    ("/// The bytes written so far, and the face memos among them by where each lives, so faces that\n/// share a memo share it again when read back.\n#[derive(Default)]\npub(crate) struct Writer(pub Vec<u8>, pub HashMap<usize, u32>);",
     "/// The bytes written so far.\n#[derive(Default)]\npub(crate) struct Writer(pub Vec<u8>);"),
    ("    bytes: &'a [u8],\n    pub memos: Vec<Arc<FaceMemo>>,\n}", "    bytes: &'a [u8],\n}"),
    ("        Reader {\n            bytes,\n            memos: Vec::new(),\n        }", "        Reader { bytes }"),
])

# boolean/mod.rs: FaceImages are no longer encoded (this also ends boolean -> codec).
edit("boolean/mod.rs", [("crate::codec::fields!(FaceImages {\n    trims,\n    samples,\n    boxes,\n    bbox\n});\ncrate::codec::fields!(TrimImage { t, uv, whole });\n\n", "")])

# codec/tests.rs: memos are not carried; the bytes do not depend on queries run before.
edit("codec/tests.rs", [
    ("// Changes: none (verbatim); this header added.",
     "// Changes: memos are no longer encoded (M2.2 T4b): the two memo tests become one that the bytes do\n// not depend on what was asked of the solid."),
    ("""fn a_built_solid_reads_back_with_its_ids_names_and_memos() {
    let b = drilled_and_rounded();
    let volume = measure::volume(&b);
    let back: Brep = back(&bytes(&b)).unwrap();
    assert_eq!(back, b);
    assert_eq!(bytes(&back), bytes(&b));
    // The volume each face's memo holds came across, and gives the same total to the last bit.
    assert!(back.face_ids().all(|f| back.memo(f).volume.get().is_some()));
    assert_eq!(measure::volume(&back).to_bits(), volume.to_bits());
}""", """fn a_built_solid_reads_back_with_its_ids_and_names() {
    let b = drilled_and_rounded();
    let volume = measure::volume(&b);
    let back: Brep = back(&bytes(&b)).unwrap();
    assert_eq!(back, b);
    assert_eq!(bytes(&back), bytes(&b));
    // Measured again from its read-back geometry, it has the same volume to the last bit.
    assert_eq!(measure::volume(&back).to_bits(), volume.to_bits());
}"""),
    ("""fn memos_come_back_shared_as_they_were_with_the_trims_a_boolean_read() {
    // A cut reads the trims of the faces it meets into their memos; a later boolean takes those
    // instead of working them out again. Here a 4 mm square post through the block's corner.
    let b = drilled_and_rounded();
    let square = Profile::rect(Vec2::new(10.0, 10.0), 4.0, 4.0);
    let post = extrude(&square, &at_z(-5.0), 20.0, "post").unwrap();
    difference(&b, &post).unwrap();
    let read: Vec<_> = b
        .face_ids()
        .filter(|&f| b.memo(f).images.get().is_some())
        .collect();
    assert!(!read.is_empty());
    let pair = vec![b.clone(), b];
    let back: Vec<Brep> = back(&bytes(&pair)).unwrap();
    for &f in &read {
        assert!(back[0].memo(f).images.get().is_some());
        assert!(Arc::ptr_eq(
            &back[0].shared_memo(f),
            &back[1].shared_memo(f)
        ));
    }
    assert_eq!(bytes(&back), bytes(&pair));
}""", """fn the_bytes_do_not_depend_on_what_was_asked_of_the_solid() {
    // Queries fill a face's memo (its volume share, its box, the trims a boolean reads); none of
    // that is written, so a solid encodes the same before and after them.
    let b = drilled_and_rounded();
    let before = bytes(&b);
    measure::volume(&b);
    measure::area(&b);
    let rays = crate::rays::Rays::new(&b).unwrap();
    rays.side(p(5.0, 5.0, 5.0));
    let square = Profile::rect(Vec2::new(10.0, 10.0), 4.0, 4.0);
    let post = extrude(&square, &at_z(-5.0), 20.0, "post").unwrap();
    difference(&b, &post).unwrap();
    assert_eq!(bytes(&b), before);
    let pair = vec![b.clone(), b];
    let back: Vec<Brep> = back(&bytes(&pair)).unwrap();
    assert_eq!(bytes(&back), bytes(&pair));
}"""),
])
print("ok")
