# M2.2 step 4 (T3): a typed memo slot. boolean's FaceImages leave topo/memo.rs: a face memo holds
# typed slots (MemoSlot, PLAN §4.1) that a higher layer fills, so topo names no boolean type. The
# loose box stays a topo cache; memo.rs no longer names measure. Same values, set once, read the same.
# Usage: python3 t3-apply.py <yack-springrep dir>
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


edit("topo/memo.rs", [
    ("// Changes: none (verbatim); this header added.",
     "// Changes: typed memo slots (MemoSlot, M2.2 T3) replace the boolean's own field, so topo names no\n// higher layer's type."),
    ("use std::collections::HashSet;\nuse std::fmt;\n", "use std::any::{Any, TypeId};\nuse std::collections::{BTreeMap, HashSet};\nuse std::fmt;\n"),
    ("    pub images: OnceLock<crate::boolean::FaceImages>,\n    /// The face's share of [`crate::measure::loose_bbox`].\n",
     "    /// What higher layers derive from the face and keep here, one value of each [`MemoSlot`] type.\n    pub slots: Slots,\n    /// The face's share of the arena's loose box (the measures' `loose_bbox`).\n"),
    ("            images: OnceLock::new(),\n", "            slots: Slots::default(),\n"),
    ("/// What a face's exact integrals depend on besides its geometry, compared bit for bit.",
     """/// A value a higher layer derives from a face's geometry and keeps on its memo (PLAN §4.1): set
/// once while the geometry is unchanged, read back by its type.
pub(crate) trait MemoSlot: Any + Send + Sync {}

/// A face memo's typed slots, at most one value of each type.
#[derive(Default)]
pub(crate) struct Slots(Mutex<BTreeMap<TypeId, Arc<dyn Any + Send + Sync>>>);

impl fmt::Debug for Slots {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let n = self.0.lock().map_or(0, |m| m.len());
        write!(f, "Slots({n})")
    }
}

impl FaceMemo {
    /// The value of type `T` kept on this memo, if one was set.
    pub(crate) fn slot<T: MemoSlot>(&self) -> Option<Arc<T>> {
        let held = self.0_slots_get(TypeId::of::<T>())?;
        held.downcast::<T>().ok()
    }

    /// Keeps `v` on this memo, unless a value of its type is already there (it then stays).
    pub(crate) fn set_slot<T: MemoSlot>(&self, v: T) {
        if let Ok(mut m) = self.slots.0.lock() {
            m.entry(TypeId::of::<T>()).or_insert_with(|| Arc::new(v));
        }
    }

    fn 0_slots_get(&self, k: TypeId) -> Option<Arc<dyn Any + Send + Sync>> {
        self.slots.0.lock().ok()?.get(&k).cloned()
    }
}

/// What a face's exact integrals depend on besides its geometry, compared bit for bit."""),
])
# a valid name for the helper
s = open(p("topo/memo.rs")).read().replace("0_slots_get", "slot_any")
open(p("topo/memo.rs"), "w").write(s)

edit("topo/mod.rs", [("pub(crate) use memo::{FaceMemo, FaceVolume, IntegralKey};",
                      "pub(crate) use memo::{FaceMemo, FaceVolume, IntegralKey, MemoSlot};")])

edit("boolean/mod.rs", [
    ("        if let Some(img) = b.memo(f).images.get() {", "        if let Some(img) = b.memo(f).slot::<FaceImages>() {"),
    ("        let _ = b.memo(f).images.set(FaceImages {", "        b.memo(f).set_slot(FaceImages {"),
    ("        let img = b.memo(face.id).images.get();", "        let img = b.memo(face.id).slot::<FaceImages>();"),
    ("                let uvs = match img {", "                let uvs = match &img {"),
    ("#[derive(Debug)]\nstruct TrimImage {", "impl crate::topo::MemoSlot for FaceImages {}\n\n#[derive(Debug)]\nstruct TrimImage {"),
])
print("ok")
