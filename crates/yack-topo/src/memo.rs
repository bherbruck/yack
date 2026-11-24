// yack: springrep module src/topo/memo.rs
// Adapted from springrep@a03e8a1 src/topo/memo.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: typed memo slots (MemoSlot, M2.2 T3) replace the boolean's own field, so topo names no
// higher layer's type; then to yack-topo (M3 W4), paths and visibility only.
use std::any::{Any, TypeId};
#[allow(clippy::disallowed_types)] use std::collections::{BTreeMap, HashSet}; // D26: BTree* in hardening, proven by digests
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use crate::math::{Aabb, Point3, Vec2, Vec3};

/// What the checker and booleans derive from one face's geometry, kept while that geometry is
/// unchanged. A boolean that passes a face through uncut gives the result's face the same memo, so
/// a part built by many features measures and samples each face once rather than once a feature.
#[derive(Debug)]
pub struct FaceMemo {
    /// Names this memo, and so one face's geometry, for as long as the process runs.
    pub id: u64,
    pub volume: OnceLock<FaceVolume>,
    /// Each coedge's edge range and sense, loop by loop, of the first face the memo was handed
    /// on from: what is kept here by a coedge's place holds only for a face laid out the same.
    #[cfg(debug_assertions)]
    pub layout: OnceLock<Vec<Vec<(u64, u64, bool)>>>,
    /// The face's last area and volume share, and its last moments with its area, each with the
    /// centre, size, tolerance and sense they were integrated for: a solid measured for its
    /// volume, then its area, then its centre, asks for each face's again with the same ones.
    pub integrals: Mutex<Option<(IntegralKey, (f64, f64))>>,
    pub moments: Mutex<Option<(IntegralKey, [f64; 11])>>,
    /// Each projected coedge's parameters along its edge, loop by loop, that seed the integrals'
    /// projections (see `measure::pcurve_tables`).
    pub pcurves: OnceLock<Vec<Vec<Vec<Vec2>>>>,
    /// The face's own loops lie on its surface, close in its parameters and run its way.
    pub clean: OnceLock<()>,
    /// What higher layers derive from the face and keep here, one value of each [`MemoSlot`] type.
    pub slots: Slots,
    /// The face's share of the arena's loose box (the measures' `loose_bbox`).
    pub loose_box: OnceLock<Aabb>,
    /// The memos of faces the checker found this one does not meet away from their shared edges
    /// and vertices.
    #[allow(clippy::disallowed_types)] pub apart: Mutex<HashSet<u64>>, // D26: BTree* in hardening, proven by digests
}

impl Default for FaceMemo {
    #[allow(clippy::disallowed_types)] fn default() -> FaceMemo { // D26: BTree* in hardening, proven by digests
        static NEXT: AtomicU64 = AtomicU64::new(0);
        FaceMemo {
            id: NEXT.fetch_add(1, Ordering::Relaxed),
            volume: OnceLock::new(),
            #[cfg(debug_assertions)]
            layout: OnceLock::new(),
            integrals: Mutex::new(None),
            moments: Mutex::new(None),
            pcurves: OnceLock::new(),
            clean: OnceLock::new(),
            slots: Slots::default(),
            loose_box: OnceLock::new(),
            apart: Mutex::new(HashSet::new()),
        }
    }
}

/// A value a higher layer derives from a face's geometry and keeps on its memo (PLAN §4.1): set
/// once while the geometry is unchanged, read back by its type.
pub trait MemoSlot: Any + Send + Sync {}

/// A face memo's typed slots, at most one value of each type.
#[derive(Default)]
pub struct Slots(Mutex<BTreeMap<TypeId, Arc<dyn Any + Send + Sync>>>);

impl fmt::Debug for Slots {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let n = self.0.lock().map_or(0, |m| m.len());
        write!(f, "Slots({n})")
    }
}

impl FaceMemo {
    /// The value of type `T` kept on this memo, if one was set.
    pub fn slot<T: MemoSlot>(&self) -> Option<Arc<T>> {
        let held = self.slot_any(TypeId::of::<T>())?;
        held.downcast::<T>().ok()
    }

    /// Keeps `v` on this memo, unless a value of its type is already there (it then stays).
    pub fn set_slot<T: MemoSlot>(&self, v: T) {
        if let Ok(mut m) = self.slots.0.lock() {
            m.entry(TypeId::of::<T>()).or_insert_with(|| Arc::new(v));
        }
    }

    fn slot_any(&self, k: TypeId) -> Option<Arc<dyn Any + Send + Sync>> {
        self.slots.0.lock().ok()?.get(&k).cloned()
    }
}

/// What a face's exact integrals depend on besides its geometry, compared bit for bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IntegralKey([u64; 5], bool);

impl IntegralKey {
    pub fn new(c: Point3, scale: f64, rel: f64, reversed: bool) -> IntegralKey {
        IntegralKey([c.x, c.y, c.z, scale, rel].map(f64::to_bits), reversed)
    }
}

/// A face's volume contribution `⅓∬ (p − anchor)·n dA` at the rough tolerance, and `∬ n dA`, which
/// moves it to any other centre.
#[derive(Clone, Copy, Debug)]
pub struct FaceVolume {
    pub anchor: Point3,
    pub volume: f64,
    pub vector_area: Vec3,
}

/// One memo per face of an arena. Never part of equality: two arenas with the same entities are
/// equal whatever either has worked out about them.
#[derive(Clone, Default)]
pub(crate) struct Memos(pub Vec<Arc<FaceMemo>>);

impl PartialEq for Memos {
    fn eq(&self, _: &Memos) -> bool {
        true
    }
}

impl fmt::Debug for Memos {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Memos({})", self.0.len())
    }
}
