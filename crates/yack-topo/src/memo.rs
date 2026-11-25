// yack: springrep module src/topo/memo.rs
// Adapted from springrep@a03e8a1 src/topo/memo.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: typed memo slots (MemoSlot, M2.2 T3) replace the boolean's own field, so topo names no
// higher layer's type; then to yack-topo (M3 W4), paths and visibility only; M3 hardening: a memo
// is known by its address, held by a Weak while another memo lists it as apart, instead of by an
// id from a process-wide counter (D27).
//! What the checker, measures and booleans work out about a face's geometry ([`FaceMemo`]), kept
//! while that geometry is unchanged and shared by faces of the same geometry.

use std::any::{Any, TypeId};
use std::collections::BTreeMap;
use std::fmt;
use std::sync::{Arc, Mutex, OnceLock, Weak};

use crate::math::{Aabb, Point3, Vec2, Vec3};

/// What the checker and booleans derive from one face's geometry, kept while that geometry is
/// unchanged. A boolean that passes a face through uncut gives the result's face the same memo, so
/// a part built by many features measures and samples each face once rather than once a feature.
#[derive(Debug)]
pub struct FaceMemo {
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
    /// and vertices, by [`address`]. Each entry's Weak keeps its memo's allocation, and so its
    /// address, from being reused while it is listed. Only ever looked up, never iterated:
    /// addresses differ from run to run.
    apart: Mutex<BTreeMap<usize, Weak<FaceMemo>>>,
}

impl Default for FaceMemo {
    fn default() -> FaceMemo {
        FaceMemo {
            volume: OnceLock::new(),
            #[cfg(debug_assertions)]
            layout: OnceLock::new(),
            integrals: Mutex::new(None),
            moments: Mutex::new(None),
            pcurves: OnceLock::new(),
            clean: OnceLock::new(),
            slots: Slots::default(),
            loose_box: OnceLock::new(),
            apart: Mutex::new(BTreeMap::new()),
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

    /// Lists `other` as a memo whose face this one's does not meet away from what they share.
    pub(crate) fn note_apart(&self, other: &Arc<FaceMemo>) {
        if let Ok(mut apart) = self.apart.lock() {
            // Dead entries go at each power-of-two length: amortised, and the list stays bounded.
            if apart.len().is_power_of_two() {
                apart.retain(|_, memo| memo.strong_count() > 0);
            }
            apart.insert(address(other), Arc::downgrade(other));
        }
    }

    /// Whether `other` is listed as apart from this memo.
    pub(crate) fn is_apart(&self, other: &FaceMemo) -> bool {
        self.apart.lock().is_ok_and(|apart| apart.contains_key(&address(other)))
    }
}

/// A memo's address: while a memo lives, or another lists it as apart, no other memo has it.
fn address(memo: &FaceMemo) -> usize {
    std::ptr::from_ref(memo).addr()
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
