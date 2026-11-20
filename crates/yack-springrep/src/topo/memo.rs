// yack: springrep module src/topo/memo.rs
// Adapted from springrep@a03e8a1 src/topo/memo.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
use std::collections::HashSet;
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use crate::math::{Aabb, Point3, Vec2, Vec3};

/// What the checker and booleans derive from one face's geometry, kept while that geometry is
/// unchanged. A boolean that passes a face through uncut gives the result's face the same memo, so
/// a part built by many features measures and samples each face once rather than once a feature.
#[derive(Debug)]
pub(crate) struct FaceMemo {
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
    pub images: OnceLock<crate::boolean::FaceImages>,
    /// The face's share of [`crate::measure::loose_bbox`].
    pub loose_box: OnceLock<Aabb>,
    /// The memos of faces the checker found this one does not meet away from their shared edges
    /// and vertices.
    pub apart: Mutex<HashSet<u64>>,
}

impl Default for FaceMemo {
    fn default() -> FaceMemo {
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
            images: OnceLock::new(),
            loose_box: OnceLock::new(),
            apart: Mutex::new(HashSet::new()),
        }
    }
}

/// What a face's exact integrals depend on besides its geometry, compared bit for bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct IntegralKey([u64; 5], bool);

impl IntegralKey {
    pub(crate) fn new(c: Point3, scale: f64, rel: f64, reversed: bool) -> IntegralKey {
        IntegralKey([c.x, c.y, c.z, scale, rel].map(f64::to_bits), reversed)
    }
}

/// A face's volume contribution `⅓∬ (p − anchor)·n dA` at the rough tolerance, and `∬ n dA`, which
/// moves it to any other centre.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FaceVolume {
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
