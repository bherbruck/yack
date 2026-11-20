// yack: springrep module src/topo/names.rs
// Adapted from springrep@a03e8a1 src/topo/names.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
use std::fmt;

/// The name a feature gave a face: `feature.face`. A face can
/// carry several (see doc/design.md, "History").
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FaceName {
    pub feature: String,
    pub face: String,
}

impl FaceName {
    /// The face a sweep or loft starts from: its profile on the sketch plane, or its first section.
    pub const START: &'static str = "start";
    /// The face a sweep or loft ends on.
    pub const END: &'static str = "end";
    /// A circle's one edge, and every side face of a loft.
    pub const SIDE: &'static str = "side";
    /// `feature`'s face for each of `names`.
    pub(crate) fn all<'a>(
        feature: &str,
        names: impl IntoIterator<Item = &'a String>,
    ) -> Vec<FaceName> {
        let names = names.into_iter();
        names.map(|n| FaceName::new(feature, n.clone())).collect()
    }

    /// The faces of the cavity a shell makes.
    pub const INNER: &'static str = "inner";

    pub fn new(feature: impl Into<String>, face: impl Into<String>) -> FaceName {
        FaceName {
            feature: feature.into(),
            face: face.into(),
        }
    }
}

impl fmt::Display for FaceName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.feature, self.face)
    }
}
