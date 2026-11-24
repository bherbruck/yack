// yack: the names features give faces, and the face-name pairs that pick edges: plain data.
// Adapted from springrep@a03e8a1 src/topo/names.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from topo/names.rs to names.rs (M2), being plain data topo and profile both use;
// EdgeName joined it from blend.rs (M2.5, T8); then to yack-topo (M3 W4), paths and visibility only.
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
    pub fn all<'a>(
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

/// One edge pair of a blend: the faces named `a` and `b` (as `feature.face`). It picks every edge
/// where a face on `a`'s surface meets one on `b`'s, or with `exact` only where the faces carrying
/// those names meet, so a pair on two surfaces that meet more than once picks one of the edges.
#[derive(Clone, Copy, Debug)]
pub struct EdgeName<'a> {
    pub a: &'a str,
    pub b: &'a str,
    pub exact: bool,
}
