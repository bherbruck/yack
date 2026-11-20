// yack: root of the temporary yack-shim crate (PLAN §3.2): the external APIs cadrs_kernel uses.
//! Zero-dependency stand-ins, shaped like the parts of nalgebra 0.35, rayon 1, rustc-hash 2.1,
//! web-time 1 and uuid 1 that cadrs_kernel calls, so that its source compiles unchanged. Only
//! `yack-cadrs` depends on this crate (gate rule 12); `yack-cadrs` depends on it under the name
//! `nalgebra` and reaches the other facades through `extern crate nalgebra as rayon;` and so on,
//! which is why every facade's items sit at this crate's root.
//!
//! - nalgebra: [`linalg`]. Fixed-size matrices up to 6 × 6, points, unit wrappers, rotations,
//!   quaternions, isometries, dynamic matrices, LU (nalgebra's algorithm step for step), and
//!   Jacobi eigen/SVD decompositions (accurate, not bit-identical to nalgebra's).
//! - rayon: [`par`]. Eager, ordered, scoped-thread parallel steps.
//! - rustc-hash: [`fx`]. FxHasher, the same hashes as rustc-hash 2.1 on 64-bit targets.
//! - web-time: [`time`]. Clocks that stand still.
//! - uuid: [`uuid`]. Ids from numbers.
//! - spade: [`spade`]. A constrained Delaunay triangulation with refinement (reached as
//!   `nalgebra::spade`, its `Point2` being spade's, not nalgebra's).
#![forbid(unsafe_code)]

pub mod fx;
pub mod linalg;
pub mod par;
pub mod spade;
pub mod time;
pub mod uuid;

// ---- nalgebra ----
pub use linalg::decomp::{DLU, DSymmetricEigen, LU, SVD, SquareSVD, SymmetricEigen};
pub use linalg::dmatrix::{ColKind, DMatrix, DVector, Dyn, Dynamic, U1};
pub use linalg::geometry::{
    Isometry, Isometry3, IsometryMatrix3, Quaternion, Rotation, Rotation2, Rotation3, Translation,
    Translation2, Translation3, UnitQuaternion,
};
pub use linalg::matrix::{
    Matrix, Matrix1, Matrix2, Matrix2x3, Matrix2x4, Matrix3, Matrix3x2, Matrix3x4, Matrix3x6,
    Matrix4, Matrix4x3, Matrix4x6, Matrix5, Matrix6, Matrix6x3, Matrix6x4, RowSVector, RowVector2,
    RowVector3, RowVector4, SMatrix, SVector, Vector1, Vector2, Vector3, Vector4, Vector5, Vector6,
};
pub use linalg::point::{OPoint, Point, Point1, Point2, Point3, Point4, center, distance, distance_squared};
pub use linalg::storage::{Scalar, XY, XYZ, XYZW};
pub use linalg::unit::{Normed, Unit};

// ---- rayon ----
pub use par::{
    ThreadBuilder, ThreadPool, ThreadPoolBuildError, ThreadPoolBuilder, current_num_threads,
    current_thread_index, join, prelude,
};

// ---- rustc-hash ----
pub use fx::{FxBuildHasher, FxHashMap, FxHashSet, FxHasher};

// ---- web-time ----
pub use time::{Instant, SystemTime, SystemTimeError, UNIX_EPOCH};

// ---- uuid ----
pub use uuid::Uuid;

#[cfg(test)]
mod tests;
