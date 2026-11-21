// yack: nalgebra-shaped linear algebra (f64): the subset cadrs_kernel uses, moved here from yack-shim (M4.1c).
//! Fixed-size matrices up to 6 × 6, points, unit wrappers, rotations, quaternions, isometries,
//! dynamic matrices, LU (nalgebra's algorithm step for step), and Jacobi eigen/SVD decompositions
//! (accurate, not bit-identical to nalgebra's), in nalgebra 0.35's shape so that the cadrs-derived
//! code compiles unchanged. The arithmetic keeps nalgebra's evaluation order.
//!
//! Its index and shape contracts (`v[i]`, `m[(r, c)]`, a slice too short for a matrix) panic as
//! nalgebra's do: counted debt in scores/lints.txt, gone when M5 moves the callers onto
//! yack-math's checked types.
#![forbid(unsafe_code)]
// Until M5, alongside the counted contract panics: nalgebra's shape indexes by position.
#![allow(clippy::indexing_slicing)]

pub mod decomp;
pub mod dmatrix;
pub mod geometry;
pub mod matrix;
pub mod ops;
pub mod point;
pub mod storage;
pub mod unit;

pub use decomp::{DLU, DSymmetricEigen, LU, SVD, SquareSVD, SymmetricEigen};
pub use dmatrix::{ColKind, DMatrix, DVector, Dyn, Dynamic, U1};
pub use geometry::{
    Isometry, Isometry3, IsometryMatrix3, Quaternion, Rotation, Rotation2, Rotation3, Translation,
    Translation2, Translation3, UnitQuaternion,
};
pub use matrix::{
    Matrix, Matrix1, Matrix2, Matrix2x3, Matrix2x4, Matrix3, Matrix3x2, Matrix3x4, Matrix3x6,
    Matrix4, Matrix4x3, Matrix4x6, Matrix5, Matrix6, Matrix6x3, Matrix6x4, RowSVector, RowVector2,
    RowVector3, RowVector4, SMatrix, SVector, Vector1, Vector2, Vector3, Vector4, Vector5, Vector6,
};
pub use point::{OPoint, Point, Point1, Point2, Point3, Point4, center, distance, distance_squared};
pub use storage::{Scalar, XY, XYZ, XYZW};
pub use unit::{Normed, Unit};
