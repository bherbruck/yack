// yack: element storage for the nalgebra-shaped fixed-size matrices (column-major).
//! A matrix's elements live in a [`Storage`]: a column-major array, or, for the 2-, 3- and
//! 4-element column vectors, a struct with named fields so that `v.x` reads as in nalgebra.
//! [`Shape`] picks the storage for each size; it is implemented for every size up to 6 × 6.

use core::fmt::Debug;

/// An element type: what a matrix may hold.
pub trait Scalar: Copy + PartialEq + Debug + 'static {}
impl<T: Copy + PartialEq + Debug + 'static> Scalar for T {}

/// Column-major element storage of a fixed-size matrix.
pub trait Storage<T>: Copy + PartialEq + Debug {
    /// The storage holding `f(i)` at linear (column-major) index `i`.
    fn from_lin(f: impl FnMut(usize) -> T) -> Self;
    /// The element at linear index `i` (panics past the end, as nalgebra's indexing does).
    fn at(&self, i: usize) -> &T;
    /// The element at linear index `i`, mutably.
    fn at_mut(&mut self, i: usize) -> &mut T;
    /// Every element, mutably, in linear order.
    fn all_mut(&mut self) -> Vec<&mut T>;
}

/// The coordinates of a 2-vector.
#[derive(Clone, Copy, PartialEq, Debug, Default, Hash, Eq)]
pub struct XY<T> {
    pub x: T,
    pub y: T,
}

/// The coordinates of a 3-vector.
#[derive(Clone, Copy, PartialEq, Debug, Default, Hash, Eq)]
pub struct XYZ<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

/// The coordinates of a 4-vector.
#[derive(Clone, Copy, PartialEq, Debug, Default, Hash, Eq)]
pub struct XYZW<T> {
    pub x: T,
    pub y: T,
    pub z: T,
    pub w: T,
}

#[expect(clippy::panic, reason = "an index contract, as nalgebra's: counted in scores/lints.txt until M5")]
fn out_of_bounds(i: usize, n: usize) -> ! {
    panic!("matrix index {i} out of bounds (length {n})")
}

impl<T: Scalar> Storage<T> for XY<T> {
    fn from_lin(mut f: impl FnMut(usize) -> T) -> Self {
        let x = f(0);
        let y = f(1);
        XY { x, y }
    }
    fn at(&self, i: usize) -> &T {
        match i {
            0 => &self.x,
            1 => &self.y,
            _ => out_of_bounds(i, 2),
        }
    }
    fn at_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => out_of_bounds(i, 2),
        }
    }
    fn all_mut(&mut self) -> Vec<&mut T> {
        vec![&mut self.x, &mut self.y]
    }
}

impl<T: Scalar> Storage<T> for XYZ<T> {
    fn from_lin(mut f: impl FnMut(usize) -> T) -> Self {
        let x = f(0);
        let y = f(1);
        let z = f(2);
        XYZ { x, y, z }
    }
    fn at(&self, i: usize) -> &T {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => out_of_bounds(i, 3),
        }
    }
    fn at_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => out_of_bounds(i, 3),
        }
    }
    fn all_mut(&mut self) -> Vec<&mut T> {
        vec![&mut self.x, &mut self.y, &mut self.z]
    }
}

impl<T: Scalar> Storage<T> for XYZW<T> {
    fn from_lin(mut f: impl FnMut(usize) -> T) -> Self {
        let x = f(0);
        let y = f(1);
        let z = f(2);
        let w = f(3);
        XYZW { x, y, z, w }
    }
    fn at(&self, i: usize) -> &T {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            3 => &self.w,
            _ => out_of_bounds(i, 4),
        }
    }
    fn at_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            3 => &mut self.w,
            _ => out_of_bounds(i, 4),
        }
    }
    fn all_mut(&mut self) -> Vec<&mut T> {
        vec![&mut self.x, &mut self.y, &mut self.z, &mut self.w]
    }
}

impl<T: Scalar, const R: usize, const C: usize> Storage<T> for [[T; R]; C] {
    fn from_lin(mut f: impl FnMut(usize) -> T) -> Self {
        core::array::from_fn(|c| core::array::from_fn(|r| f(c * R + r)))
    }
    fn at(&self, i: usize) -> &T {
        if i >= R * C {
            out_of_bounds(i, R * C)
        }
        &self[i / R][i % R]
    }
    fn at_mut(&mut self, i: usize) -> &mut T {
        if i >= R * C {
            out_of_bounds(i, R * C)
        }
        &mut self[i / R][i % R]
    }
    fn all_mut(&mut self) -> Vec<&mut T> {
        self.iter_mut().flat_map(|c| c.iter_mut()).collect()
    }
}

/// The marker type every matrix size is a [`Shape`] of.
#[derive(Clone, Copy, Debug)]
pub struct Sh;

/// Picks the storage of an `R × C` matrix of `T`.
pub trait Shape<T, const R: usize, const C: usize> {
    /// The storage.
    type S: Storage<T>;
}

impl<T: Scalar> Shape<T, 2, 1> for Sh {
    type S = XY<T>;
}
impl<T: Scalar> Shape<T, 3, 1> for Sh {
    type S = XYZ<T>;
}
impl<T: Scalar> Shape<T, 4, 1> for Sh {
    type S = XYZW<T>;
}

macro_rules! array_shapes {
    ($(($r:literal, $c:literal)),* $(,)?) => {
        $(impl<T: Scalar> Shape<T, $r, $c> for Sh {
            type S = [[T; $r]; $c];
        })*
    };
}

array_shapes!(
    (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6),
    (5, 1), (6, 1),
    (2, 2), (2, 3), (2, 4), (2, 5), (2, 6),
    (3, 2), (3, 3), (3, 4), (3, 5), (3, 6),
    (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
    (5, 2), (5, 3), (5, 4), (5, 5), (5, 6),
    (6, 2), (6, 3), (6, 4), (6, 5), (6, 6),
);
