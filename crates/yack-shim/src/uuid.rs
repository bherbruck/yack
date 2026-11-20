// yack: uuid-shaped 128-bit identifiers: made from numbers only (no random v4 ids).
//! Stored as uuid's 16 big-endian bytes, so ordering, equality and hashing behave as uuid's
//! `Uuid` does.

use std::fmt;

/// A 128-bit identifier.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Uuid([u8; 16]);

impl Uuid {
    /// The all-zero id.
    pub const fn nil() -> Uuid {
        Uuid([0; 16])
    }
    /// The id with value `v`.
    pub const fn from_u128(v: u128) -> Uuid {
        Uuid(v.to_be_bytes())
    }
    /// The id of these bytes.
    pub const fn from_bytes(b: [u8; 16]) -> Uuid {
        Uuid(b)
    }
    /// The value.
    pub const fn as_u128(&self) -> u128 {
        u128::from_be_bytes(self.0)
    }
    /// The bytes.
    pub const fn as_bytes(&self) -> &[u8; 16] {
        &self.0
    }
    /// Whether it is the nil id.
    pub fn is_nil(&self) -> bool {
        self.0 == [0; 16]
    }
}

impl fmt::Display for Uuid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let b = &self.0;
        for (i, x) in b.iter().enumerate() {
            if matches!(i, 4 | 6 | 8 | 10) {
                write!(f, "-")?;
            }
            write!(f, "{x:02x}")?;
        }
        Ok(())
    }
}

impl fmt::Debug for Uuid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

impl std::str::FromStr for Uuid {
    type Err = &'static str;
    fn from_str(s: &str) -> Result<Uuid, &'static str> {
        let hex: String = s.chars().filter(|c| *c != '-').collect();
        if hex.len() != 32 {
            return Err("uuid: wrong length");
        }
        u128::from_str_radix(&hex, 16).map(Uuid::from_u128).map_err(|_| "uuid: not hexadecimal")
    }
}
