// yack: web-time-shaped clocks that never move, so no result can depend on timing.
//! `Instant::now()` is always the same instant and `SystemTime::now()` is the Unix epoch. Code
//! that times itself reads 0 elapsed; a wall-clock budget never runs out.

use std::ops::{Add, AddAssign, Sub, SubAssign};
use std::time::Duration;

/// An instant of a clock that stands still.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Instant(Duration);

impl Instant {
    /// The (one) instant.
    pub fn now() -> Instant {
        Instant(Duration::ZERO)
    }
    /// Zero.
    pub fn elapsed(&self) -> Duration {
        Duration::ZERO
    }
    /// The time from `earlier` (saturating at zero).
    pub fn duration_since(&self, earlier: Instant) -> Duration {
        self.0.saturating_sub(earlier.0)
    }
    /// The time from `earlier`, if it is earlier.
    pub fn checked_duration_since(&self, earlier: Instant) -> Option<Duration> {
        self.0.checked_sub(earlier.0)
    }
    /// The time from `earlier` (saturating at zero).
    pub fn saturating_duration_since(&self, earlier: Instant) -> Duration {
        self.duration_since(earlier)
    }
    /// `d` later.
    pub fn checked_add(&self, d: Duration) -> Option<Instant> {
        self.0.checked_add(d).map(Instant)
    }
    /// `d` earlier.
    pub fn checked_sub(&self, d: Duration) -> Option<Instant> {
        self.0.checked_sub(d).map(Instant)
    }
}

impl Sub for Instant {
    type Output = Duration;
    fn sub(self, b: Instant) -> Duration {
        self.duration_since(b)
    }
}
impl Add<Duration> for Instant {
    type Output = Instant;
    fn add(self, d: Duration) -> Instant {
        Instant(self.0 + d)
    }
}
impl AddAssign<Duration> for Instant {
    fn add_assign(&mut self, d: Duration) {
        self.0 += d;
    }
}
impl Sub<Duration> for Instant {
    type Output = Instant;
    fn sub(self, d: Duration) -> Instant {
        Instant(self.0.saturating_sub(d))
    }
}
impl SubAssign<Duration> for Instant {
    fn sub_assign(&mut self, d: Duration) {
        self.0 = self.0.saturating_sub(d);
    }
}

/// A time of a calendar clock that stands at the Unix epoch.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct SystemTime(Duration);

/// The Unix epoch.
pub const UNIX_EPOCH: SystemTime = SystemTime(Duration::ZERO);

/// The error of a `duration_since` from a later time.
#[derive(Clone, Copy, Debug)]
pub struct SystemTimeError(Duration);

impl SystemTimeError {
    /// How much later it was.
    pub fn duration(&self) -> Duration {
        self.0
    }
}
impl std::fmt::Display for SystemTimeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "second time provided was later than self")
    }
}
impl std::error::Error for SystemTimeError {}

impl SystemTime {
    /// The Unix epoch.
    pub const UNIX_EPOCH: SystemTime = UNIX_EPOCH;
    /// The Unix epoch.
    pub fn now() -> SystemTime {
        UNIX_EPOCH
    }
    /// The time from `earlier`.
    pub fn duration_since(&self, earlier: SystemTime) -> Result<Duration, SystemTimeError> {
        self.0.checked_sub(earlier.0).ok_or(SystemTimeError(earlier.0 - self.0))
    }
    /// Zero.
    pub fn elapsed(&self) -> Result<Duration, SystemTimeError> {
        Ok(Duration::ZERO)
    }
}
