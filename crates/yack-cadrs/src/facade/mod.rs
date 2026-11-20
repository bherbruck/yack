// yack: yack's glue over cadrs (not cadrs code; owned by w-m17-cadrs): what crates/yack calls so it routes and never computes.
//! yack's own glue over cadrs (not cadrs code): face names in yack's role vocabulary
//! ([`names`]) and composite motions ([`motion`]).

pub mod motion;
pub mod names;
#[cfg(test)]
mod probe;
