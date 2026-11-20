// yack: the panic firewall (PLAN §4.2 rule 8): the one place yack calls `catch_unwind`. A caught
// panic is `Internal`, never a refusal; the adapter reports it as a crash.
use crate::{Error, ErrorKind, Result};

/// Runs `f`, turning a panic inside it into an `Internal` error from `op`.
pub(crate) fn run<T>(op: &'static str, f: impl FnOnce() -> Result<T>) -> Result<T> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(payload) => {
            let what = payload
                .downcast_ref::<&str>()
                .map(|s| (*s).to_string())
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "a panic with no message".to_string());
            Err(Error::new(ErrorKind::Internal, op, format!("panicked: {what}")))
        }
    }
}
