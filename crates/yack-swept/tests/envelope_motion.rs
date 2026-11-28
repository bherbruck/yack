// yack: runs envelope/motion.rs's tests before `mod envelope;` is wired into the crate (w-h6); delete
// this file when it is.
#![allow(dead_code)]

#[path = "../src/envelope/motion.rs"]
mod motion;
