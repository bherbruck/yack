// yack: Hashes the kernel's sources, unit tests left out, into `SPRINGREP_SOURCE`.
// Adapted from springrep@a03e8a1 build.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! Hashes the kernel's sources, unit tests left out, into `SPRINGREP_SOURCE`.

use std::path::{Path, PathBuf};

fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = std::fs::read_dir(dir).expect("the crate's sources are readable");
    for entry in entries {
        let path = entry.expect("the crate's sources are readable").path();
        if path.is_dir() {
            sources(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs")
            && path.file_name().is_some_and(|n| n != "tests.rs")
        {
            out.push(path);
        }
    }
}

fn main() {
    println!("cargo:rerun-if-changed=src");
    let mut files = Vec::new();
    sources(Path::new("src"), &mut files);
    files.sort();
    // FNV-1a: the build needs a stable digest, not a secure one, and takes no dependency for it.
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for f in files {
        let name = f.to_string_lossy().replace('\\', "/");
        let text = std::fs::read(&f).expect("the crate's sources are readable");
        for b in name.bytes().chain([0]).chain(text).chain([0]) {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    println!("cargo:rustc-env=SPRINGREP_SOURCE={h:016x}");
}
