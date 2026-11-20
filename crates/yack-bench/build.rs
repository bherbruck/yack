// yack: generates the brep-bench dispatch table from src/cases/{category}/{name_snake}.rs.
// Adapted from brep-bench@d522d62 kernels/cadrs/build.rs (MIT OR Apache-2.0, (c) the brep-bench
// contributors).
// Changes: yack's `build() -> yack::Result<Solid>` signature; errors returned, not unwrapped; no
// Cargo.lock version scrape (the runner prints yack::VERSION and yack::SOURCE).
//! Every `src/cases/<category>/<name_snake>.rs` is one case, `<category>/<name>` with the name's
//! underscores as hyphens, and defines `pub fn build() -> yack::Result<yack::Solid>`. This writes
//! `$OUT_DIR/cases.rs`: a module per file (a raw identifier, so `box.rs` is one) and `CASES`, the
//! table from case id to its `build`, in file order. Adding a case is adding its file.
// A tool crate's build script: it reads Cargo's CARGO_MANIFEST_DIR and OUT_DIR.
#![allow(clippy::disallowed_methods)]
use std::fmt::Write as _;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

fn sorted(dir: &Path, keep: impl Fn(&Path) -> bool) -> io::Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        if keep(&path) {
            out.push(path);
        }
    }
    out.sort();
    Ok(out)
}

fn utf8(p: &Path, stem: bool) -> io::Result<String> {
    let part = if stem { p.file_stem() } else { p.file_name() };
    part.and_then(|s| s.to_str())
        .map(str::to_owned)
        .ok_or_else(|| io::Error::other(format!("not a UTF-8 name: {}", p.display())))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?);
    let cases = root.join("src/cases");
    println!("cargo:rerun-if-changed={}", cases.display());

    let (mut modules, mut table) = (String::new(), String::new());
    for dir in sorted(&cases, |p| p.is_dir())? {
        println!("cargo:rerun-if-changed={}", dir.display());
        let category = utf8(&dir, false)?;
        writeln!(modules, "pub mod {category} {{")?;
        for file in sorted(&dir, |p| p.extension().is_some_and(|x| x == "rs"))? {
            let name = utf8(&file, true)?;
            writeln!(modules, "    #[path = {:?}]\n    pub mod r#{name};", file.display().to_string())?;
            let id = format!("{category}/{}", name.replace('_', "-"));
            writeln!(table, "    ({id:?}, {category}::r#{name}::build),")?;
        }
        writeln!(modules, "}}")?;
    }
    let out = PathBuf::from(std::env::var("OUT_DIR")?).join("cases.rs");
    let text = format!(
        "{modules}\n/// Every case id and the function that builds it.\n\
         pub const CASES: &[(&str, Build)] = &[\n{table}];\n"
    );
    fs::write(out, text)?;
    Ok(())
}
