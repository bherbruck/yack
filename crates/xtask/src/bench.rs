// yack: `cargo xtask bench`, brep-bench through its own harness, ratcheted (PLAN §5.2, §6.4).
//! `cargo xtask bench [--cases GLOB] [--bless] [--brep-bench DIR]`
//!
//! Builds `bin/release/yack-bench`, then runs `crates/yack-bench/harness.py`, which hands
//! brep-bench's own `harness/run.py` a local `yack` kernel entry (nothing in the brep-bench clone
//! is edited), writes `bin/bench/yack.json`, prints mean/max `loc` and runner lines per case, and
//! ratchets outcome, `loc` and time per case against `scores/bench.json` (`--bless` accepts).
//! `--cases` takes a case-id prefix, with or without a trailing `*` (`'swept/*'`).
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

/// The repository root (this crate is `crates/xtask`).
fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("..")
}

/// What `cargo xtask bench` was asked, or why it could not be read.
#[derive(Debug, Default, PartialEq)]
struct Args {
    prefix: Option<String>,
    bless: bool,
    brep_bench: Option<String>,
}

fn parse(args: &[String]) -> Result<Args, String> {
    let mut out = Args::default();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--bless" => out.bless = true,
            "--cases" => {
                let glob = it.next().ok_or("--cases needs a value, e.g. 'swept/*'")?;
                let prefix = glob.strip_suffix('*').unwrap_or(glob);
                if prefix.contains(['*', '?', '[']) {
                    return Err(format!("--cases {glob}: only a prefix, or a prefix and `*`"));
                }
                out.prefix = Some(prefix.to_owned());
            }
            "--brep-bench" => {
                out.brep_bench = Some(it.next().ok_or("--brep-bench needs a directory")?.clone());
            }
            other => return Err(format!("unknown argument `{other}`")),
        }
    }
    Ok(out)
}

/// `cargo xtask bench ARGS`: the arguments after `bench`.
pub fn run(args: &[String]) -> ExitCode {
    let args = match parse(args) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("xtask bench: {e}");
            return ExitCode::from(2);
        }
    };
    let root = root();
    let built = Command::new("cargo")
        .args(["build", "--release", "-p", "yack-bench"])
        .current_dir(&root)
        .status();
    if !built.is_ok_and(|s| s.success()) {
        eprintln!("xtask bench: cargo build --release -p yack-bench failed");
        return ExitCode::FAILURE;
    }
    let mut harness = Command::new("python3");
    harness
        .arg(root.join("crates/yack-bench/harness.py"))
        .arg("--out")
        .arg(root.join("bin/bench/yack.json"))
        .arg("--runner")
        .arg(root.join("bin/release/yack-bench"))
        .arg("--scores")
        .arg(root.join("scores/bench.json"))
        .current_dir(&root);
    if let Some(p) = &args.prefix {
        harness.args(["--case", p]);
    }
    if let Some(d) = &args.brep_bench {
        harness.args(["--brep-bench", d]);
    }
    if args.bless {
        harness.arg("--bless");
    }
    match harness.status() {
        Ok(s) if s.success() => ExitCode::SUCCESS,
        Ok(_) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("xtask bench: python3 did not run: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(xs: &[&str]) -> Vec<String> {
        xs.iter().map(|x| x.to_string()).collect()
    }

    #[test]
    fn reads_a_glob_as_a_prefix() {
        let a = parse(&s(&["--cases", "swept/*", "--bless"])).unwrap();
        assert_eq!(a.prefix.as_deref(), Some("swept/"));
        assert!(a.bless);
    }

    #[test]
    fn refuses_inner_wildcards_and_unknown_flags() {
        assert!(parse(&s(&["--cases", "sw*pt/*"])).is_err());
        assert!(parse(&s(&["--fast"])).is_err());
        assert!(parse(&s(&["--cases"])).is_err());
    }
}
