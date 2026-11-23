// yack: `cargo xtask step-gap`, yack-step's STEP measurer against the brep-bench grader on the bench's own files.
//! `cargo xtask step-gap [--run FILE] [--ref DIR]`
//!
//! Measures every STEP file the latest graded bench run wrote with yack-step's measurer (its
//! `measure` example, built with `wip`), and counts the files where the measurer refuses or where
//! its volume or area is off the grader's by more than [`AGREES`]. The expected values are only the
//! grader's own, from the run's results JSON; nothing here computes one (AGENTS rule 9).
//!
//! The run is the newest `runs/brep-bench/yack-*.json` (or `--run`). It must have been made at or
//! after the last commit that can change a bench result, and no STEP file may be newer than it;
//! otherwise the files are not the run's, and the gap is refused rather than measured.
//!
//! It prints the run, its tree, each file that misses and why, and `step-gap N`. With
//! `scores/step-gap.txt` present (the lead records it, as for `eps.txt`), a larger N fails.
use crate::json::{self, Json};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

/// The grader's own agreement: volume and area to 1e-6 relative (as yack-step's file tests).
const AGREES: f64 = 1e-6;

/// The repository root (this crate is `crates/xtask`).
fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("..")
}

pub fn run(args: &[String]) -> ExitCode {
    match drive(args) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("xtask step-gap: {e}");
            ExitCode::FAILURE
        }
    }
}

fn drive(args: &[String]) -> Result<bool, String> {
    let root = root();
    let mut reference = root.join("../yack-ref");
    let mut run_file: Option<PathBuf> = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--run" => run_file = Some(PathBuf::from(it.next().ok_or("--run needs a file")?)),
            "--ref" => reference = PathBuf::from(it.next().ok_or("--ref needs a directory")?),
            other => return Err(format!("unknown argument `{other}`")),
        }
    }
    let run_file = match run_file {
        Some(f) => f,
        None => newest_run(&reference.join("runs/brep-bench"))?,
    };
    let text = std::fs::read_to_string(&run_file).map_err(|e| format!("{}: {e}", run_file.display()))?;
    let run = json::parse(&text)?;
    let tree = run_tree(&run).ok_or("the run names no binary of a known tree")?;
    println!("step-gap: run {} (tree {tree})", run_file.display());
    // The run must postdate the last commit that can change a bench result.
    let affecting = git(&root, &["log", "-1", "--format=%h", "--", "crates", ":!crates/xtask", ":!crates/yack-conformance", ":!crates/yack-acceptance", ":!crates/*/tests", ":!crates/*/api.txt"])?;
    let is_ancestor = Command::new("git")
        .args(["merge-base", "--is-ancestor", affecting.trim(), &tree])
        .current_dir(&root)
        .status()
        .map_err(|e| format!("git: {e}"))?;
    if !is_ancestor.success() {
        return Err(format!(
            "the run's tree {tree} is older than {}, the last commit that can change a bench result: run the bench first",
            affecting.trim()
        ));
    }
    let made = std::fs::metadata(&run_file).and_then(|m| m.modified()).map_err(|e| format!("{}: {e}", run_file.display()))?;
    let meshes = reference.join("brep-bench/results/meshes/yack");

    // The files the grader measured, with its volume and area.
    let mut want: Vec<(String, PathBuf, f64, f64)> = Vec::new();
    for c in run.get("cases").map(Json::as_arr).unwrap_or(&[]) {
        if c.get("measured").and_then(Json::as_str) != Some("step") {
            continue;
        }
        let (Some(case), Some(v), Some(a)) = (c.get("case").and_then(Json::as_str), num(c.get("volume")), num(c.get("area"))) else {
            continue;
        };
        let path = meshes.join(format!("{case}.step"));
        let newer = std::fs::metadata(&path).and_then(|m| m.modified()).map_err(|e| format!("{}: {e}", path.display()))?;
        if newer > made {
            return Err(format!("{} is newer than the run: the files are not the run's; run the bench again", path.display()));
        }
        want.push((case.to_string(), path, v, a));
    }

    let built = Command::new("cargo")
        .args(["build", "--release", "-p", "yack-step", "--features", "wip", "--example", "measure"])
        .current_dir(&root)
        .status()
        .map_err(|e| format!("cargo: {e}"))?;
    if !built.success() {
        return Err("cargo build --release -p yack-step --features wip --example measure failed".into());
    }
    let measurer = root.join("bin/release/examples/measure");
    let mut misses = Vec::new();
    for chunk in want.chunks(64) {
        let out = Command::new(&measurer)
            .args(chunk.iter().map(|w| &w.1))
            .output()
            .map_err(|e| format!("{}: {e}", measurer.display()))?;
        let lines: Vec<String> = String::from_utf8_lossy(&out.stdout).lines().map(str::to_string).collect();
        for w in chunk {
            let p = w.1.display().to_string();
            let line = lines.iter().find(|l| l.split('\t').next() == Some(p.as_str()));
            let fields: Vec<&str> = line.map(|l| l.split('\t').collect()).unwrap_or_default();
            let why = match fields.as_slice() {
                [_, "error", why] => Some(format!("refused: {why}")),
                [_, v, a] => match (v.parse::<f64>(), a.parse::<f64>()) {
                    (Ok(v), Ok(a)) => {
                        let off = |x: f64, y: f64| (x - y).abs() / y.abs().max(1.0);
                        let (dv, da) = (off(v, w.2), off(a, w.3));
                        (dv > AGREES || da > AGREES).then(|| format!("volume {v} against {} ({dv:.1e}), area {a} against {} ({da:.1e})", w.2, w.3))
                    }
                    _ => Some(format!("unreadable output: {}", fields.join(" "))),
                },
                _ => Some("no output".into()),
            };
            if let Some(why) = why {
                misses.push((w.0.clone(), why));
            }
        }
    }
    for (case, why) in &misses {
        println!("  {case}: {why}");
    }
    println!("step-gap {} of {} files", misses.len(), want.len());
    let recorded = root.join("scores/step-gap.txt");
    if let Ok(t) = std::fs::read_to_string(&recorded) {
        let base = t.lines().filter(|l| !l.starts_with('#')).find_map(|l| l.split_whitespace().next()?.parse::<usize>().ok());
        if let Some(base) = base
            && misses.len() > base
        {
            println!("step-gap: {} misses against {base} recorded in {}: worse", misses.len(), recorded.display());
            return Ok(false);
        }
    }
    Ok(true)
}

/// The newest run of the whole suite: `yack-*.json` with every case, not a single case's rerun.
fn newest_run(dir: &Path) -> Result<PathBuf, String> {
    let mut best: Option<(std::time::SystemTime, PathBuf)> = None;
    for e in std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?.flatten() {
        let p = e.path();
        let name = p.file_name().and_then(|n| n.to_str()).unwrap_or("");
        if !name.starts_with("yack-") || !name.ends_with(".json") || name.contains("-alone-") {
            continue;
        }
        let Ok(t) = e.metadata().and_then(|m| m.modified()) else { continue };
        if best.as_ref().is_none_or(|b| t > b.0) {
            best = Some((t, p));
        }
    }
    best.map(|b| b.1).ok_or_else(|| format!("no yack-*.json run in {}", dir.display()))
}

/// The commit the run's binary was built from: its folder is `<tag>-<sha>+<hash>`.
fn run_tree(run: &Json) -> Option<String> {
    let bin = run.get("binary")?.get("of")?.as_arr().first()?.as_str()?;
    let folder = Path::new(bin).parent()?.file_name()?.to_str()?;
    let (head, _) = folder.rsplit_once('+')?;
    let sha = head.rsplit('-').next()?;
    (sha.len() >= 7 && sha.chars().all(|c| c.is_ascii_hexdigit())).then(|| sha.to_string())
}

fn num(j: Option<&Json>) -> Option<f64> {
    match j? {
        Json::Num(n) => n.parse().ok(),
        _ => None,
    }
}

fn git(root: &Path, args: &[&str]) -> Result<String, String> {
    let out = Command::new("git").args(args).current_dir(root).output().map_err(|e| format!("git: {e}"))?;
    if !out.status.success() {
        return Err(format!("git {}: {}", args.join(" "), String::from_utf8_lossy(&out.stderr)));
    }
    Ok(String::from_utf8_lossy(&out.stdout).into_owned())
}
