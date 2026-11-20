// yack: `cargo xtask conformance`: build the adapter, run the pinned suite, ratchet (PLAN §5.1, §6.4).
//! Builds `yack-conformance` in release into `bin/`, runs the brep-conformance runner from the
//! reference checkout, writes `bin/conformance/seed<N>.json`, and ratchets each case's outcome,
//! tier and time against `scores/conformance/seed<N>.json`.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use crate::json::{self, Json};

const USAGE: &str = "\
cargo xtask conformance [--seed N | --seeds A-B] [--family F[,G]] [--threads N] [--ref DIR]
                        [--adapter PATH] [--bless] [--record-digests]
  --seed N       one seed (default 0); --seeds A-B a range, e.g. 0-3
  --family F     only these families (passed to the runner); the ratchet then checks only them
  --threads N    runner threads (default 6)
  --adapter PATH use this adapter binary (a frozen copy) and build nothing
  --ref DIR      the yack-ref checkout holding bin/release/brep-conformance (default ../yack-ref)
  --bless        write the run as the new baseline in scores/conformance/
  --record-digests  (M1.6) record result digests in scores/digests/
";

/// A case time below this many ms is too short to ratchet: its noise exceeds 2×.
const TIME_FLOOR_MS: f64 = 20.0;

struct Opts {
    seeds: Vec<u64>,
    family: Option<String>,
    threads: u32,
    reference: PathBuf,
    adapter: Option<PathBuf>,
    bless: bool,
}

fn root() -> PathBuf {
    // crates/xtask → the workspace root.
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn parse(args: &[String]) -> Result<Opts, String> {
    let mut o = Opts {
        seeds: vec![0],
        family: None,
        threads: 6,
        reference: root().join("../yack-ref"),
        adapter: None,
        bless: false,
    };
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut value = || it.next().cloned().ok_or(format!("{a} needs a value"));
        match a.as_str() {
            "--seed" => o.seeds = vec![value()?.parse().map_err(|e| format!("--seed: {e}"))?],
            "--seeds" => {
                let v = value()?;
                let (lo, hi) = v.split_once('-').ok_or("--seeds takes A-B")?;
                let n = |s: &str| s.parse::<u64>().map_err(|e| format!("--seeds: {e}"));
                o.seeds = (n(lo)?..=n(hi)?).collect();
            }
            "--family" => o.family = Some(value()?),
            "--threads" => o.threads = value()?.parse().map_err(|e| format!("--threads: {e}"))?,
            "--ref" => o.reference = PathBuf::from(value()?),
            "--adapter" => o.adapter = Some(PathBuf::from(value()?)),
            "--bless" => o.bless = true,
            "--record-digests" => return Err("--record-digests lands with M1.6".into()),
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("unknown argument `{other}`")),
        }
    }
    Ok(o)
}

pub fn run(args: &[String]) -> ExitCode {
    let o = match parse(args) {
        Ok(o) => o,
        Err(e) => {
            if !e.is_empty() {
                eprintln!("xtask conformance: {e}\n");
            }
            eprint!("{USAGE}");
            return ExitCode::FAILURE;
        }
    };
    match drive(&o) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("xtask conformance: {e}");
            ExitCode::FAILURE
        }
    }
}

fn drive(o: &Opts) -> Result<bool, String> {
    let root = root();
    let runner = o.reference.join("bin/release/brep-conformance");
    if !runner.is_file() {
        return Err(format!("no runner at {} (build brep-conformance @ 12ffc75 there)", runner.display()));
    }
    let adapter = match &o.adapter {
        Some(a) if a.is_file() => a.clone(),
        Some(a) => return Err(format!("no adapter at {}", a.display())),
        None => {
            let status = Command::new("cargo")
                .args(["build", "--release", "-p", "yack-conformance"])
                .current_dir(&root)
                .status()
                .map_err(|e| format!("cargo build: {e}"))?;
            if !status.success() {
                return Err("building yack-conformance failed".into());
            }
            root.join("bin/release/yack-conformance")
        }
    };
    let out_dir = root.join("bin/conformance");
    std::fs::create_dir_all(&out_dir).map_err(|e| format!("{}: {e}", out_dir.display()))?;
    let mut ok = true;
    for &seed in &o.seeds {
        let out = out_dir.join(format!("seed{seed}.json"));
        let mut cmd = Command::new(&runner);
        cmd.arg("--adapter").arg(&adapter).arg("--seed").arg(seed.to_string());
        cmd.arg("--threads").arg(o.threads.to_string()).arg("--out").arg(&out);
        if let Some(f) = &o.family {
            cmd.arg("--family").arg(f);
        }
        let status = cmd.status().map_err(|e| format!("{}: {e}", runner.display()))?;
        if !status.success() {
            eprintln!("seed {seed}: the runner exited with {status}");
        }
        let text = std::fs::read_to_string(&out).map_err(|e| format!("{}: {e}", out.display()))?;
        let got = cases(&json::parse(&text)?)?;
        summary(seed, &got);
        let base_path = root.join(format!("scores/conformance/seed{seed}.json"));
        if o.bless {
            std::fs::write(&base_path, scores(&got)).map_err(|e| format!("{}: {e}", base_path.display()))?;
            println!("seed {seed}: blessed {}", base_path.display());
            continue;
        }
        match std::fs::read_to_string(&base_path) {
            Err(_) => println!("seed {seed}: no baseline at {}; --bless records one", base_path.display()),
            Ok(t) => ok &= ratchet(seed, &cases(&json::parse(&t)?)?, &got, o.family.is_some()),
        }
    }
    Ok(ok)
}

#[derive(Clone, Debug, PartialEq)]
struct Case {
    outcome: String,
    tier: String,
    time_ms: f64,
}

/// The cases of a runner report (`cases: [{id, outcome, result, time_ms}]`) or of a baseline
/// (`{id: {outcome, tier, time_ms}}`), by id.
fn cases(v: &Json) -> Result<BTreeMap<String, Case>, String> {
    let s = |c: &Json, k: &str| c.get(k).and_then(Json::as_str).unwrap_or("").to_string();
    let t = |c: &Json| match c.get("time_ms") {
        Some(Json::Num(n)) => n.parse().unwrap_or(0.0),
        _ => 0.0,
    };
    if let Some(list) = v.get("cases") {
        let one = |c: &Json| {
            let id = c.get("id").and_then(Json::as_str).ok_or("a case without an id")?;
            let case = Case { outcome: s(c, "outcome"), tier: s(c, "result"), time_ms: t(c) };
            Ok((id.to_string(), case))
        };
        return list.as_arr().iter().map(one).collect();
    }
    let Json::Obj(map) = v else {
        return Err("neither a runner report nor a baseline".into());
    };
    let one = |c: &Json| Case { outcome: s(c, "outcome"), tier: s(c, "tier"), time_ms: t(c) };
    Ok(map.iter().map(|(id, c)| (id.clone(), one(c))).collect())
}

fn summary(seed: u64, got: &BTreeMap<String, Case>) {
    let mut n: BTreeMap<&str, usize> = BTreeMap::new();
    for c in got.values() {
        *n.entry(c.outcome.as_str()).or_default() += 1;
    }
    let total = got.values().map(|c| c.time_ms).sum::<f64>().max(0.0);
    let parts: Vec<String> = n.iter().map(|(k, v)| format!("{v} {k}")).collect();
    println!("seed {seed}: {} cases: {}; {total:.0} ms in cases", got.len(), parts.join(", "));
}

/// The baseline as committed: sorted by id, one case a line, no timestamps.
fn scores(got: &BTreeMap<String, Case>) -> String {
    let mut s = String::from("{\n");
    for (i, (id, c)) in got.iter().enumerate() {
        let comma = if i + 1 < got.len() { "," } else { "" };
        let _ = writeln!(
            s,
            "  {}: {{\"outcome\": {}, \"tier\": {}, \"time_ms\": {:.1}}}{comma}",
            quote(id),
            quote(&c.outcome),
            quote(&c.tier),
            if c.time_ms > 0.0 { c.time_ms } else { 0.0 } // no "-0.0" in the file
        );
    }
    s.push_str("}\n");
    s
}

/// Worse is larger. An answer refused is no worse than one not offered.
fn rank(outcome: &str) -> u8 {
    match outcome {
        "pass" => 0,
        "unsupported" | "refused" => 1,
        "fail" => 2,
        "timeout" => 3,
        _ => 4, // crash, or a word this runner does not know
    }
}

/// Prints every regression; true when there is none to fail on.
fn ratchet(seed: u64, base: &BTreeMap<String, Case>, got: &BTreeMap<String, Case>, part: bool) -> bool {
    let mut fails = Vec::new();
    let mut warns = Vec::new();
    for (id, b) in base {
        let Some(g) = got.get(id) else {
            if !part {
                fails.push(format!("{id}: gone from the run (was {})", b.outcome));
            }
            continue;
        };
        if rank(&g.outcome) > rank(&b.outcome) {
            fails.push(format!("{id}: {} → {}", b.outcome, g.outcome));
        } else if g.outcome == "pass"
            && b.outcome == "pass"
            && let (Ok(bt), Ok(gt)) = (b.tier.parse::<f64>(), g.tier.parse::<f64>())
            && gt > bt
        {
            fails.push(format!("{id}: tier {} → {}", b.tier, g.tier));
        }
        if b.time_ms >= TIME_FLOOR_MS && g.time_ms > 2.0 * b.time_ms {
            fails.push(format!("{id}: {:.1} ms → {:.1} ms (> 2×)", b.time_ms, g.time_ms));
        } else if b.time_ms >= TIME_FLOOR_MS && g.time_ms > 1.5 * b.time_ms {
            warns.push(format!("{id}: {:.1} ms → {:.1} ms (> 1.5×)", b.time_ms, g.time_ms));
        }
    }
    let better = got.iter().filter(|(id, g)| base.get(*id).is_some_and(|b| rank(&g.outcome) < rank(&b.outcome)));
    let better = better.count();
    for w in &warns {
        println!("seed {seed}: warn {w}");
    }
    for f in &fails {
        println!("seed {seed}: REGRESSION {f}");
    }
    println!(
        "seed {seed}: ratchet {}: {} regressions, {} warnings, {better} improved{}",
        if fails.is_empty() { "ok" } else { "FAILED" },
        fails.len(),
        warns.len(),
        if better > 0 { " (--bless to record)" } else { "" }
    );
    fails.is_empty()
}

/// A string as a JSON literal.
fn quote(s: &str) -> String {
    let mut o = String::from("\"");
    for ch in s.chars() {
        match ch {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            c if (c as u32) < 0x20 => {
                let _ = write!(o, "\\u{:04x}", c as u32);
            }
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_a_report_and_round_trips_a_baseline() {
        let report = r#"{"cases": [{"id": "b-1", "outcome": "pass", "result": "1e-7", "time_ms": 3.25,
            "ops": [{"x": [1, -2.5e-3, null, true]}], "s": "é\n"}], "unix": 1}"#;
        let got = cases(&json::parse(report).unwrap()).unwrap();
        let back = cases(&json::parse(&scores(&got)).unwrap()).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(back["b-1"].outcome, "pass");
        assert_eq!(back["b-1"].tier, "1e-7");
    }

    #[test]
    fn a_worse_outcome_or_looser_tier_fails_the_ratchet() {
        let case = |o: &str, t: &str| Case { outcome: o.into(), tier: t.into(), time_ms: 1.0 };
        let base = BTreeMap::from([("a".to_string(), case("pass", "1e-7"))]);
        assert!(ratchet(0, &base, &base, false));
        assert!(!ratchet(0, &base, &BTreeMap::from([("a".to_string(), case("crash", ""))]), false));
        assert!(!ratchet(0, &base, &BTreeMap::from([("a".to_string(), case("pass", "1e-6"))]), false));
        let unsupported = BTreeMap::from([("a".to_string(), case("unsupported", "unsupported"))]);
        assert!(ratchet(0, &unsupported, &BTreeMap::from([("a".to_string(), case("refused", ""))]), false));
    }
}
