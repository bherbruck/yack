// yack: `cargo xtask corpus smlib`: the SMLib corpus (PLAN §6.7, M6), fetched, run and ratcheted.
//! ```text
//! cargo xtask corpus smlib --fetch            check the models against corpus/smlib.sha256, write each
//!                                             as STEP and resolve SMLib's picks (yack-corpus-fetch)
//!                                             into bin/corpus/smlib/
//! cargo xtask corpus smlib [--case PREFIX] [--jobs N] [--timeout S] [--bin DIR] [--bless]
//!                                             run each case (yack-corpus, one process per case),
//!                                             grade it against SMLib's recorded result and ratchet
//!                                             scores/corpus/smlib.json
//! ```
//! The models are NVIDIA usd-brep test files (Apache-2.0), read from the pinned cadrs clone in
//! ../yack-ref and never committed. Grading compares yack's measures with SMLib's recorded ones
//! (corpus/smlib-cases.json), volume and area to a relative 1e-4, face and edge counts only when
//! every input face is planar; nothing expected is computed here. A long run belongs outside the
//! swarm builder: build there (`--fetch` builds both tools), then run `bin/debug/xtask corpus smlib
//! --bin <copied binaries>` in a shell of your own.
use crate::json::{self, Json};
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, Stdio};

const CASES: &str = "corpus/smlib-cases.json";
const MANIFEST: &str = "corpus/smlib.sha256";
const DATA: &str = "../yack-ref/cadrs/crates/cadrs_kernel/tests/data/smlib";
const OUT: &str = "bin/corpus/smlib";
const SCORES: &str = "scores/corpus/smlib.json";
const REL_TOL: f64 = 1e-4; // tol: SMLib's recorded measures, as cadrs's smlib_common compares them

pub fn run(args: &[String]) -> ExitCode {
    let r = match args.first().map(String::as_str) {
        Some("smlib") if args.iter().any(|a| a == "--fetch") => fetch(),
        Some("smlib") => grade(args),
        _ => Err("usage: cargo xtask corpus smlib [--fetch] [--case P] [--jobs N] [--timeout S] [--bin DIR] [--bless]".into()),
    };
    match r {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("error: xtask corpus: {e}");
            ExitCode::FAILURE
        }
    }
}

fn num(j: Option<&Json>) -> Option<f64> {
    match j {
        Some(Json::Num(s)) => s.parse().ok(),
        _ => None,
    }
}

fn flag(j: Option<&Json>) -> Option<bool> {
    match j {
        Some(Json::Bool(b)) => Some(*b),
        _ => None,
    }
}

fn cases(root: &Path) -> Result<Vec<Json>, String> {
    let text = std::fs::read_to_string(root.join(CASES)).map_err(|e| format!("{CASES}: {e}"))?;
    Ok(json::parse(&text)?.get("cases").map(Json::as_arr).unwrap_or_default().to_vec())
}

fn build(root: &Path, pkg: &str) -> Result<PathBuf, String> {
    let ok = Command::new("cargo").args(["build", "--release", "-p", pkg]).current_dir(root).status().map_err(|e| e.to_string())?;
    if ok.success() { Ok(root.join("bin/release").join(pkg)) } else { Err(format!("building {pkg} failed")) }
}

/// `--fetch`: verify, convert, resolve, and write bin/corpus/smlib/cases.tsv.
fn fetch() -> Result<bool, String> {
    let root = crate::meta::root();
    let tool = build(&root, "yack-corpus-fetch")?;
    build(&root, "yack-corpus")?;
    let data = root.join(DATA);
    let out = root.join(OUT);
    let v = Command::new(&tool).arg("verify").arg(&data).arg(root.join(MANIFEST)).output().map_err(|e| e.to_string())?;
    print!("{}", String::from_utf8_lossy(&v.stdout));
    if !v.status.success() {
        return Err("the models differ from corpus/smlib.sha256".into());
    }
    std::fs::create_dir_all(&out).map_err(|e| e.to_string())?;
    // each folder's README: the models' source and licence (their NOTICE), kept beside them
    for e in std::fs::read_dir(&data).map_err(|e| e.to_string())?.filter_map(Result::ok) {
        let readme = e.path().join("README.md");
        if readme.is_file() {
            let dest = out.join(e.file_name());
            std::fs::create_dir_all(&dest).map_err(|e| e.to_string())?;
            std::fs::copy(&readme, dest.join("README.md")).map_err(|e| e.to_string())?;
        }
    }
    let mut tsv = String::new();
    let mut planar_of: BTreeMap<String, bool> = BTreeMap::new();
    let (mut converted, mut refused) = (0usize, 0usize);
    for c in cases(&root)? {
        let id = c.get("id").and_then(Json::as_str).unwrap_or_default().to_string();
        let op = c.get("op").and_then(Json::as_str).unwrap_or_default();
        let inputs: Vec<String> = c.get("inputs").map(Json::as_arr).unwrap_or_default().iter().filter_map(|j| j.as_str().map(str::to_string)).collect();
        let mut steps = Vec::new();
        let (mut edge_pts, mut face_pts, mut why) = (Vec::new(), Vec::new(), None);
        for (k, input) in inputs.iter().enumerate() {
            let step = out.join(format!("{}.step", input.replace('#', "-")));
            if let Some(p) = step.parent() {
                std::fs::create_dir_all(p).map_err(|e| e.to_string())?;
            }
            let mut cmd = Command::new(&tool);
            cmd.arg("convert").arg(model_arg(&data, input)).arg(&step);
            if k == 0 && (op == "fillet" || op == "chamfer") {
                let n = count(&tool, &data, input, &out)?;
                let picks = edge_indices(&c, n);
                cmd.args(["--edges", &picks.iter().map(usize::to_string).collect::<Vec<_>>().join(",")]);
            }
            if k == 0 && op == "shell" {
                let picks: Vec<String> = c.get("faces").map(Json::as_arr).unwrap_or_default().iter().filter_map(|j| match j {
                    Json::Num(s) => Some(s.clone()),
                    _ => None,
                }).collect();
                cmd.args(["--faces", &picks.join(",")]);
            }
            let o = cmd.output().map_err(|e| e.to_string())?;
            if !o.status.success() {
                why = Some(format!("fetch could not convert {input}: {}", String::from_utf8_lossy(&o.stderr).trim()));
                break;
            }
            for line in String::from_utf8_lossy(&o.stdout).lines() {
                let w: Vec<&str> = line.split_whitespace().collect();
                match w.as_slice() {
                    ["planar", p] => {
                        let all = planar_of.get(&id).copied().unwrap_or(true) && *p == "true";
                        planar_of.insert(id.clone(), all);
                    }
                    ["edge", _, "refused", ..] | ["face", _, "refused", ..] => why = Some(format!("a pick: {line}")),
                    ["edge", _, x, y, z] => edge_pts.push(format!("{x},{y},{z}")),
                    ["face", _, x, y, z] => face_pts.push(format!("{x},{y},{z}")),
                    _ => {}
                }
            }
            steps.push(step.to_string_lossy().into_owned());
        }
        let size = num(c.get("size")).unwrap_or(0.0);
        let sharp = flag(c.get("sharp")).unwrap_or(false);
        let planar = planar_of.get(&id).copied().unwrap_or(false);
        match why {
            Some(w) => {
                refused += 1;
                let _ = writeln!(tsv, "{id}\tfetch-refused\t{size}\t{sharp}\t\t\t\t{planar}\t{}", w.replace('\t', " "));
            }
            None => {
                converted += 1;
                let _ = writeln!(tsv, "{id}\t{op}\t{size}\t{sharp}\t{}\t{}\t{}\t{planar}\t", steps.join(";"), edge_pts.join(";"), face_pts.join(";"));
            }
        }
    }
    std::fs::write(out.join("cases.tsv"), tsv).map_err(|e| e.to_string())?;
    println!("corpus smlib: fetched {converted} cases, {refused} whose models or picks could not be prepared, into {OUT}");
    Ok(true)
}

/// The fetch tool's MODEL argument for `input` (`suite/file` or `suite/part.smp#n`).
fn model_arg(data: &Path, input: &str) -> String {
    let (file, part) = input.split_once('#').map_or((input, String::new()), |(f, b)| (f, format!("#{b}")));
    format!("{}{part}", data.join(file).display())
}

/// SMLib's edge count of `input`, from a first conversion.
fn count(tool: &Path, data: &Path, input: &str, out: &Path) -> Result<usize, String> {
    let o = Command::new(tool).arg("convert").arg(model_arg(data, input)).arg(out.join("probe.step")).output().map_err(|e| e.to_string())?;
    String::from_utf8_lossy(&o.stdout)
        .lines()
        .find_map(|l| l.strip_prefix("edges ").and_then(|n| n.trim().parse().ok()))
        .ok_or(format!("{input}: no edge count"))
}

/// The SMLib edge indices a blend names: its ranges (inclusive), or every other edge.
fn edge_indices(c: &Json, n: usize) -> Vec<usize> {
    let e = c.get("edges");
    let include = flag(e.and_then(|e| e.get("include"))).unwrap_or(true);
    let ranges: Vec<(usize, usize)> = e
        .and_then(|e| e.get("ranges"))
        .map(Json::as_arr)
        .unwrap_or_default()
        .iter()
        .filter_map(|r| match r.as_arr() {
            [a, b] => Some((num(Some(a))? as usize, num(Some(b))? as usize)),
            _ => None,
        })
        .collect();
    (0..n).filter(|i| ranges.iter().any(|&(a, b)| (a..=b).contains(i)) == include).collect()
}

struct Opts {
    prefix: String,
    jobs: usize,
    timeout: u64,
    bin: Option<PathBuf>,
    bless: bool,
}

fn opts(args: &[String]) -> Result<Opts, String> {
    let mut o = Opts { prefix: String::new(), jobs: 4, timeout: 120, bin: None, bless: false };
    let mut it = args.iter().skip(1);
    while let Some(a) = it.next() {
        let mut val = || it.next().cloned().ok_or(format!("{a} needs a value"));
        match a.as_str() {
            "--case" => o.prefix = val()?,
            "--jobs" => o.jobs = val()?.parse().map_err(|e| format!("--jobs: {e}"))?,
            "--timeout" => o.timeout = val()?.parse().map_err(|e| format!("--timeout: {e}"))?,
            "--bin" => o.bin = Some(PathBuf::from(val()?)),
            "--bless" => o.bless = true,
            other => return Err(format!("unknown flag {other}")),
        }
    }
    Ok(o)
}

/// One case's outcome: pass, wrong (a valid solid that disagrees with SMLib), invalid (a result
/// the checker rejects), refused, crash or timeout; `skipped` where SMLib's result is not a
/// manifold solid (a sheet).
fn grade_one(expect: &Json, planar: bool, line: &str) -> (String, String) {
    let w: Vec<&str> = line.split_whitespace().collect();
    match w.as_slice() {
        ["ok", v, a, f, e, valid, _ms] => {
            if *valid != "true" {
                return ("invalid".into(), "yack's checker rejects the result".into());
            }
            let rel = |got: &str, key: &str| -> Option<(f64, f64)> {
                let want = num(expect.get(key))?;
                let g: f64 = got.parse().ok()?;
                Some((g, want))
            };
            let mut bad = Vec::new();
            for (got, key) in [(*v, "volume"), (*a, "area")] {
                if let Some((g, want)) = rel(got, key) {
                    let r = (g - want).abs() / want.abs().max(1.0);
                    if r > REL_TOL {
                        bad.push(format!("{key} {g} vs SMLib's {want} ({r:.1e})"));
                    }
                }
            }
            if planar {
                for (got, key) in [(*f, "faces"), (*e, "edges")] {
                    if let Some(want) = num(expect.get(key))
                        && got.parse::<f64>().ok() != Some(want)
                    {
                        bad.push(format!("{key} {got} vs SMLib's {want}"));
                    }
                }
            }
            if bad.is_empty() { ("pass".into(), String::new()) } else { ("wrong".into(), bad.join("; ")) }
        }
        ["refused", _ms, why @ ..] => ("refused".into(), why.join(" ")),
        _ => ("crash".into(), line.to_string()),
    }
}

fn rank(o: &str) -> u8 {
    match o {
        "pass" => 4,
        "skipped" => 3,
        "refused" | "timeout" | "fetch-refused" => 2,
        "crash" | "invalid" => 1,
        _ => 0, // wrong
    }
}

fn grade(args: &[String]) -> Result<bool, String> {
    let o = opts(args)?;
    let root = crate::meta::root();
    let runner = match &o.bin {
        Some(d) => d.join("yack-corpus"),
        None => build(&root, "yack-corpus")?,
    };
    let tsv_path = root.join(OUT).join("cases.tsv");
    let tsv = std::fs::read_to_string(&tsv_path).map_err(|e| format!("{}: {e} (run --fetch first)", tsv_path.display()))?;
    let expect: BTreeMap<String, Json> = cases(&root)?
        .into_iter()
        .filter_map(|c| Some((c.get("id")?.as_str()?.to_string(), c.get("expect")?.clone())))
        .collect();
    let rows: Vec<Vec<String>> = tsv
        .lines()
        .map(|l| l.split('\t').map(str::to_string).collect::<Vec<_>>())
        .filter(|f| f.first().is_some_and(|id| id.starts_with(&o.prefix)))
        .collect();
    let mut results: BTreeMap<String, (String, String)> = BTreeMap::new();
    for chunk in rows.chunks(o.jobs.max(1)) {
        let mut running = Vec::new();
        for f in chunk {
            let id = f.first().cloned().unwrap_or_default();
            let e = expect.get(&id).cloned().unwrap_or(Json::Null);
            let planar = f.get(7).is_some_and(|p| p == "true");
            if flag(e.get("solid")) == Some(false) {
                results.insert(id, ("skipped".into(), "SMLib's result is not a manifold solid".into()));
                continue;
            }
            if f.get(1).is_some_and(|op| op == "fetch-refused") {
                results.insert(id, ("fetch-refused".into(), f.get(8).cloned().unwrap_or_default()));
                continue;
            }
            let child = Command::new("timeout")
                .arg(o.timeout.to_string())
                .arg(&runner)
                .arg(&tsv_path)
                .arg(&id)
                .stdout(Stdio::piped())
                .stderr(Stdio::null())
                .spawn()
                .map_err(|e| e.to_string())?;
            running.push((id, e, planar, child));
        }
        for (id, e, planar, child) in running {
            let out = child.wait_with_output().map_err(|e| e.to_string())?;
            let r = if out.status.code() == Some(124) {
                ("timeout".to_string(), format!("over {} s", o.timeout))
            } else {
                let text = String::from_utf8_lossy(&out.stdout);
                grade_one(&e, planar, text.lines().last().unwrap_or("no output"))
            };
            results.insert(id, r);
        }
    }
    // the ratchet: no case may do worse than scores/corpus/smlib.json records
    let scores = root.join(SCORES);
    let old: BTreeMap<String, String> = std::fs::read_to_string(&scores)
        .ok()
        .and_then(|t| json::parse(&t).ok())
        .map(|j| match j {
            Json::Obj(m) => m.into_iter().filter_map(|(k, v)| Some((k, v.get("outcome")?.as_str()?.to_string()))).collect(),
            _ => BTreeMap::new(),
        })
        .unwrap_or_default();
    let mut worse = Vec::new();
    let mut tally: BTreeMap<String, usize> = BTreeMap::new();
    for (id, (outcome, why)) in &results {
        *tally.entry(outcome.clone()).or_default() += 1;
        if let Some(was) = old.get(id)
            && rank(outcome) < rank(was)
        {
            worse.push(format!("{id}: {was} -> {outcome} {why}"));
        }
        if outcome == "wrong" || outcome == "invalid" || outcome == "crash" {
            println!("corpus smlib: {id}: {outcome}: {why}");
        }
    }
    println!("corpus smlib: {} cases: {tally:?}", results.len());
    let mut body = String::from("{\n");
    let n = results.len();
    for (k, (id, (outcome, _))) in results.iter().enumerate() {
        let _ = writeln!(body, "  \"{id}\": {{\"outcome\": \"{outcome}\"}}{}", if k + 1 < n { "," } else { "" });
    }
    body.push_str("}\n");
    std::fs::write(root.join(OUT).join("results.json"), &body).map_err(|e| e.to_string())?;
    for w in &worse {
        println!("corpus smlib: WORSE {w}");
    }
    if o.bless && o.prefix.is_empty() {
        if !worse.is_empty() {
            return Err("--bless refuses while any case is worse".into());
        }
        if let Some(p) = scores.parent() {
            std::fs::create_dir_all(p).map_err(|e| e.to_string())?;
        }
        std::fs::write(&scores, &body).map_err(|e| e.to_string())?;
        println!("corpus smlib: blessed {SCORES}");
    }
    Ok(worse.is_empty())
}
