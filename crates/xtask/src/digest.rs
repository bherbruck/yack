// yack: `cargo xtask digest`: result digests per conformance and bench case (PLAN §6.4, M1.6).
//! `cargo xtask digest [--check] [--only conformance|bench] [--seed N] [--threads N] [--ref DIR]
//!                    [--bin-dir DIR --tree FINGERPRINT]`
//!
//! Records `Solid::digest()` of every result, sorted and timestamp-free, in
//! `scores/digests/conformance-seed<N>.txt` and `scores/digests/bench.txt`. `--check` diffs a fresh
//! run against them instead: the bit-identical gate of the M2/M3 pure moves.
//!
//! Conformance: the runner starts its adapter as `xtask digest-proxy <log> <adapter>`, which
//! passes the protocol through unchanged. The proxy writes `op <cmd>` before each request and
//! `reset` at each case boundary to `<log>`. The adapter, seeing `YACK_DIGESTS=<log>`, appends
//! `digest <hex>` for each solid it returns. Each of `--threads` runners is single-threaded on its
//! share of the families, so the n-th block of its log is the n-th case of its results.
//!
//! Bench: `yack-bench --digests` prints `<case> <hex>` or `<case> refused` per case.
use crate::json::{self, Json};
use std::collections::BTreeSet;
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, Stdio};

const USAGE: &str = "usage: cargo xtask digest [--check] [--only conformance|bench] [--seed N] [--threads N] [--ref DIR]
                            [--bin-dir DIR --tree FINGERPRINT]";

/// The environment variable the adapter reads (set by the proxy on its child only).
pub const LOG_VAR: &str = "YACK_DIGESTS";

struct Opts {
    check: bool,
    conformance: bool,
    bench: bool,
    seed: u64,
    /// Parallel single-threaded runners, each on its own share of the families.
    threads: usize,
    reference: PathBuf,
    /// Where `yack-conformance` and `yack-bench` are; set → no build (swarm workers copy the
    /// binaries out of bin/release first, so a rebuild mid-run cannot mix trees).
    bin_dir: Option<PathBuf>,
    /// The tree the binaries were built from, recorded in each file's header.
    tree: Option<String>,
}

fn parse(args: &[String]) -> Result<Opts, String> {
    let root = crate::meta::root();
    let mut o = Opts { check: false, conformance: true, bench: true, seed: 0, threads: 4, reference: root.join("../yack-ref"), bin_dir: None, tree: None };
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--check" => o.check = true,
            "--bin-dir" => o.bin_dir = Some(PathBuf::from(it.next().ok_or(USAGE)?)),
            "--tree" => o.tree = Some(it.next().ok_or(USAGE)?.clone()),
            "--only" => match it.next().map(String::as_str) {
                Some("conformance") => o.bench = false,
                Some("bench") => o.conformance = false,
                _ => return Err(USAGE.into()),
            },
            "--seed" => o.seed = it.next().and_then(|s| s.parse().ok()).ok_or(USAGE)?,
            "--threads" => o.threads = it.next().and_then(|s| s.parse().ok()).filter(|&n| n > 0).ok_or(USAGE)?,
            "--ref" => o.reference = PathBuf::from(it.next().ok_or(USAGE)?),
            _ => return Err(format!("unknown argument `{a}`\n{USAGE}")),
        }
    }
    if o.bin_dir.is_some() != o.tree.is_some() {
        return Err(format!("--bin-dir and --tree go together\n{USAGE}"));
    }
    Ok(o)
}

/// The binary `name`: in `--bin-dir` when given, else freshly built into bin/release.
fn binary(root: &Path, o: &Opts, name: &str) -> Result<PathBuf, String> {
    match &o.bin_dir {
        Some(d) => {
            let p = d.join(name);
            if p.is_file() { Ok(p) } else { Err(format!("no {} in --bin-dir", p.display())) }
        }
        None => {
            build(root, name)?;
            Ok(root.join("bin/release").join(name))
        }
    }
}

/// `cargo xtask digest ...`.
pub fn run(args: &[String]) -> ExitCode {
    match parse(args).and_then(|o| drive(&o)) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("error: xtask digest: {e}");
            ExitCode::FAILURE
        }
    }
}

fn build(root: &Path, pkg: &str) -> Result<(), String> {
    let status = Command::new("cargo")
        .args(["build", "--release", "-p", pkg])
        .current_dir(root)
        .status()
        .map_err(|e| format!("cargo build: {e}"))?;
    if status.success() { Ok(()) } else { Err(format!("building {pkg} failed")) }
}

fn drive(o: &Opts) -> Result<bool, String> {
    let root = crate::meta::root();
    let dir = root.join("scores/digests");
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut ok = true;
    if o.conformance {
        let lines = conformance(&root, o)?;
        ok &= settle(&dir.join(format!("conformance-seed{}.txt", o.seed)), &lines, o.check, o.tree.as_deref())?;
    }
    if o.bench {
        let lines = bench(&binary(&root, o, "yack-bench")?)?;
        ok &= settle(&dir.join("bench.txt"), &lines, o.check, o.tree.as_deref())?;
    }
    Ok(ok)
}

/// Writes `lines` to `path`, or with `check` diffs them against it. True when nothing differs.
fn settle(path: &Path, lines: &BTreeSet<String>, check: bool, tree: Option<&str>) -> Result<bool, String> {
    let name = path.display();
    if !check {
        let head = format!("# Solid::digest() per result (cargo xtask digest); tree {}\n", tree.unwrap_or("unrecorded"));
        let body: String = head + &lines.iter().map(|l| format!("{l}\n")).collect::<String>();
        std::fs::write(path, body).map_err(|e| format!("{name}: {e}"))?;
        println!("digest: wrote {name} ({} lines)", lines.len());
        return Ok(true);
    }
    let old: BTreeSet<String> = std::fs::read_to_string(path)
        .map_err(|e| format!("{name}: {e} (record it first with `cargo xtask digest`)"))?
        .lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(str::to_string)
        .collect();
    let gone: Vec<_> = old.difference(lines).collect();
    let new: Vec<_> = lines.difference(&old).collect();
    for l in gone.iter().take(50) {
        println!("- {l}");
    }
    for l in new.iter().take(50) {
        println!("+ {l}");
    }
    if gone.is_empty() && new.is_empty() {
        println!("digest: {name}: identical ({} lines)", lines.len());
        Ok(true)
    } else {
        println!("error: digest: {name}: {} lines gone, {} new", gone.len(), new.len());
        Ok(false)
    }
}

fn bench(exe: &Path) -> Result<BTreeSet<String>, String> {
    let out = Command::new(exe).arg("--digests").output().map_err(|e| format!("{}: {e}", exe.display()))?;
    if !out.status.success() {
        return Err(format!("{} --digests failed: {}", exe.display(), String::from_utf8_lossy(&out.stderr)));
    }
    let lines: BTreeSet<String> =
        String::from_utf8_lossy(&out.stdout).lines().map(str::trim).filter(|l| !l.is_empty()).map(str::to_string).collect();
    if lines.is_empty() {
        return Err("yack-bench --digests printed nothing".into());
    }
    Ok(lines)
}

/// The family of a case id: everything before its last `-` (`booleans-0b66..` → `booleans`).
fn family(id: &str) -> &str {
    id.rsplit_once('-').map_or(id, |(f, _)| f)
}

/// Families with their case counts, from the conformance baseline of `seed`.
fn families(root: &Path, seed: u64) -> Result<Vec<(String, usize)>, String> {
    let p = root.join(format!("scores/conformance/seed{seed}.json"));
    let text = std::fs::read_to_string(&p)
        .map_err(|e| format!("{}: {e} (run `cargo xtask conformance --seed {seed} --bless` first)", p.display()))?;
    let mut counts: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    if let Json::Obj(members) = json::parse(&text)? {
        for (id, _) in members {
            *counts.entry(family(&id).to_string()).or_insert(0) += 1;
        }
    }
    Ok(counts.into_iter().collect())
}

/// Splits families into `n` groups of about equal case counts (largest first, to the lightest).
fn groups(fams: &[(String, usize)], n: usize) -> Vec<(Vec<String>, usize)> {
    let mut sorted: Vec<&(String, usize)> = fams.iter().collect();
    sorted.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    let mut out: Vec<(Vec<String>, usize)> = vec![(Vec::new(), 0); n.max(1)];
    for (f, c) in sorted {
        if let Some(g) = out.iter_mut().min_by_key(|g| g.1) {
            g.0.push(f.clone());
            g.1 += c;
        }
    }
    out.retain(|g| !g.0.is_empty());
    out
}

fn conformance(root: &Path, o: &Opts) -> Result<BTreeSet<String>, String> {
    let runner = o.reference.join("bin/release/brep-conformance");
    if !runner.is_file() {
        return Err(format!("no runner at {}", runner.display()));
    }
    let adapter = binary(root, o, "yack-conformance")?;
    let work = root.join("bin/digest");
    std::fs::create_dir_all(&work).map_err(|e| format!("{}: {e}", work.display()))?;
    let me = std::env::current_exe().map_err(|e| format!("current_exe: {e}"))?;
    let gs = groups(&families(root, o.seed)?, o.threads);
    let total: usize = gs.iter().map(|g| g.1).sum();
    // one single-threaded runner per group, so each log's blocks follow its results' case order
    let mut runs = Vec::new();
    for (i, (fams, n)) in gs.iter().enumerate() {
        let log = work.join(format!("conformance-seed{}-{i}.log", o.seed));
        let out = work.join(format!("seed{}-{i}.json", o.seed));
        std::fs::write(&log, "").map_err(|e| format!("{}: {e}", log.display()))?;
        // the runner splits this on whitespace and starts it once per thread
        let command = format!("{} digest-proxy {} {}", me.display(), log.display(), adapter.display());
        let child = Command::new(&runner)
            .arg("--adapter")
            .arg(&command)
            .args(["--seed", &o.seed.to_string(), "--threads", "1", "--family", &fams.join(","), "--out"])
            .arg(&out)
            .stdout(Stdio::null())
            .spawn()
            .map_err(|e| format!("{}: {e}", runner.display()))?;
        println!("digest: runner {i}: {n} cases in {}", fams.join(","));
        runs.push((child, log, out));
    }
    #[allow(clippy::disallowed_methods)] // progress display only (xtask is a tool, not kernel code)
    let started = std::time::Instant::now();
    loop {
        let mut running = 0;
        for (c, _, _) in runs.iter_mut() {
            if c.try_wait().map_err(|e| format!("runner: {e}"))?.is_none() {
                running += 1;
            }
        }
        let done: usize = runs
            .iter()
            .map(|(_, log, _)| std::fs::read_to_string(log).map(|t| t.lines().filter(|l| *l == "reset").count()).unwrap_or(0))
            .sum();
        println!("digest: {done}/{total} cases, {running} runners left, {}s", started.elapsed().as_secs());
        if running == 0 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_secs(15));
    }
    let mut lines = BTreeSet::new();
    for (_, log, out) in &runs {
        let results = json::parse(&std::fs::read_to_string(out).map_err(|e| format!("{}: {e}", out.display()))?)?;
        let ids: Vec<String> = results
            .get("cases")
            .map(Json::as_arr)
            .unwrap_or_default()
            .iter()
            .map(|c| c.get("id").and_then(Json::as_str).map(str::to_string).ok_or("a case without an id"))
            .collect::<Result<_, _>>()?;
        let text = std::fs::read_to_string(log).map_err(|e| format!("{}: {e}", log.display()))?;
        lines.extend(assemble(&ids, &text).map_err(|e| format!("{}: {e}", log.display()))?);
    }
    Ok(lines)
}

/// Pairs the log's blocks (split at `reset`) with the case ids, in order:
/// `<case> <k> <op> <hex>` for the k-th solid the case's calls returned.
pub fn assemble(ids: &[String], log: &str) -> Result<BTreeSet<String>, String> {
    let mut blocks: Vec<Vec<(String, String)>> = vec![Vec::new()];
    let mut op = String::from("?");
    for line in log.lines() {
        if line == "reset" {
            blocks.push(Vec::new());
        } else if let Some(c) = line.strip_prefix("op ") {
            op = c.to_string();
        } else if let Some(h) = line.strip_prefix("digest ")
            && let Some(b) = blocks.last_mut()
        {
            b.push((op.clone(), h.to_string()));
        }
    }
    // the last reset closes the last case; what follows it (the runner's closing probes of what
    // the kernel lacks) belongs to no case
    if blocks.len() == ids.len() + 1 {
        blocks.pop();
    }
    if blocks.len() != ids.len() {
        return Err(format!(
            "the log has {} case blocks but the results list {} cases (was the run single-threaded and complete?)",
            blocks.len(),
            ids.len()
        ));
    }
    let mut out = BTreeSet::new();
    for (id, b) in ids.iter().zip(blocks) {
        for (k, (op, h)) in b.into_iter().enumerate() {
            out.insert(format!("{id} {k:03} {op} {h}"));
        }
    }
    Ok(out)
}

/// `xtask digest-proxy <log> <adapter> [args..]`: relays the protocol between the runner and the
/// adapter, one request line then one reply line, marking requests and case boundaries in `<log>`.
pub fn proxy(args: &[String]) -> ExitCode {
    match relay(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: xtask digest-proxy: {e}");
            ExitCode::FAILURE
        }
    }
}

fn relay(args: &[String]) -> Result<(), String> {
    let (log_path, rest) = args.split_first().ok_or("usage: xtask digest-proxy <log> <adapter> [args..]")?;
    let (adapter, adapter_args) = rest.split_first().ok_or("no adapter given")?;
    let mut log = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open(log_path)
        .map_err(|e| format!("{log_path}: {e}"))?;
    let mut child = Command::new(adapter)
        .args(adapter_args)
        .env(LOG_VAR, log_path)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .map_err(|e| format!("{adapter}: {e}"))?;
    let mut to_child = child.stdin.take().ok_or("no adapter stdin")?;
    let mut from_child = BufReader::new(child.stdout.take().ok_or("no adapter stdout")?);
    let stdin = std::io::stdin();
    let mut out = std::io::stdout().lock();
    let mut reply = String::new();
    for line in stdin.lock().lines() {
        let line = line.map_err(|e| format!("stdin: {e}"))?;
        let cmd = json::parse(&line).ok().and_then(|v| v.get("cmd").and_then(Json::as_str).map(str::to_string));
        match cmd.as_deref() {
            Some("reset") => writeln!(log, "reset"),
            Some(c) => writeln!(log, "op {c}"),
            None => Ok(()),
        }
        .map_err(|e| format!("{log_path}: {e}"))?;
        writeln!(to_child, "{line}").and_then(|()| to_child.flush()).map_err(|e| format!("adapter stdin: {e}"))?;
        reply.clear();
        if from_child.read_line(&mut reply).map_err(|e| format!("adapter stdout: {e}"))? == 0 {
            return Err("the adapter closed its stdout".into());
        }
        out.write_all(reply.as_bytes()).and_then(|()| out.flush()).map_err(|e| format!("stdout: {e}"))?;
    }
    drop(to_child);
    let _ = child.wait();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn assembles_blocks_per_case() {
        let log = "op hello\nop box\ndigest aa\nop union\ndigest bb\nop volume\nreset\nop box\nreset\nop sphere\ndigest cc\nstrategy x\nreset\nop box\ndigest dd\n";
        let ids: Vec<String> = ["b-2", "a-1", "c-3"].iter().map(|s| s.to_string()).collect();
        let got: Vec<String> = assemble(&ids, log).unwrap().into_iter().collect();
        assert_eq!(got, ["b-2 000 box aa", "b-2 001 union bb", "c-3 000 sphere cc"]);
        assert!(assemble(&ids[..2], log).is_err());
    }

    #[test]
    fn groups_balance_families() {
        let fams: Vec<(String, usize)> = [("a", 10), ("b", 6), ("c", 5), ("d", 1)].iter().map(|(f, n)| (f.to_string(), *n)).collect();
        let g = groups(&fams, 2);
        assert_eq!(g.iter().map(|x| x.1).collect::<Vec<_>>(), [11, 11]);
        assert_eq!(family("booleans-0b66"), "booleans");
        assert_eq!(groups(&fams, 9).len(), 4);
    }
}
