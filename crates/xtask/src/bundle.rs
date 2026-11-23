// yack: `cargo xtask bundle`: folds `yack` and its internal dependency closure into one
// publishable crate in bin/bundle/yack (M6; design in swarm/done/w-m03-bundle-design.md).
//
// Each internal crate `yack-<x>` becomes the private module `crate::internal::<x>`. A text-preserving
// scan rewrites only code (never comments or literals): a path starting `yack_<x>` becomes
// `crate::internal::<x>`, and inside a folded crate a path starting `crate::` (or `$crate::`) gains
// `internal::<x>::`. Feature cfgs are resolved to `all()`/`any()` as the workspace build enables
// them (`bakeoff` is cut). Test-only files are left out, internal doc examples are fenced as text,
// and springrep's build-script hash is computed here, so the bundle has no build script and no
// dependencies. `--check` builds, lints, packages and API-diffs the bundle, and compares every
// brep-bench digest of a runner built on it with the workspace runner's.
use crate::gate::lex;
use crate::meta::{self, Package, Tree};
use crate::toml::{self, Toml};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

const OUT: &str = "bin/bundle/yack";
const CHECK: &str = "bin/bundle/check/yack-bench-bundle";
const STEPS: [&str; 8] = ["deterministic", "gate", "build", "clippy", "api", "package", "bench-build", "digests"];

/// One crate of the closure: its package, its module name in the bundle (`None` for `yack`), and
/// the features the workspace build turns on in it.
struct Folded {
    pkg: Package,
    module: Option<String>,
    features: BTreeMap<String, bool>,
    /// The names its code calls its dependencies by (a dependency key, which a `package =` rename
    /// sets) → their modules in the bundle.
    names: BTreeMap<String, String>,
}

/// `cargo xtask bundle [--check [STEPS]]`: STEPS a comma list of deterministic, gate, build, clippy,
/// api, package, bench-build, digests (default all but digests).
///
/// `bench-build` builds the workspace runner and one on the bundle and copies both to
/// `../yack-ref/bin/bundle-<fingerprint>/`; the digest comparison (minutes) then runs outside the
/// builder on those copies (`digests` runs it in place, for a host with no builder).
pub fn run(args: &[String]) -> ExitCode {
    let check = args.iter().position(|a| a == "--check").map(|k| {
        args.get(k + 1)
            .filter(|s| !s.starts_with("--"))
            .map(|s| s.split(',').map(str::to_string).collect::<BTreeSet<String>>())
            .unwrap_or_else(|| STEPS.iter().filter(|s| **s != "digests").map(|s| s.to_string()).collect())
    });
    match drive(check) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("error: xtask bundle: {e}");
            ExitCode::FAILURE
        }
    }
}

fn drive(check: Option<BTreeSet<String>>) -> Result<bool, String> {
    let root = meta::root();
    let tree = meta::load(&root)?;
    let out = root.join(OUT);
    let report = make(&root, &tree, &out)?;
    println!("bundle: {} files from {} crates -> {OUT} ({} lines)", report.files, report.crates, report.lines);
    println!("bundle: license = {}", report.spdx);
    let Some(steps) = check else { return Ok(true) };
    if let Some(bad) = steps.iter().find(|s| !STEPS.contains(&s.as_str())) {
        return Err(format!("no check step `{bad}` (steps: {})", STEPS.join(",")));
    }
    let mut ok = true;
    let manifest = out.join("Cargo.toml");
    for name in STEPS.iter().filter(|s| steps.contains(**s)) {
        let r = match *name {
            "deterministic" => deterministic(&root, &tree, &out),
            "gate" => bundle_gate(&out, &report),
            "build" => cargo(&root, &["build", "--release", "--manifest-path"], &manifest, &[]).map(|()| String::new()),
            "clippy" => cargo(&root, &["clippy", "--release", "--manifest-path"], &manifest, &["--", "-D", "warnings"]).map(|()| String::new()),
            "api" => api(&tree, &out),
            "package" => package(&root, &manifest),
            "bench-build" => bench_build(&root, &tree).map(|d| format!("copied to {}", d.display())),
            _ => digests(&root, &tree),
        };
        match r {
            Ok(s) => println!("bundle check: {name}: ok{}", if s.is_empty() { String::new() } else { format!(" ({s})") }),
            Err(e) => {
                println!("bundle check: {name}: FAILED: {e}");
                ok = false;
            }
        }
    }
    println!("bundle check: {}", if ok { "all passed" } else { "FAILED" });
    Ok(ok)
}

struct Report {
    files: usize,
    crates: usize,
    lines: usize,
    spdx: String,
    /// `// Adapted from` headers in the source files copied, and in the bundle.
    headers_in: usize,
    headers_out: usize,
}

// ---- the closure ----

/// `yack` and every internal crate it depends on (normal and build), dependencies first.
fn closure(tree: &Tree) -> Result<Vec<Folded>, String> {
    let mut order: Vec<String> = Vec::new();
    let mut seen = BTreeSet::new();
    fn visit(tree: &Tree, name: &str, seen: &mut BTreeSet<String>, order: &mut Vec<String>) -> Result<(), String> {
        if !seen.insert(name.to_string()) {
            return Ok(());
        }
        let p = tree.package(name).ok_or(format!("no package {name}"))?;
        for d in p.deps.iter().filter(|d| d.kind != meta::DepKind::Dev) {
            if !d.internal {
                return Err(format!("{name} depends on the external crate {}", d.name));
            }
            visit(tree, &d.name, seen, order)?;
        }
        order.push(name.to_string());
        Ok(())
    }
    visit(tree, "yack", &mut seen, &mut order)?;
    let manifests: BTreeMap<String, Toml> = order
        .iter()
        .filter_map(|n| tree.package(n))
        .map(|p| {
            let path = format!("{}/Cargo.toml", p.dir);
            let t = tree.file(&path).ok_or(format!("{path} unreadable")).and_then(toml::parse)?;
            Ok((p.name.clone(), t))
        })
        .collect::<Result<_, String>>()?;
    let mut out = Vec::new();
    for name in &order {
        let p = tree.package(name).ok_or(format!("no package {name}"))?.clone();
        let module = (name != "yack").then(|| name.trim_start_matches("yack-").replace('-', "_"));
        let declared: Vec<String> = manifests
            .get(name)
            .and_then(|m| m.get("features"))
            .and_then(Toml::as_table)
            .map(|t| t.keys().filter(|k| *k != "default").cloned().collect())
            .unwrap_or_default();
        let on = enabled(name, &manifests);
        let features = declared.into_iter().map(|f| (f.clone(), on.contains(&f))).collect();
        let mut names = BTreeMap::new();
        for (key, spec) in manifests.get(name).and_then(|m| m.get("dependencies")).and_then(Toml::as_table).into_iter().flatten() {
            let package = spec.get("package").and_then(Toml::as_str).unwrap_or(key);
            if package != "yack" {
                names.insert(key.replace('-', "_"), package.trim_start_matches("yack-").replace('-', "_"));
            }
        }
        out.push(Folded { pkg: p, module, features, names });
    }
    Ok(out)
}

/// The features of `name` that the workspace build of `yack` enables: its defaults (unless every
/// dependent turns them off) and those its dependents ask for, expanded through its own feature
/// table. `yack`'s own features are all off (`bakeoff` is cut from the bundle).
fn enabled(name: &str, manifests: &BTreeMap<String, Toml>) -> BTreeSet<String> {
    if name == "yack" {
        return BTreeSet::new();
    }
    let mut want: BTreeSet<String> = BTreeSet::new();
    let mut defaults = false;
    for m in manifests.values() {
        let Some(spec) = m.get("dependencies").and_then(|d| d.get(name)) else { continue };
        defaults |= spec.get("default-features").and_then(Toml::as_bool) != Some(false);
        want.extend(spec.get("features").map(Toml::strings).unwrap_or_default());
    }
    let table = manifests.get(name).and_then(|m| m.get("features"));
    if defaults {
        want.extend(table.and_then(|t| t.get("default")).map(Toml::strings).unwrap_or_default());
    }
    let mut on = BTreeSet::new();
    while let Some(f) = want.pop_first() {
        if on.insert(f.clone()) {
            want.extend(table.and_then(|t| t.get(&f)).map(Toml::strings).unwrap_or_default().into_iter().filter(|x| !x.contains('/')));
        }
    }
    on
}

/// The SPDX expression of the closure: the gate's own (gate 10), so the two never disagree.
fn spdx(tree: &Tree) -> Result<String, String> {
    Ok(crate::gate::closure_spdx(&crate::gate::Ctx::new(tree.clone())?))
}

// ---- making the bundle ----

fn make(root: &Path, tree: &Tree, out: &Path) -> Result<Report, String> {
    let folded = closure(tree)?;
    let mut written: BTreeMap<String, String> = BTreeMap::new();
    let mut map: Vec<(String, String)> = Vec::new();
    let mut headers_in = 0;
    for f in &folded {
        let dir = &f.pkg.dir;
        let srcs = lex::crate_sources(dir, &tree.files);
        let src_dir = format!("{dir}/src/");
        // `extern crate <dep> as <alias>;` at the root: the alias names the same module, and the
        // line goes (a module cannot be an extern crate)
        let mut names = f.names.clone();
        let mut dropped: BTreeSet<String> = BTreeSet::new();
        for line in tree.file(&format!("{src_dir}lib.rs")).unwrap_or_default().lines() {
            let words: Vec<&str> = line.trim().trim_end_matches(';').split_whitespace().collect();
            if let ["extern", "crate", dep, "as", alias] = words.as_slice()
                && let Some(m) = names.get(*dep).cloned()
            {
                names.insert((*alias).to_string(), m);
                dropped.insert(line.to_string());
            }
        }
        let dest = match &f.module {
            None => "src/".to_string(),
            Some(m) => format!("src/internal/{m}/"),
        };
        for s in srcs.iter().filter(|s| !s.test_file && s.path.starts_with(&src_dir)) {
            let rel = s.path.strip_prefix(&src_dir).unwrap_or(&s.path);
            let text = tree.file(&s.path).ok_or(format!("{} unreadable", s.path))?;
            headers_in += text.lines().filter(|l| l.starts_with("// Adapted from")).count();
            let target = match (&f.module, rel) {
                (Some(_), "lib.rs") => format!("{dest}mod.rs"),
                _ => format!("{dest}{rel}"),
            };
            let kept: String = if rel == "lib.rs" && !dropped.is_empty() {
                text.split_inclusive('\n').filter(|l| !dropped.contains(l.trim_end_matches('\n'))).collect()
            } else {
                text.to_string()
            };
            let mut body = rewrite(&kept, &names, f.module.as_deref());
            body = features(&body, &f.features);
            if let Some(m) = &f.module {
                body = fence_docs(&body);
                body = folded_root_fixes(&body, rel, m, root, dir)?;
            } else if rel == "lib.rs" {
                body.push_str("\n// The internal crates, folded in by `cargo xtask bundle` (not part of the API).\nmod internal;\n");
            }
            check_env(&body, &target)?;
            map.push((target.clone(), s.path.clone()));
            written.insert(target, body);
        }
        if let Some(m) = &f.module
            && tree.file(&format!("{dir}/README.md")).is_some()
            && written.get(&format!("src/internal/{m}/mod.rs")).is_some_and(|t| t.contains("include_str!(\"README.md\")"))
        {
            let readme = tree.file(&format!("{dir}/README.md")).unwrap_or_default();
            written.insert(format!("src/internal/{m}/README.md"), fence_readme(readme));
        }
    }
    written.insert("src/internal/mod.rs".to_string(), internal_mod(&folded));
    let spdx = spdx(tree)?;
    written.insert("Cargo.toml".to_string(), manifest(tree, &spdx)?);
    let readme = tree.file("README.md").ok_or("README.md is missing")?;
    written.insert("README.md".to_string(), readme.to_string());
    for l in ["LICENSE-MIT", "LICENSE-APACHE"] {
        let text = std::fs::read_to_string(root.join(l)).map_err(|e| format!("{l}: {e}"))?;
        written.insert(l.to_string(), text);
    }
    let mut licenses: Vec<(String, String)> = Vec::new();
    for f in &folded {
        for p in tree.paths.iter().filter(|p| p.starts_with(&format!("{}/LICENSE-", f.pkg.dir))) {
            let name = p.rsplit('/').next().unwrap_or(p).to_string();
            let text = std::fs::read_to_string(root.join(p)).map_err(|e| format!("{p}: {e}"))?;
            written.insert(format!("licenses/{name}"), text);
            licenses.push((p.clone(), format!("licenses/{name}")));
        }
    }
    written.insert("NOTICE".to_string(), notice(tree, &folded, &licenses)?);
    written.insert("BUNDLE.md".to_string(), bundle_md(root, &map));
    // only changed files are written (an unchanged bundle rebuilds nothing); stale ones go
    let old = if out.exists() { read_tree(out)? } else { BTreeMap::new() };
    for rel in old.keys().filter(|k| !written.contains_key(*k)) {
        std::fs::remove_file(out.join(rel)).map_err(|e| format!("{rel}: {e}"))?;
    }
    let mut lines = 0;
    for (rel, text) in &written {
        if rel.ends_with(".rs") {
            lines += text.lines().count();
        }
        if old.get(rel).is_some_and(|b| b.as_slice() == text.as_bytes()) {
            continue;
        }
        let p = out.join(rel);
        if let Some(parent) = p.parent() {
            std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
        }
        std::fs::write(&p, text).map_err(|e| format!("{}: {e}", p.display()))?;
    }
    let headers_out =
        written.iter().filter(|(p, _)| p.ends_with(".rs")).map(|(_, t)| t.lines().filter(|l| l.starts_with("// Adapted from")).count()).sum();
    Ok(Report { files: written.len(), crates: folded.len(), lines, spdx, headers_in, headers_out })
}

/// The rewrite of one source file. `crates`: `yack_<x>` → `<x>`; `own`: the folded crate's module.
fn rewrite(text: &str, crates: &BTreeMap<String, String>, own: Option<&str>) -> String {
    let c: Vec<char> = text.chars().collect();
    let at = |i: usize| c.get(i).copied().unwrap_or('\0');
    let mut out = String::with_capacity(text.len() + 1024);
    let mut i = 0;
    // the last significant (non-space) text emitted, to tell a path's first segment
    let mut prev = String::new();
    while i < c.len() {
        let ch = at(i);
        // comments
        if ch == '/' && at(i + 1) == '/' {
            let end = (i..c.len()).find(|&j| at(j) == '\n').unwrap_or(c.len());
            out.extend(c.get(i..end).unwrap_or_default());
            i = end;
            continue;
        }
        if ch == '/' && at(i + 1) == '*' {
            let mut depth = 0usize;
            let mut j = i;
            while j < c.len() {
                if at(j) == '/' && at(j + 1) == '*' {
                    depth += 1;
                    j += 2;
                } else if at(j) == '*' && at(j + 1) == '/' {
                    depth = depth.saturating_sub(1);
                    j += 2;
                    if depth == 0 {
                        break;
                    }
                } else {
                    j += 1;
                }
            }
            out.extend(c.get(i..j.min(c.len())).unwrap_or_default());
            i = j;
            continue;
        }
        // strings
        if ch == '"' {
            let end = string_end(&c, i + 1);
            out.extend(c.get(i..end).unwrap_or_default());
            i = end;
            prev = "\"".into();
            continue;
        }
        if ch == '\'' {
            // a char literal ('x', '\n', '\'', '\\', '\u{..}') or a lifetime ('a)
            let end = char_end(&c, i).unwrap_or(i + 1);
            out.extend(c.get(i..end).unwrap_or_default());
            i = end;
            prev = "'".into();
            continue;
        }
        if ch.is_ascii_alphabetic() || ch == '_' {
            let end = (i..c.len()).find(|&j| !(at(j).is_ascii_alphanumeric() || at(j) == '_')).unwrap_or(c.len());
            let word: String = c.get(i..end).unwrap_or_default().iter().collect();
            // raw and byte strings: r"..", r#".."#, b"..", br"..", c".."
            if matches!(word.as_str(), "r" | "br" | "cr") && (at(end) == '"' || at(end) == '#') {
                let hashes = (end..c.len()).take_while(|&j| at(j) == '#').count();
                if at(end + hashes) == '"' {
                    let close: String = std::iter::once('"').chain(std::iter::repeat_n('#', hashes)).collect();
                    let body_start = end + hashes + 1;
                    let rest: String = c.get(body_start..).unwrap_or_default().iter().collect();
                    let stop = rest
                        .find(&close)
                        .and_then(|k| rest.get(..k))
                        .map_or(c.len(), |before| body_start + before.chars().count() + close.chars().count());
                    out.extend(c.get(i..stop).unwrap_or_default());
                    i = stop;
                    prev = "\"".into();
                    continue;
                }
            }
            if matches!(word.as_str(), "b" | "c") && at(end) == '"' {
                let stop = string_end(&c, end + 1);
                out.extend(c.get(i..stop).unwrap_or_default());
                i = stop;
                prev = "\"".into();
                continue;
            }
            if word == "b" && at(end) == '\'' {
                let stop = char_end(&c, end).unwrap_or(end + 1);
                out.extend(c.get(i..stop).unwrap_or_default());
                i = stop;
                prev = "'".into();
                continue;
            }
            let path_start = !prev.ends_with("::") && (!prev.ends_with('.') || prev.ends_with(".."));
            let after_use = prev == "use";
            let next_is_path = {
                let k = (end..c.len()).find(|&j| !at(j).is_whitespace()).unwrap_or(c.len());
                at(k) == ':' && at(k + 1) == ':'
            };
            if path_start && (next_is_path || after_use) && let Some(m) = crates.get(&word) {
                let _ = write!(out, "crate::internal::{m}");
            } else if path_start && word == "crate" && next_is_path && let Some(own) = own {
                let _ = write!(out, "crate::internal::{own}");
            } else {
                out.push_str(&word);
            }
            prev = word;
            i = end;
            continue;
        }
        out.push(ch);
        if !ch.is_whitespace() {
            if (ch == ':' && prev.ends_with(':') && !prev.ends_with("::")) || (ch == '.' && prev == ".") {
                prev.push(ch);
            } else {
                prev = ch.to_string();
            }
        }
        i += 1;
    }
    out
}

/// The index just past a char literal whose opening quote is at `i`; `None` for a lifetime.
fn char_end(c: &[char], i: usize) -> Option<usize> {
    let at = |k: usize| c.get(k).copied().unwrap_or('\0');
    if at(i + 1) == '\\' {
        match at(i + 2) {
            'u' => (i + 3..c.len()).find(|&k| at(k) == '}').map(|k| k + 2),
            'x' => Some(i + 6),
            _ => Some(i + 4),
        }
    } else if at(i + 2) == '\'' {
        Some(i + 3)
    } else {
        None
    }
}

/// The index just past the closing quote of a string starting at `i` (inside the quotes).
fn string_end(c: &[char], mut i: usize) -> usize {
    while i < c.len() {
        match c.get(i) {
            Some('\\') => i += 2,
            Some('"') => return i + 1,
            _ => i += 1,
        }
    }
    c.len()
}

/// `feature = "f"` in a cfg: `all()` where the workspace build enables `f`, else `any()`.
fn features(text: &str, features: &BTreeMap<String, bool>) -> String {
    let mut t = text.to_string();
    for (f, on) in features {
        let to = if *on { "all()" } else { "any()" };
        t = t.replace(&format!("feature = \"{f}\""), to).replace(&format!("feature=\"{f}\""), to);
    }
    // a feature the crate does not declare can never be on (cadrs's `gpu`); nor can `yack`'s cut
    // `bakeoff`
    for pat in ["feature = \"", "feature=\""] {
        while let Some(k) = t.find(pat) {
            let rest = t.get(k + pat.len()..).unwrap_or_default();
            let Some(close) = rest.find('"') else { break };
            t.replace_range(k..k + pat.len() + close + 1, "any()");
        }
    }
    t
}

/// Doc examples of a folded crate's items, fenced as text: they name `yack_<x>` crates the bundle
/// does not have, and stay tested in the workspace.
fn fence_docs(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut open = false;
    for line in text.split_inclusive('\n') {
        let t = line.trim_start();
        let doc = t.starts_with("///") || t.starts_with("//!");
        if !doc {
            open = false;
            out.push_str(line);
            continue;
        }
        let after = t.get(3..).unwrap_or_default().trim_start();
        if after.starts_with("```") {
            if open {
                open = false;
                out.push_str(line);
            } else {
                open = true;
                let indent = line.len() - t.len();
                let prefix = t.get(..3).unwrap_or("///");
                let nl = if line.ends_with('\n') { "\n" } else { "" };
                let _ = write!(out, "{}{prefix} ```text{nl}", line.get(..indent).unwrap_or_default());
            }
            continue;
        }
        out.push_str(line);
    }
    out
}

fn fence_readme(text: &str) -> String {
    let mut out = String::new();
    let mut open = false;
    for line in text.split_inclusive('\n') {
        if line.trim_start().starts_with("```") {
            if open {
                out.push_str(line);
            } else {
                out.push_str("```text\n");
            }
            open = !open;
        } else {
            out.push_str(line);
        }
    }
    out
}

/// A folded crate's root: no `unreachable_pub` warning (its pub items are crate-internal now), its
/// README included from beside it, and springrep's build-script hash as a constant.
fn folded_root_fixes(text: &str, rel: &str, module: &str, root: &Path, dir: &str) -> Result<String, String> {
    let mut t = text.to_string();
    if rel == "lib.rs" {
        t = t.replace("#![warn(unreachable_pub)]\n", "").replace("include_str!(\"../README.md\")", "include_str!(\"README.md\")");
    }
    if t.contains("env!(\"SPRINGREP_SOURCE\")") {
        let h = springrep_source(&root.join(dir))?;
        t = t.replace("env!(\"SPRINGREP_SOURCE\")", &format!("\"{h}\""));
    }
    let _ = module;
    Ok(t)
}

/// springrep's build.rs, run here: FNV-1a over `src/**.rs` (but `tests.rs`), sorted by path.
fn springrep_source(crate_dir: &Path) -> Result<String, String> {
    fn sources(base: &Path, dir: &Path, out: &mut Vec<PathBuf>) -> Result<(), String> {
        for e in std::fs::read_dir(base.join(dir)).map_err(|e| format!("{}: {e}", dir.display()))? {
            let rel = dir.join(e.map_err(|e| e.to_string())?.file_name());
            if base.join(&rel).is_dir() {
                sources(base, &rel, out)?;
            } else if rel.extension().is_some_and(|x| x == "rs") && rel.file_name().is_some_and(|n| n != "tests.rs") {
                out.push(rel);
            }
        }
        Ok(())
    }
    let mut files = Vec::new();
    sources(crate_dir, Path::new("src"), &mut files)?;
    files.sort();
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for f in files {
        let name = f.to_string_lossy().replace('\\', "/");
        let text = std::fs::read(crate_dir.join(&f)).map_err(|e| format!("{}: {e}", f.display()))?;
        for b in name.bytes().chain([0]).chain(text).chain([0]) {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    Ok(format!("{h:016x}"))
}

/// Fails on an `env!` the bundle cannot honour (anything but Cargo's own `CARGO_PKG_*`; an
/// `option_env!` falls back by itself).
fn check_env(text: &str, file: &str) -> Result<(), String> {
    for (k, _) in text.match_indices("env!(\"") {
        // `option_env!` has a fallback; only a bare `env!` must have a value
        if text.get(..k).is_some_and(|t| t.ends_with("option_")) {
            continue;
        }
        let name: String = text.get(k + 6..).unwrap_or_default().chars().take_while(|c| *c != '"').collect();
        if !name.starts_with("CARGO_PKG_") && !name.starts_with("CARGO_MANIFEST_DIR") {
            return Err(format!("{file}: env!(\"{name}\") has no value in the bundle"));
        }
    }
    Ok(())
}

fn internal_mod(folded: &[Folded]) -> String {
    let mut s = String::from(
        "// yack: the internal crates of `yack`, folded in by `cargo xtask bundle`; not part of the API.\n\
         //! Each module is one workspace crate (crates/yack-<name>), its paths rewritten.\n\
         #![allow(dead_code, unreachable_pub, unused_imports, unused_macros, clippy::float_arithmetic)]\n\n",
    );
    for m in folded.iter().filter_map(|f| f.module.as_deref()) {
        // `pub` inside the private `internal`: unreachable from outside, and one folded crate may
        // `pub use` another (yack-shim re-exports yack-linalg)
        let _ = writeln!(s, "pub mod {m};");
    }
    s
}

fn manifest(tree: &Tree, spdx: &str) -> Result<String, String> {
    let ws = toml::parse(tree.file("Cargo.toml").ok_or("Cargo.toml is missing")?)?;
    let get = |k: &str| ws.path(&["workspace", "package", k]).and_then(Toml::as_str).unwrap_or_default().to_string();
    let facade = toml::parse(tree.file("crates/yack/Cargo.toml").ok_or("crates/yack/Cargo.toml is missing")?)?;
    let desc = facade.path(&["package", "description"]).and_then(Toml::as_str).unwrap_or_default();
    let mut lints = String::new();
    for table in ["rust", "clippy"] {
        let Some(t) = ws.path(&["workspace", "lints", table]).and_then(Toml::as_table) else { continue };
        let _ = writeln!(lints, "\n[lints.{table}]");
        for (k, v) in t {
            if let Some(s) = v.as_str() {
                let _ = writeln!(lints, "{k} = \"{s}\"");
            }
        }
        if table == "clippy" {
            // the resolved feature cfgs are `all()`/`any()` by design
            lints.push_str("non_minimal_cfg = \"allow\"\n");
        }
    }
    Ok(format!(
        "# Generated by `cargo xtask bundle` from the yack workspace (see BUNDLE.md); do not edit.\n\
         [package]\nname = \"yack\"\nversion = \"{}\"\nedition = \"{}\"\nrust-version = \"{}\"\nlicense = \"{spdx}\"\n\
         description = \"{desc}\"\nrepository = \"{}\"\nreadme = \"README.md\"\n\
         include = [\"src/**\", \"README.md\", \"LICENSE-MIT\", \"LICENSE-APACHE\", \"NOTICE\", \"licenses/**\", \"BUNDLE.md\"]\n\
         {lints}\n# Not a member of the yack workspace.\n[workspace]\n",
        get("version"),
        get("edition"),
        get("rust-version"),
        get("repository"),
    ))
}

fn notice(tree: &Tree, folded: &[Folded], licenses: &[(String, String)]) -> Result<String, String> {
    let mut s = tree.file("NOTICE").ok_or("NOTICE is missing")?.to_string();
    s.push_str("\nPaths in this package\n---------------------\n");
    s.push_str("This package is the yack workspace folded into one crate by `cargo xtask bundle`. Where the\n");
    s.push_str("text above names a workspace path, the code is now at:\n");
    for f in folded {
        let to = f.module.as_ref().map_or("src".to_string(), |m| format!("src/internal/{m}"));
        let _ = writeln!(s, "- {}/src -> {to}", f.pkg.dir);
    }
    for (from, to) in licenses {
        let _ = writeln!(s, "- {from} -> {to}");
    }
    for (from, _) in licenses {
        if !s.contains(from.as_str()) {
            return Err(format!("NOTICE does not name the licence file {from}"));
        }
    }
    Ok(s)
}

fn bundle_md(root: &Path, map: &[(String, String)]) -> String {
    let fp = fingerprint(root);
    let mut s = format!(
        "# yack, bundled\n\nGenerated by `cargo xtask bundle` from the yack workspace at `{fp}` (HEAD + a hash of the\nworking-tree diff). Every file below came from the path on its right.\n\n"
    );
    for (to, from) in map {
        let _ = writeln!(s, "- `{to}` <- `{from}`");
    }
    s
}

fn fingerprint(root: &Path) -> String {
    let run = |args: &[&str]| {
        Command::new("git").args(args).current_dir(root).output().map(|o| String::from_utf8_lossy(&o.stdout).into_owned()).unwrap_or_default()
    };
    let head = run(&["rev-parse", "--short", "HEAD"]).trim().to_string();
    let diff = run(&["diff"]) + &run(&["status", "--short", "--untracked-files=all"]);
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in diff.bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{head}+{h:016x}")
}

// ---- checks ----

fn deterministic(root: &Path, tree: &Tree, out: &Path) -> Result<String, String> {
    let again = root.join("bin/bundle/yack.again");
    make(root, tree, &again)?;
    let a = read_tree(out)?;
    let b = read_tree(&again)?;
    let _ = std::fs::remove_dir_all(&again);
    // BUNDLE.md records the tree fingerprint, the same both times
    if a == b { Ok(format!("{} files byte-identical", a.len())) } else { Err("two runs differ".into()) }
}

fn read_tree(dir: &Path) -> Result<BTreeMap<String, Vec<u8>>, String> {
    fn walk(base: &Path, dir: &Path, out: &mut BTreeMap<String, Vec<u8>>) -> Result<(), String> {
        for e in std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))? {
            let p = e.map_err(|e| e.to_string())?.path();
            if p.is_dir() {
                walk(base, &p, out)?;
            } else {
                let rel = p.strip_prefix(base).unwrap_or(&p).to_string_lossy().replace('\\', "/");
                out.insert(rel, std::fs::read(&p).map_err(|e| format!("{}: {e}", p.display()))?);
            }
        }
        Ok(())
    }
    let mut out = BTreeMap::new();
    walk(dir, dir, &mut out)?;
    Ok(out)
}

fn bundle_gate(out: &Path, r: &Report) -> Result<String, String> {
    let m = toml::parse(&std::fs::read_to_string(out.join("Cargo.toml")).map_err(|e| e.to_string())?)?;
    if m.get("dependencies").is_some() || m.get("build-dependencies").is_some() {
        return Err("the bundle has dependencies".into());
    }
    if out.join("build.rs").exists() || m.path(&["package", "build"]).is_some() {
        return Err("the bundle has a build script".into());
    }
    if m.path(&["package", "license"]).and_then(Toml::as_str) != Some(r.spdx.as_str()) {
        return Err("the bundle's license is not the closure's SPDX expression".into());
    }
    if m.path(&["lints", "rust", "unsafe_code"]).and_then(Toml::as_str) != Some("forbid") {
        return Err("unsafe_code is not forbidden".into());
    }
    for f in ["NOTICE", "LICENSE-MIT", "LICENSE-APACHE", "README.md"] {
        if !out.join(f).is_file() {
            return Err(format!("{f} is missing"));
        }
    }
    if r.headers_in != r.headers_out {
        return Err(format!("provenance headers: {} in the sources, {} in the bundle", r.headers_in, r.headers_out));
    }
    Ok(format!("no deps, no build script, unsafe forbidden, {} provenance headers kept", r.headers_out))
}

fn cargo(root: &Path, pre: &[&str], manifest: &Path, post: &[&str]) -> Result<(), String> {
    let status = Command::new("cargo")
        .args(pre)
        .arg(manifest)
        .args(post)
        .current_dir(root)
        .status()
        .map_err(|e| format!("cargo: {e}"))?;
    if status.success() { Ok(()) } else { Err(format!("cargo {} failed", pre.join(" "))) }
}

/// The bundle's public surface, read as gate 4 reads `yack`, equals crates/yack/api.txt.
fn api(tree: &Tree, out: &Path) -> Result<String, String> {
    let mut files = BTreeMap::new();
    for (rel, bytes) in read_tree(&out.join("src"))? {
        if !rel.starts_with("internal/") {
            files.insert(format!("crates/yack/src/{rel}"), String::from_utf8_lossy(&bytes).into_owned());
        }
    }
    files.insert("layers.toml".to_string(), tree.file("layers.toml").unwrap_or_default().to_string());
    let pkg = tree.package("yack").ok_or("no yack package")?.clone();
    let fake = Tree { paths: files.keys().cloned().collect(), files, packages: vec![pkg], ..Tree::default() };
    let ctx = crate::gate::Ctx::new(fake)?;
    let s = crate::gate::api::surface(&ctx).ok_or("no surface")?;
    let mut have: BTreeSet<String> = s.lines.iter().map(|l| unfold(l)).collect();
    let want: BTreeSet<String> = tree
        .file("crates/yack/api.txt")
        .unwrap_or_default()
        .lines()
        .map(str::trim_end)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(str::to_string)
        .collect();
    have.retain(|l| !l.is_empty());
    let extra: Vec<&String> = have.difference(&want).collect();
    let missing: Vec<&String> = want.difference(&have).collect();
    if extra.is_empty() && missing.is_empty() {
        Ok(format!("{} items, equal to crates/yack/api.txt", have.len()))
    } else {
        Err(format!("only in the bundle: {extra:?}; only in api.txt: {missing:?}"))
    }
}

/// `crate::internal::<x>` → `yack_<x>`, the spelling api.txt records.
fn unfold(line: &str) -> String {
    let mut s = line.to_string();
    while let Some(k) = s.find("crate::internal::") {
        s.replace_range(k..k + "crate::internal::".len(), "yack_");
    }
    s
}

fn package(root: &Path, manifest: &Path) -> Result<String, String> {
    cargo(root, &["package", "--allow-dirty", "--manifest-path"], manifest, &[])?;
    let dir = root.join("bin/package");
    let mut sizes = Vec::new();
    for e in std::fs::read_dir(&dir).map_err(|e| format!("{}: {e}", dir.display()))? {
        let p = e.map_err(|e| e.to_string())?.path();
        if p.file_name().is_some_and(|n| n.to_string_lossy().starts_with("yack-")) && p.extension().is_some_and(|x| x == "crate") {
            sizes.push((p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default(), p.metadata().map(|m| m.len()).unwrap_or(0)));
        }
    }
    let (name, bytes) = sizes.into_iter().max_by_key(|(_, b)| *b).ok_or("no .crate file in bin/package")?;
    let mib = bytes as f64 / f64::from(1 << 20);
    if bytes > 10 << 20 {
        return Err(format!("{name} is {mib:.2} MiB, over crates.io's 10 MB"));
    }
    Ok(format!("{name}: {bytes} bytes ({mib:.2} MiB)"))
}

/// Builds the workspace runner and one on the bundle, and copies both out of `bin/` to
/// `../yack-ref/bin/bundle-<fingerprint>/` for a digest comparison outside the builder.
fn bench_build(root: &Path, tree: &Tree) -> Result<PathBuf, String> {
    build_runners(root, tree)?;
    let dir = root.join("..").join("yack-ref").join("bin").join(format!("bundle-{}", fingerprint(root)));
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    for exe in ["yack-bench", "yack-bench-bundle"] {
        std::fs::copy(root.join("bin/release").join(exe), dir.join(exe)).map_err(|e| format!("{exe}: {e}"))?;
    }
    Ok(dir)
}

/// A copy of yack-bench built on the bundle gives every case the digest the workspace runner does.
fn digests(root: &Path, tree: &Tree) -> Result<String, String> {
    build_runners(root, tree)?;
    compare(&root.join("bin/release"))
}

fn build_runners(root: &Path, tree: &Tree) -> Result<(), String> {
    let check = root.join(CHECK);
    if check.exists() {
        std::fs::remove_dir_all(&check).map_err(|e| e.to_string())?;
    }
    let bench = tree.package("yack-bench").ok_or("no yack-bench")?;
    let prefix = format!("{}/", bench.dir);
    let none = BTreeMap::new();
    for (p, text) in tree.files.iter().filter(|(p, _)| p.starts_with(&prefix)) {
        let rel = p.strip_prefix(&prefix).unwrap_or(p);
        if !(rel.starts_with("src/") || rel == "build.rs") {
            continue;
        }
        let dest = check.join(rel);
        if let Some(d) = dest.parent() {
            std::fs::create_dir_all(d).map_err(|e| e.to_string())?;
        }
        std::fs::write(&dest, features(text, &none)).map_err(|e| e.to_string())?;
    }
    let manifest = "[package]\nname = \"yack-bench-bundle\"\nversion = \"0.0.0\"\nedition = \"2024\"\npublish = false\nbuild = \"build.rs\"\n\n\
                    [[bin]]\nname = \"yack-bench-bundle\"\npath = \"src/main.rs\"\n\n[dependencies]\nyack = { path = \"../../yack\" }\n\n[workspace]\n";
    std::fs::write(check.join("Cargo.toml"), manifest).map_err(|e| e.to_string())?;
    cargo(root, &["build", "--release", "--manifest-path"], &check.join("Cargo.toml"), &[])?;
    let status = Command::new("cargo").args(["build", "--release", "-p", "yack-bench"]).current_dir(root).status().map_err(|e| e.to_string())?;
    if !status.success() {
        return Err("building the workspace yack-bench failed".into());
    }
    Ok(())
}

/// Every case's digest from `dir/yack-bench` and `dir/yack-bench-bundle`, compared.
fn compare(dir: &Path) -> Result<String, String> {
    let run = |exe: &str| -> Result<BTreeMap<String, String>, String> {
        let o = Command::new(dir.join(exe)).arg("--digests").output().map_err(|e| format!("{exe}: {e}"))?;
        if !o.status.success() {
            return Err(format!("{exe} --digests failed"));
        }
        Ok(String::from_utf8_lossy(&o.stdout)
            .lines()
            .filter_map(|l| l.trim().split_once(' ').map(|(a, b)| (a.to_string(), b.to_string())))
            .collect())
    };
    let ws = run("yack-bench")?;
    let bu = run("yack-bench-bundle")?;
    let differ: Vec<&String> = ws.keys().chain(bu.keys()).collect::<BTreeSet<_>>().into_iter().filter(|k| ws.get(*k) != bu.get(*k)).collect();
    if differ.is_empty() {
        let refused = ws.values().filter(|v| *v == "refused").count();
        Ok(format!("{} cases, all equal ({} built, {refused} refused alike)", ws.len(), ws.len() - refused))
    } else {
        let shown: Vec<String> = differ.iter().take(10).map(|k| format!("{k}: {:?} vs {:?}", ws.get(*k), bu.get(*k))).collect();
        Err(format!("{} cases differ: {}", differ.len(), shown.join("; ")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rewrite_touches_code_only() {
        let crates: BTreeMap<String, String> = [("yack_math".to_string(), "math".to_string())].into_iter().collect();
        let src = "use yack_math::Point3; // yack_math::x\nlet s = \"yack_math::y\"; use crate::a; pub(crate) fn f() { $crate::b }\nlet yack_math = 1;\n";
        let out = rewrite(src, &crates, Some("base"));
        assert!(out.contains("use crate::internal::math::Point3; // yack_math::x"));
        assert!(out.contains("\"yack_math::y\""));
        assert!(out.contains("use crate::internal::base::a;"));
        assert!(out.contains("pub(crate) fn f()"));
        assert!(out.contains("$crate::internal::base::b"));
        assert!(out.contains("let yack_math = 1;"), "a local of the same name is not a path");
        let tricky = "let q = b'\\\\'; let r = '\\''; S { ..crate::D::d() }; yack_math::f()";
        let out = rewrite(tricky, &crates, Some("base"));
        assert!(out.ends_with("S { ..crate::internal::base::D::d() }; crate::internal::math::f()"), "{out}");
    }

    #[test]
    fn a_field_named_like_a_crate_is_left() {
        let crates: BTreeMap<String, String> = [("yack_tol".to_string(), "tol".to_string())].into_iter().collect();
        assert_eq!(rewrite("x.yack_tol + a::yack_tol", &crates, None), "x.yack_tol + a::yack_tol");
    }

    #[test]
    fn doc_examples_of_folded_crates_are_fenced() {
        let out = fence_docs("/// a\n/// ```\n/// use yack_x::y;\n/// ```\nfn f() {}\n");
        assert_eq!(out, "/// a\n/// ```text\n/// use yack_x::y;\n/// ```\nfn f() {}\n");
    }

    #[test]
    fn features_resolve_to_constant_cfgs() {
        let f: BTreeMap<String, bool> = [("testkit".to_string(), true), ("x".to_string(), false)].into_iter().collect();
        assert_eq!(features("#[cfg(feature = \"testkit\")] #[cfg(not(feature = \"x\"))]", &f), "#[cfg(all())] #[cfg(not(any()))]");
    }
}
