// yack: gates 6 (file size), 10 (provenance and licence) and 11 (build directory).
use super::lex::item_end;
use super::{Ctx, Out};
use crate::toml::{self, Toml};
use std::collections::{BTreeMap, BTreeSet};

const MAX_LINES: usize = 1500;
const MAX_FN_LINES: usize = 150;

fn rust_files(ctx: &Ctx) -> impl Iterator<Item = (&String, &String)> {
    ctx.tree.files.iter().filter(|(p, _)| p.starts_with("crates/") && p.ends_with(".rs"))
}

/// Files over the limit: path → line count.
pub fn oversize(ctx: &Ctx) -> BTreeMap<String, usize> {
    rust_files(ctx).map(|(p, t)| (p.clone(), t.lines().count())).filter(|(_, n)| *n > MAX_LINES).collect()
}

/// Gate 6: files ≤ 1,500 lines unless allow-listed in scores/filesize.txt (which may only shrink);
/// a function over 150 lines warns.
pub fn file_size(ctx: &Ctx, out: &mut Out) {
    let allowed = ctx.ratchet("filesize.txt");
    let big = oversize(ctx);
    for (path, n) in &big {
        match allowed.get(path) {
            None => out.fail(format!("{path}: {n} lines (limit {MAX_LINES}; not in scores/filesize.txt)")),
            Some(&max) if *n > max => out.fail(format!("{path}: grew to {n} lines (scores/filesize.txt allows {max})")),
            Some(_) => {}
        }
    }
    for (path, max) in &allowed {
        if !big.contains_key(path) {
            out.note(format!("scores/filesize.txt: `{path}` (allowed {max}) is now within {MAX_LINES} lines; drop it"));
        }
    }
    for s in ctx.srcs.values().flatten() {
        for (k, t) in s.toks.iter().enumerate() {
            if !t.is_ident("fn") || !s.toks.get(k + 1).is_some_and(|n| n.ident().is_some()) {
                continue;
            }
            let end = item_end(&s.toks, k);
            let Some(last) = s.toks.get(end) else { continue };
            if last.is_punct("}") && last.line.saturating_sub(t.line) + 1 > MAX_FN_LINES {
                let name = s.toks.get(k + 1).and_then(|n| n.ident()).unwrap_or_default();
                out.warn(format!("{}:{}: fn `{name}` is {} lines (> {MAX_FN_LINES})", s.path, t.line, last.line - t.line + 1));
            }
        }
    }
}

/// Donor projects a provenance header may name (PLAN §9). next.BREP is never one.
const DONORS: [&str; 14] = [
    "springrep",
    "cadrs",
    "cad-kernel",
    "geomsolver",
    "ssi",
    "solidcore",
    "solidcraft",
    "solidcraft-main",
    "truck",
    "brep-conformance",
    "brep-conformance-springrep",
    "brep-protocol",
    "brep-bench",
    "smlib",
];
const LICENCES: [&str; 4] = ["MIT", "MIT OR Apache-2.0", "Apache-2.0 OR MIT", "Apache-2.0"];
const APACHE_ONLY: &str = "Apache-2.0";
const CRATE_LICENCES: [&str; 2] = ["MIT OR Apache-2.0", APACHE_ONLY];

/// Paths that must never enter the tree (lower-cased substrings).
const BANNED_PATHS: [&str; 5] = ["next.brep", "next-brep", "next_brep", "nextbrep", "tolblend_c9"];

/// An `Adapted from <donor>@<rev> <path> (<licence>, (c) <holder>).` line → (donor, licence).
fn adapted(line: &str) -> Result<(String, String), String> {
    let rest = line.trim_start_matches("// Adapted from").trim();
    let (donor, after) = rest.split_once('@').ok_or("no `<donor>@<rev>`")?;
    let donor = donor.trim().to_string();
    let open = after.find('(').ok_or("no `(<licence>, (c) <holder>)`")?;
    let lic = after.get(open + 1..).unwrap_or_default();
    let lic = lic.split([',', ')']).next().unwrap_or_default().trim().to_string();
    Ok((donor, lic))
}

/// Gate 10: every file has a `yack` header and allow-listed donors; licences match the headers.
pub fn provenance(ctx: &Ctx, out: &mut Out) {
    for p in &ctx.tree.paths {
        let low = p.to_ascii_lowercase();
        if let Some(b) = BANNED_PATHS.iter().find(|b| low.contains(*b)) {
            out.fail(format!("{p}: banned path (`{b}`; PLAN §9 never-allowed list)"));
        }
        if low.contains("tests/data/docs/") && low.ends_with(".body") {
            out.fail(format!("{p}: unlicensed cadrs .body file"));
        }
    }
    // crate dir → (has Apache-only file)
    let mut apache: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut donors_used = BTreeSet::new();
    for (path, text) in rust_files(ctx) {
        let mut lines = text.lines();
        match lines.next() {
            Some(l) if l.strip_prefix("// yack:").is_some_and(|r| !r.trim().is_empty()) => {}
            _ => {
                out.fail(format!("{path}: first line is not `// yack: <purpose>`"));
                continue;
            }
        }
        for l in lines.take_while(|l| l.starts_with("//")) {
            if !l.starts_with("// Adapted from") {
                continue;
            }
            match adapted(l) {
                Err(e) => out.fail(format!("{path}: bad provenance line ({e}): {l}")),
                Ok((donor, lic)) => {
                    let head = donor.split('/').next().unwrap_or_default().to_ascii_lowercase();
                    if !DONORS.contains(&head.as_str()) {
                        out.fail(format!("{path}: donor `{donor}` is not allow-listed (PLAN §9)"));
                    } else {
                        donors_used.insert(head);
                    }
                    if !LICENCES.contains(&lic.as_str()) {
                        out.fail(format!("{path}: licence `{lic}` is not MIT, MIT OR Apache-2.0 or Apache-2.0"));
                    }
                    if lic == APACHE_ONLY {
                        let dir = path.split('/').take(2).collect::<Vec<_>>().join("/");
                        apache.entry(dir).or_default().push(path.clone());
                    }
                }
            }
        }
    }
    for p in &ctx.tree.packages {
        let lic = p.license.clone().unwrap_or_default();
        if !CRATE_LICENCES.contains(&lic.as_str()) {
            out.fail(format!("`{}` has licence `{lic}`; expected `MIT OR Apache-2.0` or `Apache-2.0`", p.name));
        }
        if let Some(want) = ctx.spec(&p.name).and_then(|s| s.license.as_ref())
            && *want != lic
        {
            out.fail(format!("`{}` has licence `{lic}`, layers.toml says `{want}`", p.name));
        }
        let holds_apache = apache.get(&p.dir);
        match (holds_apache, lic == APACHE_ONLY) {
            (Some(files), false) => out.fail(format!(
                "`{}` holds Apache-2.0-only code ({}) but its licence is `{lic}`",
                p.name,
                files.first().map(String::as_str).unwrap_or_default()
            )),
            (None, true) => out.fail(format!("`{}` says `Apache-2.0` but holds no Apache-2.0-only file", p.name)),
            _ => {}
        }
    }
    let notice = ctx.tree.file("NOTICE").unwrap_or_default();
    let in_tree: String = notice
        .lines()
        .skip_while(|l| !l.contains("code in this tree"))
        .skip(1)
        .take_while(|l| !l.to_ascii_lowercase().contains("planned"))
        .collect::<Vec<_>>()
        .join("\n")
        .to_ascii_lowercase();
    for d in &donors_used {
        let d = match d.as_str() {
            "solidcraft-main" => "solidcraft",
            "brep-conformance-springrep" => "brep-conformance",
            other => other,
        };
        if !in_tree.contains(d) {
            out.fail(format!("donor `{d}` has files in the tree but is not in NOTICE's \"code in this tree\" list"));
        }
    }
    out.note(format!("yack licence (dependency closure): {}", closure_spdx(ctx)));
}

/// The SPDX expression of `yack`'s normal/build dependency closure.
fn closure_spdx(ctx: &Ctx) -> String {
    let mut seen = BTreeSet::new();
    let mut stack = vec!["yack".to_string()];
    let mut apache = false;
    while let Some(n) = stack.pop() {
        if !seen.insert(n.clone()) {
            continue;
        }
        let Some(p) = ctx.tree.package(&n) else { continue };
        apache |= p.license.as_deref() == Some(APACHE_ONLY);
        for d in p.deps.iter().filter(|d| d.internal && d.kind != crate::meta::DepKind::Dev) {
            stack.push(d.name.clone());
        }
    }
    if apache { "(MIT OR Apache-2.0) AND Apache-2.0".to_string() } else { "MIT OR Apache-2.0".to_string() }
}

/// Gate 11: one target dir, `bin/`, git-ignored; no `target/`, no worktrees.
pub fn build_dir(ctx: &Ctx, out: &mut Out) {
    match ctx.tree.file(".cargo/config.toml").map(toml::parse) {
        None => out.fail(".cargo/config.toml is missing"),
        Some(Err(e)) => out.fail(format!(".cargo/config.toml: {e}")),
        Some(Ok(t)) => {
            let td = t.path(&["build", "target-dir"]).and_then(Toml::as_str);
            if td != Some("bin") {
                out.fail(format!(".cargo/config.toml: build.target-dir is {td:?}, not \"bin\""));
            }
            if t.path(&["env", "CARGO_TARGET_DIR"]).is_some() {
                out.fail(".cargo/config.toml sets CARGO_TARGET_DIR");
            }
        }
    }
    let ignored = ctx
        .tree
        .file(".gitignore")
        .unwrap_or_default()
        .lines()
        .any(|l| matches!(l.trim(), "bin/" | "/bin/" | "bin" | "/bin"));
    if !ignored {
        out.fail(".gitignore does not ignore `bin/`");
    }
    for d in &ctx.tree.forbidden_dirs {
        if d == ".git/worktrees" {
            out.fail("`.git/worktrees` exists: no worktrees (R7)");
        } else {
            out.fail(format!("`{d}/` exists: everything builds to `bin/` (R7)"));
        }
    }
}
