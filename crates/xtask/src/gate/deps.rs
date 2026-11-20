// yack: gates 1 (layers), 2 (one public crate), 3 (zero dependencies) and 12 (temporaries).
use super::{Ctx, Out};
use crate::toml::{self, Toml};
use std::collections::{BTreeMap, BTreeSet};

/// Gate 1: every crate is listed; every internal edge is allow-listed and points strictly lower.
pub fn layers(ctx: &Ctx, out: &mut Out) {
    for (name, spec) in &ctx.layers.crates {
        for d in &spec.deps {
            if !ctx.layers.crates.contains_key(d) {
                out.fail(format!("layers.toml: `{name}` allows unknown crate `{d}`"));
            }
        }
    }
    for p in &ctx.tree.packages {
        let Some(spec) = ctx.spec(&p.name) else {
            out.fail(format!("crate `{}` ({}) is not listed in layers.toml", p.name, p.dir));
            continue;
        };
        if p.dir != format!("crates/{}", p.name) {
            out.fail(format!("crate `{}` lives in `{}`, not `crates/{}`", p.name, p.dir, p.name));
        }
        for d in p.deps.iter().filter(|d| d.internal) {
            let Some(target) = ctx.spec(&d.name) else {
                out.fail(format!("`{}` depends on `{}`, which is not in layers.toml", p.name, d.name));
                continue;
            };
            if !spec.deps.contains(&d.name) {
                out.fail(format!("`{}` → `{}` is not in `{}`'s allow-list ({:?} dependency)", p.name, d.name, p.name, d.kind));
            }
            if target.layer >= spec.layer {
                out.fail(format!(
                    "`{}` (layer {}) → `{}` (layer {}) does not point strictly lower",
                    p.name, spec.layer, d.name, target.layer
                ));
            }
        }
    }
}

/// Gate 2: only `yack` is publishable; only tool crates depend on `yack`; nothing depends on a tool.
pub fn one_public(ctx: &Ctx, out: &mut Out) {
    for p in &ctx.tree.packages {
        if p.publishable && p.name != "yack" {
            out.fail(format!("`{}` lacks `publish = false`; only `yack` is public", p.name));
        }
        let tool = ctx.spec(&p.name).is_some_and(|s| s.is_tool());
        for d in p.deps.iter().filter(|d| d.internal) {
            if d.name == "yack" && !tool {
                out.fail(format!("`{}` depends on `yack`; only tool crates may", p.name));
            }
            if ctx.spec(&d.name).is_some_and(|s| s.is_tool()) {
                out.fail(format!("`{}` depends on tool crate `{}`", p.name, d.name));
            }
        }
    }
    for (name, spec) in &ctx.layers.crates {
        if spec.kind == "public" && name != "yack" {
            out.fail(format!("layers.toml: `{name}` is kind public; only `yack` is"));
        }
    }
}

struct LockPkg {
    name: String,
    version: String,
    external: bool,
    deps: Vec<(String, Option<String>)>,
}

fn read_lock(text: &str, workspace: &BTreeSet<&str>) -> Result<Vec<LockPkg>, String> {
    let t = toml::parse(text).map_err(|e| format!("Cargo.lock: {e}"))?;
    let mut out = Vec::new();
    for p in t.get("package").map(Toml::as_arr).unwrap_or_default() {
        let name = p.get("name").and_then(Toml::as_str).unwrap_or_default().to_string();
        let version = p.get("version").and_then(Toml::as_str).unwrap_or_default().to_string();
        let external = p.get("source").is_some() || !workspace.contains(name.as_str());
        let deps = p
            .get("dependencies")
            .map(Toml::strings)
            .unwrap_or_default()
            .iter()
            .map(|d| {
                let mut it = d.split_whitespace();
                (it.next().unwrap_or_default().to_string(), it.next().map(str::to_string))
            })
            .collect();
        out.push(LockPkg { name, version, external, deps });
    }
    Ok(out)
}

/// Gate 3: no external package is reachable from any crate except through a tool's `external` list.
pub fn zero_deps(ctx: &Ctx, out: &mut Out) {
    for p in &ctx.tree.packages {
        let spec = ctx.spec(&p.name);
        let allowed: &[String] = spec.map(|s| s.external.as_slice()).unwrap_or_default();
        if !allowed.is_empty() && !spec.is_some_and(|s| s.is_tool()) {
            out.fail(format!("layers.toml: `{}` lists external packages but is not a tool crate", p.name));
        }
        for d in p.deps.iter().filter(|d| !d.internal) {
            if !allowed.contains(&d.name) {
                out.fail(format!("`{}` depends on external package `{}` ({:?})", p.name, d.name, d.kind));
            }
        }
    }
    let Some(text) = ctx.tree.file("Cargo.lock") else {
        out.fail("Cargo.lock is missing");
        return;
    };
    let workspace: BTreeSet<&str> = ctx.tree.packages.iter().map(|p| p.name.as_str()).collect();
    let lock = match read_lock(text, &workspace) {
        Ok(l) => l,
        Err(e) => {
            out.fail(e);
            return;
        }
    };
    let find = |name: &str, version: Option<&String>| {
        lock.iter().position(|l| l.name == name && version.is_none_or(|v| *v == l.version))
    };
    for p in &ctx.tree.packages {
        if ctx.spec(&p.name).is_some_and(|s| !s.external.is_empty()) {
            continue; // its externals are allow-listed and checked above
        }
        let Some(start) = find(p.name.as_str(), None) else { continue };
        let mut seen = BTreeSet::from([start]);
        let mut stack = vec![start];
        let mut reached = BTreeSet::new();
        while let Some(i) = stack.pop() {
            let Some(l) = lock.get(i) else { continue };
            if l.external {
                reached.insert(l.name.clone());
            }
            for (n, v) in &l.deps {
                if let Some(j) = find(n.as_str(), v.as_ref())
                    && seen.insert(j)
                {
                    stack.push(j);
                }
            }
        }
        for r in reached {
            out.fail(format!("external package `{r}` is reachable from `{}` (Cargo.lock)", p.name));
        }
    }
}

/// Non-blank lines of a crate's Rust sources.
pub fn crate_loc(ctx: &Ctx, krate: &str) -> Option<usize> {
    let p = ctx.tree.package(krate)?;
    let prefix = format!("{}/", p.dir);
    Some(
        ctx.tree
            .files
            .iter()
            .filter(|(k, _)| k.starts_with(&prefix) && k.ends_with(".rs"))
            .map(|(_, t)| t.lines().filter(|l| !l.trim().is_empty()).count())
            .sum(),
    )
}

/// Non-blank Rust lines under `prefix`.
fn loc_under(ctx: &Ctx, prefix: &str) -> usize {
    ctx.tree
        .files
        .iter()
        .filter(|(k, _)| k.starts_with(prefix) && k.ends_with(".rs"))
        .map(|(_, t)| t.lines().filter(|l| !l.trim().is_empty()).count())
        .sum()
}

/// The imported cadrs (`src/native/**`), which may only shrink: the ratcheted count.
pub const CADRS_NATIVE: &str = "yack-cadrs/native";
/// The facade glue (`src/facade/**` + `src/yack_glue.rs`), reported but allowed to grow until M4.
pub const CADRS_GLUE: &str = "yack-cadrs/glue";

/// yack-cadrs lines, split into the imported cadrs and the glue.
pub fn cadrs_loc(ctx: &Ctx) -> BTreeMap<String, usize> {
    let Some(p) = ctx.tree.package("yack-cadrs") else { return BTreeMap::new() };
    let src = format!("{}/src/", p.dir);
    let native = loc_under(ctx, &format!("{src}native/"));
    let glue = loc_under(ctx, &format!("{src}facade/")) + loc_under(ctx, &format!("{src}yack_glue.rs"));
    BTreeMap::from([(CADRS_NATIVE.to_string(), native), (CADRS_GLUE.to_string(), glue)])
}

/// Gate 12: temporaries die by their milestone, and only the right crates lean on them.
pub fn temporaries(ctx: &Ctx, out: &mut Out) {
    for (name, spec) in ctx.layers.crates.iter().filter(|(_, s)| s.is_temporary()) {
        match &spec.delete_by {
            None => out.fail(format!("layers.toml: temporary `{name}` has no delete_by")),
            Some(m) if ctx.tree.package(name).is_some() && ctx.tree.has_tag(m) => {
                out.fail(format!("temporary `{name}` still exists after tag `{}`", m.to_ascii_lowercase()));
            }
            Some(_) => {}
        }
    }
    for p in &ctx.tree.packages {
        let mine = ctx.spec(&p.name);
        for d in p.deps.iter().filter(|d| d.internal) {
            if d.name == "yack-shim" && p.name != "yack-cadrs" {
                out.fail(format!("`{}` depends on `yack-shim`; only `yack-cadrs` may", p.name));
            }
            let target_temp = ctx.spec(&d.name).is_some_and(|s| s.is_temporary());
            let ok = mine.is_some_and(|s| s.is_temporary() || s.kind == "public");
            if target_temp && !ok {
                out.fail(format!("`{}` depends on temporary `{}`; only `yack` and temporaries may", p.name, d.name));
            }
        }
    }
    // only the imported cadrs is ratcheted; the glue may grow until M4 and is reported
    let allowed = ctx.ratchet("cadrs-loc.txt");
    let now = cadrs_loc(ctx);
    if let Some(&n) = now.get(CADRS_NATIVE) {
        match allowed.get(CADRS_NATIVE) {
            Some(&max) if n > max => {
                out.fail(format!("`{CADRS_NATIVE}` grew to {n} lines; scores/cadrs-loc.txt allows {max} (it may only shrink)"));
            }
            Some(&max) => out.note(format!("`{CADRS_NATIVE}`: {n} lines (ratchet {max})")),
            None => out.note(format!("`{CADRS_NATIVE}`: {n} lines (no ratchet yet; `gate --bless` records it)")),
        }
    }
    if let Some(&g) = now.get(CADRS_GLUE) {
        out.note(format!("`{CADRS_GLUE}`: {g} lines (not ratcheted until M4)"));
    }
}
