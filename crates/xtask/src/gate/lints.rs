// yack: gates 9 (lint configuration) and 13 (the facade routes and never computes).
use super::lex::{Src, Token, matching};
use super::{Ctx, Out};
use crate::toml::{self, Toml};

const DENIED: [&str; 6] = ["unwrap_used", "expect_used", "panic", "todo", "unimplemented", "dbg_macro"];

/// A lint's level in a `[lints]` table: `"deny"` or `{ level = "deny", .. }`.
fn level(v: Option<&Toml>) -> Option<&str> {
    match v? {
        Toml::Str(s) => Some(s.as_str()),
        t @ Toml::Table(_) => t.get("level").and_then(Toml::as_str),
        _ => None,
    }
}

/// Lint attributes `#[allow(..)]` / `#![deny(..)]` in a token stream: (index, inner, level, lint paths).
pub fn lint_attrs(toks: &[Token]) -> Vec<(usize, bool, String, Vec<String>)> {
    let mut out = Vec::new();
    for (k, t) in toks.iter().enumerate() {
        if !t.is_punct("#") {
            continue;
        }
        let inner = toks.get(k + 1).is_some_and(|x| x.is_punct("!"));
        let open = if inner { k + 2 } else { k + 1 };
        if !toks.get(open).is_some_and(|x| x.is_punct("[")) {
            continue;
        }
        let Some(lvl) = toks.get(open + 1).and_then(Token::ident) else { continue };
        if !["allow", "expect", "warn", "deny", "forbid"].contains(&lvl) {
            continue;
        }
        if !toks.get(open + 2).is_some_and(|x| x.is_punct("(")) {
            continue;
        }
        let close = matching(toks, open + 2);
        let mut lints = Vec::new();
        let mut cur = String::new();
        for x in toks.get(open + 3..close).unwrap_or_default() {
            match &x.tok {
                super::lex::Tok::Ident(i) => cur.push_str(i),
                super::lex::Tok::Punct(p) if p == "::" => cur.push_str("::"),
                super::lex::Tok::Punct(p) if p == "," => lints.push(std::mem::take(&mut cur)),
                _ => {}
            }
        }
        if !cur.is_empty() {
            lints.push(cur);
        }
        out.push((k, inner, lvl.to_string(), lints));
    }
    out
}

/// Whether the crate root `src` has an inner `deny`/`forbid` of `lint`.
fn root_denies(src: Option<&Src>, lint: &str) -> bool {
    src.is_some_and(|s| {
        lint_attrs(&s.toks)
            .iter()
            .any(|(_, inner, lvl, lints)| *inner && (lvl == "deny" || lvl == "forbid") && lints.iter().any(|l| l == lint))
    })
}

fn yack_lib(ctx: &Ctx) -> Option<&Src> {
    let dir = &ctx.tree.package("yack")?.dir;
    let lib = format!("{dir}/src/lib.rs");
    ctx.srcs.get("yack")?.iter().find(|s| s.path == lib)
}

/// Gate 9: the workspace lint table, `[lints] workspace = true` everywhere, `missing_docs` on
/// `yack`, and no shipping code that allows a denied lint.
pub fn lints(ctx: &Ctx, out: &mut Out) {
    match ctx.tree.file("Cargo.toml").map(toml::parse) {
        None => out.fail("root Cargo.toml is missing"),
        Some(Err(e)) => out.fail(format!("Cargo.toml: {e}")),
        Some(Ok(ws)) => {
            let rust = ws.path(&["workspace", "lints", "rust"]);
            let clippy = ws.path(&["workspace", "lints", "clippy"]);
            if level(rust.and_then(|r| r.get("unsafe_code"))) != Some("forbid") {
                out.fail("workspace.lints.rust.unsafe_code is not \"forbid\"");
            }
            for l in DENIED {
                let lv = level(clippy.and_then(|c| c.get(l)));
                if !matches!(lv, Some("deny" | "forbid")) {
                    out.fail(format!("workspace.lints.clippy.{l} is {lv:?}, not \"deny\""));
                }
            }
            let idx = level(clippy.and_then(|c| c.get("indexing_slicing")));
            let m5 = ctx.tree.has_tag("m5");
            let ok = if m5 { matches!(idx, Some("deny" | "forbid")) } else { matches!(idx, Some("warn" | "deny" | "forbid")) };
            if !ok {
                out.fail(format!("workspace.lints.clippy.indexing_slicing is {idx:?} (warn until M5, deny after)"));
            }
        }
    }
    for p in &ctx.tree.packages {
        if ctx.spec(&p.name).is_some_and(|s| s.is_temporary()) {
            continue; // ratcheted below instead
        }
        let path = format!("{}/Cargo.toml", p.dir);
        let ws = ctx.tree.file(&path).map(toml::parse).and_then(Result::ok);
        if ws.as_ref().and_then(|m| m.path(&["lints", "workspace"])).and_then(Toml::as_bool) != Some(true) {
            out.fail(format!("{path}: missing `[lints] workspace = true`"));
        }
    }
    if ctx.tree.package("yack").is_some() && !root_denies(yack_lib(ctx), "missing_docs") {
        out.fail("crates/yack/src/lib.rs lacks `#![deny(missing_docs)]`");
    }
    let mut banned: Vec<String> = DENIED.iter().map(|l| format!("clippy::{l}")).collect();
    banned.push("unsafe_code".to_string());
    for (name, spec, srcs) in ctx.crates() {
        if !spec.ships() || spec.is_temporary() {
            continue;
        }
        let mut banned = banned.clone();
        if name == "yack" {
            banned.extend(["missing_docs".to_string(), "clippy::float_arithmetic".to_string()]);
        }
        for s in srcs {
            for (k, _, lvl, ls) in lint_attrs(&s.toks) {
                if !s.live(k) || !(lvl == "allow" || lvl == "expect" || lvl == "warn") {
                    continue;
                }
                for l in ls.iter().filter(|l| banned.contains(l)) {
                    let line = s.toks.get(k).map(|t| t.line).unwrap_or_default();
                    out.fail(format!("{}:{line}: `{lvl}({l})` in shipping crate `{name}`", s.path));
                }
            }
        }
    }
    temporary_lints(ctx, out);
}

/// Sites in a temporary crate's non-test code that the workspace lints would deny: `.unwrap()`,
/// `.expect(`, `panic!`/`todo!`/`unimplemented!`/`dbg!`, `unsafe`, and allows of those lints.
fn temporary_sites(srcs: &[Src], banned: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    for s in srcs {
        let tok = |k: usize| s.toks.get(k);
        for (k, t) in s.toks.iter().enumerate() {
            if !s.live(k) {
                continue;
            }
            let at = format!("{}:{}", s.path, t.line);
            let method = (t.is_ident("unwrap") || t.is_ident("expect"))
                && k > 0
                && tok(k - 1).is_some_and(|x| x.is_punct("."))
                && tok(k + 1).is_some_and(|x| x.is_punct("("));
            let mac = ["panic", "todo", "unimplemented", "dbg"].iter().any(|m| t.is_ident(m))
                && tok(k + 1).is_some_and(|x| x.is_punct("!"));
            if method || mac || t.is_ident("unsafe") {
                out.push(format!("{at}: `{}`", t.ident().unwrap_or_default()));
            }
        }
        for (k, _, lvl, ls) in lint_attrs(&s.toks) {
            if s.live(k) && matches!(lvl.as_str(), "allow" | "expect" | "warn") && ls.iter().any(|l| banned.contains(l)) {
                let line = s.toks.get(k).map(|t| t.line).unwrap_or_default();
                out.push(format!("{}:{line}: `{lvl}({})`", s.path, ls.join(", ")));
            }
        }
    }
    out
}

fn temporary_banned() -> Vec<String> {
    let mut b: Vec<String> = DENIED.iter().map(|l| format!("clippy::{l}")).collect();
    b.push("unsafe_code".to_string());
    b
}

/// Denied-lint sites per temporary crate (scores/lints.txt).
pub fn lint_counts(ctx: &Ctx) -> std::collections::BTreeMap<String, usize> {
    let banned = temporary_banned();
    ctx.crates()
        .filter(|(_, s, _)| s.is_temporary())
        .map(|(n, _, srcs)| (n.to_string(), temporary_sites(srcs, &banned).len()))
        .filter(|(_, c)| *c > 0)
        .collect()
}

/// Gate 9 for temporary crates: they may lack `[lints] workspace = true` while being untangled,
/// but their denied-lint sites may only fall (scores/lints.txt, absent = 0).
fn temporary_lints(ctx: &Ctx, out: &mut Out) {
    let allowed = ctx.ratchet("lints.txt");
    let banned = temporary_banned();
    for (name, spec, srcs) in ctx.crates() {
        if !spec.is_temporary() {
            continue;
        }
        let sites = temporary_sites(srcs, &banned);
        let max = allowed.get(name).copied().unwrap_or(0);
        if sites.len() > max {
            out.fail(format!("temporary `{name}` has {} denied-lint sites (scores/lints.txt allows {max})", sites.len()));
            for x in sites.iter().take(10) {
                out.fail(format!("  {x}"));
            }
        } else if !sites.is_empty() || max > 0 {
            out.note(format!("temporary `{name}`: {} denied-lint sites (ratchet {max})", sites.len()));
        }
    }
}

/// `disallowed-*` paths of a clippy.toml.
fn disallowed(t: &Toml, key: &str) -> Vec<String> {
    t.get(key)
        .map(Toml::as_arr)
        .unwrap_or_default()
        .iter()
        .filter_map(|v| match v {
            Toml::Str(s) => Some(s.clone()),
            t @ Toml::Table(_) => t.get("path").and_then(Toml::as_str).map(str::to_string),
            _ => None,
        })
        .collect()
}

/// Maths the facade must never call (PLAN §6.6 check 13).
const FACADE_BANNED: [&str; 8] = ["f64::sqrt", "f64::sin", "f64::cos", "f64::tan", "f64::atan2", "f64::powi", "f64::powf", "f64::hypot"];

/// Gate 13: `yack` denies `clippy::float_arithmetic` and its own clippy.toml bans the maths calls
/// (and repeats the root clippy.toml, since clippy reads only the nearest one).
pub fn facade(ctx: &Ctx, out: &mut Out) {
    let Some(p) = ctx.tree.package("yack") else {
        out.note("no `yack` crate yet");
        return;
    };
    if !root_denies(yack_lib(ctx), "clippy::float_arithmetic") {
        out.fail("crates/yack/src/lib.rs lacks `#![deny(clippy::float_arithmetic)]`");
    }
    let path = format!("{}/clippy.toml", p.dir);
    let local = match ctx.tree.file(&path).map(toml::parse) {
        None => {
            out.fail(format!("{path} is missing (the facade's maths ban)"));
            return;
        }
        Some(Err(e)) => {
            out.fail(format!("{path}: {e}"));
            return;
        }
        Some(Ok(t)) => t,
    };
    let methods = disallowed(&local, "disallowed-methods");
    for m in FACADE_BANNED {
        if !methods.iter().any(|x| x == m) {
            out.fail(format!("{path}: disallowed-methods lacks `{m}`"));
        }
    }
    if p.deps.iter().any(|d| d.internal && d.name == "yack-math") {
        for m in ["dot", "cross"] {
            if !methods.iter().any(|x| x.ends_with(&format!("::{m}"))) {
                out.fail(format!("{path}: disallowed-methods lacks yack-math's `{m}`"));
            }
        }
    }
    if let Some(Ok(root)) = ctx.tree.file("clippy.toml").map(toml::parse) {
        let types = disallowed(&local, "disallowed-types");
        for t in disallowed(&root, "disallowed-types") {
            if !types.contains(&t) {
                out.fail(format!("{path}: disallowed-types lacks root clippy.toml's `{t}`"));
            }
        }
        for m in disallowed(&root, "disallowed-methods") {
            if !methods.contains(&m) {
                out.fail(format!("{path}: disallowed-methods lacks root clippy.toml's `{m}`"));
            }
        }
    }
}
