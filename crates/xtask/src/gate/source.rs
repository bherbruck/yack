// yack: gates 7 (epsilons) and 8 (hidden state), read from the token streams.
use super::lex::{Src, Tok};
use super::{Ctx, Out};
use std::collections::BTreeMap;

/// Crates whose job is tolerances and round-off, where small literals belong.
const EPS_HOMES: [&str; 2] = ["yack-tol", "yack-numeric"];
const EPS_LIMIT: f64 = 1e-2;

/// The value of a float literal, or `None` for integers.
fn float_value(text: &str) -> Option<f64> {
    let t = text.replace('_', "");
    if t.starts_with("0x") || t.starts_with("0o") || t.starts_with("0b") {
        return None;
    }
    let (body, suffixed) = match t.strip_suffix("f64").or_else(|| t.strip_suffix("f32")) {
        Some(b) => (b.to_string(), true),
        None => (t.clone(), false),
    };
    if !suffixed && !body.contains(['.', 'e', 'E']) {
        return None;
    }
    if body.chars().any(|c| c.is_ascii_alphabetic() && c != 'e' && c != 'E') {
        return None;
    }
    body.parse().ok()
}

fn line_of<'a>(ctx: &'a Ctx, s: &Src, line: usize) -> &'a str {
    ctx.tree.file(&s.path).and_then(|t| t.lines().nth(line.saturating_sub(1))).unwrap_or_default()
}

/// Unmarked small float literals in the non-test code of shipping crates: crate → locations.
fn eps_sites(ctx: &Ctx) -> BTreeMap<String, Vec<String>> {
    let mut out: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (name, spec, srcs) in ctx.crates() {
        if !spec.ships() || EPS_HOMES.contains(&name) {
            continue;
        }
        let sites = out.entry(name.to_string()).or_default();
        for s in srcs {
            for (k, t) in s.toks.iter().enumerate() {
                let Tok::Num(text) = &t.tok else { continue };
                if !s.live(k) {
                    continue;
                }
                let Some(v) = float_value(text) else { continue };
                if v > 0.0 && v < EPS_LIMIT && !line_of(ctx, s, t.line).contains("// tol:") {
                    sites.push(format!("{}:{}: `{text}`", s.path, t.line));
                }
            }
        }
    }
    out
}

pub fn eps_counts(ctx: &Ctx) -> BTreeMap<String, usize> {
    eps_sites(ctx).into_iter().filter(|(_, v)| !v.is_empty()).map(|(k, v)| (k, v.len())).collect()
}

/// Gate 7: a float literal below 1e-2 outside tol/numeric needs `// tol: <reason>`; the unmarked
/// count per crate may only fall (scores/eps.txt, absent = 0).
pub fn epsilons(ctx: &Ctx, out: &mut Out) {
    let allowed = ctx.ratchet("eps.txt");
    let mut total = 0;
    for (krate, sites) in eps_sites(ctx) {
        let max = allowed.get(&krate).copied().unwrap_or(0);
        total += sites.len();
        if sites.len() > max {
            out.fail(format!(
                "`{krate}` has {} float literals below 1e-2 without `// tol: <reason>` (scores/eps.txt allows {max})",
                sites.len()
            ));
            for s in sites.iter().take(10) {
                out.fail(format!("  {s}"));
            }
        } else if sites.len() < max {
            out.note(format!("`{krate}`: {} unmarked small literals, below the ratchet {max} (run `cargo xtask gate --bless`)", sites.len()));
        }
    }
    out.note(format!("unmarked small float literals: {total}"));
}

const HIDDEN_STATICS: [&str; 8] = ["OnceLock", "Mutex", "RwLock", "LazyLock", "OnceCell", "LazyCell", "RefCell", "Cell"];

/// One gate-8 finding: (crate, location + message, is an env read).
struct Hidden {
    krate: String,
    msg: String,
    env: bool,
}

/// Hidden-state sites in kernel and temporary crates, plus `catch_unwind` outside `yack`.
fn hidden_sites(ctx: &Ctx) -> (Vec<Hidden>, Vec<String>) {
    let mut sites = Vec::new();
    let mut unwind = Vec::new();
    for (name, spec, srcs) in ctx.crates() {
        let kernel = spec.is_kernel() || spec.is_temporary();
        for s in srcs {
            let tok = |k: usize| s.toks.get(k);
            for (k, t) in s.toks.iter().enumerate() {
                if !s.live(k) {
                    continue;
                }
                let at = format!("{}:{}", s.path, t.line);
                if name != "yack" && t.is_ident("catch_unwind") {
                    unwind.push(format!("{at}: `catch_unwind` outside `yack`"));
                }
                if !kernel {
                    continue;
                }
                let mut push = |msg: String, env: bool| sites.push(Hidden { krate: name.to_string(), msg: format!("{at}: {msg}"), env });
                let next = |n: usize, p: &str| tok(k + n).is_some_and(|x| x.is_punct(p) || x.is_ident(p));
                let prev_std = k >= 2
                    && tok(k - 1).is_some_and(|x| x.is_punct("::"))
                    && tok(k - 2).is_some_and(|x| x.is_ident("std"));
                if t.is_ident("env") && (prev_std || next(1, "::")) {
                    push(format!("`std::env` in `{name}`"), true);
                }
                if t.is_ident("thread_local") && next(1, "!") {
                    push(format!("`thread_local!` in `{name}`"), false);
                }
                if t.is_ident("static") && next(1, "mut") {
                    push(format!("`static mut` in `{name}`"), false);
                }
                if t.is_ident("static") && next(2, ":") {
                    let ty = s.toks.iter().skip(k + 3).take_while(|x| !x.is_punct("=") && !x.is_punct(";"));
                    if let Some(bad) = ty
                        .filter_map(|x| x.ident())
                        .find(|i| HIDDEN_STATICS.contains(i) || i.starts_with("Atomic"))
                    {
                        push(format!("`static` of `{bad}` (global state) in `{name}`"), false);
                    }
                }
                if t.is_ident("Instant") || t.is_ident("SystemTime") {
                    push(format!("wall-clock `{}` in `{name}`", t.ident().unwrap_or_default()), false);
                }
            }
        }
    }
    (sites, unwind)
}

/// Whether a site is ratcheted (temporary crates) rather than fatal. yack-cadrs's env reads are
/// always fatal: its `CADRS_*` switches are fixed at their defaults (PLAN §3.2).
fn ratcheted(ctx: &Ctx, h: &Hidden) -> bool {
    ctx.spec(&h.krate).is_some_and(|s| s.is_temporary()) && !(h.env && h.krate == "yack-cadrs")
}

/// Ratcheted hidden-state counts per temporary crate (scores/hidden.txt).
pub fn hidden_counts(ctx: &Ctx) -> BTreeMap<String, usize> {
    let mut m = BTreeMap::new();
    for h in hidden_sites(ctx).0.iter().filter(|h| ratcheted(ctx, h)) {
        *m.entry(h.krate.clone()).or_insert(0) += 1;
    }
    m
}

/// Gate 8: no env, thread-locals, mutable or lazily-initialised statics or clocks in kernel
/// crates; only `yack` calls `catch_unwind`. Temporary crates are ratcheted per crate in
/// scores/hidden.txt (may only fall), except that yack-cadrs may read no env at all.
pub fn hidden_state(ctx: &Ctx, out: &mut Out) {
    let (sites, unwind) = hidden_sites(ctx);
    for u in unwind {
        out.fail(u);
    }
    for h in sites.iter().filter(|h| !ratcheted(ctx, h)) {
        out.fail(h.msg.clone());
    }
    let allowed = ctx.ratchet("hidden.txt");
    for (krate, n) in hidden_counts(ctx) {
        let max = allowed.get(&krate).copied().unwrap_or(0);
        if n > max {
            out.fail(format!("temporary `{krate}` has {n} hidden-state sites (scores/hidden.txt allows {max})"));
            for h in sites.iter().filter(|h| h.krate == krate).take(10) {
                out.fail(format!("  {}", h.msg));
            }
        } else {
            out.note(format!("temporary `{krate}`: {n} hidden-state sites (ratchet {max})"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::float_value;

    #[test]
    fn float_values() {
        assert_eq!(float_value("1e-7"), Some(1e-7));
        assert_eq!(float_value("0.000_5"), Some(5e-4));
        assert_eq!(float_value("1e-9_f64"), Some(1e-9));
        assert_eq!(float_value("2f64"), Some(2.0));
        assert_eq!(float_value("10"), None);
        assert_eq!(float_value("0x1e"), None);
        assert_eq!(float_value("1u32"), None);
    }
}
