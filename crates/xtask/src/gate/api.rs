// yack: the `yack` public surface (source level, stable Rust), `cargo xtask api`, and gate 4.
use super::lex::{Src, Tok, Token, is_test_attr, item_end, matching, outer_attr};
use super::{Ctx, Out};
use std::collections::BTreeSet;

pub const API_FILE: &str = "crates/yack/api.txt";
const API_HEADER: &str = "# yack public surface: `cargo xtask api --bless` regenerates this file. One item per line.";

fn text(t: &Token) -> String {
    match &t.tok {
        Tok::Ident(s) | Tok::Num(s) | Tok::Punct(s) => s.clone(),
        Tok::Str => "\"..\"".to_string(),
        Tok::Lifetime(l) => format!("'{l}"),
    }
}

/// Tokens joined the way rustfmt would roughly print them.
pub fn norm(toks: &[Token]) -> String {
    let mut out = String::new();
    let mut prev: Option<&Token> = None;
    // for each open `{`: whether it is a `path::{..}` group (printed tight)
    let mut braces: Vec<bool> = Vec::new();
    for t in toks {
        let s = text(t);
        let group_open = s == "{" && prev.is_some_and(|p| p.is_punct("::"));
        let group_close = s == "}" && braces.last() == Some(&true);
        let tight = match prev {
            None => true,
            Some(p) => {
                group_close
                    || matches!(s.as_str(), "," | ";" | ")" | "]" | "." | "?" | ":" | "::" | ">")
                    || (matches!(s.as_str(), "(" | "<" | "[" | "!") && (p.ident().is_some() || p.is_punct(">")))
                    || ["(", "[", "<", "&", "::", "#", "'"].iter().any(|x| p.is_punct(x))
                    || (p.is_punct("{") && braces.last() == Some(&true))
            }
        };
        if s == "{" {
            braces.push(group_open);
        } else if s == "}" {
            braces.pop();
        }
        if !tight {
            out.push(' ');
        }
        out.push_str(&s);
        prev = Some(t);
    }
    out
}

/// What the surface walk finds.
#[derive(Default)]
pub struct Surface {
    pub lines: BTreeSet<String>,
    /// `#[doc(hidden)] pub` items.
    pub hidden: Vec<String>,
    /// `pub use yack_*` leaves: (location, crate, item).
    pub reexports: Vec<(String, String, String)>,
}

#[derive(Clone, Copy, PartialEq)]
enum Vis {
    Private,
    Restricted,
    Pub,
}

/// The first `{`, `(`, `=` or `;` (whichever of `stops`) at bracket depth 0 in `[a, b]`.
fn first_at_depth0(toks: &[Token], a: usize, b: usize, stops: &[&str]) -> usize {
    let mut k = a;
    while k <= b {
        let Some(t) = toks.get(k) else { break };
        if stops.iter().any(|s| t.is_punct(s)) {
            return k;
        }
        if t.is_punct("(") || t.is_punct("[") {
            k = matching(toks, k) + 1;
            continue;
        }
        k += 1;
    }
    b + 1
}

/// Splits `[a, b)` at commas outside every bracket (angle brackets included).
fn split_commas(toks: &[Token], a: usize, b: usize) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut depth = 0i64;
    let mut start = a;
    for k in a..b {
        let Some(t) = toks.get(k) else { break };
        match &t.tok {
            Tok::Punct(p) if matches!(p.as_str(), "(" | "[" | "{" | "<") => depth += 1,
            Tok::Punct(p) if matches!(p.as_str(), ")" | "]" | "}" | ">") => depth -= 1,
            Tok::Punct(p) if p == "," && depth == 0 => {
                out.push((start, k));
                start = k + 1;
            }
            _ => {}
        }
    }
    if start < b {
        out.push((start, b));
    }
    out
}

/// Drops `#[..]` attributes from a token range.
fn strip_attrs(toks: &[Token], a: usize, b: usize) -> Vec<Token> {
    let mut out = Vec::new();
    let mut k = a;
    while k < b {
        if let Some((_, _, next)) = outer_attr(toks, k) {
            k = next;
            continue;
        }
        if let Some(t) = toks.get(k) {
            out.push(t.clone());
        }
        k += 1;
    }
    out
}

fn slice(toks: &[Token], a: usize, b: usize) -> &[Token] {
    toks.get(a..b.max(a)).unwrap_or_default()
}

struct Walk<'a> {
    srcs: &'a [Src],
    out: Surface,
    seen: BTreeSet<String>,
    /// `pub` items of private modules that a crate-local `pub use` makes public: (module, name).
    reexported: BTreeSet<(String, String)>,
    /// Crate-local `pub use` leaves met on the way: (module the path names, item).
    local: BTreeSet<(String, String)>,
}

impl Walk<'_> {
    fn file(&mut self, path: &str, modpath: &str, public: bool) {
        if !self.seen.insert(path.to_string()) {
            return;
        }
        let srcs = self.srcs;
        let Some(s) = srcs.iter().find(|s| s.path == path) else { return };
        if s.test_file {
            return;
        }
        let (dir, stem) = path.rsplit_once('/').unwrap_or(("", path));
        let stem = stem.trim_end_matches(".rs");
        let child_dir = if matches!(stem, "lib" | "main" | "mod") { dir.to_string() } else { format!("{dir}/{stem}") };
        self.items(s, 0, s.toks.len(), modpath, public, &child_dir, false);
    }

    #[allow(clippy::too_many_arguments)]
    fn items(&mut self, s: &Src, a: usize, b: usize, mp: &str, public: bool, child_dir: &str, in_trait: bool) {
        let toks = &s.toks;
        let mut k = a;
        while k < b {
            let Some(t) = toks.get(k) else { break };
            if t.is_punct(";") {
                k += 1;
                continue;
            }
            if t.is_punct("#") && toks.get(k + 1).is_some_and(|x| x.is_punct("!")) {
                k = matching(toks, k + 2) + 1;
                continue;
            }
            // attributes
            let mut test = false;
            let mut hidden = false;
            let mut export = false;
            while let Some((x, y, next)) = outer_attr(toks, k) {
                test |= is_test_attr(toks, x, y);
                let inner = slice(toks, x, y);
                hidden |= norm(inner) == "doc(hidden)";
                export |= norm(inner) == "macro_export";
                k = next;
            }
            let item_start = k;
            let mut vis = Vis::Private;
            if toks.get(k).is_some_and(|x| x.is_ident("pub")) {
                if toks.get(k + 1).is_some_and(|x| x.is_punct("(")) {
                    vis = Vis::Restricted;
                    k = matching(toks, k + 1) + 1;
                } else {
                    vis = Vis::Pub;
                    k += 1;
                }
            }
            if in_trait {
                vis = Vis::Pub;
            }
            // qualifiers
            while toks.get(k).is_some_and(|x| {
                x.is_ident("async")
                    || x.is_ident("unsafe")
                    || x.is_ident("default")
                    || (x.is_ident("const") && toks.get(k + 1).is_some_and(|n| n.is_ident("fn") || n.is_ident("unsafe") || n.is_ident("async")))
                    || (x.is_ident("extern") && toks.get(k + 1).is_some_and(|n| matches!(n.tok, Tok::Str) || n.is_ident("fn")))
            }) {
                k += if toks.get(k).is_some_and(|x| x.is_ident("extern")) && toks.get(k + 1).is_some_and(|n| matches!(n.tok, Tok::Str)) { 2 } else { 1 };
            }
            let kw = toks.get(k).and_then(Token::ident).unwrap_or_default().to_string();
            let last = b.saturating_sub(1);
            let end = if matches!(kw.as_str(), "use" | "type" | "static") {
                first_at_depth0(toks, k, last, &[";"]).min(last)
            } else {
                item_end(toks, k).min(last).max(k)
            };
            let name = toks.get(k + 1).and_then(Token::ident).unwrap_or_default();
            let shown = vis == Vis::Pub && (public || self.reexported.contains(&(mp.to_string(), name.to_string())));
            let at = format!("{}:{}", s.path, toks.get(item_start).map(|t| t.line).unwrap_or_default());
            if test {
                k = end + 1;
                continue;
            }
            if hidden && vis == Vis::Pub {
                self.out.hidden.push(format!("{at}: `#[doc(hidden)] pub` item"));
            }
            match kw.as_str() {
                "mod" => {
                    let name = toks.get(k + 1).and_then(Token::ident).unwrap_or_default().to_string();
                    let sub = format!("{mp}::{name}");
                    if toks.get(k + 2).is_some_and(|x| x.is_punct(";")) {
                        let f1 = format!("{child_dir}/{name}.rs");
                        let f2 = format!("{child_dir}/{name}/mod.rs");
                        let f = if self.srcs.iter().any(|s| s.path == f1) { f1 } else { f2 };
                        self.file(&f, &sub, shown);
                    } else if toks.get(k + 2).is_some_and(|x| x.is_punct("{")) {
                        let close = matching(toks, k + 2);
                        self.items(s, k + 3, close, &sub, shown, &format!("{child_dir}/{name}"), false);
                    }
                }
                "impl" => self.impl_block(s, item_start, k, end, mp),
                "fn" if shown => {
                    let sig = first_at_depth0(toks, item_start, end, &["{", ";"]);
                    self.line(mp, &strip_attrs(toks, item_start, sig));
                }
                "const" if shown => {
                    let eq = first_at_depth0(toks, item_start, end, &["=", ";"]);
                    self.line(mp, &strip_attrs(toks, item_start, eq));
                }
                "type" | "static" if shown => self.line(mp, &strip_attrs(toks, item_start, end)),
                "use" if vis == Vis::Pub => {
                    if shown {
                        self.line(mp, &strip_attrs(toks, item_start, end));
                    }
                    self.reexports(toks, k + 1, end, &at);
                    self.local_uses(toks, k + 1, end, mp);
                }
                "struct" | "union" if shown => self.structure(toks, item_start, end, mp),
                "enum" if shown => {
                    let open = first_at_depth0(toks, item_start, end, &["{"]);
                    self.line(mp, &strip_attrs(toks, item_start, open));
                    let close = matching(toks, open);
                    let name = toks.get(k + 1).map(text).unwrap_or_default();
                    for (x, y) in split_commas(toks, open + 1, close) {
                        let v = strip_attrs(toks, x, y);
                        if !v.is_empty() {
                            self.lines_push(format!("{mp}: enum {name} variant {}", norm(&v)));
                        }
                    }
                }
                "trait" if shown => {
                    let open = first_at_depth0(toks, item_start, end, &["{"]);
                    self.line(mp, &strip_attrs(toks, item_start, open));
                    let close = matching(toks, open);
                    let name = toks.get(k + 1).map(text).unwrap_or_default();
                    self.items(s, open + 1, close, &format!("{mp}: trait {name} item"), true, child_dir, true);
                }
                "macro_rules" if export => {
                    let name = toks.get(k + 2).map(text).unwrap_or_default();
                    self.lines_push(format!("macro {name}!"));
                }
                _ => {}
            }
            k = end + 1;
        }
    }

    fn line(&mut self, mp: &str, toks: &[Token]) {
        self.lines_push(format!("{mp}: {}", norm(toks)));
    }

    fn lines_push(&mut self, l: String) {
        self.out.lines.insert(l);
    }

    fn structure(&mut self, toks: &[Token], a: usize, end: usize, mp: &str) {
        let open = first_at_depth0(toks, a, end, &["{", "(", ";"]);
        let head = strip_attrs(toks, a, open);
        let Some(o) = toks.get(open) else {
            self.line(mp, &head);
            return;
        };
        if o.is_punct(";") {
            self.line(mp, &head);
            return;
        }
        let close = matching(toks, open);
        let mut fields = Vec::new();
        let mut private = false;
        for (x, y) in split_commas(toks, open + 1, close) {
            let f = strip_attrs(toks, x, y);
            let pubf = f.first().is_some_and(|t| t.is_ident("pub")) && !f.get(1).is_some_and(|t| t.is_punct("("));
            if pubf {
                fields.push(norm(&f));
            } else if !f.is_empty() {
                private = true;
            }
        }
        if private {
            fields.push("..".to_string());
        }
        let (l, r) = if o.is_punct("{") { (" { ", " }") } else { ("(", ")") };
        let body = if fields.is_empty() { String::new() } else { format!("{l}{}{r}", fields.join(", ")) };
        // where-clauses after a tuple struct's fields
        let tail = if o.is_punct("(") { norm(&strip_attrs(toks, close + 1, end)) } else { String::new() };
        let tail = if tail.is_empty() || tail == ";" { String::new() } else { format!(" {}", tail.trim_end_matches(';').trim()) };
        self.lines_push(format!("{mp}: {}{body}{tail}", norm(&head)));
    }

    fn impl_block(&mut self, s: &Src, a: usize, kw: usize, end: usize, mp: &str) {
        let toks = &s.toks;
        let open = first_at_depth0(toks, kw, end, &["{"]);
        let head = strip_attrs(toks, a, open);
        let mut depth = 0i64;
        let is_trait = toks.get(kw + 1..open).unwrap_or_default().iter().any(|t| {
            if t.is_punct("<") {
                depth += 1;
            } else if t.is_punct(">") {
                depth -= 1;
            }
            depth == 0 && t.is_ident("for")
        });
        if is_trait {
            self.line(mp, &head);
            return;
        }
        let close = matching(toks, open);
        let mut inner = Walk { srcs: self.srcs, out: Surface::default(), seen: BTreeSet::new(), reexported: BTreeSet::new(), local: BTreeSet::new() };
        let prefix = format!("{mp}: {} {{", norm(&head));
        inner.items(s, open + 1, close, &prefix, true, "", false);
        for l in inner.out.lines {
            // "<prefix>: pub fn .." → "<prefix> pub fn .. }"
            let l = l.replacen(&format!("{prefix}: "), &format!("{prefix} "), 1);
            self.lines_push(format!("{l} }}"));
        }
        self.out.hidden.extend(inner.out.hidden);
    }

    /// Records the leaves of a crate-local `pub use <path>;` in module `mp` (`crate::`, `self::` or
    /// a child module's path; one level of braces), as (the module they are in, item).
    fn local_uses(&mut self, toks: &[Token], a: usize, end: usize, mp: &str) {
        let body = slice(toks, a, end);
        let mut module: Vec<String> = Vec::new();
        let mut i = 0;
        while let Some(t) = body.get(i) {
            let Some(id) = t.ident() else { break };
            if !body.get(i + 1).is_some_and(|n| n.is_punct("::")) {
                break;
            }
            module.push(id.to_string());
            i += 2;
        }
        let root = mp.split("::").next().unwrap_or(mp).to_string();
        let base = match module.first().map(String::as_str) {
            None => return,
            Some(first) if first.starts_with("yack_") || first == "std" || first == "core" || first == "super" => return,
            Some("crate") => std::iter::once(root).chain(module.iter().skip(1).cloned()).collect::<Vec<_>>().join("::"),
            Some("self") => std::iter::once(mp.to_string()).chain(module.iter().skip(1).cloned()).collect::<Vec<_>>().join("::"),
            Some(_) => std::iter::once(mp.to_string()).chain(module.iter().cloned()).collect::<Vec<_>>().join("::"),
        };
        let rest = body.get(i..).unwrap_or_default();
        for (j, t) in rest.iter().enumerate() {
            let Some(name) = t.ident() else { continue };
            let next = rest.get(j + 1);
            let leaf = next.is_none_or(|n| n.is_punct(",") || n.is_punct("}") || n.is_punct(";") || n.is_ident("as"));
            let renamed_to = j > 0 && rest.get(j - 1).is_some_and(|p| p.is_ident("as"));
            if leaf && !renamed_to {
                self.local.insert((base.clone(), name.to_string()));
            }
        }
    }

    /// Records the leaves of `use <path>;` when the path starts in a `yack_*` crate.
    fn reexports(&mut self, toks: &[Token], a: usize, end: usize, at: &str) {
        let body = slice(toks, a, end);
        let mut it = body.iter().skip_while(|t| t.is_punct("::"));
        let Some(first) = it.next().and_then(Token::ident) else { return };
        if !first.starts_with("yack_") {
            return;
        }
        for (i, t) in body.iter().enumerate() {
            let Some(name) = t.ident() else { continue };
            let next = body.get(i + 1);
            let leaf = next.is_none_or(|n| n.is_punct(",") || n.is_punct("}") || n.is_punct(";") || n.is_ident("as"));
            let renamed_to = i > 0 && body.get(i - 1).is_some_and(|p| p.is_ident("as"));
            if leaf && !renamed_to && name != first {
                self.out.reexports.push((at.to_string(), first.to_string(), name.to_string()));
            } else if leaf && !renamed_to && name == first && i == 0 {
                self.out.reexports.push((at.to_string(), first.to_string(), "self".to_string()));
            }
        }
    }
}

/// Walks `yack` from `src/lib.rs`.
pub fn surface(ctx: &Ctx) -> Option<Surface> {
    let p = ctx.tree.package("yack")?;
    let srcs = ctx.srcs.get("yack")?;
    let lib = format!("{}/src/lib.rs", p.dir);
    // first the crate-local re-exports, then the surface with the private modules' items they
    // make public (their enums' variants and structs' fields included)
    let mut first = Walk { srcs, out: Surface::default(), seen: BTreeSet::new(), reexported: BTreeSet::new(), local: BTreeSet::new() };
    first.file(&lib, "yack", true);
    let mut w = Walk { srcs, out: Surface::default(), seen: BTreeSet::new(), reexported: first.local, local: BTreeSet::new() };
    w.file(&lib, "yack", true);
    Some(w.out)
}

pub fn render(s: &Surface) -> String {
    let mut out = format!("{API_HEADER}\n");
    for l in &s.lines {
        out.push_str(l);
        out.push('\n');
    }
    out
}

fn body_lines(text: &str) -> BTreeSet<String> {
    text.lines().map(str::trim_end).filter(|l| !l.is_empty() && !l.starts_with('#')).map(str::to_string).collect()
}

/// (missing from api.txt, only in api.txt)
pub fn diff(ctx: &Ctx, s: &Surface) -> Option<(Vec<String>, Vec<String>)> {
    let want = body_lines(ctx.tree.file(API_FILE)?);
    let have: BTreeSet<String> = s.lines.iter().cloned().collect();
    Some((have.difference(&want).cloned().collect(), want.difference(&have).cloned().collect()))
}

/// Gate 4: no glob `pub use` anywhere; in `yack` no `#[doc(hidden)] pub`, no `pub use yack_*`
/// beyond layers.toml's `reexport` list, and a surface equal to crates/yack/api.txt.
pub fn no_leaks(ctx: &Ctx, out: &mut Out) {
    for (name, _, srcs) in ctx.crates() {
        for s in srcs {
            for (k, t) in s.toks.iter().enumerate() {
                if !s.live(k) || !t.is_ident("pub") || !s.toks.get(k + 1).is_some_and(|x| x.is_ident("use")) {
                    continue;
                }
                let end = item_end(&s.toks, k);
                if slice(&s.toks, k, end).iter().any(|x| x.is_punct("*")) {
                    out.fail(format!("{}:{}: glob `pub use` in `{name}`", s.path, t.line));
                }
            }
        }
    }
    let Some(surf) = surface(ctx) else {
        out.note("no `yack` crate yet");
        return;
    };
    for h in &surf.hidden {
        out.fail(h.clone());
    }
    for (at, krate, item) in &surf.reexports {
        if !ctx.layers.reexport.contains(item) {
            out.fail(format!("{at}: `pub use {krate}::{item}` is not in layers.toml's reexport list"));
        }
    }
    match diff(ctx, &surf) {
        None => out.fail(format!("{API_FILE} is missing (run `cargo xtask api --bless`)")),
        Some((added, removed)) => {
            for l in added.iter().take(20) {
                out.fail(format!("surface has, api.txt lacks: {l}"));
            }
            for l in removed.iter().take(20) {
                out.fail(format!("api.txt has, surface lacks: {l}"));
            }
            if !added.is_empty() || !removed.is_empty() {
                out.fail("the yack surface differs from api.txt (`cargo xtask api` shows it; `--bless` accepts it)");
            }
        }
    }
}

/// `cargo xtask api [--bless]`.
pub fn main(args: &[String]) -> Result<bool, String> {
    let root = crate::meta::root();
    let ctx = Ctx::new(crate::meta::load(&root)?)?;
    let surf = surface(&ctx).ok_or("no `yack` crate")?;
    if args.iter().any(|a| a == "--bless") {
        let p = root.join(API_FILE);
        std::fs::write(&p, render(&surf)).map_err(|e| format!("write {}: {e}", p.display()))?;
        println!("blessed {API_FILE} ({} items)", surf.lines.len());
        return Ok(true);
    }
    if args.iter().any(|a| a == "--print") {
        print!("{}", render(&surf));
        return Ok(true);
    }
    match diff(&ctx, &surf) {
        None => {
            println!("error: {API_FILE} is missing; `cargo xtask api --bless` writes it");
            Ok(false)
        }
        Some((added, removed)) => {
            for l in &added {
                println!("+ {l}");
            }
            for l in &removed {
                println!("- {l}");
            }
            let clean = added.is_empty() && removed.is_empty();
            println!("{}", if clean { "api: clean" } else { "error: api differs from api.txt" });
            Ok(clean)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meta::{Package, Tree};

    fn ctx_of(lib: &str) -> Ctx {
        let mut tree = Tree::default();
        tree.files.insert("layers.toml".into(), "[reexport]\nitems = [\"Point3\"]\n".into());
        tree.files.insert("crates/yack/src/lib.rs".into(), lib.into());
        tree.packages.push(Package { name: "yack".into(), dir: "crates/yack".into(), publishable: true, license: None, deps: vec![] });
        Ctx::new(tree).unwrap()
    }

    #[test]
    fn surface_lines() {
        let ctx = ctx_of(
            "// yack: x\npub const VERSION: &str = env!(\"X\");\npub struct S { pub a: Vec<u8>, b: u8 }\n\
             pub enum E { A, B(u8) }\nimpl S { pub fn new(x: &'static str) -> Result<S, E> { todo!() } fn hid() {} }\n\
             mod private { pub fn f() {} pub enum P { X, Y } }\npub use private::P;\npub use yack_math::{Point3, Vec3 as V};\n#[cfg(test)]\nmod tests { pub fn t() {} }\n",
        );
        let s = surface(&ctx).unwrap();
        let lines: Vec<_> = s.lines.iter().cloned().collect();
        assert_eq!(
            lines,
            [
                "yack: impl S { pub fn new(x: &'static str) -> Result<S, E> }",
                "yack: pub const VERSION: &str",
                "yack: pub enum E",
                "yack: pub struct S { pub a: Vec<u8>, .. }",
                "yack: pub use yack_math::{Point3, Vec3 as V}",
                "yack: pub use private::P",
                "yack::private: pub enum P",
                "yack::private: enum P variant X",
                "yack::private: enum P variant Y",
                "yack: enum E variant A",
                "yack: enum E variant B(u8)",
            ]
            .iter()
            .map(|s| s.to_string())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>()
        );
        let leaves: Vec<_> = s.reexports.iter().map(|r| r.2.clone()).collect();
        assert_eq!(leaves, ["Point3", "Vec3"]);
    }
}
