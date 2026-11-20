// yack: gate 14: brep-bench case programs are short, literal readings of the public API (PLAN §4.4, R3).
use super::lex::{Src, Tok};
use super::{Ctx, Out};

const CASES_DIR: &str = "crates/yack-bench/src/cases/";
const README: &str = "crates/yack-bench/README.md";
const MAX_LOC: usize = 10;

/// A case program's counted lines: not blank and not a comment (the harness's own `loc`).
pub fn loc(text: &str) -> usize {
    text.lines().filter(|l| !l.trim().is_empty() && !l.trim().starts_with("//")).count()
}

/// The README's exceptions table: case id → documented line count.
fn exceptions(readme: &str) -> std::collections::BTreeMap<String, usize> {
    readme
        .lines()
        .filter_map(|l| {
            let cells: Vec<&str> = l.trim().strip_prefix('|')?.split('|').map(str::trim).collect();
            let n = cells.get(1)?.parse().ok()?;
            Some((cells.first()?.trim_matches('`').to_string(), n))
        })
        .collect()
}

/// The ways out of the public API a case might take, from its tokens.
fn private_paths(s: &Src) -> Vec<String> {
    let mut out = Vec::new();
    let toks = &s.toks;
    for (k, t) in toks.iter().enumerate() {
        let at = format!("{}:{}", s.path, t.line);
        if let Tok::Ident(i) = &t.tok {
            if i.starts_with("yack_") {
                out.push(format!("{at}: `{i}` names an internal crate"));
            }
            if i == "use" {
                let root = toks.iter().skip(k + 1).find(|x| !x.is_punct("::")).and_then(|x| x.ident());
                if root != Some("yack") {
                    out.push(format!("{at}: `use {}` (case programs import only `yack::`)", root.unwrap_or("?")));
                }
            }
            if (i == "include" || i == "include_str") && toks.get(k + 1).is_some_and(|x| x.is_punct("!")) {
                out.push(format!("{at}: `{i}!` in a case program"));
            }
            if matches!(i.as_str(), "unsafe" | "extern") {
                out.push(format!("{at}: `{i}` in a case program"));
            }
            if i == "path" && k >= 2 && toks.get(k - 1).is_some_and(|x| x.is_punct("[")) {
                out.push(format!("{at}: `#[path]` in a case program"));
            }
        }
    }
    out
}

/// Gate 14: every case program is at most 10 counted lines unless the README's exceptions table
/// lists it (with at least its length), and reaches yack only through `yack::` public paths.
pub fn case_programs(ctx: &Ctx, out: &mut Out) {
    let files: Vec<(&String, &String)> =
        ctx.tree.files.iter().filter(|(p, _)| p.starts_with(CASES_DIR) && p.ends_with(".rs")).collect();
    if files.is_empty() {
        out.note("no brep-bench case programs");
        return;
    }
    let allowed = exceptions(ctx.tree.file(README).unwrap_or_default());
    let mut long = 0;
    for (path, text) in &files {
        let id = text
            .lines()
            .next()
            .and_then(|l| l.strip_prefix("// yack: brep-bench case "))
            .map(|s| s.trim().to_string());
        let Some(id) = id else {
            out.fail(format!("{path}: first line is not `// yack: brep-bench case <id>`"));
            continue;
        };
        let n = loc(text);
        if n > MAX_LOC {
            long += 1;
            match allowed.get(&id) {
                None => out.fail(format!("{path}: case `{id}` is {n} lines (> {MAX_LOC}) and not in {README}'s exceptions table")),
                Some(&max) if n > max => out.fail(format!("{path}: case `{id}` is {n} lines; {README} documents {max}")),
                Some(_) => {}
            }
        }
        let src = Src::new(path, text);
        for p in private_paths(&src) {
            out.fail(p);
        }
    }
    out.note(format!("{} case programs, {long} over {MAX_LOC} lines (documented)", files.len()));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loc_and_table() {
        assert_eq!(loc("// yack: x\n//! doc\nuse yack::prelude::*;\n\nfn a() {}\n"), 2);
        let t = exceptions("| Case | Lines | Why |\n|---|---|---|\n| boolean/golf-ball | 19 | data |\n");
        assert_eq!(t.get("boolean/golf-ball"), Some(&19));
        assert_eq!(t.len(), 1);
    }

    #[test]
    fn private_paths_found() {
        let s = Src::new("c.rs", "use yack::prelude::*;\nuse std::f64::consts::PI;\nfn f() { yack_geom::x(); }\n");
        let p = private_paths(&s);
        assert_eq!(p.len(), 2, "{p:?}");
    }
}
