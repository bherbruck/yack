// yack: a Rust tokenizer and source map for the gates: tokens, test-code masks, module files.
use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq)]
pub enum Tok {
    Ident(String),
    /// A numeric literal, as written.
    Num(String),
    /// A string, byte-string or char literal (content dropped).
    Str,
    /// A lifetime or label, without the quote.
    Lifetime(String),
    /// `::`, `->`, `=>` or a single punctuation character.
    Punct(String),
}

#[derive(Debug, Clone)]
pub struct Token {
    pub tok: Tok,
    pub line: usize,
}

impl Token {
    pub fn is_ident(&self, s: &str) -> bool {
        matches!(&self.tok, Tok::Ident(i) if i == s)
    }

    pub fn is_punct(&self, s: &str) -> bool {
        matches!(&self.tok, Tok::Punct(p) if p == s)
    }

    pub fn ident(&self) -> Option<&str> {
        match &self.tok {
            Tok::Ident(i) => Some(i),
            _ => None,
        }
    }
}

/// Tokenizes Rust source. Comments are dropped; unterminated constructs end the stream.
pub fn lex(src: &str) -> Vec<Token> {
    let c: Vec<char> = src.chars().collect();
    let at = |i: usize| c.get(i).copied();
    let mut out = Vec::new();
    let mut i = 0;
    let mut line = 1;
    let ident_start = |ch: char| ch.is_alphabetic() || ch == '_';
    let ident_char = |ch: char| ch.is_alphanumeric() || ch == '_';
    while let Some(ch) = at(i) {
        let start_line = line;
        if ch == '\n' {
            line += 1;
            i += 1;
        } else if ch.is_whitespace() {
            i += 1;
        } else if ch == '/' && at(i + 1) == Some('/') {
            while !matches!(at(i), None | Some('\n')) {
                i += 1;
            }
        } else if ch == '/' && at(i + 1) == Some('*') {
            let mut depth = 0usize;
            while let Some(x) = at(i) {
                if x == '/' && at(i + 1) == Some('*') {
                    depth += 1;
                    i += 2;
                } else if x == '*' && at(i + 1) == Some('/') {
                    depth = depth.saturating_sub(1);
                    i += 2;
                    if depth == 0 {
                        break;
                    }
                } else {
                    if x == '\n' {
                        line += 1;
                    }
                    i += 1;
                }
            }
        } else if let Some(skip) = raw_string_start(&c, i) {
            // r"..", r#".."#, br"..", cr".."
            let hashes = skip.1;
            i = skip.0;
            loop {
                match at(i) {
                    None => break,
                    Some('"') if (1..=hashes).all(|k| at(i + k) == Some('#')) => {
                        i += 1 + hashes;
                        break;
                    }
                    Some(x) => {
                        if x == '\n' {
                            line += 1;
                        }
                        i += 1;
                    }
                }
            }
            out.push(Token { tok: Tok::Str, line: start_line });
        } else if ch == '"' || ((ch == 'b' || ch == 'c') && at(i + 1) == Some('"')) {
            i += if ch == '"' { 1 } else { 2 };
            while let Some(x) = at(i) {
                i += 1;
                match x {
                    '\\' => {
                        if at(i) == Some('\n') {
                            line += 1;
                        }
                        i += 1;
                    }
                    '"' => break,
                    '\n' => line += 1,
                    _ => {}
                }
            }
            out.push(Token { tok: Tok::Str, line: start_line });
        } else if ch == '\'' || (ch == 'b' && at(i + 1) == Some('\'')) {
            let j = if ch == '\'' { i + 1 } else { i + 2 };
            if at(j) == Some('\\') {
                // escaped char literal
                i = j + 2;
                while !matches!(at(i), None | Some('\'') | Some('\n')) {
                    i += 1;
                }
                i += 1;
                out.push(Token { tok: Tok::Str, line });
            } else if at(j + 1) == Some('\'') {
                i = j + 2;
                out.push(Token { tok: Tok::Str, line });
            } else {
                i = j;
                while at(i).is_some_and(ident_char) {
                    i += 1;
                }
                out.push(Token { tok: Tok::Lifetime(c.get(j..i).unwrap_or_default().iter().collect()), line });
            }
        } else if ch.is_ascii_digit() {
            let s = i;
            if ch == '0' && matches!(at(i + 1), Some('x' | 'o' | 'b')) {
                i += 2;
                while at(i).is_some_and(|x| x.is_ascii_hexdigit() || x == '_') {
                    i += 1;
                }
            } else {
                while at(i).is_some_and(|x| x.is_ascii_digit() || x == '_') {
                    i += 1;
                }
                if at(i) == Some('.') && at(i + 1) != Some('.') && !at(i + 1).is_some_and(ident_start) {
                    i += 1;
                    while at(i).is_some_and(|x| x.is_ascii_digit() || x == '_') {
                        i += 1;
                    }
                }
                if matches!(at(i), Some('e' | 'E')) {
                    let k = if matches!(at(i + 1), Some('+' | '-')) { i + 2 } else { i + 1 };
                    if at(k).is_some_and(|x| x.is_ascii_digit()) {
                        i = k;
                        while at(i).is_some_and(|x| x.is_ascii_digit() || x == '_') {
                            i += 1;
                        }
                    }
                }
            }
            // suffix (f64, u32, ...)
            while at(i).is_some_and(ident_char) {
                i += 1;
            }
            out.push(Token { tok: Tok::Num(c.get(s..i).unwrap_or_default().iter().collect()), line });
        } else if ident_start(ch) {
            let s = i;
            // raw identifiers r#name
            if ch == 'r' && at(i + 1) == Some('#') && at(i + 2).is_some_and(ident_start) {
                i += 2;
            }
            while at(i).is_some_and(ident_char) {
                i += 1;
            }
            let word: String = c.get(s..i).unwrap_or_default().iter().collect();
            let word = word.strip_prefix("r#").map(str::to_string).unwrap_or(word);
            out.push(Token { tok: Tok::Ident(word), line });
        } else {
            let two: String = [ch, at(i + 1).unwrap_or(' ')].iter().collect();
            if two == "::" || two == "->" || two == "=>" {
                out.push(Token { tok: Tok::Punct(two), line });
                i += 2;
            } else {
                out.push(Token { tok: Tok::Punct(ch.to_string()), line });
                i += 1;
            }
        }
    }
    out
}

/// If a raw string starts at `i`, the index after its opening quote and its `#` count.
fn raw_string_start(c: &[char], i: usize) -> Option<(usize, usize)> {
    let at = |k: usize| c.get(k).copied();
    let mut k = i;
    if matches!(at(k), Some('b' | 'c')) {
        k += 1;
    }
    if at(k) != Some('r') {
        return None;
    }
    // must not be part of an identifier
    if i > 0 && at(i - 1).is_some_and(|x| x.is_alphanumeric() || x == '_') {
        return None;
    }
    k += 1;
    let mut hashes = 0;
    while at(k) == Some('#') {
        hashes += 1;
        k += 1;
    }
    (at(k) == Some('"')).then_some((k + 1, hashes))
}

/// Index of the token closing the bracket opened at `open` (`(`, `[` or `{`), or `toks.len()`.
pub fn matching(toks: &[Token], open: usize) -> usize {
    let mut depth = 0i64;
    for (k, t) in toks.iter().enumerate().skip(open) {
        if let Tok::Punct(p) = &t.tok {
            match p.as_str() {
                "(" | "[" | "{" => depth += 1,
                ")" | "]" | "}" => {
                    depth -= 1;
                    if depth == 0 {
                        return k;
                    }
                }
                _ => {}
            }
        }
    }
    toks.len()
}

/// The end (inclusive) of the item starting at `start`: its `;` or closing `}` at depth 0.
pub fn item_end(toks: &[Token], start: usize) -> usize {
    let mut k = start;
    while let Some(t) = toks.get(k) {
        if t.is_punct("(") || t.is_punct("[") {
            k = matching(toks, k) + 1;
            continue;
        }
        if t.is_punct("{") {
            return matching(toks, k);
        }
        if t.is_punct(";") {
            return k;
        }
        k += 1;
    }
    toks.len().saturating_sub(1)
}

/// An outer attribute `#[..]` at `k`: the token range inside the brackets and the index after it.
pub fn outer_attr(toks: &[Token], k: usize) -> Option<(usize, usize, usize)> {
    let t = toks.get(k)?;
    if !t.is_punct("#") || !toks.get(k + 1)?.is_punct("[") {
        return None;
    }
    let close = matching(toks, k + 1);
    Some((k + 2, close, close + 1))
}

/// Whether the attribute tokens `[a, b)` gate code to tests: `test`, `cfg(test)`, `cfg(all(test, ..))`.
pub fn is_test_attr(toks: &[Token], a: usize, b: usize) -> bool {
    let inner = toks.get(a..b).unwrap_or_default();
    match inner.first() {
        Some(t) if t.is_ident("test") && inner.len() == 1 => true,
        Some(t) if t.is_ident("cfg") => {
            inner.iter().any(|t| t.is_ident("test")) && !inner.iter().any(|t| t.is_ident("not") || t.is_ident("any"))
        }
        _ => false,
    }
}

/// One source file with its tokens and which of them are test-only code.
#[derive(Debug, Clone)]
pub struct Src {
    pub path: String,
    pub toks: Vec<Token>,
    /// `test[k]`: token `k` is inside test-only code.
    pub test: Vec<bool>,
    /// The whole file is test code (in `tests/`, behind `#[cfg(test)] mod`, or `#![cfg(test)]`).
    pub test_file: bool,
    /// The crate's top-level module this file belongs to (`None` for the crate root).
    pub top_module: Option<String>,
    /// Out-of-line `mod` declarations: (name, `#[path]` value, behind cfg(test)).
    pub mods: Vec<(String, Option<String>, bool)>,
}

impl Src {
    pub fn new(path: &str, text: &str) -> Src {
        let toks = lex(text);
        let mut test = vec![false; toks.len()];
        let mut mods = Vec::new();
        let mut whole = false;
        let mut k = 0;
        while k < toks.len() {
            // inner attribute #![cfg(test)]
            if toks.get(k).is_some_and(|t| t.is_punct("#"))
                && toks.get(k + 1).is_some_and(|t| t.is_punct("!"))
                && toks.get(k + 2).is_some_and(|t| t.is_punct("["))
            {
                let close = matching(&toks, k + 2);
                if is_test_attr(&toks, k + 3, close) {
                    whole = true;
                }
                k = close + 1;
                continue;
            }
            if let Some((a, b, mut next)) = outer_attr(&toks, k) {
                let mut is_test = is_test_attr(&toks, a, b);
                let mut path_attr = attr_path(&toks, a, b);
                while let Some((a2, b2, n2)) = outer_attr(&toks, next) {
                    is_test |= is_test_attr(&toks, a2, b2);
                    path_attr = path_attr.or_else(|| attr_path(&toks, a2, b2));
                    next = n2;
                }
                if let Some(m) = mod_decl(&toks, next) {
                    mods.push((m, path_attr, is_test));
                    let end = item_end(&toks, next);
                    if is_test {
                        for f in test.iter_mut().take(end + 1).skip(k) {
                            *f = true;
                        }
                    }
                    k = end + 1;
                } else if is_test {
                    let end = item_end(&toks, next);
                    for f in test.iter_mut().take(end + 1).skip(k) {
                        *f = true;
                    }
                    k = end + 1;
                } else {
                    k = next;
                }
                continue;
            }
            if let Some(m) = mod_decl(&toks, k) {
                mods.push((m, None, false));
            }
            k += 1;
        }
        Src { path: path.to_string(), toks, test, test_file: whole, top_module: None, mods }
    }

    /// Whether token `k` is ordinary (non-test) code.
    pub fn live(&self, k: usize) -> bool {
        !self.test_file && !self.test.get(k).copied().unwrap_or(true)
    }
}

/// `#[path = "x"]` → `x`.
fn attr_path(toks: &[Token], a: usize, b: usize) -> Option<String> {
    let first = toks.get(a)?;
    if b != a + 3 || !first.is_ident("path") || !toks.get(a + 1)?.is_punct("=") {
        return None;
    }
    // the string's content is dropped by the lexer; the gate re-reads it from the text when needed
    Some(String::new())
}

/// `[pub[(..)]] mod name ;` at `k` → `name` (out-of-line module declarations only).
fn mod_decl(toks: &[Token], k: usize) -> Option<String> {
    let mut j = k;
    if toks.get(j)?.is_ident("pub") {
        j += 1;
        if toks.get(j)?.is_punct("(") {
            j = matching(toks, j) + 1;
        }
    }
    if !toks.get(j)?.is_ident("mod") {
        return None;
    }
    let name = toks.get(j + 1)?.ident()?.to_string();
    toks.get(j + 2)?.is_punct(";").then_some(name)
}

/// The `.rs` files of one crate, with test-ness and top-level module resolved by following `mod`
/// declarations from the crate roots. Files no root reaches are kept as non-test code.
pub fn crate_sources(dir: &str, files: &BTreeMap<String, String>) -> Vec<Src> {
    let prefix = format!("{dir}/");
    let mut srcs: BTreeMap<String, Src> = files
        .iter()
        .filter(|(p, _)| p.starts_with(&prefix) && p.ends_with(".rs"))
        .map(|(p, t)| (p.clone(), Src::new(p, t)))
        .collect();
    for (p, s) in srcs.iter_mut() {
        let rel = p.strip_prefix(&prefix).unwrap_or(p);
        if ["tests/", "benches/", "examples/"].iter().any(|d| rel.starts_with(d)) {
            s.test_file = true;
        }
    }
    let src_dir = format!("{dir}/src");
    let mut roots: Vec<String> = vec![format!("{src_dir}/lib.rs"), format!("{src_dir}/main.rs")];
    roots.extend(srcs.keys().filter(|p| p.starts_with(&format!("{src_dir}/bin/"))).cloned());
    // (file, inherited test flag, top-level module)
    let mut queue: Vec<(String, bool, Option<String>)> = roots.into_iter().map(|r| (r, false, None)).collect();
    let mut seen = std::collections::BTreeSet::new();
    while let Some((file, test, top)) = queue.pop() {
        if !seen.insert(file.clone()) {
            continue;
        }
        let Some(s) = srcs.get_mut(&file) else { continue };
        s.test_file |= test;
        s.top_module = top.clone();
        let test_now = s.test_file;
        let text = files.get(&file).map(String::as_str).unwrap_or_default();
        let (fdir, stem) = match file.rsplit_once('/') {
            Some((d, f)) => (d.to_string(), f.trim_end_matches(".rs").to_string()),
            None => (String::new(), file.clone()),
        };
        let mod_rs = matches!(stem.as_str(), "lib" | "main" | "mod") || file.starts_with(&format!("{src_dir}/bin/"));
        let child_dir = if mod_rs { fdir.clone() } else { format!("{fdir}/{stem}") };
        let path_values = path_attr_values(text);
        let mut pv = path_values.into_iter();
        for (name, path_attr, t) in s.mods.clone() {
            let child_top = top.clone().or_else(|| Some(name.clone()));
            let cands = match path_attr {
                Some(_) => pv.next().map(|v| vec![format!("{fdir}/{v}")]).unwrap_or_default(),
                None => vec![format!("{child_dir}/{name}.rs"), format!("{child_dir}/{name}/mod.rs")],
            };
            for c in cands {
                queue.push((c, test_now || t, child_top.clone()));
            }
        }
    }
    srcs.into_values().collect()
}

/// The values of `#[path = ".."]` attributes, in order.
fn path_attr_values(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for line in text.lines() {
        let l = line.trim();
        if let Some(rest) = l.strip_prefix("#[path")
            && let Some(q) = rest.split('"').nth(1)
        {
            out.push(q.to_string());
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn idents(s: &str) -> Vec<String> {
        lex(s).into_iter().filter_map(|t| t.ident().map(str::to_string)).collect()
    }

    #[test]
    fn skips_comments_strings_and_handles_numbers() {
        let toks = lex("let x = 1e-7; // HashMap\n/* a /* b */ c */ let s = r#\"std::env\"#; 'a' 'b: 0.5f64 1..2 x.0");
        let nums: Vec<_> = toks.iter().filter_map(|t| if let Tok::Num(n) = &t.tok { Some(n.clone()) } else { None }).collect();
        assert_eq!(nums, ["1e-7", "0.5f64", "1", "2", "0"]);
        assert!(!idents("// std\n\"env\" b\"x\" 'x'").contains(&"env".to_string()));
        assert_eq!(toks.last().unwrap().line, 2);
    }

    #[test]
    fn marks_test_code() {
        let s = Src::new("a.rs", "fn a() {}\n#[cfg(test)]\nmod tests { fn b() { x.unwrap(); } }\nfn c() {}\n#[cfg(test)]\nmod t2;\n");
        let live: Vec<_> = (0..s.toks.len()).filter(|&k| s.live(k)).filter_map(|k| s.toks[k].ident()).collect();
        assert_eq!(live, ["fn", "a", "fn", "c"]);
        assert_eq!(s.mods, vec![("t2".to_string(), None, true)]);
    }
}
