// yack: a small std-only TOML reader for layers.toml, Cargo.toml, Cargo.lock and clippy.toml.
//
// Supports what those files use: tables, arrays of tables, dotted and quoted keys, basic and
// literal strings, integers, booleans, (multi-line) arrays and inline tables. Anything else is
// kept as its raw text in `Toml::Raw`.
use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq)]
pub enum Toml {
    Str(String),
    Int(i64),
    Bool(bool),
    Raw(String),
    Arr(Vec<Toml>),
    Table(BTreeMap<String, Toml>),
}

impl Toml {
    pub fn get(&self, key: &str) -> Option<&Toml> {
        match self {
            Toml::Table(t) => t.get(key),
            _ => None,
        }
    }

    /// Follows a dotted path of keys.
    pub fn path(&self, keys: &[&str]) -> Option<&Toml> {
        keys.iter().try_fold(self, |v, k| v.get(k))
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Toml::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Toml::Bool(b) => Some(*b),
            _ => None,
        }
    }

    pub fn as_arr(&self) -> &[Toml] {
        match self {
            Toml::Arr(a) => a,
            _ => &[],
        }
    }

    pub fn as_table(&self) -> Option<&BTreeMap<String, Toml>> {
        match self {
            Toml::Table(t) => Some(t),
            _ => None,
        }
    }

    /// The strings of an array of strings (other items are skipped).
    pub fn strings(&self) -> Vec<String> {
        self.as_arr().iter().filter_map(|v| v.as_str().map(str::to_string)).collect()
    }
}

/// Parses a TOML document into its root table.
pub fn parse(text: &str) -> Result<Toml, String> {
    let mut p = Parser { chars: text.chars().collect(), pos: 0, line: 1 };
    let mut root = BTreeMap::new();
    // The path of the current table header, and whether it is an array-of-tables element.
    let mut current: Vec<String> = Vec::new();
    loop {
        p.skip_ws_comments_newlines();
        let Some(c) = p.peek() else { break };
        if c == '[' {
            let array = p.peek_at(1) == Some('[');
            p.pos += if array { 2 } else { 1 };
            p.skip_inline_ws();
            let keys = p.key_path()?;
            p.skip_inline_ws();
            p.expect(']')?;
            if array {
                p.expect(']')?;
                push_array_table(&mut root, &keys).map_err(|e| p.err(&e))?;
            } else {
                table_at(&mut root, &keys).map_err(|e| p.err(&e))?;
            }
            current = keys;
            p.end_of_line()?;
        } else {
            let keys = p.key_path()?;
            p.skip_inline_ws();
            p.expect('=')?;
            p.skip_inline_ws();
            let v = p.value(0)?;
            let table = table_at(&mut root, &current).map_err(|e| p.err(&e))?;
            insert_dotted(table, &keys, v).map_err(|e| p.err(&e))?;
            p.end_of_line()?;
        }
    }
    Ok(Toml::Table(root))
}

/// The table at `keys` (creating tables on the way); an array of tables resolves to its last element.
fn table_at<'a>(
    root: &'a mut BTreeMap<String, Toml>,
    keys: &[String],
) -> Result<&'a mut BTreeMap<String, Toml>, String> {
    let mut t = root;
    for k in keys {
        let entry = t.entry(k.clone()).or_insert_with(|| Toml::Table(BTreeMap::new()));
        t = match entry {
            Toml::Table(inner) => inner,
            Toml::Arr(items) => match items.last_mut() {
                Some(Toml::Table(inner)) => inner,
                _ => return Err(format!("`{k}` is not a table")),
            },
            _ => return Err(format!("`{k}` is not a table")),
        };
    }
    Ok(t)
}

fn push_array_table(root: &mut BTreeMap<String, Toml>, keys: &[String]) -> Result<(), String> {
    let Some((last, parent)) = keys.split_last() else { return Err("empty table name".into()) };
    let t = table_at(root, parent)?;
    let entry = t.entry(last.clone()).or_insert_with(|| Toml::Arr(Vec::new()));
    match entry {
        Toml::Arr(items) => {
            items.push(Toml::Table(BTreeMap::new()));
            Ok(())
        }
        _ => Err(format!("`{last}` is not an array of tables")),
    }
}

fn insert_dotted(table: &mut BTreeMap<String, Toml>, keys: &[String], v: Toml) -> Result<(), String> {
    let Some((last, parent)) = keys.split_last() else { return Err("empty key".into()) };
    let t = table_at(table, parent)?;
    if t.insert(last.clone(), v).is_some() {
        return Err(format!("duplicate key `{last}`"));
    }
    Ok(())
}

struct Parser {
    chars: Vec<char>,
    pos: usize,
    line: usize,
}

impl Parser {
    fn err(&self, msg: &str) -> String {
        format!("toml line {}: {msg}", self.line)
    }

    fn peek(&self) -> Option<char> {
        self.chars.get(self.pos).copied()
    }

    fn peek_at(&self, n: usize) -> Option<char> {
        self.chars.get(self.pos + n).copied()
    }

    fn bump(&mut self) -> Option<char> {
        let c = self.peek();
        if let Some(ch) = c {
            self.pos += 1;
            if ch == '\n' {
                self.line += 1;
            }
        }
        c
    }

    fn expect(&mut self, want: char) -> Result<(), String> {
        match self.bump() {
            Some(c) if c == want => Ok(()),
            other => Err(self.err(&format!("expected `{want}`, found {other:?}"))),
        }
    }

    fn skip_inline_ws(&mut self) {
        while matches!(self.peek(), Some(' ' | '\t')) {
            self.pos += 1;
        }
    }

    fn skip_comment(&mut self) {
        if self.peek() == Some('#') {
            while !matches!(self.peek(), None | Some('\n')) {
                self.pos += 1;
            }
        }
    }

    fn skip_ws_comments_newlines(&mut self) {
        loop {
            match self.peek() {
                Some(' ' | '\t' | '\r') => self.pos += 1,
                Some('\n') => {
                    self.bump();
                }
                Some('#') => self.skip_comment(),
                _ => return,
            }
        }
    }

    fn end_of_line(&mut self) -> Result<(), String> {
        self.skip_inline_ws();
        self.skip_comment();
        if self.peek() == Some('\r') {
            self.pos += 1;
        }
        match self.peek() {
            None => Ok(()),
            Some('\n') => {
                self.bump();
                Ok(())
            }
            Some(c) => Err(self.err(&format!("unexpected `{c}` after value"))),
        }
    }

    fn key_path(&mut self) -> Result<Vec<String>, String> {
        let mut keys = vec![self.key()?];
        loop {
            self.skip_inline_ws();
            if self.peek() != Some('.') {
                return Ok(keys);
            }
            self.pos += 1;
            self.skip_inline_ws();
            keys.push(self.key()?);
        }
    }

    fn key(&mut self) -> Result<String, String> {
        match self.peek() {
            Some('"') => self.basic_string(),
            Some('\'') => self.literal_string(),
            _ => {
                let start = self.pos;
                while matches!(self.peek(), Some(c) if c.is_ascii_alphanumeric() || c == '_' || c == '-') {
                    self.pos += 1;
                }
                if start == self.pos {
                    return Err(self.err("expected a key"));
                }
                Ok(self.chars.get(start..self.pos).unwrap_or_default().iter().collect())
            }
        }
    }

    fn value(&mut self, depth: usize) -> Result<Toml, String> {
        if depth > 64 {
            return Err(self.err("nesting too deep"));
        }
        match self.peek() {
            Some('"') => self.basic_string().map(Toml::Str),
            Some('\'') => self.literal_string().map(Toml::Str),
            Some('[') => {
                self.pos += 1;
                let mut items = Vec::new();
                loop {
                    self.skip_ws_comments_newlines();
                    if self.peek() == Some(']') {
                        self.pos += 1;
                        return Ok(Toml::Arr(items));
                    }
                    items.push(self.value(depth + 1)?);
                    self.skip_ws_comments_newlines();
                    match self.peek() {
                        Some(',') => self.pos += 1,
                        Some(']') => {}
                        other => return Err(self.err(&format!("bad array separator {other:?}"))),
                    }
                }
            }
            Some('{') => {
                self.pos += 1;
                let mut t = BTreeMap::new();
                loop {
                    self.skip_inline_ws();
                    if self.peek() == Some('}') {
                        self.pos += 1;
                        return Ok(Toml::Table(t));
                    }
                    let keys = self.key_path()?;
                    self.skip_inline_ws();
                    self.expect('=')?;
                    self.skip_inline_ws();
                    let v = self.value(depth + 1)?;
                    insert_dotted(&mut t, &keys, v).map_err(|e| self.err(&e))?;
                    self.skip_inline_ws();
                    match self.peek() {
                        Some(',') => self.pos += 1,
                        Some('}') => {}
                        other => return Err(self.err(&format!("bad inline-table separator {other:?}"))),
                    }
                }
            }
            _ => {
                let start = self.pos;
                while !matches!(self.peek(), None | Some(',' | ']' | '}' | '\n' | '#' | '\r')) {
                    self.pos += 1;
                }
                let raw: String = self.chars.get(start..self.pos).unwrap_or_default().iter().collect();
                let raw = raw.trim().to_string();
                Ok(match raw.as_str() {
                    "true" => Toml::Bool(true),
                    "false" => Toml::Bool(false),
                    "" => return Err(self.err("expected a value")),
                    _ => raw.replace('_', "").parse::<i64>().map(Toml::Int).unwrap_or(Toml::Raw(raw)),
                })
            }
        }
    }

    fn basic_string(&mut self) -> Result<String, String> {
        if self.peek_at(1) == Some('"') && self.peek_at(2) == Some('"') {
            return self.multiline('"');
        }
        self.expect('"')?;
        let mut out = String::new();
        loop {
            match self.bump() {
                None | Some('\n') => return Err(self.err("unterminated string")),
                Some('"') => return Ok(out),
                Some('\\') => self.escape(&mut out)?,
                Some(c) => out.push(c),
            }
        }
    }

    fn escape(&mut self, out: &mut String) -> Result<(), String> {
        match self.bump() {
            Some('n') => out.push('\n'),
            Some('t') => out.push('\t'),
            Some('r') => out.push('\r'),
            Some('"') => out.push('"'),
            Some('\\') => out.push('\\'),
            Some(u @ ('u' | 'U')) => {
                let n = if u == 'u' { 4 } else { 8 };
                let mut v = 0u32;
                for _ in 0..n {
                    let d = self.bump().and_then(|c| c.to_digit(16)).ok_or_else(|| self.err("bad unicode escape"))?;
                    v = v.wrapping_mul(16).wrapping_add(d);
                }
                out.push(char::from_u32(v).unwrap_or('\u{FFFD}'));
            }
            Some('\n') => {
                // line-ending backslash in a multi-line string
                while matches!(self.peek(), Some(' ' | '\t' | '\n' | '\r')) {
                    self.bump();
                }
            }
            other => return Err(self.err(&format!("bad escape {other:?}"))),
        }
        Ok(())
    }

    fn literal_string(&mut self) -> Result<String, String> {
        if self.peek_at(1) == Some('\'') && self.peek_at(2) == Some('\'') {
            return self.multiline('\'');
        }
        self.expect('\'')?;
        let mut out = String::new();
        loop {
            match self.bump() {
                None | Some('\n') => return Err(self.err("unterminated string")),
                Some('\'') => return Ok(out),
                Some(c) => out.push(c),
            }
        }
    }

    fn multiline(&mut self, q: char) -> Result<String, String> {
        self.pos += 3;
        if self.peek() == Some('\n') {
            self.bump();
        }
        let mut out = String::new();
        loop {
            if self.peek() == Some(q) && self.peek_at(1) == Some(q) && self.peek_at(2) == Some(q) {
                self.pos += 3;
                return Ok(out);
            }
            match self.bump() {
                None => return Err(self.err("unterminated multi-line string")),
                Some('\\') if q == '"' => self.escape(&mut out)?,
                Some(c) => out.push(c),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_layers_and_lock_shapes() {
        let t = parse(
            r#"# comment
[reexport]
items = ["A", "B",
  # inner comment
  "C"]

[crate.yack-base]
layer = 0
kind = "kernel" # trailing
deps = []
version.workspace = true

[[package]]
name = "a"
dependencies = ["b 1.0 (registry+x)"]

[[package]]
name = 'b'
x = { path = "f64::sqrt", reason = "r" }
"#,
        )
        .unwrap();
        assert_eq!(t.path(&["reexport", "items"]).unwrap().strings(), ["A", "B", "C"]);
        assert_eq!(t.path(&["crate", "yack-base", "layer"]), Some(&Toml::Int(0)));
        assert_eq!(t.path(&["crate", "yack-base", "version", "workspace"]), Some(&Toml::Bool(true)));
        let pk = t.get("package").unwrap().as_arr();
        assert_eq!(pk.len(), 2);
        assert_eq!(pk[1].path(&["x", "path"]).unwrap().as_str(), Some("f64::sqrt"));
    }

    #[test]
    fn rejects_duplicates_and_junk() {
        assert!(parse("a = 1\na = 2\n").is_err());
        assert!(parse("a = \"x\" y\n").is_err());
    }
}
