// yack: a small std-only JSON reader, enough for `cargo metadata` output (PLAN §3.1, §6.6).

/// A parsed JSON value. Numbers keep their source text; objects keep their key order.
#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Num(String),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    /// The member `key` of an object, or `None`.
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(members) => members.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_arr(&self) -> &[Json] {
        match self {
            Json::Arr(items) => items,
            _ => &[],
        }
    }
}

/// Parses one JSON document.
pub fn parse(text: &str) -> Result<Json, String> {
    let mut p = Parser { chars: text.chars().collect(), pos: 0 };
    p.ws();
    let v = p.value(0)?;
    p.ws();
    if p.pos != p.chars.len() {
        return Err(format!("json: trailing input at offset {}", p.pos));
    }
    Ok(v)
}

struct Parser {
    chars: Vec<char>,
    pos: usize,
}

impl Parser {
    fn peek(&self) -> Option<char> {
        self.chars.get(self.pos).copied()
    }

    fn bump(&mut self) -> Option<char> {
        let c = self.peek();
        if c.is_some() {
            self.pos += 1;
        }
        c
    }

    fn ws(&mut self) {
        while matches!(self.peek(), Some(' ' | '\t' | '\n' | '\r')) {
            self.pos += 1;
        }
    }

    fn expect(&mut self, want: char) -> Result<(), String> {
        match self.bump() {
            Some(c) if c == want => Ok(()),
            other => Err(format!("json: expected `{want}` at offset {}, found {other:?}", self.pos)),
        }
    }

    fn word(&mut self, w: &str, v: Json) -> Result<Json, String> {
        for want in w.chars() {
            self.expect(want)?;
        }
        Ok(v)
    }

    fn value(&mut self, depth: usize) -> Result<Json, String> {
        if depth > 256 {
            return Err("json: nesting too deep".to_string());
        }
        match self.peek() {
            Some('n') => self.word("null", Json::Null),
            Some('t') => self.word("true", Json::Bool(true)),
            Some('f') => self.word("false", Json::Bool(false)),
            Some('"') => self.string().map(Json::Str),
            Some('[') => {
                self.pos += 1;
                let mut items = Vec::new();
                self.ws();
                if self.peek() == Some(']') {
                    self.pos += 1;
                    return Ok(Json::Arr(items));
                }
                loop {
                    self.ws();
                    items.push(self.value(depth + 1)?);
                    self.ws();
                    match self.bump() {
                        Some(',') => {}
                        Some(']') => return Ok(Json::Arr(items)),
                        other => return Err(format!("json: bad array separator {other:?}")),
                    }
                }
            }
            Some('{') => {
                self.pos += 1;
                let mut members = Vec::new();
                self.ws();
                if self.peek() == Some('}') {
                    self.pos += 1;
                    return Ok(Json::Obj(members));
                }
                loop {
                    self.ws();
                    let k = self.string()?;
                    self.ws();
                    self.expect(':')?;
                    self.ws();
                    let v = self.value(depth + 1)?;
                    members.push((k, v));
                    self.ws();
                    match self.bump() {
                        Some(',') => {}
                        Some('}') => return Ok(Json::Obj(members)),
                        other => return Err(format!("json: bad object separator {other:?}")),
                    }
                }
            }
            Some(c) if c == '-' || c.is_ascii_digit() => {
                let start = self.pos;
                while matches!(self.peek(), Some(c) if c.is_ascii_digit() || matches!(c, '-' | '+' | '.' | 'e' | 'E'))
                {
                    self.pos += 1;
                }
                Ok(Json::Num(self.chars.get(start..self.pos).unwrap_or_default().iter().collect()))
            }
            other => Err(format!("json: unexpected {other:?} at offset {}", self.pos)),
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.expect('"')?;
        let mut out = String::new();
        loop {
            match self.bump() {
                None => return Err("json: unterminated string".to_string()),
                Some('"') => return Ok(out),
                Some('\\') => match self.bump() {
                    Some('n') => out.push('\n'),
                    Some('t') => out.push('\t'),
                    Some('r') => out.push('\r'),
                    Some('b') => out.push('\u{8}'),
                    Some('f') => out.push('\u{c}'),
                    Some('u') => {
                        let hi = self.hex4()?;
                        let code = if (0xD800..0xDC00).contains(&hi) {
                            self.expect('\\')?;
                            self.expect('u')?;
                            let lo = self.hex4()?;
                            0x10000 + ((hi - 0xD800) << 10) + (lo.wrapping_sub(0xDC00) & 0x3FF)
                        } else {
                            hi
                        };
                        out.push(char::from_u32(code).unwrap_or('\u{FFFD}'));
                    }
                    Some(c) => out.push(c),
                    None => return Err("json: unterminated escape".to_string()),
                },
                Some(c) => out.push(c),
            }
        }
    }

    fn hex4(&mut self) -> Result<u32, String> {
        let mut v = 0u32;
        for _ in 0..4 {
            let d = self.bump().and_then(|c| c.to_digit(16)).ok_or("json: bad \\u escape")?;
            v = v * 16 + d;
        }
        Ok(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_metadata_shape() {
        let v = parse(
            r#"{"packages":[{"name":"yack","publish":null,"license":"MIT OR Apache-2.0",
            "dependencies":[{"name":"yack-base","source":null,"kind":null,"path":"/x"}]}],
            "n": -1.5e3, "s": "a\"bé😀", "t": true, "e": []}"#,
        )
        .unwrap();
        let pkg = &v.get("packages").unwrap().as_arr()[0];
        assert_eq!(pkg.get("name").unwrap().as_str(), Some("yack"));
        assert_eq!(pkg.get("publish"), Some(&Json::Null));
        assert_eq!(v.get("n"), Some(&Json::Num("-1.5e3".into())));
        assert_eq!(v.get("s").unwrap().as_str(), Some("a\"b\u{e9}\u{1F600}"));
        assert!(v.get("e").unwrap().as_arr().is_empty());
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse("{\"a\":}").is_err());
        assert!(parse("[1,2").is_err());
        assert!(parse("{} x").is_err());
    }
}
