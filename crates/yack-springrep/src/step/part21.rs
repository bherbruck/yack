// yack: The STEP exchange structure (ISO 10303-21): a parser of the header and the data section into
// Adapted from springrep@a03e8a1 src/step/part21.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! The STEP exchange structure (ISO 10303-21): a parser of the header and the data section into
//! entity instances, and the string encoding the writer uses.

use std::collections::HashMap;

use crate::{Error, Result};

/// One parameter of an entity instance.
#[derive(Clone, Debug, PartialEq)]
pub(super) enum Param {
    Ref(u64),
    Int(i64),
    Real(f64),
    Str(String),
    Enum(String),
    List(Vec<Param>),
    /// `$`: no value.
    Unset,
    /// `*`: the value is derived from other attributes.
    Derived,
    /// A value tagged with its type, such as `LENGTH_MEASURE(0.01)`.
    Typed(String, Box<Param>),
}

impl Param {
    pub(super) fn as_ref(&self) -> Option<u64> {
        match self {
            Param::Ref(r) => Some(*r),
            _ => None,
        }
    }

    pub(super) fn as_f64(&self) -> Option<f64> {
        match self {
            Param::Real(x) => Some(*x),
            Param::Int(i) => Some(*i as f64),
            Param::Typed(_, p) => p.as_f64(),
            _ => None,
        }
    }

    pub(super) fn as_i64(&self) -> Option<i64> {
        match self {
            Param::Int(i) => Some(*i),
            Param::Real(x) if x.fract() == 0.0 => Some(*x as i64),
            Param::Typed(_, p) => p.as_i64(),
            _ => None,
        }
    }

    pub(super) fn as_list(&self) -> Option<&[Param]> {
        match self {
            Param::List(l) => Some(l),
            _ => None,
        }
    }

    pub(super) fn as_str(&self) -> Option<&str> {
        match self {
            Param::Str(s) => Some(s),
            _ => None,
        }
    }

    /// A logical or boolean: `.T.` or `.F.`; `.U.` and anything else is `None`.
    pub(super) fn as_bool(&self) -> Option<bool> {
        match self {
            Param::Enum(e) if e == "T" => Some(true),
            Param::Enum(e) if e == "F" => Some(false),
            _ => None,
        }
    }

    pub(super) fn as_enum(&self) -> Option<&str> {
        match self {
            Param::Enum(e) => Some(e),
            _ => None,
        }
    }
}

/// An entity instance: one `(TYPE, parameters)` part for a simple instance, several for a complex
/// one written as `(A(..) B(..))`.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Entity {
    pub parts: Vec<(String, Vec<Param>)>,
}

impl Entity {
    /// The type of a simple instance; a complex one has no single type.
    pub(super) fn kind(&self) -> &str {
        if self.parts.len() == 1 {
            &self.parts[0].0
        } else {
            ""
        }
    }

    /// The parameters of the part named `kind`, of a simple or a complex instance.
    pub(super) fn part(&self, kind: &str) -> Option<&[Param]> {
        self.parts
            .iter()
            .find(|(k, _)| k == kind)
            .map(|(_, p)| p.as_slice())
    }

    pub(super) fn has(&self, kind: &str) -> bool {
        self.parts.iter().any(|(k, _)| k == kind)
    }
}

/// A parsed exchange file: its header entities and its data section, by instance id.
#[derive(Debug, Default)]
pub(super) struct File {
    pub header: Vec<Entity>,
    pub ids: Vec<u64>,
    pub entities: Vec<Entity>,
    index: HashMap<u64, usize>,
}

impl File {
    pub(super) fn get(&self, id: u64) -> Option<&Entity> {
        self.index.get(&id).map(|&i| &self.entities[i])
    }

    /// The `originating_system` field of the header's `FILE_NAME`.
    pub(super) fn originating_system(&self) -> Option<&str> {
        let f = self.header.iter().find_map(|e| e.part("FILE_NAME"))?;
        f.get(5)?.as_str()
    }
}

fn err<T>(msg: impl Into<String>, at: usize) -> Result<T> {
    Err(Error::InvalidInput(format!(
        "STEP: {} at byte {at}",
        msg.into()
    )))
}

/// How deeply parameter lists may nest; real files use a handful of levels, and the parser
/// recurses once per level.
const MAX_DEPTH: usize = 64;

struct Lexer<'a> {
    s: &'a [u8],
    i: usize,
    depth: usize,
}

impl<'a> Lexer<'a> {
    fn skip_ws(&mut self) {
        loop {
            while self.i < self.s.len() && self.s[self.i].is_ascii_whitespace() {
                self.i += 1;
            }
            if self.s.get(self.i..).is_some_and(|r| r.starts_with(b"/*")) {
                match find(&self.s[self.i + 2..], b"*/") {
                    Some(k) => self.i += k + 4,
                    None => self.i = self.s.len(),
                }
            } else {
                return;
            }
        }
    }

    fn peek(&mut self) -> Option<u8> {
        self.skip_ws();
        self.s.get(self.i).copied()
    }

    fn expect(&mut self, c: u8) -> Result<()> {
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            err(format!("expected '{}'", c as char), self.i)
        }
    }

    fn keyword(&mut self) -> Result<String> {
        self.skip_ws();
        let start = self.i;
        while self.i < self.s.len()
            && (self.s[self.i].is_ascii_alphanumeric()
                || self.s[self.i] == b'_'
                || self.s[self.i] == b'-')
        {
            self.i += 1;
        }
        if self.i == start {
            return err("expected a keyword", start);
        }
        Ok(String::from_utf8_lossy(&self.s[start..self.i]).to_ascii_uppercase())
    }

    fn number(&mut self) -> Result<Param> {
        let start = self.i;
        let mut real = false;
        while self.i < self.s.len() {
            match self.s[self.i] {
                b'0'..=b'9' | b'+' | b'-' => {}
                b'.' | b'E' | b'e' => real = true,
                _ => break,
            }
            self.i += 1;
        }
        let text = std::str::from_utf8(&self.s[start..self.i]).unwrap_or("");
        if !real && let Ok(v) = text.parse::<i64>() {
            return Ok(Param::Int(v));
        }
        // `1.` and `1.E3` are valid STEP reals that Rust's parser also accepts.
        match text.parse::<f64>() {
            Ok(v) => Ok(Param::Real(v)),
            Err(_) => err(format!("bad number '{text}'"), start),
        }
    }

    fn string(&mut self) -> Result<String> {
        self.i += 1;
        let mut raw = Vec::new();
        loop {
            match self.s.get(self.i) {
                None => return err("unterminated string", self.i),
                Some(b'\'') if self.s.get(self.i + 1) == Some(&b'\'') => {
                    raw.push(b'\'');
                    self.i += 2;
                }
                Some(b'\'') => {
                    self.i += 1;
                    break;
                }
                Some(b'\n') | Some(b'\r') => self.i += 1,
                Some(&c) => {
                    raw.push(c);
                    self.i += 1;
                }
            }
        }
        Ok(decode_string(&raw))
    }

    fn param(&mut self) -> Result<Param> {
        match self.peek() {
            Some(b'#') => {
                self.i += 1;
                match self.number()? {
                    Param::Int(v) if v >= 0 => Ok(Param::Ref(v as u64)),
                    _ => err("bad instance reference", self.i),
                }
            }
            Some(b'\'') => Ok(Param::Str(self.string()?)),
            Some(b'.') => {
                self.i += 1;
                let start = self.i;
                while self.i < self.s.len() && self.s[self.i] != b'.' {
                    self.i += 1;
                }
                if self.i >= self.s.len() {
                    return err("unterminated enumeration", start);
                }
                let e = String::from_utf8_lossy(&self.s[start..self.i]).to_ascii_uppercase();
                self.i += 1;
                Ok(Param::Enum(e))
            }
            Some(b'$') => {
                self.i += 1;
                Ok(Param::Unset)
            }
            Some(b'*') => {
                self.i += 1;
                Ok(Param::Derived)
            }
            Some(b'(') => Ok(Param::List(self.list()?)),
            Some(b'"') => {
                self.i += 1;
                let start = self.i;
                while self.i < self.s.len() && self.s[self.i] != b'"' {
                    self.i += 1;
                }
                if self.i >= self.s.len() {
                    return err("unterminated binary", start);
                }
                let v = String::from_utf8_lossy(&self.s[start..self.i]).into_owned();
                self.i += 1;
                Ok(Param::Str(v))
            }
            Some(c) if c.is_ascii_digit() || c == b'-' || c == b'+' => self.number(),
            Some(c) if c.is_ascii_alphabetic() || c == b'_' => {
                let k = self.keyword()?;
                let mut l = self.list()?;
                let inner = if l.len() == 1 {
                    l.pop().unwrap_or(Param::Unset)
                } else {
                    Param::List(l)
                };
                Ok(Param::Typed(k, Box::new(inner)))
            }
            _ => err("unexpected character in parameters", self.i),
        }
    }

    fn list(&mut self) -> Result<Vec<Param>> {
        if self.depth >= MAX_DEPTH {
            return err("parameter lists nest too deeply", self.i);
        }
        self.depth += 1;
        let out = self.list_items();
        self.depth -= 1;
        out
    }

    fn list_items(&mut self) -> Result<Vec<Param>> {
        self.expect(b'(')?;
        let mut out = Vec::new();
        if self.peek() == Some(b')') {
            self.i += 1;
            return Ok(out);
        }
        loop {
            out.push(self.param()?);
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b')') => {
                    self.i += 1;
                    return Ok(out);
                }
                _ => return err("expected ',' or ')'", self.i),
            }
        }
    }

    /// `TYPE(..)` or a complex `(A(..) B(..))`, up to the terminating `;`.
    fn entity(&mut self) -> Result<Entity> {
        let mut parts = Vec::new();
        if self.peek() == Some(b'(') {
            self.i += 1;
            while self.peek() != Some(b')') {
                let k = self.keyword()?;
                parts.push((k, self.list()?));
            }
            self.i += 1;
        } else {
            let k = self.keyword()?;
            parts.push((k, self.list()?));
        }
        self.expect(b';')?;
        Ok(Entity { parts })
    }
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// Decodes the control directives of ISO 10303-21 strings that carry text: `\X2\…\X0\` and
/// `\X4\…\X0\` (UTF-16 and UTF-32 hex), `\X\hh` (one ISO 8859-1 byte), and `\\`.
fn decode_string(raw: &[u8]) -> String {
    let hex =
        |b: &[u8]| -> Option<u32> { u32::from_str_radix(std::str::from_utf8(b).ok()?, 16).ok() };
    let mut out = String::new();
    let mut i = 0;
    while i < raw.len() {
        let rest = &raw[i..];
        if rest.starts_with(b"\\\\") {
            out.push('\\');
            i += 2;
            continue;
        }
        let wide = if rest.starts_with(b"\\X2\\") {
            Some(4)
        } else if rest.starts_with(b"\\X4\\") {
            Some(8)
        } else {
            None
        };
        if let (Some(width), Some(end)) = (wide, find(&rest[4.min(rest.len())..], b"\\X0\\")) {
            let units: Vec<u32> = rest[4..4 + end].chunks(width).filter_map(hex).collect();
            if width == 4 {
                let u16s: Vec<u16> = units.iter().map(|&u| u as u16).collect();
                out.push_str(&String::from_utf16_lossy(&u16s));
            } else {
                out.extend(units.iter().filter_map(|&u| char::from_u32(u)));
            }
            i += 4 + end + 4;
            continue;
        }
        if rest.starts_with(b"\\X\\")
            && rest.len() >= 5
            && let Some(c) = hex(&rest[3..5]).and_then(char::from_u32)
        {
            out.push(c);
            i += 5;
            continue;
        }
        let n = rest[1..]
            .iter()
            .position(|&c| c == b'\\')
            .map_or(rest.len(), |k| k + 1);
        out.push_str(&String::from_utf8_lossy(&rest[..n]));
        i += n;
    }
    out
}

/// Encodes `s` as an ISO 10303-21 string literal: quotes doubled, backslashes escaped, and
/// anything outside printable ASCII as `\X2\` hex.
pub(super) fn encode_string(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\'' => out.push_str("''"),
            '\\' => out.push_str("\\\\"),
            ' '..='~' => out.push(c),
            _ => {
                let mut buf = [0u16; 2];
                out.push_str("\\X2\\");
                for u in c.encode_utf16(&mut buf) {
                    out.push_str(&format!("{u:04X}"));
                }
                out.push_str("\\X0\\");
            }
        }
    }
    out.push('\'');
    out
}

/// Parses an exchange file.
pub(super) fn parse(text: &[u8]) -> Result<File> {
    let mut lx = Lexer {
        s: text,
        i: 0,
        depth: 0,
    };
    let mut file = File::default();
    let k = lx.keyword()?;
    if k != "ISO-10303-21" {
        return err("not an ISO-10303-21 file", 0);
    }
    lx.expect(b';')?;
    let mut in_data = false;
    loop {
        match lx.peek() {
            None => break,
            Some(b'#') if in_data => {
                lx.i += 1;
                let id = match lx.number()? {
                    Param::Int(v) if v >= 0 => v as u64,
                    _ => return err("bad instance id", lx.i),
                };
                lx.expect(b'=')?;
                let e = lx.entity()?;
                file.index.insert(id, file.entities.len());
                file.ids.push(id);
                file.entities.push(e);
            }
            Some(_) => {
                let k = lx.keyword()?;
                match k.as_str() {
                    "HEADER" => lx.expect(b';')?,
                    "DATA" => {
                        if lx.peek() == Some(b'(') {
                            lx.list()?;
                        }
                        lx.expect(b';')?;
                        in_data = true;
                    }
                    "ENDSEC" => {
                        lx.expect(b';')?;
                        in_data = false;
                    }
                    "END-ISO-10303-21" => break,
                    _ if !in_data => {
                        let params = lx.list()?;
                        lx.expect(b';')?;
                        file.header.push(Entity {
                            parts: vec![(k, params)],
                        });
                    }
                    _ => return err(format!("unexpected '{k}' in the data section"), lx.i),
                }
            }
        }
    }
    Ok(file)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_simple_complex_and_typed_instances() {
        let text = b"ISO-10303-21;\nHEADER;\nFILE_NAME('a.stp','t',(''),(''),'p',\n\
            'sys /* x */','');\nENDSEC;\nDATA;\n\
            /* c */ #1=CARTESIAN_POINT('',(0.,1.5E1,-2));\n\
            #2=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n\
            #3=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#2,'it''s','\\X2\\00E9\\X0\\');\n\
            ENDSEC;\nEND-ISO-10303-21;\n";
        let f = parse(text).unwrap();
        assert_eq!(f.originating_system(), Some("sys /* x */"));
        let p = f.get(1).unwrap();
        assert_eq!(p.kind(), "CARTESIAN_POINT");
        let xyz = p.parts[0].1[1].as_list().unwrap();
        assert_eq!(xyz[1].as_f64(), Some(15.0));
        assert_eq!(xyz[2], Param::Int(-2));
        let u = f.get(2).unwrap();
        assert_eq!(u.kind(), "");
        assert_eq!(u.part("SI_UNIT").unwrap()[0].as_enum(), Some("MILLI"));
        let m = f
            .get(3)
            .unwrap()
            .part("UNCERTAINTY_MEASURE_WITH_UNIT")
            .unwrap();
        assert_eq!(m[0].as_f64(), Some(1e-7));
        assert_eq!(m[2].as_str(), Some("it's"));
        assert_eq!(m[3].as_str(), Some("é"));
    }

    #[test]
    fn truncated_and_deeply_nested_input_is_an_error() {
        let deep = format!("ISO-10303-21;\nDATA;\n#1=A({}", "(".repeat(100_000));
        let inputs: [&[u8]; 4] = [
            b"ISO-10303-21;\nDATA;\n#1=A(.T",
            b"ISO-10303-21;\nDATA;\n#1=A(\"0F",
            b"ISO-10303-21;\nDATA;\n#1=A(/",
            deep.as_bytes(),
        ];
        for text in inputs {
            assert!(parse(text).is_err());
        }
    }

    #[test]
    fn strings_round_trip() {
        for s in ["plain", "a.b;c.d", "quote ' and \\ slash", "ünïcode ✓"] {
            let enc = encode_string(s);
            let mut lx = Lexer {
                s: enc.as_bytes(),
                i: 0,
                depth: 0,
            };
            assert_eq!(lx.string().unwrap(), s);
        }
    }
}
