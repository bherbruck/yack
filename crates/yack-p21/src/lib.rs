// yack: ISO 10303-21 (STEP Part 21) exchange files read into a neutral entity graph; no geometry.
//! The clear-text encoding of an exchange structure (ISO 10303-21), read into entity instances.
//!
//! [`parse`] reads the HEADER section's records and the DATA sections: each instance `#n = NAME(params);` (simple) or
//! `#n = (A(params) B(params) ...);` (complex) becomes an [`Entity`]. Parameters keep their kind:
//! integer, real, string, enumeration, entity reference, list, typed (`NAME(param)`), unset (`$`)
//! and derived (`*`). Nothing is interpreted: that is the schema's business, not the encoding's.
//! The reader is bounded by [`Limits`], so a hostile file costs at most what the limits allow.

use std::collections::BTreeMap;

use yack_base::{Error, Result, Stage};

/// One parameter of a record.
#[derive(Clone, Debug, PartialEq)]
pub enum Param {
    /// An integer.
    Int(i64),
    /// A real.
    Real(f64),
    /// A string, its `''` escapes undone (other control directives kept as written).
    Str(String),
    /// An enumeration value, without its dots: `.T.` is `Enum("T")`.
    Enum(String),
    /// A reference to instance `#n`.
    Ref(u64),
    /// A list (or set, bag, array).
    List(Vec<Param>),
    /// A typed parameter `NAME(param)`, such as `LENGTH_MEASURE(1.)`.
    Typed(String, Box<Param>),
    /// `$`: no value.
    Unset,
    /// `*`: derived in the schema.
    Derived,
    /// A binary value, as its hex digits.
    Binary(String),
}

impl Param {
    /// The value as a real (an integer reads as a real; a typed parameter by its value).
    pub fn real(&self) -> Option<f64> {
        match self {
            Param::Real(x) => Some(*x),
            Param::Int(i) => Some(*i as f64),
            Param::Typed(_, p) => p.real(),
            _ => None,
        }
    }

    /// The value as an integer.
    pub fn int(&self) -> Option<i64> {
        match self {
            Param::Int(i) => Some(*i),
            Param::Typed(_, p) => p.int(),
            _ => None,
        }
    }

    /// The referenced instance.
    pub fn reference(&self) -> Option<u64> {
        match self {
            Param::Ref(r) => Some(*r),
            _ => None,
        }
    }

    /// The list's items.
    pub fn list(&self) -> Option<&[Param]> {
        match self {
            Param::List(l) => Some(l),
            _ => None,
        }
    }

    /// The enumeration value, as `T`/`F` booleans and the like.
    pub fn enumeration(&self) -> Option<&str> {
        match self {
            Param::Enum(e) => Some(e),
            _ => None,
        }
    }

    /// A logical or boolean: `.T.` is true, `.F.` false.
    pub fn boolean(&self) -> Option<bool> {
        match self.enumeration()? {
            "T" => Some(true),
            "F" => Some(false),
            _ => None,
        }
    }

    /// A string's text, its `''` escapes undone (see [`Param::Str`]).
    pub fn string(&self) -> Option<&str> {
        match self {
            Param::Str(s) => Some(s),
            _ => None,
        }
    }
}

/// One record: an entity type's name and its parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    /// The entity type's name, in upper case as written.
    pub name: String,
    /// Its parameters, in the schema's order.
    pub params: Vec<Param>,
}

/// An instance: one record, or a complex instance's several (one per entity type it is).
#[derive(Clone, Debug, PartialEq)]
pub enum Entity {
    /// `NAME(params)`.
    Simple(Record),
    /// `(A(params) B(params) ...)`.
    Complex(Vec<Record>),
}

impl Entity {
    /// The records of this instance.
    pub fn records(&self) -> &[Record] {
        match self {
            Entity::Simple(r) => std::slice::from_ref(r),
            Entity::Complex(rs) => rs,
        }
    }

    /// The name of a simple instance (`None` for a complex one).
    pub fn name(&self) -> Option<&str> {
        match self {
            Entity::Simple(r) => Some(&r.name),
            Entity::Complex(_) => None,
        }
    }

    /// The parameters of the record named `name`, if this instance has one.
    pub fn part(&self, name: &str) -> Option<&[Param]> {
        self.records().iter().find(|r| r.name == name).map(|r| r.params.as_slice())
    }

    /// Whether this instance has a record named `name`.
    pub fn is(&self, name: &str) -> bool {
        self.records().iter().any(|r| r.name == name)
    }
}

/// An exchange structure's instances, by number.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct File {
    /// The HEADER section's records (`FILE_DESCRIPTION`, `FILE_NAME`, `FILE_SCHEMA`, ...), in order.
    pub header: Vec<Record>,
    /// Every instance of the DATA sections.
    pub entities: BTreeMap<u64, Entity>,
    /// The instance numbers in the order the file gives them, which need not be ascending.
    pub order: Vec<u64>,
}

impl File {
    /// Instance `#id`.
    pub fn get(&self, id: u64) -> Option<&Entity> {
        self.entities.get(&id)
    }

    /// The parameters of record `name` of instance `#id`.
    pub fn part(&self, id: u64, name: &str) -> Option<&[Param]> {
        self.get(id)?.part(name)
    }
}

/// The most a file may hold before it is refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// Bytes of text.
    pub bytes: usize,
    /// Instances.
    pub entities: usize,
    /// Nesting depth of lists and typed parameters.
    pub depth: usize,
}

impl Limits {
    /// Generous bounds for any real part (a 1 GB file, 20 million instances, 64 levels).
    pub const DEFAULT: Limits = Limits { bytes: 1 << 30, entities: 20_000_000, depth: 64 };
}

const OP: &str = "p21::parse";

fn bad(at: usize, what: impl Into<String>) -> Error {
    Error::failed(OP, Stage::Parse, format!("at byte {at}: {}", what.into()))
}

/// Reads the DATA sections of an exchange structure, within [`Limits::DEFAULT`].
pub fn parse(text: &str) -> Result<File> {
    parse_within(text, Limits::DEFAULT)
}

/// Reads the DATA sections of an exchange structure, refusing a file past `limits`.
pub fn parse_within(text: &str, limits: Limits) -> Result<File> {
    if text.len() > limits.bytes {
        return Err(bad(0, format!("the file is {} bytes, over the limit of {}", text.len(), limits.bytes)));
    }
    let mut p = Parser { s: text.as_bytes(), i: 0, limits };
    let mut file = File::default();
    loop {
        p.skip();
        if p.done() {
            break;
        }
        if p.keyword("HEADER") {
            p.expect(b';')?;
            p.header(&mut file)?;
        } else if p.keyword("DATA") {
            p.skip();
            // `DATA;` or `DATA(name, schemas);`
            if p.peek() == Some(b'(') {
                let _ = p.param(0)?;
                p.skip();
            }
            p.expect(b';')?;
            p.data(&mut file)?;
        } else if p.peek() == Some(b'\'') {
            // A header string, which may hold any word (even DATA).
            let _ = p.string()?;
        } else {
            // Past one word (or one other byte), so that a keyword only matches a word's start.
            let start = p.i;
            while p.peek().is_some_and(|c| c.is_ascii_alphanumeric() || c == b'_' || c == b'-') {
                p.i += 1;
            }
            if p.i == start {
                p.i += 1;
            }
        }
    }
    Ok(file)
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
    limits: Limits,
}

impl Parser<'_> {
    fn done(&self) -> bool {
        self.i >= self.s.len()
    }

    fn peek(&self) -> Option<u8> {
        self.s.get(self.i).copied()
    }

    /// Skips white space and comments.
    fn skip(&mut self) {
        loop {
            match self.peek() {
                Some(c) if c.is_ascii_whitespace() => self.i += 1,
                Some(b'/') if self.s.get(self.i + 1) == Some(&b'*') => {
                    self.i += 2;
                    while !self.done() && !(self.peek() == Some(b'*') && self.s.get(self.i + 1) == Some(&b'/')) {
                        self.i += 1;
                    }
                    self.i = (self.i + 2).min(self.s.len());
                }
                _ => return,
            }
        }
    }

    /// Consumes `word` if the text at the cursor is it (and not a longer word).
    fn keyword(&mut self, word: &str) -> bool {
        let w = word.as_bytes();
        let end = self.i + w.len();
        if self.s.get(self.i..end) == Some(w) && !self.s.get(end).is_some_and(|c| c.is_ascii_alphanumeric() || *c == b'_') {
            self.i = end;
            true
        } else {
            false
        }
    }

    fn expect(&mut self, c: u8) -> Result<()> {
        self.skip();
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            Err(bad(self.i, format!("expected '{}'", c as char)))
        }
    }

    /// Instances up to `ENDSEC;`.
    fn header(&mut self, file: &mut File) -> Result<()> {
        loop {
            self.skip();
            if self.keyword("ENDSEC") {
                self.expect(b';')?;
                return Ok(());
            }
            if self.done() {
                return Err(bad(self.i, "the HEADER section has no ENDSEC"));
            }
            // A record `NAME(params);`; a bare word before ';' is no record and is passed over.
            let name = self.name()?;
            self.skip();
            if self.peek() == Some(b';') {
                self.i += 1;
                continue;
            }
            self.expect(b'(')?;
            let record = Record { name, params: self.params(1)? };
            self.expect(b';')?;
            if file.header.len() >= self.limits.entities {
                return Err(bad(self.i, format!("over {} header records", self.limits.entities)));
            }
            file.header.push(record);
        }
    }

    fn data(&mut self, file: &mut File) -> Result<()> {
        loop {
            self.skip();
            if self.keyword("ENDSEC") {
                self.expect(b';')?;
                return Ok(());
            }
            if self.done() {
                return Err(bad(self.i, "the DATA section has no ENDSEC"));
            }
            self.expect(b'#')?;
            let id = self.number_id()?;
            self.expect(b'=')?;
            self.skip();
            let entity = if self.peek() == Some(b'(') {
                self.i += 1;
                let mut records = Vec::new();
                loop {
                    self.skip();
                    if self.peek() == Some(b')') {
                        self.i += 1;
                        break;
                    }
                    records.push(self.record(1)?);
                }
                Entity::Complex(records)
            } else {
                Entity::Simple(self.record(0)?)
            };
            self.expect(b';')?;
            if file.entities.len() >= self.limits.entities {
                return Err(bad(self.i, format!("over {} instances", self.limits.entities)));
            }
            if file.entities.insert(id, entity).is_some() {
                return Err(bad(self.i, format!("#{id} is defined twice")));
            }
            file.order.push(id);
        }
    }

    fn number_id(&mut self) -> Result<u64> {
        let start = self.i;
        while self.peek().is_some_and(|c| c.is_ascii_digit()) {
            self.i += 1;
        }
        std::str::from_utf8(self.s.get(start..self.i).unwrap_or_default())
            .ok()
            .and_then(|t| t.parse().ok())
            .ok_or_else(|| bad(start, "expected an instance number"))
    }

    fn name(&mut self) -> Result<String> {
        self.skip();
        let start = self.i;
        while self.peek().is_some_and(|c| c.is_ascii_alphanumeric() || c == b'_' || c == b'!') {
            self.i += 1;
        }
        if self.i == start {
            return Err(bad(start, "expected an entity name"));
        }
        Ok(String::from_utf8_lossy(self.s.get(start..self.i).unwrap_or_default()).into_owned())
    }

    fn record(&mut self, depth: usize) -> Result<Record> {
        let name = self.name()?;
        self.expect(b'(')?;
        let params = self.params(depth + 1)?;
        Ok(Record { name, params })
    }

    /// Parameters after an opening '(', through the closing ')'.
    fn params(&mut self, depth: usize) -> Result<Vec<Param>> {
        if depth > self.limits.depth {
            return Err(bad(self.i, format!("nested deeper than {}", self.limits.depth)));
        }
        let mut out = Vec::new();
        self.skip();
        if self.peek() == Some(b')') {
            self.i += 1;
            return Ok(out);
        }
        loop {
            out.push(self.param(depth)?);
            self.skip();
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b')') => {
                    self.i += 1;
                    return Ok(out);
                }
                _ => return Err(bad(self.i, "expected ',' or ')'")),
            }
        }
    }

    fn param(&mut self, depth: usize) -> Result<Param> {
        self.skip();
        let start = self.i;
        match self.peek() {
            Some(b'$') => {
                self.i += 1;
                Ok(Param::Unset)
            }
            Some(b'*') => {
                self.i += 1;
                Ok(Param::Derived)
            }
            Some(b'#') => {
                self.i += 1;
                Ok(Param::Ref(self.number_id()?))
            }
            Some(b'(') => {
                self.i += 1;
                Ok(Param::List(self.params(depth + 1)?))
            }
            Some(b'\'') => self.string(),
            Some(b'"') => {
                self.i += 1;
                let s = self.i;
                while self.peek().is_some_and(|c| c != b'"') {
                    self.i += 1;
                }
                if self.done() {
                    return Err(bad(start, "a binary without its closing '\"'"));
                }
                let hex = String::from_utf8_lossy(self.s.get(s..self.i).unwrap_or_default()).into_owned();
                self.i += 1;
                Ok(Param::Binary(hex))
            }
            Some(b'.') => {
                self.i += 1;
                let s = self.i;
                while self.peek().is_some_and(|c| c != b'.') {
                    self.i += 1;
                }
                let e = String::from_utf8_lossy(self.s.get(s..self.i).unwrap_or_default()).into_owned();
                if self.done() {
                    return Err(bad(start, "an enumeration without its closing '.'"));
                }
                self.i += 1;
                Ok(Param::Enum(e))
            }
            Some(c) if c.is_ascii_digit() || c == b'-' || c == b'+' => self.number(),
            Some(c) if c.is_ascii_alphabetic() => {
                let name = self.name()?;
                self.expect(b'(')?;
                let mut inner = self.params(depth + 1)?;
                let value = if inner.len() == 1 { inner.pop().unwrap_or(Param::Unset) } else { Param::List(inner) };
                Ok(Param::Typed(name, Box::new(value)))
            }
            _ => Err(bad(start, "expected a parameter")),
        }
    }

    fn string(&mut self) -> Result<Param> {
        let start = self.i;
        self.i += 1;
        let mut out = Vec::new();
        loop {
            match self.peek() {
                None => return Err(bad(start, "a string without its closing quote")),
                Some(b'\'') if self.s.get(self.i + 1) == Some(&b'\'') => {
                    out.push(b'\'');
                    self.i += 2;
                }
                Some(b'\'') => {
                    self.i += 1;
                    return Ok(Param::Str(String::from_utf8_lossy(&out).into_owned()));
                }
                Some(c) => {
                    out.push(c);
                    self.i += 1;
                }
            }
        }
    }

    fn number(&mut self) -> Result<Param> {
        let start = self.i;
        if matches!(self.peek(), Some(b'-' | b'+')) {
            self.i += 1;
        }
        let mut real = false;
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() {
                self.i += 1;
            } else if c == b'.' || c == b'E' || c == b'e' {
                real = true;
                self.i += 1;
                if (c == b'E' || c == b'e') && matches!(self.peek(), Some(b'-' | b'+')) {
                    self.i += 1;
                }
            } else {
                break;
            }
        }
        let t = std::str::from_utf8(self.s.get(start..self.i).unwrap_or_default()).unwrap_or("");
        if real {
            // `1.` and `1.E5` are reals in the encoding; Rust's parser wants a digit after the point.
            let fixed = t.replacen(".E", ".0E", 1).replacen(".e", ".0e", 1);
            let fixed = if fixed.ends_with('.') { format!("{fixed}0") } else { fixed };
            fixed.parse().map(Param::Real).map_err(|_| bad(start, format!("'{t}' is not a real")))
        } else {
            t.parse().map(Param::Int).map_err(|_| bad(start, format!("'{t}' is not an integer")))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_simple_complex_and_every_kind_of_parameter() {
        let text = "ISO-10303-21;\nHEADER;\nFILE_NAME('a','',(''),(''),'','','');\nENDSEC;\nDATA;\n\
                    #1=CARTESIAN_POINT('it''s',(1.,-2.5E-1,3));\n\
                    /* a comment */ #2=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n\
                    #3=X(#1,$,LENGTH_MEASURE(1.),.T.,\"0F\");\nENDSEC;\nEND-ISO-10303-21;\n";
        let f = parse(text).unwrap();
        let p = f.part(1, "CARTESIAN_POINT").unwrap();
        assert_eq!(p[0], Param::Str("it's".into()));
        let xyz: Vec<f64> = p[1].list().unwrap().iter().map(|q| q.real().unwrap()).collect();
        assert_eq!(xyz, vec![1.0, -0.25, 3.0]);
        let unit = f.get(2).unwrap();
        assert!(unit.is("SI_UNIT") && unit.is("LENGTH_UNIT"));
        assert_eq!(unit.part("SI_UNIT").unwrap()[0], Param::Enum("MILLI".into()));
        let x = f.part(3, "X").unwrap();
        assert_eq!(x[0].reference(), Some(1));
        assert_eq!(x[1], Param::Unset);
        assert_eq!(x[2].real(), Some(1.0));
        assert_eq!(x[3].boolean(), Some(true));
        assert_eq!(x[4], Param::Binary("0F".into()));
    }

    #[test]
    fn a_header_word_or_string_is_not_the_data_section() {
        let text = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('no DATA; here'),'2;1');\nMYDATA;\nENDSEC;\nDATA;\n#7=A(1);\nENDSEC;\n";
        let f = parse(text).unwrap();
        assert_eq!(f.entities.len(), 1);
        assert_eq!(f.part(7, "A").unwrap()[0], Param::Int(1));
    }

    #[test]
    fn keeps_the_header_records_and_the_order_the_file_gives_its_instances() {
        let text = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('d'),'2;1');\n\
                    FILE_NAME('a.stp','t',('x'),(''),'p','sys','');\nENDSEC;\nDATA;\n\
                    #9=A(1);\n#2=B(#9);\n#5=C();\nENDSEC;\nEND-ISO-10303-21;\n";
        let f = parse(text).unwrap();
        let names: Vec<&str> = f.header.iter().map(|r| r.name.as_str()).collect();
        assert_eq!(names, ["FILE_DESCRIPTION", "FILE_NAME"]);
        assert_eq!(f.header[1].params[5], Param::Str("sys".into()));
        assert_eq!(f.order, [9, 2, 5]);
        assert_eq!(f.entities.keys().copied().collect::<Vec<_>>(), [2, 5, 9]);
        // A header without its ENDSEC is no file.
        assert!(parse("ISO-10303-21;\nHEADER;\nFILE_NAME('a');\n").is_err());
    }

    #[test]
    fn a_string_reads_as_its_text_and_nothing_else_does() {
        let f = parse("DATA;\n#1=A('it''s',.T.,#1,\"0F\");\nENDSEC;").unwrap();
        let p = f.part(1, "A").unwrap();
        assert_eq!(p[0].string(), Some("it's"));
        assert!(p[1..].iter().all(|q| q.string().is_none()));
    }

    #[test]
    fn a_binary_without_its_closing_quote_is_refused() {
        assert!(parse("DATA;\n#1=A(\"0F);\nENDSEC;").is_err());
        assert!(parse("DATA;\n#1=A(\"0F").is_err());
        assert!(parse("DATA;\n#1=A(\"0F\");\nENDSEC;").is_ok());
    }

    #[test]
    fn refuses_what_it_cannot_read_and_what_is_too_big() {
        assert!(parse("DATA;\n#1=A(;\nENDSEC;").is_err());
        assert!(parse("DATA;\n#1=A(1);\n#1=A(2);\nENDSEC;").is_err());
        let deep = format!("DATA;\n#1=A({}1{});\nENDSEC;", "(".repeat(100), ")".repeat(100));
        assert!(parse(&deep).is_err());
        let small = Limits { bytes: 10, entities: 1, depth: 4 };
        assert!(parse_within("DATA;\n#1=A(1);\nENDSEC;", small).is_err());
    }
}
