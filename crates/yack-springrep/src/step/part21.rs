// yack: The STEP exchange structure (ISO 10303-21): a parser of the header and the data section into
// Adapted from springrep@a03e8a1 src/step/part21.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: parsing delegated to yack-p21 (M3 native rewrite); springrep's types, accessors and
// string codec kept, the instances in the file's order, names and enumerations upper-cased.
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

/// Whether the text opens with the exchange structure's `ISO-10303-21;`, after any white space
/// and comments: yack-p21 reads the sections wherever they are, and a file without it is refused.
fn opens_as_part21(s: &[u8]) -> bool {
    let mut i = 0;
    let skip = |mut i: usize| loop {
        while s.get(i).is_some_and(u8::is_ascii_whitespace) {
            i += 1;
        }
        if s.get(i..).is_some_and(|r| r.starts_with(b"/*")) {
            i = find(&s[i + 2..], b"*/").map_or(s.len(), |k| i + k + 4);
        } else {
            return i;
        }
    };
    i = skip(i);
    let word = b"ISO-10303-21";
    let Some(head) = s.get(i..i + word.len()) else {
        return false;
    };
    let after = s.get(i + word.len()).copied();
    let continues = after.is_some_and(|c| c.is_ascii_alphanumeric() || c == b'_' || c == b'-');
    if !head.eq_ignore_ascii_case(word) || continues {
        return false;
    }
    s.get(skip(i + word.len())) == Some(&b';')
}

/// A yack-p21 parameter as springrep's: names and enumerations upper-cased, strings with their
/// control directives decoded and their line breaks dropped, a binary as its hex string.
fn param(p: &yack_p21::Param) -> Param {
    use yack_p21::Param as P;
    match p {
        P::Int(i) => Param::Int(*i),
        P::Real(x) => Param::Real(*x),
        P::Str(s) => {
            let raw: Vec<u8> = s.bytes().filter(|&c| c != b'\n' && c != b'\r').collect();
            Param::Str(decode_string(&raw))
        }
        P::Binary(h) => Param::Str(h.clone()),
        P::Enum(e) => Param::Enum(e.to_ascii_uppercase()),
        P::Ref(r) => Param::Ref(*r),
        P::List(l) => Param::List(l.iter().map(param).collect()),
        P::Typed(n, inner) => Param::Typed(n.to_ascii_uppercase(), Box::new(param(inner))),
        P::Unset => Param::Unset,
        P::Derived => Param::Derived,
    }
}

fn part(r: &yack_p21::Record) -> (String, Vec<Param>) {
    (r.name.to_ascii_uppercase(), r.params.iter().map(param).collect())
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
    if !opens_as_part21(text) {
        return err("not an ISO-10303-21 file", 0);
    }
    let read = yack_p21::parse(&String::from_utf8_lossy(text))
        .map_err(|e| Error::InvalidInput(format!("STEP: {}", e.detail)))?;
    let mut file = File {
        header: read
            .header
            .iter()
            .map(|r| Entity {
                parts: vec![part(r)],
            })
            .collect(),
        ..File::default()
    };
    for &id in &read.order {
        let Some(e) = read.get(id) else { continue };
        file.index.insert(id, file.entities.len());
        file.ids.push(id);
        file.entities.push(Entity {
            parts: e.records().iter().map(part).collect(),
        });
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
            let text = format!("ISO-10303-21;\nDATA;\n#1=A({});\nENDSEC;\n", encode_string(s));
            let f = parse(text.as_bytes()).unwrap();
            assert_eq!(f.get(1).unwrap().part("A").unwrap()[0].as_str(), Some(s));
        }
    }

    #[test]
    fn keeps_the_file_order_and_refuses_a_file_without_its_opening() {
        let text = b"ISO-10303-21;\nHEADER;\nFILE_NAME('a','',(''),(''),'','sys','');\nENDSEC;\n\
            DATA;\n#9=a(.t.);\n#2=B(#9);\nENDSEC;\nEND-ISO-10303-21;\n";
        let f = parse(text).unwrap();
        assert_eq!(f.ids, [9, 2]);
        assert_eq!(f.get(9).unwrap().kind(), "A");
        assert_eq!(f.get(9).unwrap().parts[0].1[0].as_bool(), Some(true));
        assert_eq!(f.originating_system(), Some("sys"));
        assert!(parse(b"DATA;\n#1=A(1);\nENDSEC;\n").is_err());
        assert!(parse(b"/* c */ iso-10303-21 ;\nDATA;\n#1=A(1);\nENDSEC;\n").is_ok());
    }
}
