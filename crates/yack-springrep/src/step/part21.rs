// yack: STEP's exchange structure as the readers take it: yack-p21's file, its names and
// enumerations upper-cased, its strings' control directives decoded; and the writer's string encoding.
// Adapted from springrep@a03e8a1 src/step/part21.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: parsing delegated to yack-p21 (M3 native rewrite) and its types used directly; the
// string codec kept.
//! STEP's exchange structure as the readers take it: [`yack_p21`]'s file, normalised as springrep
//! read it, and the string encoding the writer uses.

use yack_p21::{Entity, File, Param, Record};

use crate::{Error, Result};

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
/// One parameter as springrep read it: names and enumerations upper-cased, strings with their
/// line breaks dropped and control directives decoded, a binary as its hex string.
fn normalise(p: &mut Param) {
    match p {
        Param::Str(s) => {
            let raw: Vec<u8> = s.bytes().filter(|&c| c != b'\n' && c != b'\r').collect();
            *s = decode_string(&raw);
        }
        Param::Binary(h) => {
            let hex = std::mem::take(h);
            *p = Param::Str(hex);
        }
        Param::Enum(e) => e.make_ascii_uppercase(),
        Param::List(l) => l.iter_mut().for_each(normalise),
        Param::Typed(n, inner) => {
            n.make_ascii_uppercase();
            normalise(inner);
        }
        _ => {}
    }
}

fn normalise_record(r: &mut Record) {
    r.name.make_ascii_uppercase();
    r.params.iter_mut().for_each(normalise);
}

/// The instances in the order the file gives them.
pub(super) fn entities(f: &File) -> impl Iterator<Item = (u64, &Entity)> {
    f.order.iter().filter_map(|&id| f.get(id).map(|e| (id, e)))
}

/// The `originating_system` field of the header's `FILE_NAME`.
pub(super) fn originating_system(f: &File) -> Option<&str> {
    let name = f.header.iter().find(|r| r.name == "FILE_NAME")?;
    name.params.get(5)?.string()
}

/// An integer, or a real with no fraction (a file may write a degree or an index as `2.`, and
/// springrep has always read it as 2).
pub(super) fn integral(p: &Param) -> Option<i64> {
    match p {
        Param::Int(i) => Some(*i),
        Param::Real(x) if x.fract() == 0.0 => Some(*x as i64),
        Param::Typed(_, inner) => integral(inner),
        _ => None,
    }
}

/// The type of an instance of one record, written simple or as a one-record complex instance;
/// a complex instance of several records has none (`""`).
pub(super) fn kind(e: &Entity) -> &str {
    match e.records() {
        [r] => &r.name,
        _ => "",
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
    if !opens_as_part21(text) {
        return err("not an ISO-10303-21 file", 0);
    }
    let mut file = yack_p21::parse(&String::from_utf8_lossy(text))
        .map_err(|e| Error::InvalidInput(format!("STEP: {}", e.detail)))?;
    file.header.iter_mut().for_each(normalise_record);
    for e in file.entities.values_mut() {
        match e {
            Entity::Simple(r) => normalise_record(r),
            Entity::Complex(rs) => rs.iter_mut().for_each(normalise_record),
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
        assert_eq!(originating_system(&f), Some("sys /* x */"));
        let p = f.get(1).unwrap();
        assert_eq!(kind(p), "CARTESIAN_POINT");
        let xyz = p.records()[0].params[1].list().unwrap();
        assert_eq!(xyz[1].real(), Some(15.0));
        assert_eq!(xyz[2], Param::Int(-2));
        let u = f.get(2).unwrap();
        assert_eq!(kind(u), "");
        assert_eq!(u.part("SI_UNIT").unwrap()[0].enumeration(), Some("MILLI"));
        let m = f
            .get(3)
            .unwrap()
            .part("UNCERTAINTY_MEASURE_WITH_UNIT")
            .unwrap();
        assert_eq!(m[0].real(), Some(1e-7));
        assert_eq!(m[2].string(), Some("it's"));
        assert_eq!(m[3].string(), Some("é"));
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
            assert_eq!(f.part(1, "A").unwrap()[0].string(), Some(s));
        }
    }

    #[test]
    fn keeps_the_file_order_and_refuses_a_file_without_its_opening() {
        let text = b"ISO-10303-21;\nHEADER;\nFILE_NAME('a','',(''),(''),'','sys','');\nENDSEC;\n\
            DATA;\n#9=a(.t.,2.,LENGTH(3.));\n#2=(B(#9));\nENDSEC;\nEND-ISO-10303-21;\n";
        let f = parse(text).unwrap();
        assert_eq!(entities(&f).map(|(id, _)| id).collect::<Vec<_>>(), [9, 2]);
        let a = f.get(9).unwrap();
        assert_eq!(kind(a), "A");
        assert_eq!(a.records()[0].params[0].boolean(), Some(true));
        assert_eq!(integral(&a.records()[0].params[1]), Some(2));
        assert_eq!(integral(&a.records()[0].params[2]), Some(3));
        assert_eq!(kind(f.get(2).unwrap()), "B");
        assert_eq!(originating_system(&f), Some("sys"));
        assert!(parse(b"DATA;\n#1=A(1);\nENDSEC;\n").is_err());
        assert!(parse(b"/* c */ iso-10303-21 ;\nDATA;\n#1=A(1);\nENDSEC;\n").is_ok());
    }
}
