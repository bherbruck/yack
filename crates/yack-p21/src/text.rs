// yack: the text of a Part 21 string: its control directives decoded, and a string encoded as a literal.
// Adapted from springrep@a03e8a1 src/step/part21.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: decode_string and encode_string moved here as decode_text and encode_text; the line
// breaks springrep's lexer dropped from strings are dropped here; indexing by `get`.
//! The text a Part 21 string carries. [`crate::Param::Str`] keeps a string as written (only its
//! `''` undone); [`decode_text`] reads its control directives, and [`encode_text`] writes a
//! string as a literal.

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// The text of a string as [`crate::Param::Str`] holds it: line breaks dropped (a string may
/// be wrapped across lines), and the control directives decoded: `\X2\…\X0\` and `\X4\…\X0\`
/// (UTF-16 and UTF-32 hex), `\X\hh` (one ISO 8859-1 byte), and `\\`. Anything else, `\S\`
/// among it, is kept as written.
pub fn decode_text(raw: &str) -> String {
    let bytes: Vec<u8> = raw.bytes().filter(|&c| c != b'\n' && c != b'\r').collect();
    let hex = |b: &[u8]| -> Option<u32> { u32::from_str_radix(std::str::from_utf8(b).ok()?, 16).ok() };
    let mut out = String::new();
    let mut i = 0;
    while let Some(rest) = bytes.get(i..).filter(|r| !r.is_empty()) {
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
        if let Some(width) = wide
            && let Some(body) = rest.get(4..)
            && let Some(end) = find(body, b"\\X0\\")
        {
            let units: Vec<u32> = body.get(..end).unwrap_or_default().chunks(width).filter_map(hex).collect();
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
            && let Some(c) = rest.get(3..5).and_then(hex).and_then(char::from_u32)
        {
            out.push(c);
            i += 5;
            continue;
        }
        let n = rest.get(1..).and_then(|r| r.iter().position(|&c| c == b'\\')).map_or(rest.len(), |k| k + 1);
        out.push_str(&String::from_utf8_lossy(rest.get(..n).unwrap_or_default()));
        i += n;
    }
    out
}

/// `s` as a Part 21 string literal: quotes doubled, backslashes escaped, and anything outside
/// printable ASCII as `\X2\` hex.
pub fn encode_text(s: &str) -> String {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strings_round_trip_through_the_parser() {
        for s in ["plain", "a.b;c.d", "quote ' and \\ slash", "ünïcode ✓", "𝄞 beyond the BMP"] {
            let text = format!("DATA;\n#1=A({});\nENDSEC;\n", encode_text(s));
            let f = crate::parse(&text).unwrap();
            let raw = f.part(1, "A").unwrap()[0].string().unwrap();
            assert_eq!(decode_text(raw), s);
        }
    }

    #[test]
    fn decodes_every_directive_and_keeps_the_rest() {
        assert_eq!(decode_text("\\X2\\00E9\\X0\\"), "é");
        assert_eq!(decode_text("\\X4\\0001D11E\\X0\\"), "𝄞");
        assert_eq!(decode_text("\\X\\E9t\\X\\e9"), "été");
        assert_eq!(decode_text("a\\\\b"), "a\\b");
        assert_eq!(decode_text("wrapped\r\n line"), "wrapped line");
        assert_eq!(decode_text("\\S\\a and \\X2\\00E9 unclosed"), "\\S\\a and \\X2\\00E9 unclosed");
    }
}
