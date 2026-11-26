// yack: every STEP file and codec fixture the crate's tests hold, read through the topology pass without its
// new table refusal ("the topology names ... it does not hold", hardening S2), which only a broken file meets.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::path::Path;

/// Whether `r` failed on a table the file's own topology names but lacks.
fn broken<T>(r: &yack_base::kernel::Result<T>) -> bool {
    r.as_ref().is_err_and(|e| e.to_string().contains("the topology names"))
}

#[test]
fn no_file_here_reaches_the_broken_table_refusal() {
    let data = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let mut names: Vec<_> = std::fs::read_dir(&data).unwrap().map(|e| e.unwrap().path()).collect();
    names.sort();
    let mut read = 0;
    for p in names {
        let bytes = std::fs::read(&p).unwrap();
        match p.extension().and_then(|x| x.to_str()) {
            // A file as written, read as yack reads it.
            Some("step") => {
                let r = yack_step::read_part(&bytes, "t");
                assert!(!broken(&r), "{}: {:?}", p.display(), r.err());
                read += 1;
            }
            // A solid of our own, written and read back.
            Some("yack") => {
                for b in yack_codec::from_bytes(&bytes).unwrap() {
                    let text = yack_step::write(&b, "t").unwrap();
                    let r = yack_step::read_part(text.as_bytes(), "t");
                    assert!(!broken(&r), "{}: {:?}", p.display(), r.err());
                    assert!(r.is_ok(), "{}: {:?}", p.display(), r.err());
                    read += 1;
                }
            }
            _ => {}
        }
    }
    assert!(read >= 20, "{read} files read");
}
