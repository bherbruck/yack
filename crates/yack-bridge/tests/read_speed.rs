// yack: probe of springrep's STEP reader speed on our own written files (ladder16 timeouts:
// swept/fine-thread-with-sharp-crests re-read 79.6 s). Reads each file in the probe directory.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::disallowed_methods, clippy::print_stderr)]

const DIR: &str = "/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/w-m11/l16";

#[test]
#[ignore = "probe: run by hand on the scratchpad's files"]
fn read_our_written_files() {
    let mut paths: Vec<_> = std::fs::read_dir(DIR).unwrap().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "step")).collect();
    paths.sort();
    for p in paths {
        let text = std::fs::read(&p).unwrap();
        let t = std::time::Instant::now();
        let r = yack_step::read_part(&text, "r");
        let secs = t.elapsed().as_secs_f64();
        match r {
            Ok(b) => eprintln!("{}: {secs:.2} s, volume {}, area {}", p.display(), yack_measure::volume(&b), yack_measure::area(&b)),
            Err(e) => eprintln!("{}: {secs:.2} s, refused: {e}", p.display()),
        }
    }
}
