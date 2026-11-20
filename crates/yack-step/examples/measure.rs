// yack: prints the measurer's volume and area of each STEP file named on the command line.
//! `measure [-v] FILE...`: one line per file, `path<TAB>volume<TAB>area` or `path<TAB>error<TAB>why`;
//! with `-v`, each face's instance number, area and volume share after it.
#![allow(clippy::print_stdout)]

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let faces = args.first().is_some_and(|a| a == "-v");
    for path in args.iter().skip(usize::from(faces)) {
        match std::fs::read_to_string(path) {
            Ok(text) => match yack_step::measure_file(&text) {
                Ok(m) => {
                    println!("{path}\t{:.17e}\t{:.17e}", m.volume, m.area);
                    if faces {
                        for f in &m.faces {
                            println!("  #{}\t{:.12e}\t{:.12e}", f.id, f.area, f.volume);
                        }
                    }
                }
                Err(e) => println!("{path}\terror\t{e}"),
            },
            Err(e) => println!("{path}\terror\t{e}"),
        }
    }
}
