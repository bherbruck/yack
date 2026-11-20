// yack: `cargo xtask`, the gates and benchmark drivers (PLAN §5, §6.6). Std only.
use std::process::ExitCode;

const HELP: &str = "\
cargo xtask <command>

commands (see AGENTS.md):
  gate          structure gates: layers, one public crate, zero deps, leaks, file size,
                epsilons, hidden state, lints, provenance and licence, build dir,
                temporaries, facade discipline                         (M0.2)
  api           diff the yack public surface against crates/yack/api.txt (M0.2)
  conformance   brep-conformance run, ratcheted against scores/        (M0.4)
  bench         brep-bench run, ratcheted against scores/bench.json    (M0.5)
  modgraph      module-level DAG check inside one crate                (M2)
  fuzz          seeded robustness fuzz
  corpus        run a named corpus
  help          this text
";

fn main() -> ExitCode {
    let cmd = std::env::args().nth(1).unwrap_or_else(|| "help".to_string());
    run(&cmd)
}

fn run(cmd: &str) -> ExitCode {
    match cmd {
        "help" | "-h" | "--help" => {
            print!("{HELP}");
            ExitCode::SUCCESS
        }
        other => {
            eprintln!("xtask: `{other}` is not implemented yet\n\n{HELP}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn help_lists_the_gate() {
        assert!(super::HELP.contains("gate"));
    }
}
