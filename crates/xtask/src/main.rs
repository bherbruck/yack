// yack: `cargo xtask`, the gates and benchmark drivers (PLAN §5, §6.6). Std only.
mod bench;
mod conformance;
mod digest;
mod gate;
mod json;
mod meta;
mod toml;

use std::process::ExitCode;

const HELP: &str = "\
cargo xtask <command>

commands (see AGENTS.md):
  gate [--bless] [--fixture <f>]  structure gates: layers, one public crate, zero deps, leaks, file size,
                epsilons, hidden state, lints, provenance and licence, build dir,
                temporaries, facade discipline                         (M0.2)
  api [--bless|--print]  diff the yack public surface against crates/yack/api.txt (M0.2)
  conformance   brep-conformance run, ratcheted against scores/        (M0.4)
  bench         brep-bench run, ratcheted against scores/bench.json    (M0.5)
  digest [--check] [--only conformance|bench]  result digests in scores/digests/ (M1.6)
  modgraph --crate <c>  module-level DAG check inside one crate       (M2)
  fuzz          seeded robustness fuzz
  corpus        run a named corpus
  help          this text
";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cmd = args.first().cloned().unwrap_or_else(|| "help".to_string());
    run(&cmd, args.get(1..).unwrap_or_default())
}

/// Maps a command's result to an exit code.
fn finish(r: Result<bool, String>) -> ExitCode {
    match r {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("error: xtask: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(cmd: &str, args: &[String]) -> ExitCode {
    match cmd {
        "help" | "-h" | "--help" => {
            print!("{HELP}");
            ExitCode::SUCCESS
        }
        "gate" => finish(gate::main(args)),
        "api" => finish(gate::api::main(args)),
        "modgraph" => finish(gate::modgraph::main(args)),
        "conformance" => conformance::run(args),
        "bench" => bench::run(args),
        "digest" => digest::run(args),
        "digest-proxy" => digest::proxy(args),
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
