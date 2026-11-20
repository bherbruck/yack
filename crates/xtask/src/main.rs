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
  digest [--check|--bless --reason R] [--sample P]  result digests in scores/digests/ (M1.6)
  ci [--quick] [--bin-dir D --tree FP] [--skip-suite]  the pre-PR gate: gate, api, digest --check,
                conformance ratchet (--quick: 10% family samples, seed 1 for the ratchet)
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

/// `cargo xtask ci [--quick] [--bin-dir D --tree FP] [--skip-suite]`: every gate a change must pass
/// before it is committed, all run, each reported:
/// 1. `gate` (structure) and `api` (the public surface);
/// 2. `digest --check` (bit-identical results; `--quick`: a 10% family sample of conformance);
/// 3. the conformance score ratchet (`conformance`, seeds 0-3; `--quick`: seed 1, 10% of families).
///
/// `--skip-suite` skips 2's conformance half and 3 (no brep-conformance runner, e.g. on a CI host
/// that could not fetch it), saying so.
fn ci(args: &[String]) -> ExitCode {
    let quick = args.iter().any(|a| a == "--quick");
    let skip_suite = args.iter().any(|a| a == "--skip-suite");
    let mut pass: Vec<String> = Vec::new();
    let mut bin_dir: Option<String> = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        if matches!(a.as_str(), "--bin-dir" | "--tree" | "--threads" | "--ref") {
            let v = it.next().cloned().unwrap_or_default();
            if a == "--bin-dir" {
                bin_dir = Some(v.clone());
            }
            pass.extend([a.clone(), v]);
        }
    }
    let mut steps: Vec<(String, ExitCode)> = vec![
        ("gate".into(), finish(gate::main(&[]))),
        ("api".into(), finish(gate::api::main(&[]))),
    ];
    let mut digest_args: Vec<String> = vec!["--check".into()];
    digest_args.extend(pass.iter().cloned());
    if skip_suite {
        println!("ci: SKIPPED conformance digests and the conformance ratchet (--skip-suite: no runner)");
        digest_args.extend(["--only".into(), "bench".into()]);
    } else if quick {
        digest_args.extend(["--sample".into(), "10".into()]);
    }
    steps.push(("digest --check".into(), digest::run(&digest_args)));
    if !skip_suite {
        let mut conf: Vec<String> = Vec::new();
        if quick {
            match digest::sample_families(1, 10) {
                Ok(fams) => conf.extend(["--seed".into(), "1".into(), "--family".into(), fams.join(",")]),
                Err(e) => println!("error: ci: {e}"),
            }
        } else {
            conf.extend(["--seeds".into(), "0-3".into()]);
        }
        if let Some(d) = &bin_dir {
            conf.extend(["--adapter".into(), format!("{d}/yack-conformance")]);
        }
        steps.push(("conformance ratchet".into(), conformance::run(&conf)));
    }
    for (name, code) in &steps {
        println!("ci: {name}: {}", if *code == ExitCode::SUCCESS { "ok" } else { "FAILED" });
    }
    let failed: Vec<&str> = steps.iter().filter(|(_, c)| *c != ExitCode::SUCCESS).map(|(n, _)| n.as_str()).collect();
    if failed.is_empty() {
        println!("ci: all passed");
        ExitCode::SUCCESS
    } else {
        println!("error: ci failed: {}", failed.join(", "));
        ExitCode::FAILURE
    }
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
        "ci" => ci(args),
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
