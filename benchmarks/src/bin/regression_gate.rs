//! The CI performance gate.
//!
//! Reads one or more JSON benchmark reports for the current commit, compares
//! them against a rolling baseline window, prints the whole distribution on
//! both sides with the threshold that was crossed, and exits non-zero if
//! something blocking regressed.
//!
//! The comparison itself lives in `horus_benchmarks::output`; this binary is
//! argument parsing, file I/O and the exit code, so that the gate's logic is
//! unit-testable without a CI runner. It exists because the comparison had no
//! caller at all: `BenchmarkReport::compare` was dead code, `baseline.json` was
//! restored from a CI cache that nothing ever wrote, and the workflow step
//! named "Check for performance regression" only grepped the benchmark's own
//! stdout for a PASS/FAIL string.
//!
//! ```text
//! regression_gate --history baseline-history.json \
//!                 --current run-1.json --current run-2.json \
//!                 [--record] [--trunk] [--window 20] \
//!                 [--markdown gate.md] [--json comparison.json]
//! ```
//!
//! `--record` appends this job to the window and rewrites the history file.
//! Only ever pass it on a trunk build: recording a pull request would let a
//! regression become its own baseline.

use horus_benchmarks::output::{BaselineHistory, ComparisonInput, RegressionPolicy};
use horus_benchmarks::BenchmarkReport;
use std::process::ExitCode;

struct Args {
    history: String,
    current: Vec<String>,
    record: bool,
    trunk: bool,
    window: usize,
    markdown: Option<String>,
    json: Option<String>,
    require_baseline: bool,
}

fn usage() -> &'static str {
    "usage: regression_gate --history <path> --current <report.json> [--current <report.json>...]\n\
     \n\
     options:\n\
     \x20 --history <path>       rolling baseline window (created if absent)\n\
     \x20 --current <path>       one benchmark report for the current commit; repeat for\n\
     \x20                        each repetition of the benchmark in this job\n\
     \x20 --record               append this job to the window and rewrite --history\n\
     \x20                        (trunk builds only — never on a pull request)\n\
     \x20 --trunk                enable the consecutive-runs escalation for report-only\n\
     \x20                        metrics; must not be set for pull-request runs\n\
     \x20 --window <n>           how many runs the window keeps (default 20)\n\
     \x20 --markdown <path>      write the pull-request comment body\n\
     \x20 --json <path>          write the full comparison as JSON\n\
     \x20 --require-baseline     exit non-zero if the window is empty (wiring check)\n"
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        history: String::new(),
        current: Vec::new(),
        record: false,
        trunk: false,
        window: 20,
        markdown: None,
        json: None,
        require_baseline: false,
    };

    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        let need = |i: usize, what: &str| -> Result<String, String> {
            argv.get(i + 1)
                .cloned()
                .ok_or_else(|| format!("{} needs a value", what))
        };
        match argv[i].as_str() {
            "--history" => {
                args.history = need(i, "--history")?;
                i += 2;
            }
            "--current" => {
                args.current.push(need(i, "--current")?);
                i += 2;
            }
            "--markdown" => {
                args.markdown = Some(need(i, "--markdown")?);
                i += 2;
            }
            "--json" => {
                args.json = Some(need(i, "--json")?);
                i += 2;
            }
            "--window" => {
                args.window = need(i, "--window")?
                    .parse()
                    .map_err(|_| "--window needs a number".to_string())?;
                i += 2;
            }
            "--record" => {
                args.record = true;
                i += 1;
            }
            "--trunk" => {
                args.trunk = true;
                i += 1;
            }
            "--require-baseline" => {
                args.require_baseline = true;
                i += 1;
            }
            "-h" | "--help" => return Err(usage().to_string()),
            other => return Err(format!("unknown argument: {}\n\n{}", other, usage())),
        }
    }

    if args.history.is_empty() {
        return Err(format!("--history is required\n\n{}", usage()));
    }
    if args.current.is_empty() {
        return Err(format!("--current is required\n\n{}", usage()));
    }
    if args.record && !args.trunk {
        return Err(
            "--record without --trunk: recording a pull request would let a regression \
             become its own baseline"
                .to_string(),
        );
    }
    Ok(args)
}

fn load_report(path: &str) -> Result<BenchmarkReport, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path, e))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {}", path, e))
}

fn run() -> Result<i32, String> {
    let args = parse_args()?;

    let reports: Vec<BenchmarkReport> = args
        .current
        .iter()
        .map(|p| load_report(p))
        .collect::<Result<_, _>>()?;
    let refs: Vec<&BenchmarkReport> = reports.iter().collect();

    let total_results: usize = reports.iter().map(|r| r.results.len()).sum();
    if total_results == 0 {
        // A gate with nothing to check is a failure, not a pass. This is the
        // single most common way a performance gate goes quietly green forever.
        return Err(
            "the current reports contain zero benchmark results — the benchmark produced \
             no JSON, or produced an empty one. Nothing was gated."
                .to_string(),
        );
    }

    let mut history = BaselineHistory::load(&args.history).map_err(|e| e.to_string())?;
    history.max_runs = args.window.max(1);

    if args.require_baseline && history.runs.is_empty() {
        return Err(format!(
            "--require-baseline: no baseline runs in {}",
            args.history
        ));
    }

    let policy = RegressionPolicy::default();
    let comparison = history.evaluate(&ComparisonInput {
        current: &refs,
        policy: &policy,
        is_trunk_run: args.trunk,
    });

    comparison.print_summary();

    if let Some(path) = &args.markdown {
        std::fs::write(path, comparison.to_markdown()).map_err(|e| format!("{}: {}", path, e))?;
    }
    if let Some(path) = &args.json {
        let text = serde_json::to_string_pretty(&comparison).map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| format!("{}: {}", path, e))?;
    }

    let exit = comparison.exit_code();

    if args.record {
        // Record even when the gate failed: the window must keep describing
        // what trunk actually does, or the next run compares against a fiction.
        // A regression that lands is a regression the baseline has to know
        // about, otherwise every subsequent build fails for the same reason and
        // the gate gets switched off.
        history.push_reports(&refs);
        history.save(&args.history).map_err(|e| e.to_string())?;
        println!(
            "\nRecorded this run. Baseline window now holds {} run(s) in {}.",
            history.runs.len(),
            args.history
        );
    }

    Ok(exit)
}

fn main() -> ExitCode {
    match run() {
        Ok(0) => ExitCode::SUCCESS,
        Ok(_) => ExitCode::FAILURE,
        Err(message) => {
            eprintln!("regression_gate: {}", message);
            ExitCode::FAILURE
        }
    }
}
