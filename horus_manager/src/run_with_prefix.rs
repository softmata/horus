//! Prefixed parallel subprocess runner.
//!
//! Runs multiple subprocesses concurrently with colored `[label]` prefixes
//! on their output. Used by `horus fmt`, `horus lint`, `horus test` when
//! dispatching to multiple languages in parallel.

use colored::*;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::{Command, ExitStatus, Stdio};
use std::thread;

// ─── Types ───────────────────────────────────────────────────────────────────

/// A subprocess to run with a prefix label.
#[derive(Debug, Clone)]
pub struct PrefixedCommand {
    /// Colored label for output (e.g., "[rust]", "[python]").
    pub label: String,

    /// Binary to execute.
    pub bin: String,

    /// Arguments to pass.
    pub args: Vec<String>,

    /// Working directory.
    pub working_dir: Option<String>,

    /// Additional environment variables.
    pub env: Vec<(String, String)>,
}

/// Result of a prefixed command execution.
#[derive(Debug)]
pub struct PrefixedResult {
    /// The label of this command.
    pub label: String,

    /// Exit status.
    pub status: ExitStatus,
}

impl PrefixedResult {
    pub fn success(&self) -> bool {
        self.status.success()
    }
}

// ─── Colors ──────────────────────────────────────────────────────────────────

/// Color palette for prefix labels (cycles for >4 languages).
const LABEL_COLORS: &[&str] = &["cyan", "yellow", "green", "magenta", "blue", "red"];

fn colorize_label(label: &str, index: usize) -> String {
    let color = LABEL_COLORS[index % LABEL_COLORS.len()];
    match color {
        "cyan" => label.cyan().bold().to_string(),
        "yellow" => label.yellow().bold().to_string(),
        "green" => label.green().bold().to_string(),
        "magenta" => label.magenta().bold().to_string(),
        "blue" => label.blue().bold().to_string(),
        "red" => label.red().bold().to_string(),
        _ => label.bold().to_string(),
    }
}

// ─── Runner ──────────────────────────────────────────────────────────────────

/// Run multiple commands in parallel with prefixed output.
///
/// Each command's stdout and stderr are line-buffered and prefixed with
/// its colored label. Output is printed to the real stdout as it arrives.
///
/// Returns results for all commands. Never panics on subprocess failure —
/// failures are captured in the exit status.
pub fn run_prefixed(commands: Vec<PrefixedCommand>) -> Vec<PrefixedResult> {
    if commands.is_empty() {
        return Vec::new();
    }

    // Single command: run directly without prefixing (cleaner output)
    if commands.len() == 1 {
        return vec![run_single(&commands[0])];
    }

    // Multiple commands: run in parallel with prefixed output
    run_parallel(commands)
}

/// Run a single command without prefix overhead — inherits stdio directly.
fn run_single(cmd: &PrefixedCommand) -> PrefixedResult {
    let mut process = Command::new(&cmd.bin);
    process.args(&cmd.args);

    if let Some(ref dir) = cmd.working_dir {
        process.current_dir(dir);
    }

    for (k, v) in &cmd.env {
        process.env(k, v);
    }

    process.stdin(Stdio::inherit());
    process.stdout(Stdio::inherit());
    process.stderr(Stdio::inherit());

    let status = match process.status() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{} Failed to run '{}': {}", cmd.label, cmd.bin, e);
            // Return a fake failed status
            return PrefixedResult {
                label: cmd.label.clone(),
                status: fake_exit_status(),
            };
        }
    };

    PrefixedResult {
        label: cmd.label.clone(),
        status,
    }
}

/// Run multiple commands in parallel with line-buffered prefixed output.
fn run_parallel(commands: Vec<PrefixedCommand>) -> Vec<PrefixedResult> {
    let count = commands.len();

    let mut handles = Vec::new();

    for (i, cmd) in commands.into_iter().enumerate() {
        let colored_label = colorize_label(&cmd.label, i);

        let handle = thread::spawn(move || {
            let result = run_with_prefix_output(&cmd, &colored_label);
            (i, result)
        });
        handles.push(handle);
    }

    // Collect in spawn order
    let mut ordered: Vec<(usize, PrefixedResult)> = Vec::with_capacity(count);
    for handle in handles {
        if let Ok(pair) = handle.join() {
            ordered.push(pair);
        }
    }
    ordered.sort_by_key(|(i, _)| *i);
    ordered.into_iter().map(|(_, r)| r).collect()
}

/// Run a single command, prefixing each line of stdout/stderr with label.
fn run_with_prefix_output(cmd: &PrefixedCommand, colored_label: &str) -> PrefixedResult {
    let mut process = Command::new(&cmd.bin);
    process.args(&cmd.args);

    if let Some(ref dir) = cmd.working_dir {
        process.current_dir(dir);
    }

    for (k, v) in &cmd.env {
        process.env(k, v);
    }

    process.stdin(Stdio::null());
    process.stdout(Stdio::piped());
    process.stderr(Stdio::piped());

    let mut child = match process.spawn() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{} Failed to run '{}': {}", colored_label, cmd.bin, e);
            return PrefixedResult {
                label: cmd.label.clone(),
                status: fake_exit_status(),
            };
        }
    };

    // Read stdout and stderr in separate threads, prefix each line
    let stdout = child.stdout.take().unwrap();
    let stderr = child.stderr.take().unwrap();

    let label_out = colored_label.to_string();
    let label_err = colored_label.to_string();

    let stdout_handle = thread::spawn(move || {
        let reader = BufReader::new(stdout);
        for line in reader.lines() {
            match line {
                Ok(line) => println!("{} {}", label_out, line),
                Err(_) => break,
            }
        }
    });

    let stderr_handle = thread::spawn(move || {
        let reader = BufReader::new(stderr);
        for line in reader.lines() {
            match line {
                Ok(line) => eprintln!("{} {}", label_err, line),
                Err(_) => break,
            }
        }
    });

    let status = child.wait().unwrap_or_else(|_| fake_exit_status());

    let _ = stdout_handle.join();
    let _ = stderr_handle.join();

    PrefixedResult {
        label: cmd.label.clone(),
        status,
    }
}

/// Get the worst (highest) exit code from results.
pub fn worst_exit_code(results: &[PrefixedResult]) -> i32 {
    results
        .iter()
        .filter_map(|r| r.status.code())
        .max()
        .unwrap_or(0)
}

/// Check if all results succeeded.
pub fn all_succeeded(results: &[PrefixedResult]) -> bool {
    results.iter().all(|r| r.success())
}

/// Print a summary line for each result.
pub fn print_summary(results: &[PrefixedResult]) {
    if results.len() <= 1 {
        return;
    }
    println!();
    for (i, result) in results.iter().enumerate() {
        let label = colorize_label(&result.label, i);
        if result.success() {
            println!("{} {}", label, "OK".green());
        } else {
            let code = result.status.code().unwrap_or(-1);
            println!("{} {} (exit {})", label, "FAILED".red(), code);
        }
    }
}

/// Build a `PrefixedCommand` from dispatch types.
pub fn from_resolved_tool(
    tool: &crate::dispatch::ResolvedTool,
    extra_args: &[String],
    working_dir: &Path,
) -> PrefixedCommand {
    let mut args = tool.default_args.clone();
    args.extend_from_slice(extra_args);

    PrefixedCommand {
        label: tool.label.clone(),
        bin: tool.bin.clone(),
        args,
        working_dir: Some(working_dir.to_string_lossy().to_string()),
        env: Vec::new(),
    }
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Create a fake failed ExitStatus for error cases.
fn fake_exit_status() -> ExitStatus {
    use std::os::unix::process::ExitStatusExt;
    ExitStatus::from_raw(256) // exit code 1
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_command_runs_directly() {
        let cmd = PrefixedCommand {
            label: "[test]".to_string(),
            bin: "echo".to_string(),
            args: vec!["hello".to_string()],
            working_dir: None,
            env: Vec::new(),
        };
        let results = run_prefixed(vec![cmd]);
        assert_eq!(results.len(), 1);
        assert!(results[0].success());
    }

    #[test]
    fn parallel_commands_all_succeed() {
        let commands = vec![
            PrefixedCommand {
                label: "[a]".to_string(),
                bin: "echo".to_string(),
                args: vec!["first".to_string()],
                working_dir: None,
                env: Vec::new(),
            },
            PrefixedCommand {
                label: "[b]".to_string(),
                bin: "echo".to_string(),
                args: vec!["second".to_string()],
                working_dir: None,
                env: Vec::new(),
            },
        ];
        let results = run_prefixed(commands);
        assert_eq!(results.len(), 2);
        assert!(all_succeeded(&results));
        assert_eq!(worst_exit_code(&results), 0);
    }

    #[test]
    fn failed_command_captured() {
        let cmd = PrefixedCommand {
            label: "[fail]".to_string(),
            bin: "false".to_string(),
            args: vec![],
            working_dir: None,
            env: Vec::new(),
        };
        let results = run_prefixed(vec![cmd]);
        assert_eq!(results.len(), 1);
        assert!(!results[0].success());
    }

    #[test]
    fn mixed_success_and_failure() {
        let commands = vec![
            PrefixedCommand {
                label: "[ok]".to_string(),
                bin: "true".to_string(),
                args: vec![],
                working_dir: None,
                env: Vec::new(),
            },
            PrefixedCommand {
                label: "[fail]".to_string(),
                bin: "false".to_string(),
                args: vec![],
                working_dir: None,
                env: Vec::new(),
            },
        ];
        let results = run_prefixed(commands);
        assert!(!all_succeeded(&results));
        assert_eq!(worst_exit_code(&results), 1);
    }

    #[test]
    fn empty_commands_returns_empty() {
        let results = run_prefixed(vec![]);
        assert!(results.is_empty());
    }

    #[test]
    fn nonexistent_binary_fails_gracefully() {
        let cmd = PrefixedCommand {
            label: "[bad]".to_string(),
            bin: "this_binary_does_not_exist_xyz_123".to_string(),
            args: vec![],
            working_dir: None,
            env: Vec::new(),
        };
        let results = run_prefixed(vec![cmd]);
        assert_eq!(results.len(), 1);
        assert!(!results[0].success());
    }

    #[test]
    fn env_vars_passed_through() {
        let cmd = PrefixedCommand {
            label: "[env]".to_string(),
            bin: "sh".to_string(),
            args: vec!["-c".to_string(), "test \"$MY_VAR\" = hello".to_string()],
            working_dir: None,
            env: vec![("MY_VAR".to_string(), "hello".to_string())],
        };
        let results = run_prefixed(vec![cmd]);
        assert!(results[0].success());
    }

    #[test]
    fn working_dir_respected() {
        let dir = tempfile::tempdir().unwrap();
        let cmd = PrefixedCommand {
            label: "[dir]".to_string(),
            bin: "pwd".to_string(),
            args: vec![],
            working_dir: Some(dir.path().to_string_lossy().to_string()),
            env: Vec::new(),
        };
        let results = run_prefixed(vec![cmd]);
        assert!(results[0].success());
    }
}
