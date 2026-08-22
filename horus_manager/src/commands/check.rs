//! Check command - validate horus.toml, source files, or entire workspace

use crate::cli_output;
use crate::config::CARGO_TOML;
use crate::manifest::{detect_languages, HorusManifest, Language, HORUS_TOML};
use crate::manifest_lint;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::memory::shm_base_dir;
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

/// Run the check command on a path (directory or file)
pub fn run_check(path: Option<PathBuf>, quiet: bool, json: bool) -> HorusResult<()> {
    let target_path = path.unwrap_or_else(|| PathBuf::from("."));
    log::debug!("checking path: {:?}", target_path);

    // Every other command warns on stderr about keys it does not understand
    // when it loads a manifest. `check` reports them itself — with the line, a
    // did-you-mean and an exit code — so the terse version would print twice.
    let _silence = crate::manifest::ManifestLintSilence::new();

    if !target_path.exists() {
        if json {
            let output = serde_json::json!({
                "path": target_path.display().to_string(),
                "valid": false,
                "error": "Path not found",
            });
            println!(
                "{}",
                serde_json::to_string_pretty(&output).unwrap_or_default()
            );
            return Err(HorusError::Config(ConfigError::Other(
                "Path not found".to_string(),
            )));
        }
        println!(
            "{} Path not found: {}",
            cli_output::ICON_ERROR.red(),
            target_path.display()
        );
        return Err(HorusError::Config(ConfigError::Other(
            "Path not found".to_string(),
        )));
    }

    // For JSON mode, capture check result and output as JSON only.
    // Suppress all human-readable output by temporarily redirecting stdout.
    if json {
        use std::io::Write;

        // Run the validation, capturing stdout to suppress human-readable output
        let result = {
            // Redirect stdout to null device during check (cross-platform)
            let _guard = horus_sys::fs::suppress_stdout().ok();
            let _ = std::io::stdout().flush();

            let r = if target_path.is_dir() {
                check_workspace(&target_path, true)
            } else {
                check_single_file(&target_path, true)
            };

            let _ = std::io::stdout().flush();
            drop(_guard);
            r
        };

        let valid = result.is_ok();
        let error_msg = result.as_ref().err().map(|e| e.to_string());
        let diagnostics = take_diagnostics();
        let output = serde_json::json!({
            "path": target_path.display().to_string(),
            "valid": valid,
            "error": error_msg,
            // The findings themselves, not just how many there were.
            "diagnostics": diagnostics,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return if valid { Ok(()) } else { result };
    }

    if target_path.is_dir() {
        check_workspace(&target_path, quiet)
    } else {
        check_single_file(&target_path, quiet)
    }
}

/// A single finding from `horus check`, in machine-readable form.
///
/// `--json` used to emit only a count:
///
/// ```json
/// { "error": "Configuration error: 1 error(s) found", "valid": false }
/// ```
///
/// while the human output named the actual problem — so the machine-readable
/// mode was strictly *less* informative than the terminal one, which defeats
/// the reason it exists. CI and editor integrations got a boolean.
#[derive(Debug, Clone, serde::Serialize)]
pub struct CheckDiagnostic {
    /// "error" or "warning".
    pub severity: &'static str,
    /// Stable identifier for the *kind* of finding, e.g.
    /// `manifest.unknown-key`. A consumer that wants to filter or fix one class
    /// of problem cannot match on prose, and prose is what the message is.
    pub code: &'static str,
    /// File the finding is in, relative to the scanned root.
    pub file: String,
    /// 1-based line, when known.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line: Option<usize>,
    /// 1-based column, when known.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub column: Option<usize>,
    pub message: String,
}

impl CheckDiagnostic {
    /// A finding that makes the check fail.
    pub fn error(code: &'static str, file: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            severity: "error",
            code,
            file: file.into(),
            line: None,
            column: None,
            message: message.into(),
        }
    }

    /// A finding worth saying, that does not fail the check.
    pub fn warning(
        code: &'static str,
        file: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self {
            severity: "warning",
            ..Self::error(code, file, message)
        }
    }

    /// Attach a position.
    pub fn at(mut self, line: Option<usize>, column: Option<usize>) -> Self {
        self.line = line;
        self.column = column;
        self
    }

    /// Attach a 1-based line, dropping a zero (which means "not found").
    ///
    /// Column 1: a TOML key starts its own line, and an editor placing a
    /// squiggle needs a column to anchor to.
    fn on_line(self, line: usize) -> Self {
        match line {
            0 => self,
            n => self.at(Some(n), Some(1)),
        }
    }

    /// Record it for `--json`.
    fn record(self) {
        DIAGNOSTICS.with(|d| d.borrow_mut().push(self));
    }
}

thread_local! {
    /// Findings recorded during the current `horus check` run.
    ///
    /// A thread-local rather than a threaded-through parameter: `check` is one
    /// pass over the workspace on one thread, and the alternative is changing
    /// the signature of every helper that prints.
    static DIAGNOSTICS: std::cell::RefCell<Vec<CheckDiagnostic>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// Split a `path:line:col: message` line, as `cargo --message-format=short`
/// emits it.
///
/// Returns `None` for anything that is not in that shape, so a cargo banner or
/// a Windows drive letter falls back to the whole line rather than being
/// mis-parsed into a position.
fn split_short_diagnostic(line: &str) -> Option<(&str, usize, usize, &str)> {
    let mut parts = line.splitn(4, ':');
    let file = parts.next()?;
    let line_no: usize = parts.next()?.trim().parse().ok()?;
    let col: usize = parts.next()?.trim().parse().ok()?;
    let message = parts.next()?.trim();
    (!file.is_empty() && !message.is_empty()).then_some((file, line_no, col, message))
}

/// The line number out of a Python traceback (`File "x.py", line 12`).
fn python_error_line(text: &str) -> Option<usize> {
    text.lines().find_map(|l| {
        let rest = l.split("line ").nth(1)?;
        let digits: String = rest
            .trim_start()
            .chars()
            .take_while(char::is_ascii_digit)
            .collect();
        digits.parse().ok()
    })
}

/// `file:line:col: message` — the position in front of the text a user reads,
/// not only in the JSON.
///
/// One format for every finding: "Version 'x' is not valid semver" is the same
/// sentence whichever of three manifests in a workspace it came from, and an
/// editor that can jump to a `cargo` error can jump to these.
fn located(d: &CheckDiagnostic) -> String {
    match d.line {
        // `toml`'s own errors already begin with their location; adding a
        // second one in front would be the only thing worse than none.
        Some(n) if d.message.contains(&format!(":{n}:")) => d.message.clone(),
        Some(n) => format!("{}:{}:{}: {}", d.file, n, d.column.unwrap_or(1), d.message),
        None => d.message.clone(),
    }
}

/// Where a validation finding sits in the manifest source, when it can be
/// found.
fn locate_issue(source: &str, key: Option<&str>) -> (Option<usize>, Option<usize>) {
    let Some(key) = key else {
        return (None, None);
    };
    let line = manifest_lint::locate_key(source, key);
    if line > 0 {
        // A TOML key starts its own line, so the key name is the position.
        (Some(line), Some(1))
    } else if !key.contains('.') {
        // A table that is absent has no line of its own — the file as a whole
        // is what is wrong. 1:1 is what `toml` itself reports for a missing
        // field, so a missing `[package]` reads the same way.
        (Some(1), Some(1))
    } else {
        (None, None)
    }
}

/// Describe a manifest that could not be loaded, identically wherever it
/// happens.
///
/// `horus check` (directory) and `horus check <file>` rendered the same failure
/// two different ways: the directory path printed the class and the position,
/// while the single-file path prefixed it with `Failed to parse manifest:` —
/// announcing a *syntax* error over a message whose own next line reads "The
/// file is valid TOML — a required key is absent", and making the two failure
/// classes indistinguishable on that path. One description, so they cannot
/// drift apart again.
fn describe_manifest_failure(err: &anyhow::Error, file: &str) -> CheckDiagnostic {
    use crate::manifest::{ManifestError, ManifestErrorKind};

    let manifest_err = err.downcast_ref::<ManifestError>();
    let code = match manifest_err.map(|m| &m.kind) {
        Some(ManifestErrorKind::MissingField { .. }) => "manifest.missing-field",
        Some(ManifestErrorKind::Schema) => "manifest.schema",
        Some(ManifestErrorKind::Syntax) => "manifest.syntax",
        // Unreadable file, wrong permissions: not a manifest problem at all.
        None => "manifest.unreadable",
    };
    CheckDiagnostic::error(code, file, err.to_string()).at(
        manifest_err.and_then(|m| m.line),
        manifest_err.and_then(|m| m.col),
    )
}

/// Take everything recorded so far, clearing the buffer.
fn take_diagnostics() -> Vec<CheckDiagnostic> {
    DIAGNOSTICS.with(|d| std::mem::take(&mut *d.borrow_mut()))
}

/// Scan an entire workspace directory
fn check_workspace(target_path: &Path, quiet: bool) -> HorusResult<()> {
    if !quiet {
        println!(
            "{} Scanning workspace: {}\n",
            cli_output::ICON_INFO.cyan().bold(),
            target_path
                .canonicalize()
                .unwrap_or(target_path.to_path_buf())
                .display()
        );
    }

    // Restore the IDE manifest if it is missing OR stale.
    //
    // `.horus/Cargo.toml` is gitignored, so a fresh clone of a HORUS project
    // has no cargo manifest and rust-analyzer cannot load anything. `horus
    // check` is the natural command to run on a project that "isn't working",
    // so it is the right place to repair this. Only when absent or stale,
    // though: rewriting it on every check would churn its mtime and make
    // rust-analyzer reload the whole workspace each time.
    //
    // Stale matters as much as missing. The manifest bakes an absolute path to
    // the HORUS source tree, and `horus upgrade` moves it (the installer cache
    // is keyed `horus@<version>`), so an upgrade breaks every project at once.
    // See cargo_gen::is_stale.
    //
    // Strictly advisory: this must never change run_check's exit code. A
    // project is not invalid because its IDE scaffolding is missing.
    let missing = !target_path.join(".horus/Cargo.toml").exists();
    let stale = !missing && crate::cargo_gen::is_stale(target_path);
    if missing || stale {
        if let crate::cargo_gen::Ensured::Generated = crate::cargo_gen::ensure(target_path) {
            if !quiet {
                let why = if stale {
                    "it pointed at a HORUS source tree that is no longer there"
                } else {
                    "rust-analyzer needs it; it is gitignored"
                };
                println!(
                    "{} Regenerated .horus/Cargo.toml ({why})\n",
                    cli_output::ICON_INFO.cyan()
                );
            }
        }
    }

    let mut total_errors = 0;
    let mut total_warnings = 0;
    let mut files_checked = 0;
    let mut horus_manifests: Vec<PathBuf> = Vec::new();
    let mut rust_files: Vec<PathBuf> = Vec::new();
    let mut python_files: Vec<PathBuf> = Vec::new();
    let mut cpp_files: Vec<PathBuf> = Vec::new();

    // Collect all files to check
    for entry in WalkDir::new(target_path)
        .into_iter()
        .filter_entry(|e| {
            // Allow the root entry (depth 0) through unconditionally so
            // `horus check .` doesn't get rejected by the dot-prefix rule.
            if e.depth() == 0 {
                return true;
            }
            let name = e.file_name().to_string_lossy();
            !name.starts_with('.')
                && name != "target"
                && name != "node_modules"
                && name != "__pycache__"
        })
        .filter_map(|e| e.ok())
    {
        let path = entry.path();
        if path.is_file() {
            let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
            let ext = path.extension().and_then(|e| e.to_str());

            if filename == HORUS_TOML {
                horus_manifests.push(path.to_path_buf());
            } else if ext == Some("rs") && filename != "build.rs" {
                rust_files.push(path.to_path_buf());
            } else if ext == Some("py") {
                python_files.push(path.to_path_buf());
            } else if ext.is_some_and(|e| crate::manifest::CPP_SOURCE_EXTENSIONS.contains(&e)) {
                cpp_files.push(path.to_path_buf());
            }
        }
    }

    if !quiet {
        println!("  Found {} horus.toml file(s)", horus_manifests.len());
        println!("  Found {} Rust file(s)", rust_files.len());
        println!("  Found {} Python file(s)", python_files.len());
        // Named, and named as unchecked. `check` has no C++ phase — a project
        // whose main.cpp does not compile still reported "All checks passed!",
        // which reads as "your C++ is fine" rather than "your C++ was never
        // looked at". A compiler run here would need the project's include
        // paths, which is what `horus build` is.
        if !cpp_files.is_empty() {
            println!(
                "  Found {} C++ file(s) — not syntax-checked here; `horus build` compiles them",
                cpp_files.len()
            );
        }
        println!();
    }

    // Find Cargo.toml directories for deep Rust checking
    let mut cargo_dirs: HashSet<PathBuf> = HashSet::new();
    for entry in WalkDir::new(target_path)
        .into_iter()
        .filter_entry(|e| {
            if e.depth() == 0 {
                return true;
            }
            let name = e.file_name().to_string_lossy();
            !name.starts_with('.') && name != "target" && name != "node_modules"
        })
        .filter_map(|e| e.ok())
    {
        if entry.file_name() == CARGO_TOML {
            if let Some(parent) = entry.path().parent() {
                cargo_dirs.insert(parent.to_path_buf());
            }
        }
    }

    // ═══════════════════════════════════════════════════════════
    // PHASE 1: Validate horus.toml manifests
    // ═══════════════════════════════════════════════════════════
    if !horus_manifests.is_empty() {
        if !quiet {
            println!("{}", "-".repeat(60).dimmed());
            println!(
                "{} Phase 1: Validating horus.toml manifests...\n",
                cli_output::ICON_INFO.cyan().bold()
            );
        }

        for toml_path in &horus_manifests {
            let rel_path = toml_path.strip_prefix(target_path).unwrap_or(toml_path);
            if !quiet {
                println!("  {} {}", ">".cyan(), rel_path.display());
            }

            let source = fs::read_to_string(toml_path).unwrap_or_default();
            let rel = rel_path.display().to_string();

            match HorusManifest::load_from(toml_path) {
                Ok(manifest) => {
                    let mut file_errors: Vec<CheckDiagnostic> = Vec::new();
                    let base_dir = toml_path.parent().unwrap_or(Path::new("."));

                    // Run manifest validation. One finding per problem, each
                    // pointed at the key it is about: `--json` used to carry a
                    // single entry with three errors joined into its message
                    // and no line at all.
                    let (errors, warnings) = manifest.validate_issues();
                    for w in &warnings {
                        let (line, col) = locate_issue(&source, w.key.as_deref());
                        let d = CheckDiagnostic::warning("manifest.validation", &rel, &w.message)
                            .at(line, col);
                        println!("      {} {}", "!".yellow(), located(&d));
                        d.record();
                        total_warnings += 1;
                    }
                    for e in errors {
                        let (line, col) = locate_issue(&source, e.key.as_deref());
                        file_errors.push(
                            CheckDiagnostic::error("manifest.invalid", &rel, e.message)
                                .at(line, col),
                        );
                    }

                    // Keys serde discarded. These never reach `manifest`, so
                    // they have to be read back off the source text — without
                    // this, a misspelled key silently does nothing and `check`
                    // reports the file as valid. They are errors as of 0.3.0,
                    // which is what the 0.2.x warning said would happen: a
                    // gate that exits 0 over a setting that does nothing is
                    // the defect this check exists to catch.
                    let unknown = manifest_lint::find_unknown_keys(&source);
                    for key in &unknown {
                        file_errors.push(
                            CheckDiagnostic::error("manifest.unknown-key", &rel, key.message())
                                .on_line(key.line),
                        );
                    }

                    // Check main file exists based on detected language
                    let languages = detect_languages(base_dir);
                    for lang in &languages {
                        let main_exists = match lang {
                            Language::Rust => {
                                base_dir.join("main.rs").exists()
                                    || base_dir.join("src/main.rs").exists()
                                    || base_dir.join(CARGO_TOML).exists()
                            }
                            // Both layouts, mirroring Rust above. `horus build`
                            // and `horus run` have always detected src/main.py
                            // (see dispatch.rs); only `check` insisted on the
                            // project root, so a src/-layout Python project
                            // built and ran but failed validation.
                            Language::Python => {
                                base_dir.join("main.py").exists()
                                    || base_dir.join("src/main.py").exists()
                            }
                            _ => true,
                        };
                        if !main_exists {
                            file_errors.push(CheckDiagnostic::error(
                                "project.main-missing",
                                &rel,
                                format!("main file not found for '{:?}'", lang),
                            ));
                        }
                    }

                    if file_errors.is_empty() {
                        if !quiet {
                            println!("      {} manifest valid", cli_output::ICON_SUCCESS.green());
                        }
                    } else {
                        for d in &file_errors {
                            println!("      {} {}", cli_output::ICON_ERROR.red(), located(d));
                        }
                        if !unknown.is_empty() {
                            println!("        {}", manifest_lint::UnknownKey::severity_notice());
                        }
                        total_errors += file_errors.len();
                    }
                    for d in file_errors {
                        d.record();
                    }
                }
                Err(e) => {
                    // No hardcoded "TOML parse error" prefix: the error says
                    // for itself whether the file failed to parse or parsed
                    // and lacked a required key, and the two used to be
                    // announced identically — including the one whose own
                    // body then said "The file is valid TOML".
                    let d = describe_manifest_failure(&e, &rel);
                    let mut lines = d.message.lines();
                    if let Some(first) = lines.next() {
                        println!("      {} {}", cli_output::ICON_ERROR.red(), first);
                    }
                    for rest in lines {
                        if rest.is_empty() {
                            println!();
                        } else {
                            println!("        {rest}");
                        }
                    }
                    // `--json` reported "diagnostics": [] for every manifest
                    // failure, so the machine-readable surface carried none of
                    // the position the human one had just computed.
                    d.record();
                    total_errors += 1;
                }
            }
            files_checked += 1;
        }
    }

    // ═══════════════════════════════════════════════════════════
    // PHASE 2: Deep Rust compilation check (cargo check)
    // ═══════════════════════════════════════════════════════════
    if !cargo_dirs.is_empty() {
        if !quiet {
            println!("\n{}", "-".repeat(60).dimmed());
            println!(
                "{} Phase 2: Deep Rust check (cargo check)...\n",
                cli_output::ICON_INFO.cyan().bold()
            );
        }

        for cargo_dir in &cargo_dirs {
            let rel_path = cargo_dir.strip_prefix(target_path).unwrap_or(cargo_dir);
            let display_path = if rel_path.as_os_str().is_empty() {
                "."
            } else {
                rel_path.to_str().unwrap_or(".")
            };
            if !quiet {
                print!("  {} {} ... ", ">".cyan(), display_path);
                std::io::Write::flush(&mut std::io::stdout()).ok();
            }

            let output = std::process::Command::new("cargo")
                .arg("check")
                .arg("--message-format=short")
                .current_dir(cargo_dir)
                .output();

            match output {
                Ok(result) if result.status.success() => {
                    if !quiet {
                        println!("{}", cli_output::ICON_SUCCESS.green());
                    }
                }
                Ok(result) => {
                    println!("{}", cli_output::ICON_ERROR.red());
                    let stderr = String::from_utf8_lossy(&result.stderr);
                    for line in stderr.lines().take(5) {
                        if !line.contains("error") {
                            continue;
                        }
                        let line = line.trim();
                        println!("      {} {}", cli_output::ICON_ERROR.red(), line);
                        // The human output names the error; --json used to
                        // report `"diagnostics": []` beside it, and then a
                        // version that put the *cargo directory* in `file` and
                        // left the source path buried in the prose. Python's
                        // phase had carried a real file and line all along —
                        // an editor consuming this got a jump target for one
                        // language and not the other.
                        match split_short_diagnostic(line) {
                            Some((file, line_no, col, message)) => {
                                let full = cargo_dir.join(file);
                                let shown = full
                                    .strip_prefix(target_path)
                                    .unwrap_or(&full)
                                    .display()
                                    .to_string();
                                CheckDiagnostic::error("rust.compile", shown, message)
                                    .at(Some(line_no), Some(col))
                                    .record();
                            }
                            None => {
                                CheckDiagnostic::error("rust.compile", display_path, line).record()
                            }
                        }
                    }
                    total_errors += 1;
                }
                Err(e) => {
                    println!("{} cargo error: {}", cli_output::ICON_WARN.yellow(), e);
                    CheckDiagnostic::warning(
                        "rust.toolchain",
                        rel_path.display().to_string(),
                        format!("cargo could not be run: {e}"),
                    )
                    .record();
                    total_warnings += 1;
                }
            }
            files_checked += 1;
        }
    }

    // ═══════════════════════════════════════════════════════════
    // PHASE 3: Python validation (syntax + imports)
    // ═══════════════════════════════════════════════════════════
    if !python_files.is_empty() {
        if !quiet {
            println!("\n{}", "-".repeat(60).dimmed());
            println!(
                "{} Phase 3: Python validation (syntax + imports)...\n",
                cli_output::ICON_INFO.cyan().bold()
            );
        }

        for py_path in &python_files {
            if !quiet {
                print!(
                    "  {} {} ",
                    ">".cyan(),
                    py_path
                        .strip_prefix(target_path)
                        .unwrap_or(py_path)
                        .display()
                );
            }

            // Syntax check
            let syntax_check = std::process::Command::new("python3")
                .arg("-m")
                .arg("py_compile")
                .arg(py_path)
                .output();

            match syntax_check {
                Ok(result) if result.status.success() => {
                    // Syntax OK - now check imports
                    let import_script = r#"
import ast, sys
try:
    with open(sys.argv[1]) as f:
        tree = ast.parse(f.read())
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.add(alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module.split('.')[0])
    for imp in imports:
        if imp not in ('__future__',):
            __import__(imp)
except ModuleNotFoundError as e:
    print(f'ModuleNotFoundError: {e.name}', file=sys.stderr)
    sys.exit(1)
except ImportError as e:
    print(f'ImportError: {e}', file=sys.stderr)
    sys.exit(1)
"#;

                    // The braces above are single, not doubled: this string is
                    // handed to python3 as-is. It was written as if it went
                    // through `format!`, so every missing import was reported
                    // as the literal `ModuleNotFoundError: {e.name}` — the one
                    // thing the message exists to say, replaced by its own
                    // placeholder.
                    //
                    // The filename comes from the repo being checked, so it is
                    // attacker-controlled for anyone who can get a repo cloned.
                    // It used to be interpolated straight into this script's
                    // source, where a quote or newline in a .py filename becomes
                    // Python code executed by `horus check`. Pass it as argv
                    // instead — no quoting, no escaping, nothing to get wrong.
                    let import_check = std::process::Command::new("python3")
                        .arg("-c")
                        .arg(import_script)
                        .arg(py_path)
                        .output();

                    match import_check {
                        Ok(r) if r.status.success() => {
                            if !quiet {
                                println!("{}", cli_output::ICON_SUCCESS.green());
                            }
                        }
                        Ok(r) => {
                            println!("{}", cli_output::ICON_WARN.yellow());
                            let err = String::from_utf8_lossy(&r.stderr);
                            let first = err.lines().next().unwrap_or("").trim();
                            if !err.is_empty() {
                                println!("      {} {}", cli_output::ICON_WARN.yellow(), first);
                            }
                            // A warning the terminal shows and `--json` does
                            // not is the same defect as an error it drops: the
                            // machine surface has to carry everything the
                            // human one does, not the subset that happened to
                            // be wired up.
                            CheckDiagnostic::warning(
                                "python.import",
                                py_path
                                    .strip_prefix(target_path)
                                    .unwrap_or(py_path)
                                    .display()
                                    .to_string(),
                                if first.is_empty() {
                                    "an import could not be resolved".to_string()
                                } else {
                                    first.to_string()
                                },
                            )
                            .record();
                            total_warnings += 1;
                        }
                        Err(_) => {
                            if !quiet {
                                println!("{}", cli_output::ICON_SUCCESS.green());
                            }
                        }
                    }
                }
                Ok(result) => {
                    println!("{}", cli_output::ICON_ERROR.red());
                    let error = String::from_utf8_lossy(&result.stderr);
                    let first = error.lines().next().unwrap_or("").trim().to_string();
                    if !error.is_empty() {
                        println!("      {} {}", cli_output::ICON_ERROR.red(), first);
                    }
                    CheckDiagnostic::error(
                        "python.syntax",
                        py_path
                            .strip_prefix(target_path)
                            .unwrap_or(py_path)
                            .display()
                            .to_string(),
                        if first.is_empty() {
                            "syntax check failed".to_string()
                        } else {
                            first.clone()
                        },
                    )
                    // Python reports `File "x.py", line N`; carry the number
                    // when it is there rather than dropping it.
                    .at(python_error_line(&first), None)
                    .record();
                    total_errors += 1;
                }
                Err(_) => {
                    println!("{}", "⊘".dimmed());
                    if !quiet {
                        total_warnings += 1;
                    }
                }
            }
            files_checked += 1;
        }
    }

    // Summary
    if !quiet {
        println!("\n{}", "-".repeat(60).dimmed());
        println!(
            "{} Workspace Check Summary\n",
            cli_output::ICON_INFO.cyan().bold()
        );
        println!("  Files checked: {}", files_checked);
    }

    if total_errors == 0 && total_warnings == 0 {
        if !quiet {
            // Never "All checks passed" over a language that has no phase.
            // That sentence is why a project with a main.cpp that does not
            // compile came back clean.
            if cpp_files.is_empty() {
                println!(
                    "  Status: {} All checks passed!",
                    cli_output::ICON_SUCCESS.green()
                );
            } else {
                println!(
                    "  Status: {} All checks passed — C++ sources were not among them ({} file(s); run `horus build`)",
                    cli_output::ICON_SUCCESS.green(),
                    cpp_files.len()
                );
            }
        }
    } else {
        if total_errors > 0 {
            println!(
                "  Errors: {} {}",
                cli_output::ICON_ERROR.red(),
                total_errors
            );
        }
        if total_warnings > 0 && !quiet {
            println!(
                "  Warnings: {} {}",
                cli_output::ICON_WARN.yellow(),
                total_warnings
            );
        }
    }
    println!();

    if total_errors > 0 {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "{} error(s) found",
            total_errors
        ))));
    }
    Ok(())
}

/// Check a single file (horus.toml, .rs, or .py)
fn check_single_file(file_path: &Path, quiet: bool) -> HorusResult<()> {
    let extension = file_path.extension().and_then(|s| s.to_str()).unwrap_or("");
    let name = file_path.file_name().and_then(|s| s.to_str()).unwrap_or("");

    match extension {
        "rs" => check_rust_file(file_path),
        "py" => check_python_file(file_path),
        // Another tool's manifest is valid TOML, so it parsed — and every key
        // in it then came back as unknown. Naming the file is far more use
        // than fifteen findings about a file HORUS does not own.
        "toml" if matches!(name, CARGO_TOML | "pyproject.toml") => {
            foreign_manifest(file_path, name)
        }
        "toml" => check_manifest_file(file_path, quiet),
        // Anything else used to be *parsed as a manifest*, so `horus check
        // main.cpp` answered `Failed to parse manifest: main.cpp:1:5: key with
        // no value, expected \`=\`` — a TOML diagnostic about a C++ file.
        _ => unsupported_file(file_path),
    }
}

/// Point out that this manifest belongs to another tool.
fn foreign_manifest(path: &Path, name: &str) -> HorusResult<()> {
    let display = path.display().to_string();
    let owner = if name == CARGO_TOML {
        "Cargo"
    } else {
        "Python"
    };
    let message = format!("{display} is a {owner} manifest, not a horus.toml");
    let advice = "`horus check <directory>` checks the project, horus.toml included.";

    println!("{} {}", cli_output::ICON_ERROR.red().bold(), message);
    println!("  {advice}");
    CheckDiagnostic::error(
        "check.foreign-manifest",
        &display,
        format!("{message}. {advice}"),
    )
    .record();
    Err(HorusError::Config(ConfigError::Other(message)))
}

/// Say plainly that this file is not one `check` knows how to read.
fn unsupported_file(path: &Path) -> HorusResult<()> {
    use crate::manifest::CPP_SOURCE_EXTENSIONS;

    let display = path.display().to_string();
    let extension = path.extension().and_then(|s| s.to_str()).unwrap_or("");
    let advice = if CPP_SOURCE_EXTENSIONS.contains(&extension) {
        "A C++ file is compiled with the include paths its project supplies, which a \
         single file does not carry. Run `horus check <directory>` for the project, or \
         `horus build` to compile it."
    } else {
        "`horus check` reads horus.toml manifests, .rs and .py files. Point it at a \
         project directory to check all of them."
    };
    let message = format!("horus check cannot check {display}");

    println!("{} {}", cli_output::ICON_ERROR.red().bold(), message);
    println!("  {advice}");
    CheckDiagnostic::error(
        "check.unsupported-file",
        &display,
        format!("{message}. {advice}"),
    )
    .record();
    Err(HorusError::Config(ConfigError::Other(message)))
}

/// Check a single Rust file
fn check_rust_file(path: &Path) -> HorusResult<()> {
    println!(
        "{} Checking Rust file: {}\n",
        cli_output::ICON_INFO.cyan(),
        path.display()
    );

    print!("  {} Parsing Rust syntax... ", ">".cyan());
    let content = fs::read_to_string(path)?;

    match syn::parse_file(&content) {
        Ok(_) => {
            println!("{}", cli_output::ICON_SUCCESS.green());
            println!(
                "\n{} Syntax check passed!",
                cli_output::ICON_SUCCESS.green().bold()
            );
        }
        Err(e) => {
            println!("{}", cli_output::ICON_ERROR.red());
            println!("\n{} Syntax error:", cli_output::ICON_ERROR.red().bold());
            println!("  {}", e);
            // `--json` returned `"diagnostics": []` for this whole path while
            // the `error` string carried the message: the machine-readable
            // surface was empty for exactly the file the user asked about.
            CheckDiagnostic::error("rust.syntax", path.display().to_string(), e.to_string())
                .record();
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Rust syntax error: {}",
                e
            ))));
        }
    }

    Ok(())
}

/// Check a single Python file
fn check_python_file(path: &Path) -> HorusResult<()> {
    println!(
        "{} Checking Python file: {}\n",
        cli_output::ICON_INFO.cyan(),
        path.display()
    );

    print!("  {} Parsing Python syntax... ", ">".cyan());

    let output = std::process::Command::new("python3")
        .arg("-m")
        .arg("py_compile")
        .arg(path)
        .output();

    match output {
        Ok(result) if result.status.success() => {
            println!("{}", cli_output::ICON_SUCCESS.green());
            println!(
                "\n{} Syntax check passed!",
                cli_output::ICON_SUCCESS.green().bold()
            );
        }
        Ok(result) => {
            println!("{}", cli_output::ICON_ERROR.red());
            let error = String::from_utf8_lossy(&result.stderr);
            println!("\n{} Syntax error:", cli_output::ICON_ERROR.red().bold());
            println!("  {}", error);
            // Same hole as the Rust file above: the terminal named the
            // problem and `--json` shipped an empty array.
            let first = error
                .lines()
                .find(|l| l.contains("Error"))
                .unwrap_or_else(|| error.lines().next().unwrap_or(""))
                .trim();
            CheckDiagnostic::error(
                "python.syntax",
                path.display().to_string(),
                if first.is_empty() {
                    "syntax check failed".to_string()
                } else {
                    first.to_string()
                },
            )
            .at(python_error_line(&error), None)
            .record();
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Python syntax error: {}",
                error
            ))));
        }
        Err(e) => {
            println!("{}", "[WARNING]".yellow());
            println!(
                "\n{} Could not check Python syntax (python3 not found): {}",
                "[WARNING]".yellow(),
                e
            );
            CheckDiagnostic::warning(
                "python.toolchain",
                path.display().to_string(),
                format!("python3 could not be run: {e}"),
            )
            .record();
        }
    }
    Ok(())
}

/// Check a horus.toml manifest file
fn check_manifest_file(manifest_path: &Path, quiet: bool) -> HorusResult<()> {
    println!(
        "{} Checking {}...\n",
        cli_output::ICON_INFO.cyan(),
        manifest_path.display()
    );

    let mut errors: Vec<String> = Vec::new();
    let mut warn_msgs: Vec<String> = Vec::new();
    // Findings about the manifest itself, which carry a kind and a position.
    // The two lists above are prose from the environment checks below — a
    // missing toolchain has no line in the file — and stay strings.
    let mut precise: Vec<CheckDiagnostic> = Vec::new();
    let mut unknown_keys_found = false;
    let file = manifest_path.display().to_string();
    let source = fs::read_to_string(manifest_path).unwrap_or_default();
    let base_dir = manifest_path.parent().unwrap_or(Path::new("."));
    let languages = detect_languages(base_dir);

    // 1. TOML Syntax Validation
    print!("  {} Validating TOML syntax... ", ">".cyan());
    let manifest = match HorusManifest::load_from(manifest_path) {
        Ok(m) => {
            println!("{}", cli_output::ICON_SUCCESS.green());
            Some(m)
        }
        Err(e) => {
            println!("{}", cli_output::ICON_ERROR.red());
            // Not "Failed to parse manifest: ...". A manifest that omits
            // `name` parses perfectly well, and wrapping it in a parse error
            // sent the reader looking for a syntax mistake — over a message
            // whose own next line says "The file is valid TOML".
            precise.push(describe_manifest_failure(&e, &file));
            None
        }
    };

    if let Some(ref manifest) = manifest {
        // 2. Manifest Validation (name, version, etc.)
        print!("  {} Validating manifest fields... ", ">".cyan());
        let (field_errors, field_warnings) = manifest.validate_issues();
        if field_errors.is_empty() {
            println!("{}", cli_output::ICON_SUCCESS.green());
        } else {
            println!("{}", cli_output::ICON_ERROR.red());
        }
        for issue in field_errors {
            let (line, col) = locate_issue(&source, issue.key.as_deref());
            precise.push(
                CheckDiagnostic::error("manifest.invalid", &file, issue.message).at(line, col),
            );
        }
        for issue in field_warnings {
            let (line, col) = locate_issue(&source, issue.key.as_deref());
            precise.push(
                CheckDiagnostic::warning("manifest.validation", &file, issue.message).at(line, col),
            );
        }

        // 2b. Keys HORUS does not understand.
        //
        // This path had no unknown-key check at all: `horus check horus.toml`
        // read the same file as `horus check .` and reported "All checks
        // passed!" over the typo the directory form had just flagged.
        print!("  {} Checking for unknown keys... ", ">".cyan());
        let unknown = manifest_lint::find_unknown_keys(&source);
        unknown_keys_found = !unknown.is_empty();
        if unknown.is_empty() {
            println!("{}", cli_output::ICON_SUCCESS.green());
        } else {
            println!("{}", cli_output::ICON_ERROR.red());
            for key in &unknown {
                precise.push(
                    CheckDiagnostic::error("manifest.unknown-key", &file, key.message())
                        .on_line(key.line),
                );
            }
        }

        // Optional fields warning
        if !quiet {
            if manifest.package.description.is_none() {
                warn_msgs.push("Optional field missing: description".to_string());
            }
            if manifest.package.authors.is_empty() {
                warn_msgs.push("Optional field missing: authors".to_string());
            }
        }

        // License warning
        print!("  {} Checking license field... ", ">".cyan());
        let missing_license_warning = "No license specified. Consider adding a license field (e.g., Apache-2.0, BSD-3-Clause).";
        if let Some(ref license) = manifest.package.license {
            if license.trim().is_empty() {
                println!("{}", "[WARNING]".yellow());
                warn_msgs.push(missing_license_warning.to_string());
            } else {
                println!(
                    "{} ({})",
                    cli_output::ICON_SUCCESS.green(),
                    license.dimmed()
                );
            }
        } else {
            println!("{}", "[WARNING]".yellow());
            warn_msgs.push(missing_license_warning.to_string());
        }

        // Dependency source check — warn about Simple deps that are actually well-known
        // crates.io or PyPI packages but aren't tagged with an explicit source.
        print!("  {} Checking dependency sources... ", ">".cyan());
        {
            use crate::source_resolver::{Confidence, PackageSourceResolver};
            let resolver = PackageSourceResolver::new(&languages);
            let mut ambiguous_count = 0u32;

            for (name, dep) in manifest
                .dependencies
                .iter()
                .chain(manifest.dev_dependencies.iter())
            {
                if dep.effective_source() == crate::manifest::DepSource::Registry {
                    let resolved = resolver.resolve(name);
                    if resolved.confidence >= Confidence::High {
                        ambiguous_count += 1;
                        if !quiet {
                            warn_msgs.push(format!(
                                "Dep '{}' defaults to registry but looks like a {} package. \
                                 Consider: horus add {} (or set source explicitly in horus.toml)",
                                name, resolved.source, name
                            ));
                        }
                    }
                }
            }

            if ambiguous_count == 0 {
                println!("{}", cli_output::ICON_SUCCESS.green());
            } else {
                println!(
                    "{} ({} ambiguous)",
                    cli_output::ICON_WARN.yellow(),
                    ambiguous_count
                );
            }
        }

        // Language detection
        print!("  {} Detecting project language... ", ">".cyan());
        if languages.is_empty() {
            println!("{}", cli_output::ICON_WARN.yellow());
            if !quiet {
                warn_msgs.push(
                    "No language detected - no Cargo.toml, pyproject.toml, setup.py, or CMakeLists.txt found".to_string(),
                );
            }
        } else {
            let lang_names: Vec<&str> = languages
                .iter()
                .map(|l| match l {
                    Language::Rust => "rust",
                    Language::Python => "python",
                    Language::Cpp => "cpp",
                    Language::Ros2 => "ros2",
                })
                .collect();
            println!(
                "{} ({})",
                cli_output::ICON_SUCCESS.green(),
                lang_names.join(", ")
            );
        }

        // Main file existence check
        print!("  {} Checking for main file... ", ">".cyan());
        if !languages.is_empty() {
            let mut main_files: Vec<&str> = Vec::new();
            if languages.contains(&Language::Rust) {
                main_files.extend_from_slice(&["main.rs", "src/main.rs"]);
            }
            if languages.contains(&Language::Python) {
                main_files.push("main.py");
            }

            let mut found = false;
            for main_file in &main_files {
                let path = base_dir.join(main_file);
                if path.exists() {
                    println!("{}", cli_output::ICON_SUCCESS.green());
                    found = true;
                    break;
                }
            }

            if !found && !main_files.is_empty() {
                println!("{}", cli_output::ICON_WARN.yellow());
                if !quiet {
                    warn_msgs.push(format!(
                        "No main file found - expected one of: {}",
                        main_files.join(", ")
                    ));
                }
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    }

    // 3. Workspace Structure Check
    print!("\n  {} Checking workspace structure... ", ">".cyan());
    let horus_dir = base_dir.join(".horus");

    if horus_dir.exists() && horus_dir.is_dir() {
        println!("{}", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", cli_output::ICON_WARN.yellow());
        if !quiet {
            warn_msgs.push(
                "No .horus/ workspace directory found - will be created on first run".to_string(),
            );
        }
    }

    // 4. Toolchain Check
    print!("  {} Checking toolchain... ", ">".cyan());
    if !languages.is_empty() {
        let mut all_available = true;
        for lang in &languages {
            let tool_name = match lang {
                Language::Rust => "rustc",
                Language::Python => "python3",
                Language::Cpp => "cmake",
                Language::Ros2 => "ros2",
            };

            if crate::dispatch::tool_version(tool_name).is_none() {
                all_available = false;
                errors.push(format!(
                    "Required toolchain for '{:?}' not found in PATH",
                    lang
                ));
            }
        }

        if all_available {
            println!("{}", cli_output::ICON_SUCCESS.green());
        } else {
            println!("{}", cli_output::ICON_ERROR.red());
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // 5. Code Validation
    print!("  {} Validating code syntax... ", ">".cyan());
    if !languages.is_empty() {
        let mut validated = false;
        if languages.contains(&Language::Rust) {
            let has_cargo = base_dir.join(CARGO_TOML).exists();
            let has_main =
                base_dir.join("main.rs").exists() || base_dir.join("src/main.rs").exists();

            if has_cargo || has_main {
                validated = true;
                let check_result = std::process::Command::new("cargo")
                    .arg("build")
                    .arg("--quiet")
                    .current_dir(base_dir)
                    .output();

                match check_result {
                    Ok(output) if output.status.success() => {
                        println!("{}", cli_output::ICON_SUCCESS.green());
                    }
                    Ok(_) => {
                        println!("{}", cli_output::ICON_ERROR.red());
                        errors.push(
                            "Rust code has compilation errors (run 'cargo build' for details)"
                                .to_string(),
                        );
                    }
                    Err(_) => {
                        println!("{}", cli_output::ICON_WARN.yellow());
                        if !quiet {
                            warn_msgs.push(
                                "Could not run 'cargo build' - skipping code validation"
                                    .to_string(),
                            );
                        }
                    }
                }
            }
        }
        if languages.contains(&Language::Python) {
            let main_py = base_dir.join("main.py");
            if main_py.exists() {
                validated = true;
                let check_result = std::process::Command::new("python3")
                    .arg("-m")
                    .arg("py_compile")
                    .arg(&main_py)
                    .output();

                match check_result {
                    Ok(output) if output.status.success() => {
                        if !languages.contains(&Language::Rust) {
                            println!("{}", cli_output::ICON_SUCCESS.green());
                        }
                    }
                    Ok(_) => {
                        println!("{}", cli_output::ICON_ERROR.red());
                        errors.push("Python code has syntax errors".to_string());
                    }
                    Err(_) => {
                        println!("{}", cli_output::ICON_WARN.yellow());
                        if !quiet {
                            warn_msgs.push("Could not validate Python syntax".to_string());
                        }
                    }
                }
            }
        }
        if !validated {
            println!("{}", "⊘".dimmed());
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // 12. HORUS System Check
    print!("\n  {} Checking HORUS installation... ", ">".cyan());
    let horus_version = env!("CARGO_PKG_VERSION");
    println!("v{}", horus_version.dimmed());

    // 13. Registry Connectivity
    print!("  {} Checking registry connectivity... ", ">".cyan());
    let registry_available = std::process::Command::new("ping")
        .arg("-c")
        .arg("1")
        .arg("-W")
        .arg("1")
        .arg("registry.horus.rs")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false);

    if registry_available {
        println!("{}", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", "⊘".dimmed());
        if !quiet {
            warn_msgs.push("Registry not reachable - package installation may fail".to_string());
        }
    }

    // 14. System Requirements Check
    print!("  {} Checking system requirements... ", ">".cyan());
    let mut sys_issues = Vec::new();

    {
        let shm_parent = horus_sys::shm::shm_parent_dir();
        if !shm_parent.exists() {
            sys_issues.push("SHM parent directory not available");
        } else if std::fs::create_dir_all(shm_base_dir()).is_err() {
            sys_issues.push("Cannot create shared memory directory");
        }
    }

    if sys_issues.is_empty() {
        println!("{}", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", cli_output::ICON_WARN.yellow());
        for issue in sys_issues {
            if !quiet {
                warn_msgs.push(format!("System issue: {}", issue));
            }
        }
    }

    // Disk Space Check (cross-platform via horus_sys)
    print!("  {} Checking available disk space... ", ">".cyan());
    if let Some(available_mb) = horus_sys::platform::disk_available_mb(base_dir) {
        if available_mb < 100 {
            println!("{} ({}MB free)", cli_output::ICON_ERROR.red(), available_mb);
            errors.push(format!(
                "Critically low disk space: only {}MB available",
                available_mb
            ));
        } else if available_mb < 500 {
            println!(
                "{} ({}MB free)",
                cli_output::ICON_WARN.yellow(),
                available_mb
            );
            if !quiet {
                warn_msgs.push(format!(
                    "Low disk space: only {}MB available (recommended: 500MB+)",
                    available_mb
                ));
            }
        } else {
            println!(
                "{} ({}MB free)",
                cli_output::ICON_SUCCESS.green(),
                available_mb
            );
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // 9. API Usage Check
    print!("  {} Checking API usage... ", ">".cyan());
    if languages.contains(&Language::Rust) {
        // Check if horus.toml or Cargo.toml references horus
        let uses_horus = {
            let horus_toml_path = base_dir.join(HORUS_TOML);
            if horus_toml_path.exists() {
                HorusManifest::load_from(&horus_toml_path)
                    .map(|m| m.dependencies.keys().any(|k| k.contains("horus")))
                    .unwrap_or(false)
            } else {
                let cargo_toml_path = base_dir.join(CARGO_TOML);
                if cargo_toml_path.exists() {
                    std::fs::read_to_string(&cargo_toml_path)
                        .map(|c| c.contains("horus") || c.contains("horus_macros"))
                        .unwrap_or(false)
                } else {
                    false
                }
            }
        };

        if uses_horus {
            let main_paths = vec![base_dir.join("main.rs"), base_dir.join("src/main.rs")];

            let mut has_scheduler = false;
            for main_path in main_paths {
                if main_path.exists() {
                    if let Ok(content) = std::fs::read_to_string(&main_path) {
                        has_scheduler = content.contains("Scheduler::new")
                            || content.contains("scheduler.register");
                        if has_scheduler {
                            break;
                        }
                    }
                }
            }

            if has_scheduler {
                println!("{}", cli_output::ICON_SUCCESS.green());
            } else {
                println!("{}", cli_output::ICON_WARN.yellow());
                if !quiet {
                    warn_msgs
                        .push("HORUS dependency found but no Scheduler usage detected".to_string());
                }
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    } else if languages.contains(&Language::Python) {
        let main_py = base_dir.join("main.py");
        if main_py.exists() {
            if let Ok(content) = std::fs::read_to_string(&main_py) {
                if content.contains("import horus") || content.contains("from horus") {
                    println!("{}", cli_output::ICON_SUCCESS.green());
                } else {
                    println!("{}", "⊘".dimmed());
                }
            } else {
                println!("{}", "⊘".dimmed());
            }
        } else {
            println!("{}", "⊘".dimmed());
        }

        // Check external Python dependencies
        print!("  {} Checking Python external dependencies... ", ">".cyan());
        let main_py = base_dir.join("main.py");
        if main_py.exists() {
            match parse_python_imports(&main_py) {
                Ok(imports) if !imports.is_empty() => {
                    let mut missing_packages = Vec::new();
                    for package in &imports {
                        if !check_system_package_exists(package) {
                            missing_packages.push(package.clone());
                        }
                    }

                    if missing_packages.is_empty() {
                        println!("{} ({})", cli_output::ICON_SUCCESS.green(), imports.len());
                    } else {
                        println!("{}", cli_output::ICON_ERROR.red());
                        errors.push(format!(
                            "Missing Python packages: {} (install with: pip install {})",
                            missing_packages.join(", "),
                            missing_packages.join(" ")
                        ));
                    }
                }
                Ok(_) => {
                    println!("{}", "⊘".dimmed());
                }
                Err(e) => {
                    println!("{}", cli_output::ICON_WARN.yellow());
                    if !quiet {
                        warn_msgs.push(format!("Could not parse Python imports: {}", e));
                    }
                }
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // Print Summary
    let warn_lines: Vec<String> = precise
        .iter()
        .filter(|d| d.severity == "warning")
        .map(located)
        .chain(warn_msgs.iter().cloned())
        .collect();
    println!();
    if !quiet {
        if warn_lines.is_empty() {
            println!("{} No warnings detected.", cli_output::ICON_SUCCESS.green());
        } else {
            println!("{} {} warning(s):", "[WARNING]".yellow(), warn_lines.len());
            for warn in &warn_lines {
                println!("  - {}", warn);
            }
        }
        println!();
    }

    // Everything the human output is about to print, recorded so `--json` says
    // it too. This path used to emit `"diagnostics": []` while the terminal
    // listed the errors by name.
    for d in &precise {
        d.clone().record();
    }
    for warn in &warn_msgs {
        CheckDiagnostic::warning("check.warning", &file, warn.clone()).record();
    }
    for err in &errors {
        CheckDiagnostic::error("check.error", &file, err.clone()).record();
    }

    let error_lines: Vec<String> = precise
        .iter()
        .filter(|d| d.severity == "error")
        .map(located)
        .chain(errors.iter().cloned())
        .collect();

    if error_lines.is_empty() {
        println!(
            "{} All checks passed!",
            cli_output::ICON_SUCCESS.green().bold()
        );
        Ok(())
    } else {
        println!(
            "{} {} error(s) found:\n",
            cli_output::ICON_ERROR.red().bold(),
            error_lines.len()
        );
        for (i, err) in error_lines.iter().enumerate() {
            println!("  {}. {}", i + 1, err);
        }
        if unknown_keys_found {
            println!("\n  {}", manifest_lint::UnknownKey::severity_notice());
        }
        println!();
        Err(HorusError::Config(ConfigError::Other(
            "Validation failed".to_string(),
        )))
    }
}

// Helper functions for system package detection

pub fn check_system_package_exists(package_name: &str) -> bool {
    use std::process::Command;

    let py_check = Command::new("python3")
        .args(["-m", "pip", "show", package_name])
        .output();

    if let Ok(output) = py_check {
        if output.status.success() {
            return true;
        }
    }

    if let Some(home) = dirs::home_dir() {
        let cargo_bin = home.join(".cargo/bin").join(package_name);
        if cargo_bin.exists() {
            return true;
        }
    }

    false
}

/// Parse Python imports from a file
pub(crate) fn parse_python_imports(python_file: &Path) -> Result<Vec<String>, std::io::Error> {
    let content = std::fs::read_to_string(python_file)?;
    let mut imports = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();

        if trimmed.starts_with('#') || trimmed.is_empty() {
            continue;
        }

        if let Some(rest) = trimmed.strip_prefix("import ") {
            let module = rest
                .split_whitespace()
                .next()
                .unwrap_or("")
                .split('.')
                .next()
                .unwrap_or("")
                .split(',')
                .next()
                .unwrap_or("")
                .trim();

            if !module.is_empty() && !imports.contains(&module.to_string()) {
                imports.push(module.to_string());
            }
        }

        if let Some(rest) = trimmed.strip_prefix("from ") {
            if let Some(module) = rest.split_whitespace().next() {
                let base_module = module.split('.').next().unwrap_or("");
                if !base_module.is_empty() && !imports.contains(&base_module.to_string()) {
                    imports.push(base_module.to_string());
                }
            }
        }
    }

    let stdlib_modules = vec![
        "os",
        "sys",
        "re",
        "json",
        "math",
        "time",
        "datetime",
        "collections",
        "itertools",
        "functools",
        "pathlib",
        "typing",
        "abc",
        "io",
        "logging",
        "argparse",
        "subprocess",
        "threading",
        "multiprocessing",
        "queue",
        "socket",
        "http",
        "urllib",
        "email",
        "xml",
        "html",
        "random",
        "string",
        "unittest",
        "pytest",
        "asyncio",
        "concurrent",
        "pickle",
        "copy",
        "enum",
        "dataclasses",
        "contextlib",
        "warnings",
        "traceback",
        "pdb",
        "timeit",
    ];

    imports.retain(|module| !stdlib_modules.contains(&module.as_str()) && module != "horus");

    Ok(imports)
}

#[derive(Debug, Clone, PartialEq)]
pub enum MissingSystemChoice {
    InstallGlobal,
    InstallLocal,
    Skip,
}

/// Run full validation: manifest + fmt + lint.
/// Combines multiple checks and reports results using the doctor summary format.
pub fn run_check_full(path: Option<PathBuf>, json: bool) -> HorusResult<()> {
    use super::doctor::{CheckResult, Health};
    let mut results = Vec::new();

    // 1. Manifest validation
    let manifest_ok = run_check(path.clone(), true, false).is_ok();
    results.push(CheckResult {
        category: "Manifest".to_string(),
        health: if manifest_ok {
            Health::Ok
        } else {
            Health::Fail
        },
        summary: if manifest_ok {
            "horus.toml valid".into()
        } else {
            "horus.toml has errors".into()
        },
        details: vec![],
    });

    // 2. Formatting check
    let fmt_ok = super::fmt::run_fmt(true, vec![]).is_ok();
    results.push(CheckResult {
        category: "Formatting".to_string(),
        health: if fmt_ok { Health::Ok } else { Health::Warn },
        summary: if fmt_ok {
            "Code formatted correctly".into()
        } else {
            "Unformatted code found (run horus fmt)".into()
        },
        details: vec![],
    });

    // 3. Lint check
    let lint_ok = super::lint::run_lint(false, false, vec![]).is_ok();
    results.push(CheckResult {
        category: "Lint".to_string(),
        health: if lint_ok { Health::Ok } else { Health::Warn },
        summary: if lint_ok {
            "No lint issues".into()
        } else {
            "Lint warnings found (run horus lint --fix)".into()
        },
        details: vec![],
    });

    if json {
        let json_results: Vec<_> = results
            .iter()
            .map(|r| {
                serde_json::json!({
                    "category": r.category,
                    "health": format!("{:?}", r.health),
                    "summary": r.summary,
                })
            })
            .collect();
        println!(
            "{}",
            serde_json::to_string_pretty(&json_results).unwrap_or_default()
        );
    } else {
        super::doctor::print_summary(&results, false);
    }

    let has_failures = results.iter().any(|r| r.health == Health::Fail);
    if has_failures {
        Err(HorusError::Config(ConfigError::Other(
            "Full check found failures".to_string(),
        )))
    } else {
        Ok(())
    }
}

pub fn prompt_missing_system_package(
    package_name: &str,
) -> Result<MissingSystemChoice, HorusError> {
    use std::io::{self, Write};

    println!(
        "\n  System package '{}' was expected but not found.",
        package_name
    );
    println!("  What would you like to do?");
    println!("    [1] Install to HORUS global cache (shared across projects)");
    println!("    [2] Install to HORUS local (this project only)");
    println!("    [3] Skip (you will install it manually later)");

    print!("\n  Choice [1-3]: ");
    io::stdout()
        .flush()
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    match input.trim() {
        "1" => Ok(MissingSystemChoice::InstallGlobal),
        "2" => Ok(MissingSystemChoice::InstallLocal),
        "3" => Ok(MissingSystemChoice::Skip),
        _ => {
            println!("  Invalid choice, defaulting to Skip");
            Ok(MissingSystemChoice::Skip)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    /// Helper: write a horus.toml with given content into a temp dir and return the path
    fn write_toml(dir: &TempDir, content: &str) -> PathBuf {
        let toml_path = dir.path().join("horus.toml");
        fs::write(&toml_path, content).unwrap();
        toml_path
    }

    // ─── TOML Syntax Tests ───

    #[test]
    fn valid_toml_passes_check() {
        let dir = TempDir::new().unwrap();
        let toml_path = write_toml(
            &dir,
            "[package]\nname = \"my-robot\"\nversion = \"0.1.0\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(
            result.is_ok(),
            "Valid horus.toml should pass: {:?}",
            result.err()
        );
        // Verify the manifest file is still intact after checking
        assert!(
            toml_path.exists(),
            "horus.toml should still exist after check"
        );
        let content = fs::read_to_string(&toml_path).unwrap();
        assert!(
            content.contains("my-robot"),
            "manifest content should be unchanged"
        );
    }

    #[test]
    fn malformed_toml_returns_error() {
        let dir = TempDir::new().unwrap();
        write_toml(&dir, "[package\ninvalid toml content");
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Malformed TOML should fail validation");
    }

    #[test]
    fn empty_toml_returns_error() {
        let dir = TempDir::new().unwrap();
        write_toml(&dir, "");
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Empty TOML should fail");
    }

    // ─── Required Fields Tests ───

    #[test]
    fn missing_name_field_returns_error() {
        let dir = TempDir::new().unwrap();
        write_toml(&dir, "[package]\nversion = \"0.1.0\"\n");
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Missing 'name' should fail");
    }

    #[test]
    fn missing_version_field_returns_error() {
        let dir = TempDir::new().unwrap();
        write_toml(&dir, "[package]\nname = \"my-robot\"\n");
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Missing 'version' should fail");
    }

    // ─── Version Format Tests ───

    #[test]
    fn invalid_version_format_returns_error() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            "[package]\nname = \"my-robot\"\nversion = \"not_a_version\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Invalid semver version should fail");
    }

    #[test]
    fn valid_semver_version_passes() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            "[package]\nname = \"my-robot\"\nversion = \"1.2.3\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(
            result.is_ok(),
            "Valid semver should pass: {:?}",
            result.err()
        );
        // Also verify the manifest can be independently loaded and parsed
        let toml_path = dir.path().join("horus.toml");
        let manifest = HorusManifest::load_from(&toml_path).unwrap();
        assert_eq!(manifest.package.version, "1.2.3");
        assert_eq!(manifest.package.name, "my-robot");
    }

    // ─── Project Name Validation Tests ───

    #[test]
    fn name_with_spaces_returns_error() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            "[package]\nname = \"my robot\"\nversion = \"0.1.0\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Name with spaces should fail");
    }

    #[test]
    fn name_with_special_chars_returns_error() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            "[package]\nname = \"my@robot!\"\nversion = \"0.1.0\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Name with special chars should fail");
    }

    #[test]
    fn name_with_hyphens_and_underscores_passes() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            "[package]\nname = \"my-robot_v2\"\nversion = \"0.1.0\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(
            result.is_ok(),
            "Hyphens and underscores in name should pass: {:?}",
            result.err()
        );
        // Verify the name round-trips correctly through manifest parsing
        let toml_path = dir.path().join("horus.toml");
        let manifest = HorusManifest::load_from(&toml_path).unwrap();
        assert_eq!(
            manifest.package.name, "my-robot_v2",
            "name with hyphens and underscores should round-trip through parsing"
        );
    }

    // ─── Path Not Found Tests ───

    #[test]
    fn nonexistent_path_returns_error() {
        let result = run_check(
            Some(PathBuf::from("/tmp/nonexistent_horus_path_12345")),
            true,
            false,
        );
        assert!(result.is_err(), "Non-existent path should fail");
    }

    // ─── parse_python_imports Tests ───

    #[test]
    fn parse_python_imports_basic() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import numpy\nimport pandas\nfrom torch import nn\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(imports.contains(&"numpy".to_string()));
        assert!(imports.contains(&"pandas".to_string()));
        assert!(imports.contains(&"torch".to_string()));
    }

    #[test]
    fn parse_python_imports_filters_stdlib() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import os\nimport sys\nimport json\nimport numpy\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        // os, sys, json are stdlib — should be filtered
        assert!(!imports.contains(&"os".to_string()));
        assert!(!imports.contains(&"sys".to_string()));
        assert!(!imports.contains(&"json".to_string()));
        // numpy is NOT stdlib — should be included
        assert!(imports.contains(&"numpy".to_string()));
    }

    #[test]
    fn parse_python_imports_handles_comments_and_empty() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "# This is a comment\n\nimport numpy\n# import pandas\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(imports.contains(&"numpy".to_string()));
        assert!(
            !imports.contains(&"pandas".to_string()),
            "Commented imports should be ignored"
        );
    }

    #[test]
    fn parse_python_imports_from_submodule() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(&py_file, "from scipy.spatial import KDTree\n").unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(
            imports.contains(&"scipy".to_string()),
            "Should extract base module from 'from' imports"
        );
    }

    #[test]
    fn parse_python_imports_no_duplicates() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import numpy\nimport numpy\nfrom numpy import array\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        let numpy_count = imports.iter().filter(|i| *i == "numpy").count();
        assert_eq!(numpy_count, 1, "Should not have duplicate imports");
    }

    #[test]
    fn parse_python_imports_empty_file() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(&py_file, "").unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(imports.is_empty(), "Empty file should have no imports");
    }

    #[test]
    fn parse_python_imports_mixed_stdlib_and_external() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import os\nimport numpy\nimport sys\nimport custom_pkg\nfrom json import loads\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        // stdlib filtered
        assert!(!imports.contains(&"os".to_string()));
        assert!(!imports.contains(&"sys".to_string()));
        assert!(!imports.contains(&"json".to_string()));
        // external kept
        assert!(imports.contains(&"numpy".to_string()));
        assert!(imports.contains(&"custom_pkg".to_string()));
        assert_eq!(imports.len(), 2);
    }

    #[test]
    fn parse_python_imports_horus_filtered() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import horus\nfrom horus import Scheduler\nimport numpy\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        // horus is explicitly filtered
        assert!(!imports.contains(&"horus".to_string()));
        assert!(imports.contains(&"numpy".to_string()));
    }

    // ─── System Package Existence Tests ───

    #[test]
    fn check_nonexistent_package_returns_false() {
        assert!(
            !check_system_package_exists("nonexistent_pkg_xyz_12345"),
            "Non-existent package should return false"
        );
    }

    // ─── SHM Availability Tests ───

    #[test]
    fn shm_is_available_on_linux() {
        // HORUS requires /dev/shm on Linux for zero-copy IPC
        assert!(
            horus_core::memory::has_native_shm(),
            "HORUS requires native SHM support"
        );

        #[cfg(target_os = "linux")]
        {
            let dev_shm = std::path::Path::new("/dev/shm");
            assert!(dev_shm.exists(), "/dev/shm must exist for HORUS IPC");
            assert!(dev_shm.is_dir(), "/dev/shm must be a directory");
        }
    }

    // ─── MissingSystemChoice Tests ───

    #[test]
    fn missing_system_choice_variants() {
        // Verify the enum variants exist and are distinguishable
        let install_global = MissingSystemChoice::InstallGlobal;
        let install_local = MissingSystemChoice::InstallLocal;
        let skip = MissingSystemChoice::Skip;

        assert_eq!(install_global, MissingSystemChoice::InstallGlobal);
        assert_eq!(install_local, MissingSystemChoice::InstallLocal);
        assert_eq!(skip, MissingSystemChoice::Skip);
        assert_ne!(install_global, skip);
    }

    // ── Battle-testing: check on full project structure ──────────────────

    #[test]
    fn battle_check_valid_rust_project() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            r#"[package]
name = "my-robot"
version = "0.1.0"
"#,
        );
        // Create a Cargo.toml so it detects Rust
        fs::write(
            dir.path().join("Cargo.toml"),
            "[package]\nname = \"my-robot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(dir.path().join("src")).unwrap();
        fs::write(dir.path().join("src/main.rs"), "fn main() {}").unwrap();

        let result = run_check(Some(dir.path().to_path_buf()), true, false);
        assert!(
            result.is_ok(),
            "Valid Rust project should pass check: {:?}",
            result.err()
        );
    }

    #[test]
    fn battle_check_valid_python_project() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            r#"[package]
name = "py-robot"
version = "0.1.0"
"#,
        );
        fs::write(
            dir.path().join("pyproject.toml"),
            "[project]\nname = \"py-robot\"\n",
        )
        .unwrap();
        fs::write(dir.path().join("main.py"), "print('hello')").unwrap();

        let result = run_check(Some(dir.path().to_path_buf()), true, false);
        assert!(
            result.is_ok(),
            "Valid Python project should pass check: {:?}",
            result.err()
        );
    }

    #[test]
    fn battle_check_directory_without_horus_toml() {
        let dir = TempDir::new().unwrap();
        // No horus.toml at all
        let result = run_check(Some(dir.path().to_path_buf()), true, false);
        // Should still work — check reports warnings but doesn't necessarily error
        // (depends on what check.rs does for a dir without horus.toml)
        // The important thing is it doesn't panic
        let _ = result;
    }

    #[test]
    fn battle_check_json_mode_valid() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            r#"[package]
name = "json-robot"
version = "1.0.0"
"#,
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, true);
        assert!(result.is_ok(), "JSON mode check on valid toml should pass");
    }

    #[test]
    fn battle_check_json_mode_nonexistent() {
        let result = run_check(
            Some(PathBuf::from("/tmp/nonexistent_path_battle_test")),
            true,
            true,
        );
        assert!(
            result.is_err(),
            "JSON mode check on nonexistent path should fail"
        );
    }

    #[test]
    fn battle_check_toml_with_all_sections() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            // `enable` is a *top-level* key, so it has to be written before
            // the first table header. It used to sit after `license` inside
            // `[package]`, where TOML put it in the package table and HORUS
            // dropped it: this fixture, named "all sections", was itself an
            // example of the silent-ignore the unknown-key check now catches.
            r#"enable = ["cuda", "profiling"]

[package]
name = "full-robot"
version = "2.0.0"
description = "A complete robot project"
authors = ["dev@example.com"]
license = "MIT"

[dependencies]
serde = "1.0"
tokio = { version = "1.28", features = ["full"] }

[dev-dependencies]
criterion = "0.5"

[drivers]
camera = "opencv"
lidar = true

[scripts]
sim = "echo simulation"
test-hw = "echo hardware test"
"#,
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(
            result.is_ok(),
            "Full horus.toml with all sections should pass: {:?}",
            result.err()
        );
    }

    #[test]
    fn battle_check_empty_package_name() {
        let dir = TempDir::new().unwrap();
        write_toml(&dir, "[package]\nname = \"\"\nversion = \"0.1.0\"\n");
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Empty package name should fail");
    }

    #[test]
    fn battle_check_reserved_name() {
        let dir = TempDir::new().unwrap();
        write_toml(&dir, "[package]\nname = \"horus\"\nversion = \"0.1.0\"\n");
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(result.is_err(), "Reserved name 'horus' should fail check");
    }

    #[test]
    fn battle_check_version_with_prerelease() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            "[package]\nname = \"pre-robot\"\nversion = \"1.0.0-alpha.1\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(
            result.is_ok(),
            "Pre-release semver should pass: {:?}",
            result.err()
        );
    }

    #[test]
    fn battle_check_version_with_build_metadata() {
        let dir = TempDir::new().unwrap();
        write_toml(
            &dir,
            "[package]\nname = \"build-robot\"\nversion = \"1.0.0+build.123\"\n",
        );
        let result = run_check(Some(dir.path().join("horus.toml")), true, false);
        assert!(
            result.is_ok(),
            "Build metadata version should pass: {:?}",
            result.err()
        );
    }
}
