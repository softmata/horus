//! Structured build-error diagnostics for cargo, pip, and cmake.
//!
//! Translates cryptic native tool errors into actionable HORUS hints.
//! Used by `horus run`, `horus build`, `horus test` when cargo/pip/cmake fails.
//!
//! All hint functions return `Vec<Diagnostic>` — a structured type that supports
//! both colored terminal output (`format_diagnostic`) and machine-readable JSON
//! (`Diagnostic::to_json`).

use colored::*;
use serde::Serialize;
use std::sync::atomic::{AtomicBool, Ordering};

// ── JSON diagnostics mode ───────────────────────────────────────────────────

static JSON_DIAGNOSTICS: AtomicBool = AtomicBool::new(false);

/// Enable or disable JSON diagnostic output mode.
///
/// When enabled, `emit_diagnostics` outputs one JSON object per line to stderr
/// instead of colored text. Designed for AI agents (Claude Code, Cursor, etc.).
pub fn set_json_diagnostics(enabled: bool) {
    JSON_DIAGNOSTICS.store(enabled, Ordering::SeqCst);
}

/// Check if JSON diagnostic mode is active.
pub fn is_json_diagnostics() -> bool {
    JSON_DIAGNOSTICS.load(Ordering::SeqCst)
}

/// Emit a single diagnostic in the appropriate format.
///
/// - JSON mode: one JSON object per line to stderr
/// - Normal mode: colored text to stderr via `format_diagnostic`
pub fn emit_diagnostic(diag: &Diagnostic) {
    if is_json_diagnostics() {
        match serde_json::to_string(diag) {
            Ok(json) => eprintln!("{}", json),
            Err(_) => eprintln!("{}", format_diagnostic(diag)), // fallback to text
        }
    } else {
        eprintln!("{}", format_diagnostic(diag));
    }
}

/// Emit multiple diagnostics.
pub fn emit_diagnostics(diagnostics: &[Diagnostic]) {
    for diag in diagnostics {
        emit_diagnostic(diag);
    }
}

// ── Platform detection (delegated to horus_sys) ─────────────────────────────

// Re-export from horus_sys::platform for backward compatibility.
// All platform detection, package suggestions, and directory management
// live in horus_sys. These re-exports preserve the existing API surface.
pub use horus_sys::platform::detect_distro;
pub use horus_sys::platform::suggest_dev_install;
pub use horus_sys::platform::suggest_install;
pub use horus_sys::platform::Distro;

/// Suggest a build-essential / base-devel install command.
fn suggest_build_essential() -> String {
    horus_sys::platform::suggest_build_tools()
}

// ── Diagnostic types ────────────────────────────────────────────────────────

/// A structured build diagnostic with error code, human hint, and optional auto-fix.
///
/// Produced by `cargo_error_hint`, `pip_error_hint`, `cmake_error_hint`.
/// Consumed by `format_diagnostic` (colored terminal) or `to_json` (AI agents).
#[derive(Debug, Clone, Serialize)]
pub struct Diagnostic {
    /// Tool that produced the error (cargo, pip, cmake, python, preflight)
    pub tool: String,
    /// Error code for lookup (H001, H010, H020, etc.)
    pub code: String,
    /// Severity classification
    pub severity: Severity,
    /// Short error description (plain text, no ANSI)
    pub message: String,
    /// Actionable suggestion (may contain ANSI colors for terminal display)
    pub hint: String,
    /// Auto-applicable fix command, if any
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fix: Option<Fix>,
    /// Documentation URL for this error code
    #[serde(skip_serializing_if = "Option::is_none")]
    pub docs_url: Option<String>,
    /// Source file related to the error
    #[serde(skip_serializing_if = "Option::is_none")]
    pub file: Option<String>,
    /// Line number in the source file
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line: Option<usize>,
}

/// Error severity for filtering and presentation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Error,
    Warning,
    Info,
}

/// An auto-applicable fix suggestion.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Fix {
    /// A single shell command to run.
    Command { command: String },
    /// Multiple fixes to apply in order.
    Multi { fixes: Vec<Fix> },
}

/// Look up error code metadata: `(slug, description)`.
///
/// Returns `None` for unknown codes. Used by `Diagnostic::new` to auto-populate `docs_url`.
pub fn error_code_info(code: &str) -> Option<(&'static str, &'static str)> {
    match code {
        // ── Cargo dependency/toolchain errors ────────────
        "H001" => Some(("missing-crate", "Crate not found on crates.io")),
        "H002" => Some(("version-not-found", "No matching version for crate")),
        "H003" => Some(("missing-feature", "Feature does not exist in crate")),
        "H004" => Some(("missing-linker-lib", "System library not found by linker")),
        "H005" => Some(("missing-pkg-config", "pkg-config tool not installed")),
        "H006" => Some(("missing-openssl", "OpenSSL development headers missing")),
        "H007" => Some(("missing-c-compiler", "C compiler (cc) not found")),
        // ── Pip errors ──────────────────────────────────
        "H010" => Some(("pip-not-found", "Python package not found on PyPI")),
        "H011" => Some(("pip-version-conflict", "Python dependency version conflict")),
        "H012" => Some(("pip-build-wheel", "Failed to build Python package wheel")),
        "H013" => Some((
            "pip-externally-managed",
            "System Python is externally managed",
        )),
        "H014" => Some((
            "pip-permission-denied",
            "Permission denied during pip install",
        )),
        // ── CMake errors ────────────────────────────────
        "H020" => Some(("cmake-missing-package", "CMake package not found")),
        "H021" => Some(("cmake-find-failed", "CMake could not find dependency")),
        "H022" => Some(("cmake-no-cxx", "C++ compiler not found")),
        "H023" => Some(("cmake-no-cc", "C compiler not found for cmake")),
        "H024" => Some(("cmake-not-installed", "cmake tool not installed")),
        "H025" => Some(("cmake-syntax", "CMakeLists.txt configuration error")),
        // ── Extended cargo (source code errors) ─────────
        "H030" => Some(("undeclared-crate", "Undeclared crate or module (E0433)")),
        "H031" => Some(("no-method", "No method found on type (E0599)")),
        "H032" => Some(("unresolved-name", "Cannot find value in scope (E0425)")),
        "H033" => Some(("undeclared-type", "Type not found in scope (E0412)")),
        "H034" => Some(("feature-conflict", "Cargo feature conflict between deps")),
        "H035" => Some(("edition-mismatch", "Rust edition mismatch")),
        "H036" => Some(("duplicate-dep", "Duplicate dependency in workspace")),
        "H037" => Some(("network-timeout", "Network error downloading crate")),
        // ── Python runtime errors ───────────────────────
        "H040" => Some(("module-not-found", "Python ModuleNotFoundError")),
        "H041" => Some(("import-error", "Python ImportError")),
        "H042" => Some(("syntax-error", "Python SyntaxError")),
        "H043" => Some(("permission-error", "Python PermissionError")),
        "H044" => Some(("file-not-found", "Python FileNotFoundError")),
        "H045" => Some(("type-error-args", "Wrong number of arguments")),
        "H046" => Some(("keyboard-interrupt", "Process interrupted by user")),
        // ── Exit codes ──────────────────────────────────
        "H050" => Some(("exit-misuse", "Command misuse or invalid arguments")),
        "H051" => Some(("exit-permission", "Permission denied or not executable")),
        "H052" => Some(("exit-not-found", "Command not found")),
        "H053" => Some(("exit-oom-kill", "Process killed (likely OOM)")),
        "H054" => Some(("exit-segfault", "Segmentation fault")),
        "H055" => Some(("exit-signal", "Process killed by signal")),
        // ── Preflight checks ────────────────────────────
        "H060" => Some(("preflight-no-rust", "Rust toolchain not installed")),
        "H061" => Some(("preflight-no-python", "Python interpreter not found")),
        "H062" => Some(("preflight-no-cmake", "cmake not installed")),
        "H063" => Some(("preflight-no-cxx", "C++ compiler not found")),
        "H064" => Some(("preflight-disk-space", "Low disk space warning")),
        _ => None,
    }
}

impl Diagnostic {
    /// Create a new diagnostic with default severity `Error`.
    ///
    /// Auto-populates `docs_url` from the error code catalog.
    pub fn new(
        tool: impl Into<String>,
        code: impl Into<String>,
        message: impl Into<String>,
        hint: impl Into<String>,
    ) -> Self {
        let code = code.into();
        let docs_url =
            error_code_info(&code).map(|(slug, _)| format!("https://horus.dev/errors/{}", slug));
        Self {
            tool: tool.into(),
            code,
            severity: Severity::Error,
            message: message.into(),
            hint: hint.into(),
            fix: None,
            docs_url,
            file: None,
            line: None,
        }
    }

    /// Attach an auto-fix to this diagnostic.
    pub fn with_fix(mut self, fix: Fix) -> Self {
        self.fix = Some(fix);
        self
    }

    /// Override the default severity.
    pub fn with_severity(mut self, severity: Severity) -> Self {
        self.severity = severity;
        self
    }

    /// Serialize to a JSON value for machine-readable output.
    pub fn to_json(&self) -> serde_json::Value {
        serde_json::to_value(self).unwrap_or_default()
    }
}

// ── Cargo error hints ───────────────────────────────────────────────────────

/// Scan cargo stderr for known error patterns and return structured diagnostics.
pub fn cargo_error_hint(stderr: &str) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    // Missing crate
    if let Some(crate_name) = extract_pattern(stderr, r"no matching package named `([^`]+)`") {
        let mut diag = Diagnostic::new(
            "cargo",
            "H001",
            format!("Crate '{}' not found on crates.io", crate_name),
            format!(
                "Crate '{}' not found on crates.io. Check the name or add it with:\n  {}",
                crate_name.cyan(),
                format!("horus add {}", crate_name).green()
            ),
        )
        .with_fix(Fix::Command {
            command: format!("horus add {}", crate_name),
        });
        maybe_suggest_typo(&mut diag, &crate_name, COMMON_CRATES);
        diagnostics.push(diag);
    }

    // Version not found
    if let Some(crate_name) = extract_pattern(
        stderr,
        r"failed to select a version for the requirement `([^`]+)`",
    ) {
        diagnostics.push(
            Diagnostic::new(
                "cargo",
                "H002",
                format!("No matching version for '{}'", crate_name),
                format!(
                    "No matching version for '{}'. Try:\n  {}",
                    crate_name.cyan(),
                    format!("horus add {} (without version to use latest)", crate_name).green()
                ),
            )
            .with_fix(Fix::Command {
                command: format!("horus add {}", crate_name),
            }),
        );
    }

    // Missing feature
    if stderr.contains("feature") && stderr.contains("does not exist") {
        if let Some(feature) = extract_pattern(stderr, r"feature `([^`]+)` does not exist") {
            diagnostics.push(Diagnostic::new(
                "cargo",
                "H003",
                format!("Feature '{}' does not exist in the crate", feature),
                format!(
                    "Feature '{}' does not exist in the crate. Check available features with:\n  {}",
                    feature.cyan(),
                    "cargo doc --open".green()
                ),
            ));
        }
    }

    // Linker error — missing system library
    if stderr.contains("ld: cannot find -l") || stderr.contains("cannot find -l") {
        if let Some(lib) = extract_pattern(stderr, r"cannot find -l(\w+)") {
            let fix_cmd = suggest_dev_install(&lib);
            diagnostics.push(
                Diagnostic::new(
                    "cargo",
                    "H004",
                    format!("Missing system library: lib{}", lib),
                    format!(
                        "Missing system library: lib{}. Install it with:\n  {}",
                        lib.cyan(),
                        fix_cmd.green()
                    ),
                )
                .with_fix(Fix::Command { command: fix_cmd }),
            );
        }
    }

    // pkg-config missing
    if stderr.contains("pkg-config")
        && (stderr.contains("not found") || stderr.contains("Could not run"))
    {
        let fix_cmd = suggest_install("pkg-config");
        diagnostics.push(
            Diagnostic::new(
                "cargo",
                "H005",
                "pkg-config not found",
                format!("pkg-config not found. Install with:\n  {}", fix_cmd.green()),
            )
            .with_fix(Fix::Command { command: fix_cmd }),
        );
    }

    // OpenSSL missing (very common)
    if stderr.contains("openssl") && stderr.contains("Could not find directory") {
        let fix_cmd = format!(
            "{} && {}",
            suggest_dev_install("ssl"),
            suggest_install("pkg-config")
        );
        diagnostics.push(
            Diagnostic::new(
                "cargo",
                "H006",
                "OpenSSL development headers not found",
                format!(
                    "OpenSSL development headers not found. Install with:\n  {}",
                    fix_cmd.green()
                ),
            )
            .with_fix(Fix::Command { command: fix_cmd }),
        );
    }

    // cc / C compiler missing
    if stderr.contains("failed to run custom build command") && stderr.contains("cc") {
        let fix_cmd = suggest_build_essential();
        diagnostics.push(
            Diagnostic::new(
                "cargo",
                "H007",
                "C compiler not found",
                format!(
                    "C compiler not found. Install build essentials:\n  {}",
                    fix_cmd.green()
                ),
            )
            .with_fix(Fix::Command { command: fix_cmd }),
        );
    }

    // E0433: undeclared crate or module
    if let Some(name) = extract_pattern(
        stderr,
        r"error\[E0433\]: failed to resolve: use of undeclared crate or module `([^`]+)`",
    ) {
        diagnostics.push(
            Diagnostic::new(
                "cargo",
                "H030",
                format!("Crate or module '{}' not declared", name),
                format!(
                    "Crate or module '{}' is not declared. Add it to your project:\n  {}",
                    name.cyan(),
                    format!("horus add {}", name).green()
                ),
            )
            .with_fix(Fix::Command {
                command: format!("horus add {}", name),
            }),
        );
    }

    // E0599: no method found
    if let Some(method) =
        extract_pattern(stderr, r"error\[E0599\]: no method named `([^`]+)` found")
    {
        diagnostics.push(Diagnostic::new(
            "cargo",
            "H031",
            format!("No method named '{}'", method),
            format!(
                "Method '{}' not found. Check the type's API with:\n  {}\n  or add a {} import for the trait that provides it",
                method.cyan(),
                "cargo doc --open".green(),
                "use".cyan()
            ),
        ));
    }

    // E0425: unresolved name
    if let Some(name) = extract_pattern(
        stderr,
        r"error\[E0425\]: cannot find value `([^`]+)` in this scope",
    ) {
        diagnostics.push(Diagnostic::new(
            "cargo",
            "H032",
            format!("Cannot find '{}' in scope", name),
            format!(
                "Variable or function '{}' not in scope. Check spelling or add an import",
                name.cyan()
            ),
        ));
    }

    // E0412: undeclared type
    if let Some(name) = extract_pattern(
        stderr,
        r"error\[E0412\]: cannot find type `([^`]+)` in this scope",
    ) {
        diagnostics.push(Diagnostic::new(
            "cargo",
            "H033",
            format!("Type '{}' not found", name),
            format!(
                "Type '{}' not found. Add a {} import or check spelling:\n  {}",
                name.cyan(),
                "use".cyan(),
                format!("use crate::module::{};", name).green()
            ),
        ));
    }

    // Feature conflict
    if stderr.contains("feature") && stderr.contains("required by") && stderr.contains("built with")
    {
        diagnostics.push(Diagnostic::new(
            "cargo",
            "H034",
            "Cargo feature conflict between dependencies",
            format!(
                "Feature conflict between dependencies. Check features in horus.toml:\n  {}",
                "horus deps tree".green()
            ),
        ));
    }

    // Edition mismatch
    if stderr.contains("edition") && (stderr.contains("is required") || stderr.contains("requires"))
    {
        diagnostics.push(Diagnostic::new(
            "cargo",
            "H035",
            "Rust edition mismatch",
            format!(
                "Rust edition mismatch. Set the edition in horus.toml [package]:\n  {}",
                "rust_edition = \"2021\"".green()
            ),
        ));
    }

    // Duplicate dependency
    if stderr.contains("two packages named")
        || (stderr.contains("multiple") && stderr.contains("found in this workspace"))
    {
        diagnostics.push(Diagnostic::new(
            "cargo",
            "H036",
            "Duplicate dependency in workspace",
            "Duplicate dependency detected. Check horus.toml [dependencies] for duplicates",
        ));
    }

    // Network / download timeout
    if stderr.contains("Timeout waiting for")
        || (stderr.contains("network") && stderr.contains("error"))
        || stderr.contains("couldn't download")
    {
        diagnostics.push(
            Diagnostic::new(
                "cargo",
                "H037",
                "Network error downloading crate",
                format!(
                    "Network error downloading crate. Check your connection or try:\n  {}",
                    "cargo fetch".green()
                ),
            )
            .with_fix(Fix::Command {
                command: "cargo fetch".into(),
            }),
        );
    }

    diagnostics
}

// ── Pip error hints ─────────────────────────────────────────────────────────

/// Scan pip stderr for known error patterns and return structured diagnostics.
pub fn pip_error_hint(stderr: &str) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    // Package not found
    if stderr.contains("No matching distribution found for") {
        if let Some(pkg) = extract_pattern(stderr, r"No matching distribution found for ([^\s]+)") {
            diagnostics.push(
                Diagnostic::new(
                    "pip",
                    "H010",
                    format!("Package '{}' not found on PyPI", pkg),
                    format!(
                        "Package '{}' not found on PyPI. Check the name or add it with:\n  {}",
                        pkg.cyan(),
                        format!("horus add {}", pkg).green()
                    ),
                )
                .with_fix(Fix::Command {
                    command: format!("horus add {}", pkg),
                }),
            );
        }
    }

    // Version conflict
    if stderr.contains("ResolutionImpossible") || stderr.contains("incompatible versions") {
        diagnostics.push(
            Diagnostic::new(
                "pip",
                "H011",
                "Python dependency conflict detected",
                "Python dependency conflict detected. Try:\n  \
                 1. Relax version constraints in horus.toml\n  \
                 2. Run `horus deps tree` to see the full dependency graph",
            )
            .with_severity(Severity::Error),
        );
    }

    // Build wheel failure (usually missing C headers)
    if stderr.contains("Failed building wheel for") {
        if let Some(pkg) = extract_pattern(stderr, r"Failed building wheel for ([^\s]+)") {
            diagnostics.push(
                Diagnostic::new(
                    "pip",
                    "H012",
                    format!("Failed to build '{}'", pkg),
                    format!(
                        "Failed to build '{}'. This package likely needs C/C++ build tools:\n  {}",
                        pkg.cyan(),
                        format!(
                            "{} && {}",
                            suggest_build_essential(),
                            suggest_install("python3-dev")
                        )
                        .green()
                    ),
                )
                .with_fix(Fix::Command {
                    command: format!(
                        "{} && {}",
                        suggest_build_essential(),
                        suggest_install("python3-dev")
                    ),
                }),
            );
        }
    }

    // externally-managed-environment (Python 3.11+ / Debian)
    if stderr.contains("externally-managed-environment") {
        diagnostics.push(Diagnostic::new(
            "pip",
            "H013",
            "System Python is externally managed",
            format!(
                "System Python is externally managed. Use:\n  {} or \n  {}",
                "horus add <package>".green(),
                "python3 -m venv .venv && source .venv/bin/activate".green()
            ),
        ));
    }

    // Permission denied
    if stderr.contains("Permission denied") && stderr.contains("pip") {
        diagnostics.push(Diagnostic::new(
            "pip",
            "H014",
            "Permission denied during pip install",
            format!(
                "Permission denied. Don't use sudo with pip. Instead:\n  {}",
                "python3 -m venv .venv && source .venv/bin/activate".green()
            ),
        ));
    }

    diagnostics
}

// ── CMake error hints ───────────────────────────────────────────────────────

/// Scan cmake stderr for known error patterns and return structured diagnostics.
pub fn cmake_error_hint(stderr: &str) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    // Missing cmake package (find_package failed)
    if stderr.contains("Could not find a package configuration file provided by") {
        if let Some(pkg) = extract_pattern(stderr, r#"provided by [""']?(\w+)"#) {
            let fix_cmd = suggest_dev_install(&pkg.to_lowercase());
            diagnostics.push(
                Diagnostic::new(
                    "cmake",
                    "H020",
                    format!("CMake package '{}' not found", pkg),
                    format!(
                        "CMake package '{}' not found. Install it with:\n  {}",
                        pkg.cyan(),
                        fix_cmd.green()
                    ),
                )
                .with_fix(Fix::Command { command: fix_cmd }),
            );
        }
    }

    // Generic find_package failure — "Could not find <Package>"
    if stderr.contains("Could not find") && !stderr.contains("package configuration file") {
        if let Some(pkg) = extract_pattern(stderr, r"Could not find (\w+)") {
            let fix_cmd = suggest_dev_install(&pkg.to_lowercase());
            diagnostics.push(
                Diagnostic::new(
                    "cmake",
                    "H021",
                    format!("CMake could not find '{}'", pkg),
                    format!(
                        "CMake could not find '{}'. Install it with:\n  {}",
                        pkg.cyan(),
                        fix_cmd.green()
                    ),
                )
                .with_fix(Fix::Command { command: fix_cmd }),
            );
        }
    }

    // No C++ compiler
    if stderr.contains("No CMAKE_CXX_COMPILER could be found") {
        let fix_cmd = suggest_install("g++");
        diagnostics.push(
            Diagnostic::new(
                "cmake",
                "H022",
                "No C++ compiler found",
                format!(
                    "No C++ compiler found. Install with:\n  {}",
                    fix_cmd.green()
                ),
            )
            .with_fix(Fix::Command { command: fix_cmd }),
        );
    }

    // No C compiler
    if stderr.contains("No CMAKE_C_COMPILER could be found") {
        let fix_cmd = suggest_install("gcc");
        diagnostics.push(
            Diagnostic::new(
                "cmake",
                "H023",
                "No C compiler found",
                format!("No C compiler found. Install with:\n  {}", fix_cmd.green()),
            )
            .with_fix(Fix::Command { command: fix_cmd }),
        );
    }

    // cmake itself not found
    if stderr.contains("cmake: command not found") || stderr.contains("cmake: not found") {
        let fix_cmd = suggest_install("cmake");
        diagnostics.push(
            Diagnostic::new(
                "cmake",
                "H024",
                "cmake is not installed",
                format!(
                    "cmake is not installed. Install with:\n  {}",
                    fix_cmd.green()
                ),
            )
            .with_fix(Fix::Command { command: fix_cmd }),
        );
    }

    // CMakeLists.txt syntax error or missing include
    if stderr.contains("CMake Error at") {
        if let Some(detail) = extract_pattern(stderr, r"CMake Error at [^:]+:\d+ \(([^)]+)\)") {
            diagnostics.push(Diagnostic::new(
                "cmake",
                "H025",
                format!("CMake configuration error in '{}' command", detail),
                format!(
                    "CMake configuration error in '{}' command. Check your horus.toml dependencies.",
                    detail.cyan()
                ),
            ));
        }
    }

    diagnostics
}

// ── Python runtime error hints ──────────────────────────────────────────────

/// Scan Python stderr/traceback for known runtime error patterns.
pub fn python_error_hint(stderr: &str) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    // ModuleNotFoundError
    if let Some(module) = extract_pattern(stderr, r"ModuleNotFoundError: No module named '([^']+)'")
    {
        let base_module = module.split('.').next().unwrap_or(&module).to_string();
        let mut diag = Diagnostic::new(
            "python",
            "H040",
            format!("Module '{}' not installed", module),
            format!(
                "Module '{}' not installed. Add it with:\n  {}",
                module.cyan(),
                format!("horus add {}", base_module).green()
            ),
        )
        .with_fix(Fix::Command {
            command: format!("horus add {}", base_module),
        });
        maybe_suggest_typo(&mut diag, &base_module, COMMON_PYTHON_PACKAGES);
        diagnostics.push(diag);
    }

    // ImportError
    if let Some(name) = extract_pattern(stderr, r"ImportError: cannot import name '([^']+)'") {
        diagnostics.push(Diagnostic::new(
            "python",
            "H041",
            format!("Cannot import '{}'", name),
            format!(
                "Cannot import '{}'. Check the package version or API changes",
                name.cyan()
            ),
        ));
    }

    // SyntaxError
    if stderr.contains("SyntaxError:") {
        diagnostics.push(
            Diagnostic::new(
                "python",
                "H042",
                "Python syntax error",
                "Python syntax error. Check Python version compatibility (horus uses python3)",
            )
            .with_severity(Severity::Error),
        );
    }

    // PermissionError
    if stderr.contains("PermissionError:") {
        diagnostics.push(Diagnostic::new(
            "python",
            "H043",
            "Permission denied",
            "Permission denied. Check file/device permissions on the target path",
        ));
    }

    // FileNotFoundError
    if stderr.contains("FileNotFoundError:") {
        diagnostics.push(Diagnostic::new(
            "python",
            "H044",
            "File not found",
            "File not found. Check that referenced files and paths exist in the project",
        ));
    }

    // TypeError: wrong number of arguments
    if stderr.contains("TypeError:") && stderr.contains("positional argument") {
        diagnostics.push(Diagnostic::new(
            "python",
            "H045",
            "Wrong number of arguments",
            "Wrong number of arguments passed to function. Check the function signature",
        ));
    }

    // KeyboardInterrupt
    if stderr.contains("KeyboardInterrupt") {
        diagnostics.push(
            Diagnostic::new(
                "python",
                "H046",
                "Process interrupted by user",
                "Process interrupted by user (Ctrl+C)",
            )
            .with_severity(Severity::Info),
        );
    }

    diagnostics
}

// ── Exit code hints ─────────────────────────────────────────────────────────

/// Interpret a process exit code and return diagnostics for non-obvious codes.
///
/// Common Unix exit codes and signal-based exits (128+N) are translated into
/// actionable hints. Generic exit code 1 returns empty (too ambiguous).
pub fn exit_code_hint(tool: &str, code: i32) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();

    match code {
        2 => {
            diagnostics.push(
                Diagnostic::new(
                    tool,
                    "H050",
                    "Misuse of command or invalid arguments",
                    "Exit code 2: command misuse or invalid arguments. Check usage with --help",
                )
                .with_severity(Severity::Warning),
            );
        }
        126 => {
            diagnostics.push(Diagnostic::new(
                tool,
                "H051",
                "Permission denied or not executable",
                "Exit code 126: permission denied or file is not executable.\n  Check file permissions: chmod +x <file>",
            ));
        }
        127 => {
            diagnostics.push(Diagnostic::new(
                tool,
                "H052",
                "Command not found",
                format!(
                    "Exit code 127: command not found. Check that {} is installed and in PATH",
                    tool.cyan()
                ),
            ));
        }
        137 => {
            diagnostics.push(Diagnostic::new(
                tool,
                "H053",
                "Process killed (likely OOM)",
                "Exit code 137: process killed (SIGKILL). Likely out of memory.\n  Check memory usage or reduce workload",
            ));
        }
        139 => {
            diagnostics.push(Diagnostic::new(
                tool,
                "H054",
                "Segmentation fault",
                "Exit code 139: segmentation fault (SIGSEGV). Check for null pointer access or buffer overflows",
            ));
        }
        _ if code > 128 => {
            let signal = code - 128;
            diagnostics.push(
                Diagnostic::new(
                    tool,
                    "H055",
                    format!("Killed by signal {}", signal),
                    format!("Process killed by signal {} (exit code {})", signal, code),
                )
                .with_severity(Severity::Warning),
            );
        }
        _ => {} // Exit code 1 and others: too generic, no hint
    }

    diagnostics
}

// ── Fuzzy matching ──────────────────────────────────────────────────────────

/// Common Rust crates for "did you mean?" suggestions.
const COMMON_CRATES: &[&str] = &[
    "tokio",
    "serde",
    "serde_json",
    "reqwest",
    "clap",
    "anyhow",
    "thiserror",
    "tracing",
    "tracing-subscriber",
    "hyper",
    "axum",
    "actix-web",
    "diesel",
    "sqlx",
    "rand",
    "chrono",
    "regex",
    "log",
    "env_logger",
    "futures",
    "async-trait",
    "bytes",
    "nalgebra",
    "rapier3d",
    "bevy",
    "wgpu",
    "winit",
    "image",
    "horus",
    "horus_core",
    "horus_library",
    "rclrs",
    "prost",
    "tonic",
    "toml",
    "yaml-rust",
    "csv",
    "rayon",
    "crossbeam",
    "dashmap",
    "parking_lot",
];

/// Common Python packages for "did you mean?" suggestions.
const COMMON_PYTHON_PACKAGES: &[&str] = &[
    "numpy",
    "pandas",
    "scipy",
    "matplotlib",
    "opencv-python",
    "torch",
    "tensorflow",
    "scikit-learn",
    "pillow",
    "requests",
    "flask",
    "fastapi",
    "horus",
    "horus-py",
    "rclpy",
    "pyyaml",
    "toml",
    "pytest",
    "black",
    "ruff",
    "mypy",
    "uvicorn",
    "pydantic",
    "httpx",
    "aiohttp",
    "boto3",
    "django",
    "celery",
    "redis",
    "psycopg2",
    "sqlalchemy",
];

/// Compute Levenshtein edit distance between two strings.
#[allow(clippy::needless_range_loop)]
fn edit_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let (m, n) = (a.len(), b.len());
    let mut dp = vec![vec![0usize; n + 1]; m + 1];
    for i in 0..=m {
        dp[i][0] = i;
    }
    for j in 0..=n {
        dp[0][j] = j;
    }
    for i in 1..=m {
        for j in 1..=n {
            let cost = if a[i - 1] == b[j - 1] { 0 } else { 1 };
            dp[i][j] = (dp[i - 1][j] + 1)
                .min(dp[i][j - 1] + 1)
                .min(dp[i - 1][j - 1] + cost);
        }
    }
    dp[m][n]
}

/// Find the closest match for `name` in `candidates` with distance <= max_distance.
///
/// Returns `None` if no close match is found or if the input exactly matches (distance 0).
fn fuzzy_match<'a>(name: &str, candidates: &'a [&str], max_distance: usize) -> Option<&'a str> {
    let name_lower = name.to_lowercase();
    candidates
        .iter()
        .map(|c| (*c, edit_distance(&name_lower, &c.to_lowercase())))
        .filter(|(_, d)| *d > 0 && *d <= max_distance)
        .min_by_key(|(_, d)| *d)
        .map(|(c, _)| c)
}

/// Append a "Did you mean: X?" suggestion to a diagnostic if a fuzzy match exists.
fn maybe_suggest_typo(diag: &mut Diagnostic, name: &str, candidates: &[&str]) {
    if let Some(suggestion) = fuzzy_match(name, candidates, 2) {
        diag.hint.push_str(&format!(
            "\n  {} Did you mean: {}?",
            "\u{00b7}".dimmed(),
            suggestion.green()
        ));
        // Update fix to use the corrected name
        diag.fix = Some(Fix::Command {
            command: format!("horus add {}", suggestion),
        });
    }
}

// ── Preflight validation ────────────────────────────────────────────────────

/// Run pre-build validation checks and return ALL issues at once.
///
/// Unlike build errors (which stop at the first failure), preflight reports
/// every issue so the user can fix them all before rebuilding.
pub fn preflight_check(language: &str) -> Vec<Diagnostic> {
    let mut issues = Vec::new();

    // ── Toolchain checks ────────────────────────────────────────────────
    match language {
        "rust" => {
            if std::process::Command::new("cargo")
                .arg("--version")
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .status()
                .is_err()
            {
                issues.push(
                    Diagnostic::new(
                        "preflight",
                        "H060",
                        "Rust toolchain not found",
                        "Rust toolchain not installed. Install with:\n  \
                         curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh",
                    )
                    .with_fix(Fix::Command {
                        command: "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
                            .into(),
                    }),
                );
            }
        }
        "python" => {
            let has_python = ["python3", "python"].iter().any(|cmd| {
                std::process::Command::new(cmd)
                    .arg("--version")
                    .stdout(std::process::Stdio::null())
                    .stderr(std::process::Stdio::null())
                    .status()
                    .is_ok()
            });
            if !has_python {
                let fix_cmd = suggest_install("python3");
                issues.push(
                    Diagnostic::new(
                        "preflight",
                        "H061",
                        "Python interpreter not found",
                        format!("Python 3 not found. Install with:\n  {}", fix_cmd.as_str()),
                    )
                    .with_fix(Fix::Command { command: fix_cmd }),
                );
            }
        }
        "cpp" => {
            // Check cmake
            if std::process::Command::new("cmake")
                .arg("--version")
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .status()
                .is_err()
            {
                let fix_cmd = suggest_install("cmake");
                issues.push(
                    Diagnostic::new(
                        "preflight",
                        "H062",
                        "cmake not found",
                        format!("cmake not installed. Install with:\n  {}", fix_cmd.as_str()),
                    )
                    .with_fix(Fix::Command { command: fix_cmd }),
                );
            }
            // Check C++ compiler
            if std::process::Command::new("g++")
                .arg("--version")
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .status()
                .is_err()
                && std::process::Command::new("c++")
                    .arg("--version")
                    .stdout(std::process::Stdio::null())
                    .stderr(std::process::Stdio::null())
                    .status()
                    .is_err()
            {
                let fix_cmd = suggest_install("g++");
                issues.push(
                    Diagnostic::new(
                        "preflight",
                        "H063",
                        "C++ compiler not found",
                        format!(
                            "No C++ compiler found. Install with:\n  {}",
                            fix_cmd.as_str()
                        ),
                    )
                    .with_fix(Fix::Command { command: fix_cmd }),
                );
            }
        }
        _ => {}
    }

    // ── Disk space check ────────────────────────────────────────────────
    if let Ok(output) = std::process::Command::new("df")
        .args(["--output=avail", "-B1", "."])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::null())
        .output()
    {
        let stdout = String::from_utf8_lossy(&output.stdout);
        // Second line has the available bytes
        if let Some(avail_str) = stdout.lines().nth(1) {
            if let Ok(avail_bytes) = avail_str.trim().parse::<u64>() {
                let avail_mb = avail_bytes / (1024 * 1024);
                if avail_mb < 500 {
                    issues.push(
                        Diagnostic::new(
                            "preflight",
                            "H064",
                            format!("Low disk space ({}MB free)", avail_mb),
                            format!(
                                "Only {}MB free. Builds may fail. Free up space or clean with:\n  {}",
                                avail_mb,
                                "horus clean".green()
                            ),
                        )
                        .with_severity(Severity::Warning)
                        .with_fix(Fix::Command {
                            command: "horus clean".into(),
                        }),
                    );
                }
            }
        }
    }

    issues
}

// ── Path rewriting ──────────────────────────────────────────────────────────

/// Rewrite internal `.horus/` paths in error messages to user-facing equivalents.
///
/// Users should never see raw `.horus/` paths — they point to generated files
/// that the user should not edit. This rewrites them to the source of truth.
pub fn rewrite_horus_paths(text: &str) -> String {
    text.replace(
        ".horus/CMakeLists.txt",
        "horus.toml (generated CMakeLists.txt)",
    )
    .replace(".horus/Cargo.toml", "horus.toml (generated Cargo.toml)")
    .replace(
        ".horus/pyproject.toml",
        "horus.toml (generated pyproject.toml)",
    )
    .replace(".horus/cpp-build/", "build/")
    .replace(".horus/target/", "build/")
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Extract a capture group from stderr using a regex pattern.
fn extract_pattern(text: &str, pattern: &str) -> Option<String> {
    let re = regex::Regex::new(pattern).ok()?;
    re.captures(text)
        .and_then(|caps| caps.get(1))
        .map(|m| m.as_str().to_string())
}

/// Format a diagnostic for colored terminal output.
///
/// Produces: `\nhorus hint [tool] H00N\nhint_text`
pub fn format_diagnostic(diag: &Diagnostic) -> String {
    format!(
        "\n{} {} {} {}\n{}",
        "horus".bold().cyan(),
        "hint".bold().yellow(),
        format!("[{}]", diag.tool).dimmed(),
        diag.code.dimmed(),
        diag.hint
    )
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Diagnostic type ─────────────────────────────────────────────────

    #[test]
    fn diagnostic_new_defaults() {
        let d = Diagnostic::new("cargo", "H001", "test message", "test hint");
        assert_eq!(d.tool, "cargo");
        assert_eq!(d.code, "H001");
        assert_eq!(d.severity, Severity::Error);
        assert_eq!(d.message, "test message");
        assert_eq!(d.hint, "test hint");
        assert!(d.fix.is_none());
        assert!(d.file.is_none());
        assert!(d.line.is_none());
    }

    #[test]
    fn diagnostic_with_fix() {
        let d = Diagnostic::new("cargo", "H001", "msg", "hint").with_fix(Fix::Command {
            command: "horus add tokio".into(),
        });
        assert!(d.fix.is_some());
        match d.fix.as_ref().unwrap() {
            Fix::Command { command } => assert_eq!(command, "horus add tokio"),
            _ => panic!("expected Fix::Command"),
        }
    }

    #[test]
    fn diagnostic_with_severity() {
        let d = Diagnostic::new("pip", "H011", "msg", "hint").with_severity(Severity::Warning);
        assert_eq!(d.severity, Severity::Warning);
    }

    #[test]
    fn diagnostic_to_json_has_all_fields() {
        let d = Diagnostic::new("cargo", "H001", "Crate not found", "Try horus add").with_fix(
            Fix::Command {
                command: "horus add tokio".into(),
            },
        );
        let json = d.to_json();
        assert_eq!(json["tool"], "cargo");
        assert_eq!(json["code"], "H001");
        assert_eq!(json["severity"], "error");
        assert_eq!(json["message"], "Crate not found");
        assert_eq!(json["hint"], "Try horus add");
        assert_eq!(json["fix"]["type"], "command");
        assert_eq!(json["fix"]["command"], "horus add tokio");
        // Optional fields should not be present when None
        assert!(json.get("file").is_none());
        assert!(json.get("line").is_none());
    }

    #[test]
    fn diagnostic_fix_multi_serializes() {
        let d = Diagnostic::new("preflight", "H060", "msg", "hint").with_fix(Fix::Multi {
            fixes: vec![
                Fix::Command {
                    command: "install cmake".into(),
                },
                Fix::Command {
                    command: "install g++".into(),
                },
            ],
        });
        let json = d.to_json();
        assert_eq!(json["fix"]["type"], "multi");
        assert_eq!(json["fix"]["fixes"].as_array().unwrap().len(), 2);
    }

    // ── Error code catalog ────────────────────────────────────────────

    #[test]
    fn error_code_catalog_covers_all_defined_codes() {
        let codes = [
            "H001", "H002", "H003", "H004", "H005", "H006", "H007", "H010", "H011", "H012", "H013",
            "H014", "H020", "H021", "H022", "H023", "H024", "H025", "H030", "H031", "H032", "H033",
            "H034", "H035", "H036", "H037", "H040", "H041", "H042", "H043", "H044", "H045", "H046",
            "H050", "H051", "H052", "H053", "H054", "H055", "H060", "H061", "H062", "H063", "H064",
        ];
        for code in &codes {
            assert!(
                error_code_info(code).is_some(),
                "error_code_info missing entry for {}",
                code
            );
        }
    }

    #[test]
    fn error_code_no_duplicates() {
        let codes = [
            "H001", "H002", "H003", "H004", "H005", "H006", "H007", "H010", "H011", "H012", "H013",
            "H014", "H020", "H021", "H022", "H023", "H024", "H025", "H030", "H031", "H032", "H033",
            "H034", "H035", "H036", "H037", "H040", "H041", "H042", "H043", "H044", "H045", "H046",
            "H050", "H051", "H052", "H053", "H054", "H055", "H060", "H061", "H062", "H063", "H064",
        ];
        let mut slugs: Vec<&str> = codes
            .iter()
            .filter_map(|c| error_code_info(c).map(|(slug, _)| slug))
            .collect();
        let original_len = slugs.len();
        slugs.sort();
        slugs.dedup();
        assert_eq!(
            slugs.len(),
            original_len,
            "duplicate slugs found in error code catalog"
        );
    }

    #[test]
    fn diagnostic_has_docs_url_for_known_code() {
        let d = Diagnostic::new("cargo", "H001", "msg", "hint");
        assert!(d.docs_url.is_some());
        assert!(d.docs_url.as_ref().unwrap().contains("missing-crate"));
    }

    #[test]
    fn diagnostic_unknown_code_no_docs_url() {
        let d = Diagnostic::new("cargo", "H999", "msg", "hint");
        assert!(d.docs_url.is_none());
    }

    // ── Distro detection ──────────────────────────────────────────────

    #[test]
    fn detect_distro_returns_a_variant() {
        let d = detect_distro();
        // Should return some variant, not panic
        assert!(matches!(
            d,
            Distro::Ubuntu
                | Distro::Debian
                | Distro::Fedora
                | Distro::Arch
                | Distro::NixOS
                | Distro::Alpine
                | Distro::MacOS
                | Distro::Windows
                | Distro::Unknown
        ));
    }

    #[test]
    fn suggest_install_produces_nonempty() {
        let cmd = suggest_install("cmake");
        assert!(!cmd.is_empty());
        assert!(cmd.contains("cmake"));
    }

    #[test]
    fn suggest_dev_install_produces_nonempty() {
        let cmd = suggest_dev_install("ssl");
        assert!(!cmd.is_empty());
        assert!(cmd.contains("ssl"));
    }

    #[test]
    fn suggest_build_essential_produces_nonempty() {
        let cmd = suggest_build_essential();
        assert!(!cmd.is_empty());
    }

    // ── JSON diagnostics mode ─────────────────────────────────────────

    #[test]
    fn json_diagnostics_flag_toggles() {
        set_json_diagnostics(true);
        assert!(is_json_diagnostics());
        set_json_diagnostics(false);
        assert!(!is_json_diagnostics());
    }

    #[test]
    fn emit_diagnostic_does_not_panic_normal_mode() {
        set_json_diagnostics(false);
        let d = Diagnostic::new("cargo", "H001", "test", "test hint");
        emit_diagnostic(&d); // just verify no panic
    }

    #[test]
    fn emit_diagnostic_does_not_panic_json_mode() {
        set_json_diagnostics(true);
        let d = Diagnostic::new("cargo", "H001", "test", "test hint").with_fix(Fix::Command {
            command: "horus add foo".into(),
        });
        emit_diagnostic(&d); // just verify no panic
        set_json_diagnostics(false); // reset
    }

    // ── Cargo error hints ───────────────────────────────────────────────

    #[test]
    fn cargo_missing_crate() {
        let stderr = "error: no matching package named `nonexistent-crate` found";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H001");
        assert!(diags[0].message.contains("nonexistent-crate"));
        assert!(diags[0].fix.is_some());
    }

    #[test]
    fn cargo_version_mismatch() {
        let stderr = "error: failed to select a version for the requirement `tokio = \"^99.0\"`";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H002");
    }

    #[test]
    fn cargo_missing_feature() {
        let stderr = "error: feature `nonexistent` does not exist for package `serde`";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H003");
        assert!(diags[0].message.contains("nonexistent"));
    }

    #[test]
    fn cargo_missing_linker_lib() {
        let stderr = "/usr/bin/ld: cannot find -lssl: No such file";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H004");
        assert!(diags[0].hint.contains("libssl-dev"));
    }

    #[test]
    fn cargo_missing_pkg_config() {
        let stderr = "Could not run `pkg-config`";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H005");
        assert!(diags[0].hint.contains("apt install pkg-config"));
    }

    #[test]
    fn cargo_missing_openssl() {
        let stderr = "Could not find directory of openssl installation";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H006");
        assert!(diags[0].hint.contains("libssl-dev"));
    }

    #[test]
    fn cargo_unrecognized_error_returns_empty() {
        let stderr = "error[E0308]: mismatched types";
        assert!(cargo_error_hint(stderr).is_empty());
    }

    #[test]
    fn cargo_undeclared_crate_e0433() {
        let stderr = "error[E0433]: failed to resolve: use of undeclared crate or module `tokio`";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(
            diags.iter().find(|d| d.code == "H030").unwrap().code,
            "H030"
        );
        assert!(diags.iter().any(|d| d.message.contains("tokio")));
    }

    #[test]
    fn cargo_no_method_e0599() {
        let stderr = "error[E0599]: no method named `foo_bar` found for struct `Vec<i32>`";
        let diags = cargo_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H031"));
        assert!(diags.iter().any(|d| d.message.contains("foo_bar")));
    }

    #[test]
    fn cargo_unresolved_name_e0425() {
        let stderr = "error[E0425]: cannot find value `my_var` in this scope";
        let diags = cargo_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H032"));
        assert!(diags.iter().any(|d| d.message.contains("my_var")));
    }

    #[test]
    fn cargo_undeclared_type_e0412() {
        let stderr = "error[E0412]: cannot find type `MyStruct` in this scope";
        let diags = cargo_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H033"));
        assert!(diags.iter().any(|d| d.message.contains("MyStruct")));
    }

    #[test]
    fn cargo_feature_conflict() {
        let stderr = "error: feature `std` is required by `serde` but `serde` was built with feature `no_std`";
        let diags = cargo_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H034"));
    }

    #[test]
    fn cargo_edition_mismatch() {
        let stderr = "error: edition 2021 is required but the crate was compiled with edition 2018";
        let diags = cargo_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H035"));
    }

    #[test]
    fn cargo_duplicate_dep() {
        let stderr = "error: two packages named `serde` in this workspace";
        let diags = cargo_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H036"));
    }

    #[test]
    fn cargo_network_timeout() {
        let stderr = "error: couldn't download crate `tokio`: network error";
        let diags = cargo_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H037"));
        assert!(diags.iter().any(|d| d.fix.is_some()));
    }

    // ── Pip error hints ─────────────────────────────────────────────────

    #[test]
    fn pip_package_not_found() {
        let stderr = "ERROR: No matching distribution found for nonexistent-pkg";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H010");
        assert!(diags[0].message.contains("nonexistent-pkg"));
    }

    #[test]
    fn pip_version_conflict() {
        let stderr = "ERROR: ResolutionImpossible: requirements conflict";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H011");
        assert!(diags[0].hint.contains("dependency conflict"));
    }

    #[test]
    fn pip_build_wheel_failure() {
        let stderr = "ERROR: Failed building wheel for numpy";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H012");
        assert!(diags[0].message.contains("numpy"));
    }

    #[test]
    fn pip_externally_managed() {
        let stderr = "error: externally-managed-environment";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H013");
        assert!(diags[0].hint.contains("venv"));
    }

    #[test]
    fn pip_permission_denied() {
        let stderr = "ERROR: Could not install packages due to an OSError: Permission denied (pip)";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H014");
        assert!(diags[0].hint.contains("Don't use sudo"));
    }

    #[test]
    fn pip_unrecognized_error_returns_empty() {
        let stderr = "SyntaxError: invalid syntax";
        assert!(pip_error_hint(stderr).is_empty());
    }

    // ── Python error hints ────────────────────────────────────────────────

    #[test]
    fn python_module_not_found() {
        let stderr = "Traceback:\n  File \"main.py\"\nModuleNotFoundError: No module named 'cv2'";
        let diags = python_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H040"));
        assert!(diags.iter().any(|d| d.message.contains("cv2")));
        assert!(diags.iter().any(|d| d.fix.is_some()));
    }

    #[test]
    fn python_module_not_found_submodule() {
        let stderr = "ModuleNotFoundError: No module named 'numpy.core'";
        let diags = python_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H040"));
        // Fix command should use base module name
        match &diags[0].fix {
            Some(Fix::Command { command }) => assert!(command.contains("numpy")),
            _ => panic!("expected Fix::Command with numpy"),
        }
    }

    #[test]
    fn python_import_error() {
        let stderr = "ImportError: cannot import name 'foo' from 'bar'";
        let diags = python_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H041"));
        assert!(diags.iter().any(|d| d.message.contains("foo")));
    }

    #[test]
    fn python_syntax_error() {
        let stderr = "SyntaxError: invalid syntax";
        let diags = python_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H042"));
    }

    #[test]
    fn python_permission_error() {
        let stderr = "PermissionError: [Errno 13] Permission denied: '/dev/ttyUSB0'";
        let diags = python_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H043"));
    }

    #[test]
    fn python_file_not_found() {
        let stderr = "FileNotFoundError: [Errno 2] No such file or directory: 'config.yaml'";
        let diags = python_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H044"));
    }

    #[test]
    fn python_type_error_arguments() {
        let stderr = "TypeError: foo() takes 2 positional arguments but 3 were given";
        let diags = python_error_hint(stderr);
        assert!(diags.iter().any(|d| d.code == "H045"));
    }

    #[test]
    fn python_keyboard_interrupt() {
        let stderr = "KeyboardInterrupt";
        let diags = python_error_hint(stderr);
        assert!(diags
            .iter()
            .any(|d| d.code == "H046" && d.severity == Severity::Info));
    }

    #[test]
    fn python_unrecognized_returns_empty() {
        let stderr = "RuntimeError: something went wrong";
        assert!(python_error_hint(stderr).is_empty());
    }

    // ── Preflight checks ─────────────────────────────────────────────────

    #[test]
    fn preflight_rust_returns_empty_when_cargo_installed() {
        // cargo is installed in CI/dev — should return no issues
        let issues = preflight_check("rust");
        assert!(
            !issues.iter().any(|d| d.code == "H060"),
            "cargo should be found on this system"
        );
    }

    #[test]
    fn preflight_unknown_language_returns_empty() {
        let issues = preflight_check("brainfuck");
        // Only disk space check could fire, no toolchain checks
        assert!(!issues.iter().any(|d| d.code == "H060"));
        assert!(!issues.iter().any(|d| d.code == "H061"));
    }

    #[test]
    fn preflight_python_returns_empty_when_python_installed() {
        let issues = preflight_check("python");
        assert!(
            !issues.iter().any(|d| d.code == "H061"),
            "python3 should be found on this system"
        );
    }

    // ── Fuzzy matching ───────────────────────────────────────────────────

    #[test]
    fn edit_distance_identical() {
        assert_eq!(edit_distance("tokio", "tokio"), 0);
    }

    #[test]
    fn edit_distance_one_char() {
        assert_eq!(edit_distance("toko", "tokio"), 1);
    }

    #[test]
    fn edit_distance_two_chars() {
        assert_eq!(edit_distance("tki", "tokio"), 2);
    }

    #[test]
    fn fuzzy_match_finds_tokio() {
        assert_eq!(fuzzy_match("toko", COMMON_CRATES, 2), Some("tokio"));
    }

    #[test]
    fn fuzzy_match_finds_numpy() {
        assert_eq!(
            fuzzy_match("nump", COMMON_PYTHON_PACKAGES, 2),
            Some("numpy")
        );
    }

    #[test]
    fn fuzzy_match_no_match_for_gibberish() {
        assert_eq!(fuzzy_match("xyzabc123", COMMON_CRATES, 2), None);
    }

    #[test]
    fn fuzzy_match_exact_returns_none() {
        // Distance 0 should be filtered out (exact match = not a typo)
        // Use a name that has no other close neighbors
        assert_eq!(fuzzy_match("crossbeam", COMMON_CRATES, 2), None);
    }

    #[test]
    fn cargo_h001_includes_did_you_mean_for_typo() {
        let stderr = "error: no matching package named `toko` found";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty());
        assert!(diags[0].hint.contains("Did you mean"));
        assert!(diags[0].hint.contains("tokio"));
        // Fix command should use corrected name
        match &diags[0].fix {
            Some(Fix::Command { command }) => assert!(command.contains("tokio")),
            _ => panic!("expected Fix::Command with tokio"),
        }
    }

    #[test]
    fn python_h040_includes_did_you_mean_for_typo() {
        let stderr = "ModuleNotFoundError: No module named 'nump'";
        let diags = python_error_hint(stderr);
        assert!(!diags.is_empty());
        assert!(diags[0].hint.contains("Did you mean"));
        assert!(diags[0].hint.contains("numpy"));
    }

    // ── Exit code hints ─────────────────────────────────────────────────

    #[test]
    fn exit_code_137_oom() {
        let diags = exit_code_hint("python", 137);
        assert!(diags.iter().any(|d| d.code == "H053"));
        assert!(diags.iter().any(|d| d.message.contains("OOM")));
    }

    #[test]
    fn exit_code_139_segfault() {
        let diags = exit_code_hint("rust", 139);
        assert!(diags.iter().any(|d| d.code == "H054"));
        assert!(diags.iter().any(|d| d.message.contains("egmentation")));
    }

    #[test]
    fn exit_code_127_not_found() {
        let diags = exit_code_hint("python", 127);
        assert!(diags.iter().any(|d| d.code == "H052"));
    }

    #[test]
    fn exit_code_signal_generic() {
        let diags = exit_code_hint("rust", 134); // SIGABRT = 128+6
        assert!(diags.iter().any(|d| d.code == "H055"));
        assert!(diags.iter().any(|d| d.message.contains("signal 6")));
    }

    #[test]
    fn exit_code_1_returns_empty() {
        assert!(exit_code_hint("python", 1).is_empty());
    }

    #[test]
    fn exit_code_0_returns_empty() {
        assert!(exit_code_hint("python", 0).is_empty());
    }

    // ── CMake error hints ───────────────────────────────────────────────

    #[test]
    fn cmake_missing_package_config() {
        let stderr = r#"CMake Error at CMakeLists.txt:10 (find_package):
  Could not find a package configuration file provided by "Eigen3""#;
        let diags = cmake_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H020");
        assert!(diags[0].hint.contains("eigen3-dev"));
    }

    #[test]
    fn cmake_generic_find_failure() {
        let stderr = "CMake Error: Could not find OpenCV (missing: OpenCV_DIR)";
        let diags = cmake_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H021");
        assert!(diags[0].hint.contains("opencv-dev"));
    }

    #[test]
    fn cmake_no_cxx_compiler() {
        let stderr = "CMake Error: No CMAKE_CXX_COMPILER could be found.";
        let diags = cmake_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H022");
        assert!(diags[0].hint.contains("apt install g++"));
    }

    #[test]
    fn cmake_no_c_compiler() {
        let stderr = "CMake Error: No CMAKE_C_COMPILER could be found.";
        let diags = cmake_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H023");
        assert!(diags[0].hint.contains("apt install gcc"));
    }

    #[test]
    fn cmake_not_installed() {
        let stderr = "cmake: command not found";
        let diags = cmake_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H024");
        assert!(diags[0].hint.contains("apt install cmake"));
    }

    #[test]
    fn cmake_config_error() {
        let stderr =
            "CMake Error at CMakeLists.txt:5 (include):\n  include could not find requested file";
        let diags = cmake_error_hint(stderr);
        assert!(!diags.is_empty());
        assert_eq!(diags[0].code, "H025");
        assert!(diags[0].hint.contains("include"));
    }

    #[test]
    fn cmake_unrecognized_error_returns_empty() {
        let stderr = "error: some unknown cmake issue that we don't handle";
        assert!(cmake_error_hint(stderr).is_empty());
    }

    // ── Path rewriting ──────────────────────────────────────────────────

    #[test]
    fn rewrite_horus_paths_cargo() {
        let text = "error in .horus/Cargo.toml: invalid key";
        let rewritten = rewrite_horus_paths(text);
        assert!(rewritten.contains("horus.toml (generated Cargo.toml)"));
        assert!(!rewritten.contains(".horus/Cargo.toml"));
    }

    #[test]
    fn rewrite_horus_paths_pyproject() {
        let text = "error in .horus/pyproject.toml";
        let rewritten = rewrite_horus_paths(text);
        assert!(rewritten.contains("horus.toml (generated pyproject.toml)"));
    }

    #[test]
    fn rewrite_horus_paths_cmake() {
        let text = "CMake Error at .horus/CMakeLists.txt:5";
        let rewritten = rewrite_horus_paths(text);
        assert!(rewritten.contains("horus.toml (generated CMakeLists.txt)"));
        assert!(!rewritten.contains(".horus/CMakeLists.txt"));
    }

    #[test]
    fn rewrite_horus_paths_cpp_build() {
        let text = "error in .horus/cpp-build/CMakeFiles/foo.o";
        let rewritten = rewrite_horus_paths(text);
        assert!(rewritten.contains("build/CMakeFiles/foo.o"));
        assert!(!rewritten.contains(".horus/cpp-build/"));
    }

    #[test]
    fn rewrite_horus_paths_target() {
        let text = "error in .horus/target/debug/build/foo";
        let rewritten = rewrite_horus_paths(text);
        assert!(rewritten.contains("build/debug/build/foo"));
    }

    #[test]
    fn rewrite_horus_paths_no_match() {
        let text = "error: mismatched types in src/main.rs";
        let rewritten = rewrite_horus_paths(text);
        assert_eq!(rewritten, text);
    }

    // ── Diagnostic formatting ───────────────────────────────────────────

    #[test]
    fn format_diagnostic_includes_tool_and_code() {
        let d = Diagnostic::new("cargo", "H001", "msg", "Install libssl-dev");
        let output = format_diagnostic(&d);
        assert!(output.contains("cargo"));
        assert!(output.contains("hint"));
        assert!(output.contains("Install libssl-dev"));
    }
}
