//! Smart dispatch infrastructure for multi-language horus projects.
//!
//! `ProjectContext` detects what languages and tools a project uses.
//! `ToolChain` maps abstract operations (fmt, lint, test, etc.) to
//! concrete CLI commands per language.
//!
//! Zero new Cargo dependencies — uses `std::process::Command` only.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitStatus, Stdio};

use crate::manifest::{HorusManifest, Language, HORUS_TOML};

// ─── Operation ───────────────────────────────────────────────────────────────

/// An abstract operation that can be dispatched to language-specific tools.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Operation {
    Fmt,
    Lint,
    Test,
    Build,
    Doc,
    Bench,
    Check,
}

impl std::fmt::Display for Operation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fmt => write!(f, "fmt"),
            Self::Lint => write!(f, "lint"),
            Self::Test => write!(f, "test"),
            Self::Build => write!(f, "build"),
            Self::Doc => write!(f, "doc"),
            Self::Bench => write!(f, "bench"),
            Self::Check => write!(f, "check"),
        }
    }
}

// ─── ProjectContext ──────────────────────────────────────────────────────────

/// Detected project context: languages, build files, project root.
#[derive(Debug, Clone)]
pub struct ProjectContext {
    /// Project root directory (where horus.toml lives).
    pub root: PathBuf,

    /// Detected languages in this project.
    pub languages: Vec<Language>,

    /// Whether horus.toml exists.
    pub has_horus_toml: bool,

    /// Loaded manifest (if horus.toml exists).
    pub manifest: Option<HorusManifest>,
}

impl ProjectContext {
    /// Does the project contain Rust code?
    pub fn has_rust(&self) -> bool {
        self.languages.contains(&Language::Rust)
    }

    /// Does the project contain Python code?
    pub fn has_python(&self) -> bool {
        self.languages.contains(&Language::Python)
    }

    /// Is this a mixed-language project?
    pub fn is_mixed(&self) -> bool {
        self.languages.len() > 1
    }
}

/// Detect project context from a directory.
///
/// Searches upward from `start_dir` for `horus.toml`, then detects
/// languages from the project root. If no horus.toml found, uses
/// `start_dir` as root and detects from there.
pub fn detect_context(start_dir: &Path) -> ProjectContext {
    // Try to find horus.toml by searching upward
    let (root, manifest) = match HorusManifest::find_and_load_from(start_dir.to_path_buf()) {
        Ok((m, dir, _origin)) => (dir, Some(m)),
        Err(_) => (start_dir.to_path_buf(), None),
    };

    let has_horus_toml = root.join(HORUS_TOML).exists();

    // Detect languages from project root
    let languages = detect_project_languages(&root);

    ProjectContext {
        root,
        languages,
        has_horus_toml,
        manifest,
    }
}

/// Detect languages present in a project directory.
///
/// Checks for:
/// - Rust: any `.rs` files in root or `src/`, or `Cargo.toml`, or horus.toml deps with source = "crates.io"
/// - Python: any `.py` files in root or `src/`, or `pyproject.toml`, or horus.toml deps with source = "pypi"
/// - C++: `CMakeLists.txt`
/// - ROS2: `package.xml`
fn detect_project_languages(project_dir: &Path) -> Vec<Language> {
    let mut languages = Vec::new();

    // Rust detection: .rs files, Cargo.toml (legacy), or .horus/Cargo.toml (generated)
    let has_rust = project_dir.join("Cargo.toml").exists()
        || project_dir.join(".horus/Cargo.toml").exists()
        || has_files_with_extension(project_dir, "rs");
    if has_rust {
        languages.push(Language::Rust);
    }

    // Python detection: .py files, pyproject.toml (legacy), setup.py, requirements.txt
    let has_python = project_dir.join("pyproject.toml").exists()
        || project_dir.join("setup.py").exists()
        || project_dir.join("requirements.txt").exists()
        || has_files_with_extension(project_dir, "py");
    if has_python {
        languages.push(Language::Python);
    }

    // C++ detection
    if project_dir.join("CMakeLists.txt").exists() {
        languages.push(Language::Cpp);
    }

    // ROS2 detection
    if project_dir.join("package.xml").exists() {
        languages.push(Language::Ros2);
    }

    languages
}

/// Check if a directory contains files with the given extension (non-recursive, root + src/).
fn has_files_with_extension(dir: &Path, ext: &str) -> bool {
    let check_dir = |d: &Path| -> bool {
        if let Ok(entries) = std::fs::read_dir(d) {
            for entry in entries.flatten() {
                if let Some(e) = entry.path().extension() {
                    if e == ext {
                        return true;
                    }
                }
            }
        }
        false
    };

    check_dir(dir) || check_dir(&dir.join("src"))
}

// ─── ToolChain ───────────────────────────────────────────────────────────────

/// A resolved tool for a specific language and operation.
#[derive(Debug, Clone)]
pub struct ResolvedTool {
    /// The binary name (e.g., "cargo", "ruff", "pytest").
    pub bin: String,

    /// Default arguments for this operation (e.g., ["fmt"] for cargo fmt).
    pub default_args: Vec<String>,

    /// Language this tool targets.
    pub language: Language,

    /// Human-readable label for output (e.g., "[rust]", "[python]").
    pub label: String,
}

/// Detected toolchain: maps (language, operation) to concrete tools.
#[derive(Debug, Clone)]
pub struct ToolChain {
    /// Resolved tools keyed by (language, operation).
    tools: BTreeMap<(String, String), ResolvedTool>,
}

impl ToolChain {
    /// Get the resolved tool for a language and operation, if available.
    pub fn get(&self, language: Language, operation: Operation) -> Option<&ResolvedTool> {
        let key = (language.to_string(), operation.to_string());
        self.tools.get(&key)
    }

    /// Get all tools for an operation, ordered by language.
    pub fn tools_for(&self, operation: Operation) -> Vec<&ResolvedTool> {
        let op_str = operation.to_string();
        self.tools
            .iter()
            .filter(|((_, op), _)| op == &op_str)
            .map(|(_, tool)| tool)
            .collect()
    }
}

/// Detect available toolchain by probing for installed tools.
///
/// For each language in the context, checks which tools are available
/// and maps them to operations.
pub fn detect_toolchain(ctx: &ProjectContext) -> ToolChain {
    let mut tools = BTreeMap::new();

    for lang in &ctx.languages {
        match lang {
            Language::Rust => detect_rust_tools(&mut tools),
            Language::Python => detect_python_tools(&mut tools),
            Language::Cpp => detect_cpp_tools(&mut tools),
            Language::Ros2 => {} // ROS2 uses colcon, handled separately
        }
    }

    ToolChain { tools }
}

fn detect_rust_tools(tools: &mut BTreeMap<(String, String), ResolvedTool>) {
    let lang = Language::Rust;
    let label = "[rust]".to_string();

    // cargo is required for Rust — if it's not there, nothing works
    if !tool_exists("cargo") {
        return;
    }

    let entries: Vec<(Operation, Vec<&str>)> = vec![
        (Operation::Fmt, vec!["fmt"]),
        (Operation::Lint, vec!["clippy", "--", "-D", "warnings"]),
        (Operation::Test, vec!["test"]),
        (Operation::Build, vec!["build"]),
        (Operation::Doc, vec!["doc", "--no-deps"]),
        (Operation::Bench, vec!["bench"]),
        (Operation::Check, vec!["check"]),
    ];

    for (op, args) in entries {
        tools.insert(
            (lang.to_string(), op.to_string()),
            ResolvedTool {
                bin: "cargo".to_string(),
                default_args: args.into_iter().map(String::from).collect(),
                language: lang,
                label: label.clone(),
            },
        );
    }
}

fn detect_python_tools(tools: &mut BTreeMap<(String, String), ResolvedTool>) {
    let lang = Language::Python;
    let label = "[python]".to_string();

    // Formatter: ruff > black > skip
    if tool_exists("ruff") {
        tools.insert(
            (lang.to_string(), Operation::Fmt.to_string()),
            ResolvedTool {
                bin: "ruff".to_string(),
                default_args: vec!["format".to_string(), ".".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
        // Linter: ruff check
        tools.insert(
            (lang.to_string(), Operation::Lint.to_string()),
            ResolvedTool {
                bin: "ruff".to_string(),
                default_args: vec!["check".to_string(), ".".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
    } else if tool_exists("black") {
        tools.insert(
            (lang.to_string(), Operation::Fmt.to_string()),
            ResolvedTool {
                bin: "black".to_string(),
                default_args: vec![".".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
    }

    // If no ruff for linting, try pylint or flake8
    if !tools.contains_key(&(lang.to_string(), Operation::Lint.to_string())) {
        if tool_exists("pylint") {
            tools.insert(
                (lang.to_string(), Operation::Lint.to_string()),
                ResolvedTool {
                    bin: "pylint".to_string(),
                    default_args: vec![".".to_string()],
                    language: lang,
                    label: label.clone(),
                },
            );
        } else if tool_exists("flake8") {
            tools.insert(
                (lang.to_string(), Operation::Lint.to_string()),
                ResolvedTool {
                    bin: "flake8".to_string(),
                    default_args: vec![".".to_string()],
                    language: lang,
                    label: label.clone(),
                },
            );
        }
    }

    // Test runner: pytest > python -m unittest
    if tool_exists("pytest") {
        tools.insert(
            (lang.to_string(), Operation::Test.to_string()),
            ResolvedTool {
                bin: "pytest".to_string(),
                default_args: vec![],
                language: lang,
                label: label.clone(),
            },
        );
    } else {
        // Fallback to unittest
        let python = find_python();
        if let Some(py) = python {
            tools.insert(
                (lang.to_string(), Operation::Test.to_string()),
                ResolvedTool {
                    bin: py,
                    default_args: vec!["-m".to_string(), "unittest".to_string(), "discover".to_string()],
                    language: lang,
                    label: label.clone(),
                },
            );
        }
    }

    // Doc generator: pdoc > sphinx-build > skip
    if tool_exists("pdoc") {
        tools.insert(
            (lang.to_string(), Operation::Doc.to_string()),
            ResolvedTool {
                bin: "pdoc".to_string(),
                default_args: vec!["--html".to_string(), ".".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
    } else if tool_exists("sphinx-build") {
        tools.insert(
            (lang.to_string(), Operation::Doc.to_string()),
            ResolvedTool {
                bin: "sphinx-build".to_string(),
                default_args: vec!["docs".to_string(), "_build".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
    }

    // Bench: pytest-benchmark (detected at runtime via pytest --benchmark-only)
    if tool_exists("pytest") {
        tools.insert(
            (lang.to_string(), Operation::Bench.to_string()),
            ResolvedTool {
                bin: "pytest".to_string(),
                default_args: vec!["--benchmark-only".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
    }
}

fn detect_cpp_tools(tools: &mut BTreeMap<(String, String), ResolvedTool>) {
    let lang = Language::Cpp;
    let label = "[cpp]".to_string();

    // clang-format for formatting
    if tool_exists("clang-format") {
        tools.insert(
            (lang.to_string(), Operation::Fmt.to_string()),
            ResolvedTool {
                bin: "clang-format".to_string(),
                default_args: vec!["-i".to_string(), "--style=file".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
    }

    // clang-tidy for linting
    if tool_exists("clang-tidy") {
        tools.insert(
            (lang.to_string(), Operation::Lint.to_string()),
            ResolvedTool {
                bin: "clang-tidy".to_string(),
                default_args: vec![],
                language: lang,
                label: label.clone(),
            },
        );
    }

    // cmake for build
    if tool_exists("cmake") {
        tools.insert(
            (lang.to_string(), Operation::Build.to_string()),
            ResolvedTool {
                bin: "cmake".to_string(),
                default_args: vec!["--build".to_string(), ".".to_string()],
                language: lang,
                label: label.clone(),
            },
        );
    }
}

// ─── Dispatch ────────────────────────────────────────────────────────────────

/// Result of dispatching a command.
#[derive(Debug)]
pub struct DispatchResult {
    /// Language this result is for.
    pub language: Language,

    /// Exit status of the subprocess.
    pub status: ExitStatus,

    /// Label used for output prefixing.
    pub label: String,
}

impl DispatchResult {
    pub fn success(&self) -> bool {
        self.status.success()
    }
}

/// Dispatch an operation for a single resolved tool.
///
/// Runs the tool as a subprocess, inheriting stdin/stdout/stderr.
/// Extra args are appended after the tool's default args.
pub fn dispatch_tool(
    tool: &ResolvedTool,
    extra_args: &[String],
    working_dir: &Path,
) -> anyhow::Result<DispatchResult> {
    let mut cmd = Command::new(&tool.bin);
    cmd.args(&tool.default_args);
    cmd.args(extra_args);
    cmd.current_dir(working_dir);

    // Inherit stdio for interactive output
    cmd.stdin(Stdio::inherit());
    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd
        .status()
        .map_err(|e| anyhow::anyhow!("Failed to run '{}': {}", tool.bin, e))?;

    Ok(DispatchResult {
        language: tool.language,
        status,
        label: tool.label.clone(),
    })
}

/// Dispatch an operation across all languages in the project.
///
/// For single-language projects, runs the tool directly.
/// For mixed projects, runs tools sequentially (one per language).
/// Returns results for each language, or an error if no tool is available.
pub fn dispatch_operation(
    ctx: &ProjectContext,
    toolchain: &ToolChain,
    operation: Operation,
    extra_args: &[String],
) -> anyhow::Result<Vec<DispatchResult>> {
    let tools = toolchain.tools_for(operation);

    if tools.is_empty() {
        return Err(anyhow::anyhow!(
            "No tools available for '{}' in this project. Detected languages: {}",
            operation,
            ctx.languages
                .iter()
                .map(|l| l.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }

    let mut results = Vec::new();
    for tool in tools {
        let result = dispatch_tool(tool, extra_args, &ctx.root)?;
        results.push(result);
    }

    Ok(results)
}

/// Check if all dispatch results were successful.
pub fn all_succeeded(results: &[DispatchResult]) -> bool {
    results.iter().all(|r| r.success())
}

/// Get the worst exit code from dispatch results.
pub fn worst_exit_code(results: &[DispatchResult]) -> i32 {
    results
        .iter()
        .filter_map(|r| r.status.code())
        .max()
        .unwrap_or(0)
}

// ─── Tool probing ────────────────────────────────────────────────────────────

/// Check if a tool binary exists on PATH.
fn tool_exists(name: &str) -> bool {
    Command::new("which")
        .arg(name)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Find the Python interpreter (python3 > python).
fn find_python() -> Option<String> {
    if tool_exists("python3") {
        Some("python3".to_string())
    } else if tool_exists("python") {
        Some("python".to_string())
    } else {
        None
    }
}

/// Probe a tool for its version string.
///
/// Runs `<tool> --version` and returns the first line of stdout.
pub fn tool_version(name: &str) -> Option<String> {
    Command::new(name)
        .arg("--version")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .ok()
        .and_then(|out| {
            if out.status.success() {
                let stdout = String::from_utf8_lossy(&out.stdout);
                let stderr = String::from_utf8_lossy(&out.stderr);
                // Some tools print version to stderr (rustc, python)
                let version_line = if stdout.trim().is_empty() {
                    stderr.lines().next()
                } else {
                    stdout.lines().next()
                };
                version_line.map(|s| s.trim().to_string())
            } else {
                None
            }
        })
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn detect_empty_project() {
        let dir = tempfile::tempdir().unwrap();
        let ctx = detect_context(dir.path());
        assert!(ctx.languages.is_empty());
        assert!(!ctx.has_horus_toml);
        assert!(ctx.manifest.is_none());
    }

    #[test]
    fn detect_rust_from_rs_file() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("main.rs"), "fn main() {}").unwrap();
        let ctx = detect_context(dir.path());
        assert!(ctx.has_rust());
        assert!(!ctx.has_python());
    }

    #[test]
    fn detect_python_from_py_file() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("main.py"), "print('hello')").unwrap();
        let ctx = detect_context(dir.path());
        assert!(ctx.has_python());
        assert!(!ctx.has_rust());
    }

    #[test]
    fn detect_mixed_project() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("main.rs"), "fn main() {}").unwrap();
        fs::write(dir.path().join("helper.py"), "pass").unwrap();
        let ctx = detect_context(dir.path());
        assert!(ctx.has_rust());
        assert!(ctx.has_python());
        assert!(ctx.is_mixed());
    }

    #[test]
    fn detect_rust_from_src_dir() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir(dir.path().join("src")).unwrap();
        fs::write(dir.path().join("src/main.rs"), "fn main() {}").unwrap();
        let ctx = detect_context(dir.path());
        assert!(ctx.has_rust());
    }

    #[test]
    fn detect_python_from_requirements() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("requirements.txt"), "numpy>=1.24\n").unwrap();
        let ctx = detect_context(dir.path());
        assert!(ctx.has_python());
    }

    #[test]
    fn detect_with_horus_toml() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(
            dir.path().join("horus.toml"),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::write(dir.path().join("main.rs"), "fn main() {}").unwrap();
        let ctx = detect_context(dir.path());
        assert!(ctx.has_horus_toml);
        assert!(ctx.manifest.is_some());
        assert_eq!(ctx.manifest.unwrap().package.name, "test-bot");
    }

    #[test]
    fn toolchain_detects_cargo() {
        // cargo should exist in test environment
        if !tool_exists("cargo") {
            return; // skip on CI without rust
        }
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("main.rs"), "fn main() {}").unwrap();
        let ctx = detect_context(dir.path());
        let tc = detect_toolchain(&ctx);

        assert!(tc.get(Language::Rust, Operation::Fmt).is_some());
        assert!(tc.get(Language::Rust, Operation::Lint).is_some());
        assert!(tc.get(Language::Rust, Operation::Test).is_some());
        assert!(tc.get(Language::Rust, Operation::Build).is_some());

        let fmt = tc.get(Language::Rust, Operation::Fmt).unwrap();
        assert_eq!(fmt.bin, "cargo");
        assert_eq!(fmt.default_args, vec!["fmt"]);
    }

    #[test]
    fn toolchain_no_tools_for_empty() {
        let dir = tempfile::tempdir().unwrap();
        let ctx = detect_context(dir.path());
        let tc = detect_toolchain(&ctx);
        assert!(tc.tools_for(Operation::Fmt).is_empty());
    }

    #[test]
    fn tool_exists_finds_cargo() {
        // cargo should exist in any Rust development environment
        if which_available() {
            assert!(tool_exists("cargo"));
            assert!(!tool_exists("this_tool_definitely_does_not_exist_xyz"));
        }
    }

    #[test]
    fn tool_version_works() {
        if tool_exists("cargo") {
            let version = tool_version("cargo");
            assert!(version.is_some());
            assert!(version.unwrap().contains("cargo"));
        }
    }

    #[test]
    fn dispatch_result_helpers() {
        use std::os::unix::process::ExitStatusExt;
        let results = vec![
            DispatchResult {
                language: Language::Rust,
                status: ExitStatus::from_raw(0),
                label: "[rust]".to_string(),
            },
            DispatchResult {
                language: Language::Python,
                status: ExitStatus::from_raw(256), // exit code 1
                label: "[python]".to_string(),
            },
        ];
        assert!(!all_succeeded(&results));
        assert_eq!(worst_exit_code(&results), 1);
    }

    /// Helper: check if `which` is available (it should be on Linux/macOS).
    fn which_available() -> bool {
        Command::new("which")
            .arg("which")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false)
    }
}
