//! `horus fmt` — format all code in the project.
//!
//! Dispatches to `cargo fmt` (Rust) and `ruff format` / `black` (Python).
//! For mixed projects, runs both in parallel with prefixed output.

use anyhow::Result;
use colored::*;
use std::path::Path;

use crate::dispatch::{self, Operation, ProjectContext};
use crate::run_with_prefix::{self, PrefixedCommand};

/// The Python style HORUS declares for every project it generates.
///
/// `pyproject_gen` writes exactly these two keys into `.horus/pyproject.toml`,
/// one directory *below* the sources they govern — and neither ruff nor black
/// ever looks there, because both resolve configuration by walking *up* from
/// each file. Left to itself `horus fmt` formats a generated project at ruff's
/// own default of 88 columns while the project's own generated config says 100,
/// so a 97-column line is legal and "unformatted" at the same time. Every
/// Python tool horus dispatches to is therefore handed the style explicitly by
/// [`python_style_args`]; `horus_python_style_matches_generated_pyproject`
/// pins these constants to what `pyproject_gen` writes.
pub(crate) const HORUS_PY_LINE_LENGTH: u32 = 100;
/// Companion of [`HORUS_PY_LINE_LENGTH`] — the `target-version` horus declares.
pub(crate) const HORUS_PY_TARGET_VERSION: &str = "py310";

/// Run `horus fmt`.
///
/// - `check`: If true, check formatting without modifying files (exit 1 if unformatted).
/// - `extra_args`: Additional arguments passed through to the underlying tools.
pub fn run_fmt(check: bool, extra_args: Vec<String>) -> Result<()> {
    let ctx = dispatch::detect_context(&std::env::current_dir()?);

    if ctx.languages.is_empty() {
        anyhow::bail!("No source files detected. Nothing to format.");
    }

    let commands = build_fmt_commands(&ctx, check, extra_args);

    if commands.is_empty() {
        eprintln!(
            "{} No formatting tools found. Install {} or {}.",
            "warn:".yellow(),
            "rustfmt".cyan(),
            "ruff".cyan()
        );
        return Ok(());
    }

    let results = run_with_prefix::run_prefixed(commands);
    run_with_prefix::print_summary(&results);

    if !run_with_prefix::all_succeeded(&results) {
        std::process::exit(run_with_prefix::worst_exit_code(&results));
    }

    Ok(())
}

/// Build the exact commands `horus fmt` runs, so tests can execute the same
/// thing the CLI does instead of re-deriving the arguments by hand.
pub(crate) fn build_fmt_commands(
    ctx: &ProjectContext,
    check: bool,
    extra_args: Vec<String>,
) -> Vec<PrefixedCommand> {
    let toolchain = dispatch::detect_toolchain(ctx);
    let tools = toolchain.tools_for(Operation::Fmt);

    if tools.is_empty() {
        return Vec::new();
    }

    // For horus projects (horus.toml without root Cargo.toml), point cargo at .horus/Cargo.toml
    let horus_manifest = ctx.root.join(".horus/Cargo.toml");
    let use_horus_manifest =
        ctx.has_horus_toml && !ctx.root.join("Cargo.toml").exists() && horus_manifest.exists();

    tools
        .into_iter()
        .map(|tool| {
            let mut args = tool.default_args.clone();
            if tool.bin == "cargo" && use_horus_manifest {
                args.push("--manifest-path".to_string());
                args.push(horus_manifest.to_string_lossy().to_string());
            }
            if check {
                match tool.bin.as_str() {
                    "cargo" => args.push("--check".to_string()),
                    "ruff" => args.push("--check".to_string()),
                    "black" => args.push("--check".to_string()),
                    "clang-format" => {
                        // Replace -i (in-place) with --dry-run --Werror (check mode)
                        args.retain(|a| a != "-i");
                        args.push("--dry-run".to_string());
                        args.push("--Werror".to_string());
                    }
                    _ => {}
                }
            }
            // clang-format has no file arguments of its own and cannot be
            // given any at detection time. Without them it reads stdin: `-i`
            // fails outright, and `--dry-run --Werror` sees EOF and exits 0,
            // so `horus fmt --check` passed while checking nothing.
            if tool.bin == "clang-format" {
                args.extend(
                    dispatch::cpp_source_files(&ctx.root)
                        .iter()
                        .map(|p| p.to_string_lossy().to_string()),
                );
            }
            // Hand the Python formatter the project's own style — see
            // `python_style_args`. Placed before `extra_args` so an explicit
            // `horus fmt -- --line-length 79` still wins.
            args.extend(python_style_args(&tool.bin, ctx));
            args.extend(extra_args.clone());

            PrefixedCommand {
                label: tool.label.clone(),
                bin: tool.bin.clone(),
                args,
                working_dir: Some(ctx.root.to_string_lossy().to_string()),
                env: Vec::new(),
            }
        })
        .collect()
}

/// Arguments that make a Python tool obey the style horus declares for the
/// project, instead of the tool's own built-in default.
///
/// Precedence, highest first:
///
/// 1. A style config the *user* owns (`ruff.toml`, `.ruff.toml`, or a
///    `[tool.ruff]`/`[tool.black]`/... table in the root `pyproject.toml`).
///    Horus adds nothing: the tool discovers it by itself and the user wins.
/// 2. `.horus/pyproject.toml`, generated from `horus.toml` by `pyproject_gen`.
///    ruff is pointed straight at it with `--config`; the other tools get the
///    equivalent flags, since they cannot read a `[tool.ruff]` table.
/// 3. Nothing generated yet (a freshly created project, before the first
///    build): the same constants inline, so `horus fmt` behaves identically
///    before and after `horus build` ever runs.
///
/// Non-horus directories (no `horus.toml`) are left completely alone.
pub(crate) fn python_style_args(bin: &str, ctx: &ProjectContext) -> Vec<String> {
    let root = &ctx.root;
    let root_pyproject = root.join("pyproject.toml");

    match bin {
        "ruff" => {
            if root.join("ruff.toml").exists()
                || root.join(".ruff.toml").exists()
                || has_table(&root_pyproject, "[tool.ruff")
            {
                return Vec::new();
            }
            if !ctx.has_horus_toml {
                return Vec::new();
            }
            let generated = root.join(".horus").join("pyproject.toml");
            if has_table(&generated, "[tool.ruff") {
                // `--config <path>` overrides ruff's own discovery for every
                // file in the tree, which is the point: the generated config
                // sits in a directory ruff would otherwise never consult.
                return vec![
                    "--config".to_string(),
                    generated.to_string_lossy().to_string(),
                ];
            }
            // ruff's `--config` also takes inline TOML `key=value` overrides,
            // which `ruff check` needs — it has no `--line-length` flag.
            vec![
                "--config".to_string(),
                format!("line-length={}", HORUS_PY_LINE_LENGTH),
                "--config".to_string(),
                format!("target-version=\"{}\"", HORUS_PY_TARGET_VERSION),
            ]
        }
        "black" => {
            if has_table(&root_pyproject, "[tool.black") || !ctx.has_horus_toml {
                return Vec::new();
            }
            vec![
                "--line-length".to_string(),
                HORUS_PY_LINE_LENGTH.to_string(),
            ]
        }
        "flake8" => {
            let user_owned = root.join(".flake8").exists()
                || root.join("setup.cfg").exists()
                || root.join("tox.ini").exists();
            if user_owned || !ctx.has_horus_toml {
                return Vec::new();
            }
            vec![format!("--max-line-length={}", HORUS_PY_LINE_LENGTH)]
        }
        "pylint" => {
            let user_owned = root.join("pylintrc").exists()
                || root.join(".pylintrc").exists()
                || has_table(&root_pyproject, "[tool.pylint");
            if user_owned || !ctx.has_horus_toml {
                return Vec::new();
            }
            vec![format!("--max-line-length={}", HORUS_PY_LINE_LENGTH)]
        }
        _ => Vec::new(),
    }
}

/// Whether an external tool is available, decided by scanning `PATH` rather
/// than by running it.
///
/// The contract tests below — and the template tests in `new.rs` — skip when a
/// tool is genuinely absent, so this answer decides whether a guard runs at
/// all. Probing by spawning the binary (`dispatch::tool_version`) makes that
/// decision depend on whether a `fork` succeeded, and a machine under load
/// answering "not installed" turns a contract into a silent no-op. Reading
/// `PATH` cannot fail that way.
#[cfg(test)]
pub(crate) fn tool_on_path(bin: &str) -> bool {
    let Some(path) = std::env::var_os("PATH") else {
        return false;
    };
    std::env::split_paths(&path).any(|dir| {
        let candidate = dir.join(bin);
        candidate.is_file() || candidate.with_extension("exe").is_file()
    })
}

/// Skip unless `HORUS_TEST_REQUIRE_TOOLS` is set — the switch CI uses so a
/// runner that happens not to ship ruff cannot quietly turn these contracts
/// into no-ops.
#[cfg(test)]
pub(crate) fn tool_or_skip(bin: &str, label: &str) -> bool {
    if tool_on_path(bin) {
        return true;
    }
    assert!(
        std::env::var_os("HORUS_TEST_REQUIRE_TOOLS").is_none(),
        "{bin} is not installed but HORUS_TEST_REQUIRE_TOOLS is set"
    );
    eprintln!("skipping {label}: {bin} not installed");
    false
}

/// True when `path` is readable and declares a TOML table starting with
/// `prefix` (e.g. `"[tool.ruff"`, which also matches `[tool.ruff.lint]`).
fn has_table(path: &Path, prefix: &str) -> bool {
    std::fs::read_to_string(path)
        .map(|text| text.lines().any(|l| l.trim_start().starts_with(prefix)))
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── LIVE-2: horus and its Python tools must agree on one style ───────
    //
    // `pyproject_gen` writes `[tool.ruff] line-length = 100` into
    // `.horus/pyproject.toml`, which ruff never discovers — it resolves config
    // by walking *up* from each file, and the sources sit one level *above*
    // that file. So `horus fmt` used to reformat at ruff's default of 88 while
    // the project's own config said 100. These tests pin the agreement.

    /// A 97-column line: legal at `line-length = 100`, reformatted at 88.
    const NINETY_SEVEN_COLS: &str = concat!(
        "def send(node, linear):\n",
        "    node.send(\"motors.cmd_vel\", {\"linear\": linear, \"angular\": 0.0, \"padxxxxxxxxxxxxxx\": 1234567})\n",
    );

    fn py_project(root: &Path, main_py: &str, generated_ruff: Option<&str>) {
        std::fs::write(
            root.join("horus.toml"),
            "[package]\nname = \"stylebot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        std::fs::write(root.join("main.py"), main_py).unwrap();
        if let Some(body) = generated_ruff {
            std::fs::create_dir_all(root.join(".horus")).unwrap();
            std::fs::write(root.join(".horus").join("pyproject.toml"), body).unwrap();
        }
    }

    /// Tests that shell out to ruff skip when it is absent — see
    /// [`tool_or_skip`].
    fn ruff_or_skip() -> bool {
        tool_or_skip("ruff", "python")
    }

    fn run(cmd: &PrefixedCommand) -> (bool, String) {
        let out = std::process::Command::new(&cmd.bin)
            .args(&cmd.args)
            .current_dir(cmd.working_dir.as_deref().unwrap())
            .output()
            .unwrap_or_else(|e| panic!("failed to run {}: {e}", cmd.bin));
        let mut text = String::from_utf8_lossy(&out.stdout).into_owned();
        text.push_str(&String::from_utf8_lossy(&out.stderr));
        (out.status.success(), text)
    }

    #[test]
    fn the_sample_line_is_between_ruffs_default_and_horuss_own() {
        // Without this the style tests below are vacuous: an 83-column line
        // is acceptable to ruff's 88-column default *and* to horus's 100, so
        // it would pass with or without the fix.
        let longest = NINETY_SEVEN_COLS
            .lines()
            .map(|l| l.chars().count())
            .max()
            .unwrap();
        assert_eq!(longest, 97);
        assert!(longest > 88, "must exceed ruff's own default line length");
        assert!(
            longest < HORUS_PY_LINE_LENGTH as usize,
            "must fit within the line length horus declares"
        );
    }

    #[test]
    fn python_style_args_points_ruff_at_the_generated_config() {
        let tmp = tempfile::TempDir::new().unwrap();
        py_project(
            tmp.path(),
            NINETY_SEVEN_COLS,
            Some("[tool.ruff]\nline-length = 100\n"),
        );
        let ctx = dispatch::detect_context(tmp.path());

        let generated = tmp.path().join(".horus").join("pyproject.toml");
        assert_eq!(
            python_style_args("ruff", &ctx),
            vec![
                "--config".to_string(),
                generated.to_string_lossy().to_string()
            ],
            "ruff must be handed the config horus generated; it cannot find it"
        );
    }

    #[test]
    fn python_style_args_applies_horus_defaults_before_the_first_build() {
        // A freshly created project has no .horus/ yet — horus fmt must still
        // use the style it will write there, not ruff's 88-column default.
        let tmp = tempfile::TempDir::new().unwrap();
        py_project(tmp.path(), NINETY_SEVEN_COLS, None);
        let ctx = dispatch::detect_context(tmp.path());

        assert_eq!(
            python_style_args("ruff", &ctx),
            vec![
                "--config".to_string(),
                format!("line-length={}", HORUS_PY_LINE_LENGTH),
                "--config".to_string(),
                format!("target-version=\"{}\"", HORUS_PY_TARGET_VERSION),
            ]
        );
    }

    #[test]
    fn python_style_args_defers_to_a_user_owned_config() {
        for name in ["ruff.toml", ".ruff.toml"] {
            let tmp = tempfile::TempDir::new().unwrap();
            py_project(
                tmp.path(),
                NINETY_SEVEN_COLS,
                Some("[tool.ruff]\nline-length = 100\n"),
            );
            std::fs::write(tmp.path().join(name), "line-length = 79\n").unwrap();
            let ctx = dispatch::detect_context(tmp.path());
            assert!(
                python_style_args("ruff", &ctx).is_empty(),
                "{name} in the project root must win — ruff discovers it itself"
            );
        }

        let tmp = tempfile::TempDir::new().unwrap();
        py_project(tmp.path(), NINETY_SEVEN_COLS, None);
        std::fs::write(
            tmp.path().join("pyproject.toml"),
            "[project]\nname = \"x\"\n\n[tool.ruff]\nline-length = 79\n",
        )
        .unwrap();
        let ctx = dispatch::detect_context(tmp.path());
        assert!(python_style_args("ruff", &ctx).is_empty());
        std::fs::write(
            tmp.path().join("pyproject.toml"),
            "[project]\nname = \"x\"\n\n[tool.black]\nline-length = 79\n",
        )
        .unwrap();
        let ctx = dispatch::detect_context(tmp.path());
        assert!(python_style_args("black", &ctx).is_empty());
    }

    #[test]
    fn python_style_args_leaves_non_horus_directories_alone() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), NINETY_SEVEN_COLS).unwrap();
        let ctx = dispatch::detect_context(tmp.path());
        assert!(!ctx.has_horus_toml);
        for bin in ["ruff", "black", "flake8", "pylint"] {
            assert!(
                python_style_args(bin, &ctx).is_empty(),
                "{bin}: a plain Python directory is none of horus's business"
            );
        }
    }

    #[test]
    fn python_style_args_covers_every_python_tool_horus_dispatches_to() {
        // dispatch.rs falls back to black / flake8 / pylint when ruff is
        // missing; none of them can read a [tool.ruff] table either.
        let tmp = tempfile::TempDir::new().unwrap();
        py_project(tmp.path(), NINETY_SEVEN_COLS, None);
        let ctx = dispatch::detect_context(tmp.path());

        assert_eq!(
            python_style_args("black", &ctx),
            vec![
                "--line-length".to_string(),
                HORUS_PY_LINE_LENGTH.to_string()
            ]
        );
        let max = format!("--max-line-length={}", HORUS_PY_LINE_LENGTH);
        assert_eq!(python_style_args("flake8", &ctx), vec![max.clone()]);
        assert_eq!(python_style_args("pylint", &ctx), vec![max]);
        assert!(python_style_args("cargo", &ctx).is_empty());
        assert!(python_style_args("clang-format", &ctx).is_empty());
    }

    #[test]
    fn horus_python_style_matches_generated_pyproject() {
        // Anti-drift: the constants above and pyproject_gen must never
        // disagree, or horus would go back to declaring one style and
        // enforcing another.
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"stylebot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        let manifest =
            crate::manifest::HorusManifest::load_from(&tmp.path().join("horus.toml")).unwrap();
        let (_, generated) = crate::pyproject_gen::generate(&manifest, tmp.path(), false).unwrap();

        assert!(
            generated.contains(&format!("line-length = {}", HORUS_PY_LINE_LENGTH)),
            "pyproject_gen no longer writes line-length = {}:\n{}",
            HORUS_PY_LINE_LENGTH,
            generated
        );
        assert!(
            generated.contains(&format!("target-version = \"{}\"", HORUS_PY_TARGET_VERSION)),
            "pyproject_gen no longer writes target-version = \"{}\":\n{}",
            HORUS_PY_TARGET_VERSION,
            generated
        );
    }

    #[test]
    fn fmt_check_passes_on_a_line_the_projects_own_config_allows() {
        if !ruff_or_skip() {
            return;
        }
        for generated in [
            None,
            Some("[tool.ruff]\nline-length = 100\ntarget-version = \"py310\"\n"),
        ] {
            let tmp = tempfile::TempDir::new().unwrap();
            py_project(tmp.path(), NINETY_SEVEN_COLS, generated);
            let ctx = dispatch::detect_context(tmp.path());

            let commands = build_fmt_commands(&ctx, true, Vec::new());
            let ruff = commands
                .iter()
                .find(|c| c.bin == "ruff")
                .expect("ruff is installed, so horus fmt must dispatch to it");
            let (ok, output) = run(ruff);
            assert!(
                ok,
                "horus fmt --check rejected a 97-column line that the project's \
                 own config (line-length = {}) allows; generated config: {:?}\n{}",
                HORUS_PY_LINE_LENGTH, generated, output
            );
        }
    }

    #[test]
    fn fmt_writes_files_at_the_projects_own_line_length() {
        // The other half of the same coin: `horus fmt` (no --check) must not
        // rewrap a line the project's config allows.
        if !ruff_or_skip() {
            return;
        }
        let tmp = tempfile::TempDir::new().unwrap();
        py_project(
            tmp.path(),
            NINETY_SEVEN_COLS,
            Some("[tool.ruff]\nline-length = 100\n"),
        );
        let ctx = dispatch::detect_context(tmp.path());

        let commands = build_fmt_commands(&ctx, false, Vec::new());
        let ruff = commands.iter().find(|c| c.bin == "ruff").unwrap();
        let (ok, output) = run(ruff);
        assert!(ok, "horus fmt failed: {output}");

        let after = std::fs::read_to_string(tmp.path().join("main.py")).unwrap();
        assert_eq!(
            after, NINETY_SEVEN_COLS,
            "horus fmt rewrapped a line that line-length = {} allows",
            HORUS_PY_LINE_LENGTH
        );
    }

    #[test]
    fn fmt_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
        assert!(
            result.unwrap_err().to_string().contains("No source files"),
            "Should mention no source files"
        );
    }

    #[test]
    fn fmt_check_mode_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(true, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    // ── Battle tests: dispatch and context ──────────────────────────────

    #[test]
    fn fmt_error_message_is_descriptive() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("No source files detected"),
            "Error should mention 'No source files detected', got: {}",
            err
        );
        assert!(
            err.contains("Nothing to format"),
            "Error should mention 'Nothing to format', got: {}",
            err
        );
    }

    #[test]
    fn fmt_check_error_message_matches_non_check() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let r1 = run_fmt(false, vec![]);
        let r2 = run_fmt(true, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert_eq!(
            r1.unwrap_err().to_string(),
            r2.unwrap_err().to_string(),
            "Both check and non-check should produce the same error for empty dir"
        );
    }

    #[test]
    fn fmt_with_extra_args_still_fails_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec!["--verbose".to_string()]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn fmt_empty_extra_args_is_valid() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, Vec::new());
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn fmt_rust_project_detects_cargo_fmt() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(
            ctx.languages.contains(&crate::manifest::Language::Rust),
            "Should detect Rust from .rs file"
        );

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        if dispatch::tool_version("cargo").is_some() {
            assert!(
                !tools.is_empty(),
                "Rust project should have at least cargo fmt"
            );
            let rust_tool = tools
                .iter()
                .find(|t| t.bin == "cargo")
                .expect("Should have cargo as fmt tool");
            assert_eq!(rust_tool.default_args, vec!["fmt"]);
            assert_eq!(rust_tool.label, "[rust]");
        }
    }

    #[test]
    fn fmt_python_project_detects_formatter() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), "print('hello')").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(
            ctx.languages.contains(&crate::manifest::Language::Python),
            "Should detect Python from .py file"
        );

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        if !tools.is_empty() {
            let py_tool = &tools[0];
            assert!(
                py_tool.bin == "ruff" || py_tool.bin == "black",
                "Python formatter should be ruff or black, got: {}",
                py_tool.bin
            );
        }
    }

    #[test]
    fn fmt_mixed_project_detects_both_formatters() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("lib.rs"), "pub fn foo() {}").unwrap();
        std::fs::write(tmp.path().join("helper.py"), "pass").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Should detect Rust");
        assert!(ctx.has_python(), "Should detect Python");

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        if dispatch::tool_version("cargo").is_some() {
            let has_rust_tool = tools.iter().any(|t| t.bin == "cargo");
            assert!(has_rust_tool, "Mixed project should include cargo fmt tool");
        }
    }

    #[test]
    fn fmt_check_flag_adds_check_arg_for_cargo() {
        let check = true;
        let bin = "cargo";
        let mut args = vec!["fmt".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec!["fmt", "--check"]);
    }

    #[test]
    fn fmt_check_flag_adds_check_arg_for_ruff() {
        let check = true;
        let bin = "ruff";
        let mut args = vec!["format".to_string(), ".".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec!["format", ".", "--check"]);
    }

    #[test]
    fn fmt_check_flag_adds_check_arg_for_black() {
        let check = true;
        let bin = "black";
        let mut args = vec![".".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec![".", "--check"]);
    }

    #[test]
    fn fmt_no_check_flag_for_unknown_tool() {
        let check = true;
        let bin = "clang-format";
        let mut args = vec!["-i".to_string()];
        if check {
            match bin {
                "cargo" => args.push("--check".to_string()),
                "ruff" => args.push("--check".to_string()),
                "black" => args.push("--check".to_string()),
                _ => {}
            }
        }
        assert_eq!(args, vec!["-i"]);
    }

    #[test]
    fn fmt_extra_args_appended_after_defaults() {
        let default_args = vec!["fmt".to_string()];
        let extra_args = vec!["--verbose".to_string(), "--color=always".to_string()];

        let mut args = default_args;
        args.extend(extra_args.clone());
        assert_eq!(args, vec!["fmt", "--verbose", "--color=always"]);
    }

    #[test]
    fn fmt_horus_toml_only_project_no_source_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"empty-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_fmt(false, vec![]);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn fmt_project_root_from_subdirectory() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"sub-test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();
        let sub = tmp.path().join("subdir");
        std::fs::create_dir(&sub).unwrap();

        let ctx = dispatch::detect_context(&sub);
        assert!(
            ctx.has_horus_toml,
            "Should find horus.toml from subdirectory"
        );
        assert!(ctx.has_rust(), "Should detect Rust from project root");
    }

    #[test]
    fn fmt_src_dir_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        let src = tmp.path().join("src");
        std::fs::create_dir(&src).unwrap();
        std::fs::write(src.join("lib.rs"), "pub fn foo() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Should detect Rust from src/*.rs");
    }

    #[test]
    fn fmt_pyproject_triggers_python_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("pyproject.toml"),
            "[project]\nname = \"mybot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_python(), "Should detect Python from pyproject.toml");
    }

    #[test]
    fn fmt_requirements_txt_triggers_python_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("requirements.txt"), "numpy>=1.24\n").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(
            ctx.has_python(),
            "Should detect Python from requirements.txt"
        );
    }

    #[test]
    fn fmt_cargo_toml_triggers_rust_detection() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
        )
        .unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        assert!(ctx.has_rust(), "Should detect Rust from Cargo.toml");
    }

    #[test]
    fn fmt_commands_have_correct_working_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let ctx = dispatch::detect_context(tmp.path());
        let expected_dir = ctx.root.to_string_lossy().to_string();

        let toolchain = dispatch::detect_toolchain(&ctx);
        let tools = toolchain.tools_for(dispatch::Operation::Fmt);

        for tool in &tools {
            let cmd = run_with_prefix::PrefixedCommand {
                label: tool.label.clone(),
                bin: tool.bin.clone(),
                args: tool.default_args.clone(),
                working_dir: Some(ctx.root.to_string_lossy().to_string()),
                env: Vec::new(),
            };
            assert_eq!(
                cmd.working_dir.as_deref(),
                Some(expected_dir.as_str()),
                "Working dir should be the project root"
            );
        }
    }
}
