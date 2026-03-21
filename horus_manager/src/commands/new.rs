use crate::manifest::{
    DependencyValue, HorusManifest, PackageInfo, TargetType, WorkspaceConfig, HORUS_TOML,
};
use crate::{cli_output, version};
use anyhow::{Context, Result};
use colored::*;
use std::collections::BTreeMap;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

pub fn create_new_project(
    name: String,
    path: Option<PathBuf>,
    language: String,
    use_macro: bool,
    workspace: bool,
    lib: bool,
) -> Result<()> {
    // Validate project name before doing anything
    validate_project_name(&name)?;

    // Check version compatibility before creating project
    version::check_and_prompt_update()?;

    if workspace {
        cli_output::info(&format!(
            "Creating new HORUS workspace '{}'",
            name.green().bold()
        ));
    } else {
        cli_output::info(&format!(
            "Creating new HORUS project '{}'",
            name.green().bold()
        ));
    }

    // Determine project path
    let project_path = if let Some(p) = path {
        p.join(&name)
    } else {
        PathBuf::from(&name)
    };

    // Track if we're in interactive mode
    let is_interactive = language.is_empty();

    // Get language - use flag or prompt
    let language = if is_interactive {
        prompt_language()?
    } else {
        language
    };

    // Ask about macros if Rust was selected interactively (and not already set via flag)
    let use_macro = if language == "rust" && is_interactive {
        prompt_use_macro()?
    } else {
        use_macro
    };

    let description = "A HORUS robotics project".to_string();
    let author = get_author()?;

    // Create project directory
    fs::create_dir_all(&project_path).context("Failed to create project directory")?;

    if workspace {
        // ── Workspace mode ──────────────────────────────────────────
        // Create root .horus/ directory
        create_horus_directory(&project_path)?;

        // Create .gitignore in workspace root
        create_gitignore(&project_path, &language)?;

        // Create root horus.toml with [workspace] config
        create_workspace_horus_toml(&project_path, &name)?;

        // Create crates/ directory and the first member
        let member_dir = project_path.join("crates").join(&name);
        fs::create_dir_all(&member_dir).context("Failed to create member directory")?;

        // Create the member's horus.toml with proper PackageInfo
        create_horus_toml_with_target(
            &member_dir,
            &name,
            &description,
            &author,
            TargetType::Bin,
        )?;

        // Create the member's src/main.rs
        match language.as_str() {
            "rust" => {
                create_main_rs(&member_dir, use_macro)?;
            }
            "python" => {
                create_main_py(&member_dir)?;
            }
            "cpp" => {
                create_cpp_project(&member_dir, &name)?;
            }
            other => anyhow::bail!("Unsupported language: {}", other),
        }
    } else {
        // ── Single-package mode ─────────────────────────────────────
        // Create .horus/ directory structure
        create_horus_directory(&project_path)?;

        // Create .gitignore in project root
        create_gitignore(&project_path, &language)?;

        // Determine target type
        let target_type = if lib {
            TargetType::Lib
        } else {
            TargetType::default()
        };

        // Generate horus.toml (deps added later via `horus add`)
        create_horus_toml_with_target(
            &project_path,
            &name,
            &description,
            &author,
            target_type,
        )?;

        // Generate main source file based on language
        // Note: native build files (Cargo.toml, pyproject.toml) are generated
        // automatically from horus.toml by the build pipeline (cargo_gen/pyproject_gen).
        if lib && language == "rust" {
            create_lib_rs(&project_path, &name)?;
        } else {
            match language.as_str() {
                "rust" => {
                    create_main_rs(&project_path, use_macro)?;
                }
                "python" => {
                    create_main_py(&project_path)?;
                }
                "cpp" => {
                    create_cpp_project(&project_path, &name)?;
                }
                other => anyhow::bail!("Unsupported language: {}", other),
            }
        }
    }

    // Register workspace in ~/.horus/workspaces.json
    // This makes it visible in monitors (horus monitor / horus monitor -t)
    if let Ok(mut registry) = crate::workspace::WorkspaceRegistry::load() {
        if let Ok(canonical_path) = project_path.canonicalize() {
            if registry.add(name.clone(), canonical_path).is_ok() {
                println!(
                    "  {} Registered workspace in registry",
                    cli_output::ICON_SUCCESS.green()
                );
            }
        }
    }

    println!();
    cli_output::success("Project created successfully!");
    println!("\nTo get started:");
    println!("  {} {}", "cd".cyan(), name);
    println!("  {} (auto-installs dependencies)", "horus run".cyan());

    Ok(())
}

fn prompt_language() -> Result<String> {
    println!("\n{} Select language:", "?".yellow().bold());
    println!("  {} Python", "1.".cyan());
    println!("  {} Rust", "2.".cyan());
    println!("  {} C++", "3.".cyan());

    print!("{} [1-3] (default: 2): ", ">".cyan().bold());
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let input = input.trim();

    let choice = if input.is_empty() { "2" } else { input };

    let language = match choice {
        "1" => "python",
        "2" => "rust",
        "3" => "cpp",
        _ => {
            println!("Invalid choice, defaulting to Rust");
            "rust"
        }
    };

    Ok(language.to_string())
}

fn prompt_use_macro() -> Result<bool> {
    print!(
        "\n{} Use HORUS macros for simpler syntax? [y/N]: ",
        "?".yellow().bold()
    );
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let input = input.trim().to_lowercase();

    Ok(input == "y" || input == "yes")
}

fn get_author() -> Result<String> {
    // Try to get from git config
    if let Ok(output) = std::process::Command::new("git")
        .args(["config", "user.name"])
        .output()
    {
        if output.status.success() {
            if let Ok(name) = String::from_utf8(output.stdout) {
                let name = name.trim();
                if !name.is_empty() {
                    return Ok(name.to_string());
                }
            }
        }
    }

    // Fallback to username
    Ok(std::env::var("USER").unwrap_or_else(|_| "unknown".to_string()))
}

fn create_horus_directory(project_path: &Path) -> Result<()> {
    let horus_dir = project_path.join(".horus");

    // Create empty .horus/ directory as workspace marker
    // This allows HORUS to recognize this directory as a workspace (workspace.rs:94-96)
    // Subdirectories (packages/, bin/, lib/, include/, cache/) are created
    // automatically by `horus run` on first execution
    fs::create_dir_all(&horus_dir)?;

    Ok(())
}

fn create_gitignore(project_path: &Path, language: &str) -> Result<()> {
    // Create .gitignore in project root
    let mut gitignore_content = String::from(
        r#"# HORUS environment (auto-managed by `horus run`)
.horus/packages/
.horus/bin/
.horus/lib/
.horus/include/
.horus/cache/
.horus/target/
.horus/Cargo.toml
.horus/Cargo.lock
*.log
"#,
    );

    // Add language-specific ignores
    match language {
        "rust" => {
            gitignore_content.push_str(
                r#"
# Rust
target/
Cargo.lock
"#,
            );
        }
        "python" => {
            gitignore_content.push_str(
                r#"
# Python
__pycache__/
*.py[cod]
*$py.class
.pytest_cache/
*.egg-info/
dist/
build/
"#,
            );
        }
        "cpp" => {
            gitignore_content.push_str(
                r#"
# C++
build/
.horus/cpp-build/
compile_commands.json
*.o
*.so
*.a
"#,
            );
        }
        _ => {}
    }

    fs::write(project_path.join(".gitignore"), gitignore_content)?;

    Ok(())
}

#[cfg(test)]
fn create_horus_toml(
    project_path: &Path,
    name: &str,
    description: &str,
    author: &str,
) -> Result<()> {
    let manifest = HorusManifest {
        package: PackageInfo {
            name: name.to_string(),
            version: "0.1.0".to_string(),
            description: Some(description.to_string()),
            authors: vec![author.to_string()],
            license: Some("Apache-2.0".to_string()),
            edition: "1".to_string(),
            repository: None,
            package_type: None,
            categories: Vec::new(),
            standard: None,
            rust_edition: None,
            target_type: TargetType::default(),
        },
        dependencies: Default::default(),
        dev_dependencies: Default::default(),
        sim_dependencies: Default::default(),
        drivers: Default::default(),
        scripts: Default::default(),
        ignore: Default::default(),
        enable: Vec::new(),
        cpp: None,
        hooks: Default::default(),
        workspace: None,
    };

    manifest.save_to(&project_path.join(HORUS_TOML))?;

    Ok(())
}

/// Create a horus.toml with a specific `TargetType`.
fn create_horus_toml_with_target(
    project_path: &Path,
    name: &str,
    description: &str,
    author: &str,
    target_type: TargetType,
) -> Result<()> {
    let manifest = HorusManifest {
        package: PackageInfo {
            name: name.to_string(),
            version: "0.1.0".to_string(),
            description: Some(description.to_string()),
            authors: vec![author.to_string()],
            license: Some("Apache-2.0".to_string()),
            edition: "1".to_string(),
            repository: None,
            package_type: None,
            categories: Vec::new(),
            standard: None,
            rust_edition: None,
            target_type,
        },
        dependencies: Default::default(),
        dev_dependencies: Default::default(),
        sim_dependencies: Default::default(),
        drivers: Default::default(),
        scripts: Default::default(),
        ignore: Default::default(),
        enable: Vec::new(),
        cpp: None,
        hooks: Default::default(),
        workspace: None,
    };

    manifest.save_to(&project_path.join(HORUS_TOML))?;

    Ok(())
}

/// Create a workspace-root horus.toml with `[workspace]` configuration.
fn create_workspace_horus_toml(project_path: &Path, name: &str) -> Result<()> {
    let mut ws_deps = BTreeMap::new();
    ws_deps.insert(
        "horus_library".to_string(),
        DependencyValue::Simple("0.1.9".to_string()),
    );

    let manifest = HorusManifest {
        package: PackageInfo {
            name: name.to_string(),
            version: "0.1.0".to_string(),
            description: Some("A HORUS robotics workspace".to_string()),
            authors: Vec::new(),
            license: None,
            edition: "1".to_string(),
            repository: None,
            package_type: None,
            categories: Vec::new(),
            standard: None,
            rust_edition: None,
            target_type: TargetType::default(),
        },
        dependencies: Default::default(),
        dev_dependencies: Default::default(),
        sim_dependencies: Default::default(),
        drivers: Default::default(),
        scripts: Default::default(),
        ignore: Default::default(),
        enable: Vec::new(),
        cpp: None,
        hooks: Default::default(),
        workspace: Some(WorkspaceConfig {
            members: vec!["crates/*".to_string()],
            exclude: Vec::new(),
            dependencies: ws_deps,
        }),
    };

    manifest.save_to(&project_path.join(HORUS_TOML))?;

    Ok(())
}

/// Create a minimal lib.rs for library crates.
fn create_lib_rs(project_path: &Path, name: &str) -> Result<()> {
    let content = format!("//! {} library crate\n", name);
    let src_dir = project_path.join("src");
    fs::create_dir_all(&src_dir)?;
    fs::write(src_dir.join("lib.rs"), content)?;
    Ok(())
}

fn create_main_rs(project_path: &Path, use_macro: bool) -> Result<()> {
    let content = if use_macro {
        // Macro version - clean and concise (RECOMMENDED - auto-generates monitoring metadata)
        r#"// Mobile robot controller

use horus::prelude::*;
use horus_macros::node;

node! {
    Controller {
        pub {
            cmd_vel: CmdVel -> "motors.cmd_vel",
        }

        tick {
            // Your control logic here
            let msg = CmdVel::new(1.0, 0.0);

            self.cmd_vel.send(msg);
        }
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Add the controller node with priority 0 (highest)
    scheduler.add(Controller::new()).order(0).build();

    // Run the scheduler (Ctrl+C to stop)
    scheduler.run()
}
"#
    } else {
        // Non-macro version
        r#"// Mobile robot controller

use horus::prelude::*;
use horus::core::node::TopicMetadata;

struct Controller {
    cmd_vel: Topic<CmdVel>,
}

impl Controller {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_vel: Topic::new("motors.cmd_vel")?,
        })
    }
}

impl Node for Controller {
    fn name(&self) -> &str {
        "controller"
    }

    fn tick(&mut self) {
        // Your control logic here
        let msg = CmdVel::new(1.0, 0.0);

        self.cmd_vel.send(msg);
    }

    fn publishers(&self) -> Vec<TopicMetadata> {
        vec![
            TopicMetadata {
                topic_name: "motors.cmd_vel".to_string(),
                type_name: std::any::type_name::<CmdVel>().to_string(),
            }
        ]
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Add the controller node with priority 0 (highest)
    scheduler.add(Controller::new()?).order(0).build();

    // Run the scheduler (Ctrl+C to stop)
    scheduler.run()
}
"#
    };

    let src_dir = project_path.join("src");
    fs::create_dir_all(&src_dir)?;
    fs::write(src_dir.join("main.rs"), content)?;
    Ok(())
}

/// Validate project name against Cargo crate naming rules.
///
/// Rules: non-empty, ASCII alphanumeric + hyphens + underscores, must start
/// with a letter or underscore. Rejects empty, spaces, special chars, etc.
fn validate_project_name(name: &str) -> Result<()> {
    if name.is_empty() {
        anyhow::bail!(
            "Project name cannot be empty.\n  \
             Usage: horus new <name> [--rust|--python]\n  \
             Example: horus new my_robot --rust"
        );
    }

    if name.len() > 64 {
        anyhow::bail!(
            "Project name '{}...' is too long ({} chars, max 64).",
            &name[..20],
            name.len()
        );
    }

    // Must start with letter or underscore
    let first = name.chars().next().unwrap();
    if !first.is_ascii_alphabetic() && first != '_' {
        anyhow::bail!(
            "Project name '{}' must start with a letter or underscore.\n  \
             Valid examples: my_robot, _internal, robot2",
            name
        );
    }

    // Only allow ASCII alphanumeric, hyphens, underscores
    for ch in name.chars() {
        if !ch.is_ascii_alphanumeric() && ch != '-' && ch != '_' {
            anyhow::bail!(
                "Project name '{}' contains invalid character '{}'.\n  \
                 Only ASCII letters, digits, hyphens (-), and underscores (_) are allowed.\n  \
                 Valid examples: my_robot, diff-drive, robot2",
                name,
                ch
            );
        }
    }

    Ok(())
}

pub(crate) fn create_main_py(project_path: &Path) -> Result<()> {
    let content = r#"# Mobile robot controller

import horus

def controller(node):
    """Main control logic - called repeatedly at the specified rate."""
    # Your control logic here
    # Check for incoming messages
    if node.has_msg("sensors.data"):
        sensor_data = node.get("sensors.data")
        # Process sensor data...

    # Send control commands
    cmd_vel = {"linear": 1.0, "angular": 0.0}
    node.send("motors.cmd_vel", cmd_vel)

# Create the node
node = horus.Node(
    name="controller",
    pubs="motors.cmd_vel",    # Topics to publish to
    subs="sensors.data",      # Topics to subscribe from
    tick=controller,          # Function to call repeatedly
    rate=30                   # Hz (30 times per second)
)

if __name__ == "__main__":
    # Run the node
    horus.run(node)
"#;
    fs::write(project_path.join("main.py"), content)?;
    Ok(())
}

fn create_cpp_project(project_path: &Path, name: &str) -> Result<()> {
    // Create CMakeLists.txt
    let cmake_content = format!(
        r#"cmake_minimum_required(VERSION 3.16)
project({name} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(horus QUIET)

add_executable(${{PROJECT_NAME}} src/main.cpp)

if(horus_FOUND)
    target_link_libraries(${{PROJECT_NAME}} PRIVATE horus::horus)
endif()
"#
    );
    fs::write(project_path.join("CMakeLists.txt"), cmake_content)?;

    // Create src/main.cpp
    let cpp_content = format!(
        r#"// Mobile robot controller

#include <cstdio>

// TODO: #include <horus/horus.h> when C++ bindings are available

int main() {{
    std::printf("[{name}] HORUS C++ node starting\\n");

    // Your control logic here
    // Once C++ bindings are available:
    //   auto node = horus::Node("{name}");
    //   node.subscribe("sensors.data");
    //   node.advertise("motors.cmd_vel");
    //   node.spin();

    return 0;
}}
"#
    );
    let src_dir = project_path.join("src");
    fs::create_dir_all(&src_dir)?;
    fs::write(src_dir.join("main.cpp"), cpp_content)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── validate_project_name ─────────────────────────────────────────

    #[test]
    fn valid_names_pass() {
        for name in &[
            "my_robot",
            "diff-drive",
            "robot2",
            "_internal",
            "a",
            "A_B_c",
        ] {
            assert!(
                validate_project_name(name).is_ok(),
                "should accept '{}'",
                name
            );
        }
    }

    #[test]
    fn empty_name_rejected() {
        let err = validate_project_name("").unwrap_err();
        assert!(err.to_string().contains("cannot be empty"));
    }

    #[test]
    fn name_starting_with_digit_rejected() {
        let err = validate_project_name("2fast").unwrap_err();
        assert!(err.to_string().contains("must start with a letter"));
    }

    #[test]
    fn name_starting_with_hyphen_rejected() {
        let err = validate_project_name("-bad").unwrap_err();
        assert!(err.to_string().contains("must start with a letter"));
    }

    #[test]
    fn name_with_spaces_rejected() {
        let err = validate_project_name("my robot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_special_chars_rejected() {
        for name in &["my@robot", "a/b", "a.b", "a+b", "pkg!"] {
            let err = validate_project_name(name).unwrap_err();
            assert!(
                err.to_string().contains("invalid character"),
                "'{}' should be rejected",
                name
            );
        }
    }

    #[test]
    fn name_too_long_rejected() {
        let long_name = "a".repeat(65);
        let err = validate_project_name(&long_name).unwrap_err();
        assert!(err.to_string().contains("too long"));
    }

    #[test]
    fn name_at_max_length_accepted() {
        let name = "a".repeat(64);
        assert!(validate_project_name(&name).is_ok());
    }

    // ── create_horus_toml ─────────────────────────────────────────────

    #[test]
    fn horus_toml_has_correct_fields() {
        let dir = tempfile::tempdir().unwrap();
        create_horus_toml(dir.path(), "test-bot", "A test", "alice").unwrap();

        let path = dir.path().join(HORUS_TOML);
        assert!(path.exists());

        let manifest = HorusManifest::load_from(&path).unwrap();
        assert_eq!(manifest.package.name, "test-bot");
        assert_eq!(manifest.package.version, "0.1.0");
        assert_eq!(manifest.package.description.as_deref(), Some("A test"));
        assert_eq!(manifest.package.authors, vec!["alice"]);
        assert_eq!(manifest.package.license.as_deref(), Some("Apache-2.0"));
        assert!(manifest.dependencies.is_empty());
        assert!(manifest.scripts.is_empty());
    }

    // ── create_horus_directory ────────────────────────────────────────

    #[test]
    fn horus_directory_created() {
        let dir = tempfile::tempdir().unwrap();
        create_horus_directory(dir.path()).unwrap();
        assert!(dir.path().join(".horus").is_dir());
    }

    #[test]
    fn horus_directory_idempotent() {
        let dir = tempfile::tempdir().unwrap();
        create_horus_directory(dir.path()).unwrap();
        create_horus_directory(dir.path()).unwrap(); // second call should not fail
        assert!(dir.path().join(".horus").is_dir());
    }

    // ── create_gitignore ─────────────────────────────────────────────

    #[test]
    fn gitignore_rust_includes_target() {
        let dir = tempfile::tempdir().unwrap();
        create_gitignore(dir.path(), "rust").unwrap();
        let content = fs::read_to_string(dir.path().join(".gitignore")).unwrap();
        assert!(content.contains("target/"));
        assert!(content.contains(".horus/"));
    }

    #[test]
    fn gitignore_python_includes_pycache() {
        let dir = tempfile::tempdir().unwrap();
        create_gitignore(dir.path(), "python").unwrap();
        let content = fs::read_to_string(dir.path().join(".gitignore")).unwrap();
        assert!(content.contains("__pycache__/"));
        assert!(content.contains(".horus/"));
    }

    #[test]
    fn gitignore_unknown_lang_still_has_horus() {
        let dir = tempfile::tempdir().unwrap();
        create_gitignore(dir.path(), "unknown_lang").unwrap();
        let content = fs::read_to_string(dir.path().join(".gitignore")).unwrap();
        assert!(content.contains(".horus/"));
    }

    // ── create_main_rs ───────────────────────────────────────────────

    #[test]
    fn main_rs_without_macro() {
        let dir = tempfile::tempdir().unwrap();
        create_main_rs(dir.path(), false).unwrap();
        let content = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(content.contains("impl Node for Controller"));
        assert!(!content.contains("node!"));
    }

    #[test]
    fn main_rs_with_macro() {
        let dir = tempfile::tempdir().unwrap();
        create_main_rs(dir.path(), true).unwrap();
        let content = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(content.contains("node!"));
        assert!(content.contains("horus_macros"));
    }

    // ── create_main_py ───────────────────────────────────────────────

    #[test]
    fn main_py_created() {
        let dir = tempfile::tempdir().unwrap();
        create_main_py(dir.path()).unwrap();
        let content = fs::read_to_string(dir.path().join("main.py")).unwrap();
        assert!(content.contains("import horus"));
        assert!(content.contains("horus.run(node)"));
    }

    // ── full create_new_project ──────────────────────────────────────

    #[test]
    fn create_rust_project_full() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "test_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("test_bot");
        assert!(project.join(HORUS_TOML).exists());
        assert!(project.join("src/main.rs").exists());
        assert!(project.join(".horus").is_dir());
        assert!(project.join(".gitignore").exists());
    }

    #[test]
    fn create_python_project_full() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "py_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("py_bot");
        assert!(project.join(HORUS_TOML).exists());
        assert!(project.join("main.py").exists());
        assert!(project.join(".horus").is_dir());
    }

    #[test]
    fn create_project_invalid_name_fails() {
        let dir = tempfile::tempdir().unwrap();
        let err = create_new_project(
            "".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap_err();
        assert!(err.to_string().contains("cannot be empty"));
    }

    #[test]
    fn create_project_unsupported_language_fails() {
        let dir = tempfile::tempdir().unwrap();
        let err = create_new_project(
            "test_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "java".to_string(),
            false,
            false,
            false,
        )
        .unwrap_err();
        assert!(err.to_string().contains("Unsupported language"));
    }

    // ── Battle-testing: Rust project creation ───────────────────────

    #[test]
    fn rust_project_horus_toml_has_correct_metadata() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "rust_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("rust_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        assert_eq!(manifest.package.name, "rust_bot");
        assert_eq!(manifest.package.version, "0.1.0");
        assert!(manifest.package.description.is_some());
        assert!(!manifest.package.authors.is_empty());
        assert_eq!(manifest.package.license.as_deref(), Some("Apache-2.0"));
        assert_eq!(manifest.package.edition, "1");
    }

    #[test]
    fn rust_project_main_rs_has_node_impl() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "rust_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("rust_bot");
        let content = fs::read_to_string(project.join("src/main.rs")).unwrap();

        // Non-macro project must have explicit Node impl
        assert!(content.contains("impl Node for Controller"));
        assert!(content.contains("fn tick("));
        assert!(content.contains("fn main()"));
        assert!(content.contains("Scheduler::new()"));
    }

    #[test]
    fn rust_project_has_horus_dir_and_gitignore() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "rust_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("rust_bot");
        assert!(project.join(".horus").is_dir());

        let gitignore = fs::read_to_string(project.join(".gitignore")).unwrap();
        assert!(gitignore.contains("target/"), "gitignore missing target/");
        assert!(
            gitignore.contains(".horus/"),
            "gitignore missing .horus/ entries"
        );
        assert!(
            gitignore.contains("Cargo.lock"),
            "gitignore missing Cargo.lock"
        );
    }

    // ── Battle-testing: Python project creation ─────────────────────

    #[test]
    fn python_project_horus_toml_and_main_py() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "py_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("py_bot");

        // horus.toml exists and loads
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.package.name, "py_bot");

        // main.py has horus.run()
        let content = fs::read_to_string(project.join("main.py")).unwrap();
        assert!(content.contains("import horus"));
        assert!(content.contains("horus.run(node)"));
        assert!(content.contains("horus.Node("));
    }

    #[test]
    fn python_project_gitignore_has_pycache() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "py_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("py_bot");
        let gitignore = fs::read_to_string(project.join(".gitignore")).unwrap();
        assert!(
            gitignore.contains("__pycache__/"),
            "gitignore missing __pycache__/"
        );
        assert!(
            gitignore.contains("*.py[cod]"),
            "gitignore missing *.py[cod]"
        );
        assert!(
            gitignore.contains(".pytest_cache/"),
            "gitignore missing .pytest_cache/"
        );
    }

    // ── Battle-testing: horus.toml validation ───────────────────────

    #[test]
    fn generated_horus_toml_validates_without_errors() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "valid_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("valid_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // validate() returns Ok(warnings), should have no errors
        let warnings = manifest
            .validate()
            .expect("manifest validation should pass");
        // Warnings are acceptable, but errors are not (validate returns Err on error)
        let _ = warnings;
    }

    #[test]
    fn generated_horus_toml_roundtrips_through_save_load() {
        let dir = tempfile::tempdir().unwrap();
        create_horus_toml(dir.path(), "roundtrip-bot", "Test roundtrip", "bob").unwrap();

        let path = dir.path().join(HORUS_TOML);
        let original = HorusManifest::load_from(&path).unwrap();

        // Save again and reload
        let path2 = dir.path().join("horus2.toml");
        original.save_to(&path2).unwrap();
        let reloaded = HorusManifest::load_from(&path2).unwrap();

        assert_eq!(original.package.name, reloaded.package.name);
        assert_eq!(original.package.version, reloaded.package.version);
        assert_eq!(original.package.description, reloaded.package.description);
        assert_eq!(original.package.authors, reloaded.package.authors);
        assert_eq!(original.package.license, reloaded.package.license);
    }

    // ── Battle-testing: cargo_gen integration ───────────────────────

    #[test]
    fn rust_project_cargo_gen_produces_valid_cargo_toml() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "cargo_test".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("cargo_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // Generate .horus/Cargo.toml from the manifest
        let (cargo_path, _) = crate::cargo_gen::generate(&manifest, &project, &[], false).unwrap();

        assert!(cargo_path.exists());
        let content = fs::read_to_string(&cargo_path).unwrap();

        // Basic Cargo.toml structure checks
        assert!(content.contains("[package]"), "missing [package] section");
        assert!(
            content.contains("name = \"cargo_test\""),
            "wrong package name in Cargo.toml"
        );
        assert!(
            content.contains("version = \"0.1.0\""),
            "wrong version in Cargo.toml"
        );
        assert!(
            content.contains("edition = \"2021\""),
            "missing Rust edition"
        );
        assert!(
            content.contains("[workspace]"),
            "missing workspace isolation"
        );
        assert!(
            content.contains("[dependencies]"),
            "missing [dependencies] section"
        );
        assert!(
            content.contains("[[bin]]"),
            "missing [[bin]] entry for main.rs"
        );
        assert!(
            content.contains("main.rs"),
            "bin path should reference main.rs"
        );
    }

    // ── Battle-testing: pyproject_gen integration ────────────────────

    #[test]
    fn python_project_pyproject_gen_produces_valid_pyproject_toml() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "pyproj_test".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("pyproj_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // Generate .horus/pyproject.toml from the manifest
        let (pyproj_path, _) = crate::pyproject_gen::generate(&manifest, &project, false).unwrap();

        assert!(pyproj_path.exists());
        let content = fs::read_to_string(&pyproj_path).unwrap();

        // PEP 621 compliance checks
        assert!(content.contains("[build-system]"), "missing [build-system]");
        assert!(content.contains("[project]"), "missing [project] section");
        assert!(
            content.contains("name = \"pyproj_test\""),
            "wrong project name"
        );
        assert!(content.contains("version = \"0.1.0\""), "wrong version");
        assert!(
            content.contains("requires-python"),
            "missing requires-python"
        );
        assert!(
            content.contains("dependencies = []"),
            "new project should have empty dependencies"
        );
    }

    // ── Battle-testing: edge cases ──────────────────────────────────

    #[test]
    fn unicode_project_name_rejected() {
        // Names with unicode embedded after ASCII start -> "invalid character"
        for name in &["robot\u{1F916}", "r\u{00F6}bot"] {
            let err = validate_project_name(name).unwrap_err();
            assert!(
                err.to_string().contains("invalid character"),
                "unicode name '{}' should be rejected for invalid character",
                name
            );
        }

        // Names starting with non-ASCII -> "must start with a letter"
        let err = validate_project_name("\u{4E16}\u{754C}").unwrap_err();
        assert!(
            err.to_string().contains("must start with a letter"),
            "CJK name should be rejected for bad start character"
        );
    }

    #[test]
    fn very_long_name_rejected() {
        // 100 chars
        let name = "a".repeat(100);
        let err = validate_project_name(&name).unwrap_err();
        assert!(err.to_string().contains("too long"));

        // 65 chars (just over the limit)
        let name = "b".repeat(65);
        let err = validate_project_name(&name).unwrap_err();
        assert!(err.to_string().contains("too long"));
    }

    #[test]
    fn duplicate_project_into_existing_dir_succeeds() {
        // create_new_project uses create_dir_all which succeeds on existing dirs
        let dir = tempfile::tempdir().unwrap();

        // First creation
        create_new_project(
            "dup_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        // Second creation into same directory should overwrite files
        let result = create_new_project(
            "dup_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        );

        // Should succeed (overwrites existing files)
        assert!(result.is_ok(), "creating into existing dir should succeed");

        // Verify files are still valid
        let project = dir.path().join("dup_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.package.name, "dup_bot");
    }

    // ── Battle-testing: clean slate (empty sections) ────────────────

    #[test]
    fn generated_horus_toml_has_empty_sections() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "clean_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("clean_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        assert!(
            manifest.dependencies.is_empty(),
            "new project should have no dependencies"
        );
        assert!(
            manifest.dev_dependencies.is_empty(),
            "new project should have no dev-dependencies"
        );
        assert!(
            manifest.drivers.is_empty(),
            "new project should have no drivers"
        );
        assert!(
            manifest.scripts.is_empty(),
            "new project should have no scripts"
        );
        assert!(
            manifest.enable.is_empty(),
            "new project should have no enabled features"
        );
    }

    #[test]
    fn generated_horus_toml_raw_text_has_no_phantom_sections() {
        // Verify that optional empty sections are omitted from the TOML text
        // (skip_serializing_if = "BTreeMap::is_empty" etc.)
        let dir = tempfile::tempdir().unwrap();
        create_horus_toml(dir.path(), "phantom-check", "desc", "author").unwrap();

        let raw = fs::read_to_string(dir.path().join(HORUS_TOML)).unwrap();

        // [package] should be present
        assert!(raw.contains("[package]"), "missing [package]");

        // Empty optional sections should NOT appear in output
        assert!(
            !raw.contains("[dependencies]"),
            "[dependencies] should be omitted when empty"
        );
        assert!(
            !raw.contains("[dev-dependencies]"),
            "[dev-dependencies] should be omitted when empty"
        );
        assert!(
            !raw.contains("[drivers]"),
            "[drivers] should be omitted when empty"
        );
        assert!(
            !raw.contains("[scripts]"),
            "[scripts] should be omitted when empty"
        );
        assert!(
            !raw.contains("[ignore]"),
            "[ignore] should be omitted when empty"
        );
    }

    // ── Battle-testing: macro vs non-macro Rust projects ────────────

    #[test]
    fn macro_rust_project_valid_source() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "macro_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            true,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("macro_bot");
        let content = fs::read_to_string(project.join("src/main.rs")).unwrap();

        // Macro project specifics
        assert!(
            content.contains("node!"),
            "macro project should use node! macro"
        );
        assert!(
            content.contains("use horus_macros::node"),
            "macro project should import horus_macros"
        );
        assert!(content.contains("fn main()"), "must have main()");
        assert!(
            content.contains("Scheduler::new()"),
            "must create Scheduler"
        );
        // Should NOT have explicit impl Node
        assert!(
            !content.contains("impl Node for"),
            "macro project should not have explicit Node impl"
        );
    }

    #[test]
    fn non_macro_rust_project_valid_source() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "plain_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("plain_bot");
        let content = fs::read_to_string(project.join("src/main.rs")).unwrap();

        // Non-macro project specifics
        assert!(
            content.contains("impl Node for Controller"),
            "non-macro should have explicit Node impl"
        );
        assert!(
            content.contains("fn name(&self)"),
            "non-macro should implement name()"
        );
        assert!(
            content.contains("fn tick(&mut self)"),
            "non-macro should implement tick()"
        );
        assert!(
            content.contains("fn publishers(&self)"),
            "non-macro should implement publishers()"
        );
        // Should NOT have node! macro
        assert!(
            !content.contains("node!"),
            "non-macro project should not use node! macro"
        );
    }

    #[test]
    fn both_macro_variants_produce_horus_prelude() {
        let dir = tempfile::tempdir().unwrap();

        // Macro variant
        create_main_rs(dir.path(), true).unwrap();
        let macro_content = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(
            macro_content.contains("use horus::prelude::*"),
            "macro variant should use horus prelude"
        );

        // Non-macro variant
        create_main_rs(dir.path(), false).unwrap();
        let plain_content = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(
            plain_content.contains("use horus::prelude::*"),
            "non-macro variant should use horus prelude"
        );
    }

    // ── Battle-testing: name validation edge cases ──────────────────

    #[test]
    fn name_with_only_underscores_accepted() {
        assert!(validate_project_name("___").is_ok());
        assert!(validate_project_name("_a_b_").is_ok());
    }

    #[test]
    fn name_with_leading_underscore_accepted() {
        assert!(validate_project_name("_private").is_ok());
    }

    #[test]
    fn name_with_mixed_case_accepted() {
        // validate_project_name allows uppercase (Cargo crate names can have it)
        assert!(validate_project_name("MyRobot").is_ok());
        assert!(validate_project_name("ROBOT").is_ok());
    }

    #[test]
    fn name_with_consecutive_hyphens_accepted() {
        assert!(validate_project_name("my--robot").is_ok());
    }

    #[test]
    fn name_with_dot_rejected() {
        let err = validate_project_name("my.robot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_tilde_rejected() {
        let err = validate_project_name("my~robot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_colon_rejected() {
        let err = validate_project_name("my:robot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    // ── Battle-testing: project file contents coherence ─────────────

    #[test]
    fn rust_project_no_stale_files() {
        // A new rust project should NOT produce python files
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "rs_only".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("rs_only");
        assert!(project.join("src/main.rs").exists(), "must have main.rs");
        assert!(
            !project.join("main.py").exists(),
            "rust project should not have main.py"
        );
        assert!(
            !project.join("pyproject.toml").exists(),
            "rust project should not have pyproject.toml"
        );
        assert!(
            !project.join("Cargo.toml").exists(),
            "horus new should NOT create Cargo.toml in project root"
        );
    }

    #[test]
    fn python_project_no_stale_files() {
        // A new python project should NOT produce rust files
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "py_only".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("py_only");
        assert!(project.join("main.py").exists(), "must have main.py");
        assert!(
            !project.join("src/main.rs").exists(),
            "python project should not have main.rs"
        );
        assert!(
            !project.join("Cargo.toml").exists(),
            "python project should not have Cargo.toml"
        );
    }

    // ── Battle-testing: cargo_gen with main.rs from horus new ───────

    #[test]
    fn cargo_gen_picks_up_main_rs_from_project_root() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "bin_test".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("bin_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // Generate with no explicit source_files (should auto-detect main.rs)
        let (cargo_path, _) = crate::cargo_gen::generate(&manifest, &project, &[], false).unwrap();
        let content = fs::read_to_string(&cargo_path).unwrap();

        // Should have exactly one [[bin]] entry
        assert_eq!(
            content.matches("[[bin]]").count(),
            1,
            "should have exactly one bin entry"
        );
        assert!(
            content.contains("name = \"bin_test\""),
            "bin name should match project name"
        );
    }

    #[test]
    fn cargo_gen_with_explicit_source_file() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "explicit_src".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("explicit_src");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // Pass main.rs explicitly
        let main_rs = project.join("src/main.rs");
        let (cargo_path, _) =
            crate::cargo_gen::generate(&manifest, &project, &[main_rs], false).unwrap();
        let content = fs::read_to_string(&cargo_path).unwrap();

        assert!(content.contains("[[bin]]"));
        assert!(content.contains("name = \"explicit_src\""));
    }

    // ── Battle-testing: pyproject_gen with dev deps ──────────────────

    #[test]
    fn pyproject_gen_no_dev_deps_by_default() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "pydev_test".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("pydev_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // Generate without dev deps
        let (pyproj_path, _) = crate::pyproject_gen::generate(&manifest, &project, false).unwrap();
        let content = fs::read_to_string(&pyproj_path).unwrap();

        assert!(
            !content.contains("[project.optional-dependencies]"),
            "new project should not have optional-dependencies section"
        );
    }

    // ══════════════════════════════════════════════════════════════════
    //  Phase 0: Battle tests — comprehensive edge-case coverage
    // ══════════════════════════════════════════════════════════════════

    // ── Name validation: whitespace & control characters ────────────

    #[test]
    fn name_with_newline_rejected() {
        let err = validate_project_name("my\nrobot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_tab_rejected() {
        let err = validate_project_name("my\trobot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_null_byte_rejected() {
        let err = validate_project_name("my\0robot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_trailing_space_rejected() {
        let err = validate_project_name("robot ").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    // ── Name validation: boundary values ────────────────────────────

    #[test]
    fn single_char_name_accepted() {
        // validate_project_name allows single char (manifest.validate() has a
        // separate 2-char minimum, but project creation doesn't enforce it)
        assert!(validate_project_name("a").is_ok());
        assert!(validate_project_name("Z").is_ok());
        assert!(validate_project_name("_").is_ok());
    }

    #[test]
    fn two_char_name_accepted() {
        assert!(validate_project_name("ab").is_ok());
        assert!(validate_project_name("_a").is_ok());
    }

    #[test]
    fn name_exactly_at_limit_64_accepted() {
        let name = format!("a{}", "b".repeat(63));
        assert_eq!(name.len(), 64);
        assert!(validate_project_name(&name).is_ok());
    }

    #[test]
    fn name_one_over_limit_65_rejected() {
        let name = format!("a{}", "b".repeat(64));
        assert_eq!(name.len(), 65);
        let err = validate_project_name(&name).unwrap_err();
        assert!(err.to_string().contains("too long"));
    }

    #[test]
    fn name_ending_with_hyphen_accepted() {
        // Allowed by validator rules (no rule against trailing hyphen)
        assert!(validate_project_name("robot-").is_ok());
    }

    #[test]
    fn name_ending_with_underscore_accepted() {
        assert!(validate_project_name("robot_").is_ok());
    }

    // ── Name validation: reserved words pass create (checked at manifest level) ──

    #[test]
    fn reserved_names_pass_validate_project_name() {
        // validate_project_name does NOT check reserved words;
        // that happens in manifest.validate() at a later stage
        for name in &["horus", "core", "test", "main", "config"] {
            assert!(
                validate_project_name(name).is_ok(),
                "'{}' should pass name validation (reserved check is in manifest.validate)",
                name
            );
        }
    }

    #[test]
    fn reserved_name_fails_manifest_validate() {
        let dir = tempfile::tempdir().unwrap();
        // "test" is a reserved name in manifest.validate()
        create_horus_toml(dir.path(), "test", "reserved check", "alice").unwrap();
        let manifest = HorusManifest::load_from(&dir.path().join(HORUS_TOML)).unwrap();
        let result = manifest.validate();
        assert!(
            result.is_err(),
            "reserved name 'test' should fail manifest validation"
        );
    }

    // ── Name validation: path-injection characters ──────────────────

    #[test]
    fn name_with_path_separator_rejected() {
        let err = validate_project_name("my/robot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_backslash_rejected() {
        let err = validate_project_name("my\\robot").unwrap_err();
        assert!(err.to_string().contains("invalid character"));
    }

    #[test]
    fn name_with_shell_metachar_rejected() {
        for name in &["ro$bot", "ro`bot", "ro;bot", "ro|bot", "ro&bot"] {
            let err = validate_project_name(name).unwrap_err();
            assert!(
                err.to_string().contains("invalid character"),
                "'{}' should be rejected",
                name
            );
        }
    }

    // ── Custom output path ──────────────────────────────────────────

    #[test]
    fn custom_output_path_creates_nested_dirs() {
        let dir = tempfile::tempdir().unwrap();
        let nested = dir.path().join("deep").join("nested").join("path");
        // nested doesn't exist yet — create_new_project should create all dirs
        create_new_project(
            "nested_bot".to_string(),
            Some(nested.clone()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = nested.join("nested_bot");
        assert!(project.join(HORUS_TOML).exists());
        assert!(project.join("src/main.rs").exists());
        assert!(project.join(".horus").is_dir());
    }

    #[test]
    fn path_none_creates_in_current_dir() {
        // With path=None, the project dir is just PathBuf::from(name)
        // We don't actually create it here (it would write to the real cwd),
        // but we can test the path logic in create_horus_directory etc.
        let dir = tempfile::tempdir().unwrap();
        let project = dir.path().join("local_bot");
        fs::create_dir_all(&project).unwrap();

        // Manually create each component to verify the pieces work
        create_horus_directory(&project).unwrap();
        create_gitignore(&project, "rust").unwrap();
        create_horus_toml(&project, "local_bot", "desc", "dev").unwrap();
        create_main_rs(&project, false).unwrap();

        assert!(project.join(HORUS_TOML).exists());
        assert!(project.join("src/main.rs").exists());
        assert!(project.join(".horus").is_dir());
        assert!(project.join(".gitignore").exists());
    }

    // ── use_macro flag with Python (should be silently ignored) ─────

    #[test]
    fn use_macro_true_with_python_produces_main_py() {
        let dir = tempfile::tempdir().unwrap();
        // use_macro=true should be ignored for Python projects
        create_new_project(
            "py_macro_test".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            true, // macro flag — should be ignored for python
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("py_macro_test");
        assert!(project.join("main.py").exists());
        assert!(!project.join("src/main.rs").exists());

        // main.py should be the standard Python template (no macros)
        let content = fs::read_to_string(project.join("main.py")).unwrap();
        assert!(content.contains("import horus"));
        assert!(content.contains("horus.run(node)"));
    }

    // ── Gitignore: exhaustive content checks ────────────────────────

    #[test]
    fn gitignore_rust_has_all_horus_dirs() {
        let dir = tempfile::tempdir().unwrap();
        create_gitignore(dir.path(), "rust").unwrap();
        let content = fs::read_to_string(dir.path().join(".gitignore")).unwrap();

        // All .horus/ subdirectories that should be ignored
        for pattern in &[
            ".horus/packages/",
            ".horus/bin/",
            ".horus/lib/",
            ".horus/include/",
            ".horus/cache/",
            ".horus/target/",
            ".horus/Cargo.toml",
            ".horus/Cargo.lock",
            "*.log",
        ] {
            assert!(
                content.contains(pattern),
                "gitignore missing pattern: '{}'",
                pattern
            );
        }
    }

    #[test]
    fn gitignore_python_has_all_python_patterns() {
        let dir = tempfile::tempdir().unwrap();
        create_gitignore(dir.path(), "python").unwrap();
        let content = fs::read_to_string(dir.path().join(".gitignore")).unwrap();

        for pattern in &[
            "__pycache__/",
            "*.py[cod]",
            "*$py.class",
            ".pytest_cache/",
            "*.egg-info/",
            "dist/",
            "build/",
        ] {
            assert!(
                content.contains(pattern),
                "python gitignore missing pattern: '{}'",
                pattern
            );
        }
    }

    #[test]
    fn gitignore_cpp_still_has_horus_base() {
        let dir = tempfile::tempdir().unwrap();
        create_gitignore(dir.path(), "cpp").unwrap();
        let content = fs::read_to_string(dir.path().join(".gitignore")).unwrap();

        // cpp has no extra patterns but should still have base .horus/ patterns
        assert!(content.contains(".horus/packages/"));
        assert!(content.contains("*.log"));
        // Should NOT have python or rust specific patterns
        assert!(!content.contains("__pycache__"));
        assert!(!content.contains("target/\nCargo.lock"));
    }

    // ── Main.py: detailed content verification ──────────────────────

    #[test]
    fn main_py_has_node_constructor_params() {
        let dir = tempfile::tempdir().unwrap();
        create_main_py(dir.path()).unwrap();
        let content = fs::read_to_string(dir.path().join("main.py")).unwrap();

        // Verify Node constructor includes all documented parameters
        assert!(content.contains("name="), "main.py missing name= param");
        assert!(content.contains("pubs="), "main.py missing pubs= param");
        assert!(content.contains("subs="), "main.py missing subs= param");
        assert!(content.contains("tick="), "main.py missing tick= param");
        assert!(content.contains("rate="), "main.py missing rate= param");
    }

    #[test]
    fn main_py_has_name_guard() {
        let dir = tempfile::tempdir().unwrap();
        create_main_py(dir.path()).unwrap();
        let content = fs::read_to_string(dir.path().join("main.py")).unwrap();
        assert!(
            content.contains("if __name__ == \"__main__\":"),
            "main.py should have __name__ guard"
        );
    }

    #[test]
    fn main_py_has_controller_function() {
        let dir = tempfile::tempdir().unwrap();
        create_main_py(dir.path()).unwrap();
        let content = fs::read_to_string(dir.path().join("main.py")).unwrap();
        assert!(
            content.contains("def controller(node)"),
            "main.py should have controller function"
        );
    }

    // ── Main.rs: detailed content verification ──────────────────────

    #[test]
    fn main_rs_non_macro_has_topic_metadata() {
        let dir = tempfile::tempdir().unwrap();
        create_main_rs(dir.path(), false).unwrap();
        let content = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();

        assert!(
            content.contains("TopicMetadata"),
            "non-macro main.rs should reference TopicMetadata"
        );
        assert!(
            content.contains("Topic::new("),
            "non-macro main.rs should create Topic"
        );
    }

    #[test]
    fn main_rs_macro_has_pub_section() {
        let dir = tempfile::tempdir().unwrap();
        create_main_rs(dir.path(), true).unwrap();
        let content = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();

        assert!(
            content.contains("pub {"),
            "macro main.rs should have pub section for topic declarations"
        );
        assert!(
            content.contains("cmd_vel:"),
            "macro main.rs should declare cmd_vel topic"
        );
        assert!(
            content.contains("tick {"),
            "macro main.rs should have tick block"
        );
    }

    #[test]
    fn main_rs_both_variants_send_cmd_vel() {
        let dir = tempfile::tempdir().unwrap();

        create_main_rs(dir.path(), false).unwrap();
        let plain = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(
            plain.contains("CmdVel::new("),
            "non-macro variant should send CmdVel"
        );

        create_main_rs(dir.path(), true).unwrap();
        let macro_ver = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(
            macro_ver.contains("CmdVel::new("),
            "macro variant should send CmdVel"
        );
    }

    #[test]
    fn main_rs_both_variants_use_scheduler_run() {
        let dir = tempfile::tempdir().unwrap();

        create_main_rs(dir.path(), false).unwrap();
        let plain = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(
            plain.contains("scheduler.run()"),
            "non-macro should call scheduler.run()"
        );

        create_main_rs(dir.path(), true).unwrap();
        let macro_ver = fs::read_to_string(dir.path().join("src/main.rs")).unwrap();
        assert!(
            macro_ver.contains("scheduler.run()"),
            "macro should call scheduler.run()"
        );
    }

    // ── horus.toml raw text: literal field checks ───────────────────

    #[test]
    fn horus_toml_raw_text_has_expected_fields() {
        let dir = tempfile::tempdir().unwrap();
        create_horus_toml(dir.path(), "raw-check", "A test project", "tester").unwrap();
        let raw = fs::read_to_string(dir.path().join(HORUS_TOML)).unwrap();

        assert!(
            raw.contains("name = \"raw-check\""),
            "raw toml should have exact name field"
        );
        assert!(
            raw.contains("version = \"0.1.0\""),
            "raw toml should have version field"
        );
        assert!(
            raw.contains("description = \"A test project\""),
            "raw toml should have description field"
        );
        assert!(
            raw.contains("license = \"Apache-2.0\""),
            "raw toml should have license field"
        );
        assert!(raw.contains("\"tester\""), "raw toml should contain author");
    }

    #[test]
    fn horus_toml_edition_field_behavior() {
        // Verify the edition field in generated manifests.
        // Even with skip_serializing_if, the toml crate may still write it.
        // What matters is that it round-trips correctly.
        let dir = tempfile::tempdir().unwrap();
        create_horus_toml(dir.path(), "edition-check", "desc", "dev").unwrap();

        let manifest = HorusManifest::load_from(&dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(
            manifest.package.edition, "1",
            "edition should be '1' (default or explicit)"
        );
    }

    // ── Multiple projects in same parent ────────────────────────────

    #[test]
    fn multiple_projects_in_same_parent_dir() {
        let dir = tempfile::tempdir().unwrap();

        create_new_project(
            "bot_alpha".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        create_new_project(
            "bot_beta".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        // Both should coexist independently
        let alpha = dir.path().join("bot_alpha");
        let beta = dir.path().join("bot_beta");

        assert!(alpha.join(HORUS_TOML).exists());
        assert!(beta.join(HORUS_TOML).exists());

        let m_alpha = HorusManifest::load_from(&alpha.join(HORUS_TOML)).unwrap();
        let m_beta = HorusManifest::load_from(&beta.join(HORUS_TOML)).unwrap();
        assert_eq!(m_alpha.package.name, "bot_alpha");
        assert_eq!(m_beta.package.name, "bot_beta");

        // Language-specific files are separate
        assert!(alpha.join("src/main.rs").exists());
        assert!(!alpha.join("main.py").exists());
        assert!(beta.join("main.py").exists());
        assert!(!beta.join("main.rs").exists());
    }

    // ── Overwrite: content is refreshed on re-creation ──────────────

    #[test]
    fn overwrite_refreshes_manifest_content() {
        let dir = tempfile::tempdir().unwrap();

        // Create first
        create_new_project(
            "overwrite_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("overwrite_bot");

        // Manually corrupt the manifest
        fs::write(project.join(HORUS_TOML), "garbage content").unwrap();

        // Re-create should overwrite with valid manifest
        create_new_project(
            "overwrite_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            true, // switch to macro this time
            false,
            false,
        )
        .unwrap();

        // Manifest should be valid again
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.package.name, "overwrite_bot");

        // main.rs should now be macro version
        let content = fs::read_to_string(project.join("src/main.rs")).unwrap();
        assert!(
            content.contains("node!"),
            "should have switched to macro version"
        );
    }

    // ── Overwrite: switching language replaces source file ───────────

    #[test]
    fn switching_language_on_recreate() {
        let dir = tempfile::tempdir().unwrap();

        // Create as Rust first
        create_new_project(
            "switch_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("switch_bot");
        assert!(project.join("src/main.rs").exists());

        // Re-create as Python
        create_new_project(
            "switch_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        // Python file should exist; Rust file is stale but still present
        // (create_new_project does not clean up old language files)
        assert!(project.join("main.py").exists());

        // Gitignore should now have python patterns
        let gitignore = fs::read_to_string(project.join(".gitignore")).unwrap();
        assert!(
            gitignore.contains("__pycache__/"),
            "gitignore should reflect python after switch"
        );
    }

    // ── Cargo.toml generation: workspace isolation ──────────────────

    #[test]
    fn cargo_gen_has_workspace_members_empty() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "ws_test".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("ws_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        let (cargo_path, _) = crate::cargo_gen::generate(&manifest, &project, &[], false).unwrap();
        let content = fs::read_to_string(&cargo_path).unwrap();

        // Workspace isolation: [workspace] with empty members
        assert!(
            content.contains("[workspace]"),
            "Cargo.toml must have [workspace] for isolation"
        );
    }

    // ── Cargo.toml generation: bin entry references correct path ────

    #[test]
    fn cargo_gen_bin_path_points_to_project_root_main_rs() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "path_test".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("path_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        let (cargo_path, _) = crate::cargo_gen::generate(&manifest, &project, &[], false).unwrap();
        let content = fs::read_to_string(&cargo_path).unwrap();

        // [[bin]] path should point back to main.rs in project root
        // (cargo_gen generates into .horus/, so path is relative to .horus/)
        assert!(
            content.contains("main.rs"),
            "bin path should reference main.rs"
        );
    }

    // ── pyproject.toml: PEP 621 compliance ──────────────────────────

    #[test]
    fn pyproject_gen_has_build_system_with_setuptools() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "pep621_test".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("pep621_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        let (pyproj_path, _) = crate::pyproject_gen::generate(&manifest, &project, false).unwrap();
        let content = fs::read_to_string(&pyproj_path).unwrap();

        assert!(
            content.contains("[build-system]"),
            "pyproject.toml must have [build-system]"
        );
        assert!(
            content.contains("[project]"),
            "pyproject.toml must have [project]"
        );
        assert!(
            content.contains("requires-python"),
            "pyproject.toml must declare requires-python"
        );
    }

    // ── pyproject.toml: name matches manifest ───────────────────────

    #[test]
    fn pyproject_gen_name_matches_horus_manifest() {
        let dir = tempfile::tempdir().unwrap();
        let name = "name_match_test";
        create_new_project(
            name.to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join(name);
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        let (pyproj_path, _) = crate::pyproject_gen::generate(&manifest, &project, false).unwrap();
        let content = fs::read_to_string(&pyproj_path).unwrap();

        let expected = format!("name = \"{}\"", name);
        assert!(
            content.contains(&expected),
            "pyproject.toml name should match manifest name"
        );
    }

    // ── Thread safety: concurrent project creation ──────────────────

    #[test]
    fn concurrent_project_creation_in_different_dirs() {
        use std::thread;

        let handles: Vec<_> = (0..4)
            .map(|i| {
                thread::spawn(move || {
                    let dir = tempfile::tempdir().unwrap();
                    let name = format!("concurrent_bot_{}", i);
                    create_new_project(
                        name.clone(),
                        Some(dir.path().to_path_buf()),
                        if i % 2 == 0 { "rust" } else { "python" }.to_string(),
                        false,
                        false,
                        false,
                    )
                    .unwrap();

                    let project = dir.path().join(&name);
                    let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
                    assert_eq!(manifest.package.name, name);
                    // Keep dir alive until assertions complete
                    drop(dir);
                })
            })
            .collect();

        for h in handles {
            h.join().expect("thread should not panic");
        }
    }

    // ── Manifest roundtrip: authors list preserved ──────────────────

    #[test]
    fn horus_toml_author_from_git_config() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "author_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("author_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // Should have exactly one author (from git config or $USER fallback)
        assert_eq!(manifest.package.authors.len(), 1);
        assert!(
            !manifest.package.authors[0].is_empty(),
            "author should not be empty"
        );
    }

    // ── Manifest: description is the default message ────────────────

    #[test]
    fn horus_toml_has_default_description() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "desc_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("desc_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert_eq!(
            manifest.package.description.as_deref(),
            Some("A HORUS robotics project")
        );
    }

    // ── Manifest: no package_type set by default ────────────────────

    #[test]
    fn horus_toml_no_package_type_by_default() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "type_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("type_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert!(
            manifest.package.package_type.is_none(),
            "new project should have no package-type set"
        );

        // Also verify it's absent from raw TOML
        let raw = fs::read_to_string(project.join(HORUS_TOML)).unwrap();
        assert!(
            !raw.contains("package-type"),
            "package-type should be omitted when None"
        );
    }

    // ── Manifest: no categories by default ──────────────────────────

    #[test]
    fn horus_toml_no_categories_by_default() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "cat_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("cat_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert!(manifest.package.categories.is_empty());

        let raw = fs::read_to_string(project.join(HORUS_TOML)).unwrap();
        assert!(
            !raw.contains("categories"),
            "categories should be omitted when empty"
        );
    }

    // ── Manifest: no repository by default ──────────────────────────

    #[test]
    fn horus_toml_no_repository_by_default() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "repo_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("repo_bot");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert!(manifest.package.repository.is_none());

        let raw = fs::read_to_string(project.join(HORUS_TOML)).unwrap();
        assert!(
            !raw.contains("repository"),
            "repository should be omitted when None"
        );
    }

    // ── .horus directory: is empty after creation ───────────────────

    #[test]
    fn horus_dir_is_empty_after_creation() {
        let dir = tempfile::tempdir().unwrap();
        create_horus_directory(dir.path()).unwrap();

        let horus_dir = dir.path().join(".horus");
        let entries: Vec<_> = fs::read_dir(&horus_dir).unwrap().collect();
        assert!(
            entries.is_empty(),
            ".horus/ should be empty right after creation (subdirs created by `horus run`)"
        );
    }

    // ── Project name with hyphens: verify name consistency ──────────

    #[test]
    fn hyphenated_name_consistent_across_artifacts() {
        let dir = tempfile::tempdir().unwrap();
        let name = "my-diff-drive";
        create_new_project(
            name.to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join(name);

        // horus.toml preserves hyphens
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.package.name, name);

        // Cargo.toml also uses the hyphenated name
        let (cargo_path, _) = crate::cargo_gen::generate(&manifest, &project, &[], false).unwrap();
        let cargo_content = fs::read_to_string(&cargo_path).unwrap();
        assert!(
            cargo_content.contains(&format!("name = \"{}\"", name)),
            "Cargo.toml should use exact hyphenated project name"
        );
    }

    // ── Project name with underscores: verify pyproject consistency ──

    #[test]
    fn underscored_name_consistent_in_pyproject() {
        let dir = tempfile::tempdir().unwrap();
        let name = "my_py_bot";
        create_new_project(
            name.to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join(name);
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.package.name, name);

        let (pyproj_path, _) = crate::pyproject_gen::generate(&manifest, &project, false).unwrap();
        let content = fs::read_to_string(&pyproj_path).unwrap();
        assert!(
            content.contains(&format!("name = \"{}\"", name)),
            "pyproject.toml should use exact underscored project name"
        );
    }

    // ── Cargo.toml generated in .horus/ not project root ────────────

    #[test]
    fn cargo_gen_output_lives_in_horus_dir() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "loc_test".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("loc_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        let (cargo_path, _) = crate::cargo_gen::generate(&manifest, &project, &[], false).unwrap();

        // Cargo.toml should be in .horus/, not project root
        assert!(
            cargo_path.starts_with(project.join(".horus")),
            "generated Cargo.toml should live under .horus/"
        );
        assert!(
            !project.join("Cargo.toml").exists(),
            "project root should not have Cargo.toml"
        );
    }

    // ── pyproject.toml generated in .horus/ not project root ────────

    #[test]
    fn pyproject_gen_output_lives_in_horus_dir() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "py_loc_test".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("py_loc_test");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();
        let (pyproj_path, _) = crate::pyproject_gen::generate(&manifest, &project, false).unwrap();

        assert!(
            pyproj_path.starts_with(project.join(".horus")),
            "generated pyproject.toml should live under .horus/"
        );
        assert!(
            !project.join("pyproject.toml").exists(),
            "project root should not have pyproject.toml"
        );
    }

    // ── Full Rust project: complete file tree verification ──────────

    #[test]
    fn rust_project_complete_file_tree() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "tree_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let p = dir.path().join("tree_bot");

        // Exhaustive check of every file/dir that should exist
        assert!(p.is_dir(), "project directory");
        assert!(p.join(HORUS_TOML).is_file(), "horus.toml");
        assert!(p.join("src/main.rs").is_file(), "src/main.rs");
        assert!(p.join(".horus").is_dir(), ".horus/");
        assert!(p.join(".gitignore").is_file(), ".gitignore");

        // And nothing else unexpected at the top level
        let top_entries: Vec<String> = fs::read_dir(&p)
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().to_string())
            .collect();

        for entry in &top_entries {
            assert!(
                ["horus.toml", "src", ".horus", ".gitignore"].contains(&entry.as_str()),
                "unexpected file in project root: '{}'",
                entry
            );
        }
    }

    // ── Full Python project: complete file tree verification ────────

    #[test]
    fn python_project_complete_file_tree() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "py_tree_bot".to_string(),
            Some(dir.path().to_path_buf()),
            "python".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let p = dir.path().join("py_tree_bot");

        assert!(p.is_dir(), "project directory");
        assert!(p.join(HORUS_TOML).is_file(), "horus.toml");
        assert!(p.join("main.py").is_file(), "main.py");
        assert!(p.join(".horus").is_dir(), ".horus/");
        assert!(p.join(".gitignore").is_file(), ".gitignore");

        let top_entries: Vec<String> = fs::read_dir(&p)
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().to_string())
            .collect();

        for entry in &top_entries {
            assert!(
                ["horus.toml", "main.py", ".horus", ".gitignore"].contains(&entry.as_str()),
                "unexpected file in python project root: '{}'",
                entry
            );
        }
    }

    // ── Manifest validate: generated project with 2+ char name passes ──

    #[test]
    fn generated_project_with_valid_name_passes_full_validation() {
        let dir = tempfile::tempdir().unwrap();
        create_new_project(
            "ab".to_string(), // minimum 2 chars for manifest.validate()
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("ab");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // Should pass full manifest validation (2 chars is minimum)
        let warnings = manifest
            .validate()
            .expect("2-char name should pass validation");
        let _ = warnings;
    }

    // ── Manifest validate: single-char name created but fails validation ──

    #[test]
    fn single_char_name_created_but_fails_manifest_validate() {
        let dir = tempfile::tempdir().unwrap();
        // "a" passes validate_project_name but fails manifest.validate() (< 2 chars)
        create_new_project(
            "a".to_string(),
            Some(dir.path().to_path_buf()),
            "rust".to_string(),
            false,
            false,
            false,
        )
        .unwrap();

        let project = dir.path().join("a");
        let manifest = HorusManifest::load_from(&project.join(HORUS_TOML)).unwrap();

        // manifest.validate() requires 2-64 chars
        let result = manifest.validate();
        assert!(
            result.is_err(),
            "single-char name should fail manifest validation (2-char minimum)"
        );
    }

    // ── Name validation: all-numeric after valid start ──────────────

    #[test]
    fn name_alpha_then_digits_accepted() {
        assert!(validate_project_name("robot123").is_ok());
        assert!(validate_project_name("_42").is_ok());
        assert!(validate_project_name("a0").is_ok());
    }

    // ── Name validation: all-hyphens after valid start ──────────────

    #[test]
    fn name_with_many_hyphens_accepted() {
        assert!(validate_project_name("a---b---c").is_ok());
    }

    // ── Name validation: mixed hyphens and underscores ──────────────

    #[test]
    fn name_mixed_hyphens_underscores_accepted() {
        assert!(validate_project_name("my-robot_v2").is_ok());
        assert!(validate_project_name("a_b-c_d-e").is_ok());
    }
}
