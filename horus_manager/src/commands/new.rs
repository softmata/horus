use crate::version;
use anyhow::{Context, Result};
use colored::*;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

pub fn create_new_project(
    name: String,
    path: Option<PathBuf>,
    language: String,
    use_macro: bool,
) -> Result<()> {
    // Check version compatibility before creating project
    version::check_and_prompt_update()?;

    println!(
        "{} Creating new HORUS project '{}'",
        "[*]".cyan(),
        name.green().bold()
    );

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

    // Create .horus/ directory structure
    create_horus_directory(&project_path)?;

    // Create .gitignore in project root
    create_gitignore(&project_path, &language)?;

    // Generate horus.yaml with dependencies
    create_horus_yaml(
        &project_path,
        &name,
        &description,
        &author,
        &language,
        use_macro,
    )?;

    // Generate main file based on language
    match language.as_str() {
        "rust" => {
            create_main_rs(&project_path, use_macro)?;
        }
        "python" => create_main_py(&project_path)?,
        _ => unreachable!(),
    }

    // Register workspace in ~/.horus/workspaces.json
    // This makes it visible in monitors (horus monitor / horus monitor -t)
    if let Ok(mut registry) = crate::workspace::WorkspaceRegistry::load() {
        if let Ok(canonical_path) = project_path.canonicalize() {
            if registry.add(name.clone(), canonical_path).is_ok() {
                println!("  {} Registered workspace in registry", "✓".green());
            }
        }
    }

    println!("\n{}", "✓ Project created successfully!".green().bold());
    println!("\nTo get started:");
    println!("  {} {}", "cd".cyan(), name);
    println!("  {} (auto-installs dependencies)", "horus run".cyan());

    Ok(())
}

fn prompt_language() -> Result<String> {
    println!("\n{} Select language:", "?".yellow().bold());
    println!("  {} Python", "1.".cyan());
    println!("  {} Rust", "2.".cyan());

    print!("{} [1-2] (default: 2): ", ">".cyan().bold());
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let input = input.trim();

    let choice = if input.is_empty() { "2" } else { input };

    let language = match choice {
        "1" => "python",
        "2" => "rust",
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
        "cpp" => {}
        _ => {}
    }

    fs::write(project_path.join(".gitignore"), gitignore_content)?;

    Ok(())
}

fn create_horus_yaml(
    project_path: &Path,
    name: &str,
    description: &str,
    author: &str,
    language: &str,
    use_macro: bool,
) -> Result<()> {
    // Determine dependencies based on language
    let dependencies = match language {
        "rust" => {
            if use_macro {
                r#"dependencies:
  - horus
  - horus_macros
  - horus_library  # Standard robotics messages (CmdVel, etc.)

  # For path dependencies (local development):
  # my_driver:
  #   path: "./drivers/my_driver"
"#
            } else {
                r#"dependencies:
  - horus
  - horus_library  # Standard robotics messages (CmdVel, etc.)

  # For path dependencies (local development):
  # my_driver:
  #   path: "./drivers/my_driver"
"#
            }
        }
        "python" => {
            r#"dependencies:
  - pip:horus-robotics
  # Add Python packages as needed

  # For path dependencies (local development):
  # my_module:
  #   path: "./my_module"
"#
        }
        _ => "",
    };

    let content = format!(
        r#"name: {}
version: 0.1.8
description: {}
author: {}
license: Apache-2.0
language: {}
horus_id: null  # Auto-generated on first dependency resolution

{}
# Optional: Ignore files, directories, and packages during horus run/check
# ignore:
#   files:
#     - "debug_*.py"
#     - "test_*.rs"
#     - "**/experiments/**"
#   directories:
#     - "old/"
#     - "experiments/"
#   packages:
#     - "ipython"
#     - "jupyter"
"#,
        name, description, author, language, dependencies
    );

    fs::write(project_path.join("horus.yaml"), content)?;

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

            self.cmd_vel.send(msg).ok();
        }
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Add the controller node with priority 0 (highest)
    scheduler.add(Controller::new()).order(0).done();

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
            cmd_vel: Topic::new("motors.cmd_vel")?.with_logging(),
        })
    }
}

impl Node for Controller {
    fn name(&self) -> &'static str {
        "controller"
    }

    fn tick(&mut self) {
        // Your control logic here
        let msg = CmdVel::new(1.0, 0.0);

        self.cmd_vel.send(msg).ok();
    }

    fn get_publishers(&self) -> Vec<TopicMetadata> {
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
    scheduler.add(Controller::new()?).order(0).done();

    // Run the scheduler (Ctrl+C to stop)
    scheduler.run()
}
"#
    };

    fs::write(project_path.join("main.rs"), content)?;
    Ok(())
}

fn create_main_py(project_path: &Path) -> Result<()> {
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
