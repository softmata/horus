//! HORUS initialization command
//!
//! Handles workspace initialization

use anyhow::Result;
use colored::*;
use crate::cli_output;

/// Run the init command - initialize a HORUS workspace
pub fn run_init(workspace_name: Option<String>) -> Result<()> {
    cli_output::header("Initializing HORUS workspace");
    println!();

    // Register workspace using existing workspace module
    crate::workspace::register_current_workspace(workspace_name)?;

    println!();
    cli_output::success("Workspace initialized successfully!");
    println!();
    println!("Next steps:");
    println!(
        "  1. Create a new project: {}",
        "horus new my_robot".yellow()
    );
    println!(
        "  2. Install packages:     {}",
        "horus install <package>".yellow()
    );
    println!("  3. Start monitor:        {}", "horus monitor".yellow());
    println!();

    Ok(())
}
