//! Command-line interface for sim3d binary and validation tools

pub mod validation;

// Re-export validation types for use by horus_manager
pub use validation::{
    format_batch_report, validate_file, BatchValidationReport, OutputFormat, ValidationType,
};

use bevy::prelude::Resource;
use clap::{Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

/// HORUS 3D Robotics Simulator
#[derive(Parser, Debug, Clone, Resource)]
#[command(name = "sim3d")]
#[command(about = "HORUS 3D Robotics Simulator", long_about = None)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Option<Command>,

    #[arg(short, long, value_enum, default_value_t = Mode::Visual)]
    pub mode: Mode,

    #[arg(short, long)]
    pub robot: Option<PathBuf>,

    #[arg(short, long)]
    pub world: Option<PathBuf>,

    #[arg(short, long)]
    pub config: Option<PathBuf>,

    #[arg(long, default_value_t = false)]
    pub no_gui: bool,

    #[arg(long, default_value_t = false)]
    pub hf_viz: bool,

    #[arg(long, default_value_t = 1.0)]
    pub speed: f32,
}

/// Subcommands for sim3d
#[derive(Subcommand, Debug, Clone)]
pub enum Command {
    /// Validate scene or URDF files
    Validate {
        /// Files to validate
        #[arg(required = true)]
        files: Vec<PathBuf>,

        /// Validation type (auto-detect if not specified)
        #[arg(short = 't', long, value_enum)]
        validation_type: Option<CliValidationType>,

        /// Output format
        #[arg(short, long, value_enum, default_value_t = CliOutputFormat::Text)]
        format: CliOutputFormat,

        /// Check mesh references exist
        #[arg(long, default_value_t = true)]
        check_meshes: bool,

        /// Verbose output
        #[arg(short, long, default_value_t = false)]
        verbose: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum CliValidationType {
    Scene,
    Urdf,
    Auto,
}

impl From<CliValidationType> for ValidationType {
    fn from(val: CliValidationType) -> Self {
        match val {
            CliValidationType::Scene => ValidationType::Scene,
            CliValidationType::Urdf => ValidationType::Urdf,
            CliValidationType::Auto => ValidationType::Auto,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum CliOutputFormat {
    Text,
    Json,
    Html,
}

impl From<CliOutputFormat> for OutputFormat {
    fn from(val: CliOutputFormat) -> Self {
        match val {
            CliOutputFormat::Text => OutputFormat::Text,
            CliOutputFormat::Json => OutputFormat::Json,
            CliOutputFormat::Html => OutputFormat::Html,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum, Resource)]
pub enum Mode {
    Visual,
    Headless,
}

impl Cli {
    pub fn parse() -> Self {
        Parser::parse()
    }

    /// Check if a subcommand was provided (not the default run mode)
    pub fn has_subcommand(&self) -> bool {
        self.command.is_some()
    }
}
