//! Isolated Node Runner Binary
//!
//! This binary runs a single HORUS node in process isolation.
//! It communicates with the main scheduler via shared memory IPC.
//!
//! Usage:
//!   horus-isolated-runner --node-name <name> --factory <factory> --ipc-path <path>
//!
//! The runner:
//! 1. Opens the IPC shared memory region
//! 2. Creates the node using the factory name
//! 3. Enters the IPC command loop
//! 4. Executes tick/init/shutdown commands from the main scheduler
//! 5. Reports results back via IPC

use clap::Parser;
use std::path::PathBuf;

/// Isolated Node Runner for HORUS
#[derive(Parser, Debug)]
#[command(name = "horus-isolated-runner")]
#[command(about = "Run a HORUS node in process isolation")]
struct Args {
    /// Node name (must match the IPC region name)
    #[arg(long)]
    node_name: String,

    /// Factory name to create the node (currently just informational)
    #[arg(long)]
    factory: String,

    /// Path to the IPC shared memory file
    #[arg(long)]
    ipc_path: PathBuf,
}

fn main() {
    let args = Args::parse();

    println!(
        "[IsolatedRunner] Starting node '{}' (factory: {}, ipc: {:?})",
        args.node_name, args.factory, args.ipc_path
    );

    // In production, you would:
    // 1. Look up the factory by name in a registry
    // 2. Create the node instance
    // 3. Call run_isolated_node()
    //
    // For now, this binary serves as a template for how to implement
    // custom isolated runners for specific node types.
    //
    // Example implementation:
    //
    // ```rust
    // use horus_core::scheduling::executors::isolated::run_isolated_node;
    //
    // let node = match args.factory.as_str() {
    //     "CameraNode" => Box::new(CameraNode::new()) as Box<dyn Node>,
    //     "SensorNode" => Box::new(SensorNode::new()) as Box<dyn Node>,
    //     _ => {
    //         eprintln!("Unknown factory: {}", args.factory);
    //         std::process::exit(1);
    //     }
    // };
    //
    // if let Err(e) = run_isolated_node(node, &args.ipc_path) {
    //     eprintln!("[IsolatedRunner] Error: {}", e);
    //     std::process::exit(1);
    // }
    // ```

    // For testing, create a simple demo node
    use horus_core::core::node::{Node, NodeInfo};

    struct DemoIsolatedNode {
        name: &'static str,
        tick_count: u32,
    }

    impl Node for DemoIsolatedNode {
        fn name(&self) -> &'static str {
            self.name
        }

        fn tick(&mut self) {
            self.tick_count += 1;
            // Note: Logging moved to scheduler-internal metrics
        }
    }

    // Create a demo node for testing
    let node_name_static: &'static str = Box::leak(args.node_name.clone().into_boxed_str());
    let node = Box::new(DemoIsolatedNode {
        name: node_name_static,
        tick_count: 0,
    });

    // Run the isolated node
    use horus_core::scheduling::executors::isolated::run_isolated_node;

    if let Err(e) = run_isolated_node(node, &args.ipc_path) {
        eprintln!("[IsolatedRunner] Error: {}", e);
        std::process::exit(1);
    }

    println!("[IsolatedRunner] Node '{}' exited normally", args.node_name);
}
