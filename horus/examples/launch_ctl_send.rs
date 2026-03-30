/// Send a LaunchControlCommand to a launch session's control topic.
/// Usage:
///   ./launch_ctl_send <session> stop <name>
///   ./launch_ctl_send <session> restart <name>
///   ./launch_ctl_send <session> stop-all

use horus::prelude::*;
use serde::{Deserialize, Serialize};

/// Mirror of horus_manager::commands::launch::LaunchControlCommand.
/// Must stay in sync — same serde representation.
#[derive(Debug, Clone, Serialize, Deserialize)]
enum LaunchControlCommand {
    StopNode { name: String },
    RestartNode { name: String },
    StopAll,
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: {} <session> <stop <name>|restart <name>|stop-all>", args[0]);
        std::process::exit(1);
    }

    let session = &args[1];
    let action = &args[2];
    let topic_name = format!("horus.launch.ctl.{}", session);
    let topic = Topic::<LaunchControlCommand>::new(&topic_name)?;

    let cmd = match action.as_str() {
        "stop" => {
            let name = args.get(3).expect("stop requires a node name");
            LaunchControlCommand::StopNode { name: name.clone() }
        }
        "restart" => {
            let name = args.get(3).expect("restart requires a node name");
            LaunchControlCommand::RestartNode { name: name.clone() }
        }
        "stop-all" => LaunchControlCommand::StopAll,
        _ => {
            eprintln!("Unknown action: {}. Use stop/restart/stop-all", action);
            std::process::exit(1);
        }
    };

    println!("Sending {:?} to {}", cmd, topic_name);
    topic.send(cmd);
    std::thread::sleep(std::time::Duration::from_millis(100));
    println!("Sent.");
    Ok(())
}
