//! Record command - manage, replay, diff, export, and inject recording sessions

use crate::cli_output;
use colored::*;
use horus_core::core::DurationExt;
use horus_core::error::HorusResult;
use horus_core::horus_internal;
use horus_core::scheduling::{diff_recordings, Recording, RecordingManager};
use std::path::PathBuf;

/// List recording sessions
pub fn run_list(long: bool, json: bool) -> HorusResult<()> {
    let manager = RecordingManager::new();
    let sessions = manager
        .list_sessions()
        .map_err(|e| horus_internal!("Failed to list recordings: {}", e))?;

    if json {
        let session_list: Vec<_> = sessions
            .iter()
            .map(|session| {
                let recordings = manager.session_recordings(session).unwrap_or_default();
                let total_size: u64 = recordings
                    .iter()
                    .filter_map(|p| std::fs::metadata(p).ok())
                    .map(|m| m.len())
                    .sum();
                serde_json::json!({
                    "name": session,
                    "files": recordings.len(),
                    "size_bytes": total_size,
                })
            })
            .collect();
        let output = serde_json::json!({ "sessions": session_list });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    if sessions.is_empty() {
        println!(
            "{} No recording sessions found.",
            cli_output::ICON_INFO.cyan()
        );
        println!("       Use 'horus run --record <session>' to create one.");
    } else {
        println!(
            "{} Found {} recording session(s):\n",
            cli_output::ICON_SUCCESS.green(),
            sessions.len()
        );

        for session in sessions {
            if long {
                // Get detailed info
                let recordings = manager.session_recordings(&session).unwrap_or_default();
                let total_size: u64 = recordings
                    .iter()
                    .filter_map(|p| std::fs::metadata(p).ok())
                    .map(|m| m.len())
                    .sum();

                println!(
                    "  {} {} ({} files, {:.1} MB)",
                    cli_output::ICON_INFO.green(),
                    session.yellow(),
                    recordings.len(),
                    total_size as f64 / 1_048_576.0
                );
            } else {
                println!("  {} {}", cli_output::ICON_INFO.green(), session.yellow());
            }
        }
    }
    Ok(())
}

/// Show info about a recording session
pub fn run_info(session: String, json: bool) -> HorusResult<()> {
    let manager = RecordingManager::new();
    let recordings = manager
        .session_recordings(&session)
        .map_err(|e| horus_internal!("Failed to get session info: {}", e))?;

    if json {
        let files: Vec<_> = recordings
            .iter()
            .map(|path| {
                let filename = path
                    .file_name()
                    .and_then(|n| n.to_str())
                    .unwrap_or("unknown");
                let size = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
                let (first_tick, last_tick) =
                    if let Ok(recording) = horus_core::scheduling::NodeRecording::load(path) {
                        (Some(recording.first_tick), Some(recording.last_tick))
                    } else {
                        (None, None)
                    };
                serde_json::json!({
                    "filename": filename,
                    "size_bytes": size,
                    "first_tick": first_tick,
                    "last_tick": last_tick,
                })
            })
            .collect();
        let found = !recordings.is_empty();
        let output = serde_json::json!({
            "session": session,
            "found": found,
            "files": files,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return if found {
            Ok(())
        } else {
            Err(horus_internal!("Session '{}' not found", session))
        };
    }

    if recordings.is_empty() {
        return Err(horus_internal!(
            "Session '{}' not found. Use 'horus record list' to see available sessions.",
            session
        ));
    }

    println!(
        "{} Session: {}\n",
        cli_output::ICON_SUCCESS.green(),
        session.yellow().bold()
    );

    for path in recordings {
        let filename = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown");
        let size = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);

        // Try to load and get tick count
        let tick_info = if let Ok(recording) = horus_core::scheduling::NodeRecording::load(&path) {
            format!("ticks {}-{}", recording.first_tick, recording.last_tick)
        } else {
            "scheduler recording".to_string()
        };

        println!(
            "  {} {} ({:.1} KB, {})",
            cli_output::ICON_INFO.cyan(),
            filename,
            size as f64 / 1024.0,
            tick_info
        );
    }
    Ok(())
}

/// Delete a recording session
pub fn run_delete(session: String, force: bool) -> HorusResult<()> {
    let manager = RecordingManager::new();

    // Check existence before prompting
    let sessions = manager.list_sessions().unwrap_or_default();
    if !sessions.iter().any(|s| s == &session) {
        return Err(horus_internal!("Session '{}' not found", session));
    }

    if !force {
        println!(
            "{} Delete session '{}'? (y/N)",
            cli_output::ICON_WARN.yellow(),
            session
        );
        let mut input = String::new();
        std::io::stdin().read_line(&mut input).ok();
        if !input.trim().eq_ignore_ascii_case("y") {
            println!("Cancelled.");
            return Ok(());
        }
    }

    manager
        .delete_session(&session)
        .map_err(|e| horus_internal!("Failed to delete session: {}", e))?;

    println!(
        "{} Deleted session '{}'",
        cli_output::ICON_SUCCESS.green(),
        session
    );
    Ok(())
}

/// Replay a recording session
pub fn run_replay(
    recording: String,
    start_tick: Option<u64>,
    stop_tick: Option<u64>,
    speed: f64,
    overrides: Vec<(String, String, String)>,
) -> HorusResult<()> {
    use horus_core::Scheduler;

    if speed <= 0.0 {
        return Err(horus_internal!(
            "--speed must be greater than 0 (got {})",
            speed
        ));
    }

    let manager = RecordingManager::new();

    // Resolve the recording path - could be a direct path or a session name
    let scheduler_path = if PathBuf::from(&recording).exists() {
        // Direct path to recording file
        PathBuf::from(&recording)
    } else {
        // Treat as session name - find the scheduler recording
        let recordings = manager
            .session_recordings(&recording)
            .map_err(|e| horus_internal!("Failed to get session '{}': {}", recording, e))?;

        if recordings.is_empty() {
            return Err(horus_internal!(
                "Session '{}' not found or has no recordings",
                recording
            ));
        }

        // Find the scheduler recording (file starting with "scheduler@")
        recordings
            .iter()
            .find(|p| {
                p.file_name()
                    .and_then(|n| n.to_str())
                    .map(|n| n.starts_with("scheduler@"))
                    .unwrap_or(false)
            })
            .cloned()
            .ok_or_else(|| {
                horus_internal!("No scheduler recording found in session '{}'", recording)
            })?
    };

    println!(
        "{} Loading recording from: {}",
        cli_output::ICON_INFO.cyan(),
        scheduler_path.display()
    );

    // Load the scheduler from recording
    let mut scheduler = Scheduler::replay_from(scheduler_path)?;

    // Apply start tick if specified
    if let Some(start) = start_tick {
        scheduler = scheduler.start_at_tick(start);
    }

    // Apply stop tick if specified
    if let Some(stop) = stop_tick {
        scheduler = scheduler.stop_at_tick(stop);
        println!(
            "{} Will stop at tick {}",
            cli_output::ICON_INFO.cyan(),
            stop
        );
    }

    // Apply speed multiplier
    if (speed - 1.0).abs() > f64::EPSILON {
        scheduler = scheduler.with_replay_speed(speed);
        println!(
            "{} Playback speed: {}x",
            cli_output::ICON_INFO.cyan(),
            speed
        );
    }

    // Apply overrides
    for (node, output, value_str) in &overrides {
        // Parse the value string into bytes
        // Support formats: hex (0x...), decimal numbers, or raw strings
        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
            // Hex format
            parse_hex_string(hex_str).unwrap_or_else(|_| value_str.as_bytes().to_vec())
        } else if let Ok(num) = value_str.parse::<i64>() {
            // Integer number (checked before f64 so "42" stays i64, not IEEE 754)
            num.to_le_bytes().to_vec()
        } else if let Ok(num) = value_str.parse::<f64>() {
            // Float number (only matches values with '.', 'e', 'inf', 'nan', etc.)
            num.to_le_bytes().to_vec()
        } else {
            // Raw string bytes
            value_str.as_bytes().to_vec()
        };
        scheduler = scheduler.with_override(node, output, value_bytes);
    }

    println!("{} Starting replay...\n", cli_output::ICON_INFO.green());

    // Run the replay
    scheduler.run()?;

    println!("\n{} Replay completed", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Parse a hex string into bytes
fn parse_hex_string(s: &str) -> Result<Vec<u8>, String> {
    let s = s.trim();
    if !s.len().is_multiple_of(2) {
        return Err("Hex string must have even length".to_string());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| format!("Invalid hex: {}", e)))
        .collect()
}

/// Diff two recording sessions
pub fn run_diff(session1: String, session2: String, limit: Option<usize>) -> HorusResult<()> {
    let manager = RecordingManager::new();

    println!(
        "{} Comparing '{}' vs '{}'...\n",
        cli_output::ICON_INFO.cyan(),
        session1,
        session2
    );

    // Get recordings from both sessions
    let recordings1 = manager
        .session_recordings(&session1)
        .map_err(|e| horus_internal!("Failed to load session '{}': {}", session1, e))?;

    let recordings2 = manager
        .session_recordings(&session2)
        .map_err(|e| horus_internal!("Failed to load session '{}': {}", session2, e))?;

    // Build a lookup map for session2 recordings by node base name
    let session2_by_name: std::collections::HashMap<&str, &std::path::PathBuf> = recordings2
        .iter()
        .filter_map(|path| {
            let name = path
                .file_stem()
                .and_then(|n| n.to_str())
                .unwrap_or("")
                .split('@')
                .next()
                .unwrap_or("");
            if !name.is_empty() && name != "scheduler" {
                Some((name, path))
            } else {
                None
            }
        })
        .collect();

    // Find matching nodes via HashMap lookup (O(n+m) instead of O(n*m))
    let mut total_diffs = 0;
    let max_diffs = limit.unwrap_or(100);

    for path1 in &recordings1 {
        let name1 = path1
            .file_stem()
            .and_then(|n| n.to_str())
            .unwrap_or("")
            .split('@')
            .next()
            .unwrap_or("");

        if name1.is_empty() || name1 == "scheduler" {
            continue;
        }

        if let Some(path2) = session2_by_name.get(name1) {
            // Load and compare
            if let (Ok(rec1), Ok(rec2)) = (
                horus_core::scheduling::NodeRecording::load(path1),
                horus_core::scheduling::NodeRecording::load(path2),
            ) {
                let diffs = diff_recordings(&rec1, &rec2);
                if !diffs.is_empty() {
                    println!(
                        "  {} Node '{}': {} differences",
                        cli_output::ICON_WARN.yellow(),
                        name1,
                        diffs.len()
                    );
                    for diff in diffs.iter().take(max_diffs.saturating_sub(total_diffs)) {
                        match diff {
                            horus_core::scheduling::RecordingDiff::OutputDifference {
                                tick,
                                topic,
                                ..
                            } => {
                                println!("    Tick {}: output '{}' differs", tick, topic);
                            }
                            horus_core::scheduling::RecordingDiff::MissingTick {
                                tick,
                                in_recording,
                            } => {
                                println!("    Tick {} missing in recording {}", tick, in_recording);
                            }
                            horus_core::scheduling::RecordingDiff::MissingOutput {
                                tick,
                                topic,
                                in_recording,
                            } => {
                                println!(
                                    "    Tick {}: output '{}' missing in recording {}",
                                    tick, topic, in_recording
                                );
                            }
                        }
                        total_diffs += 1;
                    }
                } else {
                    println!(
                        "  {} Node '{}': identical",
                        cli_output::ICON_SUCCESS.green(),
                        name1
                    );
                }
            }
        }
    }

    if total_diffs == 0 {
        println!(
            "\n{} No differences found!",
            cli_output::ICON_SUCCESS.green()
        );
    } else {
        println!(
            "\n{} Total: {} difference(s)",
            cli_output::ICON_WARN.yellow(),
            total_diffs
        );
    }
    Ok(())
}

/// Export a recording session to JSON or CSV
pub fn run_export(session: String, output: PathBuf, format: String) -> HorusResult<()> {
    let manager = RecordingManager::new();

    println!(
        "{} Exporting session '{}' to {:?} (format: {})",
        cli_output::ICON_INFO.cyan(),
        session,
        output,
        format
    );

    let recordings = manager
        .session_recordings(&session)
        .map_err(|e| horus_internal!("Failed to load session: {}", e))?;

    if format == "json" {
        use std::io::Write;
        let mut file = std::fs::File::create(&output)
            .map_err(|e| horus_internal!("Failed to create output file: {}", e))?;

        // Use serde_json for proper escaping of all string values
        let mut recording_entries = Vec::new();
        for path in &recordings {
            if let Ok(recording) = horus_core::scheduling::NodeRecording::load(path) {
                recording_entries.push(serde_json::json!({
                    "node_name": recording.node_name,
                    "node_id": recording.node_id,
                    "first_tick": recording.first_tick,
                    "last_tick": recording.last_tick,
                    "snapshot_count": recording.snapshots.len(),
                }));
            }
        }

        let output_json = serde_json::json!({
            "session": session,
            "recordings": recording_entries,
        });

        writeln!(
            file,
            "{}",
            serde_json::to_string_pretty(&output_json).unwrap_or_default()
        )?;

        println!(
            "{} Exported to {:?}",
            cli_output::ICON_SUCCESS.green(),
            output
        );
    } else if format == "csv" {
        use std::io::Write;
        let mut file = std::fs::File::create(&output)
            .map_err(|e| horus_internal!("Failed to create output file: {}", e))?;

        // Write CSV header
        writeln!(
            file,
            "node_name,node_id,tick,timestamp_us,input_count,output_count,outputs"
        )?;

        // Process each recording
        for path in &recordings {
            if let Ok(recording) = horus_core::scheduling::NodeRecording::load(path) {
                // Skip scheduler recordings (they have different structure)
                if recording.node_name.starts_with("scheduler") {
                    continue;
                }

                for snapshot in &recording.snapshots {
                    // Serialize outputs as hex string for CSV
                    let outputs_str: Vec<String> = snapshot
                        .outputs
                        .iter()
                        .map(|(k, v): (&String, &Vec<u8>)| {
                            let hex: String = v.iter().map(|b| format!("{:02x}", b)).collect();
                            format!("{}={}", k, hex)
                        })
                        .collect();

                    // Quote all string fields and escape inner quotes for CSV safety
                    let outputs_joined = outputs_str.join(";").replace('"', "\"\"");
                    writeln!(
                        file,
                        "\"{}\",\"{}\",{},{},{},{},\"{}\"",
                        recording.node_name.replace('"', "\"\""),
                        recording.node_id.replace('"', "\"\""),
                        snapshot.tick,
                        snapshot.timestamp_us,
                        snapshot.inputs.len(),
                        snapshot.outputs.len(),
                        outputs_joined
                    )?;
                }
            }
        }

        println!(
            "{} Exported to {:?} (CSV format)",
            cli_output::ICON_SUCCESS.green(),
            output
        );
    } else {
        println!(
            "{} Format '{}' not supported. Use 'json' or 'csv'.",
            cli_output::ICON_WARN.yellow(),
            format
        );
    }
    Ok(())
}

/// RAII guard that kills a child process on drop, preventing zombie/orphan
/// processes if the parent thread panics or returns early.
struct ChildGuard {
    child: std::process::Child,
    name: String,
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        match self.child.try_wait() {
            Ok(Some(_)) => {} // Already exited
            _ => {
                log::info!(
                    "ChildGuard: killing '{}' (PID {})",
                    self.name,
                    self.child.id()
                );
                let _ = self.child.kill();
                let _ = self.child.wait(); // Reap to prevent zombie
            }
        }
    }
}

/// CLI arguments for `horus record inject`.
pub struct InjectArgs {
    pub session: String,
    pub nodes: Vec<String>,
    pub all: bool,
    pub script: Option<PathBuf>,
    pub start_tick: Option<u64>,
    pub stop_tick: Option<u64>,
    pub speed: f64,
    pub loop_playback: bool,
}

/// Inject recorded nodes into a live/scripted run
pub fn run_inject(args: InjectArgs) -> HorusResult<()> {
    let InjectArgs {
        session,
        nodes,
        all,
        script,
        start_tick,
        stop_tick,
        speed,
        loop_playback,
    } = args;
    use horus_core::Scheduler;

    if speed <= 0.0 {
        return Err(horus_internal!(
            "--speed must be greater than 0 (got {})",
            speed
        ));
    }

    let manager = RecordingManager::new();

    println!(
        "{} Injecting recorded nodes from session '{}'",
        cli_output::ICON_INFO.cyan(),
        session
    );

    // Get all recordings from the session
    let recordings = manager
        .session_recordings(&session)
        .map_err(|e| horus_internal!("Failed to load session '{}': {}", session, e))?;

    if recordings.is_empty() {
        return Err(horus_internal!(
            "Session '{}' not found or has no recordings",
            session
        ));
    }

    // Create a new scheduler for hybrid execution
    let mut scheduler = Scheduler::new();
    scheduler.scheduler_name = format!("Inject({})", session);

    // Filter and add replay nodes
    let mut injected_count = 0;
    for path in &recordings {
        let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("");

        // Skip scheduler recordings
        if filename.starts_with("scheduler@") {
            continue;
        }

        // Extract node name from filename (format: node_name@id.horus)
        let node_name = filename.split('@').next().unwrap_or("");

        // Check if we should inject this node
        let should_inject = all || nodes.iter().any(|n| n == node_name);

        if should_inject {
            match scheduler.add_replay(path.clone(), 0) {
                Ok(_) => {
                    println!(
                        "  {} Injected '{}'",
                        cli_output::ICON_SUCCESS.green(),
                        node_name
                    );
                    injected_count += 1;
                }
                Err(e) => {
                    log::error!("Failed to inject '{}': {}", node_name, e);
                    eprintln!(
                        "  {} Failed to inject '{}': {}",
                        cli_output::ICON_ERROR.red(),
                        node_name,
                        e
                    );
                }
            }
        }
    }

    if injected_count == 0 {
        return Err(horus_internal!(
            "No nodes were injected. Check node names or use --all"
        ));
    }

    // Apply start tick if specified
    if let Some(start) = start_tick {
        scheduler = scheduler.start_at_tick(start);
    }

    // Apply stop tick if specified
    if let Some(stop) = stop_tick {
        scheduler = scheduler.stop_at_tick(stop);
    }

    // Apply speed multiplier
    if (speed - 1.0).abs() > f64::EPSILON {
        scheduler = scheduler.with_replay_speed(speed);
        println!(
            "{} Playback speed: {}x",
            cli_output::ICON_INFO.cyan(),
            speed
        );
    }

    // If a script is provided, launch it as a child process alongside the replay
    let mut script_child = None;
    if let Some(script_path) = &script {
        if !script_path.exists() {
            return Err(horus_internal!(
                "Script file not found: {}",
                script_path.display()
            ));
        }

        println!(
            "\n{} Launching script alongside replay: {}",
            cli_output::ICON_INFO.cyan(),
            script_path.display()
        );

        // Spawn `horus run <script>` as a child process
        // This handles compilation, dependency resolution, and execution
        let current_exe = std::env::current_exe()
            .map_err(|e| horus_internal!("Failed to get current executable path: {}", e))?;

        let inject_nodes_val = if all {
            "*".to_string()
        } else {
            nodes.join(",")
        };

        let child = std::process::Command::new(&current_exe)
            .arg("run")
            .arg(script_path)
            .env("HORUS_INJECT_SESSION", &session)
            .env("HORUS_INJECT_NODES", &inject_nodes_val)
            .spawn()
            .map_err(|e| {
                horus_internal!("Failed to launch script '{}': {}", script_path.display(), e)
            })?;

        println!(
            "{} Script started (PID: {})",
            cli_output::ICON_SUCCESS.green(),
            child.id()
        );

        script_child = Some(ChildGuard {
            name: script_path.display().to_string(),
            child,
        });

        // Brief delay to let the script process start and subscribe to topics
        // before the scheduler begins publishing replay data
        std::thread::sleep(1_u64.secs());

        println!(
            "\n{} Running {} injected node(s) + script...\n",
            cli_output::ICON_INFO.green(),
            injected_count
        );
    } else {
        println!(
            "\n{} Running {} injected node(s)...\n",
            cli_output::ICON_INFO.green(),
            injected_count
        );
    }

    // Handle loop playback
    if loop_playback {
        println!(
            "{} Loop mode: Recording will restart when finished",
            cli_output::ICON_INFO.cyan()
        );

        // Run in a loop until interrupted
        loop {
            // Reset tick counter for new iteration
            scheduler = scheduler.start_at_tick(start_tick.unwrap_or(0));

            match scheduler.run() {
                Ok(()) => {
                    println!(
                        "\n{} Recording finished, restarting...\n",
                        cli_output::ICON_INFO.cyan()
                    );

                    // Recreate scheduler for next iteration
                    scheduler = Scheduler::new();
                    scheduler.scheduler_name = format!("Inject({})", session);

                    // Re-inject nodes
                    for path in &recordings {
                        let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
                        if filename.starts_with("scheduler@") {
                            continue;
                        }
                        let node_name = filename.split('@').next().unwrap_or("");
                        let should_inject = all || nodes.iter().any(|n| n == node_name);
                        if should_inject {
                            if let Err(e) = scheduler.add_replay(path.clone(), 0) {
                                log::warn!("Failed to re-inject '{}' in loop: {}", node_name, e);
                            }
                        }
                    }

                    // Re-apply settings
                    if let Some(start) = start_tick {
                        scheduler = scheduler.start_at_tick(start);
                    }
                    if let Some(stop) = stop_tick {
                        scheduler = scheduler.stop_at_tick(stop);
                    }
                    if (speed - 1.0).abs() > f64::EPSILON {
                        scheduler = scheduler.with_replay_speed(speed);
                    }
                }
                Err(e) => {
                    // If interrupted (Ctrl+C), exit loop
                    println!(
                        "\n{} Loop interrupted: {}",
                        cli_output::ICON_WARN.yellow(),
                        e
                    );
                    break;
                }
            }
        }
    } else {
        // Single run
        scheduler.run()?;
        println!(
            "\n{} Injection replay completed",
            cli_output::ICON_SUCCESS.green()
        );
    }

    // Clean up script child process if it was spawned.
    // The ChildGuard will kill on drop automatically, but we check status for reporting.
    if let Some(mut guard) = script_child {
        match guard.child.try_wait() {
            Ok(Some(status)) => {
                if status.success() {
                    println!(
                        "{} Script exited successfully",
                        cli_output::ICON_SUCCESS.green()
                    );
                } else {
                    eprintln!(
                        "{} Script exited with code: {}",
                        cli_output::ICON_WARN.yellow(),
                        status.code().unwrap_or(-1)
                    );
                }
            }
            Ok(None) => {
                // Still running - ChildGuard drop will kill it
                println!(
                    "{} Stopping script process...",
                    cli_output::ICON_INFO.cyan()
                );
            }
            Err(e) => {
                log::warn!("Failed to check script status: {}", e);
            }
        }
        // ChildGuard drops here, killing if still running
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_hex_string_valid_bytes() {
        assert_eq!(
            parse_hex_string("deadbeef").unwrap(),
            vec![0xde, 0xad, 0xbe, 0xef]
        );
    }

    #[test]
    fn parse_hex_string_empty() {
        assert_eq!(parse_hex_string("").unwrap(), Vec::<u8>::new());
    }

    #[test]
    fn parse_hex_string_single_byte() {
        assert_eq!(parse_hex_string("ff").unwrap(), vec![0xff]);
    }

    #[test]
    fn parse_hex_string_odd_length_errors() {
        let err = parse_hex_string("abc").unwrap_err();
        assert!(
            err.contains("even length"),
            "should mention even length requirement: {}",
            err
        );
    }

    #[test]
    fn parse_hex_string_invalid_chars_errors() {
        let err = parse_hex_string("zzzz").unwrap_err();
        assert!(
            err.contains("Invalid hex"),
            "should mention invalid hex: {}",
            err
        );
    }

    #[test]
    fn parse_hex_string_trims_whitespace() {
        assert_eq!(parse_hex_string("  0a0b  ").unwrap(), vec![0x0a, 0x0b]);
    }

    #[test]
    fn parse_hex_string_uppercase() {
        assert_eq!(parse_hex_string("AABB").unwrap(), vec![0xaa, 0xbb]);
    }

    // ── Battle tests: parse_hex_string edge cases ────────────────────────

    #[test]
    fn parse_hex_string_mixed_case() {
        assert_eq!(parse_hex_string("aAbB").unwrap(), vec![0xaa, 0xbb]);
    }

    #[test]
    fn parse_hex_string_all_zeros() {
        assert_eq!(parse_hex_string("0000").unwrap(), vec![0x00, 0x00]);
    }

    #[test]
    fn parse_hex_string_all_ff() {
        assert_eq!(parse_hex_string("ffff").unwrap(), vec![0xff, 0xff]);
    }

    #[test]
    fn parse_hex_string_long_input() {
        let hex = "00".repeat(256);
        let result = parse_hex_string(&hex).unwrap();
        assert_eq!(result.len(), 256);
        assert!(result.iter().all(|&b| b == 0));
    }

    #[test]
    fn parse_hex_string_with_internal_whitespace() {
        // Internal whitespace is NOT valid hex
        let result = parse_hex_string("aa bb");
        assert!(result.is_err(), "internal spaces should fail");
    }

    #[test]
    fn parse_hex_string_with_0x_prefix() {
        // The function does NOT strip 0x prefix; caller does
        let result = parse_hex_string("0xff");
        // "0xff" is 4 chars, even length, but "0x" is not valid hex
        assert!(
            result.is_err(),
            "0x prefix should fail as invalid hex chars"
        );
    }

    #[test]
    fn parse_hex_string_unicode_chars() {
        let result = parse_hex_string("\u{00E9}\u{00E9}");
        assert!(result.is_err(), "unicode chars should fail");
    }

    // ── Battle tests: run_replay error paths ─────────────────────────────

    #[test]
    fn run_replay_zero_speed_returns_error() {
        let result = run_replay("nonexistent".to_string(), None, None, 0.0, vec![]);
        assert!(result.is_err(), "speed=0 should fail");
        let err = result.unwrap_err().to_string();
        assert!(err.contains("greater than 0"), "error: {}", err);
    }

    #[test]
    fn run_replay_negative_speed_returns_error() {
        let result = run_replay("nonexistent".to_string(), None, None, -1.0, vec![]);
        assert!(result.is_err(), "negative speed should fail");
        let err = result.unwrap_err().to_string();
        assert!(err.contains("greater than 0"), "error: {}", err);
    }

    #[test]
    fn run_replay_nonexistent_session_returns_error() {
        let result = run_replay(
            "session_that_never_existed_abc123".to_string(),
            None,
            None,
            1.0,
            vec![],
        );
        assert!(result.is_err(), "nonexistent session should fail");
    }

    #[test]
    fn run_replay_with_ticks_and_zero_speed_still_fails_on_speed() {
        let result = run_replay("x".to_string(), Some(0), Some(100), 0.0, vec![]);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("greater than 0"),
            "speed check should come first: {}",
            err
        );
    }

    // ── Battle tests: run_inject error paths ─────────────────────────────

    #[test]
    fn run_inject_zero_speed_returns_error() {
        let args = InjectArgs {
            session: "nonexistent".to_string(),
            nodes: vec![],
            all: true,
            script: None,
            start_tick: None,
            stop_tick: None,
            speed: 0.0,
            loop_playback: false,
        };
        let result = run_inject(args);
        assert!(result.is_err(), "speed=0 should fail");
        let err = result.unwrap_err().to_string();
        assert!(err.contains("greater than 0"), "error: {}", err);
    }

    #[test]
    fn run_inject_negative_speed_returns_error() {
        let args = InjectArgs {
            session: "test".to_string(),
            nodes: vec![],
            all: true,
            script: None,
            start_tick: None,
            stop_tick: None,
            speed: -5.0,
            loop_playback: false,
        };
        let result = run_inject(args);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("greater than 0"), "error: {}", err);
    }

    #[test]
    fn run_inject_nonexistent_session_returns_error() {
        let args = InjectArgs {
            session: "ghost_session_zzz_123".to_string(),
            nodes: vec![],
            all: true,
            script: None,
            start_tick: None,
            stop_tick: None,
            speed: 1.0,
            loop_playback: false,
        };
        let result = run_inject(args);
        assert!(result.is_err(), "nonexistent session should fail");
    }

    #[test]
    fn run_inject_nonexistent_script_after_speed_check() {
        // Speed is valid (1.0), session is nonexistent, so it fails on session first
        let args = InjectArgs {
            session: "nonexistent_session_xyz".to_string(),
            nodes: vec!["node_a".to_string()],
            all: false,
            script: Some(PathBuf::from("/tmp/no_such_script.rs")),
            start_tick: None,
            stop_tick: None,
            speed: 1.0,
            loop_playback: false,
        };
        let result = run_inject(args);
        assert!(result.is_err());
    }

    // ── Battle tests: run_list ────────────────────────────────────────────

    #[test]
    fn run_list_short_succeeds() {
        let result = run_list(false, false);
        assert!(
            result.is_ok(),
            "listing sessions should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn run_list_long_succeeds() {
        let result = run_list(true, false);
        assert!(
            result.is_ok(),
            "long listing should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn run_list_json_succeeds() {
        let result = run_list(false, true);
        assert!(
            result.is_ok(),
            "json listing should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn run_list_long_json_succeeds() {
        let result = run_list(true, true);
        assert!(
            result.is_ok(),
            "long+json listing should succeed: {:?}",
            result.err()
        );
    }

    // ── Battle tests: run_info ────────────────────────────────────────────

    #[test]
    fn run_info_nonexistent_session_returns_error() {
        let result = run_info("nonexistent_session_abc123".to_string(), false);
        assert!(result.is_err(), "info on nonexistent session should fail");
    }

    #[test]
    fn run_info_empty_session_returns_error() {
        let result = run_info("".to_string(), false);
        assert!(result.is_err(), "info on empty session should fail");
    }

    #[test]
    fn run_info_unicode_session_returns_error() {
        let result = run_info("\u{1F916}_session".to_string(), false);
        assert!(result.is_err(), "info on unicode session should fail");
    }

    #[test]
    fn run_info_json_nonexistent_returns_error() {
        let result = run_info("nonexistent_json_session".to_string(), true);
        assert!(
            result.is_err(),
            "json info on nonexistent session should fail"
        );
    }

    #[test]
    fn run_info_very_long_session_name() {
        let long_name = "s".repeat(4096);
        let result = run_info(long_name, false);
        assert!(result.is_err(), "very long session name should fail");
    }

    // ── Battle tests: run_delete ──────────────────────────────────────────

    #[test]
    fn run_delete_nonexistent_session_returns_error() {
        let result = run_delete("nonexistent_delete_target_xyz".to_string(), true);
        assert!(result.is_err(), "deleting nonexistent session should fail");
        let err = result.unwrap_err().to_string();
        assert!(err.contains("not found"), "error: {}", err);
    }

    #[test]
    fn run_delete_empty_name_returns_error() {
        let result = run_delete("".to_string(), true);
        assert!(result.is_err(), "deleting empty session should fail");
    }

    // ── Battle tests: run_export ──────────────────────────────────────────

    #[test]
    fn run_export_nonexistent_session_produces_empty_output() {
        // session_recordings returns empty for nonexistent sessions (no error),
        // so export writes an empty JSON file
        let dir = tempfile::tempdir().unwrap();
        let output = dir.path().join("export_test.json");
        let result = run_export(
            "ghost_session_xyz".to_string(),
            output.clone(),
            "json".to_string(),
        );
        // Either Ok (empty export) or Err is acceptable
        if result.is_ok() {
            // Verify the output file was created (even if empty content)
            assert!(output.exists(), "output file should be created");
        }
    }

    #[test]
    fn run_export_unsupported_format_warns() {
        // Unsupported format prints a warning and returns Ok (does not error)
        let dir = tempfile::tempdir().unwrap();
        let output = dir.path().join("export_test.xyz");
        let result = run_export("ghost_session_xyz".to_string(), output, "xyz".to_string());
        // The function loads recordings (empty), then hits the else branch
        // which prints a warning but returns Ok
        assert!(
            result.is_ok(),
            "unsupported format should warn, not error: {:?}",
            result.err()
        );
    }

    #[test]
    fn run_export_csv_format_nonexistent_session() {
        let dir = tempfile::tempdir().unwrap();
        let output = dir.path().join("export_test.csv");
        let result = run_export(
            "ghost_csv_session".to_string(),
            output.clone(),
            "csv".to_string(),
        );
        if result.is_ok() {
            assert!(output.exists(), "CSV output file should be created");
        }
    }

    #[test]
    fn run_export_to_invalid_path() {
        let result = run_export(
            "ghost_session_abc".to_string(),
            PathBuf::from("/nonexistent_dir_xyz/export.json"),
            "json".to_string(),
        );
        // Should fail because the output directory doesn't exist
        assert!(result.is_err(), "exporting to invalid path should fail");
    }

    // ── Battle tests: run_diff ────────────────────────────────────────────

    #[test]
    fn run_diff_nonexistent_sessions_returns_ok_no_diffs() {
        // session_recordings returns empty for nonexistent sessions,
        // so diff finds zero recordings to compare and reports "no diffs"
        let result = run_diff(
            "session_alpha_zzz".to_string(),
            "session_beta_zzz".to_string(),
            None,
        );
        assert!(
            result.is_ok(),
            "diff with empty sessions should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn run_diff_empty_session_names_returns_ok() {
        // Empty session names -> empty recording lists -> no diffs -> Ok
        let result = run_diff("".to_string(), "".to_string(), None);
        assert!(
            result.is_ok(),
            "diff with empty names should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn run_diff_same_session_returns_ok() {
        // Same session name -> same (empty) recordings -> no diffs
        let result = run_diff("same_name".to_string(), "same_name".to_string(), None);
        assert!(result.is_ok());
    }

    #[test]
    fn run_diff_with_limit_zero() {
        let result = run_diff("alpha".to_string(), "beta".to_string(), Some(0));
        assert!(
            result.is_ok(),
            "diff with limit=0 should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn run_diff_with_limit_one() {
        let result = run_diff("alpha".to_string(), "beta".to_string(), Some(1));
        assert!(
            result.is_ok(),
            "diff with limit=1 should succeed: {:?}",
            result.err()
        );
    }

    // ── Battle tests: value parsing in run_replay overrides ──────────────

    #[test]
    fn override_value_hex_format() {
        // Simulate the hex parsing branch in run_replay
        let value_str = "0xdeadbeef";
        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
            parse_hex_string(hex_str).unwrap_or_else(|_| value_str.as_bytes().to_vec())
        } else {
            value_str.as_bytes().to_vec()
        };
        assert_eq!(value_bytes, vec![0xde, 0xad, 0xbe, 0xef]);
    }

    #[test]
    fn override_value_integer_format() {
        let value_str = "42";
        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
            parse_hex_string(hex_str).unwrap_or_else(|_| value_str.as_bytes().to_vec())
        } else if let Ok(num) = value_str.parse::<i64>() {
            num.to_le_bytes().to_vec()
        } else if let Ok(num) = value_str.parse::<f64>() {
            num.to_le_bytes().to_vec()
        } else {
            value_str.as_bytes().to_vec()
        };
        assert_eq!(value_bytes, 42i64.to_le_bytes().to_vec());
    }

    #[test]
    fn override_value_float_format() {
        let value_str = "3.14";
        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
            parse_hex_string(hex_str).unwrap_or_else(|_| value_str.as_bytes().to_vec())
        } else if let Ok(num) = value_str.parse::<i64>() {
            num.to_le_bytes().to_vec()
        } else if let Ok(num) = value_str.parse::<f64>() {
            num.to_le_bytes().to_vec()
        } else {
            value_str.as_bytes().to_vec()
        };
        assert_eq!(value_bytes, 3.14f64.to_le_bytes().to_vec());
    }

    #[test]
    fn override_value_string_fallback() {
        let value_str = "hello_world";
        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
            parse_hex_string(hex_str).unwrap_or_else(|_| value_str.as_bytes().to_vec())
        } else if let Ok(num) = value_str.parse::<i64>() {
            num.to_le_bytes().to_vec()
        } else if let Ok(num) = value_str.parse::<f64>() {
            num.to_le_bytes().to_vec()
        } else {
            value_str.as_bytes().to_vec()
        };
        assert_eq!(value_bytes, b"hello_world".to_vec());
    }

    #[test]
    fn override_value_negative_integer() {
        let value_str = "-100";
        let value_bytes = if let Ok(num) = value_str.parse::<i64>() {
            num.to_le_bytes().to_vec()
        } else {
            value_str.as_bytes().to_vec()
        };
        assert_eq!(value_bytes, (-100i64).to_le_bytes().to_vec());
    }

    #[test]
    fn override_value_hex_invalid_fallback() {
        // 0xZZZZ is not valid hex, should fall back to raw bytes
        let value_str = "0xZZZZ";
        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
            parse_hex_string(hex_str).unwrap_or_else(|_| value_str.as_bytes().to_vec())
        } else {
            value_str.as_bytes().to_vec()
        };
        // Falls back to raw bytes of "0xZZZZ"
        assert_eq!(value_bytes, b"0xZZZZ".to_vec());
    }

    // ── Battle tests: InjectArgs struct ───────────────────────────────────

    #[test]
    fn inject_args_default_construction() {
        let args = InjectArgs {
            session: "test".to_string(),
            nodes: vec![],
            all: false,
            script: None,
            start_tick: None,
            stop_tick: None,
            speed: 1.0,
            loop_playback: false,
        };
        assert_eq!(args.session, "test");
        assert!(args.nodes.is_empty());
        assert!(!args.all);
        assert!(args.script.is_none());
        assert!(args.start_tick.is_none());
        assert!(args.stop_tick.is_none());
        assert!((args.speed - 1.0).abs() < f64::EPSILON);
        assert!(!args.loop_playback);
    }

    #[test]
    fn inject_args_with_all_options() {
        let args = InjectArgs {
            session: "my_session".to_string(),
            nodes: vec!["motor".to_string(), "sensor".to_string()],
            all: true,
            script: Some(PathBuf::from("/tmp/script.rs")),
            start_tick: Some(100),
            stop_tick: Some(500),
            speed: 2.5,
            loop_playback: true,
        };
        assert_eq!(args.nodes.len(), 2);
        assert!(args.all);
        assert!(args.script.is_some());
        assert_eq!(args.start_tick, Some(100));
        assert_eq!(args.stop_tick, Some(500));
        assert!((args.speed - 2.5).abs() < f64::EPSILON);
        assert!(args.loop_playback);
    }

    // ── Battle tests: RecordingManager ────────────────────────────────────

    #[test]
    fn recording_manager_new_succeeds() {
        let manager = RecordingManager::new();
        // Just verify it can be created without panicking
        let _ = manager;
    }

    #[test]
    fn recording_manager_list_sessions_succeeds() {
        let manager = RecordingManager::new();
        let result = manager.list_sessions();
        // Should succeed (possibly empty)
        assert!(
            result.is_ok(),
            "list_sessions should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn recording_manager_session_recordings_nonexistent() {
        let manager = RecordingManager::new();
        let result = manager.session_recordings("this_session_does_not_exist_xyz");
        // Should return Ok(empty) or Err
        match result {
            Ok(recordings) => assert!(
                recordings.is_empty(),
                "nonexistent session should have no recordings"
            ),
            Err(_) => {} // Also acceptable
        }
    }
}
