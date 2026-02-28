//! Record command - manage, replay, diff, export, and inject recording sessions

use crate::cli_output;
use colored::*;
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
        let session_list: Vec<_> = sessions.iter().map(|session| {
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
        }).collect();
        let output = serde_json::json!({ "sessions": session_list });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
        return Ok(());
    }

    if sessions.is_empty() {
        println!("{} No recording sessions found.", cli_output::ICON_INFO.cyan());
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
        let files: Vec<_> = recordings.iter().map(|path| {
            let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("unknown");
            let size = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
            let (first_tick, last_tick) = if let Ok(recording) = horus_core::scheduling::NodeRecording::load(path) {
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
        }).collect();
        let output = serde_json::json!({
            "session": session,
            "found": !recordings.is_empty(),
            "files": files,
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
        return Ok(());
    }

    if recordings.is_empty() {
        println!("{} Session '{}' not found.", cli_output::ICON_WARN.yellow(), session);
        return Ok(());
    }

    println!("{} Session: {}\n", cli_output::ICON_SUCCESS.green(), session.yellow().bold());

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

    println!("{} Deleted session '{}'", cli_output::ICON_SUCCESS.green(), session);
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
        println!("{} Will stop at tick {}", cli_output::ICON_INFO.cyan(), stop);
    }

    // Apply speed multiplier
    if (speed - 1.0).abs() > f64::EPSILON {
        scheduler = scheduler.with_replay_speed(speed);
        println!("{} Playback speed: {}x", cli_output::ICON_INFO.cyan(), speed);
    }

    // Apply overrides
    for (node, output, value_str) in &overrides {
        // Parse the value string into bytes
        // Support formats: hex (0x...), decimal numbers, or raw strings
        let value_bytes = if let Some(hex_str) = value_str.strip_prefix("0x") {
            // Hex format
            parse_hex_string(hex_str).unwrap_or_else(|_| value_str.as_bytes().to_vec())
        } else if let Ok(num) = value_str.parse::<f64>() {
            // Float number
            num.to_le_bytes().to_vec()
        } else if let Ok(num) = value_str.parse::<i64>() {
            // Integer number
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

    // Find matching nodes
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

        for path2 in &recordings2 {
            let name2 = path2
                .file_stem()
                .and_then(|n| n.to_str())
                .unwrap_or("")
                .split('@')
                .next()
                .unwrap_or("");

            if name1 == name2 && !name1.is_empty() && name1 != "scheduler" {
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
                        for diff in diffs.iter().take(max_diffs - total_diffs) {
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
                                    println!(
                                        "    Tick {} missing in recording {}",
                                        tick, in_recording
                                    );
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
                        println!("  {} Node '{}': identical", cli_output::ICON_SUCCESS.green(), name1);
                    }
                }
            }
        }
    }

    if total_diffs == 0 {
        println!("\n{} No differences found!", cli_output::ICON_SUCCESS.green());
    } else {
        println!("\n{} Total: {} difference(s)", cli_output::ICON_WARN.yellow(), total_diffs);
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

        writeln!(file, "{{")?;
        writeln!(file, "  \"session\": \"{}\",", session)?;
        writeln!(file, "  \"recordings\": [")?;

        for (i, path) in recordings.iter().enumerate() {
            if let Ok(recording) = horus_core::scheduling::NodeRecording::load(path) {
                let comma = if i < recordings.len() - 1 { "," } else { "" };
                writeln!(file, "    {{")?;
                writeln!(file, "      \"node_name\": \"{}\",", recording.node_name)?;
                writeln!(file, "      \"node_id\": \"{}\",", recording.node_id)?;
                writeln!(file, "      \"first_tick\": {},", recording.first_tick)?;
                writeln!(file, "      \"last_tick\": {},", recording.last_tick)?;
                writeln!(
                    file,
                    "      \"snapshot_count\": {}",
                    recording.snapshots.len()
                )?;
                writeln!(file, "    }}{}", comma)?;
            }
        }

        writeln!(file, "  ]")?;
        writeln!(file, "}}")?;

        println!("{} Exported to {:?}", cli_output::ICON_SUCCESS.green(), output);
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

                    writeln!(
                        file,
                        "{},{},{},{},{},{},\"{}\"",
                        recording.node_name,
                        recording.node_id,
                        snapshot.tick,
                        snapshot.timestamp_us,
                        snapshot.inputs.len(),
                        snapshot.outputs.len(),
                        outputs_str.join(";")
                    )?;
                }
            }
        }

        println!("{} Exported to {:?} (CSV format)", cli_output::ICON_SUCCESS.green(), output);
    } else {
        println!(
            "{} Format '{}' not supported. Use 'json' or 'csv'.",
            cli_output::ICON_WARN.yellow(),
            format
        );
    }
    Ok(())
}

/// Inject recorded nodes into a live/scripted run
#[allow(clippy::too_many_arguments)]
pub fn run_inject(
    session: String,
    nodes: Vec<String>,
    all: bool,
    script: Option<PathBuf>,
    start_tick: Option<u64>,
    stop_tick: Option<u64>,
    speed: f64,
    loop_playback: bool,
) -> HorusResult<()> {
    use horus_core::Scheduler;

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
    let mut scheduler = Scheduler::new().with_name(&format!("Inject({})", session));

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
                    println!("  {} Injected '{}'", cli_output::ICON_SUCCESS.green(), node_name);
                    injected_count += 1;
                }
                Err(e) => {
                    log::error!("Failed to inject '{}': {}", node_name, e);
                    eprintln!("  {} Failed to inject '{}': {}", cli_output::ICON_ERROR.red(), node_name, e);
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
        println!("{} Playback speed: {}x", cli_output::ICON_INFO.cyan(), speed);
    }

    // If a script is provided, compile and run it with the injected nodes
    if let Some(script_path) = &script {
        if !script_path.exists() {
            return Err(horus_internal!(
                "Script file not found: {}",
                script_path.display()
            ));
        }

        println!(
            "\n{} Compiling script: {}",
            cli_output::ICON_INFO.cyan(),
            script_path.display()
        );

        // Use the existing run infrastructure to compile and execute
        // We need to set up the environment for injection
        std::env::set_var("HORUS_INJECT_SESSION", &session);
        std::env::set_var(
            "HORUS_INJECT_NODES",
            if all {
                "*".to_string()
            } else {
                nodes.join(",")
            },
        );

        // For now, just inform user how to properly use injection with scripts
        // Full integration would require modifying the `horus run` command
        println!(
            "\n{} To run a script with injected recordings, use:",
            cli_output::ICON_INFO.cyan()
        );
        let inject_arg = if all {
            "--inject-all".to_string()
        } else {
            format!("--inject-nodes {}", nodes.join(","))
        };
        println!(
            "       horus run {} --inject {} {}",
            script_path.display(),
            session,
            inject_arg
        );
        println!(
            "\n{} Running injected nodes only for now...\n",
            cli_output::ICON_WARN.yellow()
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
                    println!("\n{} Recording finished, restarting...\n", cli_output::ICON_INFO.cyan());

                    // Recreate scheduler for next iteration
                    scheduler = Scheduler::new().with_name(&format!("Inject({})", session));

                    // Re-inject nodes
                    for path in &recordings {
                        let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
                        if filename.starts_with("scheduler@") {
                            continue;
                        }
                        let node_name = filename.split('@').next().unwrap_or("");
                        let should_inject = all || nodes.iter().any(|n| n == node_name);
                        if should_inject {
                            let _ = scheduler.add_replay(path.clone(), 0);
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
                    println!("\n{} Loop interrupted: {}", cli_output::ICON_WARN.yellow(), e);
                    break;
                }
            }
        }
    } else {
        // Single run
        scheduler.run()?;
        println!("\n{} Injection replay completed", cli_output::ICON_SUCCESS.green());
    }

    Ok(())
}
