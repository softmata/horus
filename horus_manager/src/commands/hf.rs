//! HFrame (hf) commands - Interact with HORUS coordinate transform frames
//!
//! Provides commands for listing, echoing, and inspecting transform frames.
//! HORUS equivalent to ROS2 tf2 tools (tf_echo, view_frames).
//!
//! Usage:
//!   horus hf list          - List all frames
//!   horus hf echo A B      - Echo transform from A to B
//!   horus hf tree          - Show frame tree structure
//!   horus hf info `<frame>`  - Show frame details

use colored::*;
use horus_core::communication::Topic;
use horus_core::error::HorusResult;
use horus_library::hframe::{HFMessage, HFrame, Transform, TransformStamped};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::discovery::discover_shared_memory;

/// Standard HFrame topic names
const HF_TOPIC: &str = "hf";
const HF_STATIC_TOPIC: &str = "hf_static";

/// Frame data collected from shared memory
#[derive(Debug, Clone)]
pub struct FrameData {
    pub parent: String,
    pub child: String,
    pub transform: Transform,
    pub timestamp_ns: u64,
    pub is_static: bool,
    pub last_update: Instant,
    pub update_count: u64,
}

/// Live HFrame reader that collects transforms from shared memory
pub struct HFrameReader {
    hframe: HFrame,
    pub frame_data: HashMap<String, FrameData>,
    pub root_frames: HashSet<String>,
    /// Pending transforms: child -> (parent, transform, timestamp_ns, is_static)
    pending_transforms: HashMap<String, (String, Transform, u64, bool)>,
}

impl Default for HFrameReader {
    fn default() -> Self {
        Self::new()
    }
}

impl HFrameReader {
    pub fn new() -> Self {
        Self {
            hframe: HFrame::new(),
            frame_data: HashMap::new(),
            root_frames: HashSet::new(),
            pending_transforms: HashMap::new(),
        }
    }

    /// Try to read transforms from shared memory topics
    pub fn read_from_shm(&mut self) -> bool {
        let mut found_any = false;

        // Try to read from dynamic tf topic
        // Use recv() for streaming (new messages only)
        if let Ok(topic) = Topic::<HFMessage>::new(HF_TOPIC) {
            if let Some(msg) = topic.recv() {
                for tf in msg.iter() {
                    self.add_transform(tf, false);
                    found_any = true;
                }
            }
        }

        // Try to read from static tf topic
        // Use read_latest() since static transforms are published once and we need
        // to see them even if they were published before we opened the topic
        if let Ok(topic) = Topic::<HFMessage>::new(HF_STATIC_TOPIC) {
            if let Some(msg) = topic.read_latest() {
                for tf in msg.iter() {
                    self.add_transform(tf, true);
                    found_any = true;
                }
            }
        }

        // Also check for any other transform-related topics
        if let Ok(topics) = discover_shared_memory() {
            for topic_info in topics {
                if topic_info.topic_name.contains("transform")
                    || topic_info.topic_name.contains("hframe")
                {
                    if let Ok(topic) = Topic::<TransformStamped>::new(&topic_info.topic_name) {
                        if let Some(tf) = topic.recv() {
                            self.add_transform(&tf, false);
                            found_any = true;
                        }
                    }
                }
            }
        }

        // Now process pending transforms in the correct order
        self.process_pending_transforms();

        found_any
    }

    /// Process pending transforms in topological order (roots first, then children)
    fn process_pending_transforms(&mut self) {
        if self.pending_transforms.is_empty() {
            return;
        }

        // Find all root frames (frames that appear as parents but not as children)
        let children: HashSet<_> = self.pending_transforms.keys().cloned().collect();
        let parents: HashSet<_> = self
            .pending_transforms
            .values()
            .map(|(p, _, _, _)| p.clone())
            .collect();
        let roots: Vec<_> = parents.difference(&children).cloned().collect();

        // Clear root_frames and rebuild
        self.root_frames.clear();

        // Register root frames first (they have no parent)
        for root in &roots {
            if !self.hframe.has_frame(root) {
                let _ = self
                    .hframe
                    .register_static_frame(root, None, &Transform::identity());
                self.root_frames.insert(root.clone());
            }
        }

        // Process transforms in order: we need to ensure parents are registered before children
        // Use a simple iterative approach - keep processing until all are done
        let mut remaining = self.pending_transforms.clone();
        let mut max_iterations = remaining.len() * 2; // Prevent infinite loops

        while !remaining.is_empty() && max_iterations > 0 {
            max_iterations -= 1;
            let mut processed = Vec::new();

            for (child, (parent, transform, timestamp, is_static)) in &remaining {
                // Only process if parent is already registered
                if self.hframe.has_frame(parent) {
                    if !self.hframe.has_frame(child) {
                        // Register new frame
                        if *is_static {
                            let _ =
                                self.hframe
                                    .register_static_frame(child, Some(parent), transform);
                        } else {
                            let _ = self.hframe.register_frame(child, Some(parent));
                            let _ = self.hframe.update_transform(child, transform, *timestamp);
                        }
                    } else {
                        // Update existing frame
                        if *is_static {
                            let _ = self.hframe.set_static_transform(child, transform);
                        } else {
                            let _ = self.hframe.update_transform(child, transform, *timestamp);
                        }
                    }
                    processed.push(child.clone());
                }
            }

            // Remove processed transforms
            for child in processed {
                remaining.remove(&child);
            }

            // If we didn't process anything, we might have a cycle or missing parent
            // Try to register any remaining frames' parents as roots
            for (parent, _transform, _, _is_static) in remaining.values() {
                if !self.hframe.has_frame(parent) {
                    let _ = self
                        .hframe
                        .register_static_frame(parent, None, &Transform::identity());
                }
            }
        }

        // Clear pending transforms after processing
        self.pending_transforms.clear();
    }

    /// Add a transform to the pending list
    /// We collect all transforms first, then process them in the correct order
    fn add_transform(&mut self, tf: &TransformStamped, is_static: bool) {
        let parent = tf.parent_frame_id();
        let child = tf.child_frame_id();

        if parent.is_empty() || child.is_empty() {
            return;
        }

        // Store the transform info for later processing
        // Overwrite any existing transform for this child (keep latest parent info)
        self.pending_transforms.insert(
            child.clone(),
            (parent.clone(), tf.transform, tf.timestamp_ns, is_static),
        );

        // Track frame data for display
        let entry = self.frame_data.entry(child.clone()).or_insert(FrameData {
            parent: parent.clone(),
            child: child.clone(),
            transform: tf.transform,
            timestamp_ns: tf.timestamp_ns,
            is_static,
            last_update: Instant::now(),
            update_count: 0,
        });

        entry.transform = tf.transform;
        entry.timestamp_ns = tf.timestamp_ns;
        entry.last_update = Instant::now();
        entry.update_count += 1;
    }

    /// Get the frame tree as a list of (parent, children) pairs
    pub fn get_frame_tree(&self) -> Vec<(String, Vec<String>)> {
        let mut tree: HashMap<String, Vec<String>> = HashMap::new();

        for (child, data) in &self.frame_data {
            tree.entry(data.parent.clone())
                .or_default()
                .push(child.clone());
        }

        // Sort children for consistent display
        for children in tree.values_mut() {
            children.sort();
        }

        let mut result: Vec<_> = tree.into_iter().collect();
        result.sort_by(|a, b| a.0.cmp(&b.0));
        result
    }

    /// Get all frame names
    pub fn get_all_frames(&self) -> Vec<String> {
        let mut frames: HashSet<String> = HashSet::new();
        for data in self.frame_data.values() {
            frames.insert(data.parent.clone());
            frames.insert(data.child.clone());
        }
        let mut result: Vec<_> = frames.into_iter().collect();
        result.sort();
        result
    }
}

/// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
fn quaternion_to_euler(rotation: [f64; 4]) -> (f64, f64, f64) {
    let [x, y, z, w] = rotation;

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f64::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (roll, pitch, yaw)
}

/// List all coordinate frames in the HFrame tree
pub fn list_frames(verbose: bool, json: bool) -> HorusResult<()> {
    // First try to read actual frame data from shared memory
    let mut reader = HFrameReader::new();
    let found_live_data = reader.read_from_shm();

    // Also get topic discovery info
    let topics = discover_shared_memory()?;
    let tf_topics: Vec<_> = topics
        .iter()
        .filter(|t| {
            t.topic_name.contains("hf")
                || t.topic_name.contains("hframe")
                || t.topic_name.contains("transform")
        })
        .collect();

    if json {
        let frames = reader.get_all_frames();
        let json_output = serde_json::json!({
            "frames": frames.iter().map(|name| {
                if let Some(data) = reader.frame_data.get(name) {
                    serde_json::json!({
                        "name": name,
                        "parent": data.parent,
                        "is_static": data.is_static,
                        "translation": data.transform.translation,
                        "rotation": data.transform.rotation,
                        "timestamp_ns": data.timestamp_ns,
                    })
                } else {
                    serde_json::json!({
                        "name": name,
                        "parent": null,
                        "is_static": false,
                    })
                }
            }).collect::<Vec<_>>(),
            "topics": tf_topics.iter().map(|t| {
                serde_json::json!({
                    "topic": t.topic_name,
                    "active": t.active,
                    "rate_hz": t.message_rate_hz,
                })
            }).collect::<Vec<_>>(),
            "frame_count": reader.get_all_frames().len(),
            "topic_count": tf_topics.len(),
            "live_data": found_live_data,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&json_output).unwrap_or_default()
        );
        return Ok(());
    }

    // Show frames if we found any
    let frames = reader.get_all_frames();

    if frames.is_empty() && tf_topics.is_empty() {
        println!("{}", "No active transform frames found.".yellow());
        println!(
            "  {} Start a HORUS application with HFrame publishing enabled",
            "Tip:".dimmed()
        );
        println!(
            "  {} Use sim3d or add HFramePublisher to your nodes",
            "    ".dimmed()
        );
        return Ok(());
    }

    if !frames.is_empty() {
        println!("{}", "Coordinate Frames:".green().bold());
        println!();

        if verbose {
            for name in &frames {
                if let Some(data) = reader.frame_data.get(name) {
                    println!("  {} {}", "Frame:".cyan(), name.white().bold());
                    println!("    {} {}", "Parent:".dimmed(), data.parent);
                    println!(
                        "    {} {}",
                        "Type:".dimmed(),
                        if data.is_static {
                            "static".green()
                        } else {
                            "dynamic".yellow()
                        }
                    );
                    println!(
                        "    {} [{:.4}, {:.4}, {:.4}]",
                        "Translation:".dimmed(),
                        data.transform.translation[0],
                        data.transform.translation[1],
                        data.transform.translation[2]
                    );
                    println!(
                        "    {} [{:.4}, {:.4}, {:.4}, {:.4}]",
                        "Rotation:".dimmed(),
                        data.transform.rotation[0],
                        data.transform.rotation[1],
                        data.transform.rotation[2],
                        data.transform.rotation[3]
                    );
                    println!();
                } else {
                    println!(
                        "  {} {} {}",
                        "Frame:".cyan(),
                        name.white().bold(),
                        "(root)".dimmed()
                    );
                    println!();
                }
            }
        } else {
            println!(
                "  {:<25} {:<20} {:>10}",
                "FRAME".dimmed(),
                "PARENT".dimmed(),
                "TYPE".dimmed()
            );
            println!("  {}", "-".repeat(58).dimmed());

            for name in &frames {
                if let Some(data) = reader.frame_data.get(name) {
                    let type_str = if data.is_static {
                        "static".green()
                    } else {
                        "dynamic".yellow()
                    };
                    println!("  {:<25} {:<20} {:>10}", name, data.parent, type_str);
                } else {
                    println!("  {:<25} {:<20} {:>10}", name, "(root)", "root".cyan());
                }
            }
        }

        println!();
        println!(
            "  {} {} frames",
            "Total:".dimmed(),
            frames.len().to_string().green()
        );
    }

    // Show topic info
    if !tf_topics.is_empty() {
        println!();
        println!("{}", "Transform Topics:".green().bold());
        println!();
        println!(
            "  {:<25} {:>10} {:>12}",
            "TOPIC".dimmed(),
            "RATE".dimmed(),
            "STATUS".dimmed()
        );
        println!("  {}", "-".repeat(50).dimmed());

        for topic in &tf_topics {
            let status = if topic.active {
                "active".green()
            } else {
                "stale".red()
            };
            println!(
                "  {:<25} {:>8.1} Hz {:>12}",
                topic.topic_name, topic.message_rate_hz, status
            );
        }
    }

    Ok(())
}

/// Echo transform between two frames continuously (like ros2 run tf2_ros tf_echo)
pub fn echo_transform(
    source_frame: &str,
    target_frame: &str,
    rate: Option<f64>,
    count: Option<usize>,
) -> HorusResult<()> {
    println!(
        "{} {} {} {}",
        "Echoing transform from".cyan(),
        source_frame.white().bold(),
        "to".cyan(),
        target_frame.white().bold()
    );
    println!("{}", "(Ctrl+C to stop)".dimmed());
    println!();

    let rate_hz = rate.unwrap_or(10.0);
    let interval = Duration::from_secs_f64(1.0 / rate_hz);
    let mut messages_received = 0;
    let max_messages = count.unwrap_or(usize::MAX);

    // Set up Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .ok();

    // Create HFrame reader
    let mut reader = HFrameReader::new();

    // Initial read to populate frames
    reader.read_from_shm();

    while running.load(Ordering::SeqCst) && messages_received < max_messages {
        // Read latest transforms
        reader.read_from_shm();

        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();

        // Try to get the transform between frames
        match reader.hframe.tf(source_frame, target_frame) {
            Ok(tf) => {
                let (roll, pitch, yaw) = quaternion_to_euler(tf.rotation);

                println!("At time {:.3}", now.as_secs_f64());
                println!(
                    "- Translation: [{:.6}, {:.6}, {:.6}]",
                    tf.translation[0], tf.translation[1], tf.translation[2]
                );
                println!(
                    "- Rotation: in Quaternion [{:.6}, {:.6}, {:.6}, {:.6}]",
                    tf.rotation[0], tf.rotation[1], tf.rotation[2], tf.rotation[3]
                );
                println!(
                    "- Rotation: in RPY (radian) [{:.6}, {:.6}, {:.6}]",
                    roll, pitch, yaw
                );
                println!(
                    "- Rotation: in RPY (degree) [{:.2}, {:.2}, {:.2}]",
                    roll.to_degrees(),
                    pitch.to_degrees(),
                    yaw.to_degrees()
                );
                println!();
            }
            Err(_) => {
                println!(
                    "{} Transform from '{}' to '{}' not available",
                    "Warning:".yellow(),
                    source_frame,
                    target_frame
                );

                // Show available frames
                let frames = reader.get_all_frames();
                if !frames.is_empty() {
                    println!("  {} {}", "Available frames:".dimmed(), frames.join(", "));
                } else {
                    println!(
                        "  {}",
                        "No frames found. Is a HORUS application running?".dimmed()
                    );
                }
                println!();
            }
        }

        messages_received += 1;
        std::thread::sleep(interval);
    }

    println!("{}", "Stopped.".dimmed());
    Ok(())
}

/// Recursively print frame tree
fn print_tree_recursive(
    parent: &str,
    tree: &HashMap<String, Vec<String>>,
    frame_data: &HashMap<String, FrameData>,
    prefix: &str,
    is_last: bool,
) {
    let connector = if is_last { "└── " } else { "├── " };
    let frame_type = frame_data
        .get(parent)
        .map(|d| {
            if d.is_static {
                " (static)".dimmed()
            } else {
                " (dynamic)".dimmed()
            }
        })
        .unwrap_or_else(|| "".normal());

    println!("{}{}{}{}", prefix, connector, parent.cyan(), frame_type);

    if let Some(children) = tree.get(parent) {
        let new_prefix = format!("{}{}", prefix, if is_last { "    " } else { "│   " });
        for (i, child) in children.iter().enumerate() {
            let child_is_last = i == children.len() - 1;
            print_tree_recursive(child, tree, frame_data, &new_prefix, child_is_last);
        }
    }
}

/// Show the frame tree structure (like view_frames in ROS)
pub fn view_frames(output_file: Option<&str>) -> HorusResult<()> {
    let mut reader = HFrameReader::new();
    reader.read_from_shm();

    println!("{}", "HFrame Tree Structure:".green().bold());
    println!();

    let frames = reader.get_all_frames();

    if frames.is_empty() {
        println!("{}", "No frames found.".yellow());
        println!(
            "  {}",
            "Start a HORUS application with HFrame publishing enabled".dimmed()
        );
        return Ok(());
    }

    // Build tree structure
    let mut tree: HashMap<String, Vec<String>> = HashMap::new();
    let mut all_children: HashSet<String> = HashSet::new();

    for (child, data) in &reader.frame_data {
        tree.entry(data.parent.clone())
            .or_default()
            .push(child.clone());
        all_children.insert(child.clone());
    }

    // Sort children
    for children in tree.values_mut() {
        children.sort();
    }

    // Find root frames (frames that are parents but not children)
    let mut roots: Vec<String> = tree
        .keys()
        .filter(|p| !all_children.contains(*p))
        .cloned()
        .collect();
    roots.sort();

    if roots.is_empty() {
        // If no clear root, use all frames that have no parent in our data
        roots = frames
            .iter()
            .filter(|f| !reader.frame_data.contains_key(*f))
            .cloned()
            .collect();
    }

    // Print tree starting from each root
    for (i, root) in roots.iter().enumerate() {
        let is_last = i == roots.len() - 1;
        let root_type = if reader.root_frames.contains(root) {
            " (root)".dimmed()
        } else {
            "".normal()
        };

        if is_last {
            println!("  └── {}{}", root.white().bold(), root_type);
        } else {
            println!("  ├── {}{}", root.white().bold(), root_type);
        }

        if let Some(children) = tree.get(root) {
            let prefix = if is_last { "      " } else { "  │   " };
            for (j, child) in children.iter().enumerate() {
                let child_is_last = j == children.len() - 1;
                print_tree_recursive(child, &tree, &reader.frame_data, prefix, child_is_last);
            }
        }
    }

    println!();
    println!(
        "  {} {} frames",
        "Total:".dimmed(),
        frames.len().to_string().green()
    );

    if let Some(file) = output_file {
        // Generate DOT file for graphviz
        let mut dot = String::from("digraph HFrameTree {\n");
        dot.push_str("  rankdir=TB;\n");
        dot.push_str("  node [shape=box];\n\n");

        for (child, data) in &reader.frame_data {
            let style = if data.is_static {
                "style=filled,fillcolor=lightblue"
            } else {
                "style=filled,fillcolor=lightyellow"
            };
            dot.push_str(&format!("  \"{}\" [{}];\n", child, style));
            dot.push_str(&format!("  \"{}\" -> \"{}\";\n", data.parent, child));
        }

        dot.push_str("}\n");

        std::fs::write(file, &dot)?;
        println!();
        println!("{} {}", "Frame tree saved to:".green(), file.white().bold());
        println!(
            "  {} dot -Tpng {} -o frames.png",
            "Render with:".dimmed(),
            file
        );
    }

    Ok(())
}

/// Show information about a specific frame
pub fn frame_info(frame_name: &str) -> HorusResult<()> {
    let mut reader = HFrameReader::new();
    reader.read_from_shm();

    println!(
        "{} {}",
        "Frame Info:".green().bold(),
        frame_name.white().bold()
    );
    println!();

    if let Some(data) = reader.frame_data.get(frame_name) {
        println!("  {} {}", "Name:".cyan(), frame_name);
        println!("  {} {}", "Parent:".cyan(), data.parent);
        println!(
            "  {} {}",
            "Type:".cyan(),
            if data.is_static {
                "static".green()
            } else {
                "dynamic".yellow()
            }
        );

        // Format timestamp
        let secs = data.timestamp_ns / 1_000_000_000;
        let nanos = data.timestamp_ns % 1_000_000_000;
        println!(
            "  {} {}.{:09} ({} ns)",
            "Last Update:".cyan(),
            secs,
            nanos,
            data.timestamp_ns
        );

        println!();
        println!("  {}", "Transform to parent:".cyan());
        println!(
            "    {} [{:.6}, {:.6}, {:.6}]",
            "Translation:".dimmed(),
            data.transform.translation[0],
            data.transform.translation[1],
            data.transform.translation[2]
        );
        println!(
            "    {} [{:.6}, {:.6}, {:.6}, {:.6}]",
            "Rotation (quat):".dimmed(),
            data.transform.rotation[0],
            data.transform.rotation[1],
            data.transform.rotation[2],
            data.transform.rotation[3]
        );

        let (roll, pitch, yaw) = quaternion_to_euler(data.transform.rotation);
        println!(
            "    {} [{:.4}, {:.4}, {:.4}]",
            "Rotation (RPY rad):".dimmed(),
            roll,
            pitch,
            yaw
        );
        println!(
            "    {} [{:.2}°, {:.2}°, {:.2}°]",
            "Rotation (RPY deg):".dimmed(),
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees()
        );

        // Get children
        let children = reader.hframe.children(frame_name);
        if !children.is_empty() {
            println!();
            println!("  {} {}", "Children:".cyan(), children.join(", "));
        }
    } else {
        // Check if frame exists as a root
        if reader.get_all_frames().contains(&frame_name.to_string()) {
            println!("  {} {}", "Name:".cyan(), frame_name);
            println!("  {} {}", "Type:".cyan(), "root".green());

            let children = reader.hframe.children(frame_name);
            if !children.is_empty() {
                println!("  {} {}", "Children:".cyan(), children.join(", "));
            }
        } else {
            println!("{} Frame '{}' not found", "Error:".red(), frame_name);

            let frames = reader.get_all_frames();
            if !frames.is_empty() {
                println!();
                println!("  {} {}", "Available frames:".dimmed(), frames.join(", "));
            } else {
                println!(
                    "  {}",
                    "No frames found. Is a HORUS application running?".dimmed()
                );
            }
        }
    }

    Ok(())
}

/// Check if a transform is available between two frames
pub fn can_transform(source_frame: &str, target_frame: &str) -> HorusResult<()> {
    let mut reader = HFrameReader::new();
    reader.read_from_shm();

    println!(
        "{} {} {} {}",
        "Checking transform from".cyan(),
        source_frame.white().bold(),
        "to".cyan(),
        target_frame.white().bold()
    );
    println!();

    let can = reader.hframe.can_transform(source_frame, target_frame);

    println!(
        "  {} {}",
        "Available:".cyan(),
        if can {
            "Yes".green().bold()
        } else {
            "No".red().bold()
        }
    );

    if can {
        // Show the chain
        if let Ok(chain) = reader.hframe.frame_chain(source_frame, target_frame) {
            println!("  {} {}", "Chain:".cyan(), chain.join(" → "));
        }

        // Show the transform
        if let Ok(tf) = reader.hframe.tf(source_frame, target_frame) {
            println!();
            println!("  {}", "Transform:".cyan());
            println!(
                "    {} [{:.6}, {:.6}, {:.6}]",
                "Translation:".dimmed(),
                tf.translation[0],
                tf.translation[1],
                tf.translation[2]
            );
            println!(
                "    {} [{:.6}, {:.6}, {:.6}, {:.6}]",
                "Rotation:".dimmed(),
                tf.rotation[0],
                tf.rotation[1],
                tf.rotation[2],
                tf.rotation[3]
            );
        }
    } else {
        let frames = reader.get_all_frames();
        let source_exists = frames.contains(&source_frame.to_string());
        let target_exists = frames.contains(&target_frame.to_string());

        if !source_exists {
            println!(
                "  {} Frame '{}' not found",
                "Reason:".yellow(),
                source_frame
            );
        }
        if !target_exists {
            println!(
                "  {} Frame '{}' not found",
                "Reason:".yellow(),
                target_frame
            );
        }
        if source_exists && target_exists {
            println!("  {} No path exists between frames", "Reason:".yellow());
        }

        if !frames.is_empty() {
            println!();
            println!("  {} {}", "Available frames:".dimmed(), frames.join(", "));
        }
    }

    Ok(())
}

/// Monitor transform update rates
pub fn monitor_rates(window: Option<usize>) -> HorusResult<()> {
    let window_size = window.unwrap_or(10);

    println!(
        "{} (window: {} samples)",
        "Transform Update Rates:".green().bold(),
        window_size
    );
    println!("{}", "(Ctrl+C to stop)".dimmed());
    println!();

    // Set up Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .ok();

    // Track update rates per frame
    let mut rate_trackers: HashMap<String, Vec<Instant>> = HashMap::new();
    let mut last_update_counts: HashMap<String, u64> = HashMap::new();

    let mut reader = HFrameReader::new();

    while running.load(Ordering::SeqCst) {
        // Clear screen and move cursor to top
        print!("\x1B[2J\x1B[1;1H");

        println!(
            "{} (window: {} samples)",
            "Transform Update Rates:".green().bold(),
            window_size
        );
        println!("{}", "(Ctrl+C to stop)".dimmed());
        println!();

        reader.read_from_shm();

        // Update rate trackers
        let now = Instant::now();
        for (frame, data) in &reader.frame_data {
            let prev_count = last_update_counts.get(frame).copied().unwrap_or(0);
            if data.update_count > prev_count {
                let tracker = rate_trackers.entry(frame.clone()).or_default();
                tracker.push(now);

                // Keep only recent samples within window
                while tracker.len() > window_size {
                    tracker.remove(0);
                }

                last_update_counts.insert(frame.clone(), data.update_count);
            }
        }

        // Display rates
        println!(
            "  {:<25} {:>12} {:>12} {:>10}",
            "FRAME".dimmed(),
            "RATE (Hz)".dimmed(),
            "AVG (Hz)".dimmed(),
            "TYPE".dimmed()
        );
        println!("  {}", "-".repeat(62).dimmed());

        let mut frames: Vec<_> = reader.frame_data.keys().collect();
        frames.sort();

        for frame in frames {
            let data = &reader.frame_data[frame];
            let tracker = rate_trackers.get(frame);

            let (current_rate, avg_rate) = if let Some(samples) = tracker {
                if let (Some(first), Some(last)) = (samples.first(), samples.last()) {
                    let duration = last.duration_since(*first);
                    let rate = if duration.as_secs_f64() > 0.0 {
                        (samples.len() - 1) as f64 / duration.as_secs_f64()
                    } else {
                        0.0
                    };
                    (rate, rate)
                } else {
                    (0.0, 0.0)
                }
            } else {
                (0.0, 0.0)
            };

            let type_str = if data.is_static {
                "static".green()
            } else {
                "dynamic".yellow()
            };

            println!(
                "  {:<25} {:>10.1} Hz {:>10.1} Hz {:>10}",
                frame, current_rate, avg_rate, type_str
            );
        }

        if reader.frame_data.is_empty() {
            println!(
                "  {}",
                "No frames found. Is a HORUS application running?".dimmed()
            );
        }

        std::thread::sleep(Duration::from_millis(100));
    }

    println!();
    println!("{}", "Stopped.".dimmed());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quaternion_to_euler_identity() {
        let (roll, pitch, yaw) = quaternion_to_euler([0.0, 0.0, 0.0, 1.0]);
        assert!((roll).abs() < 1e-10);
        assert!((pitch).abs() < 1e-10);
        assert!((yaw).abs() < 1e-10);
    }

    #[test]
    fn test_list_frames_empty() {
        // Should handle empty case gracefully
        let result = list_frames(false, false);
        assert!(result.is_ok());
    }

    #[test]
    fn test_view_frames() {
        let result = view_frames(None);
        assert!(result.is_ok());
    }

    #[test]
    fn test_frame_info() {
        let result = frame_info("base_link");
        assert!(result.is_ok());
    }

    #[test]
    fn test_can_transform() {
        let result = can_transform("base_link", "camera_link");
        assert!(result.is_ok());
    }

    #[test]
    fn test_hframe_reader_new() {
        let reader = HFrameReader::new();
        assert!(reader.frame_data.is_empty());
        assert!(reader.root_frames.is_empty());
    }
}
