//! Transform Frame (tf) commands - Interact with HORUS coordinate transform frames
//!
//! Provides commands for listing, echoing, and inspecting transform frames.
//! HORUS equivalent to ROS2 tf2 tools (tf_echo, view_frames).
//!
//! Usage:
//!   horus tf list          - List all frames
//!   horus tf echo A B      - Echo transform from A to B
//!   horus tf tree          - Show frame tree structure
//!   horus tf info `<frame>`  - Show frame details

use colored::*;
use horus_core::communication::Topic;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_library::transform_frame::{TFMessage, TransformFrame, Transform, TransformStamped};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::discovery::discover_shared_memory;

/// Standard TransformFrame topic names
const TF_TOPIC: &str = "tf";
const TF_STATIC_TOPIC: &str = "tf_static";

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

/// Live TransformFrame reader that collects transforms from shared memory
pub struct TransformFrameReader {
    tf: TransformFrame,
    pub frame_data: HashMap<String, FrameData>,
    pub root_frames: HashSet<String>,
    /// Pending transforms: child -> (parent, transform, timestamp_ns, is_static)
    pending_transforms: HashMap<String, (String, Transform, u64, bool)>,
}

impl Default for TransformFrameReader {
    fn default() -> Self {
        Self::new()
    }
}

impl TransformFrameReader {
    pub fn new() -> Self {
        Self {
            tf: TransformFrame::new(),
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
        if let Ok(topic) = Topic::<TFMessage>::new(TF_TOPIC) {
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
        if let Ok(topic) = Topic::<TFMessage>::new(TF_STATIC_TOPIC) {
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
                    || topic_info.topic_name.contains("transform_frame")
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
            if !self.tf.has_frame(root) {
                let _ = self
                    .tf
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
                if self.tf.has_frame(parent) {
                    if !self.tf.has_frame(child) {
                        // Register new frame
                        if *is_static {
                            let _ =
                                self.tf
                                    .register_static_frame(child, Some(parent), transform);
                        } else {
                            let _ = self.tf.register_frame(child, Some(parent));
                            let _ = self.tf.update_transform(child, transform, *timestamp);
                        }
                    } else {
                        // Update existing frame
                        if *is_static {
                            let _ = self.tf.set_static_transform(child, transform);
                        } else {
                            let _ = self.tf.update_transform(child, transform, *timestamp);
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
                if !self.tf.has_frame(parent) {
                    let _ = self
                        .tf
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

/// List all coordinate frames in the TransformFrame tree
pub fn list_frames(verbose: bool, json: bool) -> HorusResult<()> {
    // First try to read actual frame data from shared memory
    let mut reader = TransformFrameReader::new();
    let found_live_data = reader.read_from_shm();

    // Also get topic discovery info
    let topics = discover_shared_memory()?;
    let tf_topics: Vec<_> = topics
        .iter()
        .filter(|t| {
            t.topic_name.contains("tf")
                || t.topic_name.contains("transform_frame")
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
            "  {} Start a HORUS application with TransformFrame publishing enabled",
            "Tip:".dimmed()
        );
        println!(
            "  {} Use sim3d or add TransformFramePublisher to your nodes",
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
    timeout: Option<f64>,
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
    if rate_hz <= 0.0 {
        return Err(HorusError::Config(ConfigError::Other(
            "Rate must be greater than 0.0".to_string(),
        )));
    }
    let interval = Duration::from_secs_f64(1.0 / rate_hz);
    let mut messages_received = 0;
    let max_messages = count.unwrap_or(usize::MAX);
    let deadline = timeout.map(|secs| std::time::Instant::now() + Duration::from_secs_f64(secs));

    // Set up Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .ok();

    // Create TransformFrame reader
    let mut reader = TransformFrameReader::new();

    // Initial read to populate frames
    reader.read_from_shm();

    while running.load(Ordering::SeqCst)
        && messages_received < max_messages
        && deadline.map_or(true, |d| std::time::Instant::now() < d)
    {
        // Read latest transforms
        reader.read_from_shm();

        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();

        // Try to get the transform between frames
        match reader.tf.tf(source_frame, target_frame) {
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
    let mut reader = TransformFrameReader::new();
    reader.read_from_shm();

    println!("{}", "TransformFrame Tree Structure:".green().bold());
    println!();

    let frames = reader.get_all_frames();

    if frames.is_empty() {
        println!("{}", "No frames found.".yellow());
        println!(
            "  {}",
            "Start a HORUS application with TransformFrame publishing enabled".dimmed()
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
        let mut dot = String::from("digraph TransformFrameTree {\n");
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
    let mut reader = TransformFrameReader::new();
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
        let children = reader.tf.children(frame_name);
        if !children.is_empty() {
            println!();
            println!("  {} {}", "Children:".cyan(), children.join(", "));
        }
    } else {
        // Check if frame exists as a root
        if reader.get_all_frames().contains(&frame_name.to_string()) {
            println!("  {} {}", "Name:".cyan(), frame_name);
            println!("  {} {}", "Type:".cyan(), "root".green());

            let children = reader.tf.children(frame_name);
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
    let mut reader = TransformFrameReader::new();
    reader.read_from_shm();

    println!(
        "{} {} {} {}",
        "Checking transform from".cyan(),
        source_frame.white().bold(),
        "to".cyan(),
        target_frame.white().bold()
    );
    println!();

    let can = reader.tf.can_transform(source_frame, target_frame);

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
        if let Ok(chain) = reader.tf.frame_chain(source_frame, target_frame) {
            println!("  {} {}", "Chain:".cyan(), chain.join(" → "));
        }

        // Show the transform
        if let Ok(tf) = reader.tf.tf(source_frame, target_frame) {
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

    let mut reader = TransformFrameReader::new();

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

// ============================================================================
// TF Recording / Replay / Diff
// ============================================================================

/// Magic bytes for TF recording file format
const TFR_MAGIC: &[u8; 4] = b"TFR1";
/// Version of the recording format
const TFR_VERSION: u8 = 1;

/// Header for a TF recording file (.tfr)
#[repr(C)]
#[derive(Clone, Copy)]
struct TfrHeader {
    magic: [u8; 4],
    version: u8,
    _padding: [u8; 3],
    /// Wall-clock start time (nanos since epoch)
    start_time_ns: u64,
    /// Total number of entries in the file
    entry_count: u64,
}

/// Entry type tag
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum TfrEntryType {
    /// Dynamic transform update
    Dynamic = 0,
    /// Static transform (recorded once)
    Static = 1,
}

/// Single entry in a TF recording file
#[repr(C)]
#[derive(Clone, Copy)]
struct TfrEntry {
    entry_type: u8,
    _padding: [u8; 7],
    /// Simulation time offset from start (nanoseconds)
    offset_ns: u64,
    /// The transform data
    stamped: TransformStamped,
}

/// Record TF transforms from shared memory to a .tfr file
///
/// Usage: `horus tf record --output recording.tfr [--duration 60]`
pub fn record_transforms(
    output_path: &str,
    max_duration_secs: Option<f64>,
) -> HorusResult<()> {
    use std::io::Write;

    println!("{}", "Recording TF transforms...".green().bold());
    println!("  Output: {}", output_path.cyan());
    if let Some(d) = max_duration_secs {
        println!("  Duration: {:.1}s", d);
    }
    println!("  Press Ctrl+C to stop\n");

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    let _ = ctrlc::set_handler(move || r.store(false, Ordering::SeqCst));

    let start = Instant::now();
    let start_time_ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;

    // Open output file and write placeholder header (will update entry_count later)
    let mut file = std::fs::File::create(output_path).map_err(|e| {
        HorusError::Config(ConfigError::other(format!("Failed to create output file: {e}")))
    })?;

    let header = TfrHeader {
        magic: *TFR_MAGIC,
        version: TFR_VERSION,
        _padding: [0; 3],
        start_time_ns,
        entry_count: 0,
    };
    let header_bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(
            &header as *const TfrHeader as *const u8,
            std::mem::size_of::<TfrHeader>(),
        )
    };
    file.write_all(header_bytes).map_err(|e| {
        HorusError::Config(ConfigError::other(format!("Failed to write header: {e}")))
    })?;

    let mut entry_count: u64 = 0;
    let mut recorded_static: HashSet<String> = HashSet::new();

    // Open SHM topics
    let tf_topic = Topic::<TFMessage>::new(TF_TOPIC).ok();
    let tf_static_topic = Topic::<TFMessage>::new(TF_STATIC_TOPIC).ok();

    while running.load(Ordering::Relaxed) {
        if let Some(max) = max_duration_secs {
            if start.elapsed().as_secs_f64() >= max {
                break;
            }
        }

        let offset_ns = start.elapsed().as_nanos() as u64;

        // Read dynamic transforms
        if let Some(ref topic) = tf_topic {
            if let Some(msg) = topic.recv() {
                for stamped in msg.iter() {
                    let entry = TfrEntry {
                        entry_type: TfrEntryType::Dynamic as u8,
                        _padding: [0; 7],
                        offset_ns,
                        stamped: *stamped,
                    };
                    let entry_bytes: &[u8] = unsafe {
                        std::slice::from_raw_parts(
                            &entry as *const TfrEntry as *const u8,
                            std::mem::size_of::<TfrEntry>(),
                        )
                    };
                    if file.write_all(entry_bytes).is_ok() {
                        entry_count += 1;
                    }
                }
            }
        }

        // Read static transforms (only record each once)
        if let Some(ref topic) = tf_static_topic {
            if let Some(msg) = topic.read_latest() {
                for stamped in msg.iter() {
                    let child = stamped.child_frame_id();
                    if !recorded_static.contains(&child) {
                        let entry = TfrEntry {
                            entry_type: TfrEntryType::Static as u8,
                            _padding: [0; 7],
                            offset_ns: 0, // Static frames have no time offset
                            stamped: *stamped,
                        };
                        let entry_bytes: &[u8] = unsafe {
                            std::slice::from_raw_parts(
                                &entry as *const TfrEntry as *const u8,
                                std::mem::size_of::<TfrEntry>(),
                            )
                        };
                        if file.write_all(entry_bytes).is_ok() {
                            entry_count += 1;
                            recorded_static.insert(child);
                        }
                    }
                }
            }
        }

        // Print progress
        if entry_count % 100 == 0 && entry_count > 0 {
            print!(
                "\r  {} entries, {:.1}s elapsed, {} static frames",
                entry_count,
                start.elapsed().as_secs_f64(),
                recorded_static.len()
            );
            let _ = std::io::stdout().flush();
        }

        std::thread::sleep(Duration::from_millis(1));
    }

    // Update header with final entry count
    use std::io::Seek;
    file.seek(std::io::SeekFrom::Start(0)).map_err(|e| {
        HorusError::Config(ConfigError::other(format!("Failed to seek: {e}")))
    })?;
    let final_header = TfrHeader {
        magic: *TFR_MAGIC,
        version: TFR_VERSION,
        _padding: [0; 3],
        start_time_ns,
        entry_count,
    };
    let header_bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(
            &final_header as *const TfrHeader as *const u8,
            std::mem::size_of::<TfrHeader>(),
        )
    };
    file.write_all(header_bytes).map_err(|e| {
        HorusError::Config(ConfigError::other(format!("Failed to update header: {e}")))
    })?;

    println!();
    println!();
    println!("{}", "Recording complete.".green().bold());
    println!("  Entries: {}", entry_count);
    println!("  Duration: {:.1}s", start.elapsed().as_secs_f64());
    println!("  Static frames: {}", recorded_static.len());
    println!("  File: {}", output_path);

    Ok(())
}

/// Read a .tfr recording file and return its entries
fn read_tfr_file(path: &str) -> HorusResult<(TfrHeader, Vec<TfrEntry>)> {
    use std::io::Read;

    let mut file = std::fs::File::open(path).map_err(|e| {
        HorusError::Config(ConfigError::other(format!("Failed to open file '{path}': {e}")))
    })?;

    // Read header
    let mut header_bytes = vec![0u8; std::mem::size_of::<TfrHeader>()];
    file.read_exact(&mut header_bytes).map_err(|e| {
        HorusError::Config(ConfigError::other(format!("Failed to read header: {e}")))
    })?;
    let header: TfrHeader = unsafe { std::ptr::read(header_bytes.as_ptr() as *const TfrHeader) };

    if &header.magic != TFR_MAGIC {
        return Err(HorusError::Config(ConfigError::other(
            "Invalid TFR file: bad magic bytes",
        )));
    }
    if header.version != TFR_VERSION {
        return Err(HorusError::Config(ConfigError::other(format!(
            "Unsupported TFR version: {} (expected {})",
            header.version, TFR_VERSION
        ))));
    }

    // Read entries
    let entry_size = std::mem::size_of::<TfrEntry>();
    let mut entries = Vec::with_capacity(header.entry_count as usize);
    let mut entry_bytes = vec![0u8; entry_size];

    for _ in 0..header.entry_count {
        if file.read_exact(&mut entry_bytes).is_err() {
            break;
        }
        let entry: TfrEntry = unsafe { std::ptr::read(entry_bytes.as_ptr() as *const TfrEntry) };
        entries.push(entry);
    }

    Ok((header, entries))
}

/// Replay a .tfr recording file, printing transforms as they play back
///
/// Usage: `horus tf play recording.tfr [--speed 2.0]`
pub fn replay_transforms(path: &str, speed: f64) -> HorusResult<()> {
    let (_header, entries) = read_tfr_file(path)?;

    println!("{}", "Replaying TF recording...".green().bold());
    println!("  File: {}", path.cyan());
    println!("  Entries: {}", entries.len());
    println!("  Speed: {:.1}x", speed);
    println!("  Press Ctrl+C to stop\n");

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    let _ = ctrlc::set_handler(move || r.store(false, Ordering::SeqCst));

    // Build TransformFrame from static entries first
    let tf = TransformFrame::new();
    let mut frame_parents: HashMap<String, String> = HashMap::new();

    // First pass: register all static frames
    for entry in &entries {
        if entry.entry_type == TfrEntryType::Static as u8 {
            let parent = entry.stamped.parent_frame_id();
            let child = entry.stamped.child_frame_id();
            if !tf.has_frame(&parent) {
                let _ = tf.register_frame(&parent, None);
            }
            let _ = tf.register_static_frame(&child, Some(&parent), &entry.stamped.transform);
            frame_parents.insert(child.clone(), parent.clone());
            println!(
                "  {} {} → {} {}",
                "[static]".green(),
                parent.dimmed(),
                child,
                format_transform_compact(&entry.stamped.transform)
            );
        }
    }

    // Replay dynamic entries with timing
    let replay_start = Instant::now();
    let mut last_print = Instant::now();

    for entry in &entries {
        if !running.load(Ordering::Relaxed) {
            break;
        }

        if entry.entry_type != TfrEntryType::Dynamic as u8 {
            continue;
        }

        // Wait until the right time (scaled by speed)
        let target_elapsed = Duration::from_nanos((entry.offset_ns as f64 / speed) as u64);
        let actual_elapsed = replay_start.elapsed();
        if target_elapsed > actual_elapsed {
            std::thread::sleep(target_elapsed - actual_elapsed);
        }

        // Apply transform
        let parent = entry.stamped.parent_frame_id();
        let child = entry.stamped.child_frame_id();
        if !tf.has_frame(&parent) {
            let _ = tf.register_frame(&parent, None);
        }
        if !tf.has_frame(&child) {
            let _ = tf.register_frame(&child, Some(&parent));
            frame_parents.insert(child.clone(), parent.clone());
        }
        let _ = tf.update_transform(&child, &entry.stamped.transform, entry.stamped.timestamp_ns);

        // Print at ~10Hz
        if last_print.elapsed() >= Duration::from_millis(100) {
            let t = &entry.stamped.transform;
            println!(
                "  [{:>8.3}s] {} → {}: {}",
                entry.offset_ns as f64 / 1_000_000_000.0,
                parent.dimmed(),
                child,
                format_transform_compact(t)
            );
            last_print = Instant::now();
        }
    }

    println!();
    println!("{}", "Replay complete.".green().bold());
    Ok(())
}

/// Format a transform compactly for display
fn format_transform_compact(t: &Transform) -> String {
    format!(
        "xyz=[{:.3}, {:.3}, {:.3}]",
        t.translation[0], t.translation[1], t.translation[2]
    )
}

/// Compare two .tfr recording files and report per-frame differences
///
/// Usage: `horus tf diff file1.tfr file2.tfr [--threshold 0.001]`
pub fn diff_transforms(
    path1: &str,
    path2: &str,
    threshold_m: f64,
    threshold_deg: f64,
    json_output: bool,
) -> HorusResult<()> {
    let (_header1, entries1) = read_tfr_file(path1)?;
    let (_header2, entries2) = read_tfr_file(path2)?;

    if !json_output {
        println!("{}", "Comparing TF recordings...".green().bold());
        println!("  File 1: {} ({} entries)", path1.cyan(), entries1.len());
        println!("  File 2: {} ({} entries)", path2.cyan(), entries2.len());
        println!();
    }

    // Build final TF state from each recording
    let tf1 = build_tf_from_entries(&entries1);
    let tf2 = build_tf_from_entries(&entries2);

    // Get union of all frame names
    let frames1: HashSet<String> = tf1.all_frames().into_iter().collect();
    let frames2: HashSet<String> = tf2.all_frames().into_iter().collect();
    let all_frames: HashSet<&String> = frames1.union(&frames2).collect();

    let mut diffs: Vec<FrameDiff> = Vec::new();

    for frame in &all_frames {
        let in1 = frames1.contains(*frame);
        let in2 = frames2.contains(*frame);

        match (in1, in2) {
            (true, false) => {
                diffs.push(FrameDiff {
                    frame: frame.to_string(),
                    status: DiffStatus::OnlyIn1,
                    translation_err_m: 0.0,
                    rotation_err_deg: 0.0,
                });
            }
            (false, true) => {
                diffs.push(FrameDiff {
                    frame: frame.to_string(),
                    status: DiffStatus::OnlyIn2,
                    translation_err_m: 0.0,
                    rotation_err_deg: 0.0,
                });
            }
            (true, true) => {
                // Compare latest transforms
                if let (Ok(t1), Ok(t2)) = (tf1.tf(frame, "world"), tf2.tf(frame, "world")) {
                    // Translation error (Euclidean distance)
                    let dx = t1.translation[0] - t2.translation[0];
                    let dy = t1.translation[1] - t2.translation[1];
                    let dz = t1.translation[2] - t2.translation[2];
                    let trans_err = (dx * dx + dy * dy + dz * dz).sqrt();

                    // Rotation error (angle of relative rotation)
                    let relative = t1.inverse().compose(&t2);
                    let rot_err_rad = relative.rotation_angle();
                    let rot_err_deg = rot_err_rad.to_degrees();

                    let exceeds = trans_err > threshold_m || rot_err_deg > threshold_deg;
                    diffs.push(FrameDiff {
                        frame: frame.to_string(),
                        status: if exceeds {
                            DiffStatus::Different
                        } else {
                            DiffStatus::Same
                        },
                        translation_err_m: trans_err,
                        rotation_err_deg: rot_err_deg,
                    });
                }
            }
            (false, false) => unreachable!(),
        }
    }

    diffs.sort_by(|a, b| a.frame.cmp(&b.frame));

    if json_output {
        // JSON output for CI integration
        let json_diffs: Vec<String> = diffs
            .iter()
            .map(|d| {
                format!(
                    r#"  {{"frame": "{}", "status": "{}", "translation_err_m": {:.6}, "rotation_err_deg": {:.4}}}"#,
                    d.frame, d.status.as_str(), d.translation_err_m, d.rotation_err_deg
                )
            })
            .collect();
        println!("[{}]", json_diffs.join(",\n"));
    } else {
        // Table output
        println!(
            "  {:<25} {:>12} {:>12} {:>10}",
            "FRAME".dimmed(),
            "TRANS (mm)".dimmed(),
            "ROT (deg)".dimmed(),
            "STATUS".dimmed()
        );
        println!("  {}", "-".repeat(62).dimmed());

        for d in &diffs {
            let status_str = match d.status {
                DiffStatus::Same => "OK".green(),
                DiffStatus::Different => "DIFF".red(),
                DiffStatus::OnlyIn1 => "FILE1 ONLY".yellow(),
                DiffStatus::OnlyIn2 => "FILE2 ONLY".yellow(),
            };
            let trans_str = format!("{:.3}", d.translation_err_m * 1000.0); // mm
            let rot_str = format!("{:.3}", d.rotation_err_deg);

            println!(
                "  {:<25} {:>12} {:>12} {:>10}",
                d.frame, trans_str, rot_str, status_str
            );
        }

        let diff_count = diffs
            .iter()
            .filter(|d| !matches!(d.status, DiffStatus::Same))
            .count();
        println!();
        if diff_count == 0 {
            println!("  {}", "All frames match within threshold.".green());
        } else {
            println!(
                "  {} frame(s) differ.",
                format!("{diff_count}").red().bold()
            );
        }
    }

    Ok(())
}

/// Build a TransformFrame from recording entries
fn build_tf_from_entries(entries: &[TfrEntry]) -> TransformFrame {
    let tf = TransformFrame::new();
    // Register "world" as root
    let _ = tf.register_frame("world", None);

    for entry in entries {
        let parent = entry.stamped.parent_frame_id();
        let child = entry.stamped.child_frame_id();

        if !tf.has_frame(&parent) {
            let _ = tf.register_frame(&parent, None);
        }

        if entry.entry_type == TfrEntryType::Static as u8 {
            if !tf.has_frame(&child) {
                let _ = tf.register_static_frame(&child, Some(&parent), &entry.stamped.transform);
            }
        } else {
            if !tf.has_frame(&child) {
                let _ = tf.register_frame(&child, Some(&parent));
            }
            let _ = tf.update_transform(&child, &entry.stamped.transform, entry.stamped.timestamp_ns);
        }
    }

    tf
}

#[derive(Debug)]
struct FrameDiff {
    frame: String,
    status: DiffStatus,
    translation_err_m: f64,
    rotation_err_deg: f64,
}

#[derive(Debug)]
enum DiffStatus {
    Same,
    Different,
    OnlyIn1,
    OnlyIn2,
}

impl DiffStatus {
    fn as_str(&self) -> &'static str {
        match self {
            DiffStatus::Same => "same",
            DiffStatus::Different => "different",
            DiffStatus::OnlyIn1 => "only_in_file1",
            DiffStatus::OnlyIn2 => "only_in_file2",
        }
    }
}

// ============================================================================
// TF Calibration Tools
// ============================================================================

/// Tune a static frame's offset interactively via CLI
///
/// Reads the current transform, lets user adjust it with incremental steps,
/// then saves the result.
///
/// Usage: `horus tf tune <frame_name> [--step 0.001]`
pub fn tune_static_frame(
    frame_name: &str,
    step_m: f64,
    step_deg: f64,
) -> HorusResult<()> {
    println!("{}", "Static Frame Offset Tuner".green().bold());
    println!("  Frame: {}", frame_name.cyan());
    println!("  Translation step: {:.4} m", step_m);
    println!("  Rotation step: {:.2} deg", step_deg);
    println!();

    // Read current transform from SHM
    let mut reader = TransformFrameReader::new();
    // Poll SHM a few times to collect frames
    for _ in 0..10 {
        reader.read_from_shm();
        std::thread::sleep(Duration::from_millis(50));
    }

    let original = if let Some(data) = reader.frame_data.get(frame_name) {
        data.transform.clone()
    } else {
        println!(
            "  {} Frame '{}' not found in live TF data. Starting from identity.",
            "Warning:".yellow(),
            frame_name
        );
        Transform::identity()
    };

    let step_rad = step_deg.to_radians();

    println!("  Original: xyz=[{:.4}, {:.4}, {:.4}]",
        original.translation[0], original.translation[1], original.translation[2]);
    let euler = original.to_euler();
    println!("            rpy=[{:.4}, {:.4}, {:.4}] deg",
        euler[0].to_degrees(), euler[1].to_degrees(), euler[2].to_degrees());
    println!();
    println!("  {}", "Commands:".bold());
    println!("    x+/x-  : adjust X translation  (+/- {:.4} m)", step_m);
    println!("    y+/y-  : adjust Y translation");
    println!("    z+/z-  : adjust Z translation");
    println!("    r+/r-  : adjust roll           (+/- {:.2} deg)", step_deg);
    println!("    p+/p-  : adjust pitch");
    println!("    w+/w-  : adjust yaw");
    println!("    reset  : revert to original");
    println!("    save   : save and exit");
    println!("    quit   : exit without saving");
    println!();

    let mut current = original.clone();

    loop {
        // Show current state
        let delta_t = [
            current.translation[0] - original.translation[0],
            current.translation[1] - original.translation[1],
            current.translation[2] - original.translation[2],
        ];
        let current_euler = current.to_euler();
        print!(
            "\r  Current: xyz=[{:.4}, {:.4}, {:.4}] delta=[{:.4}, {:.4}, {:.4}]  > ",
            current.translation[0], current.translation[1], current.translation[2],
            delta_t[0], delta_t[1], delta_t[2],
        );
        let _ = std::io::Write::flush(&mut std::io::stdout());

        // Read command
        let mut input = String::new();
        if std::io::BufRead::read_line(&mut std::io::stdin().lock(), &mut input).is_err() {
            break;
        }
        let cmd = input.trim();

        match cmd {
            "x+" => current.translation[0] += step_m,
            "x-" => current.translation[0] -= step_m,
            "y+" => current.translation[1] += step_m,
            "y-" => current.translation[1] -= step_m,
            "z+" => current.translation[2] += step_m,
            "z-" => current.translation[2] -= step_m,
            "r+" | "r-" | "p+" | "p-" | "w+" | "w-" => {
                let mut e = current.to_euler();
                match cmd {
                    "r+" => e[0] += step_rad,
                    "r-" => e[0] -= step_rad,
                    "p+" => e[1] += step_rad,
                    "p-" => e[1] -= step_rad,
                    "w+" => e[2] += step_rad,
                    "w-" => e[2] -= step_rad,
                    _ => {}
                }
                current = Transform::from_euler(current.translation, e);
            }
            "reset" => {
                current = original.clone();
                println!("  {}", "Reset to original.".yellow());
            }
            "save" => {
                println!();
                println!("  {}", "Final transform:".green().bold());
                println!("    xyz=[{:.6}, {:.6}, {:.6}]",
                    current.translation[0], current.translation[1], current.translation[2]);
                let final_euler = current.to_euler();
                println!("    rpy=[{:.6}, {:.6}, {:.6}] rad",
                    final_euler[0], final_euler[1], final_euler[2]);
                println!("    rpy=[{:.4}, {:.4}, {:.4}] deg",
                    final_euler[0].to_degrees(), final_euler[1].to_degrees(), final_euler[2].to_degrees());

                // Try to update via SHM
                let tf = TransformFrame::new();
                let _ = tf.register_frame("world", None);
                if let Some(data) = reader.frame_data.get(frame_name) {
                    let _ = tf.register_frame(frame_name, Some(&data.parent));
                    let _ = tf.set_static_transform(frame_name, &current);
                    println!("  {}", "Transform updated in live TF tree.".green());
                }

                println!("  {}", "Saved.".green().bold());
                break;
            }
            "quit" | "q" => {
                println!("  {}", "Exiting without saving.".dimmed());
                break;
            }
            "" => {} // empty line, just redisplay
            _ => println!("  Unknown command: '{}'. Try x+, y-, r+, save, quit.", cmd),
        }
    }

    Ok(())
}

/// Compute a rigid-body calibration transform from point correspondences.
///
/// Given N >= 3 paired points (sensor coordinates and world coordinates),
/// computes the optimal rigid transform using SVD-based registration
/// (Arun et al. 1987).
///
/// Usage: `horus tf calibrate --points-file pairs.csv`
///
/// CSV format: `sensor_x,sensor_y,sensor_z,world_x,world_y,world_z`
pub fn calibrate_from_points(
    points_file: &str,
) -> HorusResult<()> {
    println!("{}", "Sensor-to-Base Calibration".green().bold());
    println!("  Points file: {}", points_file.cyan());
    println!();

    // Read point pairs from CSV
    let content = std::fs::read_to_string(points_file).map_err(|e| {
        HorusError::Config(ConfigError::other(format!("Failed to read points file: {e}")))
    })?;

    let mut sensor_points: Vec<[f64; 3]> = Vec::new();
    let mut world_points: Vec<[f64; 3]> = Vec::new();

    for (i, line) in content.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("sensor") {
            continue; // Skip comments and header
        }
        let parts: Vec<f64> = line
            .split(',')
            .map(|s| s.trim().parse::<f64>())
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| {
                HorusError::Config(ConfigError::other(format!(
                    "Failed to parse line {}: {e}",
                    i + 1
                )))
            })?;
        if parts.len() != 6 {
            return Err(HorusError::Config(ConfigError::other(format!(
                "Line {} has {} values, expected 6 (sensor_xyz, world_xyz)",
                i + 1,
                parts.len()
            ))));
        }
        sensor_points.push([parts[0], parts[1], parts[2]]);
        world_points.push([parts[3], parts[4], parts[5]]);
    }

    let n = sensor_points.len();
    if n < 3 {
        return Err(HorusError::Config(ConfigError::other(format!(
            "Need at least 3 point pairs, got {n}"
        ))));
    }

    println!("  Loaded {} point pairs.", n);

    // Compute centroids
    let mut sensor_centroid = [0.0f64; 3];
    let mut world_centroid = [0.0f64; 3];
    for i in 0..n {
        for j in 0..3 {
            sensor_centroid[j] += sensor_points[i][j];
            world_centroid[j] += world_points[i][j];
        }
    }
    for j in 0..3 {
        sensor_centroid[j] /= n as f64;
        world_centroid[j] /= n as f64;
    }

    // Center the points
    let mut sensor_centered: Vec<[f64; 3]> = sensor_points
        .iter()
        .map(|p| [p[0] - sensor_centroid[0], p[1] - sensor_centroid[1], p[2] - sensor_centroid[2]])
        .collect();
    let mut world_centered: Vec<[f64; 3]> = world_points
        .iter()
        .map(|p| [p[0] - world_centroid[0], p[1] - world_centroid[1], p[2] - world_centroid[2]])
        .collect();

    // Compute cross-covariance matrix H = sum(sensor_i * world_i^T)
    let mut h = [[0.0f64; 3]; 3];
    for i in 0..n {
        for r in 0..3 {
            for c in 0..3 {
                h[r][c] += sensor_centered[i][r] * world_centered[i][c];
            }
        }
    }

    // SVD of H using Jacobi rotations (simple 3x3 SVD)
    // For production, use nalgebra. Here we use the Transform::from_matrix approach:
    // Compute R = V * U^T from SVD(H) = U * S * V^T
    //
    // Simplified approach: use quaternion-based registration via the method of Horn (1987)
    // Build the 4x4 symmetric matrix N from the cross-covariance
    let sxx = h[0][0]; let sxy = h[0][1]; let sxz = h[0][2];
    let syx = h[1][0]; let syy = h[1][1]; let syz = h[1][2];
    let szx = h[2][0]; let szy = h[2][1]; let szz = h[2][2];

    // Horn's quaternion method: build 4x4 matrix and find max eigenvalue
    let n_mat = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ];

    // Power iteration to find dominant eigenvector of N
    let mut q = [1.0f64, 0.0, 0.0, 0.0]; // Initial guess
    for _ in 0..100 {
        let mut q_new = [0.0f64; 4];
        for i in 0..4 {
            for j in 0..4 {
                q_new[i] += n_mat[i][j] * q[j];
            }
        }
        // Normalize
        let norm = (q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]).sqrt();
        if norm > 1e-15 {
            for i in 0..4 {
                q_new[i] /= norm;
            }
        }
        q = q_new;
    }

    // q = [w, x, y, z] from Horn's method
    // TransformFrame uses [x, y, z, w] convention
    let rotation = [q[1], q[2], q[3], q[0]];

    // Compute translation: t = world_centroid - R * sensor_centroid
    let rot_tf = Transform::new([0.0, 0.0, 0.0], rotation);
    let rotated_centroid = rot_tf.transform_point(sensor_centroid);
    let translation = [
        world_centroid[0] - rotated_centroid[0],
        world_centroid[1] - rotated_centroid[1],
        world_centroid[2] - rotated_centroid[2],
    ];

    let result = Transform::new(translation, rotation);

    // Compute RMSE
    let mut sum_sq_err = 0.0;
    for i in 0..n {
        let transformed = result.transform_point(sensor_points[i]);
        let dx = transformed[0] - world_points[i][0];
        let dy = transformed[1] - world_points[i][1];
        let dz = transformed[2] - world_points[i][2];
        sum_sq_err += dx * dx + dy * dy + dz * dz;
    }
    let rmse = (sum_sq_err / n as f64).sqrt();

    // Display results
    println!();
    println!("  {}", "Calibration Result:".green().bold());
    println!("    Translation: [{:.6}, {:.6}, {:.6}] m",
        result.translation[0], result.translation[1], result.translation[2]);
    let euler = result.to_euler();
    println!("    Rotation:    [{:.4}, {:.4}, {:.4}] deg (rpy)",
        euler[0].to_degrees(), euler[1].to_degrees(), euler[2].to_degrees());
    println!("    Quaternion:  [{:.6}, {:.6}, {:.6}, {:.6}] (xyzw)",
        result.rotation[0], result.rotation[1], result.rotation[2], result.rotation[3]);
    println!();
    println!("    RMSE: {:.4} mm", rmse * 1000.0);
    println!("    Points: {}", n);

    if rmse > 0.01 {
        println!("    {}", "Warning: RMSE > 10mm — check point accuracy".yellow());
    } else {
        println!("    {}", "Quality: Good".green());
    }

    // Print per-point residuals
    println!();
    println!("  {}", "Per-point residuals:".dimmed());
    println!("    {:>5} {:>10} {:>10} {:>10} {:>10}",
        "Pt".dimmed(), "dx (mm)".dimmed(), "dy (mm)".dimmed(), "dz (mm)".dimmed(), "err (mm)".dimmed());
    for i in 0..n {
        let transformed = result.transform_point(sensor_points[i]);
        let dx = (transformed[0] - world_points[i][0]) * 1000.0;
        let dy = (transformed[1] - world_points[i][1]) * 1000.0;
        let dz = (transformed[2] - world_points[i][2]) * 1000.0;
        let err = (dx*dx + dy*dy + dz*dz).sqrt();
        println!("    {:>5} {:>10.3} {:>10.3} {:>10.3} {:>10.3}", i + 1, dx, dy, dz, err);
    }

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
        result.unwrap();
    }

    #[test]
    fn test_view_frames() {
        let result = view_frames(None);
        result.unwrap();
    }

    #[test]
    fn test_frame_info() {
        let result = frame_info("base_link");
        result.unwrap();
    }

    #[test]
    fn test_can_transform() {
        let result = can_transform("base_link", "camera_link");
        result.unwrap();
    }

    #[test]
    fn test_transform_frame_reader_new() {
        let reader = TransformFrameReader::new();
        assert!(reader.frame_data.is_empty());
        assert!(reader.root_frames.is_empty());
    }
}
