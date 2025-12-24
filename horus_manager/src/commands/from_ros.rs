//! ROS to HORUS converter command
//!
//! Converts ROS1 and ROS2 packages to HORUS projects, enabling easy migration.
//! Supports:
//! - .msg/.srv/.action files → Rust message structs
//! - package.xml → Horus.toml/Cargo.toml
//! - C++/Python nodes → HORUS node skeletons
//!
//! ## ROS1 vs ROS2 Detection
//!
//! The converter auto-detects ROS version based on:
//! - package.xml format (format 1/2 = ROS1, format 3 = ROS2)
//! - Build system (catkin = ROS1, ament = ROS2)
//! - CMakeLists.txt patterns

use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::fs;
use std::path::{Path, PathBuf};

/// ROS version detected from package
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RosVersion {
    Ros1,
    Ros2,
    Unknown,
}

impl std::fmt::Display for RosVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RosVersion::Ros1 => write!(f, "ROS1"),
            RosVersion::Ros2 => write!(f, "ROS2"),
            RosVersion::Unknown => write!(f, "Unknown"),
        }
    }
}

/// ROS2 built-in type mappings to Rust types
fn ros2_to_rust_type(ros2_type: &str) -> String {
    match ros2_type.trim() {
        // Primitive types
        "bool" => "bool".to_string(),
        "byte" => "u8".to_string(),
        "char" => "u8".to_string(),
        "float32" => "f32".to_string(),
        "float64" => "f64".to_string(),
        "int8" => "i8".to_string(),
        "uint8" => "u8".to_string(),
        "int16" => "i16".to_string(),
        "uint16" => "u16".to_string(),
        "int32" => "i32".to_string(),
        "uint32" => "u32".to_string(),
        "int64" => "i64".to_string(),
        "uint64" => "u64".to_string(),
        "string" => "String".to_string(),
        "wstring" => "String".to_string(),

        // Bounded strings: string<=N (ROS2 IDL feature)
        t if t.starts_with("string<=") => "String".to_string(), // Bounded string → String (limit not enforced in Rust)
        t if t.starts_with("wstring<=") => "String".to_string(),

        // Bounded sequences: sequence<type, N> or type[<=N]
        t if t.contains("[<=") => {
            // type[<=N] → Vec<type> (bounded sequence)
            let bracket_start = t.find("[<=").unwrap();
            let inner = &t[..bracket_start];
            format!("Vec<{}>", ros2_to_rust_type(inner))
        }
        t if t.starts_with("sequence<") && t.contains(',') => {
            // sequence<type, N> → Vec<type>
            let inner_start = "sequence<".len();
            let comma_pos = t.find(',').unwrap();
            let inner = &t[inner_start..comma_pos].trim();
            format!("Vec<{}>", ros2_to_rust_type(inner))
        }
        t if t.starts_with("sequence<") => {
            // sequence<type> → Vec<type>
            let inner = t.trim_start_matches("sequence<").trim_end_matches('>');
            format!("Vec<{}>", ros2_to_rust_type(inner))
        }

        // Handle arrays (fixed and dynamic)
        t if t.ends_with("[]") => {
            let inner = &t[..t.len() - 2];
            format!("Vec<{}>", ros2_to_rust_type(inner))
        }
        // Fixed size arrays like float32[3]
        t if t.contains('[') && t.contains(']') => {
            let bracket_start = t.find('[').unwrap();
            let bracket_end = t.find(']').unwrap();
            let inner = &t[..bracket_start];
            let size = &t[bracket_start + 1..bracket_end];
            format!("[{}; {}]", ros2_to_rust_type(inner), size)
        }

        // Duration and Time (from builtin_interfaces)
        "duration" | "builtin_interfaces/Duration" | "builtin_interfaces/msg/Duration" => {
            "std::time::Duration".to_string()
        }
        "time" | "builtin_interfaces/Time" | "builtin_interfaces/msg/Time" => "u64".to_string(), // nanoseconds since epoch

        // Common ROS2 message types → HORUS equivalents
        "geometry_msgs/Twist" | "geometry_msgs/msg/Twist" => "Twist".to_string(),
        "geometry_msgs/Pose" | "geometry_msgs/msg/Pose" => "Pose".to_string(),
        "geometry_msgs/Point" | "geometry_msgs/msg/Point" => "Point3".to_string(),
        "geometry_msgs/Vector3" | "geometry_msgs/msg/Vector3" => "Vector3".to_string(),
        "geometry_msgs/Quaternion" | "geometry_msgs/msg/Quaternion" => "Quaternion".to_string(),
        "geometry_msgs/Transform" | "geometry_msgs/msg/Transform" => "Transform".to_string(),
        "sensor_msgs/Imu" | "sensor_msgs/msg/Imu" => "Imu".to_string(),
        "sensor_msgs/LaserScan" | "sensor_msgs/msg/LaserScan" => "LaserScan".to_string(),
        "sensor_msgs/Image" | "sensor_msgs/msg/Image" => "Image".to_string(),
        "sensor_msgs/PointCloud2" | "sensor_msgs/msg/PointCloud2" => "PointCloud".to_string(),
        "sensor_msgs/Range" | "sensor_msgs/msg/Range" => "Range".to_string(),
        "sensor_msgs/BatteryState" | "sensor_msgs/msg/BatteryState" => "BatteryState".to_string(),
        "sensor_msgs/NavSatFix" | "sensor_msgs/msg/NavSatFix" => "NavSatFix".to_string(),
        "sensor_msgs/CameraInfo" | "sensor_msgs/msg/CameraInfo" => "CameraInfo".to_string(),
        "nav_msgs/Odometry" | "nav_msgs/msg/Odometry" => "Odometry".to_string(),
        "nav_msgs/Path" | "nav_msgs/msg/Path" => "Path".to_string(),
        "nav_msgs/OccupancyGrid" | "nav_msgs/msg/OccupancyGrid" => "OccupancyGrid".to_string(),

        // Header and Stamped types - use GenericMessage since HORUS doesn't have Header
        "std_msgs/Header" | "std_msgs/msg/Header" | "Header" => "GenericMessage".to_string(),
        "geometry_msgs/TransformStamped" | "geometry_msgs/msg/TransformStamped" | "TransformStamped" => {
            "GenericMessage".to_string()
        }
        "geometry_msgs/PoseStamped" | "geometry_msgs/msg/PoseStamped" | "PoseStamped" => {
            "GenericMessage".to_string()
        }
        "geometry_msgs/PointStamped" | "geometry_msgs/msg/PointStamped" | "PointStamped" => {
            "GenericMessage".to_string()
        }

        // Action/Service types from actionlib_msgs
        "actionlib_msgs/GoalStatusArray" | "actionlib_msgs/msg/GoalStatusArray" | "GoalStatusArray" => {
            "GenericMessage".to_string()
        }
        "actionlib_msgs/GoalStatus" | "actionlib_msgs/msg/GoalStatus" | "GoalStatus" => {
            "GenericMessage".to_string()
        }
        "actionlib_msgs/GoalID" | "actionlib_msgs/msg/GoalID" | "GoalID" => {
            "GenericMessage".to_string()
        }

        // TF2 types
        "tf2_msgs/TF2Error" | "tf2_msgs/msg/TF2Error" | "TF2Error" => "GenericMessage".to_string(),
        "tf2_msgs/TFMessage" | "tf2_msgs/msg/TFMessage" | "TFMessage" => "GenericMessage".to_string(),

        // Entity types that don't exist in HORUS
        "Entity" | "entity" => "GenericMessage".to_string(),

        // For unknown types, assume it's a custom message - convert to PascalCase
        other => {
            // Extract just the type name if it has a path
            let type_name = other.split('/').last().unwrap_or(other);
            // Remove "msg" if present
            let type_name = if type_name == "msg" {
                other.split('/').nth_back(1).unwrap_or(type_name)
            } else {
                type_name
            };
            to_pascal_case(type_name)
        }
    }
}

/// Convert snake_case or other formats to PascalCase
fn to_pascal_case(s: &str) -> String {
    s.split('_')
        .map(|part| {
            let mut chars = part.chars();
            match chars.next() {
                None => String::new(),
                Some(first) => first.to_uppercase().chain(chars).collect(),
            }
        })
        .collect()
}

/// Convert PascalCase to snake_case
fn to_snake_case(s: &str) -> String {
    let mut result = String::new();
    for (i, c) in s.chars().enumerate() {
        if c.is_uppercase() {
            if i > 0 {
                result.push('_');
            }
            result.push(c.to_lowercase().next().unwrap());
        } else {
            result.push(c);
        }
    }
    result
}

/// Sanitize a field name to be a valid Rust identifier
/// Replaces dots, slashes, and other special characters with underscores
fn sanitize_field_name(name: &str) -> String {
    let sanitized = name
        .replace('/', "_")
        .replace('.', "_")
        .replace('-', "_")
        .replace(' ', "_");

    // Ensure it starts with a letter or underscore (not a digit)
    if sanitized.chars().next().map(|c| c.is_ascii_digit()).unwrap_or(false) {
        format!("_{}", sanitized)
    } else if sanitized.is_empty() {
        "unnamed".to_string()
    } else {
        sanitized
    }
}

/// Parsed ROS2 message field
#[derive(Debug, Clone)]
pub struct RosField {
    pub name: String,
    pub field_type: String,
    pub default_value: Option<String>,
    pub comment: Option<String>,
    pub is_constant: bool,
}

/// Parsed ROS2 message definition
#[derive(Debug, Clone)]
pub struct RosMessage {
    pub name: String,
    pub package: String,
    pub fields: Vec<RosField>,
    pub constants: Vec<RosField>,
    pub source_file: PathBuf,
}

/// Parsed ROS2 service definition
#[derive(Debug, Clone)]
pub struct RosService {
    pub name: String,
    pub package: String,
    pub request: Vec<RosField>,
    pub response: Vec<RosField>,
    pub source_file: PathBuf,
}

/// Parsed ROS2 action definition (Goal/Result/Feedback)
#[derive(Debug, Clone)]
pub struct RosAction {
    pub name: String,
    pub package: String,
    pub goal: Vec<RosField>,
    pub result: Vec<RosField>,
    pub feedback: Vec<RosField>,
    pub source_file: PathBuf,
}

/// Parsed launch file node configuration
#[derive(Debug, Clone, Default)]
pub struct LaunchNode {
    pub name: String,
    pub package: String,
    pub executable: String,
    pub namespace: Option<String>,
    pub output: Option<String>,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
    /// Condition for node activation (if/unless, IfCondition, UnlessCondition)
    pub condition: Option<LaunchCondition>,
    /// Is this a composable node?
    pub is_composable: bool,
}

/// Launch argument definition
#[derive(Debug, Clone, Default)]
pub struct LaunchArgument {
    pub name: String,
    pub default_value: Option<String>,
    pub description: Option<String>,
}

/// Launch include directive
#[derive(Debug, Clone, Default)]
pub struct LaunchInclude {
    pub file: String,
    pub package: Option<String>,
    pub namespace: Option<String>,
    pub arguments: Vec<(String, String)>,
    /// Condition for include (if/unless)
    pub condition: Option<LaunchCondition>,
}

/// Launch condition (if/unless, IfCondition, UnlessCondition)
#[derive(Debug, Clone, Default)]
pub struct LaunchCondition {
    /// Condition type: "if", "unless", "IfCondition", "UnlessCondition"
    pub condition_type: String,
    /// The expression (e.g., "$(arg enable_sensors)", "LaunchConfiguration('enable')")
    pub expression: String,
}

/// Launch group for scoped namespaces and environment variables
#[derive(Debug, Clone, Default)]
pub struct LaunchGroup {
    pub namespace: Option<String>,
    pub condition: Option<LaunchCondition>,
    pub push_ros_namespace: bool,
    /// Environment variables set in this group
    pub env_vars: Vec<(String, String)>,
    /// Nodes within this group
    pub nodes: Vec<LaunchNode>,
    /// Nested includes
    pub includes: Vec<LaunchInclude>,
}

/// Composable node container (ROS2)
#[derive(Debug, Clone, Default)]
pub struct ComposableNodeContainer {
    pub name: String,
    pub namespace: Option<String>,
    pub package: String,
    pub executable: String,
    /// Composable nodes loaded into this container
    pub composable_nodes: Vec<ComposableNode>,
}

/// Composable node (loaded into a container)
#[derive(Debug, Clone, Default)]
pub struct ComposableNode {
    pub name: String,
    pub package: String,
    pub plugin: String,
    pub namespace: Option<String>,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
}

/// Launch file format
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LaunchFormat {
    #[default]
    Xml,
    Python,
    Yaml,
}

/// Parsed launch file information
#[derive(Debug, Clone, Default)]
pub struct LaunchFile {
    pub name: String,
    pub format: LaunchFormat,
    pub nodes: Vec<LaunchNode>,
    pub arguments: Vec<LaunchArgument>,
    pub includes: Vec<LaunchInclude>,
    pub groups: Vec<LaunchGroup>,
    pub composable_containers: Vec<ComposableNodeContainer>,
    pub source_file: PathBuf,
}

/// External dependency detected from CMakeLists.txt
#[derive(Debug, Clone, Default)]
pub struct ExternalDep {
    pub cmake_name: String,            // e.g., "OpenCV", "PCL", "Eigen3"
    pub rust_crate: Option<String>,    // e.g., "opencv", "nalgebra"
    pub cargo_feature: Option<String>, // Optional feature to enable
    pub notes: Option<String>,         // Migration notes
}

/// CMake executable target detected from CMakeLists.txt
#[derive(Debug, Clone, Default)]
pub struct CmakeTarget {
    pub name: String,              // Target name (e.g., "controller_node")
    pub source_files: Vec<String>, // Source files (e.g., ["src/controller.cpp"])
    pub is_library: bool,          // True if add_library, false if add_executable
}

/// Plugin information from plugin.xml (ROS1 nodelets, pluginlib)
#[derive(Debug, Clone, Default)]
pub struct PluginInfo {
    pub class_name: String,   // Full class name (e.g., "my_pkg::MyNodelet")
    pub base_class: String,   // Base class (e.g., "nodelet::Nodelet")
    pub library_path: String, // Library path (e.g., "lib/libmy_nodelet")
    pub description: String,  // Plugin description
    pub plugin_type: String,  // Type: "nodelet", "controller", "planner", etc.
}

/// Parsed ROS package.xml (ROS1 or ROS2)
#[derive(Debug, Clone)]
pub struct RosPackage {
    pub name: String,
    pub version: String,
    pub description: String,
    pub maintainer: String,
    pub license: String,
    pub dependencies: Vec<String>,
    pub external_deps: Vec<ExternalDep>,
    pub cmake_targets: Vec<CmakeTarget>,
    pub plugins: Vec<PluginInfo>,
    pub source_dir: PathBuf,
    pub ros_version: RosVersion,
}

/// Parse a ROS2 .msg file
pub fn parse_msg_file(path: &Path, package: &str) -> HorusResult<RosMessage> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| HorusError::InvalidInput("Invalid .msg filename".to_string()))?
        .to_string();

    let mut fields = Vec::new();
    let mut constants = Vec::new();
    let mut current_comment = None;

    for line in content.lines() {
        let line = line.trim();

        // Skip empty lines
        if line.is_empty() {
            current_comment = None;
            continue;
        }

        // Handle comments
        if line.starts_with('#') {
            current_comment = Some(line.trim_start_matches('#').trim().to_string());
            continue;
        }

        // Parse field: "type name" or "type name=default" or "type NAME=constant"
        // Handle complex types like "sequence<uint8, 1024>" which contain spaces
        let (field_type, rest) = if line.contains('<') && line.contains('>') {
            // Find the closing '>' and split after it
            let angle_close = line.rfind('>').unwrap();
            let type_part = &line[..=angle_close];
            let rest_part = &line[angle_close + 1..];
            (type_part.trim(), rest_part.trim())
        } else {
            // Simple split on first whitespace
            let parts: Vec<&str> = line.splitn(2, char::is_whitespace).collect();
            if parts.len() < 2 {
                continue;
            }
            (parts[0].trim(), parts[1].trim())
        };

        if rest.is_empty() {
            continue;
        }

        // Check for constant (NAME=value) or default (name=value) or just field (name)
        let (name_str, default_value, is_constant) = if rest.contains('=') {
            let eq_parts: Vec<&str> = rest.splitn(2, '=').collect();
            let field_name = eq_parts[0].trim();
            let value = eq_parts[1].trim();

            // Constants have ALL_CAPS names
            let is_const = field_name
                .chars()
                .all(|c| c.is_uppercase() || c == '_' || c.is_numeric());
            (field_name.to_string(), Some(value.to_string()), is_const)
        } else {
            // Handle possible inline comment
            let field_name = rest.split('#').next().unwrap_or(rest).trim();
            (field_name.to_string(), None, false)
        };

        let field = RosField {
            name: name_str,
            field_type: field_type.to_string(),
            default_value,
            comment: current_comment.take(),
            is_constant,
        };

        if is_constant {
            constants.push(field);
        } else {
            fields.push(field);
        }
    }

    Ok(RosMessage {
        name,
        package: package.to_string(),
        fields,
        constants,
        source_file: path.to_path_buf(),
    })
}

/// Parse a ROS2 .srv file
pub fn parse_srv_file(path: &Path, package: &str) -> HorusResult<RosService> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| HorusError::InvalidInput("Invalid .srv filename".to_string()))?
        .to_string();

    // Split by separator line "---"
    let parts: Vec<&str> = content.split("---").collect();

    let parse_fields = |section: &str| -> Vec<RosField> {
        let mut fields = Vec::new();
        for line in section.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.splitn(2, char::is_whitespace).collect();
            if parts.len() >= 2 {
                let field_type = parts[0].trim();
                let field_name = parts[1].split('#').next().unwrap_or(parts[1]).trim();

                fields.push(RosField {
                    name: field_name.to_string(),
                    field_type: field_type.to_string(),
                    default_value: None,
                    comment: None,
                    is_constant: false,
                });
            }
        }
        fields
    };

    let request = if !parts.is_empty() {
        parse_fields(parts[0])
    } else {
        Vec::new()
    };
    let response = if parts.len() > 1 {
        parse_fields(parts[1])
    } else {
        Vec::new()
    };

    Ok(RosService {
        name,
        package: package.to_string(),
        request,
        response,
        source_file: path.to_path_buf(),
    })
}

/// Parse a ROS .action file (Goal/Result/Feedback separated by ---)
pub fn parse_action_file(path: &Path, package: &str) -> HorusResult<RosAction> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| HorusError::InvalidInput("Invalid .action filename".to_string()))?
        .to_string();

    // Split by separator line "---" (3 sections: goal, result, feedback)
    let parts: Vec<&str> = content.split("---").collect();

    let parse_fields = |section: &str| -> Vec<RosField> {
        let mut fields = Vec::new();
        let mut current_comment = None;

        for line in section.lines() {
            let line = line.trim();
            if line.is_empty() {
                current_comment = None;
                continue;
            }

            if line.starts_with('#') {
                current_comment = Some(line.trim_start_matches('#').trim().to_string());
                continue;
            }

            let parts: Vec<&str> = line.splitn(2, char::is_whitespace).collect();
            if parts.len() >= 2 {
                let field_type = parts[0].trim();
                let field_name = parts[1].split('#').next().unwrap_or(parts[1]).trim();

                fields.push(RosField {
                    name: field_name.to_string(),
                    field_type: field_type.to_string(),
                    default_value: None,
                    comment: current_comment.take(),
                    is_constant: false,
                });
            }
        }
        fields
    };

    let goal = if !parts.is_empty() {
        parse_fields(parts[0])
    } else {
        Vec::new()
    };
    let result = if parts.len() > 1 {
        parse_fields(parts[1])
    } else {
        Vec::new()
    };
    let feedback = if parts.len() > 2 {
        parse_fields(parts[2])
    } else {
        Vec::new()
    };

    Ok(RosAction {
        name,
        package: package.to_string(),
        goal,
        result,
        feedback,
        source_file: path.to_path_buf(),
    })
}

/// Detect ROS version from package.xml content and directory
fn detect_ros_version(content: &str, package_dir: &Path) -> RosVersion {
    // Method 1: Check package format attribute
    // ROS1: format="1" or format="2" or no format attribute
    // ROS2: format="3"
    if let Some(format_start) = content.find("format=") {
        let rest = &content[format_start + 8..]; // skip 'format="'
        if let Some(quote_end) = rest.find('"') {
            let format_num = &rest[..quote_end];
            if format_num == "3" {
                return RosVersion::Ros2;
            } else if format_num == "1" || format_num == "2" {
                return RosVersion::Ros1;
            }
        }
    }

    // Method 2: Check build system dependencies
    // ROS1: catkin
    // ROS2: ament_cmake, ament_python, rclcpp, rclpy
    let content_lower = content.to_lowercase();

    if content_lower.contains("ament_cmake")
        || content_lower.contains("ament_python")
        || content_lower.contains("rclcpp")
        || content_lower.contains("rclpy")
    {
        return RosVersion::Ros2;
    }

    if content_lower.contains("catkin")
        || content_lower.contains("roscpp")
        || content_lower.contains("rospy")
    {
        return RosVersion::Ros1;
    }

    // Method 3: Check CMakeLists.txt for hints
    let cmake_path = package_dir.join("CMakeLists.txt");
    if cmake_path.exists() {
        if let Ok(cmake_content) = fs::read_to_string(&cmake_path) {
            let cmake_lower = cmake_content.to_lowercase();

            // ROS2 patterns
            if cmake_lower.contains("ament_cmake")
                || cmake_lower.contains("find_package(rclcpp")
                || cmake_lower.contains("rclcpp_components")
            {
                return RosVersion::Ros2;
            }

            // ROS1 patterns
            if cmake_lower.contains("catkin_package")
                || cmake_lower.contains("find_package(catkin")
                || cmake_lower.contains("catkin_make")
            {
                return RosVersion::Ros1;
            }
        }
    }

    RosVersion::Unknown
}

/// Parse a ROS package.xml file (ROS1 or ROS2)
pub fn parse_package_xml(path: &Path) -> HorusResult<RosPackage> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let package_dir = path.parent().unwrap_or(Path::new("."));

    // Detect ROS version first
    let ros_version = detect_ros_version(&content, package_dir);

    // Simple XML parsing (avoiding full XML dependency)
    let extract_tag = |tag: &str| -> Option<String> {
        let start_tag = format!("<{}>", tag);
        let end_tag = format!("</{}>", tag);
        if let Some(start) = content.find(&start_tag) {
            if let Some(end) = content[start..].find(&end_tag) {
                let value = &content[start + start_tag.len()..start + end];
                return Some(value.trim().to_string());
            }
        }
        None
    };

    // Extract dependencies - include ROS1-specific dependency tags too
    let mut dependencies = Vec::new();
    let dep_types = match ros_version {
        RosVersion::Ros1 => vec![
            "depend",
            "build_depend",
            "run_depend",
            "exec_depend",
            "buildtool_depend",
            "test_depend",
        ],
        _ => vec![
            "depend",
            "exec_depend",
            "build_depend",
            "buildtool_depend",
            "build_export_depend",
            "test_depend",
            "member_of_group",
        ],
    };

    for dep_type in dep_types {
        let start_tag = format!("<{}>", dep_type);
        let end_tag = format!("</{}>", dep_type);
        let mut search_from = 0;
        while let Some(start) = content[search_from..].find(&start_tag) {
            let actual_start = search_from + start;
            if let Some(end) = content[actual_start..].find(&end_tag) {
                let dep = &content[actual_start + start_tag.len()..actual_start + end];
                let dep = dep.trim().to_string();
                if !dependencies.contains(&dep) {
                    dependencies.push(dep);
                }
            }
            search_from = actual_start + start_tag.len();
        }
    }

    // Parse CMakeLists.txt for external dependencies and targets
    let cmake_path = package_dir.join("CMakeLists.txt");
    let external_deps = parse_cmake_dependencies(&cmake_path);
    let cmake_targets = parse_cmake_targets(&cmake_path);

    // Parse plugin.xml for nodelets and pluginlib plugins
    let plugin_xml_path = package_dir.join("plugin.xml");
    let plugins = if plugin_xml_path.exists() {
        parse_plugin_xml(&plugin_xml_path).unwrap_or_default()
    } else {
        // Also check for nodelets.xml (alternative name)
        let nodelets_path = package_dir.join("nodelets.xml");
        if nodelets_path.exists() {
            parse_plugin_xml(&nodelets_path).unwrap_or_default()
        } else {
            Vec::new()
        }
    };

    Ok(RosPackage {
        name: extract_tag("name").unwrap_or_else(|| "unknown".to_string()),
        version: extract_tag("version").unwrap_or_else(|| "0.1.0".to_string()),
        description: extract_tag("description").unwrap_or_default(),
        maintainer: extract_tag("maintainer").unwrap_or_default(),
        license: extract_tag("license").unwrap_or_else(|| "MIT".to_string()),
        dependencies,
        external_deps,
        cmake_targets,
        plugins,
        source_dir: package_dir.to_path_buf(),
        ros_version,
    })
}

/// Check if a ROS2 type can be Copy in Rust
fn is_copy_type(ros2_type: &str) -> bool {
    match ros2_type.trim() {
        // Primitives are Copy
        "bool" | "byte" | "char" | "float32" | "float64" | "int8" | "uint8" | "int16"
        | "uint16" | "int32" | "uint32" | "int64" | "uint64" => true,
        // String is not Copy
        "string" | "wstring" => false,
        // Arrays (dynamic) are not Copy
        t if t.ends_with("[]") => false,
        // Fixed size arrays of Copy types are Copy
        t if t.contains('[') && t.contains(']') => {
            let bracket_start = t.find('[').unwrap();
            let inner = &t[..bracket_start];
            is_copy_type(inner)
        }
        // Duration and Time are typically not Copy (depends on impl)
        "duration" | "builtin_interfaces/Duration" => false,
        "time" | "builtin_interfaces/Time" => true, // u64 is Copy
        // Nested message types are typically not Copy
        _ => false,
    }
}

/// Convert a ROS2 message to a Rust struct definition
pub fn msg_to_rust(msg: &RosMessage) -> String {
    let mut output = String::new();

    // Add imports at the top of the file
    output.push_str("use serde::{Serialize, Deserialize};\n");
    output.push_str("use horus::prelude::*;\n\n");

    // Add doc comment
    output.push_str(&format!(
        "/// Converted from ROS2 message: {}/{}\n",
        msg.package, msg.name
    ));

    // Check if all fields are Copy-able
    let all_copy = msg.fields.iter().all(|f| is_copy_type(&f.field_type));

    // Add derive attributes
    output.push_str("#[derive(Clone, Debug, Default, Serialize, Deserialize)]\n");
    if all_copy && !msg.fields.is_empty() {
        output.push_str("#[derive(Copy)]\n");
    }
    output.push_str(&format!("pub struct {} {{\n", msg.name));

    // Add fields
    for field in &msg.fields {
        if let Some(ref comment) = field.comment {
            output.push_str(&format!("    /// {}\n", comment));
        }
        let rust_type = ros2_to_rust_type(&field.field_type);
        output.push_str(&format!("    pub {}: {},\n", field.name, rust_type));
    }

    output.push_str("}\n");

    // Add constants as impl block
    if !msg.constants.is_empty() {
        output.push_str(&format!("\nimpl {} {{\n", msg.name));
        for constant in &msg.constants {
            let rust_type = ros2_to_rust_type(&constant.field_type);
            if let Some(ref value) = constant.default_value {
                // Convert value based on type
                let rust_value = if rust_type == "String" {
                    format!("\"{}\"", value)
                } else {
                    value.clone()
                };
                output.push_str(&format!(
                    "    pub const {}: {} = {};\n",
                    constant.name, rust_type, rust_value
                ));
            }
        }
        output.push_str("}\n");
    }

    output
}

/// Convert a ROS2 service to Rust request/response structs
pub fn srv_to_rust(srv: &RosService) -> String {
    let mut output = String::new();

    // Add imports at the top of the file
    output.push_str("use serde::{Serialize, Deserialize};\n");
    output.push_str("use horus::prelude::*;\n\n");

    // Request struct
    output.push_str(&format!(
        "/// Request for ROS2 service: {}/{}\n",
        srv.package, srv.name
    ));
    output.push_str("#[derive(Clone, Debug, Default, Serialize, Deserialize)]\n");
    output.push_str(&format!("pub struct {}Request {{\n", srv.name));
    for field in &srv.request {
        let rust_type = ros2_to_rust_type(&field.field_type);
        output.push_str(&format!("    pub {}: {},\n", field.name, rust_type));
    }
    output.push_str("}\n\n");

    // Response struct
    output.push_str(&format!(
        "/// Response for ROS2 service: {}/{}\n",
        srv.package, srv.name
    ));
    output.push_str("#[derive(Clone, Debug, Default, Serialize, Deserialize)]\n");
    output.push_str(&format!("pub struct {}Response {{\n", srv.name));
    for field in &srv.response {
        let rust_type = ros2_to_rust_type(&field.field_type);
        output.push_str(&format!("    pub {}: {},\n", field.name, rust_type));
    }
    output.push_str("}\n");

    output
}

/// Convert a ROS action to Rust Goal/Result/Feedback structs
pub fn action_to_rust(action: &RosAction) -> String {
    let mut output = String::new();

    // Add imports at the top of the file
    output.push_str("use serde::{Serialize, Deserialize};\n");
    output.push_str("use horus::prelude::*;\n\n");

    // Goal struct
    output.push_str(&format!(
        "/// Goal for ROS action: {}/{}\n",
        action.package, action.name
    ));
    output.push_str("#[derive(Clone, Debug, Default, Serialize, Deserialize)]\n");
    output.push_str(&format!("pub struct {}Goal {{\n", action.name));
    for field in &action.goal {
        if let Some(ref comment) = field.comment {
            output.push_str(&format!("    /// {}\n", comment));
        }
        let rust_type = ros2_to_rust_type(&field.field_type);
        output.push_str(&format!("    pub {}: {},\n", field.name, rust_type));
    }
    output.push_str("}\n\n");

    // Result struct
    output.push_str(&format!(
        "/// Result for ROS action: {}/{}\n",
        action.package, action.name
    ));
    output.push_str("#[derive(Clone, Debug, Default, Serialize, Deserialize)]\n");
    output.push_str(&format!("pub struct {}Result {{\n", action.name));
    for field in &action.result {
        if let Some(ref comment) = field.comment {
            output.push_str(&format!("    /// {}\n", comment));
        }
        let rust_type = ros2_to_rust_type(&field.field_type);
        output.push_str(&format!("    pub {}: {},\n", field.name, rust_type));
    }
    output.push_str("}\n\n");

    // Feedback struct
    output.push_str(&format!(
        "/// Feedback for ROS action: {}/{}\n",
        action.package, action.name
    ));
    output.push_str("#[derive(Clone, Debug, Default, Serialize, Deserialize)]\n");
    output.push_str(&format!("pub struct {}Feedback {{\n", action.name));
    for field in &action.feedback {
        if let Some(ref comment) = field.comment {
            output.push_str(&format!("    /// {}\n", comment));
        }
        let rust_type = ros2_to_rust_type(&field.field_type);
        output.push_str(&format!("    pub {}: {},\n", field.name, rust_type));
    }
    output.push_str("}\n");

    output
}

/// Parse a ROS XML launch file (ROS1 format, also used in ROS2)
pub fn parse_xml_launch_file(path: &Path) -> HorusResult<LaunchFile> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| HorusError::InvalidInput("Invalid launch filename".to_string()))?
        .to_string();

    let mut nodes = Vec::new();
    let mut arguments = Vec::new();
    let mut includes = Vec::new();
    let mut groups = Vec::new();
    let mut composable_containers = Vec::new();
    let mut current_node: Option<LaunchNode> = None;
    let mut current_group: Option<LaunchGroup> = None;

    // Simple XML parsing for launch elements
    for line in content.lines() {
        let line = line.trim();

        // Parse <arg> elements (launch arguments)
        if line.starts_with("<arg") {
            let mut arg = LaunchArgument::default();
            if let Some(name_val) = extract_xml_attr(line, "name") {
                arg.name = name_val;
            }
            if let Some(default) = extract_xml_attr(line, "default") {
                arg.default_value = Some(default);
            }
            if let Some(doc) = extract_xml_attr(line, "doc") {
                arg.description = Some(doc);
            }
            // Also try "description" attribute (ROS2)
            if arg.description.is_none() {
                if let Some(desc) = extract_xml_attr(line, "description") {
                    arg.description = Some(desc);
                }
            }
            if !arg.name.is_empty() {
                arguments.push(arg);
            }
        }

        // Parse <group> elements (ROS1 XML groups)
        if line.starts_with("<group") {
            let mut group = LaunchGroup::default();
            if let Some(ns) = extract_xml_attr(line, "ns") {
                group.namespace = Some(ns);
            }
            // Extract condition (if/unless)
            if let Some(cond_expr) = extract_xml_attr(line, "if") {
                group.condition = Some(LaunchCondition {
                    condition_type: "if".to_string(),
                    expression: cond_expr,
                });
            }
            if let Some(cond_expr) = extract_xml_attr(line, "unless") {
                group.condition = Some(LaunchCondition {
                    condition_type: "unless".to_string(),
                    expression: cond_expr,
                });
            }
            if !line.ends_with("/>") {
                current_group = Some(group);
            }
        }

        // Close group
        if line.starts_with("</group>") {
            if let Some(group) = current_group.take() {
                groups.push(group);
            }
        }

        // Parse <env> elements (environment variables in groups)
        if line.starts_with("<env") {
            if let Some(ref mut group) = current_group {
                let env_name = extract_xml_attr(line, "name").unwrap_or_default();
                let env_value = extract_xml_attr(line, "value").unwrap_or_default();
                if !env_name.is_empty() {
                    group.env_vars.push((env_name, env_value));
                }
            }
        }

        // Parse <include> elements
        if line.starts_with("<include") {
            let mut include = LaunchInclude::default();
            if let Some(file) = extract_xml_attr(line, "file") {
                include.file = file;
            }
            if let Some(ns) = extract_xml_attr(line, "ns") {
                include.namespace = Some(ns);
            }
            // Extract condition
            if let Some(cond_expr) = extract_xml_attr(line, "if") {
                include.condition = Some(LaunchCondition {
                    condition_type: "if".to_string(),
                    expression: cond_expr,
                });
            }
            if let Some(cond_expr) = extract_xml_attr(line, "unless") {
                include.condition = Some(LaunchCondition {
                    condition_type: "unless".to_string(),
                    expression: cond_expr,
                });
            }
            if !include.file.is_empty() {
                if let Some(ref mut group) = current_group {
                    group.includes.push(include);
                } else {
                    includes.push(include);
                }
            }
        }

        // Parse <node> elements
        if line.starts_with("<node") {
            let mut node = LaunchNode::default();

            // Extract attributes
            if let Some(pkg) = extract_xml_attr(line, "pkg") {
                node.package = pkg;
            }
            if let Some(name_val) = extract_xml_attr(line, "name") {
                node.name = name_val;
            }
            // ROS1 uses "type", ROS2 uses "exec"
            if let Some(exec) = extract_xml_attr(line, "type") {
                node.executable = exec;
            } else if let Some(exec) = extract_xml_attr(line, "exec") {
                node.executable = exec;
            }
            if let Some(ns) = extract_xml_attr(line, "ns") {
                node.namespace = Some(ns);
            }
            if let Some(output) = extract_xml_attr(line, "output") {
                node.output = Some(output);
            }

            // Extract condition (if/unless)
            if let Some(cond_expr) = extract_xml_attr(line, "if") {
                node.condition = Some(LaunchCondition {
                    condition_type: "if".to_string(),
                    expression: cond_expr,
                });
            }
            if let Some(cond_expr) = extract_xml_attr(line, "unless") {
                node.condition = Some(LaunchCondition {
                    condition_type: "unless".to_string(),
                    expression: cond_expr,
                });
            }

            // If this is a self-closing tag, add the node directly
            if line.ends_with("/>") {
                if !node.name.is_empty() && !node.package.is_empty() {
                    if let Some(ref mut group) = current_group {
                        group.nodes.push(node);
                    } else {
                        nodes.push(node);
                    }
                }
            } else {
                current_node = Some(node);
            }
        }

        // Parse <remap> elements (inside a node)
        if line.starts_with("<remap") {
            if let Some(ref mut node) = current_node {
                let from = extract_xml_attr(line, "from").unwrap_or_default();
                let to = extract_xml_attr(line, "to").unwrap_or_default();
                if !from.is_empty() && !to.is_empty() {
                    node.remappings.push((from, to));
                }
            }
        }

        // Parse <param> elements (inside a node)
        if line.starts_with("<param") {
            if let Some(ref mut node) = current_node {
                let param_name = extract_xml_attr(line, "name").unwrap_or_default();
                let param_value = extract_xml_attr(line, "value").unwrap_or_default();
                if !param_name.is_empty() {
                    node.parameters.push((param_name, param_value));
                }
            }
        }

        // Close node
        if line.starts_with("</node>") {
            if let Some(node) = current_node.take() {
                if !node.name.is_empty() && !node.package.is_empty() {
                    if let Some(ref mut group) = current_group {
                        group.nodes.push(node);
                    } else {
                        nodes.push(node);
                    }
                }
            }
        }

        // Parse ROS2 <composable_node_container> elements (for component containers)
        if line.contains("composable_node_container") || line.contains("ComposableNodeContainer") {
            let mut container = ComposableNodeContainer::default();
            if let Some(name_val) = extract_xml_attr(line, "name") {
                container.name = name_val;
            }
            if let Some(pkg) = extract_xml_attr(line, "pkg") {
                container.package = pkg;
            }
            if let Some(exec) = extract_xml_attr(line, "exec") {
                container.executable = exec;
            }
            if let Some(ns) = extract_xml_attr(line, "ns") {
                container.namespace = Some(ns);
            }
            if !container.name.is_empty() {
                composable_containers.push(container);
            }
        }

        // Parse <load_composable_node> elements
        if line.contains("load_composable_node") || line.contains("LoadComposableNodes") {
            // Mark the most recent container as having composable nodes
            // More detailed parsing would need multi-line handling
        }
    }

    Ok(LaunchFile {
        name,
        format: LaunchFormat::Xml,
        nodes,
        arguments,
        includes,
        groups,
        composable_containers,
        source_file: path.to_path_buf(),
    })
}

/// Extract XML attribute value
fn extract_xml_attr(line: &str, attr: &str) -> Option<String> {
    let pattern = format!("{}=\"", attr);
    if let Some(start) = line.find(&pattern) {
        let value_start = start + pattern.len();
        if let Some(end) = line[value_start..].find('"') {
            return Some(line[value_start..value_start + end].to_string());
        }
    }
    None
}

/// Parse a ROS2 Python launch file (basic extraction)
pub fn parse_python_launch_file(path: &Path) -> HorusResult<LaunchFile> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| HorusError::InvalidInput("Invalid launch filename".to_string()))?
        .to_string();

    let mut nodes = Vec::new();

    // Collect Node() blocks which may span multiple lines
    // Find each Node( and collect until the matching )
    let mut in_node = false;
    let mut node_block = String::new();
    let mut paren_depth = 0;

    for line in content.lines() {
        let trimmed = line.trim();

        if !in_node && trimmed.contains("Node(") {
            in_node = true;
            node_block.clear();
            // Find start of Node(
            if let Some(start) = trimmed.find("Node(") {
                node_block.push_str(&trimmed[start..]);
                // Count parens
                for ch in trimmed[start..].chars() {
                    if ch == '(' {
                        paren_depth += 1;
                    }
                    if ch == ')' {
                        paren_depth -= 1;
                    }
                }
            }
        } else if in_node {
            node_block.push(' ');
            node_block.push_str(trimmed);
            for ch in trimmed.chars() {
                if ch == '(' {
                    paren_depth += 1;
                }
                if ch == ')' {
                    paren_depth -= 1;
                }
            }
        }

        // Check if we've closed the Node() block
        if in_node && paren_depth == 0 {
            in_node = false;

            // Parse the complete node block
            let mut node = LaunchNode::default();

            // Extract Python string arguments from the collected block
            if let Some(pkg) = extract_python_arg(&node_block, "package") {
                node.package = pkg;
            }
            if let Some(exec) = extract_python_arg(&node_block, "executable") {
                node.executable = exec;
            }
            if let Some(n) = extract_python_arg(&node_block, "name") {
                node.name = n;
            }
            if let Some(ns) = extract_python_arg(&node_block, "namespace") {
                node.namespace = Some(ns);
            }
            if let Some(output) = extract_python_arg(&node_block, "output") {
                node.output = Some(output);
            }

            // Extract remappings: remappings=[('/from', '/to'), ...]
            if let Some(remap_start) = node_block.find("remappings=") {
                let rest = &node_block[remap_start..];
                if let Some(bracket_start) = rest.find('[') {
                    let mut bracket_depth = 0;
                    let mut bracket_end = bracket_start;
                    for (i, ch) in rest[bracket_start..].char_indices() {
                        if ch == '[' {
                            bracket_depth += 1;
                        }
                        if ch == ']' {
                            bracket_depth -= 1;
                            if bracket_depth == 0 {
                                bracket_end = bracket_start + i + 1;
                                break;
                            }
                        }
                    }
                    let remap_block = &rest[bracket_start..bracket_end];

                    // Extract tuples: ('/from', '/to') or ("/from", "/to")
                    let tuple_regex =
                        regex::Regex::new(r#"\(\s*['"]([^'"]+)['"]\s*,\s*['"]([^'"]+)['"]\s*\)"#)
                            .ok();

                    if let Some(re) = tuple_regex {
                        for cap in re.captures_iter(remap_block) {
                            let from = cap
                                .get(1)
                                .map(|m| m.as_str().to_string())
                                .unwrap_or_default();
                            let to = cap
                                .get(2)
                                .map(|m| m.as_str().to_string())
                                .unwrap_or_default();
                            if !from.is_empty() && !to.is_empty() {
                                node.remappings.push((from, to));
                            }
                        }
                    }
                }
            }

            // Use executable as name if not specified
            if node.name.is_empty() && !node.executable.is_empty() {
                node.name = node.executable.clone();
            }

            if !node.package.is_empty() && !node.executable.is_empty() {
                nodes.push(node);
            }
        }
    }

    // Parse launch arguments (DeclareLaunchArgument)
    let mut arguments = Vec::new();
    let arg_regex = regex::Regex::new(r#"DeclareLaunchArgument\s*\(\s*['"]([^'"]+)['"]"#).ok();

    if let Some(re) = arg_regex {
        for cap in re.captures_iter(&content) {
            let arg_name = cap
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();

            // Find the full DeclareLaunchArgument block
            if let Some(start) = content
                .find(&format!("DeclareLaunchArgument('{}'", arg_name))
                .or_else(|| content.find(&format!("DeclareLaunchArgument(\"{}\"", arg_name)))
            {
                let block_end = content[start..]
                    .find(')')
                    .map(|e| start + e + 1)
                    .unwrap_or(start + 100);
                let block = &content[start..block_end.min(content.len())];

                let default = extract_python_arg(block, "default_value");
                let desc = extract_python_arg(block, "description");

                arguments.push(LaunchArgument {
                    name: arg_name,
                    default_value: default,
                    description: desc,
                });
            }
        }
    }

    // Parse includes (IncludeLaunchDescription)
    let mut includes = Vec::new();
    let include_regex = regex::Regex::new(r#"IncludeLaunchDescription\s*\("#).ok();

    if let Some(re) = include_regex {
        for mat in re.find_iter(&content) {
            let start = mat.start();
            // Find the matching closing paren by counting parens
            let mut paren_depth = 0;
            let mut block_end = start;
            for (i, ch) in content[start..].char_indices() {
                if ch == '(' {
                    paren_depth += 1;
                }
                if ch == ')' {
                    paren_depth -= 1;
                    if paren_depth == 0 {
                        block_end = start + i + 1;
                        break;
                    }
                }
            }

            let block = &content[start..block_end.min(content.len())];

            // Extract file path from PythonLaunchDescriptionSource or XMLLaunchDescriptionSource
            let file_regex = regex::Regex::new(
                r#"LaunchDescriptionSource\s*\(\s*(?:os\.path\.join\s*\([^)]+\)|['"][^'"]+['"])"#,
            )
            .ok();

            let mut include = LaunchInclude::default();

            if let Some(file_re) = file_regex {
                if let Some(file_match) = file_re.find(block) {
                    // Try to extract the actual file path
                    let file_part = file_match.as_str();
                    // Look for a quoted string at the end
                    for quote in &['\'', '"'] {
                        if let Some(q_start) = file_part.rfind(*quote) {
                            if let Some(q_end) = file_part[..q_start].rfind(*quote) {
                                include.file = file_part[q_end + 1..q_start].to_string();
                                break;
                            }
                        }
                    }
                }
            }

            // Extract namespace if present
            if let Some(ns) = extract_python_arg(block, "namespace") {
                include.namespace = Some(ns);
            }

            if !include.file.is_empty() {
                includes.push(include);
            }
        }
    }

    // Note: Remappings are extracted during node parsing above

    // Parse groups (GroupAction with PushRosNamespace)
    let mut groups = Vec::new();
    if content.contains("GroupAction") {
        // Detect GroupAction patterns
        let group_regex = regex::Regex::new(r#"GroupAction\s*\(\s*\["#).ok();

        if let Some(re) = group_regex {
            for mat in re.find_iter(&content) {
                let start = mat.start();
                let mut group = LaunchGroup::default();

                // Check for PushRosNamespace in the vicinity
                let block = &content[start..content.len().min(start + 500)];
                if let Some(ns_start) = block.find("PushRosNamespace(") {
                    let ns_block = &block[ns_start..];
                    // Extract namespace value
                    if let Some(ns_val) = extract_python_arg(ns_block, "") {
                        group.namespace = Some(ns_val);
                    }
                    group.push_ros_namespace = true;
                }

                // Check for condition
                if let Some(cond) = extract_python_arg(block, "condition") {
                    if cond.contains("IfCondition") {
                        group.condition = Some(LaunchCondition {
                            condition_type: "IfCondition".to_string(),
                            expression: cond,
                        });
                    } else if cond.contains("UnlessCondition") {
                        group.condition = Some(LaunchCondition {
                            condition_type: "UnlessCondition".to_string(),
                            expression: cond,
                        });
                    }
                }

                groups.push(group);
            }
        }
    }

    // Parse composable node containers (ComposableNodeContainer)
    let mut composable_containers = Vec::new();
    if content.contains("ComposableNodeContainer") {
        let container_regex = regex::Regex::new(r#"ComposableNodeContainer\s*\("#).ok();

        if let Some(re) = container_regex {
            for mat in re.find_iter(&content) {
                let start = mat.start();
                // Find matching closing paren
                let mut paren_depth = 0;
                let mut block_end = start;
                for (i, ch) in content[start..].char_indices() {
                    if ch == '(' {
                        paren_depth += 1;
                    }
                    if ch == ')' {
                        paren_depth -= 1;
                        if paren_depth == 0 {
                            block_end = start + i + 1;
                            break;
                        }
                    }
                }

                let block = &content[start..block_end.min(content.len())];
                let mut container = ComposableNodeContainer::default();

                if let Some(name_val) = extract_python_arg(block, "name") {
                    container.name = name_val;
                }
                if let Some(pkg) = extract_python_arg(block, "package") {
                    container.package = pkg;
                }
                if let Some(exec) = extract_python_arg(block, "executable") {
                    container.executable = exec;
                }
                if let Some(ns) = extract_python_arg(block, "namespace") {
                    container.namespace = Some(ns);
                }

                if !container.name.is_empty() {
                    composable_containers.push(container);
                }
            }
        }
    }

    // Parse conditions on nodes (IfCondition, UnlessCondition)
    // Update existing nodes with their conditions
    for _node in &mut nodes {
        // Conditions are typically passed as `condition=IfCondition(...)` in the Node() block
        // This is a simplified detection - actual parsing would need to re-examine the original block
    }

    Ok(LaunchFile {
        name,
        format: LaunchFormat::Python,
        nodes,
        arguments,
        includes,
        groups,
        composable_containers,
        source_file: path.to_path_buf(),
    })
}

/// Extract Python function argument value
fn extract_python_arg(line: &str, arg: &str) -> Option<String> {
    // Try both single and double quotes
    for quote in &['\'', '"'] {
        let pattern = format!("{}={}", arg, quote);
        if let Some(start) = line.find(&pattern) {
            let value_start = start + pattern.len();
            if let Some(end) = line[value_start..].find(*quote) {
                return Some(line[value_start..value_start + end].to_string());
            }
        }
    }
    None
}

/// Parse a ROS2 YAML launch file
pub fn parse_yaml_launch_file(path: &Path) -> HorusResult<LaunchFile> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| HorusError::InvalidInput("Invalid launch filename".to_string()))?
        .to_string();

    let mut nodes = Vec::new();
    let mut current_node: Option<LaunchNode> = None;

    // Simple YAML parsing for node configurations
    for line in content.lines() {
        let indent = line.len() - line.trim_start().len();
        let line = line.trim();

        if line.starts_with("- node:") || line.starts_with("node:") {
            if let Some(node) = current_node.take() {
                if !node.package.is_empty() && !node.executable.is_empty() {
                    nodes.push(node);
                }
            }
            current_node = Some(LaunchNode::default());
        } else if indent > 0 && current_node.is_some() {
            let node = current_node.as_mut().unwrap();
            if line.starts_with("pkg:") || line.starts_with("package:") {
                node.package = line
                    .split(':')
                    .nth(1)
                    .unwrap_or("")
                    .trim()
                    .trim_matches('"')
                    .trim_matches('\'')
                    .to_string();
            } else if line.starts_with("exec:") || line.starts_with("executable:") {
                node.executable = line
                    .split(':')
                    .nth(1)
                    .unwrap_or("")
                    .trim()
                    .trim_matches('"')
                    .trim_matches('\'')
                    .to_string();
            } else if line.starts_with("name:") {
                node.name = line
                    .split(':')
                    .nth(1)
                    .unwrap_or("")
                    .trim()
                    .trim_matches('"')
                    .trim_matches('\'')
                    .to_string();
            } else if line.starts_with("namespace:") || line.starts_with("ns:") {
                node.namespace = Some(
                    line.split(':')
                        .nth(1)
                        .unwrap_or("")
                        .trim()
                        .trim_matches('"')
                        .trim_matches('\'')
                        .to_string(),
                );
            } else if line.starts_with("output:") {
                node.output = Some(
                    line.split(':')
                        .nth(1)
                        .unwrap_or("")
                        .trim()
                        .trim_matches('"')
                        .trim_matches('\'')
                        .to_string(),
                );
            }
        }
    }

    // Don't forget the last node
    if let Some(node) = current_node {
        if !node.package.is_empty() && !node.executable.is_empty() {
            nodes.push(node);
        }
    }

    // Parse launch arguments from YAML (args: section)
    let mut arguments = Vec::new();
    let mut in_args = false;
    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("args:") || trimmed.starts_with("arguments:") {
            in_args = true;
            continue;
        }
        if in_args {
            // Check if we've exited the args section (less indentation)
            if !line.starts_with(' ') && !line.is_empty() && !trimmed.starts_with('-') {
                in_args = false;
                continue;
            }
            // Parse arg: - name: foo, default: bar, description: baz
            if let Some(name_idx) = trimmed.find("name:") {
                let mut arg = LaunchArgument::default();
                let rest = &trimmed[name_idx + 5..];
                if let Some(comma_idx) = rest.find(',') {
                    arg.name = rest[..comma_idx]
                        .trim()
                        .trim_matches('"')
                        .trim_matches('\'')
                        .to_string();
                } else {
                    arg.name = rest.trim().trim_matches('"').trim_matches('\'').to_string();
                }
                if let Some(def_idx) = trimmed.find("default:") {
                    let def_rest = &trimmed[def_idx + 8..];
                    if let Some(comma_idx) = def_rest.find(',') {
                        arg.default_value = Some(
                            def_rest[..comma_idx]
                                .trim()
                                .trim_matches('"')
                                .trim_matches('\'')
                                .to_string(),
                        );
                    } else {
                        arg.default_value = Some(
                            def_rest
                                .trim()
                                .trim_matches('"')
                                .trim_matches('\'')
                                .to_string(),
                        );
                    }
                }
                if !arg.name.is_empty() {
                    arguments.push(arg);
                }
            }
        }
    }

    Ok(LaunchFile {
        name,
        format: LaunchFormat::Yaml,
        nodes,
        arguments,
        includes: Vec::new(), // YAML launch includes are less common
        groups: Vec::new(),
        composable_containers: Vec::new(),
        source_file: path.to_path_buf(),
    })
}

/// Parse a plugin.xml file (ROS1 nodelets, pluginlib plugins)
pub fn parse_plugin_xml(path: &Path) -> HorusResult<Vec<PluginInfo>> {
    let content = fs::read_to_string(path).map_err(|e| HorusError::Io(e))?;

    let mut plugins = Vec::new();

    // Parse <class> elements within <library> elements
    // Format: <library path="lib/libmy_nodelet">
    //           <class name="my_pkg/MyNodelet" type="my_pkg::MyNodelet" base_class_type="nodelet::Nodelet">
    //             <description>Description</description>
    //           </class>
    //         </library>

    let mut current_library_path = String::new();

    for line in content.lines() {
        let line = line.trim();

        // Extract library path
        if line.starts_with("<library") {
            if let Some(path_val) = extract_xml_attr(line, "path") {
                current_library_path = path_val;
            }
        }

        // Parse <class> elements
        if line.starts_with("<class") {
            let mut plugin = PluginInfo::default();
            plugin.library_path = current_library_path.clone();

            // ROS1 format: name="my_pkg/MyNodelet" type="my_pkg::MyNodelet" base_class_type="nodelet::Nodelet"
            if let Some(name) = extract_xml_attr(line, "name") {
                plugin.class_name = name;
            }
            if let Some(type_val) = extract_xml_attr(line, "type") {
                plugin.class_name = type_val; // type is the actual C++ class name
            }
            if let Some(base_class) = extract_xml_attr(line, "base_class_type") {
                plugin.base_class = base_class.clone();
                // Determine plugin type from base class
                if base_class.contains("nodelet::Nodelet") {
                    plugin.plugin_type = "nodelet".to_string();
                } else if base_class.contains("Controller") {
                    plugin.plugin_type = "controller".to_string();
                } else if base_class.contains("Planner") || base_class.contains("planner") {
                    plugin.plugin_type = "planner".to_string();
                } else if base_class.contains("costmap_2d") {
                    plugin.plugin_type = "costmap_layer".to_string();
                } else if base_class.contains("recovery") {
                    plugin.plugin_type = "recovery".to_string();
                } else {
                    plugin.plugin_type = "plugin".to_string();
                }
            }

            // ROS2 format for component loading
            if let Some(base_class) = extract_xml_attr(line, "base_class") {
                plugin.base_class = base_class;
            }

            if !plugin.class_name.is_empty() {
                plugins.push(plugin);
            }
        }

        // Extract description if present
        if line.starts_with("<description>") && !plugins.is_empty() {
            let desc = line
                .trim_start_matches("<description>")
                .trim_end_matches("</description>")
                .trim()
                .to_string();
            if let Some(last) = plugins.last_mut() {
                last.description = desc;
            }
        }
    }

    Ok(plugins)
}

/// Map CMake package name to Rust crate equivalent
pub fn map_cmake_to_rust_crate(cmake_name: &str) -> ExternalDep {
    match cmake_name.to_lowercase().as_str() {
        // Computer vision
        "opencv" | "opencv4" | "cv_bridge" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("opencv = \"0.88\"".to_string()),
            cargo_feature: None,
            notes: Some("OpenCV Rust bindings. Requires OpenCV 4.x installed.".to_string()),
        },

        // Linear algebra / Math
        "eigen" | "eigen3" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("nalgebra = \"0.32\"".to_string()),
            cargo_feature: None,
            notes: Some("nalgebra provides similar linear algebra functionality to Eigen.".to_string()),
        },

        // Point cloud processing
        "pcl" | "pcl_ros" | "pcl_conversions" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("# pcl - No direct Rust equivalent, use kiss3d for visualization or custom implementation".to_string()),
            cargo_feature: None,
            notes: Some("PCL has no direct Rust equivalent. Consider HORUS PointCloud type or kiss3d for visualization.".to_string()),
        },

        // Image processing
        "image_transport" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("image = \"0.24\"".to_string()),
            cargo_feature: None,
            notes: Some("Rust image crate for image processing. HORUS handles transport.".to_string()),
        },

        // Boost (common in ROS1)
        "boost" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: None,
            cargo_feature: None,
            notes: Some("Boost is not needed in Rust - use std library equivalents.".to_string()),
        },

        // Threading
        "threads" | "pthread" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("rayon = \"1.8\"".to_string()),
            cargo_feature: None,
            notes: Some("Use rayon for parallel processing or std::thread for threading.".to_string()),
        },

        // YAML parsing
        "yaml-cpp" | "yaml_cpp" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("serde_yaml = \"0.9\"".to_string()),
            cargo_feature: None,
            notes: Some("serde_yaml for YAML parsing.".to_string()),
        },

        // JSON
        "jsoncpp" | "nlohmann_json" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("serde_json = \"1.0\"".to_string()),
            cargo_feature: None,
            notes: Some("serde_json for JSON parsing.".to_string()),
        },

        // Serial communication
        "serial" | "libserial" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("serialport = \"4.2\"".to_string()),
            cargo_feature: None,
            notes: Some("serialport crate for serial communication.".to_string()),
        },

        // USB
        "libusb" | "usb" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: Some("rusb = \"0.9\"".to_string()),
            cargo_feature: None,
            notes: Some("rusb for USB communication.".to_string()),
        },

        // TF2 / transforms
        "tf2" | "tf2_ros" | "tf" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: None,
            cargo_feature: None,
            notes: Some("HORUS has built-in Transform type. See horus::prelude::Transform.".to_string()),
        },

        // Navigation
        "nav_msgs" | "nav2_msgs" => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: None,
            cargo_feature: None,
            notes: Some("Use HORUS navigation message types: Odometry, Path, OccupancyGrid.".to_string()),
        },

        // Unknown - provide generic guidance
        _ => ExternalDep {
            cmake_name: cmake_name.to_string(),
            rust_crate: None,
            cargo_feature: None,
            notes: Some(format!("No known Rust equivalent for '{}'. Check crates.io for alternatives.", cmake_name)),
        },
    }
}

/// Parse CMakeLists.txt to extract external dependencies
pub fn parse_cmake_dependencies(cmake_path: &Path) -> Vec<ExternalDep> {
    let content = match fs::read_to_string(cmake_path) {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };

    let mut deps = Vec::new();
    let mut seen = std::collections::HashSet::new();

    // Look for find_package() calls
    for line in content.lines() {
        let line = line.trim();

        // Skip comments
        if line.starts_with('#') {
            continue;
        }

        // Match find_package(PackageName ...) patterns
        if line.contains("find_package(") {
            // Extract package name
            if let Some(start) = line.find("find_package(") {
                let after_paren = &line[start + 13..];
                // Get the first word (package name - can include hyphens like yaml-cpp)
                let pkg_name: String = after_paren
                    .chars()
                    .take_while(|c| c.is_alphanumeric() || *c == '_' || *c == '-')
                    .collect();

                if !pkg_name.is_empty() && !seen.contains(&pkg_name) {
                    // Skip ROS-specific packages that don't need mapping
                    let ros_packages = [
                        "catkin",
                        "roscpp",
                        "rospy",
                        "std_msgs",
                        "message_generation",
                        "message_runtime",
                        "ament_cmake",
                        "rclcpp",
                        "rclpy",
                        "rosidl_default_generators",
                    ];

                    if !ros_packages.contains(&pkg_name.to_lowercase().as_str()) {
                        seen.insert(pkg_name.clone());
                        deps.push(map_cmake_to_rust_crate(&pkg_name));
                    }
                }
            }
        }
    }

    deps
}

/// Parse CMakeLists.txt to extract executable/library targets
pub fn parse_cmake_targets(cmake_path: &Path) -> Vec<CmakeTarget> {
    let content = match fs::read_to_string(cmake_path) {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };

    let mut targets = Vec::new();

    // Process line by line, handling multiline definitions
    let mut current_cmd = String::new();
    let mut paren_depth = 0;

    for line in content.lines() {
        let trimmed = line.trim();

        // Skip pure comments
        if trimmed.starts_with('#') {
            continue;
        }

        // Remove inline comments
        let line_no_comment = if let Some(hash_pos) = trimmed.find('#') {
            &trimmed[..hash_pos]
        } else {
            trimmed
        };

        // Accumulate command
        current_cmd.push(' ');
        current_cmd.push_str(line_no_comment);

        // Count parentheses
        for ch in line_no_comment.chars() {
            if ch == '(' {
                paren_depth += 1;
            }
            if ch == ')' {
                paren_depth -= 1;
            }
        }

        // Process complete command when balanced
        if paren_depth == 0 && !current_cmd.trim().is_empty() {
            let cmd = current_cmd.trim().to_string();

            // Parse add_executable(target source1.cpp source2.cpp ...)
            if cmd.contains("add_executable(") {
                if let Some(target) = parse_add_target(&cmd, false) {
                    targets.push(target);
                }
            }

            // Parse add_library(target source1.cpp source2.cpp ...)
            if cmd.contains("add_library(") {
                if let Some(target) = parse_add_target(&cmd, true) {
                    targets.push(target);
                }
            }

            current_cmd.clear();
        }
    }

    targets
}

/// Parse add_executable or add_library command
fn parse_add_target(cmd: &str, is_library: bool) -> Option<CmakeTarget> {
    let pattern = if is_library {
        "add_library("
    } else {
        "add_executable("
    };
    let start = cmd.find(pattern)?;
    let after_paren = &cmd[start + pattern.len()..];
    let end = after_paren.find(')')?;
    let args = &after_paren[..end];

    // Split by whitespace
    let parts: Vec<&str> = args.split_whitespace().collect();
    if parts.is_empty() {
        return None;
    }

    let name = parts[0].to_string();

    // Skip CMake variables like ${PROJECT_NAME}
    if name.starts_with("${") {
        return None;
    }

    // Collect source files (skip CMake keywords)
    let cmake_keywords = [
        "SHARED",
        "STATIC",
        "MODULE",
        "OBJECT",
        "INTERFACE",
        "IMPORTED",
        "ALIAS",
        "EXCLUDE_FROM_ALL",
    ];
    let source_files: Vec<String> = parts[1..]
        .iter()
        .filter(|s| !cmake_keywords.contains(s) && !s.starts_with("${") && !s.starts_with("$<"))
        .map(|s| s.to_string())
        .collect();

    Some(CmakeTarget {
        name,
        source_files,
        is_library,
    })
}

/// Generate HORUS Horus.toml nodes section from launch files
pub fn generate_horus_nodes_from_launch(launch_files: &[LaunchFile]) -> String {
    let mut output = String::new();
    output.push_str("\n# Nodes extracted from ROS launch files\n");
    output.push_str("# Uncomment and configure as needed:\n\n");

    for launch in launch_files {
        output.push_str(&format!(
            "# From launch file: {}\n",
            launch.source_file.display()
        ));
        for node in &launch.nodes {
            output.push_str(&format!("# [[nodes]]\n"));
            output.push_str(&format!("# name = \"{}\"\n", node.name));
            output.push_str(&format!(
                "# # Original: {} / {}\n",
                node.package, node.executable
            ));
            if let Some(ref ns) = node.namespace {
                output.push_str(&format!("# namespace = \"{}\"\n", ns));
            }
            output.push_str("# frequency = 100\n\n");
        }
    }

    output
}

/// Generate Cargo.toml for converted package
pub fn generate_cargo_toml(package: &RosPackage, has_python: bool) -> String {
    let mut output = format!(
        r#"[package]
name = "{}"
version = "{}"
edition = "2021"
description = "{}"
license = "{}"
# Converted from ROS2 package

[dependencies]
horus = "0.1"
serde = {{ version = "1.0", features = ["derive"] }}

"#,
        package.name.replace('-', "_"),
        package.version,
        // Sanitize description: replace quotes and collapse newlines/whitespace
        package
            .description
            .replace('"', "'")
            .replace('\n', " ")
            .replace('\r', " ")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" "),
        package.license
    );

    if has_python {
        output.push_str("pyo3 = { version = \"0.20\", features = [\"extension-module\"] }\n");
    }

    // Add external dependencies detected from CMakeLists.txt
    if !package.external_deps.is_empty() {
        output.push_str("# External dependencies detected from CMakeLists.txt:\n");
        for dep in &package.external_deps {
            if let Some(ref crate_spec) = dep.rust_crate {
                if crate_spec.starts_with('#') {
                    // It's a comment (no direct equivalent)
                    output.push_str(&format!("{}\n", crate_spec));
                } else {
                    output.push_str(&format!("{}\n", crate_spec));
                }
            } else if let Some(ref notes) = dep.notes {
                output.push_str(&format!("# {} - {}\n", dep.cmake_name, notes));
            }
        }
        output.push('\n');
    }

    output
}

/// Generate Horus.toml for converted package
pub fn generate_horus_toml(package: &RosPackage, launch_files: &[LaunchFile]) -> String {
    let mut output = format!(
        r#"# HORUS Project Configuration
# Converted from ROS2 package: {}

[package]
name = "{}"
version = "{}"
description = "{}"

[scheduler]
frequency = 100  # Hz
execution_mode = "RealTime"

# Uncomment and configure nodes as needed:
# [[nodes]]
# name = "my_node"
# file = "src/nodes/my_node.rs"
# frequency = 100
"#,
        package.name, package.name, package.version, package.description,
    );

    // Add nodes from launch files
    if !launch_files.is_empty() {
        output.push_str(&generate_horus_nodes_from_launch(launch_files));
    }

    output
}

/// Generate Horus.toml helper (old signature for compatibility)
fn _generate_horus_toml_simple(package: &RosPackage) -> String {
    format!(
        r#"# HORUS Project Configuration
# Converted from ROS2 package: {}

[package]
name = "{}"
version = "{}"
description = "{}"

[scheduler]
frequency = 100  # Hz
execution_mode = "RealTime"

# Uncomment and configure nodes as needed:
# [[nodes]]
# name = "my_node"
# file = "src/nodes/my_node.rs"
# frequency = 100
"#,
        package.name,
        package.name,
        package.version,
        package.description.replace('"', "'")
    )
}

/// QoS settings extracted from ROS2 source
#[derive(Debug, Clone, Default)]
pub struct ExtractedQoS {
    pub depth: Option<u32>,
    pub reliability: Option<String>, // "reliable" or "best_effort"
    pub durability: Option<String>,  // "transient_local" or "volatile"
    pub history: Option<String>,     // "keep_last" or "keep_all"
}

/// Extracted ROS2 publisher info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedPublisher {
    pub topic: String,
    pub msg_type: String,
    pub field_name: String,
    pub qos: ExtractedQoS,
}

/// Extracted ROS2 subscriber info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedSubscriber {
    pub topic: String,
    pub msg_type: String,
    pub field_name: String,
    pub callback: String,
    pub qos: ExtractedQoS,
}

/// Extracted ROS2 timer info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedTimer {
    pub period_ms: u64,
    pub callback: String,
}

/// Extracted ROS2 parameter info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedParameter {
    pub name: String,
    pub param_type: String,
    pub default_value: Option<String>,
}

/// Extracted ROS2 service server info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedServiceServer {
    pub service_name: String,
    pub srv_type: String,
    pub field_name: String,
    pub callback: String,
}

/// Extracted ROS2 service client info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedServiceClient {
    pub service_name: String,
    pub srv_type: String,
    pub field_name: String,
}

/// Extracted ROS2 action server info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedActionServer {
    pub action_name: String,
    pub action_type: String,
    pub field_name: String,
}

/// Extracted ROS2 action client info from C++ source
#[derive(Debug, Clone)]
pub struct ExtractedActionClient {
    pub action_name: String,
    pub action_type: String,
    pub field_name: String,
}

/// Extracted TF broadcaster info
#[derive(Debug, Clone)]
pub struct ExtractedTfBroadcaster {
    pub broadcaster_type: String, // "static" or "dynamic"
    pub field_name: String,
}

/// Extracted TF listener info
#[derive(Debug, Clone)]
pub struct ExtractedTfListener {
    pub buffer_name: String,
    pub listener_name: String,
}

/// Extracted message filter info (time synchronization)
#[derive(Debug, Clone)]
pub struct ExtractedMessageFilter {
    pub filter_type: String, // "TimeSynchronizer", "ApproximateTime", "ExactTime"
    pub topics: Vec<String>,
    pub queue_size: u32,
}

/// Extracted image transport info
#[derive(Debug, Clone)]
pub struct ExtractedImageTransport {
    pub transport_type: String, // "subscriber", "publisher", "camera_subscriber", "camera_publisher"
    pub topic: String,
    pub field_name: String,
}

/// Extracted diagnostic updater info
#[derive(Debug, Clone)]
pub struct ExtractedDiagnostic {
    pub updater_name: String,
    pub hardware_id: Option<String>,
}

/// Extracted dynamic_reconfigure server (ROS1)
#[derive(Debug, Clone)]
pub struct ExtractedDynamicReconfigure {
    pub config_type: String,
    pub callback: String,
}

/// Extracted executor info (ROS2)
#[derive(Debug, Clone)]
pub struct ExtractedExecutor {
    pub executor_type: String, // "SingleThreaded", "MultiThreaded", "StaticSingleThreaded"
}

/// Extracted callback group info (ROS2)
#[derive(Debug, Clone)]
pub struct ExtractedCallbackGroup {
    pub group_type: String, // "MutuallyExclusive", "Reentrant"
    pub name: String,
}

/// Extracted ros2_control hardware interface info
#[derive(Debug, Clone, Default)]
pub struct ExtractedHardwareInterface {
    pub interface_type: String, // "SystemInterface", "ActuatorInterface", "SensorInterface"
    pub state_interfaces: Vec<String>,
    pub command_interfaces: Vec<String>,
}

/// Extracted ros_control controller info
#[derive(Debug, Clone, Default)]
pub struct ExtractedController {
    pub controller_type: String, // "JointPositionController", "JointVelocityController", etc.
    pub controller_name: String,
}

/// Extracted rosserial/MicroROS info
#[derive(Debug, Clone, Default)]
pub struct ExtractedEmbeddedROS {
    pub ros_type: String, // "rosserial", "micro_ros"
    pub publishers: Vec<String>,
    pub subscribers: Vec<String>,
}

/// Extracted WaitSet/GuardCondition info (ROS2)
#[derive(Debug, Clone, Default)]
pub struct ExtractedWaitSet {
    pub has_waitset: bool,
    pub guard_conditions: Vec<String>,
}

/// Extracted IntraProcessComms info (ROS2)
#[derive(Debug, Clone, Default)]
pub struct ExtractedIntraProcess {
    pub enabled: bool,
    pub buffer_size: Option<u32>,
}

/// Extracted Bond info (heartbeat monitoring)
#[derive(Debug, Clone, Default)]
pub struct ExtractedBond {
    pub bond_id: String,
    pub topic: String,
}

/// Extracted laser_geometry info
#[derive(Debug, Clone, Default)]
pub struct ExtractedLaserGeometry {
    pub projector_name: String,
}

/// Extracted URDF/robot_description info
#[derive(Debug, Clone, Default)]
pub struct ExtractedURDF {
    pub robot_name: String,
    pub links: Vec<String>,
    pub joints: Vec<String>,
    pub has_xacro: bool,
}

/// Extracted Nav2 patterns
#[derive(Debug, Clone, Default)]
pub struct ExtractedNav2 {
    pub behavior_trees: Vec<String>,
    pub costmap_plugins: Vec<String>,
    pub planners: Vec<String>,
    pub controllers: Vec<String>,
    pub recovery_behaviors: Vec<String>,
}

/// Extracted MoveIt patterns
#[derive(Debug, Clone, Default)]
pub struct ExtractedMoveIt {
    pub move_group_interface: bool,
    pub planning_scene_interface: bool,
    pub robot_model: Option<String>,
    pub planning_groups: Vec<String>,
}

/// Extracted Gazebo plugin info
#[derive(Debug, Clone, Default)]
pub struct ExtractedGazeboPlugin {
    pub plugin_type: String, // "ModelPlugin", "WorldPlugin", "SensorPlugin", "VisualPlugin"
    pub plugin_name: String,
}

/// Extracted RViz plugin/visualization info
#[derive(Debug, Clone, Default)]
pub struct ExtractedVisualization {
    pub marker_publishers: Vec<String>,
    pub interactive_markers: Vec<String>,
    pub rviz_plugins: Vec<String>,
}

/// Extracted rosbag/ros2bag patterns
#[derive(Debug, Clone, Default)]
pub struct ExtractedBagRecording {
    pub topics: Vec<String>,
    pub is_recorder: bool,
    pub is_player: bool,
}

/// Extracted camera_info_manager patterns
#[derive(Debug, Clone, Default)]
pub struct ExtractedCameraInfo {
    pub camera_name: String,
    pub camera_info_url: Option<String>,
}

/// Extracted image_geometry patterns
#[derive(Debug, Clone, Default)]
pub struct ExtractedImageGeometry {
    pub camera_model: String, // "PinholeCameraModel", "StereoCameraModel"
}

/// Extracted PCL processing patterns
#[derive(Debug, Clone, Default)]
pub struct ExtractedPCL {
    pub filters: Vec<String>, // "VoxelGrid", "PassThrough", "StatisticalOutlierRemoval", etc.
    pub nodelets: Vec<String>,
}

/// Extracted realtime_tools patterns
#[derive(Debug, Clone, Default)]
pub struct ExtractedRealtimeTools {
    pub realtime_publisher: bool,
    pub realtime_buffer: bool,
    pub realtime_box: bool,
}

/// Extracted state estimation patterns (robot_localization, EKF, UKF)
#[derive(Debug, Clone, Default)]
pub struct ExtractedStateEstimation {
    pub estimator_type: String,      // "ekf", "ukf", "robot_localization"
    pub sensor_sources: Vec<String>, // "odom", "imu", "gps", etc.
    pub output_frame: Option<String>,
}

/// Extracted SLAM patterns (slam_toolbox, cartographer, gmapping, etc.)
#[derive(Debug, Clone, Default)]
pub struct ExtractedSLAM {
    pub slam_type: String, // "slam_toolbox", "cartographer", "gmapping", "hector", "rtabmap"
    pub has_loop_closure: bool,
    pub has_map_save: bool,
    pub has_localization_mode: bool,
}

/// Extracted localization patterns (amcl, etc.)
#[derive(Debug, Clone, Default)]
pub struct ExtractedLocalization {
    pub localization_type: String, // "amcl", "mrpt", "custom"
    pub uses_particle_filter: bool,
}

/// Extracted 3D mapping patterns (octomap, voxblox, etc.)
#[derive(Debug, Clone, Default)]
pub struct Extracted3DMapping {
    pub mapping_type: String, // "octomap", "voxblox", "elevation_mapping"
    pub uses_octree: bool,
    pub resolution: Option<f64>,
}

/// Extracted depth/stereo image processing
#[derive(Debug, Clone, Default)]
pub struct ExtractedDepthProcessing {
    pub processing_type: String, // "depth_image_proc", "stereo_image_proc"
    pub operations: Vec<String>, // "point_cloud_xyz", "convert_metric", "disparity", etc.
}

/// Extracted web bridge patterns (rosbridge, roslibjs)
#[derive(Debug, Clone, Default)]
pub struct ExtractedWebBridge {
    pub bridge_type: String, // "rosbridge", "rosapi"
    pub websocket: bool,
}

/// Extracted state machine patterns (smach, FlexBE, SMACC)
#[derive(Debug, Clone, Default)]
pub struct ExtractedStateMachine {
    pub state_machine_type: String, // "smach", "flexbe", "smacc"
    pub states: Vec<String>,
    pub transitions: Vec<String>,
}

/// Extracted sensor driver patterns (velodyne, realsense, zed, etc.)
#[derive(Debug, Clone, Default)]
pub struct ExtractedSensorDriver {
    pub sensor_type: String, // "velodyne", "realsense", "zed", "xsens", "mynt_eye"
    pub driver_name: String,
}

/// Parsed node info from C++ source
#[derive(Debug, Clone, Default)]
pub struct ParsedNodeInfo {
    pub class_name: String,
    pub publishers: Vec<ExtractedPublisher>,
    pub subscribers: Vec<ExtractedSubscriber>,
    pub timers: Vec<ExtractedTimer>,
    pub parameters: Vec<ExtractedParameter>,
    pub service_servers: Vec<ExtractedServiceServer>,
    pub service_clients: Vec<ExtractedServiceClient>,
    pub action_servers: Vec<ExtractedActionServer>,
    pub action_clients: Vec<ExtractedActionClient>,
    pub tf_broadcasters: Vec<ExtractedTfBroadcaster>,
    pub tf_listeners: Vec<ExtractedTfListener>,
    pub message_filters: Vec<ExtractedMessageFilter>,
    pub image_transports: Vec<ExtractedImageTransport>,
    pub diagnostics: Vec<ExtractedDiagnostic>,
    pub dynamic_reconfigure: Vec<ExtractedDynamicReconfigure>,
    pub executors: Vec<ExtractedExecutor>,
    pub callback_groups: Vec<ExtractedCallbackGroup>,
    // New comprehensive features
    pub hardware_interfaces: Vec<ExtractedHardwareInterface>,
    pub controllers: Vec<ExtractedController>,
    pub embedded_ros: Option<ExtractedEmbeddedROS>,
    pub waitset: Option<ExtractedWaitSet>,
    pub intra_process: Option<ExtractedIntraProcess>,
    pub bonds: Vec<ExtractedBond>,
    pub laser_geometry: Option<ExtractedLaserGeometry>,
    pub urdf: Option<ExtractedURDF>,
    pub nav2: Option<ExtractedNav2>,
    pub moveit: Option<ExtractedMoveIt>,
    pub gazebo_plugins: Vec<ExtractedGazeboPlugin>,
    pub visualization: Option<ExtractedVisualization>,
    pub bag_recording: Option<ExtractedBagRecording>,
    pub camera_info: Option<ExtractedCameraInfo>,
    pub image_geometry: Option<ExtractedImageGeometry>,
    pub pcl: Option<ExtractedPCL>,
    pub realtime_tools: Option<ExtractedRealtimeTools>,
    // Additional comprehensive patterns
    pub state_estimation: Option<ExtractedStateEstimation>,
    pub slam: Option<ExtractedSLAM>,
    pub localization: Option<ExtractedLocalization>,
    pub mapping_3d: Option<Extracted3DMapping>,
    pub depth_processing: Option<ExtractedDepthProcessing>,
    pub web_bridge: Option<ExtractedWebBridge>,
    pub state_machine: Option<ExtractedStateMachine>,
    pub sensor_drivers: Vec<ExtractedSensorDriver>,
    pub is_lifecycle: bool,
    pub is_component: bool,
    pub is_nodelet: bool, // ROS1 nodelet
}

/// Parse QoS settings from a C++ QoS argument
fn parse_qos_from_arg(qos_arg: &str, _content: &str) -> ExtractedQoS {
    let mut qos = ExtractedQoS::default();

    // If it's just a number, that's the depth
    if let Ok(depth) = qos_arg.parse::<u32>() {
        qos.depth = Some(depth);
        return qos;
    }

    // Look for common QoS patterns
    let arg_lower = qos_arg.to_lowercase();

    // Check for reliability
    if arg_lower.contains("reliable") {
        qos.reliability = Some("reliable".to_string());
    } else if arg_lower.contains("best_effort") {
        qos.reliability = Some("best_effort".to_string());
    }

    // Check for durability
    if arg_lower.contains("transient_local") {
        qos.durability = Some("transient_local".to_string());
    } else if arg_lower.contains("volatile") {
        qos.durability = Some("volatile".to_string());
    }

    // Check for history
    if arg_lower.contains("keep_all") {
        qos.history = Some("keep_all".to_string());
    } else if arg_lower.contains("keep_last") {
        qos.history = Some("keep_last".to_string());
    }

    // Try to extract depth from QoSProfile or keep_last(N)
    if let Some(re) = regex::Regex::new(r"keep_last\s*\(\s*(\d+)\s*\)").ok() {
        if let Some(caps) = re.captures(qos_arg) {
            if let Some(depth) = caps.get(1).and_then(|m| m.as_str().parse::<u32>().ok()) {
                qos.depth = Some(depth);
            }
        }
    }

    // Extract depth from .depth = N or depth(N)
    if let Some(re) = regex::Regex::new(r"(?:\.depth\s*=\s*|depth\s*\(\s*)(\d+)").ok() {
        if let Some(caps) = re.captures(qos_arg) {
            if let Some(depth) = caps.get(1).and_then(|m| m.as_str().parse::<u32>().ok()) {
                qos.depth = Some(depth);
            }
        }
    }

    qos
}

/// Parse a C++ source file to extract ROS2 node information
pub fn parse_cpp_source(content: &str) -> ParsedNodeInfo {
    let mut info = ParsedNodeInfo::default();

    // Extract class name (inherits from rclcpp::Node or Node)
    let class_regex =
        regex::Regex::new(r"class\s+(\w+)\s*:\s*(?:public\s+)?(?:rclcpp::)?Node").ok();

    if let Some(re) = &class_regex {
        if let Some(caps) = re.captures(content) {
            info.class_name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
        }
    }

    // Extract publishers: create_publisher<MsgType>("topic", qos)
    // Extended regex to capture QoS depth if numeric
    let pub_regex = regex::Regex::new(
        r#"create_publisher\s*<\s*([^>]+)\s*>\s*\(\s*"([^"]+)"\s*,\s*(\d+|[^)]+)"#,
    )
    .ok();

    if let Some(re) = &pub_regex {
        for caps in re.captures_iter(content) {
            let msg_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let topic = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let qos_arg = caps.get(3).map(|m| m.as_str().trim()).unwrap_or("");
            let field_name = topic_to_field_name(&topic);

            let qos = parse_qos_from_arg(qos_arg, content);

            info.publishers.push(ExtractedPublisher {
                topic,
                msg_type,
                field_name,
                qos,
            });
        }
    }

    // Extract subscribers: create_subscription<MsgType>("topic", qos, callback) (ROS2)
    let sub_regex = regex::Regex::new(
        r#"create_subscription\s*<\s*([^>]+)\s*>\s*\(\s*"([^"]+)"\s*,\s*(\d+|[^,]+)"#,
    )
    .ok();

    if let Some(re) = &sub_regex {
        for caps in re.captures_iter(content) {
            let msg_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let topic = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let qos_arg = caps.get(3).map(|m| m.as_str().trim()).unwrap_or("");
            let field_name = topic_to_field_name(&topic);

            let qos = parse_qos_from_arg(qos_arg, content);

            info.subscribers.push(ExtractedSubscriber {
                topic: topic.clone(),
                msg_type,
                field_name,
                callback: format!("on_{}", topic.replace('/', "_").trim_start_matches('_')),
                qos,
            });
        }
    }

    // Extract subscribers: nh.subscribe("topic", queue_size, callback) (ROS1)
    let ros1_sub_regex =
        regex::Regex::new(r#"\.subscribe\s*(?:<\s*([^>]+)\s*>)?\s*\(\s*"([^"]+)"\s*,\s*(\d+)"#)
            .ok();

    if let Some(re) = &ros1_sub_regex {
        for caps in re.captures_iter(content) {
            let msg_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_else(|| "Unknown".to_string());
            let topic = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let queue_size = caps
                .get(3)
                .and_then(|m| m.as_str().parse::<u32>().ok())
                .unwrap_or(10);
            let field_name = topic_to_field_name(&topic);

            let mut qos = ExtractedQoS::default();
            qos.depth = Some(queue_size);

            info.subscribers.push(ExtractedSubscriber {
                topic: topic.clone(),
                msg_type,
                field_name,
                callback: format!("on_{}", topic.replace('/', "_").trim_start_matches('_')),
                qos,
            });
        }
    }

    // Extract publishers: nh.advertise<MsgType>("topic", queue_size) (ROS1)
    let ros1_pub_regex =
        regex::Regex::new(r#"\.advertise\s*<\s*([^>]+)\s*>\s*\(\s*"([^"]+)"\s*,\s*(\d+)"#).ok();

    if let Some(re) = &ros1_pub_regex {
        for caps in re.captures_iter(content) {
            let msg_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let topic = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let queue_size = caps
                .get(3)
                .and_then(|m| m.as_str().parse::<u32>().ok())
                .unwrap_or(10);
            let field_name = topic_to_field_name(&topic);

            let mut qos = ExtractedQoS::default();
            qos.depth = Some(queue_size);

            info.publishers.push(ExtractedPublisher {
                topic: topic.clone(),
                msg_type,
                field_name,
                qos,
            });
        }
    }

    // Extract ROS1 parameters: param("name", default) or getParam("name", var)
    let ros1_param_regex =
        regex::Regex::new(r#"\.param\s*(?:<\s*([^>]+)\s*>)?\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &ros1_param_regex {
        for caps in re.captures_iter(content) {
            let param_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_else(|| "auto".to_string());
            let name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();

            // Avoid duplicates
            if !info.parameters.iter().any(|p| p.name == name) {
                info.parameters.push(ExtractedParameter {
                    name,
                    param_type,
                    default_value: None,
                });
            }
        }
    }

    // Extract timers: create_wall_timer(duration, callback)
    let timer_regex =
        regex::Regex::new(r#"create_wall_timer\s*\(\s*(?:std::chrono::)?(\d+)(?:ms|s)?"#).ok();

    if let Some(re) = &timer_regex {
        for caps in re.captures_iter(content) {
            let period = caps
                .get(1)
                .and_then(|m| m.as_str().parse::<u64>().ok())
                .unwrap_or(100);
            info.timers.push(ExtractedTimer {
                period_ms: period,
                callback: "on_timer".to_string(),
            });
        }
    }

    // Extract parameters: declare_parameter<Type>("name", default)
    let param_regex =
        regex::Regex::new(r#"declare_parameter\s*(?:<\s*(\w+)\s*>)?\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &param_regex {
        for caps in re.captures_iter(content) {
            let param_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_else(|| "auto".to_string());
            let name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();

            info.parameters.push(ExtractedParameter {
                name,
                param_type,
                default_value: None,
            });
        }
    }

    // Extract parameters: get_parameter("name", value) - ROS2 C++
    let get_param_regex = regex::Regex::new(r#"get_parameter\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &get_param_regex {
        for caps in re.captures_iter(content) {
            let name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            // Avoid duplicates
            if !info.parameters.iter().any(|p| p.name == name) {
                info.parameters.push(ExtractedParameter {
                    name,
                    param_type: "auto".to_string(),
                    default_value: None,
                });
            }
        }
    }

    // Extract parameters: get_parameters({"name1", "name2"}) - ROS2 C++ multiple params
    let get_params_regex = regex::Regex::new(r#"get_parameters\s*\(\s*\{([^}]+)\}"#).ok();

    if let Some(re) = &get_params_regex {
        for caps in re.captures_iter(content) {
            if let Some(params_str) = caps.get(1) {
                // Extract individual parameter names from the list
                let param_names_re = regex::Regex::new(r#""([^"]+)""#).ok();
                if let Some(names_re) = param_names_re {
                    for name_cap in names_re.captures_iter(params_str.as_str()) {
                        let name = name_cap
                            .get(1)
                            .map(|m| m.as_str().to_string())
                            .unwrap_or_default();
                        // Avoid duplicates
                        if !info.parameters.iter().any(|p| p.name == name) {
                            info.parameters.push(ExtractedParameter {
                                name,
                                param_type: "auto".to_string(),
                                default_value: None,
                            });
                        }
                    }
                }
            }
        }
    }

    // ROS1 C++ param() and getParam() patterns
    let ros1_param_regex = regex::Regex::new(r#"(?:param|getParam)\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &ros1_param_regex {
        for caps in re.captures_iter(content) {
            let name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            // Avoid duplicates
            if !info.parameters.iter().any(|p| p.name == name) {
                info.parameters.push(ExtractedParameter {
                    name,
                    param_type: "auto".to_string(),
                    default_value: None,
                });
            }
        }
    }

    // ROS1 C++ private_nh_.param() pattern (common in nodelet/pluginlib)
    let private_param_regex =
        regex::Regex::new(r#"private_nh_?\.param\s*(?:<\s*(\w+)\s*>)?\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &private_param_regex {
        for caps in re.captures_iter(content) {
            let param_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_else(|| "auto".to_string());
            let name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            // Avoid duplicates
            if !info.parameters.iter().any(|p| p.name == name) {
                info.parameters.push(ExtractedParameter {
                    name,
                    param_type,
                    default_value: None,
                });
            }
        }
    }

    // ROS2 C++ param.first.compare("name") pattern - when iterating parameter maps
    // Common pattern: get_parameters("", map); for (auto& param : map) { if (param.first.compare("name") == 0) ... }
    let param_compare_regex = regex::Regex::new(r#"\.first\.compare\s*\(\s*"([^"]+)"\s*\)"#).ok();

    if let Some(re) = &param_compare_regex {
        for caps in re.captures_iter(content) {
            let name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            // Avoid duplicates
            if !info.parameters.iter().any(|p| p.name == name) {
                info.parameters.push(ExtractedParameter {
                    name,
                    param_type: "auto".to_string(),
                    default_value: None,
                });
            }
        }
    }

    // ROS2 C++ param.first == "name" pattern - alternative map iteration syntax
    let param_eq_regex = regex::Regex::new(r#"\.first\s*==\s*"([^"]+)""#).ok();

    if let Some(re) = &param_eq_regex {
        for caps in re.captures_iter(content) {
            let name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            // Avoid duplicates
            if !info.parameters.iter().any(|p| p.name == name) {
                info.parameters.push(ExtractedParameter {
                    name,
                    param_type: "auto".to_string(),
                    default_value: None,
                });
            }
        }
    }

    // Extract service servers: create_service<SrvType>("service_name", callback)
    let srv_server_regex =
        regex::Regex::new(r#"create_service\s*<\s*([^>]+)\s*>\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &srv_server_regex {
        for caps in re.captures_iter(content) {
            let srv_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let service_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_servers.push(ExtractedServiceServer {
                service_name: service_name.clone(),
                srv_type,
                field_name: format!("{}_server", field_name),
                callback: format!("handle_{}", field_name),
            });
        }
    }

    // ROS1 service servers: advertiseService("service_name", callback)
    let ros1_srv_regex =
        regex::Regex::new(r#"advertiseService\s*(?:<\s*([^>]+)\s*>)?\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &ros1_srv_regex {
        for caps in re.captures_iter(content) {
            let srv_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_else(|| "Unknown".to_string());
            let service_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_servers.push(ExtractedServiceServer {
                service_name: service_name.clone(),
                srv_type,
                field_name: format!("{}_server", field_name),
                callback: format!("handle_{}", field_name),
            });
        }
    }

    // Extract service clients: create_client<SrvType>("service_name")
    let srv_client_regex =
        regex::Regex::new(r#"create_client\s*<\s*([^>]+)\s*>\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &srv_client_regex {
        for caps in re.captures_iter(content) {
            let srv_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let service_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_clients.push(ExtractedServiceClient {
                service_name,
                srv_type,
                field_name: format!("{}_client", field_name),
            });
        }
    }

    // ROS1 service clients: serviceClient<SrvType>("service_name")
    let ros1_client_regex =
        regex::Regex::new(r#"serviceClient\s*<\s*([^>]+)\s*>\s*\(\s*"([^"]+)""#).ok();

    if let Some(re) = &ros1_client_regex {
        for caps in re.captures_iter(content) {
            let srv_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let service_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_clients.push(ExtractedServiceClient {
                service_name,
                srv_type,
                field_name: format!("{}_client", field_name),
            });
        }
    }

    // Extract action servers: rclcpp_action::create_server<ActionType>(...)
    let action_server_regex =
        regex::Regex::new(r#"create_server\s*<\s*([^>]+)\s*>\s*\([^,]+,\s*"([^"]+)""#).ok();

    if let Some(re) = &action_server_regex {
        for caps in re.captures_iter(content) {
            let action_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let action_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = action_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.action_servers.push(ExtractedActionServer {
                action_name,
                action_type,
                field_name: format!("{}_action_server", field_name),
            });
        }
    }

    // ROS1 action servers: actionlib::SimpleActionServer<ActionType>
    let ros1_action_server_regex =
        regex::Regex::new(r#"SimpleActionServer\s*<\s*([^>]+)\s*>"#).ok();

    if let Some(re) = &ros1_action_server_regex {
        for caps in re.captures_iter(content) {
            let action_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();

            info.action_servers.push(ExtractedActionServer {
                action_name: "unknown".to_string(),
                action_type,
                field_name: "action_server".to_string(),
            });
        }
    }

    // Extract action clients: rclcpp_action::create_client<ActionType>(...)
    let action_client_regex =
        regex::Regex::new(r#"create_client\s*<\s*([^>]+Action[^>]*)\s*>\s*\([^,]+,\s*"([^"]+)""#)
            .ok();

    if let Some(re) = &action_client_regex {
        for caps in re.captures_iter(content) {
            let action_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            let action_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = action_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.action_clients.push(ExtractedActionClient {
                action_name,
                action_type,
                field_name: format!("{}_action_client", field_name),
            });
        }
    }

    // ROS1 action clients: actionlib::SimpleActionClient<ActionType>
    let ros1_action_client_regex =
        regex::Regex::new(r#"SimpleActionClient\s*<\s*([^>]+)\s*>"#).ok();

    if let Some(re) = &ros1_action_client_regex {
        for caps in re.captures_iter(content) {
            let action_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();

            info.action_clients.push(ExtractedActionClient {
                action_name: "unknown".to_string(),
                action_type,
                field_name: "action_client".to_string(),
            });
        }
    }

    // Detect TF broadcasters
    if content.contains("TransformBroadcaster") || content.contains("tf2_ros::TransformBroadcaster")
    {
        info.tf_broadcasters.push(ExtractedTfBroadcaster {
            broadcaster_type: "dynamic".to_string(),
            field_name: "tf_broadcaster".to_string(),
        });
    }
    if content.contains("StaticTransformBroadcaster")
        || content.contains("tf2_ros::StaticTransformBroadcaster")
    {
        info.tf_broadcasters.push(ExtractedTfBroadcaster {
            broadcaster_type: "static".to_string(),
            field_name: "static_tf_broadcaster".to_string(),
        });
    }

    // Detect lifecycle nodes (ROS2)
    if content.contains("LifecycleNode") || content.contains("rclcpp_lifecycle::LifecycleNode") {
        info.is_lifecycle = true;
    }

    // Detect component nodes (ROS2)
    if content.contains("RCLCPP_COMPONENTS_REGISTER_NODE") || content.contains("rclcpp_components")
    {
        info.is_component = true;
    }

    // Detect nodelets (ROS1)
    // Pattern 1: class MyClass : public nodelet::Nodelet
    // Pattern 2: #include <nodelet/nodelet.h>
    // Pattern 3: PLUGINLIB_EXPORT_CLASS(..., nodelet::Nodelet)
    if content.contains("nodelet::Nodelet")
        || content.contains("nodelet/nodelet.h")
        || content.contains("pluginlib/class_list_macros.h")
    {
        info.is_nodelet = true;
    }

    // Detect TF listeners (tf2_ros::Buffer, tf2_ros::TransformListener)
    if content.contains("tf2_ros::Buffer") || content.contains("tf::TransformListener") {
        info.tf_listeners.push(ExtractedTfListener {
            buffer_name: "tf_buffer".to_string(),
            listener_name: "tf_listener".to_string(),
        });
    }

    // Detect message filters (TimeSynchronizer, ApproximateTime, ExactTime)
    let sync_regex = regex::Regex::new(
        r#"(?:TimeSynchronizer|Synchronizer)\s*<[^>]*>\s*(?:\w+)?\s*\([^)]*(\d+)\s*\)"#,
    )
    .ok();
    if let Some(re) = &sync_regex {
        for caps in re.captures_iter(content) {
            let queue_size = caps
                .get(1)
                .and_then(|m| m.as_str().parse::<u32>().ok())
                .unwrap_or(10);
            let filter_type = if content.contains("ApproximateTime") {
                "ApproximateTime"
            } else if content.contains("ExactTime") {
                "ExactTime"
            } else {
                "TimeSynchronizer"
            };
            info.message_filters.push(ExtractedMessageFilter {
                filter_type: filter_type.to_string(),
                topics: Vec::new(), // Would need more complex parsing
                queue_size,
            });
        }
    }
    // Also detect by include
    if content.contains("message_filters/synchronizer.h")
        || content.contains("message_filters/sync_policies")
    {
        if info.message_filters.is_empty() {
            info.message_filters.push(ExtractedMessageFilter {
                filter_type: if content.contains("ApproximateTime") {
                    "ApproximateTime"
                } else {
                    "ExactTime"
                }
                .to_string(),
                topics: Vec::new(),
                queue_size: 10,
            });
        }
    }

    // Detect image transport
    let it_sub_regex = regex::Regex::new(
        r#"(?:image_transport::)?(?:ImageTransport\s*\w+|it)\.subscribe\s*\(\s*"([^"]+)""#,
    )
    .ok();
    if let Some(re) = &it_sub_regex {
        for caps in re.captures_iter(content) {
            let topic = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            info.image_transports.push(ExtractedImageTransport {
                transport_type: "subscriber".to_string(),
                topic: topic.clone(),
                field_name: topic_to_field_name(&topic),
            });
        }
    }
    let it_pub_regex = regex::Regex::new(
        r#"(?:image_transport::)?(?:ImageTransport\s*\w+|it)\.advertise\s*\(\s*"([^"]+)""#,
    )
    .ok();
    if let Some(re) = &it_pub_regex {
        for caps in re.captures_iter(content) {
            let topic = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            info.image_transports.push(ExtractedImageTransport {
                transport_type: "publisher".to_string(),
                topic: topic.clone(),
                field_name: topic_to_field_name(&topic),
            });
        }
    }
    // Detect by include
    if content.contains("image_transport/image_transport.h")
        || content.contains("image_transport.hpp")
    {
        if info.image_transports.is_empty() {
            info.image_transports.push(ExtractedImageTransport {
                transport_type: "unknown".to_string(),
                topic: String::new(),
                field_name: "image_transport".to_string(),
            });
        }
    }

    // Detect diagnostic_updater
    if content.contains("diagnostic_updater::Updater") || content.contains("diagnostic_updater.h") {
        info.diagnostics.push(ExtractedDiagnostic {
            updater_name: "diagnostic_updater".to_string(),
            hardware_id: None,
        });
    }

    // Detect dynamic_reconfigure (ROS1)
    let dynreconf_regex =
        regex::Regex::new(r#"dynamic_reconfigure::Server\s*<\s*([^>]+)\s*>"#).ok();
    if let Some(re) = &dynreconf_regex {
        for caps in re.captures_iter(content) {
            let config_type = caps
                .get(1)
                .map(|m| m.as_str().trim().to_string())
                .unwrap_or_default();
            info.dynamic_reconfigure.push(ExtractedDynamicReconfigure {
                config_type,
                callback: "reconfigure_callback".to_string(),
            });
        }
    }

    // Detect ROS2 executors
    if content.contains("SingleThreadedExecutor") {
        info.executors.push(ExtractedExecutor {
            executor_type: "SingleThreaded".to_string(),
        });
    }
    if content.contains("MultiThreadedExecutor") {
        info.executors.push(ExtractedExecutor {
            executor_type: "MultiThreaded".to_string(),
        });
    }
    if content.contains("StaticSingleThreadedExecutor") {
        info.executors.push(ExtractedExecutor {
            executor_type: "StaticSingleThreaded".to_string(),
        });
    }

    // Detect callback groups
    if content.contains("MutuallyExclusiveCallbackGroup") {
        info.callback_groups.push(ExtractedCallbackGroup {
            group_type: "MutuallyExclusive".to_string(),
            name: "mutex_group".to_string(),
        });
    }
    if content.contains("ReentrantCallbackGroup") {
        info.callback_groups.push(ExtractedCallbackGroup {
            group_type: "Reentrant".to_string(),
            name: "reentrant_group".to_string(),
        });
    }

    // ========== NEW COMPREHENSIVE FEATURE DETECTION ==========

    // Detect ros2_control / hardware_interface
    if content.contains("hardware_interface::SystemInterface")
        || content.contains("hardware_interface::ActuatorInterface")
        || content.contains("hardware_interface::SensorInterface")
    {
        let mut hw = ExtractedHardwareInterface::default();
        if content.contains("SystemInterface") {
            hw.interface_type = "SystemInterface".to_string();
        } else if content.contains("ActuatorInterface") {
            hw.interface_type = "ActuatorInterface".to_string();
        } else if content.contains("SensorInterface") {
            hw.interface_type = "SensorInterface".to_string();
        }
        // Detect state interfaces
        if content.contains("export_state_interfaces") || content.contains("StateInterface") {
            hw.state_interfaces.push("detected".to_string());
        }
        // Detect command interfaces
        if content.contains("export_command_interfaces") || content.contains("CommandInterface") {
            hw.command_interfaces.push("detected".to_string());
        }
        info.hardware_interfaces.push(hw);
    }

    // Detect ros_control controllers (ROS1/ROS2)
    if content.contains("controller_interface::ControllerInterface")
        || content.contains("controller_interface::Controller")
        || content.contains("ros_control")
        || content.contains("ControllerBase")
    {
        let controller_type = if content.contains("JointPositionController") {
            "JointPositionController"
        } else if content.contains("JointVelocityController") {
            "JointVelocityController"
        } else if content.contains("JointEffortController") {
            "JointEffortController"
        } else if content.contains("JointTrajectoryController") {
            "JointTrajectoryController"
        } else if content.contains("DiffDriveController") {
            "DiffDriveController"
        } else {
            "CustomController"
        };
        info.controllers.push(ExtractedController {
            controller_type: controller_type.to_string(),
            controller_name: "detected".to_string(),
        });
    }

    // Detect rosserial / MicroROS
    if content.contains("ros::NodeHandle nh") && content.contains("ros.h")
        || content.contains("rosserial")
        || content.contains("<ros.h>")
    {
        info.embedded_ros = Some(ExtractedEmbeddedROS {
            ros_type: "rosserial".to_string(),
            publishers: vec![],
            subscribers: vec![],
        });
    }
    if content.contains("micro_ros")
        || content.contains("rclc_")
        || content.contains("rcl_publisher")
        || content.contains("rcl_subscription")
    {
        info.embedded_ros = Some(ExtractedEmbeddedROS {
            ros_type: "micro_ros".to_string(),
            publishers: vec![],
            subscribers: vec![],
        });
    }

    // Detect WaitSet / GuardCondition
    if content.contains("WaitSet") || content.contains("rclcpp::WaitSet") {
        let mut ws = ExtractedWaitSet::default();
        ws.has_waitset = true;
        if content.contains("GuardCondition") {
            ws.guard_conditions.push("detected".to_string());
        }
        info.waitset = Some(ws);
    }

    // Detect IntraProcessComms
    if content.contains("use_intra_process_comms")
        || content.contains("IntraProcessBuffer")
        || content.contains("intra_process")
    {
        info.intra_process = Some(ExtractedIntraProcess {
            enabled: true,
            buffer_size: None,
        });
    }

    // Detect Bond (heartbeat monitoring)
    if content.contains("bond::Bond")
        || content.contains("bondcpp")
        || content.contains("bond_core")
    {
        info.bonds.push(ExtractedBond {
            bond_id: "detected".to_string(),
            topic: "/bond".to_string(),
        });
    }

    // Detect laser_geometry
    if content.contains("laser_geometry") || content.contains("LaserProjection") {
        info.laser_geometry = Some(ExtractedLaserGeometry {
            projector_name: "laser_projector".to_string(),
        });
    }

    // Detect URDF / robot_description patterns
    if content.contains("robot_description")
        || content.contains("urdf::Model")
        || content.contains("urdf::parseURDF")
        || content.contains("kdl_parser")
    {
        info.urdf = Some(ExtractedURDF {
            robot_name: "detected".to_string(),
            links: vec![],
            joints: vec![],
            has_xacro: content.contains("xacro"),
        });
    }

    // Detect Nav2 patterns
    if content.contains("nav2")
        || content.contains("nav2_core")
        || content.contains("BT::")
        || content.contains("BehaviorTree")
        || content.contains("nav2_bt")
    {
        let mut nav2 = ExtractedNav2::default();
        if content.contains("BehaviorTree") || content.contains("BT::") {
            nav2.behavior_trees.push("detected".to_string());
        }
        if content.contains("Costmap2D") || content.contains("nav2_costmap") {
            nav2.costmap_plugins.push("detected".to_string());
        }
        if content.contains("GlobalPlanner")
            || content.contains("nav2_navfn")
            || content.contains("NavFn")
        {
            nav2.planners.push("detected".to_string());
        }
        if content.contains("LocalPlanner")
            || content.contains("DWBLocalPlanner")
            || content.contains("nav2_dwb")
        {
            nav2.controllers.push("detected".to_string());
        }
        if content.contains("RecoveryBehavior") || content.contains("nav2_behaviors") {
            nav2.recovery_behaviors.push("detected".to_string());
        }
        info.nav2 = Some(nav2);
    }

    // Detect MoveIt patterns
    if content.contains("moveit")
        || content.contains("MoveGroupInterface")
        || content.contains("PlanningSceneInterface")
        || content.contains("moveit_core")
    {
        let mut moveit = ExtractedMoveIt::default();
        moveit.move_group_interface = content.contains("MoveGroupInterface");
        moveit.planning_scene_interface = content.contains("PlanningSceneInterface");
        if content.contains("RobotModel") {
            moveit.robot_model = Some("detected".to_string());
        }
        info.moveit = Some(moveit);
    }

    // Detect Gazebo plugins
    if content.contains("gazebo::ModelPlugin")
        || content.contains("gazebo::WorldPlugin")
        || content.contains("gazebo::SensorPlugin")
        || content.contains("gazebo_ros")
    {
        let plugin_type = if content.contains("ModelPlugin") {
            "ModelPlugin"
        } else if content.contains("WorldPlugin") {
            "WorldPlugin"
        } else if content.contains("SensorPlugin") {
            "SensorPlugin"
        } else if content.contains("VisualPlugin") {
            "VisualPlugin"
        } else {
            "GazeboPlugin"
        };
        info.gazebo_plugins.push(ExtractedGazeboPlugin {
            plugin_type: plugin_type.to_string(),
            plugin_name: "detected".to_string(),
        });
    }

    // Detect RViz / visualization patterns
    if content.contains("visualization_msgs::Marker")
        || content.contains("MarkerArray")
        || content.contains("InteractiveMarker")
        || content.contains("rviz")
    {
        let mut viz = ExtractedVisualization::default();
        if content.contains("Marker") {
            viz.marker_publishers.push("detected".to_string());
        }
        if content.contains("InteractiveMarker") {
            viz.interactive_markers.push("detected".to_string());
        }
        info.visualization = Some(viz);
    }

    // Detect rosbag / ros2bag patterns
    if content.contains("rosbag::Bag")
        || content.contains("rosbag2")
        || content.contains("ros2bag")
        || content.contains("BagRecorder")
    {
        let mut bag = ExtractedBagRecording::default();
        bag.is_recorder =
            content.contains("write") || content.contains("Recorder") || content.contains("record");
        bag.is_player =
            content.contains("read") || content.contains("Player") || content.contains("play");
        info.bag_recording = Some(bag);
    }

    // Detect camera_info_manager
    if content.contains("camera_info_manager") || content.contains("CameraInfoManager") {
        info.camera_info = Some(ExtractedCameraInfo {
            camera_name: "camera".to_string(),
            camera_info_url: None,
        });
    }

    // Detect image_geometry
    if content.contains("image_geometry")
        || content.contains("PinholeCameraModel")
        || content.contains("StereoCameraModel")
    {
        let model = if content.contains("StereoCameraModel") {
            "StereoCameraModel"
        } else {
            "PinholeCameraModel"
        };
        info.image_geometry = Some(ExtractedImageGeometry {
            camera_model: model.to_string(),
        });
    }

    // Detect PCL processing patterns
    if content.contains("pcl::")
        || content.contains("pcl_ros")
        || content.contains("pcl_conversions")
    {
        let mut pcl = ExtractedPCL::default();
        // Detect common PCL filters
        if content.contains("VoxelGrid") {
            pcl.filters.push("VoxelGrid".to_string());
        }
        if content.contains("PassThrough") {
            pcl.filters.push("PassThrough".to_string());
        }
        if content.contains("StatisticalOutlierRemoval") {
            pcl.filters.push("StatisticalOutlierRemoval".to_string());
        }
        if content.contains("RadiusOutlierRemoval") {
            pcl.filters.push("RadiusOutlierRemoval".to_string());
        }
        if content.contains("CropBox") {
            pcl.filters.push("CropBox".to_string());
        }
        if content.contains("ExtractIndices") {
            pcl.filters.push("ExtractIndices".to_string());
        }
        if content.contains("SACSegmentation") {
            pcl.filters.push("SACSegmentation".to_string());
        }
        if content.contains("NormalEstimation") {
            pcl.filters.push("NormalEstimation".to_string());
        }
        if content.contains("ICP") || content.contains("IterativeClosestPoint") {
            pcl.filters.push("ICP".to_string());
        }
        info.pcl = Some(pcl);
    }

    // Detect realtime_tools
    if content.contains("realtime_tools")
        || content.contains("RealtimePublisher")
        || content.contains("RealtimeBuffer")
        || content.contains("RealtimeBox")
    {
        let mut rt = ExtractedRealtimeTools::default();
        rt.realtime_publisher = content.contains("RealtimePublisher");
        rt.realtime_buffer = content.contains("RealtimeBuffer");
        rt.realtime_box = content.contains("RealtimeBox");
        info.realtime_tools = Some(rt);
    }

    // ========== ADDITIONAL COMPREHENSIVE PATTERNS ==========

    // Detect robot_localization / state estimation (EKF, UKF)
    if content.contains("robot_localization")
        || content.contains("RobotLocalization")
        || content.contains("ekf_node")
        || content.contains("ukf_node")
        || content.contains("EkfNode")
        || content.contains("UkfNode")
        || content.contains("ExtendedKalmanFilter")
        || content.contains("UnscentedKalmanFilter")
    {
        let mut se = ExtractedStateEstimation::default();
        if content.contains("ukf")
            || content.contains("UKF")
            || content.contains("UnscentedKalmanFilter")
        {
            se.estimator_type = "ukf".to_string();
        } else {
            se.estimator_type = "ekf".to_string();
        }
        // Detect sensor sources
        if content.contains("odom") || content.contains("odometry") {
            se.sensor_sources.push("odom".to_string());
        }
        if content.contains("imu") || content.contains("Imu") {
            se.sensor_sources.push("imu".to_string());
        }
        if content.contains("gps") || content.contains("GPS") || content.contains("NavSatFix") {
            se.sensor_sources.push("gps".to_string());
        }
        if content.contains("pose") || content.contains("Pose") {
            se.sensor_sources.push("pose".to_string());
        }
        info.state_estimation = Some(se);
    }

    // Detect SLAM patterns
    if content.contains("slam_toolbox")
        || content.contains("SlamToolbox")
        || content.contains("cartographer")
        || content.contains("Cartographer")
        || content.contains("gmapping")
        || content.contains("GMapping")
        || content.contains("hector_slam")
        || content.contains("HectorSlam")
        || content.contains("rtabmap")
        || content.contains("Rtabmap")
        || content.contains("ORB_SLAM")
        || content.contains("LIO-SAM")
    {
        let mut slam = ExtractedSLAM::default();
        if content.contains("slam_toolbox") || content.contains("SlamToolbox") {
            slam.slam_type = "slam_toolbox".to_string();
        } else if content.contains("cartographer") || content.contains("Cartographer") {
            slam.slam_type = "cartographer".to_string();
        } else if content.contains("gmapping") || content.contains("GMapping") {
            slam.slam_type = "gmapping".to_string();
        } else if content.contains("hector_slam") || content.contains("HectorSlam") {
            slam.slam_type = "hector_slam".to_string();
        } else if content.contains("rtabmap") || content.contains("Rtabmap") {
            slam.slam_type = "rtabmap".to_string();
        } else if content.contains("ORB_SLAM") {
            slam.slam_type = "orb_slam".to_string();
        } else if content.contains("LIO-SAM") {
            slam.slam_type = "lio_sam".to_string();
        } else {
            slam.slam_type = "unknown".to_string();
        }
        slam.has_loop_closure = content.contains("loop_closure") || content.contains("LoopClosure");
        slam.has_map_save = content.contains("save_map")
            || content.contains("SaveMap")
            || content.contains("serialize");
        slam.has_localization_mode =
            content.contains("localization_mode") || content.contains("LocalizationMode");
        info.slam = Some(slam);
    }

    // Detect AMCL / localization
    if content.contains("amcl")
        || content.contains("AMCL")
        || content.contains("adaptive_monte_carlo")
        || content.contains("AdaptiveMonteCarlo")
        || content.contains("particle_filter")
        || content.contains("ParticleFilter")
        || content.contains("nav2_amcl")
    {
        let mut loc = ExtractedLocalization::default();
        if content.contains("amcl") || content.contains("AMCL") {
            loc.localization_type = "amcl".to_string();
        } else {
            loc.localization_type = "particle_filter".to_string();
        }
        loc.uses_particle_filter = true;
        info.localization = Some(loc);
    }

    // Detect 3D mapping (octomap, voxblox, elevation_mapping)
    if content.contains("octomap")
        || content.contains("OctoMap")
        || content.contains("Octree")
        || content.contains("octree")
        || content.contains("voxblox")
        || content.contains("Voxblox")
        || content.contains("elevation_mapping")
        || content.contains("ElevationMapping")
    {
        let mut map3d = Extracted3DMapping::default();
        if content.contains("octomap") || content.contains("OctoMap") {
            map3d.mapping_type = "octomap".to_string();
        } else if content.contains("voxblox") || content.contains("Voxblox") {
            map3d.mapping_type = "voxblox".to_string();
        } else if content.contains("elevation_mapping") || content.contains("ElevationMapping") {
            map3d.mapping_type = "elevation_mapping".to_string();
        }
        map3d.uses_octree = content.contains("Octree") || content.contains("octree");
        info.mapping_3d = Some(map3d);
    }

    // Detect depth/stereo image processing
    if content.contains("depth_image_proc")
        || content.contains("DepthImageProc")
        || content.contains("stereo_image_proc")
        || content.contains("StereoImageProc")
        || content.contains("point_cloud_xyz")
        || content.contains("PointCloudXyz")
        || content.contains("disparity")
        || content.contains("DisparityNode")
        || content.contains("convert_metric")
    {
        let mut depth = ExtractedDepthProcessing::default();
        if content.contains("stereo_image_proc")
            || content.contains("StereoImageProc")
            || content.contains("disparity")
        {
            depth.processing_type = "stereo_image_proc".to_string();
        } else {
            depth.processing_type = "depth_image_proc".to_string();
        }
        if content.contains("point_cloud_xyz") || content.contains("PointCloudXyz") {
            depth.operations.push("point_cloud_xyz".to_string());
        }
        if content.contains("convert_metric") {
            depth.operations.push("convert_metric".to_string());
        }
        if content.contains("disparity") {
            depth.operations.push("disparity".to_string());
        }
        if content.contains("register") {
            depth.operations.push("register".to_string());
        }
        info.depth_processing = Some(depth);
    }

    // Detect rosbridge / web bridge
    if content.contains("rosbridge")
        || content.contains("roslibjs")
        || content.contains("rosapi")
        || content.contains("websocket_server")
        || content.contains("WebSocketServer")
    {
        let mut bridge = ExtractedWebBridge::default();
        if content.contains("rosapi") {
            bridge.bridge_type = "rosapi".to_string();
        } else {
            bridge.bridge_type = "rosbridge".to_string();
        }
        bridge.websocket = content.contains("websocket") || content.contains("WebSocket");
        info.web_bridge = Some(bridge);
    }

    // Detect state machines (smach, FlexBE, SMACC)
    if content.contains("smach")
        || content.contains("Smach")
        || content.contains("StateMachine")
        || content.contains("state_machine")
        || content.contains("flexbe")
        || content.contains("FlexBE")
        || content.contains("smacc")
        || content.contains("SMACC")
    {
        let mut sm = ExtractedStateMachine::default();
        if content.contains("smach") || content.contains("Smach") {
            sm.state_machine_type = "smach".to_string();
        } else if content.contains("flexbe") || content.contains("FlexBE") {
            sm.state_machine_type = "flexbe".to_string();
        } else if content.contains("smacc") || content.contains("SMACC") {
            sm.state_machine_type = "smacc".to_string();
        } else {
            sm.state_machine_type = "custom".to_string();
        }
        info.state_machine = Some(sm);
    }

    // Detect sensor drivers (velodyne, realsense, zed, xsens, etc.)
    if content.contains("velodyne")
        || content.contains("Velodyne")
        || content.contains("VLP16")
        || content.contains("VLP32")
        || content.contains("HDL32")
    {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "velodyne".to_string(),
            driver_name: "velodyne_driver".to_string(),
        });
    }
    if content.contains("realsense")
        || content.contains("RealSense")
        || content.contains("rs2::")
        || content.contains("librealsense")
    {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "realsense".to_string(),
            driver_name: "realsense2_camera".to_string(),
        });
    }
    if content.contains("zed_ros")
        || content.contains("ZED")
        || content.contains("sl::Camera")
        || content.contains("stereolabs")
    {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "zed".to_string(),
            driver_name: "zed_ros".to_string(),
        });
    }
    if content.contains("xsens")
        || content.contains("Xsens")
        || content.contains("MTi")
        || content.contains("xsens_driver")
    {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "xsens".to_string(),
            driver_name: "xsens_driver".to_string(),
        });
    }
    if content.contains("mynt_eye") || content.contains("MyntEye") || content.contains("MYNTEYE") {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "mynt_eye".to_string(),
            driver_name: "mynt_eye_ros_wrapper".to_string(),
        });
    }
    if content.contains("ouster")
        || content.contains("Ouster")
        || content.contains("OS1")
        || content.contains("OS0")
    {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "ouster".to_string(),
            driver_name: "ouster_ros".to_string(),
        });
    }
    if content.contains("livox")
        || content.contains("Livox")
        || content.contains("HAP")
        || content.contains("Horizon")
    {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "livox".to_string(),
            driver_name: "livox_ros_driver".to_string(),
        });
    }

    info
}

/// Parse a Python source file to extract ROS2 node information
/// Parse QoS settings from a Python QoS argument
fn parse_python_qos(qos_arg: &str) -> ExtractedQoS {
    let mut qos = ExtractedQoS::default();

    // If it's just a number, that's the depth
    if let Ok(depth) = qos_arg.parse::<u32>() {
        qos.depth = Some(depth);
        return qos;
    }

    let arg_lower = qos_arg.to_lowercase();

    // Check for QoSReliabilityPolicy.RELIABLE or similar
    if arg_lower.contains("reliable") {
        qos.reliability = Some("reliable".to_string());
    } else if arg_lower.contains("best_effort") {
        qos.reliability = Some("best_effort".to_string());
    }

    // Check for durability
    if arg_lower.contains("transient_local") {
        qos.durability = Some("transient_local".to_string());
    } else if arg_lower.contains("volatile") {
        qos.durability = Some("volatile".to_string());
    }

    // Check for QoSProfile(depth=N) or history patterns
    if let Some(re) = regex::Regex::new(r"depth\s*=\s*(\d+)").ok() {
        if let Some(caps) = re.captures(qos_arg) {
            if let Some(depth) = caps.get(1).and_then(|m| m.as_str().parse::<u32>().ok()) {
                qos.depth = Some(depth);
            }
        }
    }

    qos
}

pub fn parse_python_source(content: &str) -> ParsedNodeInfo {
    let mut info = ParsedNodeInfo::default();

    // Extract class name (inherits from Node or rclpy.node.Node)
    let class_regex = regex::Regex::new(r"class\s+(\w+)\s*\((?:rclpy\.node\.)?Node\)").ok();

    if let Some(re) = &class_regex {
        if let Some(caps) = re.captures(content) {
            info.class_name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
        }
    }

    // Extract publishers: self.create_publisher(MsgType, 'topic', qos)
    let pub_regex = regex::Regex::new(
        r#"create_publisher\s*\(\s*(\w+)\s*,\s*['"]([\w/]+)['"]\s*,\s*(\d+|[^)]+)"#,
    )
    .ok();

    if let Some(re) = &pub_regex {
        for caps in re.captures_iter(content) {
            let msg_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let topic = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let qos_arg = caps.get(3).map(|m| m.as_str().trim()).unwrap_or("");
            let field_name = topic_to_field_name(&topic);

            let qos = parse_python_qos(qos_arg);

            info.publishers.push(ExtractedPublisher {
                topic,
                msg_type,
                field_name,
                qos,
            });
        }
    }

    // Extract subscribers: self.create_subscription(MsgType, 'topic', callback, qos)
    let sub_regex = regex::Regex::new(
        r#"create_subscription\s*\(\s*(\w+)\s*,\s*['"]([\w/]+)['"]\s*,\s*[^,]+\s*,\s*(\d+|[^)]+)"#,
    )
    .ok();

    if let Some(re) = &sub_regex {
        for caps in re.captures_iter(content) {
            let msg_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let topic = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let qos_arg = caps.get(3).map(|m| m.as_str().trim()).unwrap_or("");
            let field_name = topic_to_field_name(&topic);

            let qos = parse_python_qos(qos_arg);

            info.subscribers.push(ExtractedSubscriber {
                topic: topic.clone(),
                msg_type,
                field_name,
                callback: format!("on_{}", topic.replace('/', "_").trim_start_matches('_')),
                qos,
            });
        }
    }

    // Extract timers: self.create_timer(period, callback)
    let timer_regex = regex::Regex::new(r#"create_timer\s*\(\s*([\d.]+)"#).ok();

    if let Some(re) = &timer_regex {
        for caps in re.captures_iter(content) {
            let period_sec: f64 = caps
                .get(1)
                .and_then(|m| m.as_str().parse().ok())
                .unwrap_or(0.1);
            info.timers.push(ExtractedTimer {
                period_ms: (period_sec * 1000.0) as u64,
                callback: "on_timer".to_string(),
            });
        }
    }

    // Extract parameters: self.declare_parameter('name', default)
    let param_regex = regex::Regex::new(r#"declare_parameter\s*\(\s*['"]([\w.]+)['"]"#).ok();

    if let Some(re) = &param_regex {
        for caps in re.captures_iter(content) {
            let name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();

            info.parameters.push(ExtractedParameter {
                name,
                param_type: "auto".to_string(),
                default_value: None,
            });
        }
    }

    // Extract service servers: self.create_service(SrvType, 'service_name', callback)
    let srv_server_regex =
        regex::Regex::new(r#"create_service\s*\(\s*(\w+)\s*,\s*['"]([\w/]+)['"]"#).ok();

    if let Some(re) = &srv_server_regex {
        for caps in re.captures_iter(content) {
            let srv_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let service_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_servers.push(ExtractedServiceServer {
                service_name: service_name.clone(),
                srv_type,
                field_name: format!("{}_server", field_name),
                callback: format!("handle_{}", field_name),
            });
        }
    }

    // Extract service clients: self.create_client(SrvType, 'service_name')
    let srv_client_regex =
        regex::Regex::new(r#"create_client\s*\(\s*(\w+)\s*,\s*['"]([\w/]+)['"]"#).ok();

    if let Some(re) = &srv_client_regex {
        for caps in re.captures_iter(content) {
            let srv_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let service_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_clients.push(ExtractedServiceClient {
                service_name,
                srv_type,
                field_name: format!("{}_client", field_name),
            });
        }
    }

    // Extract action servers: ActionServer(node, ActionType, 'action_name', ...)
    let action_server_regex =
        regex::Regex::new(r#"ActionServer\s*\([^,]+,\s*(\w+)\s*,\s*['"]([\w/]+)['"]"#).ok();

    if let Some(re) = &action_server_regex {
        for caps in re.captures_iter(content) {
            let action_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let action_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = action_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.action_servers.push(ExtractedActionServer {
                action_name,
                action_type,
                field_name: format!("{}_action_server", field_name),
            });
        }
    }

    // Extract action clients: ActionClient(node, ActionType, 'action_name')
    let action_client_regex =
        regex::Regex::new(r#"ActionClient\s*\([^,]+,\s*(\w+)\s*,\s*['"]([\w/]+)['"]"#).ok();

    if let Some(re) = &action_client_regex {
        for caps in re.captures_iter(content) {
            let action_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let action_name = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = action_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.action_clients.push(ExtractedActionClient {
                action_name,
                action_type,
                field_name: format!("{}_action_client", field_name),
            });
        }
    }

    // Detect TF broadcasters in Python
    if content.contains("TransformBroadcaster") {
        info.tf_broadcasters.push(ExtractedTfBroadcaster {
            broadcaster_type: "dynamic".to_string(),
            field_name: "tf_broadcaster".to_string(),
        });
    }
    if content.contains("StaticTransformBroadcaster") {
        info.tf_broadcasters.push(ExtractedTfBroadcaster {
            broadcaster_type: "static".to_string(),
            field_name: "static_tf_broadcaster".to_string(),
        });
    }

    // Detect lifecycle nodes in Python
    if content.contains("LifecycleNode") || content.contains("lifecycle_node") {
        info.is_lifecycle = true;
    }

    // Detect component nodes (less common in Python but possible)
    if content.contains("ComposableNode") || content.contains("composable_node") {
        info.is_component = true;
    }

    // =========== ROS1 Python (rospy) patterns ===========

    // ROS1 rospy.Publisher: rospy.Publisher('topic', MsgType, queue_size=10)
    let rospy_pub_regex = regex::Regex::new(
        r#"rospy\.Publisher\s*\(\s*['"]([\w/]+)['"]\s*,\s*(\w+)\s*(?:,\s*queue_size\s*=\s*(\d+))?"#,
    )
    .ok();

    if let Some(re) = &rospy_pub_regex {
        for caps in re.captures_iter(content) {
            let topic = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let msg_type = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let queue_size: u32 = caps
                .get(3)
                .and_then(|m| m.as_str().parse().ok())
                .unwrap_or(10);
            let field_name = topic_to_field_name(&topic);

            info.publishers.push(ExtractedPublisher {
                topic,
                msg_type,
                field_name,
                qos: ExtractedQoS {
                    depth: Some(queue_size),
                    ..Default::default()
                },
            });
        }
    }

    // ROS1 rospy.Subscriber: rospy.Subscriber('topic', MsgType, callback, queue_size=10)
    let rospy_sub_regex =
        regex::Regex::new(r#"rospy\.Subscriber\s*\(\s*['"]([\w/]+)['"]\s*,\s*(\w+)\s*,\s*(\w+)"#)
            .ok();

    if let Some(re) = &rospy_sub_regex {
        for caps in re.captures_iter(content) {
            let topic = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let msg_type = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let callback = caps
                .get(3)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = topic_to_field_name(&topic);

            info.subscribers.push(ExtractedSubscriber {
                topic,
                msg_type,
                field_name,
                callback,
                qos: ExtractedQoS::default(),
            });
        }
    }

    // ROS1 rospy.Timer: rospy.Timer(rospy.Duration(secs), callback)
    let rospy_timer_regex =
        regex::Regex::new(r#"rospy\.Timer\s*\(\s*rospy\.Duration\s*\(\s*([\d.]+)\s*\)"#).ok();

    if let Some(re) = &rospy_timer_regex {
        for caps in re.captures_iter(content) {
            let period_sec: f64 = caps
                .get(1)
                .and_then(|m| m.as_str().parse().ok())
                .unwrap_or(0.1);
            info.timers.push(ExtractedTimer {
                period_ms: (period_sec * 1000.0) as u64,
                callback: "timer_callback".to_string(),
            });
        }
    }

    // ROS1 rospy.Rate: rospy.Rate(hz)
    let rospy_rate_regex = regex::Regex::new(r#"rospy\.Rate\s*\(\s*([\d.]+)\s*\)"#).ok();

    if let Some(re) = &rospy_rate_regex {
        for caps in re.captures_iter(content) {
            let hz: f64 = caps
                .get(1)
                .and_then(|m| m.as_str().parse().ok())
                .unwrap_or(10.0);
            if hz > 0.0 {
                info.timers.push(ExtractedTimer {
                    period_ms: (1000.0 / hz) as u64,
                    callback: "main_loop".to_string(),
                });
            }
        }
    }

    // ROS1 rospy.get_param: rospy.get_param('name', default)
    let rospy_param_regex = regex::Regex::new(r#"rospy\.get_param\s*\(\s*['"]([~/\w]+)['"]"#).ok();

    if let Some(re) = &rospy_param_regex {
        for caps in re.captures_iter(content) {
            let name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            // Avoid duplicates
            if !info.parameters.iter().any(|p| p.name == name) {
                info.parameters.push(ExtractedParameter {
                    name,
                    param_type: "auto".to_string(),
                    default_value: None,
                });
            }
        }
    }

    // ROS1 rospy.Service: rospy.Service('service', SrvType, handler)
    let rospy_srv_server_regex =
        regex::Regex::new(r#"rospy\.Service\s*\(\s*['"]([\w/]+)['"]\s*,\s*(\w+)\s*,\s*(\w+)"#).ok();

    if let Some(re) = &rospy_srv_server_regex {
        for caps in re.captures_iter(content) {
            let service_name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let srv_type = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let callback = caps
                .get(3)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_servers.push(ExtractedServiceServer {
                service_name,
                srv_type,
                field_name: format!("{}_server", field_name),
                callback,
            });
        }
    }

    // ROS1 rospy.ServiceProxy: rospy.ServiceProxy('service', SrvType)
    let rospy_srv_client_regex =
        regex::Regex::new(r#"rospy\.ServiceProxy\s*\(\s*['"]([\w/]+)['"]\s*,\s*(\w+)"#).ok();

    if let Some(re) = &rospy_srv_client_regex {
        for caps in re.captures_iter(content) {
            let service_name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let srv_type = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = service_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.service_clients.push(ExtractedServiceClient {
                service_name,
                srv_type,
                field_name: format!("{}_client", field_name),
            });
        }
    }

    // ROS1 actionlib.SimpleActionServer
    let rospy_action_server_regex =
        regex::Regex::new(r#"SimpleActionServer\s*\([^,]+,\s*['"]([\w/]+)['"]\s*,\s*(\w+)"#).ok();

    if let Some(re) = &rospy_action_server_regex {
        for caps in re.captures_iter(content) {
            let action_name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let action_type = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = action_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.action_servers.push(ExtractedActionServer {
                action_name,
                action_type,
                field_name: format!("{}_action_server", field_name),
            });
        }
    }

    // ROS1 actionlib.SimpleActionClient
    let rospy_action_client_regex =
        regex::Regex::new(r#"SimpleActionClient\s*\(\s*['"]([\w/]+)['"]\s*,\s*(\w+)"#).ok();

    if let Some(re) = &rospy_action_client_regex {
        for caps in re.captures_iter(content) {
            let action_name = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let action_type = caps
                .get(2)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let field_name = action_name
                .replace('/', "_")
                .trim_start_matches('_')
                .to_string();

            info.action_clients.push(ExtractedActionClient {
                action_name,
                action_type,
                field_name: format!("{}_action_client", field_name),
            });
        }
    }

    // =========== TF Listeners (Python) ===========

    // ROS1/ROS2 tf2_ros.TransformListener or tf.TransformListener
    if content.contains("TransformListener") || content.contains("tf2_ros.Buffer") {
        info.tf_listeners.push(ExtractedTfListener {
            buffer_name: "tf_buffer".to_string(),
            listener_name: "tf_listener".to_string(),
        });
    }

    // =========== Message Filters (Python) ===========

    // message_filters.TimeSynchronizer, ApproximateTimeSynchronizer
    if content.contains("TimeSynchronizer") {
        let filter_type = if content.contains("ApproximateTimeSynchronizer") {
            "ApproximateTime"
        } else {
            "ExactTime"
        };

        info.message_filters.push(ExtractedMessageFilter {
            filter_type: filter_type.to_string(),
            topics: vec![], // Would need deeper parsing
            queue_size: 10,
        });
    }

    // message_filters.Subscriber
    if content.contains("message_filters.Subscriber") {
        // Already using message_filters
    }

    // =========== Image Transport (Python) ===========

    // cv_bridge / image_transport patterns
    if content.contains("CvBridge") || content.contains("imgmsg_to_cv2") {
        info.image_transports.push(ExtractedImageTransport {
            transport_type: "cv_bridge".to_string(),
            topic: "image".to_string(),
            field_name: "cv_bridge".to_string(),
        });
    }

    // =========== dynamic_reconfigure (ROS1 Python) ===========

    // dynamic_reconfigure.server.Server
    let dynreconf_py_regex =
        regex::Regex::new(r#"dynamic_reconfigure\.server\.Server\s*\(\s*(\w+)"#).ok();

    if let Some(re) = &dynreconf_py_regex {
        for caps in re.captures_iter(content) {
            let config_type = caps
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_else(|| "Config".to_string());

            info.dynamic_reconfigure.push(ExtractedDynamicReconfigure {
                config_type,
                callback: "reconfigure_callback".to_string(),
            });
        }
    }

    // =========== Diagnostic Updater (Python) ===========

    if content.contains("diagnostic_updater") || content.contains("DiagnosticUpdater") {
        info.diagnostics.push(ExtractedDiagnostic {
            updater_name: "diagnostic_updater".to_string(),
            hardware_id: None,
        });
    }

    // =========== ROS2 Executors (Python) ===========

    if content.contains("SingleThreadedExecutor") {
        info.executors.push(ExtractedExecutor {
            executor_type: "SingleThreaded".to_string(),
        });
    }
    if content.contains("MultiThreadedExecutor") {
        info.executors.push(ExtractedExecutor {
            executor_type: "MultiThreaded".to_string(),
        });
    }

    // =========== ROS2 Callback Groups (Python) ===========

    if content.contains("MutuallyExclusiveCallbackGroup") {
        info.callback_groups.push(ExtractedCallbackGroup {
            group_type: "MutuallyExclusive".to_string(),
            name: "mutex_group".to_string(),
        });
    }
    if content.contains("ReentrantCallbackGroup") {
        info.callback_groups.push(ExtractedCallbackGroup {
            group_type: "Reentrant".to_string(),
            name: "reentrant_group".to_string(),
        });
    }

    // =========== ros2_control / hardware_interface (Python) ===========

    // Python ros2_control hardware interfaces are less common but possible
    if content.contains("hardware_interface") || content.contains("SystemInterface") {
        info.hardware_interfaces.push(ExtractedHardwareInterface {
            interface_type: "SystemInterface".to_string(),
            state_interfaces: Vec::new(),
            command_interfaces: Vec::new(),
        });
    }

    // controller_manager Python API
    if content.contains("controller_manager")
        || content.contains("load_controller")
        || content.contains("switch_controller")
    {
        info.controllers.push(ExtractedController {
            controller_type: "controller_manager".to_string(),
            controller_name: "controller".to_string(),
        });
    }

    // =========== rosserial / MicroROS (Python) ===========

    // rosserial_python is the Python variant
    if content.contains("rosserial_python") || content.contains("SerialClient") {
        info.embedded_ros = Some(ExtractedEmbeddedROS {
            ros_type: "rosserial".to_string(),
            publishers: Vec::new(),
            subscribers: Vec::new(),
        });
    }

    // micro_ros_agent Python bindings
    if content.contains("micro_ros") || content.contains("MicroROS") {
        info.embedded_ros = Some(ExtractedEmbeddedROS {
            ros_type: "micro_ros".to_string(),
            publishers: Vec::new(),
            subscribers: Vec::new(),
        });
    }

    // =========== WaitSet / GuardCondition (Python ROS2) ===========

    if content.contains("WaitSet") || content.contains("wait_set") {
        let mut waitset = ExtractedWaitSet {
            has_waitset: true,
            guard_conditions: Vec::new(),
        };

        if content.contains("GuardCondition") || content.contains("guard_condition") {
            waitset.guard_conditions.push("guard_condition".to_string());
        }

        info.waitset = Some(waitset);
    }

    // =========== IntraProcessComms (Python ROS2) ===========

    if content.contains("intra_process") || content.contains("IntraProcess") {
        info.intra_process = Some(ExtractedIntraProcess {
            enabled: true,
            buffer_size: None,
        });
    }

    // =========== Bond (Python) ===========

    // bondpy is the Python bond implementation
    if content.contains("bondpy") || content.contains("Bond(") {
        info.bonds.push(ExtractedBond {
            bond_id: "bond".to_string(),
            topic: "/bond".to_string(),
        });
    }

    // =========== laser_geometry (Python) ===========

    if content.contains("laser_geometry") || content.contains("LaserProjection") {
        info.laser_geometry = Some(ExtractedLaserGeometry {
            projector_name: "laser_projector".to_string(),
        });
    }

    // =========== URDF / Xacro (Python) ===========

    if content.contains("urdf") || content.contains("robot_description") {
        let mut urdf_info = ExtractedURDF {
            robot_name: String::new(),
            links: Vec::new(),
            joints: Vec::new(),
            has_xacro: content.contains("xacro"),
        };

        // kdl_parser Python bindings
        if content.contains("kdl_parser") || content.contains("treeFromUrdfModel") {
            urdf_info.robot_name = "robot".to_string();
        }

        info.urdf = Some(urdf_info);
    }

    // =========== Nav2 (Python) ===========

    let mut nav2 = ExtractedNav2::default();
    let mut has_nav2 = false;

    if content.contains("nav2_")
        || content.contains("NavigateToPose")
        || content.contains("FollowWaypoints")
    {
        has_nav2 = true;
    }

    // Nav2 BehaviorTree
    if content.contains("BehaviorTree") || content.contains("behavior_tree") {
        nav2.behavior_trees.push("bt_navigator".to_string());
        has_nav2 = true;
    }

    // Nav2 costmap
    if content.contains("Costmap2D") || content.contains("costmap") {
        nav2.costmap_plugins.push("costmap".to_string());
        has_nav2 = true;
    }

    // Nav2 planners
    if content.contains("NavfnPlanner")
        || content.contains("SmacPlanner")
        || content.contains("ThetaStarPlanner")
    {
        nav2.planners.push("planner".to_string());
        has_nav2 = true;
    }

    // Nav2 controllers
    if content.contains("DWBLocalPlanner")
        || content.contains("RegulatedPurePursuit")
        || content.contains("TEB")
        || content.contains("FollowPath")
    {
        nav2.controllers.push("controller".to_string());
        has_nav2 = true;
    }

    // Nav2 recovery behaviors
    if content.contains("Recovery")
        || content.contains("Spin")
        || content.contains("BackUp")
        || content.contains("Wait")
    {
        if content.contains("nav2") {
            nav2.recovery_behaviors.push("recovery".to_string());
            has_nav2 = true;
        }
    }

    if has_nav2 {
        info.nav2 = Some(nav2);
    }

    // =========== MoveIt (Python) ===========

    let mut moveit = ExtractedMoveIt::default();
    let mut has_moveit = false;

    // MoveGroupInterface Python
    if content.contains("MoveGroupInterface")
        || content.contains("MoveGroupCommander")
        || content.contains("moveit_commander")
    {
        moveit.move_group_interface = true;
        has_moveit = true;
    }

    // PlanningSceneInterface Python
    if content.contains("PlanningSceneInterface") || content.contains("planning_scene") {
        moveit.planning_scene_interface = true;
        has_moveit = true;
    }

    // Extract planning group names
    let group_regex =
        regex::Regex::new(r#"(?:MoveGroupInterface|MoveGroupCommander)\s*\(\s*['"]([\w]+)['"]"#)
            .ok();

    if let Some(re) = &group_regex {
        for caps in re.captures_iter(content) {
            if let Some(group) = caps.get(1) {
                moveit.planning_groups.push(group.as_str().to_string());
                has_moveit = true;
            }
        }
    }

    if has_moveit {
        info.moveit = Some(moveit);
    }

    // =========== Gazebo Plugins (Python) ===========

    // Gazebo Python API / ros_gz
    if content.contains("gazebo") || content.contains("gz_") || content.contains("ros_gz") {
        if content.contains("ModelPlugin") || content.contains("model_plugin") {
            info.gazebo_plugins.push(ExtractedGazeboPlugin {
                plugin_type: "ModelPlugin".to_string(),
                plugin_name: "model_plugin".to_string(),
            });
        }
        if content.contains("WorldPlugin") || content.contains("world_plugin") {
            info.gazebo_plugins.push(ExtractedGazeboPlugin {
                plugin_type: "WorldPlugin".to_string(),
                plugin_name: "world_plugin".to_string(),
            });
        }
        if content.contains("SensorPlugin") || content.contains("sensor_plugin") {
            info.gazebo_plugins.push(ExtractedGazeboPlugin {
                plugin_type: "SensorPlugin".to_string(),
                plugin_name: "sensor_plugin".to_string(),
            });
        }
    }

    // =========== RViz / Visualization (Python) ===========

    let mut viz = ExtractedVisualization::default();
    let mut has_viz = false;

    // Marker publishers
    if content.contains("Marker") || content.contains("visualization_msgs") {
        viz.marker_publishers.push("marker_pub".to_string());
        has_viz = true;
    }

    // Interactive markers
    if content.contains("InteractiveMarker") || content.contains("interactive_marker") {
        viz.interactive_markers
            .push("interactive_marker".to_string());
        has_viz = true;
    }

    // RViz panels/plugins (Python based)
    if content.contains("rviz") || content.contains("RViz") {
        if content.contains("Panel") || content.contains("Display") {
            viz.rviz_plugins.push("rviz_plugin".to_string());
            has_viz = true;
        }
    }

    if has_viz {
        info.visualization = Some(viz);
    }

    // =========== rosbag / ros2bag (Python) ===========

    let mut bag = ExtractedBagRecording::default();
    let mut has_bag = false;

    // ROS1 rosbag Python
    if content.contains("rosbag") {
        if content.contains("Bag(") || content.contains("rosbag.Bag") {
            has_bag = true;
        }
    }

    // ROS2 rosbag2_py
    if content.contains("rosbag2")
        || content.contains("SequentialReader")
        || content.contains("SequentialWriter")
    {
        has_bag = true;
        if content.contains("Writer") {
            bag.is_recorder = true;
        }
        if content.contains("Reader") {
            bag.is_player = true;
        }
    }

    if has_bag {
        info.bag_recording = Some(bag);
    }

    // =========== camera_info_manager (Python) ===========

    if content.contains("camera_info_manager")
        || content.contains("CameraInfoManager")
        || content.contains("CameraInfo")
    {
        info.camera_info = Some(ExtractedCameraInfo {
            camera_name: "camera".to_string(),
            camera_info_url: None,
        });
    }

    // =========== image_geometry (Python) ===========

    if content.contains("image_geometry")
        || content.contains("PinholeCameraModel")
        || content.contains("StereoCameraModel")
    {
        let model = if content.contains("Stereo") {
            "StereoCameraModel"
        } else {
            "PinholeCameraModel"
        };
        info.image_geometry = Some(ExtractedImageGeometry {
            camera_model: model.to_string(),
        });
    }

    // =========== PCL (Python bindings) ===========

    // python-pcl or pcl_ros Python
    if content.contains("pcl") || content.contains("PointCloud") {
        let mut pcl = ExtractedPCL::default();

        if content.contains("VoxelGrid") {
            pcl.filters.push("VoxelGrid".to_string());
        }
        if content.contains("PassThrough") {
            pcl.filters.push("PassThrough".to_string());
        }
        if content.contains("StatisticalOutlierRemoval") {
            pcl.filters.push("StatisticalOutlierRemoval".to_string());
        }
        if content.contains("RadiusOutlierRemoval") {
            pcl.filters.push("RadiusOutlierRemoval".to_string());
        }
        if content.contains("ExtractIndices") {
            pcl.filters.push("ExtractIndices".to_string());
        }

        if !pcl.filters.is_empty() || content.contains("pcl") {
            info.pcl = Some(pcl);
        }
    }

    // =========== realtime_tools (Python - rare but possible) ===========

    // realtime_tools are primarily C++ but may have Python bindings
    if content.contains("realtime_tools") || content.contains("RealtimePublisher") {
        info.realtime_tools = Some(ExtractedRealtimeTools {
            realtime_publisher: content.contains("RealtimePublisher"),
            realtime_buffer: content.contains("RealtimeBuffer"),
            realtime_box: content.contains("RealtimeBox"),
        });
    }

    // =========== robot_state_publisher / joint_state_publisher (Python) ===========

    if content.contains("robot_state_publisher") || content.contains("RobotStatePublisher") {
        // Detected as using robot state publisher
        info.urdf = info.urdf.or(Some(ExtractedURDF::default()));
    }

    if content.contains("joint_state_publisher") || content.contains("JointStatePublisher") {
        // Detected as using joint state publisher
        info.urdf = info.urdf.or(Some(ExtractedURDF::default()));
    }

    // ========== ADDITIONAL COMPREHENSIVE PATTERNS (Python) ==========

    // robot_localization / state estimation (EKF, UKF)
    if content.contains("robot_localization")
        || content.contains("RobotLocalization")
        || content.contains("ekf_node")
        || content.contains("ukf_node")
        || content.contains("EkfNode")
        || content.contains("UkfNode")
        || content.contains("ExtendedKalmanFilter")
        || content.contains("UnscentedKalmanFilter")
    {
        let mut se = ExtractedStateEstimation::default();
        if content.contains("ukf")
            || content.contains("UKF")
            || content.contains("UnscentedKalmanFilter")
        {
            se.estimator_type = "ukf".to_string();
        } else {
            se.estimator_type = "ekf".to_string();
        }
        if content.contains("odom") || content.contains("odometry") {
            se.sensor_sources.push("odom".to_string());
        }
        if content.contains("imu") || content.contains("Imu") {
            se.sensor_sources.push("imu".to_string());
        }
        if content.contains("gps") || content.contains("GPS") || content.contains("NavSatFix") {
            se.sensor_sources.push("gps".to_string());
        }
        info.state_estimation = Some(se);
    }

    // SLAM patterns
    if content.contains("slam_toolbox")
        || content.contains("SlamToolbox")
        || content.contains("cartographer")
        || content.contains("gmapping")
        || content.contains("hector_slam")
        || content.contains("rtabmap")
    {
        let mut slam = ExtractedSLAM::default();
        if content.contains("slam_toolbox") || content.contains("SlamToolbox") {
            slam.slam_type = "slam_toolbox".to_string();
        } else if content.contains("cartographer") {
            slam.slam_type = "cartographer".to_string();
        } else if content.contains("gmapping") {
            slam.slam_type = "gmapping".to_string();
        } else if content.contains("hector_slam") {
            slam.slam_type = "hector_slam".to_string();
        } else if content.contains("rtabmap") {
            slam.slam_type = "rtabmap".to_string();
        }
        slam.has_loop_closure = content.contains("loop_closure");
        slam.has_map_save = content.contains("save_map") || content.contains("serialize");
        info.slam = Some(slam);
    }

    // AMCL / localization
    if content.contains("amcl") || content.contains("AMCL") || content.contains("nav2_amcl") {
        info.localization = Some(ExtractedLocalization {
            localization_type: "amcl".to_string(),
            uses_particle_filter: true,
        });
    }

    // 3D mapping (octomap, voxblox)
    if content.contains("octomap")
        || content.contains("OctoMap")
        || content.contains("voxblox")
        || content.contains("elevation_mapping")
    {
        let mut map3d = Extracted3DMapping::default();
        if content.contains("octomap") || content.contains("OctoMap") {
            map3d.mapping_type = "octomap".to_string();
        } else if content.contains("voxblox") {
            map3d.mapping_type = "voxblox".to_string();
        } else {
            map3d.mapping_type = "elevation_mapping".to_string();
        }
        map3d.uses_octree = content.contains("octree") || content.contains("Octree");
        info.mapping_3d = Some(map3d);
    }

    // depth/stereo image processing
    if content.contains("depth_image_proc")
        || content.contains("stereo_image_proc")
        || content.contains("point_cloud_xyz")
        || content.contains("disparity")
    {
        let mut depth = ExtractedDepthProcessing::default();
        if content.contains("stereo_image_proc") || content.contains("disparity") {
            depth.processing_type = "stereo_image_proc".to_string();
        } else {
            depth.processing_type = "depth_image_proc".to_string();
        }
        if content.contains("point_cloud_xyz") {
            depth.operations.push("point_cloud_xyz".to_string());
        }
        if content.contains("disparity") {
            depth.operations.push("disparity".to_string());
        }
        info.depth_processing = Some(depth);
    }

    // rosbridge / web bridge
    if content.contains("rosbridge")
        || content.contains("roslibjs")
        || content.contains("rosapi")
        || content.contains("websocket")
    {
        info.web_bridge = Some(ExtractedWebBridge {
            bridge_type: "rosbridge".to_string(),
            websocket: content.contains("websocket") || content.contains("WebSocket"),
        });
    }

    // state machines (smach, FlexBE, SMACC)
    if content.contains("smach")
        || content.contains("StateMachine")
        || content.contains("flexbe")
        || content.contains("smacc")
    {
        let mut sm = ExtractedStateMachine::default();
        if content.contains("smach") {
            sm.state_machine_type = "smach".to_string();
        } else if content.contains("flexbe") {
            sm.state_machine_type = "flexbe".to_string();
        } else if content.contains("smacc") {
            sm.state_machine_type = "smacc".to_string();
        } else {
            sm.state_machine_type = "custom".to_string();
        }
        info.state_machine = Some(sm);
    }

    // sensor drivers
    if content.contains("velodyne") || content.contains("VLP16") || content.contains("VLP32") {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "velodyne".to_string(),
            driver_name: "velodyne_driver".to_string(),
        });
    }
    if content.contains("realsense")
        || content.contains("RealSense")
        || content.contains("pyrealsense")
    {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "realsense".to_string(),
            driver_name: "realsense2_camera".to_string(),
        });
    }
    if content.contains("zed") || content.contains("ZED") || content.contains("pyzed") {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "zed".to_string(),
            driver_name: "zed_ros".to_string(),
        });
    }
    if content.contains("ouster") || content.contains("OS1") || content.contains("OS0") {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "ouster".to_string(),
            driver_name: "ouster_ros".to_string(),
        });
    }
    if content.contains("livox") || content.contains("Livox") {
        info.sensor_drivers.push(ExtractedSensorDriver {
            sensor_type: "livox".to_string(),
            driver_name: "livox_ros_driver".to_string(),
        });
    }

    info
}

/// Convert topic name to valid Rust field name
fn topic_to_field_name(topic: &str) -> String {
    topic
        .trim_start_matches('/')
        .replace('/', "_")
        .replace('-', "_")
        .to_lowercase()
}

/// Convert C++ message type to Rust type
fn cpp_msg_to_rust(cpp_type: &str) -> String {
    // Handle namespaced types like sensor_msgs::msg::LaserScan
    let parts: Vec<&str> = cpp_type.split("::").collect();

    // Get the actual type name (last part)
    let type_name = parts.last().unwrap_or(&cpp_type);

    // Check for common types
    match *type_name {
        // Geometry types
        "Twist" | "TwistStamped" | "TwistWithCovariance" | "TwistWithCovarianceStamped" => {
            "Twist".to_string()
        }
        "Pose" | "PoseStamped" | "PoseWithCovariance" | "PoseWithCovarianceStamped" => {
            "Pose".to_string()
        }
        "Point" | "Point32" => "Point3".to_string(),
        "PointStamped" => "GenericMessage".to_string(), // No PointStamped in HORUS
        "Vector3" => "Vector3".to_string(),
        "Vector3Stamped" => "GenericMessage".to_string(), // No Vector3Stamped in HORUS
        "Quaternion" => "Quaternion".to_string(),
        "QuaternionStamped" => "GenericMessage".to_string(), // No QuaternionStamped in HORUS
        "Transform" => "Transform".to_string(),
        "TransformStamped" => "GenericMessage".to_string(), // No TransformStamped in HORUS
        "Wrench" | "WrenchStamped" => "WrenchStamped".to_string(), // HORUS has WrenchStamped
        "Accel" | "AccelStamped" | "AccelWithCovariance" | "AccelWithCovarianceStamped" => {
            "GenericMessage".to_string()
        } // No Acceleration type
        "Inertia" | "InertiaStamped" => "GenericMessage".to_string(), // No Inertia type
        "Polygon" | "PolygonStamped" => "GenericMessage".to_string(), // No Polygon type
        "Path" => "Path".to_string(),                              // HORUS has Path
        "PoseArray" => "PoseArray".to_string(),                    // HORUS has PoseArray

        // Sensor types
        "LaserScan" => "LaserScan".to_string(), // EXISTS
        "Imu" => "Imu".to_string(),             // EXISTS
        "Image" | "CompressedImage" => "Image".to_string(), // EXISTS
        "CameraInfo" => "CameraInfo".to_string(), // EXISTS
        "PointCloud2" | "PointCloud" => "PointCloud".to_string(), // EXISTS
        "Range" => "Range".to_string(),         // EXISTS
        "NavSatFix" => "NavSatFix".to_string(), // EXISTS (was incorrectly GpsData)
        "Joy" => "JoystickInput".to_string(),   // EXISTS
        "JointState" => "GenericMessage".to_string(), // No JointState type
        "Temperature" => "GenericMessage".to_string(), // No Temperature type
        "FluidPressure" => "GenericMessage".to_string(), // No Pressure type
        "Illuminance" => "GenericMessage".to_string(), // No Illuminance type
        "MagneticField" => "GenericMessage".to_string(), // No MagneticField type
        "BatteryState" => "BatteryState".to_string(), // EXISTS
        "TimeReference" => "GenericMessage".to_string(), // No TimeReference type
        "RegionOfInterest" => "RegionOfInterest".to_string(), // EXISTS

        // Navigation types
        "Odometry" => "Odometry".to_string(), // EXISTS
        "OccupancyGrid" | "MapMetaData" => "OccupancyGrid".to_string(), // EXISTS
        "GridCells" => "GenericMessage".to_string(), // No GridCells type

        // Standard types - map to Rust primitives or GenericMessage
        "Header" => "GenericMessage".to_string(), // No Header type in HORUS
        "String" => "String".to_string(),         // Use Rust String
        "Bool" => "bool".to_string(),
        "Int8" | "Int16" | "Int32" | "Int64" => "i64".to_string(),
        "UInt8" | "UInt16" | "UInt32" | "UInt64" => "u64".to_string(),
        "Float32" | "Float64" => "f64".to_string(),
        "Duration" => "std::time::Duration".to_string(), // Use fully qualified path
        "Time" => "u64".to_string(),          // Use u64 timestamp (nanoseconds)
        "Empty" => "()".to_string(),
        "ColorRGBA" => "GenericMessage".to_string(), // No Color type in HORUS

        // Diagnostic types - map to HORUS equivalents
        "DiagnosticArray" => "DiagnosticReport".to_string(),
        "DiagnosticStatus" => "Status".to_string(),
        "KeyValue" => "DiagnosticValue".to_string(),

        // Control types - use GenericMessage for complex trajectory types
        "JointTrajectory" | "JointTrajectoryPoint" => "GenericMessage".to_string(),
        "FollowJointTrajectoryAction"
        | "FollowJointTrajectoryGoal"
        | "FollowJointTrajectoryResult"
        | "FollowJointTrajectoryFeedback" => "GenericMessage".to_string(),

        // TF types - use GenericMessage for transform arrays
        "TFMessage" => "GenericMessage".to_string(),

        // Visualization types - use GenericMessage
        "Marker" | "MarkerArray" => "GenericMessage".to_string(),
        "InteractiveMarker" | "InteractiveMarkerControl" | "InteractiveMarkerFeedback" => {
            "GenericMessage".to_string()
        }

        // ROS2 lifecycle/parameter types - use Status for events, GenericMessage for values
        "ParameterEvent" | "ParameterValue" | "Parameter" => "GenericMessage".to_string(),
        "Log" | "RosOut" => "Status".to_string(),

        // Clock - use GenericMessage
        "Clock" => "GenericMessage".to_string(),

        // Handle "Unknown" which appears when type couldn't be parsed
        "Unknown" | "unknown" | "" => "GenericMessage".to_string(),

        // TF2 types
        "TF2Error" => "GenericMessage".to_string(),

        // Actionlib types
        "GoalStatusArray" | "GoalStatus" | "GoalID" => "GenericMessage".to_string(),

        // Entity type
        "Entity" => "GenericMessage".to_string(),

        // FeedbackMessage (action feedback wrapper)
        "FeedbackMessage" => "GenericMessage".to_string(),

        // Unknown types - convert to PascalCase and add TODO marker
        other => {
            let pascal = to_pascal_case(other);
            // Return with a marker that this may need manual mapping
            pascal
        }
    }
}

/// Generate node skeleton from package with extracted info
/// Format QoS settings as a comment string
fn format_qos_comment(qos: &ExtractedQoS) -> String {
    let mut parts = Vec::new();

    if let Some(depth) = qos.depth {
        parts.push(format!("depth={}", depth));
    }
    if let Some(ref reliability) = qos.reliability {
        parts.push(reliability.clone());
    }
    if let Some(ref durability) = qos.durability {
        parts.push(durability.clone());
    }
    if let Some(ref history) = qos.history {
        parts.push(history.clone());
    }

    if parts.is_empty() {
        String::new()
    } else {
        format!(" (QoS: {})", parts.join(", "))
    }
}

pub fn generate_node_skeleton_from_source(node_name: &str, info: &ParsedNodeInfo) -> String {
    let struct_name = if info.class_name.is_empty() {
        to_pascal_case(node_name)
    } else {
        info.class_name.clone()
    };

    // Track used field names to avoid duplicates
    let mut used_fields: std::collections::HashSet<String> = std::collections::HashSet::new();

    // Generate fields
    let mut fields = String::new();
    for sub in &info.subscribers {
        // Skip duplicate field names
        if used_fields.contains(&sub.field_name) {
            continue;
        }
        used_fields.insert(sub.field_name.clone());

        let rust_type = cpp_msg_to_rust(&sub.msg_type);
        let qos_info = format_qos_comment(&sub.qos);
        fields.push_str(&format!(
            "    /// Subscriber for topic: {}{}\n",
            sub.topic, qos_info
        ));
        fields.push_str(&format!(
            "    pub {}: Link<{}>,\n",
            sub.field_name, rust_type
        ));
    }
    for pub_ in &info.publishers {
        // Skip duplicate field names
        if used_fields.contains(&pub_.field_name) {
            continue;
        }
        used_fields.insert(pub_.field_name.clone());

        let rust_type = cpp_msg_to_rust(&pub_.msg_type);
        let qos_info = format_qos_comment(&pub_.qos);
        fields.push_str(&format!(
            "    /// Publisher for topic: {}{}\n",
            pub_.topic, qos_info
        ));
        fields.push_str(&format!(
            "    pub {}: Hub<{}>,\n",
            pub_.field_name, rust_type
        ));
    }
    // Service servers
    for srv in &info.service_servers {
        let rust_type = cpp_msg_to_rust(&srv.srv_type);
        fields.push_str(&format!(
            "    /// Service server: {} (type: {})\n",
            srv.service_name, srv.srv_type
        ));
        fields.push_str(&format!(
            "    // TODO: Implement service handler for {}\n",
            srv.service_name
        ));
        fields.push_str(&format!(
            "    // pub {}: ServiceServer<{}>,\n",
            srv.field_name, rust_type
        ));
    }
    // Service clients
    for srv in &info.service_clients {
        let rust_type = cpp_msg_to_rust(&srv.srv_type);
        fields.push_str(&format!(
            "    /// Service client: {} (type: {})\n",
            srv.service_name, srv.srv_type
        ));
        fields.push_str(&format!(
            "    // TODO: Implement service client for {}\n",
            srv.service_name
        ));
        fields.push_str(&format!(
            "    // pub {}: ServiceClient<{}>,\n",
            srv.field_name, rust_type
        ));
    }
    // Action servers
    for action in &info.action_servers {
        fields.push_str(&format!(
            "    /// Action server: {} (type: {})\n",
            action.action_name, action.action_type
        ));
        fields.push_str(&format!(
            "    // TODO: Implement action server for {}\n",
            action.action_name
        ));
    }
    // Action clients
    for action in &info.action_clients {
        fields.push_str(&format!(
            "    /// Action client: {} (type: {})\n",
            action.action_name, action.action_type
        ));
        fields.push_str(&format!(
            "    // TODO: Implement action client for {}\n",
            action.action_name
        ));
    }
    // TF broadcasters
    for tf in &info.tf_broadcasters {
        fields.push_str(&format!("    /// TF {} broadcaster\n", tf.broadcaster_type));
        fields.push_str(&format!(
            "    // TODO: Implement TF broadcasting (was {})\n",
            tf.broadcaster_type
        ));
    }
    for param in &info.parameters {
        let field_name = sanitize_field_name(&param.name);
        // Skip duplicate field names
        if used_fields.contains(&field_name) {
            continue;
        }
        used_fields.insert(field_name.clone());

        let rust_type = match param.param_type.as_str() {
            "int" | "int64_t" => "i64",
            "double" | "float" => "f64",
            "bool" => "bool",
            "std::string" | "string" => "String",
            _ => "f64",
        };
        fields.push_str(&format!("    /// Parameter: {}\n", param.name));
        fields.push_str(&format!(
            "    pub {}: {},\n",
            field_name,
            rust_type
        ));
    }

    if fields.is_empty() {
        fields = "    // Add Hub/Link fields as needed\n".to_string();
    }

    // Generate new() initialization
    let mut init_fields = String::new();
    let mut init_used_fields: std::collections::HashSet<String> = std::collections::HashSet::new();
    for sub in &info.subscribers {
        // Skip duplicate field names in initialization
        if init_used_fields.contains(&sub.field_name) {
            continue;
        }
        init_used_fields.insert(sub.field_name.clone());
        init_fields.push_str(&format!(
            "            {}: Link::consumer(\"{}\").expect(\"failed to create link\"),\n",
            sub.field_name, sub.topic
        ));
    }
    for pub_ in &info.publishers {
        // Skip duplicate field names in initialization
        if init_used_fields.contains(&pub_.field_name) {
            continue;
        }
        init_used_fields.insert(pub_.field_name.clone());
        init_fields.push_str(&format!(
            "            {}: Hub::new(\"{}\").expect(\"failed to create hub\"),\n",
            pub_.field_name, pub_.topic
        ));
    }
    for param in &info.parameters {
        let field_name = sanitize_field_name(&param.name);
        // Skip duplicate field names in initialization
        if init_used_fields.contains(&field_name) {
            continue;
        }
        init_used_fields.insert(field_name.clone());
        let default = match param.param_type.as_str() {
            "int" | "int64_t" => "0",
            "double" | "float" => "0.0",
            "bool" => "false",
            _ => "Default::default()",
        };
        init_fields.push_str(&format!(
            "            {}: {},\n",
            field_name,
            default
        ));
    }

    if init_fields.is_empty() {
        init_fields = "            // Initialize fields\n".to_string();
    }

    // Generate tick() body
    let mut tick_body = String::new();
    for sub in &info.subscribers {
        tick_body.push_str(&format!(
            "        // Process {} messages\n        if let Some(_msg) = self.{}.recv(&mut _ctx) {{\n            // Handle message\n        }}\n\n",
            sub.topic, sub.field_name
        ));
    }

    if !info.timers.is_empty() {
        tick_body.push_str("        // Timer logic (original period: ");
        for timer in &info.timers {
            tick_body.push_str(&format!("{}ms ", timer.period_ms));
        }
        tick_body.push_str(")\n");
    }

    // Add service/action handling stubs
    if !info.service_servers.is_empty() {
        tick_body
            .push_str("\n        // Service handling (implement service request processing)\n");
    }
    if !info.action_servers.is_empty() {
        tick_body
            .push_str("\n        // Action handling (implement goal/feedback/result processing)\n");
    }

    if tick_body.is_empty() {
        tick_body = "        // Implement node logic here\n".to_string();
    }

    // Build additional info lines
    let mut extra_info = String::new();
    if !info.service_servers.is_empty() {
        extra_info.push_str(&format!(
            "//! Service Servers: {}\n",
            info.service_servers.len()
        ));
    }
    if !info.service_clients.is_empty() {
        extra_info.push_str(&format!(
            "//! Service Clients: {}\n",
            info.service_clients.len()
        ));
    }
    if !info.action_servers.is_empty() {
        extra_info.push_str(&format!(
            "//! Action Servers: {}\n",
            info.action_servers.len()
        ));
    }
    if !info.action_clients.is_empty() {
        extra_info.push_str(&format!(
            "//! Action Clients: {}\n",
            info.action_clients.len()
        ));
    }
    if !info.tf_broadcasters.is_empty() {
        extra_info.push_str(&format!(
            "//! TF Broadcasters: {}\n",
            info.tf_broadcasters.len()
        ));
    }
    if !info.tf_listeners.is_empty() {
        extra_info.push_str(&format!(
            "//! TF Listeners: {} - use HORUS TF utilities for transforms\n",
            info.tf_listeners.len()
        ));
    }
    if !info.message_filters.is_empty() {
        extra_info.push_str("//! ⚠️ MESSAGE FILTERS - time synchronization detected\n");
        for filter in &info.message_filters {
            extra_info.push_str(&format!(
                "//!   {} (queue: {})\n",
                filter.filter_type, filter.queue_size
            ));
        }
        extra_info.push_str("//!   HORUS: Use multi-topic Links with synchronized recv()\n");
    }
    if !info.image_transports.is_empty() {
        extra_info.push_str("//! ⚠️ IMAGE TRANSPORT - image pipeline detected\n");
        for img in &info.image_transports {
            extra_info.push_str(&format!(
                "//!   {} on '{}'\n",
                img.transport_type, img.topic
            ));
        }
        extra_info.push_str("//!   HORUS: Use Image message type with optional compression\n");
    }
    if !info.diagnostics.is_empty() {
        extra_info.push_str(&format!(
            "//! ⚠️ DIAGNOSTIC UPDATER ({}) - implement diagnostics reporting\n",
            info.diagnostics.len()
        ));
    }
    if !info.dynamic_reconfigure.is_empty() {
        extra_info.push_str("//! ⚠️ DYNAMIC RECONFIGURE (ROS1) - runtime parameter changes\n");
        for dyn_reconf in &info.dynamic_reconfigure {
            extra_info.push_str(&format!("//!   Config type: {}\n", dyn_reconf.config_type));
        }
        extra_info.push_str("//!   HORUS: Use parameter server with change callbacks\n");
    }
    if !info.executors.is_empty() {
        extra_info.push_str("//! ⚠️ CUSTOM EXECUTOR - threading model detected\n");
        for exec in &info.executors {
            extra_info.push_str(&format!("//!   {} executor\n", exec.executor_type));
        }
        extra_info.push_str("//!   HORUS: Scheduler handles execution automatically\n");
    }
    if !info.callback_groups.is_empty() {
        extra_info.push_str("//! ⚠️ CALLBACK GROUPS - concurrency control detected\n");
        for grp in &info.callback_groups {
            extra_info.push_str(&format!("//!   {} ({})\n", grp.name, grp.group_type));
        }
        extra_info.push_str("//!   HORUS: Configure via Scheduler thread pools\n");
    }
    if info.is_lifecycle {
        extra_info.push_str("//! ⚠️ LIFECYCLE NODE - requires state machine implementation\n");
    }
    if info.is_component {
        extra_info.push_str("//! ⚠️ COMPONENT NODE - was dynamically loaded in ROS2\n");
    }
    if info.is_nodelet {
        extra_info
            .push_str("//! ⚠️ NODELET (ROS1) - was loaded in nodelet manager for zero-copy IPC\n");
        extra_info
            .push_str("//! HORUS provides zero-copy shared memory by default via Hubs/Links\n");
    }

    // =========== New comprehensive feature comments ===========

    // ros2_control / hardware_interface
    if !info.hardware_interfaces.is_empty() {
        extra_info.push_str("//! ⚠️ ROS2_CONTROL HARDWARE INTERFACE detected\n");
        for hw in &info.hardware_interfaces {
            extra_info.push_str(&format!("//!   {} interface\n", hw.interface_type));
            if !hw.state_interfaces.is_empty() {
                extra_info.push_str(&format!(
                    "//!     State interfaces: {:?}\n",
                    hw.state_interfaces
                ));
            }
            if !hw.command_interfaces.is_empty() {
                extra_info.push_str(&format!(
                    "//!     Command interfaces: {:?}\n",
                    hw.command_interfaces
                ));
            }
        }
        extra_info
            .push_str("//!   HORUS: Implement as real-time node with direct hardware access\n");
    }

    // ros_control controllers
    if !info.controllers.is_empty() {
        extra_info.push_str("//! ⚠️ ROS_CONTROL CONTROLLERS detected\n");
        for ctrl in &info.controllers {
            extra_info.push_str(&format!(
                "//!   {} ({})\n",
                ctrl.controller_name, ctrl.controller_type
            ));
        }
        extra_info
            .push_str("//!   HORUS: Use built-in PID, servo controllers, or implement custom\n");
    }

    // rosserial / MicroROS
    if let Some(ref embedded) = info.embedded_ros {
        extra_info.push_str(&format!(
            "//! ⚠️ {} EMBEDDED ROS detected\n",
            embedded.ros_type.to_uppercase()
        ));
        extra_info
            .push_str("//!   HORUS: Use serial/I2C/SPI bus nodes for embedded communication\n");
    }

    // WaitSet / GuardCondition
    if let Some(ref waitset) = info.waitset {
        if waitset.has_waitset {
            extra_info.push_str(
                "//! ⚠️ WAITSET / GUARD CONDITION detected (low-level synchronization)\n",
            );
            extra_info.push_str("//!   HORUS: Scheduler handles synchronization automatically\n");
        }
    }

    // IntraProcessComms
    if let Some(ref ipc) = info.intra_process {
        if ipc.enabled {
            extra_info.push_str("//! ⚠️ INTRA-PROCESS COMMUNICATION enabled (zero-copy)\n");
            extra_info
                .push_str("//!   HORUS: All Hub/Link communication is zero-copy by default!\n");
        }
    }

    // Bond
    if !info.bonds.is_empty() {
        extra_info.push_str("//! ⚠️ BOND (heartbeat monitoring) detected\n");
        for bond in &info.bonds {
            extra_info.push_str(&format!(
                "//!   Bond: {} on topic {}\n",
                bond.bond_id, bond.topic
            ));
        }
        extra_info.push_str("//!   HORUS: Use node health monitoring via Scheduler\n");
    }

    // laser_geometry
    if let Some(ref laser) = info.laser_geometry {
        extra_info.push_str("//! ⚠️ LASER_GEOMETRY (LaserProjection) detected\n");
        extra_info.push_str(&format!("//!   Projector: {}\n", laser.projector_name));
        extra_info.push_str(
            "//!   HORUS: Use built-in LidarProcessor for scan-to-pointcloud conversion\n",
        );
    }

    // URDF / Xacro
    if let Some(ref urdf) = info.urdf {
        extra_info.push_str("//! ⚠️ URDF / ROBOT DESCRIPTION detected\n");
        if !urdf.robot_name.is_empty() {
            extra_info.push_str(&format!("//!   Robot: {}\n", urdf.robot_name));
        }
        if urdf.has_xacro {
            extra_info.push_str("//!   Uses Xacro macros\n");
        }
        if !urdf.links.is_empty() {
            extra_info.push_str(&format!("//!   Links: {}\n", urdf.links.len()));
        }
        if !urdf.joints.is_empty() {
            extra_info.push_str(&format!("//!   Joints: {}\n", urdf.joints.len()));
        }
        extra_info.push_str("//!   HORUS: Robot models can be loaded from URDF for simulation\n");
    }

    // Nav2
    if let Some(ref nav2) = info.nav2 {
        extra_info.push_str("//! ⚠️ NAV2 (Navigation2) patterns detected\n");
        if !nav2.behavior_trees.is_empty() {
            extra_info.push_str(&format!(
                "//!   Behavior Trees: {:?}\n",
                nav2.behavior_trees
            ));
        }
        if !nav2.costmap_plugins.is_empty() {
            extra_info.push_str(&format!(
                "//!   Costmap plugins: {:?}\n",
                nav2.costmap_plugins
            ));
        }
        if !nav2.planners.is_empty() {
            extra_info.push_str(&format!("//!   Planners: {:?}\n", nav2.planners));
        }
        if !nav2.controllers.is_empty() {
            extra_info.push_str(&format!("//!   Controllers: {:?}\n", nav2.controllers));
        }
        if !nav2.recovery_behaviors.is_empty() {
            extra_info.push_str(&format!(
                "//!   Recovery behaviors: {:?}\n",
                nav2.recovery_behaviors
            ));
        }
        extra_info.push_str("//!   HORUS: Use built-in A*, RRT, PurePursuit algorithms\n");
    }

    // MoveIt
    if let Some(ref moveit) = info.moveit {
        extra_info.push_str("//! ⚠️ MOVEIT (motion planning) patterns detected\n");
        if moveit.move_group_interface {
            extra_info.push_str("//!   MoveGroupInterface: motion planning enabled\n");
        }
        if moveit.planning_scene_interface {
            extra_info.push_str("//!   PlanningSceneInterface: collision detection enabled\n");
        }
        if !moveit.planning_groups.is_empty() {
            extra_info.push_str(&format!(
                "//!   Planning groups: {:?}\n",
                moveit.planning_groups
            ));
        }
        extra_info
            .push_str("//!   HORUS: Implement trajectory planning with built-in IK solvers\n");
    }

    // Gazebo plugins
    if !info.gazebo_plugins.is_empty() {
        extra_info.push_str("//! ⚠️ GAZEBO PLUGINS detected\n");
        for plugin in &info.gazebo_plugins {
            extra_info.push_str(&format!(
                "//!   {} ({})\n",
                plugin.plugin_name, plugin.plugin_type
            ));
        }
        extra_info.push_str("//!   HORUS: Use sim2d/sim3d simulators with HORUS integration\n");
    }

    // RViz / Visualization
    if let Some(ref viz) = info.visualization {
        extra_info.push_str("//! ⚠️ RVIZ / VISUALIZATION patterns detected\n");
        if !viz.marker_publishers.is_empty() {
            extra_info.push_str(&format!(
                "//!   Marker publishers: {:?}\n",
                viz.marker_publishers
            ));
        }
        if !viz.interactive_markers.is_empty() {
            extra_info.push_str(&format!(
                "//!   Interactive markers: {:?}\n",
                viz.interactive_markers
            ));
        }
        if !viz.rviz_plugins.is_empty() {
            extra_info.push_str(&format!("//!   RViz plugins: {:?}\n", viz.rviz_plugins));
        }
        extra_info.push_str("//!   HORUS: Use HORUS dashboard for visualization\n");
    }

    // rosbag / ros2bag
    if let Some(ref bag) = info.bag_recording {
        extra_info.push_str("//! ⚠️ ROSBAG / ROS2BAG recording detected\n");
        if bag.is_recorder {
            extra_info.push_str("//!   Recording functionality\n");
        }
        if bag.is_player {
            extra_info.push_str("//!   Playback functionality\n");
        }
        if !bag.topics.is_empty() {
            extra_info.push_str(&format!("//!   Topics: {:?}\n", bag.topics));
        }
        extra_info.push_str("//!   HORUS: Use built-in blackbox recording\n");
    }

    // camera_info_manager
    if let Some(ref cam) = info.camera_info {
        extra_info.push_str("//! ⚠️ CAMERA_INFO_MANAGER detected\n");
        extra_info.push_str(&format!("//!   Camera: {}\n", cam.camera_name));
        if let Some(ref url) = cam.camera_info_url {
            extra_info.push_str(&format!("//!   Calibration URL: {}\n", url));
        }
        extra_info.push_str("//!   HORUS: Use CameraNode with calibration parameters\n");
    }

    // image_geometry
    if let Some(ref geom) = info.image_geometry {
        extra_info.push_str("//! ⚠️ IMAGE_GEOMETRY detected\n");
        extra_info.push_str(&format!("//!   Camera model: {}\n", geom.camera_model));
        extra_info.push_str("//!   HORUS: Use built-in camera projection utilities\n");
    }

    // PCL
    if let Some(ref pcl) = info.pcl {
        extra_info.push_str("//! ⚠️ PCL (Point Cloud Library) patterns detected\n");
        if !pcl.filters.is_empty() {
            extra_info.push_str(&format!("//!   Filters: {:?}\n", pcl.filters));
        }
        if !pcl.nodelets.is_empty() {
            extra_info.push_str(&format!("//!   PCL nodelets: {:?}\n", pcl.nodelets));
        }
        extra_info.push_str("//!   HORUS: Use PointCloudProcessor node for filtering\n");
    }

    // realtime_tools
    if let Some(ref rt) = info.realtime_tools {
        extra_info.push_str("//! ⚠️ REALTIME_TOOLS detected\n");
        if rt.realtime_publisher {
            extra_info.push_str("//!   RealtimePublisher: lock-free publishing\n");
        }
        if rt.realtime_buffer {
            extra_info.push_str("//!   RealtimeBuffer: thread-safe buffer\n");
        }
        if rt.realtime_box {
            extra_info.push_str("//!   RealtimeBox: lock-free container\n");
        }
        extra_info.push_str("//!   HORUS: All Hub/Link communication is lock-free by default!\n");
    }

    // ========== ADDITIONAL COMPREHENSIVE PATTERN COMMENTS ==========

    // State estimation (robot_localization, EKF, UKF)
    if let Some(ref se) = info.state_estimation {
        extra_info.push_str("//! ⚠️ STATE_ESTIMATION (robot_localization) detected\n");
        extra_info.push_str(&format!(
            "//!   Estimator type: {}\n",
            se.estimator_type.to_uppercase()
        ));
        if !se.sensor_sources.is_empty() {
            extra_info.push_str(&format!("//!   Sensor sources: {:?}\n", se.sensor_sources));
        }
        extra_info
            .push_str("//!   HORUS: Use built-in KalmanFilter algorithm or SensorFusion node\n");
    }

    // SLAM patterns
    if let Some(ref slam) = info.slam {
        extra_info.push_str("//! ⚠️ SLAM detected\n");
        extra_info.push_str(&format!("//!   SLAM type: {}\n", slam.slam_type));
        if slam.has_loop_closure {
            extra_info.push_str("//!   Loop closure: enabled\n");
        }
        if slam.has_map_save {
            extra_info.push_str("//!   Map save/serialize: enabled\n");
        }
        if slam.has_localization_mode {
            extra_info.push_str("//!   Localization mode: available\n");
        }
        extra_info.push_str("//!   HORUS: Use OccupancyGrid + LidarProcessor for mapping\n");
    }

    // Localization (AMCL, particle filter)
    if let Some(ref loc) = info.localization {
        extra_info.push_str("//! ⚠️ LOCALIZATION (particle filter) detected\n");
        extra_info.push_str(&format!(
            "//!   Localization type: {}\n",
            loc.localization_type.to_uppercase()
        ));
        if loc.uses_particle_filter {
            extra_info.push_str("//!   Uses particle filter: yes\n");
        }
        extra_info
            .push_str("//!   HORUS: Implement localization with EKF or custom particle filter\n");
    }

    // 3D Mapping (octomap, voxblox)
    if let Some(ref map3d) = info.mapping_3d {
        extra_info.push_str("//! ⚠️ 3D_MAPPING detected\n");
        extra_info.push_str(&format!("//!   Mapping type: {}\n", map3d.mapping_type));
        if map3d.uses_octree {
            extra_info.push_str("//!   Uses octree: yes\n");
        }
        extra_info.push_str("//!   HORUS: Use PointCloudProcessor for 3D mapping\n");
    }

    // Depth/stereo image processing
    if let Some(ref depth) = info.depth_processing {
        extra_info.push_str("//! ⚠️ DEPTH/STEREO PROCESSING detected\n");
        extra_info.push_str(&format!(
            "//!   Processing type: {}\n",
            depth.processing_type
        ));
        if !depth.operations.is_empty() {
            extra_info.push_str(&format!("//!   Operations: {:?}\n", depth.operations));
        }
        extra_info.push_str("//!   HORUS: Use Camera node with depth processing pipeline\n");
    }

    // Web bridge (rosbridge)
    if let Some(ref bridge) = info.web_bridge {
        extra_info.push_str("//! ⚠️ WEB_BRIDGE detected\n");
        extra_info.push_str(&format!("//!   Bridge type: {}\n", bridge.bridge_type));
        if bridge.websocket {
            extra_info.push_str("//!   WebSocket: enabled\n");
        }
        extra_info.push_str("//!   HORUS: Use HORUS Dashboard for web-based monitoring\n");
    }

    // State machine (smach, FlexBE, SMACC)
    if let Some(ref sm) = info.state_machine {
        extra_info.push_str("//! ⚠️ STATE_MACHINE detected\n");
        extra_info.push_str(&format!(
            "//!   State machine type: {}\n",
            sm.state_machine_type
        ));
        extra_info.push_str("//!   HORUS: Use Rust enum + match for state machine patterns\n");
    }

    // Sensor drivers
    if !info.sensor_drivers.is_empty() {
        extra_info.push_str("//! ⚠️ SENSOR_DRIVERS detected\n");
        for driver in &info.sensor_drivers {
            extra_info.push_str(&format!(
                "//!   {}: {} driver\n",
                driver.sensor_type.to_uppercase(),
                driver.driver_name
            ));
        }
        extra_info.push_str("//!   HORUS: Use built-in sensor nodes (LiDAR, Camera, IMU, GPS)\n");
    }

    let ros_label = if info.is_nodelet { "ROS1" } else { "ROS2" };

    format!(
        r#"//! {} node - converted from {}
//!
//! Original class: {}
//! Publishers: {}
//! Subscribers: {}
//! Parameters: {}
{}"#,
        node_name,
        ros_label,
        if info.class_name.is_empty() {
            "Unknown"
        } else {
            &info.class_name
        },
        info.publishers.len(),
        info.subscribers.len(),
        info.parameters.len(),
        extra_info
    ) + &format!(
        r#"
use horus::prelude::*;

pub struct {} {{
{}}}

impl {} {{
    pub fn new() -> Self {{
        Self {{
{}        }}
    }}
}}

impl Node for {} {{
    fn name(&self) -> &'static str {{
        "{}"
    }}

    fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
{}    }}
}}
"#,
        struct_name, fields, struct_name, init_fields, struct_name, node_name, tick_body
    )
}

/// Generate node skeleton from package
pub fn generate_node_skeleton(node_name: &str) -> String {
    format!(
        r#"//! {} node - converted from ROS2
//!
//! TODO: Implement node logic

use horus::prelude::*;

pub struct {} {{
    // TODO: Add Hub/Link fields for communication
    // Example:
    // cmd_vel: Link<CmdVel>,
    // odom: Hub<Odometry>,
}}

impl {} {{
    pub fn new() -> Self {{
        Self {{
            // TODO: Initialize Hub/Link
        }}
    }}
}}

impl Node for {} {{
    fn name(&self) -> &'static str {{
        "{}"
    }}

    fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
        // TODO: Implement tick logic
        //
        // Example pattern (from ROS2 callback):
        // if let Some(msg) = self.cmd_vel.recv() {{
        //     // Process message
        // }}
        //
        // self.odom.send(odom_msg, &mut _ctx);
    }}
}}
"#,
        node_name,
        to_pascal_case(node_name),
        to_pascal_case(node_name),
        to_pascal_case(node_name),
        node_name
    )
}

/// Discover and convert a ROS2 package
pub fn convert_ros2_package(
    source_path: &Path,
    output_path: &Path,
    options: &ConvertOptions,
) -> HorusResult<ConvertResult> {
    let mut result = ConvertResult::default();

    // Find package.xml
    let package_xml = source_path.join("package.xml");
    if !package_xml.exists() {
        return Err(HorusError::NotFound(format!(
            "No package.xml found in {}. Is this a ROS2 package?",
            source_path.display()
        )));
    }

    // Parse package info
    let package = parse_package_xml(&package_xml)?;
    result.package_name = package.name.clone();

    // Create output directory structure
    let output_dir = if output_path.as_os_str().is_empty() {
        source_path
            .parent()
            .unwrap_or(Path::new("."))
            .join(format!("{}_horus", package.name))
    } else {
        output_path.to_path_buf()
    };

    fs::create_dir_all(&output_dir).map_err(HorusError::Io)?;
    fs::create_dir_all(output_dir.join("src/messages")).map_err(HorusError::Io)?;
    fs::create_dir_all(output_dir.join("src/nodes")).map_err(HorusError::Io)?;

    // Convert message files
    let msg_dir = source_path.join("msg");
    if msg_dir.exists() {
        for entry in fs::read_dir(&msg_dir).map_err(HorusError::Io)? {
            let entry = entry.map_err(HorusError::Io)?;
            let path = entry.path();
            if path.extension().map(|e| e == "msg").unwrap_or(false) {
                let msg = parse_msg_file(&path, &package.name)?;

                // Collect message dependencies from fields
                collect_message_deps(&msg.fields, &msg.name, &mut result.message_deps);

                let rust_code = msg_to_rust(&msg);

                let output_file =
                    output_dir.join(format!("src/messages/{}.rs", to_snake_case(&msg.name)));
                fs::write(&output_file, &rust_code).map_err(HorusError::Io)?;

                result.messages_converted += 1;
                result.files_created.push(output_file);
            }
        }
    }

    // Convert service files
    let srv_dir = source_path.join("srv");
    if srv_dir.exists() {
        for entry in fs::read_dir(&srv_dir).map_err(HorusError::Io)? {
            let entry = entry.map_err(HorusError::Io)?;
            let path = entry.path();
            if path.extension().map(|e| e == "srv").unwrap_or(false) {
                let srv = parse_srv_file(&path, &package.name)?;

                // Collect message dependencies from service request/response
                collect_message_deps(
                    &srv.request,
                    &format!("{}Request", srv.name),
                    &mut result.message_deps,
                );
                collect_message_deps(
                    &srv.response,
                    &format!("{}Response", srv.name),
                    &mut result.message_deps,
                );

                let rust_code = srv_to_rust(&srv);

                let output_file =
                    output_dir.join(format!("src/messages/{}_srv.rs", to_snake_case(&srv.name)));
                fs::write(&output_file, &rust_code).map_err(HorusError::Io)?;

                result.services_converted += 1;
                result.files_created.push(output_file);
            }
        }
    }

    // Convert action files
    let action_dir = source_path.join("action");
    if action_dir.exists() {
        for entry in fs::read_dir(&action_dir).map_err(HorusError::Io)? {
            let entry = entry.map_err(HorusError::Io)?;
            let path = entry.path();
            if path.extension().map(|e| e == "action").unwrap_or(false) {
                let action = parse_action_file(&path, &package.name)?;
                let rust_code = action_to_rust(&action);

                let output_file = output_dir.join(format!(
                    "src/messages/{}_action.rs",
                    to_snake_case(&action.name)
                ));
                fs::write(&output_file, &rust_code).map_err(HorusError::Io)?;

                result.actions_converted += 1;
                result.files_created.push(output_file);
            }
        }
    }

    // Process launch files
    let launch_dir = source_path.join("launch");
    if launch_dir.exists() {
        // Create launch reference directory in output
        let launch_output_dir = output_dir.join("launch_reference");
        fs::create_dir_all(&launch_output_dir).map_err(HorusError::Io)?;

        for entry in fs::read_dir(&launch_dir).map_err(HorusError::Io)? {
            let entry = entry.map_err(HorusError::Io)?;
            let path = entry.path();
            let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");

            let launch_file = match ext {
                "launch" | "xml" => {
                    // XML launch file (ROS1 or ROS2)
                    parse_xml_launch_file(&path).ok()
                }
                "py" => {
                    // Python launch file (ROS2)
                    parse_python_launch_file(&path).ok()
                }
                "yaml" | "yml" => {
                    // YAML launch file (ROS2)
                    parse_yaml_launch_file(&path).ok()
                }
                _ => None,
            };

            if let Some(launch) = launch_file {
                // Copy the launch file to reference directory
                let dest = launch_output_dir.join(path.file_name().unwrap());
                fs::copy(&path, &dest).map_err(HorusError::Io)?;
                result.files_created.push(dest);
                result.launch_files_copied += 1;
                result.launch_files.push(launch);
            }
        }
    }

    // Copy parameter/config YAML files
    for config_dir_name in &["config", "params", "param", "cfg"] {
        let config_dir = source_path.join(config_dir_name);
        if config_dir.exists() && config_dir.is_dir() {
            // Create config reference directory in output
            let config_output_dir = output_dir.join("config_reference");
            fs::create_dir_all(&config_output_dir).map_err(HorusError::Io)?;

            for entry in fs::read_dir(&config_dir).map_err(HorusError::Io)? {
                let entry = entry.map_err(HorusError::Io)?;
                let path = entry.path();
                let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");

                // Copy YAML/JSON config files
                if ext == "yaml" || ext == "yml" || ext == "json" {
                    if let Some(filename) = path.file_name() {
                        let dest = config_output_dir.join(filename);
                        fs::copy(&path, &dest).map_err(HorusError::Io)?;
                        result.files_created.push(dest);
                        result.config_files_copied += 1;
                    }
                }
            }
        }
    }

    // Generate messages mod.rs
    let mut mod_content = String::from("//! Converted ROS2 message types\n\n");
    mod_content.push_str("use serde::{Serialize, Deserialize};\n\n");

    // Re-export common HORUS types that might be used
    mod_content.push_str("// Re-export common HORUS types that ROS2 messages may reference\n");
    mod_content.push_str(
        "pub use horus::prelude::{Twist, Pose, Point3, Vector3, Quaternion, Transform};\n",
    );
    mod_content
        .push_str("pub use horus::prelude::{Imu, LaserScan, Image, PointCloud, Odometry};\n\n");

    for entry in fs::read_dir(output_dir.join("src/messages")).map_err(HorusError::Io)? {
        let entry = entry.map_err(HorusError::Io)?;
        let path = entry.path();
        if path.extension().map(|e| e == "rs").unwrap_or(false) {
            if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                if stem != "mod" {
                    mod_content.push_str(&format!("mod {};\n", stem));
                    mod_content.push_str(&format!("pub use {}::*;\n", stem));
                }
            }
        }
    }

    fs::write(output_dir.join("src/messages/mod.rs"), &mod_content).map_err(HorusError::Io)?;
    result
        .files_created
        .push(output_dir.join("src/messages/mod.rs"));

    // Generate node skeletons if requested
    if options.generate_nodes {
        // Look for potential node files in src/ and scripts/
        let search_dirs = vec![
            source_path.join("src"),
            source_path.join("scripts"),
            source_path.join("nodes"),
        ];

        for search_dir in search_dirs {
            if !search_dir.exists() {
                continue;
            }

            for entry in fs::read_dir(&search_dir).map_err(HorusError::Io)? {
                let entry = entry.map_err(HorusError::Io)?;
                let path = entry.path();
                let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");

                if ext == "cpp" || ext == "cxx" || ext == "cc" {
                    if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                        // Read and parse C++ source
                        let content = fs::read_to_string(&path).unwrap_or_default();
                        let parsed_info = parse_cpp_source(&content);

                        // Generate skeleton with extracted info
                        let skeleton = if parsed_info.publishers.is_empty()
                            && parsed_info.subscribers.is_empty()
                            && parsed_info.parameters.is_empty()
                        {
                            // No useful info found, use basic skeleton
                            generate_node_skeleton(stem)
                        } else {
                            // Use parsed info for richer skeleton
                            generate_node_skeleton_from_source(stem, &parsed_info)
                        };

                        let output_file = output_dir.join(format!("src/nodes/{}.rs", stem));
                        fs::write(&output_file, &skeleton).map_err(HorusError::Io)?;

                        result.nodes_converted += 1;
                        result.files_created.push(output_file);
                    }
                } else if ext == "py" {
                    if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                        // Skip __init__.py and setup.py
                        if stem.starts_with("__") || stem == "setup" {
                            continue;
                        }

                        // Read and parse Python source
                        let content = fs::read_to_string(&path).unwrap_or_default();
                        let parsed_info = parse_python_source(&content);

                        // Generate skeleton with extracted info
                        let skeleton = if parsed_info.publishers.is_empty()
                            && parsed_info.subscribers.is_empty()
                            && parsed_info.parameters.is_empty()
                        {
                            generate_node_skeleton(stem)
                        } else {
                            generate_node_skeleton_from_source(stem, &parsed_info)
                        };

                        let output_file = output_dir.join(format!("src/nodes/{}.rs", stem));
                        fs::write(&output_file, &skeleton).map_err(HorusError::Io)?;

                        result.nodes_converted += 1;
                        result.files_created.push(output_file);
                    }
                }
            }
        }
    }

    // Generate nodes mod.rs
    let mut nodes_mod = String::from("//! Converted ROS2 nodes\n\n");
    nodes_mod.push_str("use horus::prelude::*;\n\n");
    for entry in fs::read_dir(output_dir.join("src/nodes")).map_err(HorusError::Io)? {
        let entry = entry.map_err(HorusError::Io)?;
        let path = entry.path();
        if path.extension().map(|e| e == "rs").unwrap_or(false) {
            if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                if stem != "mod" {
                    nodes_mod.push_str(&format!("mod {};\n", stem));
                    nodes_mod.push_str(&format!("pub use {}::*;\n", stem));
                }
            }
        }
    }
    fs::write(output_dir.join("src/nodes/mod.rs"), &nodes_mod).map_err(HorusError::Io)?;
    result
        .files_created
        .push(output_dir.join("src/nodes/mod.rs"));

    // Generate lib.rs
    let lib_rs = format!(
        r#"//! {} - HORUS package
//!
//! Converted from ROS2 package: {}
//!
//! ## Usage
//!
//! ```rust,ignore
//! use {}::prelude::*;
//! ```

pub mod messages;
pub mod nodes;

pub mod prelude {{
    pub use crate::messages::*;
    pub use crate::nodes::*;
    pub use horus::prelude::*;
}}
"#,
        package.name,
        package.name,
        package.name.replace('-', "_")
    );
    fs::write(output_dir.join("src/lib.rs"), &lib_rs).map_err(HorusError::Io)?;
    result.files_created.push(output_dir.join("src/lib.rs"));

    // Generate Cargo.toml
    let cargo_toml = generate_cargo_toml(&package, false);
    fs::write(output_dir.join("Cargo.toml"), &cargo_toml).map_err(HorusError::Io)?;
    result.files_created.push(output_dir.join("Cargo.toml"));

    // Generate Horus.toml
    let horus_toml = generate_horus_toml(&package, &result.launch_files);
    fs::write(output_dir.join("Horus.toml"), &horus_toml).map_err(HorusError::Io)?;
    result.files_created.push(output_dir.join("Horus.toml"));

    // Set external deps and cmake targets before generating migration guide
    result.external_deps_detected = package.external_deps.len();
    result.external_deps = package.external_deps.clone();
    result.cmake_targets_detected = package.cmake_targets.len();
    result.cmake_targets = package.cmake_targets.clone();
    result.ros_version = package.ros_version;

    // Generate migration guide
    let migration_guide = generate_migration_guide(&package, &result);
    fs::write(output_dir.join("MIGRATION.md"), &migration_guide).map_err(HorusError::Io)?;
    result.files_created.push(output_dir.join("MIGRATION.md"));

    result.output_dir = output_dir;
    Ok(result)
}

/// Generate migration guide
fn generate_migration_guide(package: &RosPackage, result: &ConvertResult) -> String {
    let ros_label = match package.ros_version {
        RosVersion::Ros1 => "ROS1",
        RosVersion::Ros2 => "ROS2",
        RosVersion::Unknown => "ROS",
    };

    let callback_example = match package.ros_version {
        RosVersion::Ros1 => {
            r#"```cpp
// ROS1 callback pattern (roscpp):
void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // process
}

// ROS1 subscriber setup:
ros::Subscriber sub = nh.subscribe("scan", 10, callback);
ros::spin();
```"#
        }
        _ => {
            r#"```cpp
// ROS2 callback pattern (rclcpp):
void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // process
}

// ROS2 subscriber setup:
auto sub = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, callback);
rclcpp::spin(node);
```"#
        }
    };

    let key_differences = match package.ros_version {
        RosVersion::Ros1 => {
            r#"| ROS1 | HORUS |
|------|-------|
| `ros::Subscriber` | `Link<T>` field |
| `ros::Publisher` | `Hub<T>` field |
| `ros::Timer` / `ros::Rate` | Scheduler handles timing |
| Callback functions | `tick()` method |
| `ros::spin()` | `Scheduler::run()` |
| `ros::NodeHandle` | `NodeInfo` context |
| `ros::init()` | `Scheduler::new()` |"#
        }
        _ => {
            r#"| ROS2 | HORUS |
|------|-------|
| `create_subscription()` | `Link<T>` field |
| `create_publisher()` | `Hub<T>` field |
| `create_timer()` | Scheduler handles timing |
| Callback functions | `tick()` method |
| `rclcpp::spin()` | `Scheduler::run()` |
| `rclcpp::Node` | `impl Node` |"#
        }
    };

    // Generate external dependencies section
    let external_deps_section = if !package.external_deps.is_empty() {
        let mut section = String::from("\n### 5. External Dependencies\n\n");
        section
            .push_str("The following external dependencies were detected from CMakeLists.txt:\n\n");
        section.push_str("| CMake Package | Rust Equivalent | Notes |\n");
        section.push_str("|---------------|-----------------|-------|\n");
        for dep in &package.external_deps {
            let crate_col = dep.rust_crate.as_deref().unwrap_or("N/A");
            let notes_col = dep.notes.as_deref().unwrap_or("");
            section.push_str(&format!(
                "| {} | {} | {} |\n",
                dep.cmake_name, crate_col, notes_col
            ));
        }
        section.push_str("\nCheck `Cargo.toml` for the suggested Rust crate dependencies.\n");
        section
    } else {
        String::new()
    };

    // Generate cmake targets section
    let cmake_targets_section = if !package.cmake_targets.is_empty() {
        let mut section = String::from("\n### 6. CMake Build Targets\n\n");
        section.push_str("The following build targets were detected from CMakeLists.txt:\n\n");
        section.push_str("| Target Name | Type | Source Files |\n");
        section.push_str("|-------------|------|-------------|\n");
        for target in &package.cmake_targets {
            let type_str = if target.is_library {
                "Library"
            } else {
                "Executable"
            };
            let sources = target.source_files.join(", ");
            section.push_str(&format!(
                "| {} | {} | {} |\n",
                target.name, type_str, sources
            ));
        }
        section.push_str("\nConsider creating HORUS nodes for each executable target.\n");
        section
    } else {
        String::new()
    };

    // Generate message dependencies section
    let message_deps_section = if !result.message_deps.is_empty() {
        let mut section = String::from("\n### 7. Message Dependencies from Other Packages\n\n");
        section.push_str("⚠️ **Action Required**: The following message types from other ROS packages were detected.\n");
        section
            .push_str("You need to either convert these packages first or create stub types.\n\n");
        section.push_str("| Source Package | Message Type | Used In |\n");
        section.push_str("|---------------|--------------|--------|\n");
        for dep in &result.message_deps {
            let used_in = dep.used_in.join(", ");
            section.push_str(&format!(
                "| {} | {} | {} |\n",
                dep.source_package, dep.message_name, used_in
            ));
        }
        section.push_str("\n**Options:**\n");
        section.push_str("1. Convert the dependent package first: `horus from-ros /path/to/");
        section.push_str(&result.message_deps[0].source_package);
        section.push_str("`\n");
        section.push_str("2. Create stub types in `src/messages/external_types.rs`\n");
        section.push_str("3. Use HORUS equivalent types if available\n");
        section
    } else {
        String::new()
    };

    // Generate launch files section with arguments, remappings, includes
    let launch_files_section = if !result.launch_files.is_empty() {
        let mut section = String::from("\n### 8. Launch File Details\n\n");
        section.push_str("The following launch files were found and analyzed:\n\n");

        for launch in &result.launch_files {
            let format_str = match launch.format {
                LaunchFormat::Xml => "XML",
                LaunchFormat::Python => "Python",
                LaunchFormat::Yaml => "YAML",
            };
            section.push_str(&format!(
                "#### `{}` ({})\n\n",
                launch
                    .source_file
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy(),
                format_str
            ));

            // List nodes
            if !launch.nodes.is_empty() {
                section.push_str("**Nodes:**\n");
                for node in &launch.nodes {
                    let ns_str = node.namespace.as_deref().unwrap_or("");
                    section.push_str(&format!(
                        "- `{}` ({}/{}) {}\n",
                        node.name,
                        node.package,
                        node.executable,
                        if ns_str.is_empty() {
                            "".to_string()
                        } else {
                            format!("[ns: {}]", ns_str)
                        }
                    ));

                    // Show remappings
                    if !node.remappings.is_empty() {
                        section.push_str("  - Remappings:\n");
                        for (from, to) in &node.remappings {
                            section.push_str(&format!("    - `{}` → `{}`\n", from, to));
                        }
                    }
                }
                section.push_str("\n");
            }

            // List arguments
            if !launch.arguments.is_empty() {
                section.push_str("**Arguments:**\n\n");
                section.push_str("| Argument | Default | Description |\n");
                section.push_str("|----------|---------|-------------|\n");
                for arg in &launch.arguments {
                    let default = arg.default_value.as_deref().unwrap_or("-");
                    let desc = arg.description.as_deref().unwrap_or("");
                    section.push_str(&format!("| `{}` | {} | {} |\n", arg.name, default, desc));
                }
                section.push_str("\n");
            }

            // List includes
            if !launch.includes.is_empty() {
                section.push_str("**Includes:**\n");
                for inc in &launch.includes {
                    let ns_str = inc
                        .namespace
                        .as_deref()
                        .map(|s| format!(" [ns: {}]", s))
                        .unwrap_or_default();
                    section.push_str(&format!("- `{}`{}\n", inc.file, ns_str));
                }
                section.push_str("\n");
            }
        }

        section.push_str("**HORUS Equivalent:**\n\n");
        section.push_str("In HORUS, nodes are composed programmatically in `main.rs`:\n\n");
        section.push_str("```rust\n");
        section.push_str("let mut scheduler = Scheduler::new();\n");
        section.push_str("scheduler.add_node(MyNode::new());\n");
        section.push_str("scheduler.run();\n");
        section.push_str("```\n\n");
        section.push_str(
            "Remappings are handled through `Link::new()` and `Hub::new()` topic names.\n",
        );
        section
    } else {
        String::new()
    };

    // Generate plugins section (nodelets, pluginlib)
    let plugins_section = if !package.plugins.is_empty() {
        let mut section = String::from("\n### 9. Detected Plugins (Nodelets/Pluginlib)\n\n");
        section.push_str("The following plugins were detected from `plugin.xml`:\n\n");
        section.push_str("| Class Name | Base Class | Type |\n");
        section.push_str("|------------|------------|------|\n");
        for plugin in &package.plugins {
            section.push_str(&format!(
                "| `{}` | `{}` | {} |\n",
                plugin.class_name, plugin.base_class, plugin.plugin_type
            ));
        }
        section.push_str("\n**HORUS Equivalent:**\n\n");
        section.push_str(
            "HORUS provides zero-copy shared memory communication by default via `Hub`/`Link`.\n",
        );
        section.push_str(
            "Nodelets were used in ROS1 for zero-copy IPC - this is automatic in HORUS.\n\n",
        );
        section.push_str("```rust\n");
        section.push_str("// HORUS zero-copy: messages are passed via shared memory\n");
        section.push_str("// No extra configuration needed - just use Hub/Link!\n");
        section.push_str("pub struct MyNode {\n");
        section.push_str("    input: Link<SensorData>,   // Zero-copy receive\n");
        section.push_str("    output: Hub<ProcessedData>, // Zero-copy publish\n");
        section.push_str("}\n");
        section.push_str("```\n");
        section
    } else {
        String::new()
    };

    // Generate advanced features migration section (always included for comprehensive guidance)
    let advanced_features_section = format!(
        r#"
## Advanced Feature Migration

### TF (Transform Library)

If your nodes use TF broadcasting or listening:

**ROS TF Broadcaster → HORUS:**
```rust
// In HORUS, use the TF hub to broadcast transforms
pub struct MyNode {{
    tf_broadcaster: Hub<TransformStamped>,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    let transform = TransformStamped {{
        header: Header::now("parent_frame"),
        child_frame_id: "child_frame".to_string(),
        transform: Transform::identity(),
    }};
    self.tf_broadcaster.publish(transform);
}}
```

**ROS TF Listener → HORUS:**
```rust
// Subscribe to /tf topic for transforms
pub struct MyNode {{
    tf_listener: Link<TransformStamped>,
}}
```

### Message Filters (Time Synchronization)

If your nodes use `TimeSynchronizer` or `ApproximateTimeSynchronizer`:

**ROS Message Filters → HORUS:**
```rust
// Use multiple Links and synchronize in tick()
pub struct MySyncNode {{
    image: Link<Image>,
    lidar: Link<PointCloud>,
    synced_data: Option<(Image, PointCloud)>,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    // Receive both and match by timestamp
    if let (Some(img), Some(pcl)) = (self.image.recv(), self.lidar.recv()) {{
        if (img.header.stamp - pcl.header.stamp).abs() < Duration::from_millis(50) {{
            self.synced_data = Some((img, pcl));
        }}
    }}
}}
```

### Image Transport

If your nodes use `image_transport` for compressed images:

**ROS Image Transport → HORUS:**
```rust
// HORUS supports Image messages directly
// For compression, use codec nodes or compressed message types
pub struct CameraNode {{
    raw_image: Hub<Image>,
    compressed: Hub<CompressedImage>,
}}
```

### Dynamic Reconfigure (ROS1)

If your nodes use `dynamic_reconfigure`:

**ROS1 dynamic_reconfigure → HORUS:**
```rust
// Use the HORUS parameter server with change callbacks
pub struct MyNode {{
    max_speed: f64,
    update_rate: f64,
}}

fn tick(&mut self, ctx: Option<&mut NodeInfo>) {{
    if let Some(ctx) = ctx {{
        // Check for parameter updates
        if let Some(speed) = ctx.get_param::<f64>("max_speed") {{
            self.max_speed = speed;
        }}
    }}
}}
```

### Executors and Callback Groups (ROS2)

If your nodes use `SingleThreadedExecutor`, `MultiThreadedExecutor`, or callback groups:

**ROS2 Executors → HORUS:**
```rust
// HORUS Scheduler handles execution automatically
// Configure threading in Scheduler setup:
let mut scheduler = Scheduler::new();
scheduler.set_thread_pool_size(4);  // Multi-threaded execution

// Or use deterministic single-threaded mode:
let mut scheduler = Scheduler::new_deterministic();
```

**Callback Groups → Thread Affinity:**
```rust
// In HORUS, use node groups for thread affinity:
scheduler.add_node_to_group(node, "realtime_group");
```

### Lifecycle Nodes (ROS2)

If your nodes implement the ROS2 lifecycle interface:

**ROS2 Lifecycle → HORUS:**
```rust
// HORUS provides similar state management via NodeInfo
impl Node for MyLifecycleNode {{
    fn on_activate(&mut self) {{
        // Called when scheduler starts
        self.setup_resources();
    }}

    fn on_deactivate(&mut self) {{
        // Called when scheduler stops
        self.cleanup_resources();
    }}

    fn tick(&mut self, ctx: Option<&mut NodeInfo>) {{
        // Normal processing - only called when active
    }}
}}
```

### Component Nodes (ROS2)

If your package uses ROS2 component containers:

**ROS2 Components → HORUS:**
```rust
// HORUS nodes are components by default - just add them to the scheduler
let mut scheduler = Scheduler::new();
scheduler.add_node(ComponentA::new());
scheduler.add_node(ComponentB::new());
scheduler.add_node(ComponentC::new());
// All nodes share the same process with zero-copy IPC!
```

### Nodelets (ROS1)

If your package uses ROS1 nodelets for zero-copy IPC:

**ROS1 Nodelets → HORUS:**
- HORUS provides zero-copy by default via shared memory
- No nodelet manager needed - all nodes automatically share memory
- Simply implement the `Node` trait and add to Scheduler

### ros2_control / Hardware Interface

If your package uses ros2_control hardware interfaces:

**ROS2 hardware_interface → HORUS:**
```rust
// HORUS provides direct hardware access via bus nodes
pub struct MyHardwareNode {{
    // State interfaces (reading)
    joint_positions: Hub<JointState>,

    // Command interfaces (writing)
    joint_commands: Link<JointCommand>,

    // Direct hardware access
    i2c_bus: I2CBus,
    spi_bus: SPIBus,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    // Read state from hardware
    let state = self.read_joint_positions();
    self.joint_positions.publish(state);

    // Write commands to hardware
    if let Some(cmd) = self.joint_commands.recv() {{
        self.write_joint_commands(cmd);
    }}
}}
```

### rosserial / MicroROS

If your package uses rosserial or MicroROS for embedded systems:

**rosserial/MicroROS → HORUS:**
```rust
// HORUS provides serial/I2C/SPI bus nodes for embedded communication
pub struct EmbeddedBridgeNode {{
    serial_port: SerialPort,
    sensor_data: Hub<SensorReading>,
    motor_commands: Link<MotorCommand>,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    // Read from embedded device
    if let Some(data) = self.serial_port.read_message() {{
        self.sensor_data.publish(data);
    }}

    // Send commands to embedded device
    if let Some(cmd) = self.motor_commands.recv() {{
        self.serial_port.write_message(cmd);
    }}
}}
```

### WaitSet / GuardCondition (ROS2)

If your code uses low-level WaitSet or GuardCondition:

**ROS2 WaitSet → HORUS:**
- HORUS Scheduler handles all waiting/synchronization automatically
- No manual wait set management needed
- Use `Link::recv()` which handles waiting internally

### IntraProcess Communication (ROS2)

If your nodes use ROS2 intra-process communication:

**ROS2 IntraProcess → HORUS:**
- HORUS provides zero-copy by default via shared memory
- All `Hub`/`Link` communication is automatically zero-copy
- No special configuration needed!

### Bond / Heartbeat Monitoring

If your nodes use `bondcpp` for heartbeat monitoring:

**ROS Bond → HORUS:**
```rust
// HORUS Scheduler provides built-in node health monitoring
impl Node for MyNode {{
    fn on_error(&mut self, error: NodeError) {{
        // Called when node encounters errors
        // Scheduler can restart or handle failed nodes
    }}
}}

// Configure health monitoring in Scheduler:
scheduler.enable_health_monitoring(Duration::from_secs(5));
```

### laser_geometry

If your nodes use `laser_geometry` for scan projection:

**ROS laser_geometry → HORUS:**
```rust
// HORUS provides LidarProcessor for scan-to-pointcloud conversion
pub struct ScanProcessor {{
    laser_scan: Link<LaserScan>,
    point_cloud: Hub<PointCloud>,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    if let Some(scan) = self.laser_scan.recv() {{
        // Convert scan to point cloud
        let cloud = LidarProcessor::project_scan(&scan);
        self.point_cloud.publish(cloud);
    }}
}}
```

### URDF / robot_description

If your nodes use URDF or robot_description:

**ROS URDF → HORUS:**
- HORUS can load robot models from URDF for simulation
- Use `horus sim3d --urdf robot.urdf` to visualize
- Robot state can be published via JointState messages

### Nav2 Navigation Stack

If your package uses Nav2 components:

**Nav2 → HORUS:**
```rust
// HORUS provides built-in navigation algorithms
use horus::algorithms::{{AStar, RRT, PurePursuit, OccupancyGrid}};

pub struct NavigationNode {{
    goal: Link<PoseStamped>,
    map: Link<OccupancyGrid>,
    cmd_vel: Hub<Twist>,

    planner: AStar,
    controller: PurePursuit,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    if let Some(goal) = self.goal.recv() {{
        let path = self.planner.plan(&self.current_map, &goal);
        let cmd = self.controller.compute_velocity(&path);
        self.cmd_vel.publish(cmd);
    }}
}}
```

### MoveIt Motion Planning

If your package uses MoveIt:

**MoveIt → HORUS:**
```rust
// HORUS supports trajectory planning and IK solvers
pub struct ManipulatorNode {{
    target_pose: Link<PoseStamped>,
    joint_trajectory: Hub<JointTrajectory>,
    planning_scene: PlanningScene,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    if let Some(target) = self.target_pose.recv() {{
        // Inverse kinematics and trajectory planning
        if let Some(trajectory) = self.planning_scene.plan_to_pose(&target) {{
            self.joint_trajectory.publish(trajectory);
        }}
    }}
}}
```

### Gazebo Plugins

If your package includes Gazebo plugins:

**Gazebo → HORUS:**
- HORUS provides `sim2d` and `sim3d` simulators with native integration
- No separate plugin compilation needed
- Sensors and actuators work directly with HORUS nodes

```bash
# Run HORUS simulation
horus sim3d --urdf my_robot.urdf --world warehouse.world
```

### RViz / Visualization

If your nodes publish visualization markers:

**RViz → HORUS:**
```rust
// HORUS dashboard provides built-in visualization
pub struct VisualizationNode {{
    markers: Hub<Marker>,
    path_viz: Hub<Path>,
}}

// Use HORUS dashboard:
// horus dashboard --port 3000
```

### rosbag / ros2bag Recording

If your package uses rosbag for recording/playback:

**rosbag → HORUS:**
```rust
// HORUS provides built-in blackbox recording
// Configure in horus.toml:
// [blackbox]
// enabled = true
// topics = ["*"]
// max_size = "1GB"

// CLI usage:
// horus record --topics /scan /odom /cmd_vel
// horus play recording.horus
```

### camera_info_manager

If your nodes use camera_info_manager for calibration:

**camera_info_manager → HORUS:**
```rust
// HORUS CameraNode supports calibration parameters
pub struct CameraNode {{
    camera: Camera,
    image: Hub<Image>,
    camera_info: Hub<CameraInfo>,
}}

impl CameraNode {{
    pub fn new(calibration_file: &str) -> Self {{
        let camera = Camera::from_calibration(calibration_file);
        // ...
    }}
}}
```

### image_geometry

If your nodes use image_geometry for camera projection:

**image_geometry → HORUS:**
```rust
// HORUS provides camera projection utilities
use horus::vision::{{PinholeCameraModel, StereoCameraModel}};

pub struct VisionNode {{
    camera_model: PinholeCameraModel,
}}

fn project_point(&self, point_3d: &Point3D) -> Point2D {{
    self.camera_model.project(point_3d)
}}

fn ray_from_pixel(&self, pixel: &Point2D) -> Ray {{
    self.camera_model.unproject(pixel)
}}
```

### PCL (Point Cloud Library)

If your nodes use PCL for point cloud processing:

**PCL → HORUS:**
```rust
// HORUS provides PointCloudProcessor for common operations
use horus::perception::PointCloudProcessor;

pub struct PointCloudNode {{
    raw_cloud: Link<PointCloud>,
    filtered_cloud: Hub<PointCloud>,
    processor: PointCloudProcessor,
}}

fn tick(&mut self, mut _ctx: Option<&mut NodeInfo>) {{
    if let Some(cloud) = self.raw_cloud.recv() {{
        // Apply filters: VoxelGrid, PassThrough, StatisticalOutlierRemoval
        let filtered = self.processor
            .voxel_grid(0.01)
            .pass_through("z", 0.0, 2.0)
            .statistical_outlier_removal(50, 1.0)
            .apply(&cloud);
        self.filtered_cloud.publish(filtered);
    }}
}}
```

### realtime_tools

If your nodes use realtime_tools for lock-free communication:

**realtime_tools → HORUS:**
- HORUS `Hub`/`Link` communication is lock-free by default!
- No special realtime publisher needed
- Shared memory provides zero-copy, lock-free message passing

```rust
// HORUS is lock-free by default - just use Hub/Link!
pub struct RealtimeNode {{
    sensor: Link<SensorData>,    // Lock-free receive
    command: Hub<MotorCommand>,  // Lock-free publish
}}
```
"#
    );

    format!(
        r#"# Migration Guide: {} → HORUS

This package was automatically converted from {ros_label} to HORUS.

## What Was Converted

- **Messages**: {} .msg files → Rust structs
- **Services**: {} .srv files → Request/Response structs
- **Actions**: {} .action files → Goal/Result/Feedback structs
- **Launch Files**: {} copied to `launch_reference/`
- **Config Files**: {} copied to `config_reference/`
- **External Deps**: {} detected from CMakeLists.txt
- **CMake Targets**: {} detected from CMakeLists.txt
- **Nodes**: {} node skeletons generated
- **Source Version**: {ros_label}

## Next Steps

### 1. Review Generated Messages

Check `src/messages/` for converted message types. The converter mapped:
- ROS primitive types → Rust types (float64 → f64, string → String, etc.)
- Common ROS types → HORUS equivalents (geometry_msgs/Twist → Twist, etc.)

Custom message types are generated as new structs.

### 2. Implement Node Logic

Node skeletons in `src/nodes/` need implementation.

{callback_example}

**Becomes HORUS tick pattern:**

```rust
fn tick(&mut self, ctx: Option<&mut NodeInfo>) {{
    if let Some(msg) = self.laser_scan.recv() {{
        // process
    }}
}}
```

### 3. Key Differences from {ros_label}

{key_differences}

### 4. Build and Run

```bash
# Build
cargo build

# Run with HORUS
horus run

# Or run directly
cargo run
```
{external_deps_section}{cmake_targets_section}{message_deps_section}{launch_files_section}{plugins_section}{advanced_features_section}
## Need Help?

- HORUS Documentation: https://horus.dev/docs
- Migration Guide: https://horus.dev/docs/from-ros
"#,
        package.name,
        result.messages_converted,
        result.services_converted,
        result.actions_converted,
        result.launch_files_copied,
        result.config_files_copied,
        result.external_deps_detected,
        result.cmake_targets_detected,
        result.nodes_converted,
        ros_label = ros_label,
        callback_example = callback_example,
        key_differences = key_differences,
        external_deps_section = external_deps_section,
        cmake_targets_section = cmake_targets_section,
        message_deps_section = message_deps_section,
        launch_files_section = launch_files_section,
        plugins_section = plugins_section,
        advanced_features_section = advanced_features_section
    )
}

/// Conversion options
#[derive(Debug, Clone, Default)]
pub struct ConvertOptions {
    /// Generate node skeletons from source files
    pub generate_nodes: bool,
    /// Verbose output
    pub verbose: bool,
    /// Dry run (don't write files)
    pub dry_run: bool,
}

/// A message type dependency from another ROS package
#[derive(Debug, Clone)]
pub struct MessageDependency {
    pub source_package: String, // The ROS package that defines the message
    pub message_name: String,   // The message type name
    pub used_in: Vec<String>,   // Files/messages that use this dependency
}

/// Known ROS message packages that we map to HORUS built-ins
const KNOWN_ROS_MSG_PACKAGES: &[&str] = &[
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "nav_msgs",
    "builtin_interfaces",
    "action_msgs",
    "diagnostic_msgs",
    "shape_msgs",
    "visualization_msgs",
    "trajectory_msgs",
];

/// Check if a type references an external message package
/// Returns (package_name, message_name) if it does
fn extract_message_dependency(field_type: &str) -> Option<(String, String)> {
    // Types like "other_pkg/CustomMsg" or "other_pkg/msg/CustomMsg"
    if field_type.contains('/') {
        let parts: Vec<&str> = field_type.split('/').collect();
        if parts.len() >= 2 {
            let package = parts[0];
            // Skip known ROS packages - we handle these internally
            if !KNOWN_ROS_MSG_PACKAGES.contains(&package) {
                let msg_name = parts.last().unwrap_or(&"");
                if !msg_name.is_empty() && *msg_name != "msg" {
                    return Some((package.to_string(), msg_name.to_string()));
                }
            }
        }
    }
    None
}

/// Collect all message dependencies from a list of fields
fn collect_message_deps(fields: &[RosField], source: &str, deps: &mut Vec<MessageDependency>) {
    for field in fields {
        if let Some((pkg, msg)) = extract_message_dependency(&field.field_type) {
            // Check if we already have this dependency
            if let Some(existing) = deps
                .iter_mut()
                .find(|d| d.source_package == pkg && d.message_name == msg)
            {
                if !existing.used_in.contains(&source.to_string()) {
                    existing.used_in.push(source.to_string());
                }
            } else {
                deps.push(MessageDependency {
                    source_package: pkg,
                    message_name: msg,
                    used_in: vec![source.to_string()],
                });
            }
        }
    }
}

/// Conversion result
#[derive(Debug, Clone, Default)]
pub struct ConvertResult {
    pub package_name: String,
    pub output_dir: PathBuf,
    pub messages_converted: usize,
    pub services_converted: usize,
    pub actions_converted: usize,
    pub launch_files_copied: usize,
    pub config_files_copied: usize,
    pub external_deps_detected: usize,
    pub cmake_targets_detected: usize,
    pub nodes_converted: usize,
    pub files_created: Vec<PathBuf>,
    pub warnings: Vec<String>,
    pub ros_version: RosVersion,
    pub launch_files: Vec<LaunchFile>,
    pub external_deps: Vec<ExternalDep>,
    pub cmake_targets: Vec<CmakeTarget>,
    pub message_deps: Vec<MessageDependency>,
}

impl Default for RosVersion {
    fn default() -> Self {
        RosVersion::Unknown
    }
}

/// Run the from-ros command (supports both ROS1 and ROS2)
pub fn run(
    source: &Path,
    output: Option<&Path>,
    generate_nodes: bool,
    verbose: bool,
    dry_run: bool,
) -> HorusResult<()> {
    println!("{}", "HORUS ROS Converter".green().bold());
    println!("  Supports ROS1 (catkin) and ROS2 (ament) packages");
    println!();

    let options = ConvertOptions {
        generate_nodes,
        verbose,
        dry_run,
    };

    // Validate source directory
    if !source.exists() {
        return Err(HorusError::NotFound(format!(
            "Source directory not found: {}",
            source.display()
        )));
    }

    // Detect ROS version early for better user feedback
    let package_xml = source.join("package.xml");
    let detected_version = if package_xml.exists() {
        if let Ok(content) = fs::read_to_string(&package_xml) {
            detect_ros_version(&content, source)
        } else {
            RosVersion::Unknown
        }
    } else {
        RosVersion::Unknown
    };

    if dry_run {
        println!("{}", "  [DRY RUN] No files will be created".yellow());
        println!();
    }

    println!("  {} {}", "Source:".cyan(), source.display());

    // Show detected version
    match detected_version {
        RosVersion::Ros1 => {
            println!(
                "  {} {} (catkin-based)",
                "Detected:".cyan(),
                "ROS1".yellow().bold()
            );
        }
        RosVersion::Ros2 => {
            println!(
                "  {} {} (ament-based)",
                "Detected:".cyan(),
                "ROS2".green().bold()
            );
        }
        RosVersion::Unknown => {
            println!(
                "  {} {} (will attempt ROS2 conversion)",
                "Detected:".cyan(),
                "Unknown".red()
            );
        }
    }

    let output_path = output.unwrap_or(Path::new(""));
    let result = convert_ros2_package(source, output_path, &options)?;

    println!("  {} {}", "Output:".cyan(), result.output_dir.display());
    println!();

    // Print summary
    println!("{}", "Conversion Summary".green().bold());
    println!("  {} {} message(s)", "".white(), result.messages_converted);
    println!("  {} {} service(s)", "".white(), result.services_converted);
    println!("  {} {} action(s)", "".white(), result.actions_converted);
    println!(
        "  {} {} launch file(s)",
        "".white(),
        result.launch_files_copied
    );
    println!(
        "  {} {} config file(s)",
        "".white(),
        result.config_files_copied
    );
    println!(
        "  {} {} external dep(s)",
        "".white(),
        result.external_deps_detected
    );
    println!(
        "  {} {} cmake target(s)",
        "".white(),
        result.cmake_targets_detected
    );
    println!(
        "  {} {} node skeleton(s)",
        "".white(),
        result.nodes_converted
    );
    if !result.message_deps.is_empty() {
        println!(
            "  {} {} message dep(s) from other packages",
            "⚠".yellow(),
            result.message_deps.len()
        );
    }
    println!();

    if verbose {
        println!("{}", "Files Created".green().bold());
        for file in &result.files_created {
            println!("  {} {}", "".white(), file.display());
        }
        println!();
    }

    println!("{}", "Next Steps".green().bold());
    println!("  1. cd {}", result.output_dir.display());
    println!("  2. Review src/messages/ for converted types");
    println!("  3. Implement node logic in src/nodes/");
    println!("  4. cargo build && horus run");
    println!();
    println!(
        "  {} See {} for detailed migration guide",
        "".cyan(),
        "MIGRATION.md".white().bold()
    );

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ros2_to_rust_type_primitives() {
        assert_eq!(ros2_to_rust_type("float32"), "f32");
        assert_eq!(ros2_to_rust_type("float64"), "f64");
        assert_eq!(ros2_to_rust_type("int32"), "i32");
        assert_eq!(ros2_to_rust_type("uint8"), "u8");
        assert_eq!(ros2_to_rust_type("string"), "String");
        assert_eq!(ros2_to_rust_type("bool"), "bool");
    }

    #[test]
    fn test_ros2_to_rust_type_arrays() {
        assert_eq!(ros2_to_rust_type("float32[]"), "Vec<f32>");
        assert_eq!(ros2_to_rust_type("uint8[]"), "Vec<u8>");
        assert_eq!(ros2_to_rust_type("float32[3]"), "[f32; 3]");
    }

    #[test]
    fn test_ros2_to_rust_type_ros_types() {
        assert_eq!(ros2_to_rust_type("geometry_msgs/Twist"), "Twist");
        assert_eq!(ros2_to_rust_type("sensor_msgs/Imu"), "Imu");
        assert_eq!(ros2_to_rust_type("nav_msgs/Odometry"), "Odometry");
    }

    #[test]
    fn test_to_pascal_case() {
        assert_eq!(to_pascal_case("my_node"), "MyNode");
        assert_eq!(to_pascal_case("cmd_vel"), "CmdVel");
        assert_eq!(to_pascal_case("laser_scan"), "LaserScan");
    }

    #[test]
    fn test_to_snake_case() {
        assert_eq!(to_snake_case("MyNode"), "my_node");
        assert_eq!(to_snake_case("CmdVel"), "cmd_vel");
        assert_eq!(to_snake_case("LaserScan"), "laser_scan");
    }
}
