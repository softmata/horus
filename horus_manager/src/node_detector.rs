//! Auto-detect HORUS nodes used in Python and Rust code
//!
//! Parses source files to find which built-in nodes are imported/used,
//! then automatically enables the required Cargo features.

use std::collections::HashSet;
use std::fs;
use std::path::Path;

/// Node to feature mapping
/// Maps node names to the Cargo features they require
const NODE_FEATURES: &[(&str, &[&str])] = &[
    // Vision nodes
    ("CameraNode", &["opencv-backend"]),
    ("DepthCameraNode", &[]),
    ("ImageProcessorNode", &["opencv-backend"]),
    // Input devices
    ("JoystickInputNode", &["gilrs"]),
    ("KeyboardInputNode", &["crossterm"]),
    // Sensors
    ("BatteryMonitorNode", &["i2c-hardware"]),
    ("ImuNode", &["bno055-imu"]), // Will detect specific backend later
    ("GpsNode", &["nmea-gps"]),
    ("LidarNode", &["rplidar"]),
    ("ForceTorqueSensorNode", &["netft"]),
    ("EncoderNode", &["gpio-hardware"]),
    ("UltrasonicNode", &["gpio-hardware"]),
    // Motors/Actuators
    ("BldcMotorNode", &["gpio-hardware"]),
    ("DcMotorNode", &["gpio-hardware"]),
    ("StepperMotorNode", &["gpio-hardware"]),
    ("ServoControllerNode", &["gpio-hardware"]),
    ("DynamixelNode", &["serial-hardware"]),
    ("RoboclawMotorNode", &["serial-hardware"]),
    // Industrial interfaces
    ("CanBusNode", &[]),
    ("DigitalIONode", &["gpio-hardware"]),
    ("I2cBusNode", &["i2c-hardware"]),
    ("ModbusNode", &["modbus-hardware"]),
    ("SerialNode", &["serial-hardware"]),
    ("SpiBusNode", &["spi-hardware"]),
    // Hardware-independent nodes (no features needed)
    ("CollisionDetectorNode", &[]),
    ("DifferentialDriveNode", &[]),
    ("EmergencyStopNode", &[]),
    ("LocalizationNode", &[]),
    ("OdometryNode", &[]),
    ("PathPlannerNode", &[]),
    ("PidControllerNode", &[]),
    ("SafetyMonitorNode", &[]),
];

/// Detect nodes used in Python code
pub fn detect_python_nodes(source: &str) -> HashSet<String> {
    let mut nodes = HashSet::new();

    for line in source.lines() {
        let line = line.trim();

        // Pattern 1: from horus import CameraNode, LidarNode
        if line.starts_with("from horus import") || line.starts_with("from horus.") {
            let import_part = line.split("import").nth(1).unwrap_or("");
            for token in import_part.split(',') {
                let token = token.trim();
                if token.ends_with("Node") {
                    nodes.insert(token.to_string());
                }
            }
        }

        // Pattern 2: import horus; horus.CameraNode()
        if line.contains("horus.") {
            for (node_name, _) in NODE_FEATURES {
                if line.contains(&format!("horus.{}", node_name)) {
                    nodes.insert(node_name.to_string());
                }
            }
        }

        // Pattern 3: Direct usage like CameraNode()
        for (node_name, _) in NODE_FEATURES {
            if line.contains(&format!("{}(", node_name)) {
                nodes.insert(node_name.to_string());
            }
        }
    }

    nodes
}

/// Detect nodes used in Rust code
pub fn detect_rust_nodes(source: &str) -> HashSet<String> {
    let mut nodes = HashSet::new();

    for line in source.lines() {
        let line = line.trim();

        // Pattern 1: use horus_library::CameraNode;
        // Pattern 2: use horus_library::nodes::CameraNode;
        // Pattern 3: use horus::library::nodes::CameraNode;
        if line.starts_with("use horus_library::") || line.starts_with("use horus::library::") {
            for (node_name, _) in NODE_FEATURES {
                if line.contains(node_name) {
                    nodes.insert(node_name.to_string());
                }
            }
        }

        // Pattern 4: Direct usage like CameraNode::new()
        for (node_name, _) in NODE_FEATURES {
            if line.contains(&format!("{}::", node_name)) {
                nodes.insert(node_name.to_string());
            }
        }
    }

    nodes
}

/// Convert detected nodes to required Cargo features
pub fn nodes_to_features(nodes: &HashSet<String>) -> Vec<String> {
    let mut features = HashSet::new();

    for node_name in nodes {
        // Find matching features for this node
        for (name, node_features) in NODE_FEATURES {
            if *name == node_name {
                features.extend(node_features.iter().map(|s| s.to_string()));
            }
        }
    }

    let mut feature_list: Vec<String> = features.into_iter().collect();
    feature_list.sort();
    feature_list
}

/// Scan a file and detect required features
pub fn detect_features_from_file(file_path: &Path) -> std::io::Result<Vec<String>> {
    let content = fs::read_to_string(file_path)?;

    let nodes = if file_path.extension().and_then(|s| s.to_str()) == Some("py") {
        detect_python_nodes(&content)
    } else if file_path.extension().and_then(|s| s.to_str()) == Some("rs") {
        detect_rust_nodes(&content)
    } else {
        HashSet::new()
    };

    Ok(nodes_to_features(&nodes))
}

/// Scan multiple files and detect all required features
pub fn detect_features_from_files(file_paths: &[&Path]) -> std::io::Result<Vec<String>> {
    let mut all_features = HashSet::new();

    for file_path in file_paths {
        let features = detect_features_from_file(file_path)?;
        all_features.extend(features);
    }

    let mut feature_list: Vec<String> = all_features.into_iter().collect();
    feature_list.sort();
    Ok(feature_list)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_python_import_single() {
        let code = "from horus import CameraNode";
        let nodes = detect_python_nodes(code);
        assert!(nodes.contains("CameraNode"));
    }

    #[test]
    fn test_detect_python_import_multiple() {
        let code = "from horus import CameraNode, LidarNode, ImuNode";
        let nodes = detect_python_nodes(code);
        assert!(nodes.contains("CameraNode"));
        assert!(nodes.contains("LidarNode"));
        assert!(nodes.contains("ImuNode"));
    }

    #[test]
    fn test_detect_python_usage() {
        let code = r#"
import horus
camera = horus.CameraNode()
lidar = horus.LidarNode()
"#;
        let nodes = detect_python_nodes(code);
        assert!(nodes.contains("CameraNode"));
        assert!(nodes.contains("LidarNode"));
    }

    #[test]
    fn test_detect_rust_use() {
        let code = "use horus_library::CameraNode;";
        let nodes = detect_rust_nodes(code);
        assert!(nodes.contains("CameraNode"));
    }

    #[test]
    fn test_detect_rust_nodes_use() {
        let code = "use horus_library::nodes::{CameraNode, LidarNode};";
        let nodes = detect_rust_nodes(code);
        assert!(nodes.contains("CameraNode"));
        assert!(nodes.contains("LidarNode"));
    }

    #[test]
    fn test_detect_rust_horus_library_nodes() {
        // This is the pattern used in snakesim
        let code = "use horus::library::nodes::{KeyboardInputNode, JoystickInputNode};";
        let nodes = detect_rust_nodes(code);
        assert!(
            nodes.contains("KeyboardInputNode"),
            "Should detect KeyboardInputNode"
        );
        assert!(
            nodes.contains("JoystickInputNode"),
            "Should detect JoystickInputNode"
        );

        let features = nodes_to_features(&nodes);
        assert!(
            features.contains(&"crossterm".to_string()),
            "KeyboardInputNode requires crossterm"
        );
        assert!(
            features.contains(&"gilrs".to_string()),
            "JoystickInputNode requires gilrs"
        );
    }

    #[test]
    fn test_detect_rust_usage() {
        let code = r#"
let camera = CameraNode::new();
let lidar = LidarNode::new();
"#;
        let nodes = detect_rust_nodes(code);
        assert!(nodes.contains("CameraNode"));
        assert!(nodes.contains("LidarNode"));
    }

    #[test]
    fn test_nodes_to_features() {
        let mut nodes = HashSet::new();
        nodes.insert("CameraNode".to_string());
        nodes.insert("LidarNode".to_string());
        nodes.insert("DifferentialDriveNode".to_string()); // No features

        let features = nodes_to_features(&nodes);
        assert!(features.contains(&"opencv-backend".to_string()));
        assert!(features.contains(&"rplidar".to_string()));
    }

    #[test]
    fn test_hardware_independent_nodes_no_features() {
        let mut nodes = HashSet::new();
        nodes.insert("DifferentialDriveNode".to_string());
        nodes.insert("EmergencyStopNode".to_string());

        let features = nodes_to_features(&nodes);
        assert!(features.is_empty());
    }
}
