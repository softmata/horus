//! Auto-detect HORUS nodes used in Python and Rust code
//!
//! Parses source files to find which built-in nodes are imported/used.
//! Feature resolution is delegated to the driver/registry system.

use std::collections::HashSet;
use std::fs;
use std::path::Path;

/// Node name list used for auto-detection in source files.
///
/// Previously mapped node names to Cargo features, but those hardware features
/// (e.g. `i2c-hardware`, `gpio-hardware`) never existed in any horus crate.
/// Feature resolution now happens through the driver/registry system instead.
const NODE_NAMES: &[&str] = &[
    // Vision nodes
    "CameraNode",
    "DepthCameraNode",
    "ImageProcessorNode",
    // Input devices
    "JoystickInputNode",
    "KeyboardInputNode",
    // Sensors
    "BatteryMonitorNode",
    "ImuNode",
    "GpsNode",
    "LidarNode",
    "ForceTorqueSensorNode",
    "EncoderNode",
    "UltrasonicNode",
    // Motors/Actuators
    "BldcMotorNode",
    "DcMotorNode",
    "StepperMotorNode",
    "ServoControllerNode",
    "DynamixelNode",
    "RoboclawMotorNode",
    // Industrial interfaces
    "CanBusNode",
    "DigitalIONode",
    "I2cBusNode",
    "ModbusNode",
    "SerialNode",
    "SpiBusNode",
    // Hardware-independent nodes
    "CollisionDetectorNode",
    "DifferentialDriveNode",
    "EmergencyStopNode",
    "LocalizationNode",
    "OdometryNode",
    "PathPlannerNode",
    "PidControllerNode",
    "SafetyMonitorNode",
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
            for node_name in NODE_NAMES {
                if line.contains(&format!("horus.{}", node_name)) {
                    nodes.insert(node_name.to_string());
                }
            }
        }

        // Pattern 3: Direct usage like CameraNode()
        for node_name in NODE_NAMES {
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

        // Pattern 1: use horus_robotics::CameraNode;
        // Pattern 2: use horus_library::CameraNode; (legacy)
        if line.starts_with("use horus_robotics::")
            || line.starts_with("use horus_tf::")
            || line.starts_with("use horus_library::")
            || line.starts_with("use horus::library::")
        {
            for node_name in NODE_NAMES {
                if line.contains(node_name) {
                    nodes.insert(node_name.to_string());
                }
            }
        }

        // Pattern 4: Direct usage like CameraNode::new()
        for node_name in NODE_NAMES {
            if line.contains(&format!("{}::", node_name)) {
                nodes.insert(node_name.to_string());
            }
        }
    }

    nodes
}

/// Convert detected nodes to required Cargo features.
///
/// Feature resolution now happens through the driver/registry system,
/// so this always returns an empty list. Kept for API compatibility.
pub fn nodes_to_features(_nodes: &HashSet<String>) -> Vec<String> {
    Vec::new()
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
    fn test_nodes_to_features_returns_empty() {
        let mut nodes = HashSet::new();
        nodes.insert("CameraNode".to_string());
        nodes.insert("LidarNode".to_string());
        nodes.insert("DifferentialDriveNode".to_string());

        let features = nodes_to_features(&nodes);
        assert!(
            features.is_empty(),
            "Feature resolution now happens via driver/registry system"
        );
    }
}
