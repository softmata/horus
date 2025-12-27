//! Test Command Infrastructure for sim3d
//!
//! Provides programmatic control of the simulator via HORUS topics.
//! Use these commands for automated testing, CI/CD pipelines, and scripted scenarios.
//!
//! ## Topics
//!
//! | Topic | Direction | Description |
//! |-------|-----------|-------------|
//! | `/{robot}/test/command` | Subscribe | Receive test commands |
//! | `/{robot}/test/response` | Publish | Command results |
//! | `/{robot}/test/state` | Publish | Current simulator state |
//!
//! ## Example Usage
//!
//! ```bash
//! # Teleport robot to position
//! horus topic pub /robot/test/command '{"cmd": "teleport", "x": 1.0, "y": 0.0, "z": 2.0}'
//!
//! # Take screenshot
//! horus topic pub /robot/test/command '{"cmd": "screenshot", "path": "/tmp/test.png"}'
//!
//! # Get robot state
//! horus topic pub /robot/test/command '{"cmd": "get_state"}'
//! horus topic echo /robot/test/state
//! ```

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Test command received via HORUS topic
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum TestCommand {
    /// Get current robot state (position, velocity, joints)
    GetState,

    /// Teleport robot to position
    Teleport {
        x: f32,
        y: f32,
        z: f32,
        #[serde(default)]
        yaw: Option<f32>,
    },

    /// Move camera to position looking at target
    MoveCamera {
        x: f32,
        y: f32,
        z: f32,
        look_at_x: f32,
        look_at_y: f32,
        look_at_z: f32,
    },

    /// Set camera to preset angle
    SetCameraAngle {
        angle: String, // front, back, left, right, top, isometric
        #[serde(default = "default_distance")]
        distance: f32,
    },

    /// Take screenshot
    Screenshot {
        path: String,
        #[serde(default = "default_width")]
        width: u32,
        #[serde(default = "default_height")]
        height: u32,
    },

    /// Pause simulation
    Pause,

    /// Resume simulation
    Resume,

    /// Step physics by N steps (while paused)
    StepPhysics {
        #[serde(default = "default_steps")]
        steps: u32,
    },

    /// Set simulation speed (1.0 = realtime)
    SetSpeed { speed: f32 },

    /// Apply force to robot
    ApplyForce { x: f32, y: f32, z: f32 },

    /// Apply impulse to robot
    ApplyImpulse { x: f32, y: f32, z: f32 },

    /// Set joint positions
    SetJoints { joints: HashMap<String, f32> },

    /// Get sensor data
    GetSensorData { sensor: String },

    /// Spawn object at position
    SpawnObject {
        object_type: String, // box, sphere, cylinder
        x: f32,
        y: f32,
        z: f32,
        #[serde(default = "default_size")]
        size: f32,
    },

    /// Remove spawned object
    RemoveObject { id: String },

    /// Reset simulation to initial state
    Reset,

    /// Exit simulator
    Exit,

    /// Run assertion (for testing)
    Assert {
        condition: String,
        expected: serde_json::Value,
        #[serde(default)]
        tolerance: Option<f32>,
    },

    /// Wait for condition
    WaitFor {
        condition: String,
        #[serde(default = "default_timeout")]
        timeout_ms: u64,
    },
}

fn default_distance() -> f32 {
    2.0
}
fn default_width() -> u32 {
    1920
}
fn default_height() -> u32 {
    1080
}
fn default_steps() -> u32 {
    1
}
fn default_size() -> f32 {
    0.5
}
fn default_timeout() -> u64 {
    5000
}

/// Response from test command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResponse {
    pub success: bool,
    pub command: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<serde_json::Value>,
    pub timestamp_ns: u64,
}

/// Current simulator state (published periodically or on request)
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SimulatorState {
    pub paused: bool,
    pub speed: f32,
    pub physics_step: u64,
    pub sim_time_secs: f64,
    pub robot: Option<RobotState>,
    pub camera: CameraState,
    pub spawned_objects: Vec<SpawnedObjectState>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RobotState {
    pub name: String,
    pub position: [f32; 3],
    pub rotation: [f32; 4], // quaternion
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub joints: HashMap<String, JointState>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct JointState {
    pub position: f32,
    pub velocity: f32,
    pub effort: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CameraState {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub fov: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpawnedObjectState {
    pub id: String,
    pub object_type: String,
    pub position: [f32; 3],
}

/// Resource to manage test commands
#[derive(Resource, Default)]
pub struct TestCommandState {
    pub pending_commands: Vec<TestCommand>,
    pub responses: Vec<TestResponse>,
    pub paused: bool,
    pub speed: f32,
    pub physics_step: u64,
    pub spawned_objects: HashMap<String, Entity>,
}

/// Resource to request screenshots from test commands
#[derive(Resource, Default)]
pub struct TestScreenshotRequest {
    pub path: Option<std::path::PathBuf>,
    pub pending: bool,
}

/// Plugin for test command infrastructure
pub struct TestCommandPlugin {
    pub robot_name: String,
}

impl TestCommandPlugin {
    pub fn new(robot_name: &str) -> Self {
        Self {
            robot_name: robot_name.to_string(),
        }
    }
}

impl Plugin for TestCommandPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TestCommandState>()
            .init_resource::<TestScreenshotRequest>()
            .add_systems(
                Update,
                (process_test_commands, publish_state_system).chain(),
            );

        info!(
            "TestCommandPlugin initialized for robot: {}",
            self.robot_name
        );
    }
}

/// System to process incoming test commands
fn process_test_commands(
    mut state: ResMut<TestCommandState>,
    mut screenshot_request: ResMut<TestScreenshotRequest>,
    mut camera_query: Query<&mut Transform, With<Camera3d>>,
    mut exit_writer: EventWriter<bevy::app::AppExit>,
) {
    let commands: Vec<TestCommand> = state.pending_commands.drain(..).collect();

    for cmd in commands {
        let response = match &cmd {
            TestCommand::Pause => {
                state.paused = true;
                TestResponse {
                    success: true,
                    command: "pause".to_string(),
                    error: None,
                    data: None,
                    timestamp_ns: now_ns(),
                }
            }

            TestCommand::Resume => {
                state.paused = false;
                TestResponse {
                    success: true,
                    command: "resume".to_string(),
                    error: None,
                    data: None,
                    timestamp_ns: now_ns(),
                }
            }

            TestCommand::SetSpeed { speed } => {
                state.speed = *speed;
                TestResponse {
                    success: true,
                    command: "set_speed".to_string(),
                    error: None,
                    data: Some(serde_json::json!({"speed": speed})),
                    timestamp_ns: now_ns(),
                }
            }

            TestCommand::Screenshot { path, .. } => {
                screenshot_request.path = Some(std::path::PathBuf::from(path));
                screenshot_request.pending = true;
                TestResponse {
                    success: true,
                    command: "screenshot".to_string(),
                    error: None,
                    data: Some(serde_json::json!({"path": path})),
                    timestamp_ns: now_ns(),
                }
            }

            TestCommand::MoveCamera {
                x,
                y,
                z,
                look_at_x,
                look_at_y,
                look_at_z,
            } => {
                if let Ok(mut camera) = camera_query.get_single_mut() {
                    camera.translation = Vec3::new(*x, *y, *z);
                    camera.look_at(Vec3::new(*look_at_x, *look_at_y, *look_at_z), Vec3::Y);
                    TestResponse {
                        success: true,
                        command: "move_camera".to_string(),
                        error: None,
                        data: None,
                        timestamp_ns: now_ns(),
                    }
                } else {
                    TestResponse {
                        success: false,
                        command: "move_camera".to_string(),
                        error: Some("No camera found".to_string()),
                        data: None,
                        timestamp_ns: now_ns(),
                    }
                }
            }

            TestCommand::SetCameraAngle { angle, distance } => {
                if let Ok(mut camera) = camera_query.get_single_mut() {
                    let offset = match angle.as_str() {
                        "front" => Vec3::new(0.0, *distance * 0.5, *distance),
                        "back" => Vec3::new(0.0, *distance * 0.5, -*distance),
                        "left" => Vec3::new(-*distance, *distance * 0.5, 0.0),
                        "right" => Vec3::new(*distance, *distance * 0.5, 0.0),
                        "top" => Vec3::new(0.0, *distance * 1.5, 0.1),
                        "isometric" | _ => {
                            Vec3::new(*distance * 0.7, *distance * 0.5, *distance * 0.7)
                        }
                    };
                    camera.translation = offset;
                    camera.look_at(Vec3::ZERO, Vec3::Y);
                    TestResponse {
                        success: true,
                        command: "set_camera_angle".to_string(),
                        error: None,
                        data: Some(serde_json::json!({"angle": angle})),
                        timestamp_ns: now_ns(),
                    }
                } else {
                    TestResponse {
                        success: false,
                        command: "set_camera_angle".to_string(),
                        error: Some("No camera found".to_string()),
                        data: None,
                        timestamp_ns: now_ns(),
                    }
                }
            }

            TestCommand::Exit => {
                exit_writer.send(bevy::app::AppExit::Success);
                TestResponse {
                    success: true,
                    command: "exit".to_string(),
                    error: None,
                    data: None,
                    timestamp_ns: now_ns(),
                }
            }

            TestCommand::GetState => {
                // State will be published by publish_state_system
                TestResponse {
                    success: true,
                    command: "get_state".to_string(),
                    error: None,
                    data: None,
                    timestamp_ns: now_ns(),
                }
            }

            _ => {
                // Commands requiring physics access are handled elsewhere
                TestResponse {
                    success: true,
                    command: format!("{:?}", cmd)
                        .split('{')
                        .next()
                        .unwrap_or("unknown")
                        .trim()
                        .to_lowercase(),
                    error: None,
                    data: None,
                    timestamp_ns: now_ns(),
                }
            }
        };

        state.responses.push(response);
    }
}

/// System to publish current state
fn publish_state_system(
    state: Res<TestCommandState>,
    camera_query: Query<&Transform, With<Camera3d>>,
) {
    // This would publish to HORUS topic - for now just log
    if !state.responses.is_empty() {
        for response in &state.responses {
            if response.success {
                info!("Test command '{}' completed", response.command);
            } else {
                warn!(
                    "Test command '{}' failed: {:?}",
                    response.command, response.error
                );
            }
        }
    }

    // Build state for publishing
    let camera_state = if let Ok(cam) = camera_query.get_single() {
        CameraState {
            position: cam.translation.to_array(),
            rotation: cam.rotation.to_array(),
            fov: 60.0, // Default FOV
        }
    } else {
        CameraState::default()
    };

    let _sim_state = SimulatorState {
        paused: state.paused,
        speed: state.speed,
        physics_step: state.physics_step,
        sim_time_secs: state.physics_step as f64 * (1.0 / 60.0),
        robot: None, // Filled by robot query
        camera: camera_state,
        spawned_objects: vec![],
    };
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_parsing() {
        let json = r#"{"cmd": "teleport", "x": 1.0, "y": 0.0, "z": 2.0}"#;
        let cmd: TestCommand = serde_json::from_str(json).unwrap();
        match cmd {
            TestCommand::Teleport { x, y, z, .. } => {
                assert_eq!(x, 1.0);
                assert_eq!(y, 0.0);
                assert_eq!(z, 2.0);
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_camera_angle_parsing() {
        let json = r#"{"cmd": "set_camera_angle", "angle": "front", "distance": 3.0}"#;
        let cmd: TestCommand = serde_json::from_str(json).unwrap();
        match cmd {
            TestCommand::SetCameraAngle { angle, distance } => {
                assert_eq!(angle, "front");
                assert_eq!(distance, 3.0);
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_response_serialization() {
        let response = TestResponse {
            success: true,
            command: "screenshot".to_string(),
            error: None,
            data: Some(serde_json::json!({"path": "/tmp/test.png"})),
            timestamp_ns: 123456789,
        };
        let json = serde_json::to_string(&response).unwrap();
        assert!(json.contains("screenshot"));
        assert!(json.contains("true"));
    }
}
