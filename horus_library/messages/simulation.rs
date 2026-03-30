//! Simulator control service message types.
//!
//! These types define the request/response payloads for simulator control
//! services (spawn, despawn, teleport, pause, raycast, etc.). They are
//! **simulator-agnostic** — any simulator plugin (sim3d, mujoco, isaac)
//! implements handlers for the same message types.
//!
//! Horus nodes import these types to call simulator services without
//! depending on any specific simulator.
//!
//! # Service Names
//!
//! Well-known service names are in `horus_library::topic_convention::service`.

use serde::{Deserialize, Serialize};

// ── Spawn ──────────────────────────────────────────────────────────────────

/// Request to spawn a model (URDF/SDF/primitive) in the simulation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpawnRequest {
    /// Path to model file (URDF, SDF, MJCF) or primitive name ("box", "sphere", "cylinder").
    pub model: String,
    /// Entity name. Auto-generated if empty.
    pub name: String,
    /// Spawn position [x, y, z] in meters.
    pub position: [f64; 3],
    /// Spawn rotation as quaternion [x, y, z, w].
    pub rotation: [f64; 4],
    /// Scale factor [x, y, z]. Default [1, 1, 1].
    pub scale: [f64; 3],
    /// Whether the spawned entity is static (immovable). Default false.
    pub is_static: bool,
}

impl Default for SpawnRequest {
    fn default() -> Self {
        Self {
            model: String::new(),
            name: String::new(),
            position: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0; 3],
            is_static: false,
        }
    }
}

/// Response after spawning a model.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpawnResponse {
    /// Unique entity ID for subsequent operations (despawn, teleport).
    pub entity_id: u64,
    /// Assigned name (may differ from request if auto-generated).
    pub name: String,
    /// Whether the spawn succeeded.
    pub success: bool,
    /// Error message if spawn failed.
    pub error: String,
}

// ── Despawn ────────────────────────────────────────────────────────────────

/// Request to remove an entity from the simulation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DespawnRequest {
    /// Entity ID returned by SpawnResponse.
    pub entity_id: u64,
}

// ── Teleport ───────────────────────────────────────────────────────────────

/// Request to instantly move an entity to a new pose.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TeleportRequest {
    /// Entity ID to teleport.
    pub entity_id: u64,
    /// New position [x, y, z] in meters.
    pub position: [f64; 3],
    /// New rotation as quaternion [x, y, z, w].
    pub rotation: [f64; 4],
    /// Whether to zero the entity's velocity after teleporting.
    pub reset_velocity: bool,
}

// ── Pause / Resume ─────────────────────────────────────────────────────────

/// Request to pause the simulation (physics stops, sensors stop updating).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PauseRequest;

/// Request to resume a paused simulation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ResumeRequest;

// ── Raycast ────────────────────────────────────────────────────────────────

/// Request a ray query against the simulation world.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RaycastRequest {
    /// Ray origin [x, y, z] in meters.
    pub origin: [f64; 3],
    /// Ray direction [dx, dy, dz] (normalized).
    pub direction: [f64; 3],
    /// Maximum ray distance in meters.
    pub max_distance: f64,
}

/// Result of a raycast query.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RaycastResponse {
    /// Whether the ray hit anything.
    pub hit: bool,
    /// Hit point [x, y, z] in meters (zero if no hit).
    pub point: [f64; 3],
    /// Surface normal at hit point (zero if no hit).
    pub normal: [f64; 3],
    /// Distance from origin to hit point (0.0 if no hit).
    pub distance: f64,
    /// Entity ID of the hit object (None if no hit).
    pub entity_id: Option<u64>,
}

impl Default for RaycastResponse {
    fn default() -> Self {
        Self {
            hit: false,
            point: [0.0; 3],
            normal: [0.0; 3],
            distance: 0.0,
            entity_id: None,
        }
    }
}

// ── State ──────────────────────────────────────────────────────────────────

/// Request the current simulation state.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GetStateRequest;

/// Snapshot of the simulation state.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SimState {
    /// Whether the simulation is paused.
    pub paused: bool,
    /// Simulation elapsed time in nanoseconds.
    pub sim_time_ns: u64,
    /// Number of entities in the world.
    pub entity_count: u32,
    /// Physics timestep in seconds.
    pub physics_dt: f64,
}

// ── Parameter ──────────────────────────────────────────────────────────────

/// Request to modify a simulation parameter at runtime.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SetParamRequest {
    /// Parameter key (e.g., "physics.gravity.y", "physics.dt").
    pub key: String,
    /// New value.
    pub value: f64,
}

// ── Generic Response ───────────────────────────────────────────────────────

/// Generic success/error response for services that don't return data.
///
/// Used by: despawn, teleport, pause, resume, set_param.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SimOk {
    /// Whether the operation succeeded.
    pub success: bool,
    /// Error or status message.
    pub message: String,
}

impl SimOk {
    /// Create a success response.
    pub fn ok() -> Self {
        Self { success: true, message: String::new() }
    }

    /// Create an error response.
    pub fn err(msg: impl Into<String>) -> Self {
        Self { success: false, message: msg.into() }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip<T: Serialize + for<'de> Deserialize<'de> + std::fmt::Debug>(val: &T) -> T {
        let json = serde_json::to_string(val).unwrap();
        serde_json::from_str(&json).unwrap()
    }

    #[test]
    fn spawn_request_roundtrip() {
        let req = SpawnRequest {
            model: "robot.urdf".into(),
            name: "my_robot".into(),
            position: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, 0.707, 0.707],
            scale: [1.0, 1.0, 1.0],
            is_static: false,
        };
        let rt = roundtrip(&req);
        assert_eq!(rt.model, "robot.urdf");
        assert!((rt.position[0] - 1.0).abs() < 1e-10);
        assert!((rt.rotation[2] - 0.707).abs() < 1e-10);
    }

    #[test]
    fn spawn_request_default() {
        let req = SpawnRequest::default();
        assert!(req.model.is_empty());
        assert!(req.name.is_empty());
        assert_eq!(req.rotation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(req.scale, [1.0, 1.0, 1.0]);
        assert!(!req.is_static);
    }

    #[test]
    fn spawn_response_roundtrip() {
        let resp = SpawnResponse {
            entity_id: 42,
            name: "box_0".into(),
            success: true,
            error: String::new(),
        };
        let rt = roundtrip(&resp);
        assert_eq!(rt.entity_id, 42);
        assert!(rt.success);
    }

    #[test]
    fn despawn_request_roundtrip() {
        let req = DespawnRequest { entity_id: 99 };
        let rt = roundtrip(&req);
        assert_eq!(rt.entity_id, 99);
    }

    #[test]
    fn teleport_request_roundtrip() {
        let req = TeleportRequest {
            entity_id: 5,
            position: [10.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            reset_velocity: true,
        };
        let rt = roundtrip(&req);
        assert_eq!(rt.entity_id, 5);
        assert!(rt.reset_velocity);
    }

    #[test]
    fn pause_resume_roundtrip() {
        let p = roundtrip(&PauseRequest);
        let r = roundtrip(&ResumeRequest);
        // Empty structs — just verify they serialize/deserialize without error
        let _ = (p, r);
    }

    #[test]
    fn raycast_request_roundtrip() {
        let req = RaycastRequest {
            origin: [0.0, 1.0, 0.0],
            direction: [1.0, 0.0, 0.0],
            max_distance: 50.0,
        };
        let rt = roundtrip(&req);
        assert!((rt.max_distance - 50.0).abs() < 1e-10);
    }

    #[test]
    fn raycast_response_hit() {
        let resp = RaycastResponse {
            hit: true,
            point: [5.0, 1.0, 0.0],
            normal: [-1.0, 0.0, 0.0],
            distance: 5.0,
            entity_id: Some(42),
        };
        let rt = roundtrip(&resp);
        assert!(rt.hit);
        assert_eq!(rt.entity_id, Some(42));
        assert!((rt.distance - 5.0).abs() < 1e-10);
    }

    #[test]
    fn raycast_response_miss() {
        let resp = RaycastResponse::default();
        let rt = roundtrip(&resp);
        assert!(!rt.hit);
        assert_eq!(rt.entity_id, None);
        assert_eq!(rt.distance, 0.0);
    }

    #[test]
    fn sim_state_roundtrip() {
        let state = SimState {
            paused: true,
            sim_time_ns: 1_000_000_000,
            entity_count: 15,
            physics_dt: 0.004166,
        };
        let rt = roundtrip(&state);
        assert!(rt.paused);
        assert_eq!(rt.sim_time_ns, 1_000_000_000);
        assert_eq!(rt.entity_count, 15);
    }

    #[test]
    fn set_param_request_roundtrip() {
        let req = SetParamRequest {
            key: "physics.gravity.y".into(),
            value: -9.81,
        };
        let rt = roundtrip(&req);
        assert_eq!(rt.key, "physics.gravity.y");
        assert!((rt.value - (-9.81)).abs() < 1e-10);
    }

    #[test]
    fn sim_ok_constructors() {
        let ok = SimOk::ok();
        assert!(ok.success);
        assert!(ok.message.is_empty());

        let err = SimOk::err("entity not found");
        assert!(!err.success);
        assert_eq!(err.message, "entity not found");

        // Roundtrip both
        let rt_ok = roundtrip(&ok);
        assert!(rt_ok.success);
        let rt_err = roundtrip(&err);
        assert!(!rt_err.success);
    }
}
