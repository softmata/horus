use axum::{extract::Path, http::StatusCode, response::IntoResponse, Json};
use horus_core::scheduling::{
    BreakpointCondition, DebugSessionState, NodeRecording, Recording, ReplayDebugger,
    WatchExpression, WatchType,
};
use serde::Deserialize;
use std::sync::Mutex;

use crate::monitor::is_safe_path_component;

lazy_static::lazy_static! {
    /// Global debug sessions storage
    static ref DEBUG_SESSIONS: Mutex<std::collections::HashMap<String, DebugSessionData>> =
        Mutex::new(std::collections::HashMap::new());
}

/// Debug session data with debugger and state
struct DebugSessionData {
    debugger: ReplayDebugger,
    state: DebugSessionState,
}

/// Request body for creating a debug session
#[derive(Debug, Deserialize)]
pub struct CreateDebugSessionRequest {
    recording_session: String,
    recording_file: String,
    session_name: Option<String>,
}

/// Request body for adding a breakpoint
#[derive(Debug, Deserialize)]
pub struct AddBreakpointRequest {
    breakpoint_type: String,
    tick: Option<u64>,
    topic: Option<String>,
    pattern: Option<Vec<u8>>,
    name: Option<String>,
}

/// Request body for adding a watch expression
#[derive(Debug, Deserialize)]
pub struct AddWatchRequest {
    id: String,
    name: String,
    topic: String,
    watch_type: String,
    byte_offset: Option<usize>,
    byte_length: Option<usize>,
}

/// Request body for seeking
#[derive(Debug, Deserialize)]
pub struct SeekRequest {
    tick: u64,
}

/// List all debug sessions
pub async fn debug_sessions_list_handler() -> impl IntoResponse {
    let sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    let session_list: Vec<serde_json::Value> = sessions
        .iter()
        .map(|(id, data)| {
            serde_json::json!({
                "id": id,
                "session_name": data.state.session_name,
                "recording_path": data.state.recording_path.to_string_lossy(),
                "current_tick": data.state.current_tick,
                "breakpoint_count": data.state.breakpoints.len(),
                "watch_count": data.state.watches.len(),
                "state": format!("{:?}", data.debugger.state()),
                "created_at": data.state.created_at,
                "updated_at": data.state.updated_at,
            })
        })
        .collect();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "sessions": session_list,
            "count": session_list.len()
        })),
    )
}

/// Create a new debug session
pub async fn debug_session_create_handler(
    Json(req): Json<CreateDebugSessionRequest>,
) -> impl IntoResponse {
    if !is_safe_path_component(&req.recording_session)
        || !is_safe_path_component(&req.recording_file)
    {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid recording session or file name"
            })),
        );
    }

    let recordings_dir = crate::paths::recordings_dir()
        .unwrap_or_else(|_| std::path::PathBuf::from(".horus/recordings"))
        .join(&req.recording_session);

    let recording_path = recordings_dir.join(&req.recording_file);

    if !recording_path.exists() {
        return (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Recording file not found: {}", recording_path.display())
            })),
        );
    }

    let recording = match NodeRecording::load(&recording_path) {
        Ok(r) => r,
        Err(e) => {
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({
                    "error": format!("Failed to load recording: {}", e)
                })),
            );
        }
    };

    let replayer = horus_core::scheduling::NodeReplayer::from_recording(recording);
    let debugger = ReplayDebugger::new(replayer);

    let session_name = req.session_name.unwrap_or_else(|| {
        format!(
            "debug_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs()
        )
    });

    let state = DebugSessionState::new(recording_path.clone(), &session_name);

    let session_id = format!(
        "{:x}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
    );

    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");
    sessions.insert(session_id.clone(), DebugSessionData { debugger, state });

    (
        StatusCode::CREATED,
        Json(serde_json::json!({
            "id": session_id,
            "session_name": session_name,
            "recording_path": recording_path.to_string_lossy(),
            "message": "Debug session created"
        })),
    )
}

/// Get debug session details
pub async fn debug_session_get_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get(&session_id) {
        Some(data) => {
            let recording = data.debugger.recording();
            let current_snapshot = data.debugger.current_snapshot();

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "id": session_id,
                    "session_name": data.state.session_name,
                    "recording_path": data.state.recording_path.to_string_lossy(),
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "total_ticks": recording.snapshots.len(),
                    "first_tick": recording.first_tick,
                    "last_tick": recording.last_tick,
                    "node_name": recording.node_name,
                    "node_id": recording.node_id,
                    "breakpoints": data.debugger.breakpoints().iter().map(|bp| {
                        serde_json::json!({
                            "id": bp.id,
                            "condition": format!("{:?}", bp.condition),
                            "enabled": bp.enabled,
                            "hit_count": bp.hit_count,
                            "name": bp.name
                        })
                    }).collect::<Vec<_>>(),
                    "watches": data.debugger.watches().iter().map(|w| {
                        serde_json::json!({
                            "id": w.id,
                            "name": w.name,
                            "topic": w.topic,
                            "watch_type": format!("{:?}", w.watch_type),
                            "byte_offset": w.byte_offset,
                            "byte_length": w.byte_length
                        })
                    }).collect::<Vec<_>>(),
                    "current_snapshot": current_snapshot.map(|s| {
                        serde_json::json!({
                            "tick": s.tick,
                            "timestamp_us": s.timestamp_us,
                            "duration_ns": s.duration_ns,
                            "input_topics": s.inputs.keys().collect::<Vec<_>>(),
                            "output_topics": s.outputs.keys().collect::<Vec<_>>(),
                            "has_state": s.state.is_some()
                        })
                    }),
                    "recent_events": data.debugger.recent_events(10).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Delete a debug session
pub async fn debug_session_delete_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.remove(&session_id) {
        Some(_) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Debug session '{}' deleted", session_id)
            })),
        ),
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Add a breakpoint to a debug session
pub async fn debug_add_breakpoint_handler(
    Path(session_id): Path<String>,
    Json(req): Json<AddBreakpointRequest>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let condition = match req.breakpoint_type.as_str() {
                "at_tick" => {
                    let tick = req.tick.unwrap_or(0);
                    BreakpointCondition::AtTick(tick)
                }
                "topic_has_data" => {
                    let topic = req.topic.unwrap_or_default();
                    BreakpointCondition::TopicHasData(topic)
                }
                "output_matches" => {
                    let topic = req.topic.unwrap_or_default();
                    let pattern = req.pattern.unwrap_or_default();
                    BreakpointCondition::OutputMatches { topic, pattern }
                }
                "on_error" => BreakpointCondition::OnError,
                "after_ticks" => {
                    let n = req.tick.unwrap_or(1);
                    BreakpointCondition::AfterTicks(n)
                }
                _ => {
                    return (
                        StatusCode::BAD_REQUEST,
                        Json(serde_json::json!({
                            "error": format!("Unknown breakpoint type: {}", req.breakpoint_type)
                        })),
                    );
                }
            };

            let bp_id = if let Some(name) = req.name {
                data.debugger.add_named_breakpoint(&name, condition)
            } else {
                data.debugger.add_breakpoint(condition)
            };

            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::CREATED,
                Json(serde_json::json!({
                    "success": true,
                    "breakpoint_id": bp_id,
                    "message": "Breakpoint added"
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Remove a breakpoint from a debug session
pub async fn debug_remove_breakpoint_handler(
    Path((session_id, bp_id)): Path<(String, u32)>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            if data.debugger.remove_breakpoint(bp_id) {
                data.state.update_from_debugger(&data.debugger);
                (
                    StatusCode::OK,
                    Json(serde_json::json!({
                        "success": true,
                        "message": format!("Breakpoint {} removed", bp_id)
                    })),
                )
            } else {
                (
                    StatusCode::NOT_FOUND,
                    Json(serde_json::json!({
                        "error": format!("Breakpoint {} not found", bp_id)
                    })),
                )
            }
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Add a watch expression to a debug session
pub async fn debug_add_watch_handler(
    Path(session_id): Path<String>,
    Json(req): Json<AddWatchRequest>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let watch_type = match req.watch_type.as_str() {
                "input" => WatchType::Input,
                "output" => WatchType::Output,
                _ => {
                    return (
                        StatusCode::BAD_REQUEST,
                        Json(serde_json::json!({
                            "error": format!("Unknown watch type: {}", req.watch_type)
                        })),
                    );
                }
            };

            let mut watch = WatchExpression::new(&req.id, &req.name, &req.topic, watch_type);

            if let (Some(offset), Some(length)) = (req.byte_offset, req.byte_length) {
                watch = watch.with_range(offset, length);
            }

            data.debugger.add_watch(watch);
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::CREATED,
                Json(serde_json::json!({
                    "success": true,
                    "watch_id": req.id,
                    "message": "Watch expression added"
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Remove a watch expression from a debug session
pub async fn debug_remove_watch_handler(
    Path((session_id, watch_id)): Path<(String, String)>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            if data.debugger.remove_watch(&watch_id) {
                data.state.update_from_debugger(&data.debugger);
                (
                    StatusCode::OK,
                    Json(serde_json::json!({
                        "success": true,
                        "message": format!("Watch '{}' removed", watch_id)
                    })),
                )
            } else {
                (
                    StatusCode::NOT_FOUND,
                    Json(serde_json::json!({
                        "error": format!("Watch '{}' not found", watch_id)
                    })),
                )
            }
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Step forward one tick
pub async fn debug_step_forward_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let success = data.debugger.step_forward();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": success,
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "recent_events": data.debugger.recent_events(5).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Step backward one tick
pub async fn debug_step_backward_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let success = data.debugger.step_backward();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": success,
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "recent_events": data.debugger.recent_events(5).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Continue execution until breakpoint
pub async fn debug_continue_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let _event = data.debugger.continue_execution();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "recent_events": data.debugger.recent_events(10).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Pause execution
pub async fn debug_pause_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            data.debugger.pause();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": true,
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Seek to a specific tick
pub async fn debug_seek_handler(
    Path(session_id): Path<String>,
    Json(req): Json<SeekRequest>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let success = data.debugger.seek(req.tick);
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": success,
                    "current_tick": data.debugger.current_tick(),
                    "state": format!("{:?}", data.debugger.state())
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Reset to the beginning
pub async fn debug_reset_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            data.debugger.reset();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": true,
                    "current_tick": data.debugger.current_tick(),
                    "state": format!("{:?}", data.debugger.state())
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Get current snapshot data
pub async fn debug_snapshot_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get(&session_id) {
        Some(data) => match data.debugger.current_snapshot() {
            Some(snapshot) => {
                let inputs: serde_json::Map<String, serde_json::Value> = snapshot
                    .inputs
                    .iter()
                    .map(|(k, v)| {
                        (
                            k.clone(),
                            serde_json::json!({
                                "size": v.len(),
                                "hex": format!("{:02x?}", &v[..v.len().min(64)]),
                                "truncated": v.len() > 64
                            }),
                        )
                    })
                    .collect();

                let outputs: serde_json::Map<String, serde_json::Value> = snapshot
                    .outputs
                    .iter()
                    .map(|(k, v)| {
                        (
                            k.clone(),
                            serde_json::json!({
                                "size": v.len(),
                                "hex": format!("{:02x?}", &v[..v.len().min(64)]),
                                "truncated": v.len() > 64
                            }),
                        )
                    })
                    .collect();

                (
                    StatusCode::OK,
                    Json(serde_json::json!({
                        "tick": snapshot.tick,
                        "timestamp_us": snapshot.timestamp_us,
                        "duration_ns": snapshot.duration_ns,
                        "duration_human": format!("{:.3} ms", snapshot.duration_ns as f64 / 1_000_000.0),
                        "inputs": inputs,
                        "outputs": outputs,
                        "has_state": snapshot.state.is_some(),
                        "state_size": snapshot.state.as_ref().map(|s| s.len()).unwrap_or(0)
                    })),
                )
            }
            None => (
                StatusCode::OK,
                Json(serde_json::json!({
                    "snapshot": null,
                    "message": "No snapshot at current position"
                })),
            ),
        },
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Get current watch values
pub async fn debug_watches_values_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().expect("debug sessions lock poisoned");

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let values = data.debugger.evaluate_watches();

            let values_json: Vec<serde_json::Value> = values
                .iter()
                .map(|v| {
                    serde_json::json!({
                        "expression_id": v.expression_id,
                        "tick": v.tick,
                        "display_value": v.display_value,
                        "raw_bytes_hex": format!("{:02x?}", &v.raw_bytes[..v.raw_bytes.len().min(32)]),
                        "raw_bytes_size": v.raw_bytes.len()
                    })
                })
                .collect();

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "values": values_json,
                    "count": values_json.len(),
                    "current_tick": data.debugger.current_tick()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}
