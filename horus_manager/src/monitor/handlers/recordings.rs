use axum::{extract::Path, http::StatusCode, response::IntoResponse, Json};

use horus_core::scheduling::Recording;

use crate::monitor::is_safe_path_component;

/// Response structure for recording list
#[derive(serde::Serialize)]
struct RecordingListItem {
    session_name: String,
    started_at: String,
    ended_at: Option<String>,
    total_ticks: u64,
    node_count: usize,
    size_bytes: u64,
}

/// List all recordings
pub async fn recordings_list_handler() -> impl IntoResponse {
    let recordings_dir = crate::paths::recordings_dir()
        .unwrap_or_else(|_| std::path::PathBuf::from(".horus/recordings"));

    if !recordings_dir.exists() {
        return (
            StatusCode::OK,
            Json(serde_json::json!({
                "recordings": [],
                "count": 0
            })),
        );
    }

    let mut recordings = Vec::new();

    if let Ok(entries) = std::fs::read_dir(&recordings_dir) {
        for entry in entries.flatten() {
            if entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                let session_name = entry.file_name().to_string_lossy().to_string();
                let session_dir = entry.path();

                let scheduler_file = session_dir
                    .read_dir()
                    .ok()
                    .and_then(|mut d| {
                        d.find(|e| {
                            e.as_ref()
                                .map(|e| e.file_name().to_string_lossy().starts_with("scheduler@"))
                                .unwrap_or(false)
                        })
                    })
                    .and_then(|r| r.ok());

                let node_count = session_dir
                    .read_dir()
                    .map(|d| {
                        d.filter(|e| {
                            e.as_ref()
                                .map(|e| {
                                    let name = e.file_name();
                                    let name_str = name.to_string_lossy();
                                    name_str.ends_with(".horus")
                                        && !name_str.starts_with("scheduler@")
                                })
                                .unwrap_or(false)
                        })
                        .count()
                    })
                    .unwrap_or(0);

                let size_bytes = session_dir
                    .read_dir()
                    .map(|d| {
                        d.filter_map(|e| e.ok())
                            .filter_map(|e| e.metadata().ok())
                            .map(|m| m.len())
                            .sum()
                    })
                    .unwrap_or(0u64);

                let (started_at, ended_at, total_ticks) = if let Some(sf) = scheduler_file {
                    use horus_core::scheduling::SchedulerRecording;
                    if let Ok(sr) = SchedulerRecording::load(&sf.path()) {
                        let started = chrono::DateTime::from_timestamp_micros(sr.started_at as i64)
                            .map(|dt| dt.format("%Y-%m-%d %H:%M:%S").to_string())
                            .unwrap_or_else(|| "Unknown".to_string());
                        let ended = sr.ended_at.and_then(|e| {
                            chrono::DateTime::from_timestamp_micros(e as i64)
                                .map(|dt| dt.format("%Y-%m-%d %H:%M:%S").to_string())
                        });
                        (started, ended, sr.total_ticks)
                    } else {
                        ("Unknown".to_string(), None, 0)
                    }
                } else {
                    let started = entry
                        .metadata()
                        .ok()
                        .and_then(|m| m.created().ok())
                        .map(|t| {
                            chrono::DateTime::<chrono::Utc>::from(t)
                                .format("%Y-%m-%d %H:%M:%S")
                                .to_string()
                        })
                        .unwrap_or_else(|| "Unknown".to_string());
                    (started, None, 0)
                };

                recordings.push(RecordingListItem {
                    session_name,
                    started_at,
                    ended_at,
                    total_ticks,
                    node_count,
                    size_bytes,
                });
            }
        }
    }

    recordings.sort_by(|a, b| b.started_at.cmp(&a.started_at));

    let count = recordings.len();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "recordings": recordings,
            "count": count
        })),
    )
}

/// Get detailed info about a specific recording
pub async fn recordings_info_handler(Path(session): Path<String>) -> impl IntoResponse {
    if !is_safe_path_component(&session) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid session name"
            })),
        );
    }

    let recordings_dir = crate::paths::recordings_dir()
        .unwrap_or_else(|_| std::path::PathBuf::from(".horus/recordings"))
        .join(&session);

    if !recordings_dir.exists() {
        return (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Recording session '{}' not found", session)
            })),
        );
    }

    let scheduler_file = recordings_dir
        .read_dir()
        .ok()
        .and_then(|mut d| {
            d.find(|e| {
                e.as_ref()
                    .map(|e| e.file_name().to_string_lossy().starts_with("scheduler@"))
                    .unwrap_or(false)
            })
        })
        .and_then(|r| r.ok());

    let mut node_recordings: Vec<serde_json::Value> = recordings_dir
        .read_dir()
        .map(|d| {
            d.filter_map(|e| e.ok())
                .filter(|e| {
                    let name = e.file_name();
                    let name_str = name.to_string_lossy();
                    name_str.ends_with(".horus") && !name_str.starts_with("scheduler@")
                })
                .map(|e| {
                    let size = e.metadata().map(|m| m.len()).unwrap_or(0);
                    let filename = e.file_name().to_string_lossy().to_string();
                    let node_name = filename.split('@').next().unwrap_or(&filename).to_string();
                    serde_json::json!({
                        "filename": filename,
                        "node_name": node_name,
                        "size_bytes": size,
                        "size_human": format_size(size)
                    })
                })
                .collect()
        })
        .unwrap_or_default();

    node_recordings.sort_by(|a, b| a["node_name"].as_str().cmp(&b["node_name"].as_str()));

    let scheduler_info = if let Some(sf) = scheduler_file {
        use horus_core::scheduling::SchedulerRecording;
        if let Ok(sr) = SchedulerRecording::load(&sf.path()) {
            let started = chrono::DateTime::from_timestamp_micros(sr.started_at as i64)
                .map(|dt| dt.format("%Y-%m-%d %H:%M:%S UTC").to_string())
                .unwrap_or_else(|| "Unknown".to_string());
            let ended = sr.ended_at.and_then(|e| {
                chrono::DateTime::from_timestamp_micros(e as i64)
                    .map(|dt| dt.format("%Y-%m-%d %H:%M:%S UTC").to_string())
            });
            let duration_secs = sr
                .ended_at
                .map(|e| (e - sr.started_at) / 1_000_000)
                .unwrap_or(0);

            serde_json::json!({
                "scheduler_id": sr.scheduler_id,
                "session_name": sr.session_name,
                "started_at": started,
                "ended_at": ended,
                "duration_secs": duration_secs,
                "total_ticks": sr.total_ticks,
                "execution_order_ticks": sr.execution_order.len(),
                "config": sr.config
            })
        } else {
            serde_json::json!(null)
        }
    } else {
        serde_json::json!(null)
    };

    let total_size: u64 = recordings_dir
        .read_dir()
        .map(|d| {
            d.filter_map(|e| e.ok())
                .filter_map(|e| e.metadata().ok())
                .map(|m| m.len())
                .sum()
        })
        .unwrap_or(0);

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "session_name": session,
            "path": recordings_dir.to_string_lossy(),
            "scheduler": scheduler_info,
            "node_recordings": node_recordings,
            "total_size_bytes": total_size,
            "total_size_human": format_size(total_size)
        })),
    )
}

/// Delete a recording session
pub async fn recordings_delete_handler(Path(session): Path<String>) -> impl IntoResponse {
    if !is_safe_path_component(&session) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid session name"
            })),
        );
    }

    let recordings_dir = crate::paths::recordings_dir()
        .unwrap_or_else(|_| std::path::PathBuf::from(".horus/recordings"))
        .join(&session);

    if !recordings_dir.exists() {
        return (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Recording session '{}' not found", session)
            })),
        );
    }

    match std::fs::remove_dir_all(&recordings_dir) {
        Ok(_) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Deleted recording session '{}'", session)
            })),
        ),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": format!("Failed to delete recording: {}", e)
            })),
        ),
    }
}

/// Format bytes to human-readable size
fn format_size(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;

    if bytes >= GB {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.2} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.2} KB", bytes as f64 / KB as f64)
    } else {
        format!("{} B", bytes)
    }
}
