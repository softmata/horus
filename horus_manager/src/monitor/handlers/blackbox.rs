//! HTTP API handlers for the BlackBox flight recorder.
//!
//! - `GET  /api/blackbox`           — list events with optional filters
//! - `GET  /api/blackbox/anomalies` — shorthand for anomalies-only
//! - `DELETE /api/blackbox`         — clear all blackbox data

use axum::{extract::Query, http::StatusCode, response::IntoResponse, Json};
use horus_core::scheduling::{BlackBoxEvent, BlackBoxRecord};
use std::io::BufRead;

/// Query parameters for `GET /api/blackbox`.
#[derive(serde::Deserialize, Default)]
pub struct BlackboxQuery {
    pub node: Option<String>,
    pub event: Option<String>,
    pub tick: Option<String>,
    pub limit: Option<usize>,
}

/// `GET /api/blackbox?node=X&event=Y&tick=A-B&limit=N`
pub async fn blackbox_list_handler(Query(params): Query<BlackboxQuery>) -> impl IntoResponse {
    match load_records() {
        Ok(records) => {
            let filtered = filter_records(records, &params, false);
            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "events": filtered,
                    "count": filtered.len()
                })),
            )
        }
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({ "error": e.to_string() })),
        ),
    }
}

/// `GET /api/blackbox/anomalies`
pub async fn blackbox_anomalies_handler() -> impl IntoResponse {
    match load_records() {
        Ok(records) => {
            let params = BlackboxQuery::default();
            let filtered = filter_records(records, &params, true);
            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "anomalies": filtered,
                    "count": filtered.len()
                })),
            )
        }
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({ "error": e.to_string() })),
        ),
    }
}

/// `DELETE /api/blackbox`
pub async fn blackbox_clear_handler() -> impl IntoResponse {
    let dir = match crate::paths::blackbox_dir() {
        Ok(d) => d,
        Err(e) => {
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({ "error": e.to_string() })),
            );
        }
    };

    let wal_path = dir.join("blackbox.wal");
    let json_path = dir.join("blackbox.json");
    let mut removed = 0u32;

    if wal_path.is_file() {
        if let Err(e) = std::fs::remove_file(&wal_path) {
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({ "error": format!("failed to remove WAL: {}", e) })),
            );
        }
        removed += 1;
    }
    if json_path.is_file() {
        if let Err(e) = std::fs::remove_file(&json_path) {
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({ "error": format!("failed to remove snapshot: {}", e) })),
            );
        }
        removed += 1;
    }

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "cleared": true,
            "files_removed": removed
        })),
    )
}

// ── Internal helpers ────────────────────────────────────────────────────────

fn load_records() -> Result<Vec<BlackBoxRecord>, String> {
    let dir = crate::paths::blackbox_dir().map_err(|e| e.to_string())?;
    let wal_path = dir.join("blackbox.wal");
    let json_path = dir.join("blackbox.json");

    if wal_path.is_file() {
        let file = std::fs::File::open(&wal_path).map_err(|e| e.to_string())?;
        let reader = std::io::BufReader::new(file);
        let mut records = Vec::new();
        for line in reader.lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => continue,
            };
            let trimmed = line.trim();
            if trimmed.is_empty() {
                continue;
            }
            if let Ok(record) = serde_json::from_str::<BlackBoxRecord>(trimmed) {
                records.push(record);
            }
        }
        return Ok(records);
    }

    if json_path.is_file() {
        let content = std::fs::read_to_string(&json_path).map_err(|e| e.to_string())?;
        let records: Vec<BlackBoxRecord> =
            serde_json::from_str(&content).map_err(|e| e.to_string())?;
        return Ok(records);
    }

    Ok(Vec::new())
}

fn filter_records(
    mut records: Vec<BlackBoxRecord>,
    params: &BlackboxQuery,
    anomalies_only: bool,
) -> Vec<BlackBoxRecord> {
    if anomalies_only {
        records.retain(|r| is_anomaly(&r.event));
    }

    if let Some(ref node) = params.node {
        records.retain(|r| {
            let name = event_node_name(&r.event);
            name.to_lowercase().contains(&node.to_lowercase())
        });
    }

    if let Some(ref ev) = params.event {
        records.retain(|r| {
            let type_name = event_type_name(&r.event);
            type_name.eq_ignore_ascii_case(ev)
        });
    }

    if let Some(ref tick_str) = params.tick {
        if let Some((start, end)) = parse_tick_range(tick_str) {
            records.retain(|r| r.tick >= start && r.tick <= end);
        }
    }

    if let Some(limit) = params.limit {
        if records.len() > limit {
            records = records.split_off(records.len() - limit);
        }
    }

    records
}

fn parse_tick_range(s: &str) -> Option<(u64, u64)> {
    let parts: Vec<&str> = s.split('-').collect();
    match parts.len() {
        1 => {
            let t = parts[0].trim().parse().ok()?;
            Some((t, t))
        }
        2 => {
            let a = parts[0].trim().parse().ok()?;
            let b = parts[1].trim().parse().ok()?;
            Some((a, b))
        }
        _ => None,
    }
}

fn is_anomaly(event: &BlackBoxEvent) -> bool {
    matches!(
        event,
        BlackBoxEvent::NodeError { .. }
            | BlackBoxEvent::DeadlineMiss { .. }
            | BlackBoxEvent::WCETViolation { .. }
            | BlackBoxEvent::EmergencyStop { .. }
            | BlackBoxEvent::CircuitBreakerChange { .. }
    )
}

fn event_node_name(event: &BlackBoxEvent) -> &str {
    match event {
        BlackBoxEvent::SchedulerStart { name, .. } => name,
        BlackBoxEvent::NodeAdded { name, .. } => name,
        BlackBoxEvent::NodeTick { name, .. } => name,
        BlackBoxEvent::NodeError { name, .. } => name,
        BlackBoxEvent::DeadlineMiss { name, .. } => name,
        BlackBoxEvent::WCETViolation { name, .. } => name,
        BlackBoxEvent::CircuitBreakerChange { name, .. } => name,
        _ => "",
    }
}

fn event_type_name(event: &BlackBoxEvent) -> &'static str {
    match event {
        BlackBoxEvent::SchedulerStart { .. } => "SchedulerStart",
        BlackBoxEvent::SchedulerStop { .. } => "SchedulerStop",
        BlackBoxEvent::NodeAdded { .. } => "NodeAdded",
        BlackBoxEvent::NodeTick { .. } => "NodeTick",
        BlackBoxEvent::NodeError { .. } => "NodeError",
        BlackBoxEvent::DeadlineMiss { .. } => "DeadlineMiss",
        BlackBoxEvent::WCETViolation { .. } => "WCETViolation",
        BlackBoxEvent::CircuitBreakerChange { .. } => "CircuitBreakerChange",
        BlackBoxEvent::LearningComplete { .. } => "LearningComplete",
        BlackBoxEvent::EmergencyStop { .. } => "EmergencyStop",
        BlackBoxEvent::Custom { .. } => "Custom",
    }
}
