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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_tick_range_single() {
        assert_eq!(parse_tick_range("42"), Some((42, 42)));
    }

    #[test]
    fn parse_tick_range_range() {
        assert_eq!(parse_tick_range("10-20"), Some((10, 20)));
    }

    #[test]
    fn parse_tick_range_with_spaces() {
        assert_eq!(parse_tick_range(" 5 - 15 "), Some((5, 15)));
    }

    #[test]
    fn parse_tick_range_invalid() {
        assert_eq!(parse_tick_range("abc"), None);
        assert_eq!(parse_tick_range("1-2-3"), None);
    }

    #[test]
    fn is_anomaly_node_error() {
        let event = BlackBoxEvent::NodeError {
            name: "test".into(),
            error: "fail".into(),
        };
        assert!(is_anomaly(&event));
    }

    #[test]
    fn is_anomaly_node_tick_is_not() {
        let event = BlackBoxEvent::NodeTick {
            name: "test".into(),
            duration_us: 1000,
            success: true,
        };
        assert!(!is_anomaly(&event));
    }

    #[test]
    fn event_node_name_extraction() {
        let event = BlackBoxEvent::NodeAdded {
            name: "my_node".into(),
            order: 1,
        };
        assert_eq!(event_node_name(&event), "my_node");
    }

    #[test]
    fn event_node_name_empty_for_scheduler_stop() {
        let event = BlackBoxEvent::SchedulerStop {
            reason: "done".into(),
            total_ticks: 0,
        };
        assert_eq!(event_node_name(&event), "");
    }

    #[test]
    fn filter_records_by_limit() {
        let records: Vec<BlackBoxRecord> = (0..10)
            .map(|i| BlackBoxRecord {
                tick: i,
                timestamp_us: i * 1000,
                event: BlackBoxEvent::NodeTick {
                    name: "n".into(),
                    duration_us: 100,
                    success: true,
                },
            })
            .collect();
        let params = BlackboxQuery {
            limit: Some(3),
            ..Default::default()
        };
        let result = filter_records(records, &params, false);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn filter_records_anomalies_only() {
        let records = vec![
            BlackBoxRecord {
                tick: 1,
                timestamp_us: 1000,
                event: BlackBoxEvent::NodeTick {
                    name: "n".into(),
                    duration_us: 100,
                    success: true,
                },
            },
            BlackBoxRecord {
                tick: 2,
                timestamp_us: 2000,
                event: BlackBoxEvent::NodeError {
                    name: "n".into(),
                    error: "oops".into(),
                },
            },
        ];
        let params = BlackboxQuery::default();
        let result = filter_records(records, &params, true);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].tick, 2);
    }

    #[test]
    fn filter_records_by_node_name() {
        let records = vec![
            BlackBoxRecord {
                tick: 1,
                timestamp_us: 1000,
                event: BlackBoxEvent::NodeTick {
                    name: "lidar_node".into(),
                    duration_us: 100,
                    success: true,
                },
            },
            BlackBoxRecord {
                tick: 2,
                timestamp_us: 2000,
                event: BlackBoxEvent::NodeTick {
                    name: "camera_node".into(),
                    duration_us: 200,
                    success: true,
                },
            },
            BlackBoxRecord {
                tick: 3,
                timestamp_us: 3000,
                event: BlackBoxEvent::NodeError {
                    name: "lidar_node".into(),
                    error: "timeout".into(),
                },
            },
        ];
        let params = BlackboxQuery {
            node: Some("lidar".into()),
            ..Default::default()
        };
        let result = filter_records(records, &params, false);
        assert_eq!(result.len(), 2);
        assert!(result.iter().all(|r| event_node_name(&r.event).contains("lidar")));
    }

    #[test]
    fn filter_records_by_event_type() {
        let records = vec![
            BlackBoxRecord {
                tick: 1,
                timestamp_us: 1000,
                event: BlackBoxEvent::NodeTick {
                    name: "n".into(),
                    duration_us: 100,
                    success: true,
                },
            },
            BlackBoxRecord {
                tick: 2,
                timestamp_us: 2000,
                event: BlackBoxEvent::NodeError {
                    name: "n".into(),
                    error: "fail".into(),
                },
            },
        ];
        let params = BlackboxQuery {
            event: Some("NodeError".into()),
            ..Default::default()
        };
        let result = filter_records(records, &params, false);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].tick, 2);
    }

    #[test]
    fn filter_records_by_tick_range() {
        let records: Vec<BlackBoxRecord> = (0..20)
            .map(|i| BlackBoxRecord {
                tick: i,
                timestamp_us: i * 1000,
                event: BlackBoxEvent::NodeTick {
                    name: "n".into(),
                    duration_us: 100,
                    success: true,
                },
            })
            .collect();
        let params = BlackboxQuery {
            tick: Some("5-10".into()),
            ..Default::default()
        };
        let result = filter_records(records, &params, false);
        assert_eq!(result.len(), 6); // ticks 5,6,7,8,9,10
        assert_eq!(result[0].tick, 5);
        assert_eq!(result[5].tick, 10);
    }

    #[test]
    fn filter_records_combined_filters() {
        let records = vec![
            BlackBoxRecord {
                tick: 5,
                timestamp_us: 5000,
                event: BlackBoxEvent::NodeTick {
                    name: "sensor".into(),
                    duration_us: 100,
                    success: true,
                },
            },
            BlackBoxRecord {
                tick: 10,
                timestamp_us: 10000,
                event: BlackBoxEvent::NodeError {
                    name: "sensor".into(),
                    error: "fail".into(),
                },
            },
            BlackBoxRecord {
                tick: 15,
                timestamp_us: 15000,
                event: BlackBoxEvent::NodeError {
                    name: "motor".into(),
                    error: "stall".into(),
                },
            },
        ];
        let params = BlackboxQuery {
            node: Some("sensor".into()),
            event: Some("NodeError".into()),
            ..Default::default()
        };
        let result = filter_records(records, &params, false);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].tick, 10);
    }

    #[test]
    fn event_type_name_all_variants() {
        assert_eq!(
            event_type_name(&BlackBoxEvent::SchedulerStart {
                name: "s".into(),
                node_count: 1,
                config: Default::default(),
            }),
            "SchedulerStart"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::SchedulerStop {
                reason: "done".into(),
                total_ticks: 0,
            }),
            "SchedulerStop"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::NodeTick {
                name: "n".into(),
                duration_us: 0,
                success: true,
            }),
            "NodeTick"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::NodeError {
                name: "n".into(),
                error: "e".into(),
            }),
            "NodeError"
        );
    }

    #[test]
    fn is_anomaly_deadline_miss() {
        let event = BlackBoxEvent::DeadlineMiss {
            name: "n".into(),
            deadline_us: 1000,
            actual_us: 2000,
        };
        assert!(is_anomaly(&event));
    }

    #[test]
    fn is_anomaly_scheduler_start_is_not() {
        let event = BlackBoxEvent::SchedulerStart {
            name: "s".into(),
            node_count: 1,
            config: Default::default(),
        };
        assert!(!is_anomaly(&event));
    }

    #[test]
    fn blackbox_query_default_has_no_filters() {
        let q = BlackboxQuery::default();
        assert!(q.node.is_none());
        assert!(q.event.is_none());
        assert!(q.tick.is_none());
        assert!(q.limit.is_none());
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
