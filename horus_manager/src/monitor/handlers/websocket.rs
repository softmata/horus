use axum::{
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    response::IntoResponse,
};
use futures::{sink::SinkExt, stream::StreamExt};

/// Simple WebSocket upgrade handler.
///
/// Authentication is enforced by `ws_auth_middleware` (applied in `mod.rs`),
/// which runs before this handler and rejects unauthenticated requests with
/// 401 before the WebSocket connection is upgraded.
pub async fn websocket_handler(ws: WebSocketUpgrade) -> impl IntoResponse {
    ws.on_upgrade(handle_websocket)
}

/// Extract `session_token` from the `Cookie` request header.
/// Exported for use by `ws_auth_middleware` in `mod.rs`.
pub fn extract_cookie_token(headers: &axum::http::HeaderMap) -> Option<&str> {
    let cookie_header = headers.get(axum::http::header::COOKIE)?;
    let cookie_str = cookie_header.to_str().ok()?;
    for cookie in cookie_str.split(';') {
        let cookie = cookie.trim();
        if let Some(value) = cookie.strip_prefix("session_token=") {
            return Some(value);
        }
    }
    None
}

/// Extract token from `Authorization: Bearer <token>` header.
/// Exported for use by `ws_auth_middleware` in `mod.rs`.
pub fn extract_bearer_token(headers: &axum::http::HeaderMap) -> Option<&str> {
    let auth = headers.get(axum::http::header::AUTHORIZATION)?;
    let auth_str = auth.to_str().ok()?;
    auth_str.strip_prefix("Bearer ")
}

async fn handle_websocket(socket: WebSocket) {
    let (mut sender, _receiver) = socket.split();

    let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(
        crate::config::WS_BROADCAST_INTERVAL_MS,
    ));

    // Track the log buffer write index so we only send new entries each tick.
    let mut last_log_idx = horus_core::core::log_buffer::GLOBAL_LOG_BUFFER.write_idx();
    let mut last_error_idx = horus_core::core::log_buffer::GLOBAL_ERROR_BUFFER.write_idx();

    loop {
        interval.tick().await;

        // Capture current indices for this tick's delta queries.
        let current_log_idx = last_log_idx;
        let current_error_idx = last_error_idx;

        // Gather all data in parallel
        let (nodes_result, topics_result, graph_result, logs_result, error_logs_result) = tokio::join!(
            tokio::task::spawn_blocking(|| {
                crate::discovery::discover_nodes()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|n| {
                        serde_json::json!({
                            "name": n.name,
                            "status": n.status,
                            "health": n.health.as_str(),
                            "health_color": n.health.color(),
                            "cpu": format!("{:.1}%", n.cpu_usage),
                            "memory": format!("{} MB", n.memory_usage / 1024 / 1024),
                            "scheduler_name": n.scheduler_name,
                            "tick_count": n.live_tick_count.unwrap_or(n.tick_count),
                            "error_count": n.error_count,
                            "avg_tick_us": n.live_avg_tick_ns.map(|ns| ns / 1000).unwrap_or(0),
                            "max_tick_us": n.live_max_tick_ns.map(|ns| ns / 1000).unwrap_or(0),
                            "budget_misses": n.live_budget_misses.unwrap_or(0),
                            "deadline_misses": n.live_deadline_misses.unwrap_or(0),
                            "rate_hz": n.actual_rate_hz,
                        })
                    })
                    .collect::<Vec<_>>()
            }),
            tokio::task::spawn_blocking(|| {
                crate::discovery::discover_shared_memory()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|t| {
                        serde_json::json!({
                            "name": t.topic_name,
                            "type": t.message_type.as_deref().unwrap_or("unknown"),
                            "size": format!("{} KB", t.size_bytes / 1024),
                            "active": t.active,
                            "processes": t.accessing_processes.len(),
                            "messages_total": t.messages_total,
                            "rate_hz": t.message_rate_hz,
                        })
                    })
                    .collect::<Vec<_>>()
            }),
            tokio::task::spawn_blocking(|| {
                let (nodes, edges) = crate::graph::discover_graph_data();
                (nodes, edges)
            }),
            tokio::task::spawn_blocking(move || { collect_new_logs(current_log_idx) }),
            tokio::task::spawn_blocking(move || { collect_new_error_logs(current_error_idx) })
        );

        // Unwrap results
        let nodes = nodes_result.unwrap_or_default();
        let topics = topics_result.unwrap_or_default();
        let (graph_nodes, graph_edges) = graph_result.unwrap_or_default();
        let (new_logs, new_log_idx) = logs_result.unwrap_or_default();
        let (new_error_logs, new_error_idx) = error_logs_result.unwrap_or_default();
        last_log_idx = new_log_idx;
        last_error_idx = new_error_idx;

        // Convert graph data — PID omitted from graph nodes for the same reason.
        let graph_nodes_json = graph_nodes
            .into_iter()
            .map(|n| {
                serde_json::json!({
                    "id": n.id,
                    "label": n.label,
                    "type": match n.node_type {
                        crate::graph::NodeType::Process => "process",
                        crate::graph::NodeType::Topic => "topic",
                    },
                    // "pid" field removed — PIDs must not be disclosed to clients.
                    "active": n.active,
                })
            })
            .collect::<Vec<_>>();

        let graph_edges_json = graph_edges
            .into_iter()
            .map(|e| {
                serde_json::json!({
                    "from": e.from,
                    "to": e.to,
                    "type": match e.edge_type {
                        crate::graph::EdgeType::Publish => "publish",
                        crate::graph::EdgeType::Subscribe => "subscribe",
                    },
                    "active": e.active,
                })
            })
            .collect::<Vec<_>>();

        // Build update message
        let update = serde_json::json!({
            "type": "update",
            "timestamp": chrono::Utc::now().to_rfc3339(),
            "data": {
                "nodes": nodes,
                "topics": topics,
                "graph": {
                    "nodes": graph_nodes_json,
                    "edges": graph_edges_json
                },
                "logs": new_logs,
                "error_logs": new_error_logs
            }
        });

        // Send to client
        if sender
            .send(Message::Text(update.to_string()))
            .await
            .is_err()
        {
            break; // Client disconnected
        }
    }
}

/// Collect log entries written since `last_idx`.
///
/// Returns `(entries_json, new_write_idx)`.  The caller stores `new_write_idx`
/// so the next call only fetches the delta.  At most 200 entries are returned
/// per tick to avoid oversized WebSocket frames.
fn collect_new_logs(last_idx: u64) -> (Vec<serde_json::Value>, u64) {
    use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;

    let current_idx = GLOBAL_LOG_BUFFER.write_idx();
    if current_idx <= last_idx {
        return (Vec::new(), current_idx);
    }

    // Fetch all entries and take only the ones written after last_idx.
    // The ring buffer is 5000 slots; we cap at 200 per tick to keep frames small.
    let all = GLOBAL_LOG_BUFFER.get_all();
    let new_count = (current_idx - last_idx) as usize;
    let capped = new_count.min(200);

    let entries: Vec<serde_json::Value> = all
        .into_iter()
        .rev()
        .take(capped)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .map(|e| {
            serde_json::json!({
                "timestamp": e.timestamp,
                "tick_number": e.tick_number,
                "node_name": e.node_name,
                "log_type": format!("{:?}", e.log_type),
                "topic": e.topic,
                "message": e.message,
                "tick_us": e.tick_us,
                "ipc_ns": e.ipc_ns,
            })
        })
        .collect();

    (entries, current_idx)
}

/// Collect error log entries written since `last_idx`.
///
/// Same delta-tracking pattern as [`collect_new_logs`] but reads from the
/// persistent error buffer. Error entries have much longer retention.
fn collect_new_error_logs(last_idx: u64) -> (Vec<serde_json::Value>, u64) {
    use horus_core::core::log_buffer::GLOBAL_ERROR_BUFFER;

    let current_idx = GLOBAL_ERROR_BUFFER.write_idx();
    if current_idx <= last_idx {
        return (Vec::new(), current_idx);
    }

    let all = GLOBAL_ERROR_BUFFER.get_all();
    let new_count = (current_idx - last_idx) as usize;
    let capped = new_count.min(100); // Errors are rare; 100/tick is generous

    let entries: Vec<serde_json::Value> = all
        .into_iter()
        .rev()
        .take(capped)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .map(|e| {
            serde_json::json!({
                "timestamp": e.timestamp,
                "tick_number": e.tick_number,
                "node_name": e.node_name,
                "log_type": format!("{:?}", e.log_type),
                "topic": e.topic,
                "message": e.message,
                "tick_us": e.tick_us,
                "ipc_ns": e.ipc_ns,
            })
        })
        .collect();

    (entries, current_idx)
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::http::HeaderMap;

    #[test]
    fn extract_cookie_token_present() {
        let mut headers = HeaderMap::new();
        headers.insert(
            axum::http::header::COOKIE,
            "other=abc; session_token=my_token_value; foo=bar"
                .parse()
                .unwrap(),
        );
        assert_eq!(extract_cookie_token(&headers), Some("my_token_value"));
    }

    #[test]
    fn extract_cookie_token_missing() {
        let headers = HeaderMap::new();
        assert_eq!(extract_cookie_token(&headers), None);
    }

    #[test]
    fn extract_cookie_token_no_session_cookie() {
        let mut headers = HeaderMap::new();
        headers.insert(
            axum::http::header::COOKIE,
            "other=abc; foo=bar".parse().unwrap(),
        );
        assert_eq!(extract_cookie_token(&headers), None);
    }

    #[test]
    fn extract_bearer_token_present() {
        let mut headers = HeaderMap::new();
        headers.insert(
            axum::http::header::AUTHORIZATION,
            "Bearer my_bearer_token".parse().unwrap(),
        );
        assert_eq!(extract_bearer_token(&headers), Some("my_bearer_token"));
    }

    #[test]
    fn extract_bearer_token_missing() {
        let headers = HeaderMap::new();
        assert_eq!(extract_bearer_token(&headers), None);
    }

    #[test]
    fn extract_bearer_token_wrong_scheme() {
        let mut headers = HeaderMap::new();
        headers.insert(
            axum::http::header::AUTHORIZATION,
            "Basic dXNlcjpwYXNz".parse().unwrap(),
        );
        assert_eq!(extract_bearer_token(&headers), None);
    }
}
