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

    loop {
        interval.tick().await;

        // Gather all data in parallel
        let (nodes_result, topics_result, graph_result) = tokio::join!(
            tokio::task::spawn_blocking(|| {
                crate::discovery::discover_nodes()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|n| {
                        serde_json::json!({
                            "name": n.name,
                            // PID intentionally omitted — system PIDs must not
                            // be disclosed to clients (security hardening).
                            "status": n.status,
                            "health": n.health.as_str(),
                            "health_color": n.health.color(),
                            "cpu": format!("{:.1}%", n.cpu_usage),
                            "memory": format!("{} MB", n.memory_usage / 1024 / 1024),
                            "scheduler_name": n.scheduler_name,
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
                            "size": format!("{} KB", t.size_bytes / 1024),
                            "active": t.active,
                            "processes": t.accessing_processes.len(),
                        })
                    })
                    .collect::<Vec<_>>()
            }),
            tokio::task::spawn_blocking(|| {
                let (nodes, edges) = crate::graph::discover_graph_data();
                (nodes, edges)
            })
        );

        // Unwrap results
        let nodes = nodes_result.unwrap_or_default();
        let topics = topics_result.unwrap_or_default();
        let (graph_nodes, graph_edges) = graph_result.unwrap_or_default();

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
                }
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
