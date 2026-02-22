use axum::{
    extract::State,
    http::StatusCode,
    response::{Html, IntoResponse, Response},
    Json,
};
use std::sync::Arc;

use crate::monitor::{html, AppState};

pub async fn index_handler(
    State(state): State<Arc<AppState>>,
    req: axum::http::Request<axum::body::Body>,
) -> Response {
    // If authentication is disabled, go straight to monitor
    let is_authenticated = if state.auth_disabled {
        true // Skip authentication entirely
    } else {
        // Check if user is authenticated by looking for session cookie
        if let Some(cookie_header) = req.headers().get(axum::http::header::COOKIE) {
            if let Ok(cookie_str) = cookie_header.to_str() {
                // Extract session token from cookies
                let token = cookie_str.split(';').find_map(|cookie| {
                    let cookie = cookie.trim();
                    cookie.strip_prefix("session_token=")
                });

                // Validate session if token exists
                if let Some(token) = token {
                    state.auth_service.validate_session(token)
                } else {
                    false
                }
            } else {
                false
            }
        } else {
            false
        }
    };

    if is_authenticated {
        // User is logged in - show monitor
        Html(html::generate_html(state.port)).into_response()
    } else {
        // User is not logged in - show login page
        Html(html::generate_login_html()).into_response()
    }
}

pub async fn status_handler(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    use horus_core::core::HealthStatus;

    // Get all nodes and their health
    let nodes = crate::discovery::discover_nodes().unwrap_or_default();
    let nodes_count = nodes.len();

    let topics_count = crate::discovery::discover_shared_memory()
        .map(|t| t.len())
        .unwrap_or(0);

    // Calculate system-wide health by aggregating all node health
    let (system_status, system_health, health_color) = if nodes_count == 0 {
        ("Idle".to_string(), "No nodes running".to_string(), "gray")
    } else {
        // Count nodes by health status
        let mut healthy = 0;
        let mut warning = 0;
        let mut error = 0;
        let mut critical = 0;
        let mut unknown = 0;

        for node in &nodes {
            match node.health {
                HealthStatus::Healthy => healthy += 1,
                HealthStatus::Warning => warning += 1,
                HealthStatus::Error => error += 1,
                HealthStatus::Critical => critical += 1,
                HealthStatus::Unknown => unknown += 1,
            }
        }

        // System health is determined by worst node health
        let (status, color) = if critical > 0 {
            ("Critical", "red")
        } else if error > 0 {
            ("Degraded", "orange")
        } else if warning > 0 {
            ("Warning", "yellow")
        } else if unknown > 0 && healthy == 0 {
            ("Unknown", "gray")
        } else {
            ("Healthy", "green")
        };

        // Build detailed health summary
        let mut details = Vec::new();
        if critical > 0 {
            details.push(format!("{} critical", critical));
        }
        if error > 0 {
            details.push(format!("{} error", error));
        }
        if warning > 0 {
            details.push(format!("{} warning", warning));
        }
        if healthy > 0 {
            details.push(format!("{} healthy", healthy));
        }
        if unknown > 0 {
            details.push(format!("{} unknown", unknown));
        }

        let health_summary = if details.is_empty() {
            format!("{} nodes", nodes_count)
        } else {
            details.join(", ")
        };

        (status.to_string(), health_summary, color)
    };

    // Build workspace info
    let workspace_info = if let Some(ref ws_path) = state.current_workspace {
        let ws_name = ws_path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown");
        serde_json::json!({
            "name": ws_name,
            "path": ws_path.display().to_string(),
            "detected": true
        })
    } else {
        serde_json::json!({
            "detected": false
        })
    };

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "status": system_status,
            "health": system_health,
            "health_color": health_color,
            "version": "0.1.9",
            "nodes": nodes_count,
            "topics": topics_count,
            "workspace": workspace_info
        })),
    )
        .into_response()
}

pub async fn nodes_handler() -> impl IntoResponse {
    let nodes = crate::discovery::discover_nodes()
        .unwrap_or_default()
        .into_iter()
        .map(|n| {
            serde_json::json!({
                "name": n.name,
                "pid": n.process_id,
                "status": n.status,
                "health": n.health.as_str(),
                "health_color": n.health.color(),
                "cpu": format!("{:.1}%", n.cpu_usage),
                "memory": format!("{} MB", n.memory_usage / 1024 / 1024),
                "tick_count": n.tick_count,
                "error_count": n.error_count,
                "tick_rate": n.actual_rate_hz,
                "scheduler_name": n.scheduler_name,
            })
        })
        .collect::<Vec<_>>();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "nodes": nodes
        })),
    )
        .into_response()
}

pub async fn topics_handler() -> impl IntoResponse {
    let topics = crate::discovery::discover_shared_memory()
        .unwrap_or_default()
        .into_iter()
        .map(|t| {
            let display_name = t
                .topic_name
                .strip_prefix("horus_")
                .unwrap_or(&t.topic_name)
                .to_string();

            serde_json::json!({
                "name": display_name,
                "size": format!("{} KB", t.size_bytes / 1024),
                "active": t.active,
                "processes": t.accessing_processes.len(),
            })
        })
        .collect::<Vec<_>>();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "topics": topics
        })),
    )
        .into_response()
}

pub async fn graph_handler() -> impl IntoResponse {
    let (nodes, edges) = crate::graph::discover_graph_data();

    let graph_nodes = nodes
        .into_iter()
        .map(|n| {
            serde_json::json!({
                "id": n.id,
                "label": n.label,
                "type": match n.node_type {
                    crate::graph::NodeType::Process => "process",
                    crate::graph::NodeType::Topic => "topic",
                },
                "pid": n.pid,
                "active": n.active,
            })
        })
        .collect::<Vec<_>>();

    let graph_edges = edges
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

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "nodes": graph_nodes,
            "edges": graph_edges
        })),
    )
        .into_response()
}

pub async fn network_handler() -> impl IntoResponse {
    let summary = crate::discovery::get_network_summary();

    let node_statuses = summary
        .node_statuses
        .iter()
        .map(|s| {
            serde_json::json!({
                "node_name": s.node_name,
                "transport_type": s.transport_type,
                "local_endpoint": s.local_endpoint,
                "remote_endpoints": s.remote_endpoints,
                "network_topics_pub": s.network_topics_pub,
                "network_topics_sub": s.network_topics_sub,
                "bytes_sent": s.bytes_sent,
                "bytes_received": s.bytes_received,
                "packets_sent": s.packets_sent,
                "packets_received": s.packets_received,
                "timestamp_secs": s.timestamp_secs,
            })
        })
        .collect::<Vec<_>>();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "total_nodes": summary.total_nodes,
            "total_bytes_sent": summary.total_bytes_sent,
            "total_bytes_received": summary.total_bytes_received,
            "total_packets_sent": summary.total_packets_sent,
            "total_packets_received": summary.total_packets_received,
            "transport_breakdown": summary.transport_breakdown,
            "unique_endpoints": summary.unique_endpoints,
            "node_statuses": node_statuses,
        })),
    )
        .into_response()
}
