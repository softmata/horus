use axum::{
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    response::IntoResponse,
};
use futures::{sink::SinkExt, stream::StreamExt};

pub async fn websocket_handler(ws: WebSocketUpgrade) -> impl IntoResponse {
    ws.on_upgrade(handle_websocket)
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
                            "pid": n.process_id,
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

        // Convert graph data
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
                    "pid": n.pid,
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
