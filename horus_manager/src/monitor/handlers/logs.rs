use axum::{
    extract::Path,
    http::StatusCode,
    response::IntoResponse,
    Json,
};

pub async fn logs_all_handler() -> impl IntoResponse {
    use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;

    let logs = GLOBAL_LOG_BUFFER.get_all();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "logs": logs
        })),
    )
        .into_response()
}

pub async fn logs_node_handler(Path(node_name): Path<String>) -> impl IntoResponse {
    use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;

    let logs = GLOBAL_LOG_BUFFER.for_node(&node_name);

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "node": node_name,
            "logs": logs
        })),
    )
        .into_response()
}

pub async fn logs_topic_handler(Path(topic_name): Path<String>) -> impl IntoResponse {
    use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;

    // Topic names use dot notation - just strip horus_ prefix
    let original_topic = topic_name
        .strip_prefix("horus_")
        .unwrap_or(&topic_name)
        .to_string();

    let logs = GLOBAL_LOG_BUFFER.for_topic(&original_topic);

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "topic": topic_name,
            "logs": logs
        })),
    )
        .into_response()
}
