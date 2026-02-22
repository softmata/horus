use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::IntoResponse,
    Json,
};
use std::sync::Arc;

use crate::monitor::AppState;

/// List all parameters
pub async fn params_list_handler(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let params_map = state.params.get_all();

    let params_list: Vec<_> = params_map
        .iter()
        .map(|(key, value)| {
            let type_str = if value.is_number() {
                "number"
            } else if value.is_string() {
                "string"
            } else if value.is_boolean() {
                "boolean"
            } else if value.is_array() {
                "array"
            } else if value.is_object() {
                "object"
            } else {
                "null"
            };

            serde_json::json!({
                "key": key,
                "value": value,
                "type": type_str,
                "version": state.params.get_version(key)
            })
        })
        .collect();

    (
        StatusCode::OK,
        serde_json::json!({
            "success": true,
            "params": params_list,
            "count": params_list.len()
        })
        .to_string(),
    )
}

/// Get a specific parameter
pub async fn params_get_handler(
    State(state): State<Arc<AppState>>,
    Path(key): Path<String>,
) -> impl IntoResponse {
    match state.params.get::<serde_json::Value>(&key) {
        Some(value) => (
            StatusCode::OK,
            serde_json::json!({
                "success": true,
                "key": key,
                "value": value,
                "version": state.params.get_version(&key)
            })
            .to_string(),
        ),
        None => (
            StatusCode::NOT_FOUND,
            serde_json::json!({
                "success": false,
                "error": format!("Parameter '{}' not found", key)
            })
            .to_string(),
        ),
    }
}

#[derive(serde::Deserialize)]
pub struct SetParamRequest {
    pub value: serde_json::Value,
    pub version: Option<u64>,
}

/// Set a parameter
pub async fn params_set_handler(
    State(state): State<Arc<AppState>>,
    Path(key): Path<String>,
    Json(req): Json<SetParamRequest>,
) -> impl IntoResponse {
    let result = if let Some(expected_version) = req.version {
        state
            .params
            .set_with_version(&key, req.value.clone(), expected_version)
    } else {
        state.params.set(&key, req.value.clone())
    };

    match result {
        Ok(_) => {
            let _ = state.params.save_to_disk();

            let new_version = state.params.get_version(&key);

            (
                StatusCode::OK,
                serde_json::json!({
                    "success": true,
                    "message": format!("Parameter '{}' updated", key),
                    "key": key,
                    "value": req.value,
                    "version": new_version
                })
                .to_string(),
            )
        }
        Err(e) => {
            let status_code = if e.to_string().contains("Version mismatch") {
                StatusCode::CONFLICT
            } else {
                StatusCode::INTERNAL_SERVER_ERROR
            };

            (
                status_code,
                serde_json::json!({
                    "success": false,
                    "error": e.to_string()
                })
                .to_string(),
            )
        }
    }
}

/// Delete a parameter
pub async fn params_delete_handler(
    State(state): State<Arc<AppState>>,
    Path(key): Path<String>,
) -> impl IntoResponse {
    match state.params.remove(&key) {
        Some(old_value) => {
            let _ = state.params.save_to_disk();

            (
                StatusCode::OK,
                serde_json::json!({
                    "success": true,
                    "message": format!("Parameter '{}' deleted", key),
                    "key": key,
                    "old_value": old_value
                })
                .to_string(),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            serde_json::json!({
                "success": false,
                "error": format!("Parameter '{}' not found", key)
            })
            .to_string(),
        ),
    }
}

/// Export all parameters
pub async fn params_export_handler(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let params = state.params.get_all();

    match serde_yaml::to_string(&params) {
        Ok(yaml) => (
            StatusCode::OK,
            serde_json::json!({
                "success": true,
                "format": "yaml",
                "data": yaml
            })
            .to_string(),
        ),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            serde_json::json!({
                "success": false,
                "error": e.to_string()
            })
            .to_string(),
        ),
    }
}

#[derive(serde::Deserialize)]
pub struct ImportParamsRequest {
    pub data: String,
    pub format: String,
}

/// Import parameters
pub async fn params_import_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<ImportParamsRequest>,
) -> impl IntoResponse {
    let import_result: Result<
        std::collections::BTreeMap<String, serde_json::Value>,
        Box<dyn std::error::Error>,
    > =
        match req.format.as_str() {
            "yaml" => serde_yaml::from_str(&req.data)
                .map_err(|e| Box::new(e) as Box<dyn std::error::Error>),
            "json" => serde_json::from_str(&req.data)
                .map_err(|e| Box::new(e) as Box<dyn std::error::Error>),
            _ => {
                return (
                    StatusCode::BAD_REQUEST,
                    serde_json::json!({
                        "success": false,
                        "error": "Invalid format. Use 'yaml' or 'json'"
                    })
                    .to_string(),
                );
            }
        };

    match import_result {
        Ok(params_map) => {
            let mut count = 0;
            for (key, value) in params_map {
                if state.params.set(&key, value).is_ok() {
                    count += 1;
                }
            }

            let _ = state.params.save_to_disk();

            (
                StatusCode::OK,
                serde_json::json!({
                    "success": true,
                    "message": format!("Imported {} parameters", count),
                    "count": count
                })
                .to_string(),
            )
        }
        Err(e) => (
            StatusCode::BAD_REQUEST,
            serde_json::json!({
                "success": false,
                "error": format!("failed to parse {}: {}", req.format, e)
            })
            .to_string(),
        ),
    }
}
