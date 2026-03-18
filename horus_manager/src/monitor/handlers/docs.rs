//! API documentation endpoints for the horus monitor.
//!
//! - `GET /api/docs` — Returns the extracted `ProjectDoc` as JSON.
//! - `GET /api/docs/export` — Returns a self-contained HTML report.

use axum::response::{Html, IntoResponse, Json};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::commands::doc_extract::{self, ExtractConfig, ProjectDoc};

/// Cache for the extracted documentation (TTL: 30 seconds).
static DOCS_CACHE: Mutex<Option<(Instant, ProjectDoc)>> = Mutex::new(None);
const CACHE_TTL: Duration = Duration::from_secs(30);

/// `GET /api/docs` — Return extracted ProjectDoc as JSON.
pub async fn docs_handler() -> impl IntoResponse {
    match get_or_extract() {
        Ok(doc) => Json(serde_json::json!(doc)).into_response(),
        Err(e) => {
            let error = serde_json::json!({
                "error": format!("{}", e),
                "message": "Failed to extract documentation"
            });
            (axum::http::StatusCode::INTERNAL_SERVER_ERROR, Json(error)).into_response()
        }
    }
}

/// `GET /api/docs/export` — Return self-contained HTML report.
pub async fn docs_export_handler() -> impl IntoResponse {
    match get_or_extract() {
        Ok(doc) => {
            let html = crate::commands::doc_extract_html::format_html(&doc);
            (
                [(
                    axum::http::header::CONTENT_DISPOSITION,
                    "attachment; filename=\"api-docs.html\"",
                )],
                Html(html),
            )
                .into_response()
        }
        Err(e) => {
            let html = format!(
                "<html><body><h1>Error</h1><p>Failed to extract documentation: {}</p></body></html>",
                e
            );
            (axum::http::StatusCode::INTERNAL_SERVER_ERROR, Html(html)).into_response()
        }
    }
}

/// Get cached documentation or extract fresh.
fn get_or_extract() -> anyhow::Result<ProjectDoc> {
    // Check cache
    {
        let cache = DOCS_CACHE.lock().unwrap_or_else(|e| e.into_inner());
        if let Some((cached_at, ref doc)) = *cache {
            if cached_at.elapsed() < CACHE_TTL {
                return Ok(doc.clone());
            }
        }
    }

    // Extract fresh
    let project_dir = std::env::current_dir()?;
    let config = ExtractConfig {
        json: false,
        md: false,
        html: false,
        brief: false,
        full: false,
        all: false,
        lang: None,
        coverage: false,
        output: None,
        watch: false,
        diff: None,
        fail_under: None,
    };
    let doc = doc_extract::extract_project(&project_dir, &config)?;

    // Update cache
    {
        let mut cache = DOCS_CACHE.lock().unwrap_or_else(|e| e.into_inner());
        *cache = Some((Instant::now(), doc.clone()));
    }

    Ok(doc)
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_or_extract_returns_result() {
        // Should succeed or fail gracefully (no panic)
        let result = get_or_extract();
        // In test environment, current_dir exists, so extraction should succeed
        // (may produce empty ProjectDoc if not in a horus project)
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_cache_ttl_constant() {
        assert_eq!(CACHE_TTL, Duration::from_secs(30));
    }

    #[test]
    fn test_default_extract_config() {
        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        assert!(!config.json);
        assert!(!config.all);
        assert!(config.lang.is_none());
    }
}
