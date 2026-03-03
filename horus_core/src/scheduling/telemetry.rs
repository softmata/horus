//! Telemetry endpoint for remote monitoring
//!
//! Provides metrics export to various backends:
//! - Local file (JSON)
//! - HTTP endpoint
//! - UDP broadcast

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Write;
use std::net::UdpSocket;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Telemetry metric types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MetricValue {
    Counter(u64),
    Gauge(f64),
    Histogram(Vec<f64>),
    Text(String),
}

/// A single metric with metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metric {
    pub name: String,
    pub value: MetricValue,
    pub labels: HashMap<String, String>,
    pub timestamp_secs: u64,
}

/// Telemetry snapshot containing all metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetrySnapshot {
    pub timestamp_secs: u64,
    pub scheduler_name: String,
    pub uptime_secs: f64,
    pub metrics: Vec<Metric>,
}

/// Telemetry endpoint configuration
#[derive(Debug, Clone)]
pub enum TelemetryEndpoint {
    /// Write to local JSON file
    LocalFile(PathBuf),
    /// Send to UDP address (e.g., "127.0.0.1:9999")
    Udp(String),
    /// HTTP POST endpoint (e.g., "http://localhost:8080/metrics")
    Http(String),
    /// Stdout (for debugging)
    Stdout,
    /// Disabled
    Disabled,
}

impl TelemetryEndpoint {
    /// Parse endpoint from string
    pub fn from_string(s: &str) -> Self {
        if s.is_empty() || s == "disabled" {
            TelemetryEndpoint::Disabled
        } else if s == "stdout" || s == "local" {
            TelemetryEndpoint::Stdout
        } else if s.starts_with("file://") || s.starts_with("/") {
            TelemetryEndpoint::LocalFile(PathBuf::from(s.trim_start_matches("file://")))
        } else if s.starts_with("udp://") {
            TelemetryEndpoint::Udp(s.trim_start_matches("udp://").to_string())
        } else if s.starts_with("http://") || s.starts_with("https://") {
            TelemetryEndpoint::Http(s.to_string())
        } else {
            // Assume local file
            TelemetryEndpoint::LocalFile(PathBuf::from(s))
        }
    }
}

/// Telemetry collector and exporter
pub(crate) struct TelemetryManager {
    /// Endpoint for export
    endpoint: TelemetryEndpoint,
    /// Export interval
    interval: Duration,
    /// Last export time
    last_export: Instant,
    /// Accumulated metrics
    metrics: HashMap<String, Metric>,
    /// Scheduler name
    scheduler_name: String,
    /// Start time
    start_time: Instant,
    /// UDP socket (cached)
    udp_socket: Option<UdpSocket>,
    /// Whether telemetry is enabled
    enabled: bool,
    /// Bounded sender to the background HTTP export thread.
    ///
    /// `Some` only when the endpoint is `Http`.  The RT scheduler thread calls
    /// `try_send()` (non-blocking); if the channel is full the snapshot is
    /// silently dropped rather than stalling the scheduler.
    http_tx: Option<mpsc::SyncSender<TelemetrySnapshot>>,
    /// Background thread that performs blocking HTTP I/O.
    ///
    /// Dropped (and joined) when `TelemetryManager` is dropped.
    http_thread: Option<std::thread::JoinHandle<()>>,
}

impl TelemetryManager {
    /// Create a new telemetry manager.
    ///
    /// For `Http` endpoints, spawns a dedicated background thread that
    /// performs blocking network I/O.  The scheduler thread calls `export()`
    /// which posts a snapshot to a bounded channel; the background thread
    /// reads and POSTs at its own pace without blocking the RT loop.
    pub fn new(endpoint: TelemetryEndpoint, interval_ms: u64) -> Self {
        let enabled = !matches!(endpoint, TelemetryEndpoint::Disabled);

        let udp_socket = if let TelemetryEndpoint::Udp(_) = &endpoint {
            UdpSocket::bind("0.0.0.0:0").ok()
        } else {
            None
        };

        // For HTTP endpoints, spin up a background export thread so that
        // TcpStream::connect() never runs on the RT scheduler thread.
        // Channel capacity 4: enough to absorb a brief burst while the
        // background thread is busy with a slow POST, without growing unboundedly.
        let (http_tx, http_thread) = if let TelemetryEndpoint::Http(url) = &endpoint {
            let (tx, rx) = mpsc::sync_channel::<TelemetrySnapshot>(4);
            let url = url.clone();
            let handle = std::thread::Builder::new()
                .name("horus-telemetry-http".to_string())
                .spawn(move || {
                    // Run until the sender is dropped (scheduler shuts down).
                    while let Ok(snapshot) = rx.recv() {
                        if let Err(e) = http_post_blocking(&url, &snapshot) {
                            log::warn!("[TELEMETRY] HTTP export failed: {}", e);
                        }
                    }
                })
                .ok();
            (Some(tx), handle)
        } else {
            (None, None)
        };

        Self {
            endpoint,
            interval: Duration::from_millis(interval_ms),
            last_export: Instant::now(),
            metrics: HashMap::new(),
            scheduler_name: "horus".to_string(),
            start_time: Instant::now(),
            udp_socket,
            enabled,
            http_tx,
            http_thread,
        }
    }

    /// Set scheduler name
    pub fn set_scheduler_name(&mut self, name: &str) {
        self.scheduler_name = name.to_string();
    }

    /// Record a counter metric (monotonically increasing)
    pub fn counter(&mut self, name: &str, value: u64) {
        self.record(name, MetricValue::Counter(value), HashMap::new());
    }

    /// Record a gauge metric (can go up or down)
    pub fn gauge(&mut self, name: &str, value: f64) {
        self.record(name, MetricValue::Gauge(value), HashMap::new());
    }

    /// Record a counter with labels
    pub fn counter_with_labels(&mut self, name: &str, value: u64, labels: HashMap<String, String>) {
        self.record(name, MetricValue::Counter(value), labels);
    }

    /// Record a gauge with labels
    pub fn gauge_with_labels(&mut self, name: &str, value: f64, labels: HashMap<String, String>) {
        self.record(name, MetricValue::Gauge(value), labels);
    }

    /// Record a metric with custom value type and labels.
    pub fn record(&mut self, name: &str, value: MetricValue, labels: HashMap<String, String>) {
        if !self.enabled {
            return;
        }

        let metric = Metric {
            name: name.to_string(),
            value,
            labels,
            timestamp_secs: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
        };

        self.metrics.insert(name.to_string(), metric);
    }

    /// Check if it's time to export
    pub fn should_export(&self) -> bool {
        self.enabled && self.last_export.elapsed() >= self.interval
    }

    /// Build a snapshot of all current metrics.
    fn build_snapshot(&self) -> TelemetrySnapshot {
        TelemetrySnapshot {
            timestamp_secs: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            scheduler_name: self.scheduler_name.clone(),
            uptime_secs: self.start_time.elapsed().as_secs_f64(),
            metrics: self.metrics.values().cloned().collect(),
        }
    }

    /// Export metrics to the configured endpoint.
    ///
    /// For `Http` endpoints this method is **non-blocking**: it posts the
    /// snapshot to the background thread's channel and returns immediately.
    /// If the channel is full (background thread is lagging) the snapshot
    /// is silently dropped to preserve RT scheduling latency.
    ///
    /// For `LocalFile`, `Udp`, and `Stdout` endpoints, the export is
    /// performed synchronously (these operations are fast and bounded).
    pub fn export(&mut self) -> Result<(), String> {
        if !self.enabled {
            return Ok(());
        }

        let snapshot = self.build_snapshot();

        let result = if let Some(ref tx) = self.http_tx {
            // Non-blocking send: drop snapshot if channel is full rather than
            // stalling the RT scheduler thread waiting for the HTTP export to
            // drain.  The background thread will catch up on subsequent exports.
            match tx.try_send(snapshot) {
                Ok(()) | Err(mpsc::TrySendError::Full(_)) => Ok(()),
                Err(mpsc::TrySendError::Disconnected(_)) => {
                    Err("HTTP export thread has exited".to_string())
                }
            }
        } else {
            match &self.endpoint {
                TelemetryEndpoint::LocalFile(path) => {
                    let path = path.clone();
                    self.export_to_file(&path, &snapshot)
                }
                TelemetryEndpoint::Udp(addr) => {
                    let addr = addr.clone();
                    self.export_to_udp(&addr, &snapshot)
                }
                TelemetryEndpoint::Http(_) => {
                    // http_tx is always Some when endpoint is Http; this branch
                    // is unreachable, but handled gracefully.
                    Ok(())
                }
                TelemetryEndpoint::Stdout => self.export_to_stdout(&snapshot),
                TelemetryEndpoint::Disabled => Ok(()),
            }
        };

        self.last_export = Instant::now();
        result
    }

    /// Export to local file
    fn export_to_file(&self, path: &PathBuf, snapshot: &TelemetrySnapshot) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }

        let json = serde_json::to_string_pretty(snapshot).map_err(|e| e.to_string())?;

        let mut file = File::create(path).map_err(|e| e.to_string())?;
        file.write_all(json.as_bytes()).map_err(|e| e.to_string())?;

        Ok(())
    }

    /// Export to UDP endpoint
    fn export_to_udp(&self, addr: &str, snapshot: &TelemetrySnapshot) -> Result<(), String> {
        if let Some(ref socket) = self.udp_socket {
            let json = serde_json::to_string(snapshot).map_err(|e| e.to_string())?;

            socket
                .send_to(json.as_bytes(), addr)
                .map_err(|e| e.to_string())?;

            Ok(())
        } else {
            Err("UDP socket not initialized".to_string())
        }
    }

    /// Export to stdout (for debugging)
    fn export_to_stdout(&self, snapshot: &TelemetrySnapshot) -> Result<(), String> {
        println!("[TELEMETRY] === Metrics Snapshot ===");
        println!(
            "  Scheduler: {} | Uptime: {:.1}s",
            snapshot.scheduler_name, snapshot.uptime_secs
        );
        for metric in &snapshot.metrics {
            println!("  {} = {:?}", metric.name, metric.value);
        }
        println!("[TELEMETRY] ========================");
        Ok(())
    }
}

impl Default for TelemetryManager {
    fn default() -> Self {
        Self::new(TelemetryEndpoint::Disabled, 1000)
    }
}

impl Drop for TelemetryManager {
    fn drop(&mut self) {
        // Dropping the sender signals the background thread to exit (the
        // receiver's `recv()` returns `Err` when all senders are gone).
        drop(self.http_tx.take());
        // Join the background thread so it drains any queued snapshots
        // before the process exits.
        if let Some(handle) = self.http_thread.take() {
            let _ = handle.join();
        }
    }
}

/// Blocking HTTP POST of a telemetry snapshot.
///
/// This function runs exclusively on the dedicated `horus-telemetry-http`
/// background thread — **never** on the RT scheduler thread.
fn http_post_blocking(url: &str, snapshot: &TelemetrySnapshot) -> Result<(), String> {
    use std::io::{BufRead, BufReader};
    use std::net::TcpStream;

    let json = serde_json::to_string(snapshot).map_err(|e| e.to_string())?;

    let url_stripped = url
        .trim_start_matches("http://")
        .trim_start_matches("https://");
    let (host, path) = if let Some(idx) = url_stripped.find('/') {
        (&url_stripped[..idx], &url_stripped[idx..])
    } else {
        (url_stripped, "/")
    };

    let mut stream =
        TcpStream::connect(format!("{}:80", host)).map_err(|e| format!("Connect failed: {}", e))?;

    stream.set_write_timeout(Some(Duration::from_secs(5))).ok();
    stream.set_read_timeout(Some(Duration::from_secs(5))).ok();

    let request = format!(
        "POST {} HTTP/1.1\r\n\
         Host: {}\r\n\
         Content-Type: application/json\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n\
         {}",
        path,
        host,
        json.len(),
        json
    );

    stream
        .write_all(request.as_bytes())
        .map_err(|e| format!("Write failed: {}", e))?;

    let mut reader = BufReader::new(&stream);
    let mut status_line = String::new();
    reader
        .read_line(&mut status_line)
        .map_err(|e| format!("Read failed: {}", e))?;

    if !status_line.contains("200") && !status_line.contains("201") && !status_line.contains("204")
    {
        return Err(format!("HTTP error: {}", status_line.trim()));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_telemetry_file_export() {
        let temp_dir = TempDir::new().unwrap();
        let path = temp_dir.path().join("metrics.json");

        let mut tm = TelemetryManager::new(TelemetryEndpoint::LocalFile(path.clone()), 100);

        tm.counter("scheduler_ticks", 1000);
        tm.gauge("cpu_usage", 45.5);

        let result = tm.export();
        assert!(result.is_ok());
        assert!(path.exists());
    }

    /// HTTP export must be non-blocking from the caller's perspective.
    ///
    /// We create a TelemetryManager with an Http endpoint pointing at a port
    /// where no server is listening.  `export()` must return almost instantly
    /// regardless — the actual (failing) TCP connect happens on the background
    /// thread, not on the test thread.
    ///
    /// A generous 200 ms budget is used to allow for scheduling jitter; a
    /// blocking connect would typically take ~75 s (OS TCP timeout) and would
    /// trigger the 200 ms assertion failure well before then.
    #[test]
    fn http_export_is_non_blocking() {
        // Port 19999 is unlikely to have a server, so the background thread
        // will get a connection refused error — but the test thread must not
        // wait for that.
        let mut tm = TelemetryManager::new(
            TelemetryEndpoint::Http("http://127.0.0.1:19999/metrics".to_string()),
            100,
        );
        tm.gauge("test_metric", 1.0);

        let start = std::time::Instant::now();
        // Call export multiple times to ensure no single call blocks.
        for _ in 0..5 {
            let _ = tm.export();
        }
        let elapsed = start.elapsed();

        assert!(
            elapsed < std::time::Duration::from_millis(200),
            "export() with Http endpoint must be non-blocking; took {:?}",
            elapsed
        );
        // TelemetryManager drop will join the background thread; the test waits
        // for it to finish (the thread will quickly error on connect refused).
    }

    #[test]
    fn test_telemetry_endpoint_parsing() {
        assert!(matches!(
            TelemetryEndpoint::from_string("disabled"),
            TelemetryEndpoint::Disabled
        ));
        assert!(matches!(
            TelemetryEndpoint::from_string("stdout"),
            TelemetryEndpoint::Stdout
        ));
        assert!(matches!(
            TelemetryEndpoint::from_string("/tmp/metrics.json"),
            TelemetryEndpoint::LocalFile(_)
        ));
        assert!(matches!(
            TelemetryEndpoint::from_string("udp://127.0.0.1:9999"),
            TelemetryEndpoint::Udp(_)
        ));
    }

}
