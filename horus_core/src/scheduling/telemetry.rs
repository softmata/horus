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
}

impl TelemetryManager {
    /// Create a new telemetry manager
    pub fn new(endpoint: TelemetryEndpoint, interval_ms: u64) -> Self {
        let enabled = !matches!(endpoint, TelemetryEndpoint::Disabled);

        let udp_socket = if let TelemetryEndpoint::Udp(_) = &endpoint {
            UdpSocket::bind("0.0.0.0:0").ok()
        } else {
            None
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

    /// Record a metric with custom value type and labels
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

    /// Export metrics to configured endpoint
    pub fn export(&mut self) -> Result<(), String> {
        if !self.enabled {
            return Ok(());
        }

        let snapshot = TelemetrySnapshot {
            timestamp_secs: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            scheduler_name: self.scheduler_name.clone(),
            uptime_secs: self.start_time.elapsed().as_secs_f64(),
            metrics: self.metrics.values().cloned().collect(),
        };

        let result = match &self.endpoint {
            TelemetryEndpoint::LocalFile(path) => self.export_to_file(path, &snapshot),
            TelemetryEndpoint::Udp(addr) => self.export_to_udp(addr, &snapshot),
            TelemetryEndpoint::Http(url) => self.export_to_http(url, &snapshot),
            TelemetryEndpoint::Stdout => self.export_to_stdout(&snapshot),
            TelemetryEndpoint::Disabled => Ok(()),
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

    /// Export to HTTP endpoint (blocking, use sparingly)
    fn export_to_http(&self, url: &str, snapshot: &TelemetrySnapshot) -> Result<(), String> {
        // Use a simple blocking POST
        // In production, you'd want async here
        let json = serde_json::to_string(snapshot).map_err(|e| e.to_string())?;

        // Simple HTTP POST without external dependencies
        // Parse URL
        let url = url
            .trim_start_matches("http://")
            .trim_start_matches("https://");
        let (host, path) = if let Some(idx) = url.find('/') {
            (&url[..idx], &url[idx..])
        } else {
            (url, "/")
        };

        // Connect and send
        use std::io::{BufRead, BufReader};
        use std::net::TcpStream;

        let mut stream = TcpStream::connect(format!("{}:80", host))
            .map_err(|e| format!("Connect failed: {}", e))?;

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

        // Read response (just check status)
        let mut reader = BufReader::new(&stream);
        let mut status_line = String::new();
        reader
            .read_line(&mut status_line)
            .map_err(|e| format!("Read failed: {}", e))?;

        if !status_line.contains("200")
            && !status_line.contains("201")
            && !status_line.contains("204")
        {
            return Err(format!("HTTP error: {}", status_line.trim()));
        }

        Ok(())
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
