//! Tracing and instrumentation support for detailed profiling.
//!
//! Provides:
//! - Span-based tracing for benchmark phases
//! - Event logging with timestamps
//! - Integration hooks for external profilers (perf, flamegraph)
//! - Structured trace output for analysis
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_benchmarks::tracing::{BenchmarkTracer, TraceEvent};
//!
//! let mut tracer = BenchmarkTracer::new("ipc_latency");
//! tracer.start_phase("warmup");
//! // ... warmup code ...
//! tracer.end_phase();
//!
//! tracer.start_phase("measurement");
//! for i in 0..iterations {
//!     tracer.record_event(TraceEvent::SendStart(i));
//!     // ... send ...
//!     tracer.record_event(TraceEvent::SendEnd(i));
//! }
//! tracer.end_phase();
//!
//! tracer.write_chrome_trace("trace.json")?;
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::{Duration, Instant};

/// Trace event types for IPC benchmarking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TraceEvent {
    /// Message send started
    SendStart(u64),
    /// Message send completed
    SendEnd(u64),
    /// Message receive started
    ReceiveStart(u64),
    /// Message receive completed
    ReceiveEnd(u64),
    /// Custom event with name
    Custom(String),
    /// Latency measurement recorded
    LatencyRecorded(u64, u64), // (sequence, latency_ns)
}

/// A recorded trace entry with timestamp
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TraceEntry {
    /// Timestamp in nanoseconds from trace start
    pub timestamp_ns: u64,
    /// Event that occurred
    pub event: TraceEvent,
    /// Thread ID that recorded this event
    pub thread_id: u64,
    /// Optional additional metadata
    pub metadata: Option<HashMap<String, String>>,
}

/// A trace phase (span) with start/end times
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TracePhase {
    /// Phase name
    pub name: String,
    /// Start time in nanoseconds from trace start
    pub start_ns: u64,
    /// End time in nanoseconds (None if still active)
    pub end_ns: Option<u64>,
    /// Events recorded during this phase
    pub events: Vec<TraceEntry>,
}

/// Benchmark tracer for detailed profiling
#[derive(Debug)]
pub struct BenchmarkTracer {
    /// Benchmark name
    name: String,
    /// Trace start instant
    start: Instant,
    /// Recorded phases
    phases: Vec<TracePhase>,
    /// Currently active phase index
    active_phase: Option<usize>,
    /// Global events (outside phases)
    global_events: Vec<TraceEntry>,
    /// Whether tracing is enabled
    enabled: bool,
}

impl BenchmarkTracer {
    /// Create a new tracer for a benchmark
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            start: Instant::now(),
            phases: Vec::new(),
            active_phase: None,
            global_events: Vec::new(),
            enabled: true,
        }
    }

    /// Create a disabled tracer (no-op for production)
    pub fn disabled() -> Self {
        Self {
            name: String::new(),
            start: Instant::now(),
            phases: Vec::new(),
            active_phase: None,
            global_events: Vec::new(),
            enabled: false,
        }
    }

    /// Check if tracing is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Get elapsed nanoseconds since trace start
    fn elapsed_ns(&self) -> u64 {
        self.start.elapsed().as_nanos() as u64
    }

    /// Get current thread ID
    fn thread_id() -> u64 {
        // Use format + parse to get a stable thread ID (as_u64 is unstable)
        let id = std::thread::current().id();
        let id_str = format!("{:?}", id);
        // Extract the number from "ThreadId(N)"
        id_str
            .trim_start_matches("ThreadId(")
            .trim_end_matches(')')
            .parse()
            .unwrap_or(0)
    }

    /// Start a new trace phase
    pub fn start_phase(&mut self, name: &str) {
        if !self.enabled {
            return;
        }

        // End any active phase first
        if self.active_phase.is_some() {
            self.end_phase();
        }

        let phase = TracePhase {
            name: name.to_string(),
            start_ns: self.elapsed_ns(),
            end_ns: None,
            events: Vec::new(),
        };

        self.phases.push(phase);
        self.active_phase = Some(self.phases.len() - 1);
    }

    /// End the current phase
    pub fn end_phase(&mut self) {
        if !self.enabled {
            return;
        }

        if let Some(idx) = self.active_phase {
            self.phases[idx].end_ns = Some(self.elapsed_ns());
            self.active_phase = None;
        }
    }

    /// Record a trace event
    pub fn record_event(&mut self, event: TraceEvent) {
        if !self.enabled {
            return;
        }

        let entry = TraceEntry {
            timestamp_ns: self.elapsed_ns(),
            event,
            thread_id: Self::thread_id(),
            metadata: None,
        };

        if let Some(idx) = self.active_phase {
            self.phases[idx].events.push(entry);
        } else {
            self.global_events.push(entry);
        }
    }

    /// Record an event with metadata
    pub fn record_event_with_metadata(
        &mut self,
        event: TraceEvent,
        metadata: HashMap<String, String>,
    ) {
        if !self.enabled {
            return;
        }

        let entry = TraceEntry {
            timestamp_ns: self.elapsed_ns(),
            event,
            thread_id: Self::thread_id(),
            metadata: Some(metadata),
        };

        if let Some(idx) = self.active_phase {
            self.phases[idx].events.push(entry);
        } else {
            self.global_events.push(entry);
        }
    }

    /// Record a latency measurement
    pub fn record_latency(&mut self, sequence: u64, latency_ns: u64) {
        self.record_event(TraceEvent::LatencyRecorded(sequence, latency_ns));
    }

    /// Get all phases
    pub fn phases(&self) -> &[TracePhase] {
        &self.phases
    }

    /// Get total trace duration
    pub fn total_duration(&self) -> Duration {
        self.start.elapsed()
    }

    /// Write trace in Chrome Trace Format (for chrome://tracing)
    pub fn write_chrome_trace<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);

        // Chrome trace format is JSON array of events
        writeln!(writer, "[")?;

        let mut first = true;
        let pid = std::process::id();

        // Write phase spans
        for phase in &self.phases {
            if !first {
                writeln!(writer, ",")?;
            }
            first = false;

            // Duration event (B/E pair)
            write!(
                writer,
                r#"{{"name":"{}","cat":"phase","ph":"B","ts":{},"pid":{},"tid":1}}"#,
                phase.name,
                phase.start_ns / 1000, // Chrome uses microseconds
                pid
            )?;

            if let Some(end_ns) = phase.end_ns {
                writeln!(writer, ",")?;
                write!(
                    writer,
                    r#"{{"name":"{}","cat":"phase","ph":"E","ts":{},"pid":{},"tid":1}}"#,
                    phase.name,
                    end_ns / 1000,
                    pid
                )?;
            }

            // Write events within phase
            for event in &phase.events {
                writeln!(writer, ",")?;
                self.write_chrome_event(&mut writer, event, pid)?;
            }
        }

        // Write global events
        for event in &self.global_events {
            if !first {
                writeln!(writer, ",")?;
            }
            first = false;
            self.write_chrome_event(&mut writer, event, pid)?;
        }

        writeln!(writer, "\n]")?;
        Ok(())
    }

    fn write_chrome_event<W: Write>(
        &self,
        writer: &mut W,
        entry: &TraceEntry,
        pid: u32,
    ) -> std::io::Result<()> {
        let (name, cat) = match &entry.event {
            TraceEvent::SendStart(seq) => (format!("send_{}", seq), "ipc"),
            TraceEvent::SendEnd(seq) => (format!("send_{}_end", seq), "ipc"),
            TraceEvent::ReceiveStart(seq) => (format!("recv_{}", seq), "ipc"),
            TraceEvent::ReceiveEnd(seq) => (format!("recv_{}_end", seq), "ipc"),
            TraceEvent::Custom(name) => (name.clone(), "custom"),
            TraceEvent::LatencyRecorded(seq, lat) => {
                (format!("latency_{}_{}", seq, lat), "measurement")
            }
        };

        write!(
            writer,
            r#"{{"name":"{}","cat":"{}","ph":"i","ts":{},"pid":{},"tid":{},"s":"g"}}"#,
            name,
            cat,
            entry.timestamp_ns / 1000,
            pid,
            entry.thread_id
        )
    }

    /// Write trace as JSON for analysis
    pub fn write_json<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let file = File::create(path)?;
        let writer = BufWriter::new(file);

        let trace_data = TraceData {
            name: self.name.clone(),
            duration_ns: self.start.elapsed().as_nanos() as u64,
            phases: self.phases.clone(),
            global_events: self.global_events.clone(),
        };

        serde_json::to_writer_pretty(writer, &trace_data)?;
        Ok(())
    }

    /// Print a summary of the trace
    pub fn print_summary(&self) {
        println!("╔══════════════════════════════════════════════════════════════════╗");
        println!("║                    TRACE SUMMARY: {:30} ║", self.name);
        println!("╠══════════════════════════════════════════════════════════════════╣");
        println!(
            "║ Total duration: {:>10.3} ms                                    ║",
            self.start.elapsed().as_secs_f64() * 1000.0
        );
        println!("║ Phases: {:>3}                                                      ║", self.phases.len());
        println!("╠══════════════════════════════════════════════════════════════════╣");

        for phase in &self.phases {
            let duration_ms = phase
                .end_ns
                .map(|e| (e - phase.start_ns) as f64 / 1_000_000.0)
                .unwrap_or(0.0);
            println!(
                "║ {:20} │ {:>8.3} ms │ {:>6} events                   ║",
                phase.name,
                duration_ms,
                phase.events.len()
            );
        }

        if !self.global_events.is_empty() {
            println!(
                "║ {:20} │            │ {:>6} events                   ║",
                "global",
                self.global_events.len()
            );
        }

        println!("╚══════════════════════════════════════════════════════════════════╝");
    }
}

/// Serializable trace data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TraceData {
    /// Benchmark name
    pub name: String,
    /// Total duration in nanoseconds
    pub duration_ns: u64,
    /// Recorded phases
    pub phases: Vec<TracePhase>,
    /// Global events
    pub global_events: Vec<TraceEntry>,
}

/// Scoped phase guard for automatic phase ending
pub struct PhaseGuard<'a> {
    tracer: &'a mut BenchmarkTracer,
}

impl<'a> PhaseGuard<'a> {
    /// Create a new phase guard
    pub fn new(tracer: &'a mut BenchmarkTracer, name: &str) -> Self {
        tracer.start_phase(name);
        Self { tracer }
    }
}

impl<'a> Drop for PhaseGuard<'a> {
    fn drop(&mut self) {
        self.tracer.end_phase();
    }
}

/// Convenience macro for scoped tracing
#[macro_export]
macro_rules! trace_phase {
    ($tracer:expr, $name:expr) => {
        let _guard = $crate::tracing::PhaseGuard::new($tracer, $name);
    };
}

/// Integration with Linux perf
#[cfg(target_os = "linux")]
pub mod perf_integration {
    use std::process::Command;

    /// Check if perf is available
    pub fn is_perf_available() -> bool {
        Command::new("perf")
            .arg("--version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    }

    /// Generate perf command for benchmarking
    pub fn perf_stat_command(binary: &str, args: &[&str]) -> Command {
        let mut cmd = Command::new("perf");
        cmd.args([
            "stat",
            "-e",
            "cycles,instructions,cache-references,cache-misses,branches,branch-misses",
            "--",
            binary,
        ]);
        cmd.args(args);
        cmd
    }

    /// Generate perf record command for profiling
    pub fn perf_record_command(binary: &str, args: &[&str], output: &str) -> Command {
        let mut cmd = Command::new("perf");
        cmd.args(["record", "-g", "-o", output, "--", binary]);
        cmd.args(args);
        cmd
    }
}

#[cfg(not(target_os = "linux"))]
pub mod perf_integration {
    use std::process::Command;

    pub fn is_perf_available() -> bool {
        false
    }

    pub fn perf_stat_command(binary: &str, args: &[&str]) -> Command {
        let mut cmd = Command::new(binary);
        cmd.args(args);
        cmd
    }

    pub fn perf_record_command(binary: &str, args: &[&str], _output: &str) -> Command {
        let mut cmd = Command::new(binary);
        cmd.args(args);
        cmd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tracer_basic() {
        let mut tracer = BenchmarkTracer::new("test");

        tracer.start_phase("phase1");
        tracer.record_event(TraceEvent::SendStart(0));
        tracer.record_event(TraceEvent::SendEnd(0));
        tracer.end_phase();

        assert_eq!(tracer.phases().len(), 1);
        assert_eq!(tracer.phases()[0].name, "phase1");
        assert_eq!(tracer.phases()[0].events.len(), 2);
    }

    #[test]
    fn test_tracer_disabled() {
        let mut tracer = BenchmarkTracer::disabled();

        tracer.start_phase("phase1");
        tracer.record_event(TraceEvent::SendStart(0));
        tracer.end_phase();

        // Should not record anything when disabled
        assert_eq!(tracer.phases().len(), 0);
    }

    #[test]
    fn test_latency_recording() {
        let mut tracer = BenchmarkTracer::new("latency_test");

        tracer.start_phase("measurement");
        for i in 0..10 {
            tracer.record_latency(i, 100 + i);
        }
        tracer.end_phase();

        assert_eq!(tracer.phases()[0].events.len(), 10);
    }

    #[test]
    fn test_phase_auto_end() {
        let mut tracer = BenchmarkTracer::new("test");

        tracer.start_phase("phase1");
        tracer.start_phase("phase2"); // Should auto-end phase1

        assert_eq!(tracer.phases().len(), 2);
        assert!(tracer.phases()[0].end_ns.is_some()); // phase1 ended
        assert!(tracer.phases()[1].end_ns.is_none()); // phase2 still active
    }
}
