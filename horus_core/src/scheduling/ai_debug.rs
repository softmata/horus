//! AI Debug Assistant
//!
//! Intelligent debugging assistant for HORUS recordings that provides:
//! - Pattern detection for common robotics issues
//! - Anomaly detection using statistical analysis
//! - Root cause analysis through causal tracing
//! - Suggested fixes based on detected patterns
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::scheduling::ai_debug::*;
//!
//! // Create assistant with recording data
//! let mut assistant = DebugAssistant::new();
//!
//! // Analyze a recording
//! let analysis = assistant.analyze_recording(&recording)?;
//!
//! // Get detected issues
//! for issue in &analysis.issues {
//!     println!("{}: {}", issue.severity, issue.description);
//!     for suggestion in &issue.suggestions {
//!         println!("  - {}", suggestion);
//!     }
//! }
//! ```

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

/// Issue severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Severity {
    Info,
    Warning,
    Error,
    Critical,
}

impl std::fmt::Display for Severity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Severity::Info => write!(f, "INFO"),
            Severity::Warning => write!(f, "WARN"),
            Severity::Error => write!(f, "ERROR"),
            Severity::Critical => write!(f, "CRITICAL"),
        }
    }
}

/// Category of detected issue
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueCategory {
    /// Timing-related issues (missed deadlines, jitter)
    Timing,
    /// Sensor data anomalies (dropouts, spikes, drift)
    SensorAnomaly,
    /// Communication issues (message loss, latency)
    Communication,
    /// Resource issues (memory, CPU)
    Resource,
    /// Control system issues (oscillation, instability)
    Control,
    /// State machine issues (invalid transitions)
    StateMachine,
    /// Data flow issues (stale data, race conditions)
    DataFlow,
    /// Configuration issues
    Configuration,
}

/// A detected issue with context and suggestions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedIssue {
    /// Unique identifier for this issue
    pub id: String,
    /// Severity level
    pub severity: Severity,
    /// Category of the issue
    pub category: IssueCategory,
    /// Human-readable description
    pub description: String,
    /// Tick range where the issue was detected
    pub tick_range: (u64, u64),
    /// Affected nodes
    pub affected_nodes: Vec<String>,
    /// Affected topics
    pub affected_topics: Vec<String>,
    /// Suggested fixes
    pub suggestions: Vec<String>,
    /// Confidence score (0.0 - 1.0)
    pub confidence: f64,
    /// Related issues (by ID)
    pub related_issues: Vec<String>,
    /// Additional context data
    pub context: HashMap<String, String>,
}

/// Analysis result from the debug assistant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisResult {
    /// Detected issues
    pub issues: Vec<DetectedIssue>,
    /// Overall health score (0.0 - 1.0)
    pub health_score: f64,
    /// Summary statistics
    pub stats: AnalysisStats,
    /// Timeline of events
    pub timeline: Vec<TimelineEvent>,
}

/// Summary statistics from analysis
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AnalysisStats {
    pub total_ticks: u64,
    pub total_messages: u64,
    pub nodes_analyzed: usize,
    pub topics_analyzed: usize,
    pub timing_violations: usize,
    pub anomalies_detected: usize,
    pub avg_tick_duration_us: f64,
    pub max_tick_duration_us: f64,
    pub message_loss_rate: f64,
}

/// Event in the analysis timeline
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimelineEvent {
    pub tick: u64,
    pub timestamp_ns: u64,
    pub event_type: TimelineEventType,
    pub description: String,
    pub severity: Severity,
}

/// Types of timeline events
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TimelineEventType {
    IssueDetected(String), // Issue ID
    AnomalyStart,
    AnomalyEnd,
    PerformanceDegradation,
    Recovery,
    StateChange,
}

/// Pattern detector trait for extensibility
pub trait PatternDetector: Send + Sync {
    /// Name of this detector
    fn name(&self) -> &str;

    /// Analyze tick data and return detected issues
    fn analyze(&self, context: &AnalysisContext) -> Vec<DetectedIssue>;
}

/// Context provided to pattern detectors
#[derive(Debug, Clone)]
pub struct AnalysisContext {
    /// Tick timings (tick -> duration_ns)
    pub tick_timings: Vec<(u64, u64)>,
    /// Node execution data
    pub node_data: HashMap<String, NodeAnalysisData>,
    /// Topic message counts
    pub topic_messages: HashMap<String, Vec<(u64, usize)>>,
    /// Current tick being analyzed
    pub current_tick: u64,
    /// Total ticks in recording
    pub total_ticks: u64,
}

/// Analysis data for a single node
#[derive(Debug, Clone, Default)]
pub struct NodeAnalysisData {
    pub name: String,
    pub tick_durations_ns: Vec<u64>,
    pub message_counts: Vec<usize>,
    pub error_counts: Vec<usize>,
}

/// AI Debug Assistant
pub struct DebugAssistant {
    /// Registered pattern detectors
    detectors: Vec<Box<dyn PatternDetector>>,
    /// Configuration
    config: AssistantConfig,
}

/// Configuration for the debug assistant
#[derive(Debug, Clone)]
pub struct AssistantConfig {
    /// Timing threshold for warnings (in microseconds)
    pub timing_warn_threshold_us: f64,
    /// Timing threshold for errors (in microseconds)
    pub timing_error_threshold_us: f64,
    /// Jitter threshold (percentage)
    pub jitter_threshold_percent: f64,
    /// Message loss threshold (percentage)
    pub message_loss_threshold_percent: f64,
    /// Minimum confidence to report issues
    pub min_confidence: f64,
    /// Enable ML-based anomaly detection
    pub enable_ml_detection: bool,
}

impl Default for AssistantConfig {
    fn default() -> Self {
        Self {
            timing_warn_threshold_us: 1000.0,   // 1ms
            timing_error_threshold_us: 10000.0, // 10ms
            jitter_threshold_percent: 20.0,
            message_loss_threshold_percent: 1.0,
            min_confidence: 0.5,
            enable_ml_detection: true,
        }
    }
}

impl Default for DebugAssistant {
    fn default() -> Self {
        Self::new()
    }
}

impl DebugAssistant {
    /// Create a new debug assistant with default detectors
    pub fn new() -> Self {
        let mut assistant = Self {
            detectors: Vec::new(),
            config: AssistantConfig::default(),
        };

        // Register built-in detectors
        assistant.register_detector(Box::new(TimingViolationDetector::new()));
        assistant.register_detector(Box::new(MessageLossDetector::new()));
        assistant.register_detector(Box::new(JitterDetector::new()));
        assistant.register_detector(Box::new(SensorAnomalyDetector::new()));
        assistant.register_detector(Box::new(ControlInstabilityDetector::new()));

        assistant
    }

    /// Create with custom configuration
    pub fn with_config(config: AssistantConfig) -> Self {
        let mut assistant = Self::new();
        assistant.config = config;
        assistant
    }

    /// Register a custom pattern detector
    pub fn register_detector(&mut self, detector: Box<dyn PatternDetector>) {
        self.detectors.push(detector);
    }

    /// Analyze tick timing data
    pub fn analyze_timings(&self, tick_timings: &[(u64, u64)]) -> AnalysisResult {
        let context = AnalysisContext {
            tick_timings: tick_timings.to_vec(),
            node_data: HashMap::new(),
            topic_messages: HashMap::new(),
            current_tick: tick_timings.last().map(|(t, _)| *t).unwrap_or(0),
            total_ticks: tick_timings.len() as u64,
        };

        self.run_analysis(&context)
    }

    /// Run all detectors on the analysis context
    fn run_analysis(&self, context: &AnalysisContext) -> AnalysisResult {
        let mut all_issues = Vec::new();
        let mut timeline = Vec::new();

        // Run each detector
        for detector in &self.detectors {
            let issues = detector.analyze(context);
            for issue in issues {
                if issue.confidence >= self.config.min_confidence {
                    // Add timeline event for the issue
                    timeline.push(TimelineEvent {
                        tick: issue.tick_range.0,
                        timestamp_ns: 0, // Would come from actual recording
                        event_type: TimelineEventType::IssueDetected(issue.id.clone()),
                        description: issue.description.clone(),
                        severity: issue.severity,
                    });
                    all_issues.push(issue);
                }
            }
        }

        // Sort issues by severity (critical first)
        all_issues.sort_by(|a, b| b.severity.cmp(&a.severity));

        // Calculate health score
        let health_score = self.calculate_health_score(&all_issues, context);

        // Calculate stats
        let stats = self.calculate_stats(context, &all_issues);

        // Sort timeline by tick
        timeline.sort_by_key(|e| e.tick);

        AnalysisResult {
            issues: all_issues,
            health_score,
            stats,
            timeline,
        }
    }

    /// Calculate overall health score
    fn calculate_health_score(&self, issues: &[DetectedIssue], context: &AnalysisContext) -> f64 {
        if context.total_ticks == 0 {
            return 1.0;
        }

        let mut score = 1.0;

        for issue in issues {
            let penalty = match issue.severity {
                Severity::Critical => 0.3,
                Severity::Error => 0.15,
                Severity::Warning => 0.05,
                Severity::Info => 0.01,
            };
            score -= penalty * issue.confidence;
        }

        score.max(0.0)
    }

    /// Calculate analysis statistics
    fn calculate_stats(
        &self,
        context: &AnalysisContext,
        issues: &[DetectedIssue],
    ) -> AnalysisStats {
        let timing_durations: Vec<f64> = context
            .tick_timings
            .iter()
            .map(|(_, d)| *d as f64 / 1000.0) // ns to us
            .collect();

        let avg_tick = if timing_durations.is_empty() {
            0.0
        } else {
            timing_durations.iter().sum::<f64>() / timing_durations.len() as f64
        };

        let max_tick = timing_durations.iter().cloned().fold(0.0, f64::max);

        AnalysisStats {
            total_ticks: context.total_ticks,
            total_messages: context
                .topic_messages
                .values()
                .flat_map(|v| v.iter().map(|(_, c)| *c))
                .sum::<usize>() as u64,
            nodes_analyzed: context.node_data.len(),
            topics_analyzed: context.topic_messages.len(),
            timing_violations: issues
                .iter()
                .filter(|i| i.category == IssueCategory::Timing)
                .count(),
            anomalies_detected: issues
                .iter()
                .filter(|i| i.category == IssueCategory::SensorAnomaly)
                .count(),
            avg_tick_duration_us: avg_tick,
            max_tick_duration_us: max_tick,
            message_loss_rate: 0.0, // Would need expected vs actual
        }
    }

    /// Get suggested fixes for a specific issue
    pub fn get_suggestions(&self, issue: &DetectedIssue) -> Vec<String> {
        let mut suggestions = issue.suggestions.clone();

        // Add category-specific suggestions
        match issue.category {
            IssueCategory::Timing => {
                suggestions.push("Consider increasing node priority".to_string());
                suggestions.push("Profile tick() implementation for bottlenecks".to_string());
                suggestions.push("Check for blocking operations in tick()".to_string());
            }
            IssueCategory::SensorAnomaly => {
                suggestions.push("Verify sensor hardware connections".to_string());
                suggestions.push("Add input validation/filtering".to_string());
                suggestions.push("Check sensor calibration".to_string());
            }
            IssueCategory::Communication => {
                suggestions.push("Check network latency and bandwidth".to_string());
                suggestions.push("Consider adding message buffering".to_string());
                suggestions
                    .push("Verify topic names match between publishers/subscribers".to_string());
            }
            IssueCategory::Control => {
                suggestions.push("Review PID gains".to_string());
                suggestions.push("Check for sensor delay compensation".to_string());
                suggestions.push("Add rate limiting to control outputs".to_string());
            }
            _ => {}
        }

        suggestions
    }
}

// ============================================================================
// Built-in Pattern Detectors
// ============================================================================

/// Detects timing violations (missed deadlines)
struct TimingViolationDetector {
    threshold_us: f64,
}

impl TimingViolationDetector {
    fn new() -> Self {
        Self {
            threshold_us: 10000.0, // 10ms default
        }
    }
}

impl PatternDetector for TimingViolationDetector {
    fn name(&self) -> &str {
        "TimingViolationDetector"
    }

    fn analyze(&self, context: &AnalysisContext) -> Vec<DetectedIssue> {
        let mut issues = Vec::new();
        let mut violation_start: Option<u64> = None;
        let mut violation_count = 0;

        for (tick, duration_ns) in &context.tick_timings {
            let duration_us = *duration_ns as f64 / 1000.0;

            if duration_us > self.threshold_us {
                if violation_start.is_none() {
                    violation_start = Some(*tick);
                }
                violation_count += 1;
            } else if let Some(start) = violation_start {
                // End of violation period
                let severity = if violation_count > 10 {
                    Severity::Critical
                } else if violation_count > 5 {
                    Severity::Error
                } else {
                    Severity::Warning
                };

                issues.push(DetectedIssue {
                    id: format!("timing_violation_{}", start),
                    severity,
                    category: IssueCategory::Timing,
                    description: format!(
                        "Timing violation: {} ticks exceeded {}us threshold",
                        violation_count, self.threshold_us
                    ),
                    tick_range: (start, *tick - 1),
                    affected_nodes: Vec::new(),
                    affected_topics: Vec::new(),
                    suggestions: vec![
                        "Reduce tick() execution time".to_string(),
                        "Increase scheduler frequency budget".to_string(),
                    ],
                    confidence: 0.95,
                    related_issues: Vec::new(),
                    context: HashMap::new(),
                });

                violation_start = None;
                violation_count = 0;
            }
        }

        // Handle violations that persisted to the end of the recording
        if let Some(start) = violation_start {
            let last_tick = context
                .tick_timings
                .last()
                .map(|(t, _)| *t)
                .unwrap_or(start);
            let severity = if violation_count > 10 {
                Severity::Critical
            } else if violation_count > 5 {
                Severity::Error
            } else {
                Severity::Warning
            };

            issues.push(DetectedIssue {
                id: format!("timing_violation_{}", start),
                severity,
                category: IssueCategory::Timing,
                description: format!(
                    "Timing violation: {} ticks exceeded {}us threshold (ongoing)",
                    violation_count, self.threshold_us
                ),
                tick_range: (start, last_tick),
                affected_nodes: Vec::new(),
                affected_topics: Vec::new(),
                suggestions: vec![
                    "Reduce tick() execution time".to_string(),
                    "Increase scheduler frequency budget".to_string(),
                    "System is consistently overloaded - review workload".to_string(),
                ],
                confidence: 0.95,
                related_issues: Vec::new(),
                context: HashMap::new(),
            });
        }

        issues
    }
}

/// Detects message loss patterns
struct MessageLossDetector {
    threshold_percent: f64,
}

impl MessageLossDetector {
    fn new() -> Self {
        Self {
            threshold_percent: 1.0,
        }
    }
}

impl PatternDetector for MessageLossDetector {
    fn name(&self) -> &str {
        "MessageLossDetector"
    }

    fn analyze(&self, context: &AnalysisContext) -> Vec<DetectedIssue> {
        let mut issues = Vec::new();

        for (topic, messages) in &context.topic_messages {
            if messages.len() < 2 {
                continue;
            }

            // Look for gaps in message sequence
            let mut gaps = Vec::new();
            for window in messages.windows(2) {
                let tick_diff = window[1].0 - window[0].0;
                if tick_diff > 1 {
                    gaps.push((window[0].0, window[1].0, tick_diff));
                }
            }

            let gap_rate = gaps.len() as f64 / messages.len() as f64 * 100.0;

            if gap_rate > self.threshold_percent {
                issues.push(DetectedIssue {
                    id: format!("message_loss_{}", topic),
                    severity: if gap_rate > 10.0 {
                        Severity::Error
                    } else {
                        Severity::Warning
                    },
                    category: IssueCategory::Communication,
                    description: format!(
                        "Message loss on topic '{}': {:.1}% gap rate",
                        topic, gap_rate
                    ),
                    tick_range: (
                        messages.first().map(|(t, _)| *t).unwrap_or(0),
                        messages.last().map(|(t, _)| *t).unwrap_or(0),
                    ),
                    affected_nodes: Vec::new(),
                    affected_topics: vec![topic.clone()],
                    suggestions: vec![
                        "Check publisher health".to_string(),
                        "Verify buffer sizes".to_string(),
                    ],
                    confidence: 0.85,
                    related_issues: Vec::new(),
                    context: HashMap::new(),
                });
            }
        }

        issues
    }
}

/// Detects timing jitter
struct JitterDetector {
    threshold_percent: f64,
}

impl JitterDetector {
    fn new() -> Self {
        Self {
            threshold_percent: 20.0,
        }
    }
}

impl PatternDetector for JitterDetector {
    fn name(&self) -> &str {
        "JitterDetector"
    }

    fn analyze(&self, context: &AnalysisContext) -> Vec<DetectedIssue> {
        let mut issues = Vec::new();

        if context.tick_timings.len() < 10 {
            return issues;
        }

        let durations: Vec<f64> = context
            .tick_timings
            .iter()
            .map(|(_, d)| *d as f64)
            .collect();

        let mean = durations.iter().sum::<f64>() / durations.len() as f64;
        let variance =
            durations.iter().map(|d| (d - mean).powi(2)).sum::<f64>() / durations.len() as f64;
        let std_dev = variance.sqrt();
        let jitter_percent = (std_dev / mean) * 100.0;

        if jitter_percent > self.threshold_percent {
            issues.push(DetectedIssue {
                id: "high_jitter".to_string(),
                severity: if jitter_percent > 50.0 {
                    Severity::Error
                } else {
                    Severity::Warning
                },
                category: IssueCategory::Timing,
                description: format!(
                    "High timing jitter: {:.1}% (threshold: {:.1}%)",
                    jitter_percent, self.threshold_percent
                ),
                tick_range: (0, context.total_ticks),
                affected_nodes: Vec::new(),
                affected_topics: Vec::new(),
                suggestions: vec![
                    "Enable real-time scheduling".to_string(),
                    "Pin threads to CPU cores".to_string(),
                    "Reduce system load variability".to_string(),
                ],
                confidence: 0.9,
                related_issues: Vec::new(),
                context: {
                    let mut ctx = HashMap::new();
                    ctx.insert(
                        "jitter_percent".to_string(),
                        format!("{:.2}", jitter_percent),
                    );
                    ctx.insert("std_dev_ns".to_string(), format!("{:.0}", std_dev));
                    ctx
                },
            });
        }

        issues
    }
}

/// Detects sensor data anomalies (execution time spikes)
struct SensorAnomalyDetector {
    spike_threshold: f64,
}

impl SensorAnomalyDetector {
    fn new() -> Self {
        Self {
            spike_threshold: 3.0, // 3 standard deviations
        }
    }
}

impl PatternDetector for SensorAnomalyDetector {
    fn name(&self) -> &str {
        "SensorAnomalyDetector"
    }

    fn analyze(&self, context: &AnalysisContext) -> Vec<DetectedIssue> {
        let mut issues = Vec::new();

        // Analyze node data for anomalies
        for (node_name, data) in &context.node_data {
            if data.tick_durations_ns.len() < 10 {
                continue;
            }

            // Check for unusual execution time spikes
            let mean = data.tick_durations_ns.iter().sum::<u64>() as f64
                / data.tick_durations_ns.len() as f64;
            let variance = data
                .tick_durations_ns
                .iter()
                .map(|d| (*d as f64 - mean).powi(2))
                .sum::<f64>()
                / data.tick_durations_ns.len() as f64;
            let std_dev = variance.sqrt();

            let spikes: Vec<usize> = data
                .tick_durations_ns
                .iter()
                .enumerate()
                .filter(|(_, d)| (**d as f64 - mean).abs() > self.spike_threshold * std_dev)
                .map(|(i, _)| i)
                .collect();

            if !spikes.is_empty() {
                issues.push(DetectedIssue {
                    id: format!("sensor_spike_{}", node_name),
                    severity: Severity::Warning,
                    category: IssueCategory::SensorAnomaly,
                    description: format!(
                        "Node '{}' shows {} anomalous execution time spikes",
                        node_name,
                        spikes.len()
                    ),
                    tick_range: (
                        *spikes.first().unwrap_or(&0) as u64,
                        *spikes.last().unwrap_or(&0) as u64,
                    ),
                    affected_nodes: vec![node_name.clone()],
                    affected_topics: Vec::new(),
                    suggestions: vec![
                        "Check for GC pauses or system interrupts".to_string(),
                        "Profile node for intermittent bottlenecks".to_string(),
                    ],
                    confidence: 0.75,
                    related_issues: Vec::new(),
                    context: HashMap::new(),
                });
            }
        }

        issues
    }
}

/// Detects control system instability
struct ControlInstabilityDetector {
    oscillation_threshold: f64,
}

impl ControlInstabilityDetector {
    fn new() -> Self {
        Self {
            oscillation_threshold: 0.3, // 30% sign changes
        }
    }
}

impl PatternDetector for ControlInstabilityDetector {
    fn name(&self) -> &str {
        "ControlInstabilityDetector"
    }

    fn analyze(&self, context: &AnalysisContext) -> Vec<DetectedIssue> {
        let mut issues = Vec::new();

        // Look for rapid oscillation patterns in node behavior
        for (node_name, data) in &context.node_data {
            if data.tick_durations_ns.len() < 20 {
                continue;
            }

            // Calculate derivative (changes between samples)
            let changes: Vec<i64> = data
                .tick_durations_ns
                .windows(2)
                .map(|w| w[1] as i64 - w[0] as i64)
                .collect();

            // Count sign changes
            let sign_changes = changes
                .windows(2)
                .filter(|w| (w[0] > 0 && w[1] < 0) || (w[0] < 0 && w[1] > 0))
                .count();

            let oscillation_rate = sign_changes as f64 / changes.len() as f64;

            if oscillation_rate > self.oscillation_threshold {
                issues.push(DetectedIssue {
                    id: format!("control_oscillation_{}", node_name),
                    severity: Severity::Warning,
                    category: IssueCategory::Control,
                    description: format!(
                        "Possible control oscillation in '{}': {:.1}% sign changes",
                        node_name,
                        oscillation_rate * 100.0
                    ),
                    tick_range: (0, context.total_ticks),
                    affected_nodes: vec![node_name.clone()],
                    affected_topics: Vec::new(),
                    suggestions: vec![
                        "Reduce controller gains".to_string(),
                        "Add low-pass filtering".to_string(),
                        "Check sensor noise levels".to_string(),
                    ],
                    confidence: 0.65,
                    related_issues: Vec::new(),
                    context: {
                        let mut ctx = HashMap::new();
                        ctx.insert(
                            "oscillation_rate".to_string(),
                            format!("{:.2}", oscillation_rate),
                        );
                        ctx
                    },
                });
            }
        }

        issues
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timing_violation_detection() {
        let assistant = DebugAssistant::new();

        // Create timing data with violations
        let mut timings: Vec<(u64, u64)> = (0..100)
            .map(|i| (i, 5_000_000)) // 5ms normal
            .collect();

        // Add violations at ticks 50-60
        for i in 50..60 {
            timings[i].1 = 50_000_000; // 50ms violation
        }

        let result = assistant.analyze_timings(&timings);

        assert!(!result.issues.is_empty());
        assert!(result
            .issues
            .iter()
            .any(|i| i.category == IssueCategory::Timing));
    }

    #[test]
    fn test_jitter_detection() {
        let assistant = DebugAssistant::new();

        // Create timing data with high jitter
        let timings: Vec<(u64, u64)> = (0..100)
            .map(|i| {
                let base = 5_000_000u64;
                let jitter = if i % 2 == 0 { 3_000_000 } else { 0 };
                (i, base + jitter)
            })
            .collect();

        let result = assistant.analyze_timings(&timings);

        // Should detect jitter
        assert!(result
            .issues
            .iter()
            .any(|i| i.category == IssueCategory::Timing && i.description.contains("jitter")));
    }

    #[test]
    fn test_health_score() {
        let assistant = DebugAssistant::new();

        // Normal timings should have high health
        let good_timings: Vec<(u64, u64)> = (0..100).map(|i| (i, 5_000_000)).collect();

        let good_result = assistant.analyze_timings(&good_timings);
        assert!(good_result.health_score > 0.8);

        // Bad timings should have low health
        let bad_timings: Vec<(u64, u64)> = (0..100)
            .map(|i| (i, 100_000_000)) // 100ms per tick
            .collect();

        let bad_result = assistant.analyze_timings(&bad_timings);
        assert!(bad_result.health_score < good_result.health_score);
    }

    #[test]
    fn test_issue_suggestions() {
        let assistant = DebugAssistant::new();

        let issue = DetectedIssue {
            id: "test".to_string(),
            severity: Severity::Warning,
            category: IssueCategory::Timing,
            description: "Test issue".to_string(),
            tick_range: (0, 10),
            affected_nodes: Vec::new(),
            affected_topics: Vec::new(),
            suggestions: vec!["Original suggestion".to_string()],
            confidence: 0.9,
            related_issues: Vec::new(),
            context: HashMap::new(),
        };

        let suggestions = assistant.get_suggestions(&issue);

        assert!(suggestions.len() > 1);
        assert!(suggestions.contains(&"Original suggestion".to_string()));
    }
}
