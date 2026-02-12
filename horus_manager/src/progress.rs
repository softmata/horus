//! Progress indicators for HORUS CLI
//!
//! Provides spinners and progress bars for long-running operations.
//!
//! # Features
//! - Determinate progress bars with percentages and ETA
//! - Step-based progress tracking for multi-phase operations
//! - Simple dot-based animations

use console::style;
use indicatif::{HumanDuration, MultiProgress, ProgressBar, ProgressStyle};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Global quiet mode flag
static QUIET_MODE: AtomicBool = AtomicBool::new(false);

/// Set global quiet mode
pub fn set_quiet(quiet: bool) {
    QUIET_MODE.store(quiet, Ordering::SeqCst);
}

/// Check if quiet mode is enabled
pub fn is_quiet() -> bool {
    QUIET_MODE.load(Ordering::SeqCst)
}

// =============================================================================
// Status Indicators
// =============================================================================
// [+] - Success
// [-] - Error
// [!] - Warning
// [*] - Info
// =============================================================================

/// Simple dots spinner animation
pub const DOTS_SPINNER: &[&str] = &[
    ".   ", "..  ", "... ", "....", "... ", "..  ", ".   ", "    ",
];

/// Alternative braille spinner
pub const BRAILLE_SPINNER: &[&str] = &["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"];

/// Line spinner
pub const LINE_SPINNER: &[&str] = &["—", "\\", "|", "/"];

/// Success indicator
pub const STATUS_SUCCESS: &str = "[+]";

/// Warning indicator
pub const STATUS_WARNING: &str = "[!]";

/// Error indicator
pub const STATUS_ERROR: &str = "[-]";

/// Info indicator
pub const STATUS_INFO: &str = "[*]";

/// Simple dots for less intrusive operations
pub const DOTS: &[&str] = &[".", "..", "...", "....", "...", "..", "."];

/// Create a spinner for indefinite operations
pub fn spinner(message: &str) -> ProgressBar {
    if is_quiet() {
        return ProgressBar::hidden();
    }
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::default_spinner()
            .tick_strings(BRAILLE_SPINNER)
            .template("{spinner} {msg}")
            .unwrap(),
    );
    pb.set_message(message.to_string());
    pb.enable_steady_tick(Duration::from_millis(80));
    pb
}

/// Create a build spinner
pub fn build_spinner(message: &str) -> ProgressBar {
    spinner(message)
}

/// Create a run spinner
pub fn run_spinner(message: &str) -> ProgressBar {
    spinner(message)
}

/// Create a download/install spinner
pub fn download_spinner(message: &str) -> ProgressBar {
    spinner(message)
}

/// Create a thinking spinner
pub fn think_spinner(message: &str) -> ProgressBar {
    if is_quiet() {
        return ProgressBar::hidden();
    }
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::default_spinner()
            .tick_strings(DOTS_SPINNER)
            .template("{spinner} {msg}")
            .unwrap(),
    );
    pb.set_message(message.to_string());
    pb.enable_steady_tick(Duration::from_millis(150));
    pb
}

/// Create a subtle dots spinner (for less prominent operations)
pub fn dots_spinner(message: &str) -> ProgressBar {
    if is_quiet() {
        return ProgressBar::hidden();
    }
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::default_spinner()
            .tick_strings(DOTS)
            .template("{spinner:.cyan} {msg}")
            .unwrap(),
    );
    pb.set_message(message.to_string());
    pb.enable_steady_tick(Duration::from_millis(150));
    pb
}

/// Create a progress bar
pub fn progress_bar(total: u64, message: &str) -> ProgressBar {
    if is_quiet() {
        return ProgressBar::hidden();
    }
    let pb = ProgressBar::new(total);
    pb.set_style(
        ProgressStyle::default_bar()
            .template(
                "{msg}\n      [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({percent}%)",
            )
            .unwrap()
            .progress_chars("█░-"),
    );
    pb.set_message(message.to_string());
    pb
}

/// Create a progress bar for byte downloads
pub fn download_progress_bar(total: u64) -> ProgressBar {
    if is_quiet() {
        return ProgressBar::hidden();
    }
    let pb = ProgressBar::new(total);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("{msg}\n       [{elapsed_precise}] [{bar:40.green/black}] {bytes}/{total_bytes} ({bytes_per_sec})")
            .unwrap()
            .progress_chars("█░-")
    );
    pb
}

/// Finish a spinner with success
pub fn finish_success(pb: &ProgressBar, message: &str) {
    pb.set_style(ProgressStyle::default_spinner().template("{msg}").unwrap());
    pb.finish_with_message(format!("{} {}", STATUS_SUCCESS, style(message).green()));
}

/// Finish a spinner with warning
pub fn finish_warning(pb: &ProgressBar, message: &str) {
    pb.set_style(ProgressStyle::default_spinner().template("{msg}").unwrap());
    pb.finish_with_message(format!("{} {}", STATUS_WARNING, style(message).yellow()));
}

/// Finish a spinner with error
pub fn finish_error(pb: &ProgressBar, message: &str) {
    pb.set_style(ProgressStyle::default_spinner().template("{msg}").unwrap());
    pb.finish_with_message(format!("{} {}", STATUS_ERROR, style(message).red()));
}

/// Finish a spinner and clear it (no message)
pub fn finish_clear(pb: &ProgressBar) {
    pb.finish_and_clear();
}

/// A multi-progress manager for parallel operations
pub struct HorusMultiProgress {
    mp: MultiProgress,
    quiet: bool,
}

impl HorusMultiProgress {
    pub fn new(quiet: bool) -> Self {
        Self {
            mp: MultiProgress::new(),
            quiet,
        }
    }

    /// Add a spinner
    pub fn add_spinner(&self, message: &str) -> ProgressBar {
        if self.quiet {
            return ProgressBar::hidden();
        }
        let pb = spinner(message);
        self.mp.add(pb)
    }

    /// Add a build spinner
    pub fn add_build_spinner(&self, message: &str) -> ProgressBar {
        if self.quiet {
            return ProgressBar::hidden();
        }
        let pb = build_spinner(message);
        self.mp.add(pb)
    }

    /// Add a progress bar
    pub fn add_progress_bar(&self, total: u64, message: &str) -> ProgressBar {
        if self.quiet {
            return ProgressBar::hidden();
        }
        let pb = progress_bar(total, message);
        self.mp.add(pb)
    }
}

/// Helper to create a spinner that respects quiet mode
pub fn maybe_spinner(quiet: bool, message: &str) -> ProgressBar {
    if quiet {
        ProgressBar::hidden()
    } else {
        spinner(message)
    }
}

/// Helper to create a build spinner that respects quiet mode
pub fn maybe_build_spinner(quiet: bool, message: &str) -> ProgressBar {
    if quiet {
        ProgressBar::hidden()
    } else {
        build_spinner(message)
    }
}

/// Helper to create a run spinner that respects quiet mode
pub fn maybe_run_spinner(quiet: bool, message: &str) -> ProgressBar {
    if quiet {
        ProgressBar::hidden()
    } else {
        run_spinner(message)
    }
}

/// Helper to create a download spinner that respects quiet mode
pub fn maybe_download_spinner(quiet: bool, message: &str) -> ProgressBar {
    if quiet {
        ProgressBar::hidden()
    } else {
        download_spinner(message)
    }
}

// =============================================================================
// Legacy aliases for backward compatibility
// =============================================================================

/// Create a spinner (legacy alias)
pub fn robot_spinner(message: &str) -> ProgressBar {
    spinner(message)
}

/// Create a rolling spinner (legacy alias)
pub fn robot_roll_spinner(message: &str) -> ProgressBar {
    spinner(message)
}

/// Create a build spinner (legacy alias)
pub fn robot_build_spinner(message: &str) -> ProgressBar {
    build_spinner(message)
}

/// Create a run spinner (legacy alias)
pub fn robot_run_spinner(message: &str) -> ProgressBar {
    run_spinner(message)
}

/// Create a download spinner (legacy alias)
pub fn robot_download_spinner(message: &str) -> ProgressBar {
    download_spinner(message)
}

/// Create a thinking spinner (legacy alias)
pub fn robot_think_spinner(message: &str) -> ProgressBar {
    think_spinner(message)
}

/// Create a progress bar (legacy alias)
pub fn robot_progress_bar(total: u64, message: &str) -> ProgressBar {
    progress_bar(total, message)
}

/// Legacy multi-progress alias
pub type RobotMultiProgress = HorusMultiProgress;

// =============================================================================
// ENHANCED PROGRESS TRACKING WITH PERCENTAGES AND ETA
// =============================================================================

/// A progress tracker that estimates time remaining based on progress rate
pub struct ProgressTracker {
    pb: ProgressBar,
    start_time: Instant,
    total: u64,
    current: u64,
    message: String,
}

impl ProgressTracker {
    /// Create a new progress tracker with a determinate total
    pub fn new(total: u64, message: &str) -> Self {
        let pb = if is_quiet() {
            ProgressBar::hidden()
        } else {
            let pb = ProgressBar::new(total);
            pb.set_style(
                ProgressStyle::default_bar()
                    .template("{msg}\n      [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({percent}%) ETA: {eta}")
                    .unwrap()
                    .progress_chars("█░-")
            );
            pb.set_message(message.to_string());
            pb
        };

        Self {
            pb,
            start_time: Instant::now(),
            total,
            current: 0,
            message: message.to_string(),
        }
    }

    /// Create a tracker for byte-based operations (downloads, file processing)
    pub fn new_bytes(total_bytes: u64, message: &str) -> Self {
        let pb = if is_quiet() {
            ProgressBar::hidden()
        } else {
            let pb = ProgressBar::new(total_bytes);
            pb.set_style(
                ProgressStyle::default_bar()
                    .template("{msg}\n       [{elapsed_precise}] [{bar:40.green/black}] {bytes}/{total_bytes} ({percent}%) {bytes_per_sec} ETA: {eta}")
                    .unwrap()
                    .progress_chars("█░-")
            );
            pb.set_message(message.to_string());
            pb
        };

        Self {
            pb,
            start_time: Instant::now(),
            total: total_bytes,
            current: 0,
            message: message.to_string(),
        }
    }

    /// Update progress by incrementing the current position
    pub fn inc(&mut self, delta: u64) {
        self.current += delta;
        self.pb.inc(delta);
    }

    /// Set absolute progress position
    pub fn set_position(&mut self, pos: u64) {
        self.current = pos;
        self.pb.set_position(pos);
    }

    /// Get current percentage (0-100)
    pub fn percentage(&self) -> f64 {
        if self.total == 0 {
            100.0
        } else {
            (self.current as f64 / self.total as f64) * 100.0
        }
    }

    /// Get elapsed time
    pub fn elapsed(&self) -> Duration {
        self.start_time.elapsed()
    }

    /// Estimate time remaining based on current progress rate
    pub fn eta(&self) -> Option<Duration> {
        if self.current == 0 {
            return None;
        }

        let elapsed = self.start_time.elapsed();
        let rate = self.current as f64 / elapsed.as_secs_f64();

        if rate <= 0.0 {
            return None;
        }

        let remaining = self.total.saturating_sub(self.current);
        let eta_secs = remaining as f64 / rate;

        Some(Duration::from_secs_f64(eta_secs))
    }

    /// Update the message
    pub fn set_message(&mut self, message: &str) {
        self.message = message.to_string();
        self.pb.set_message(message.to_string());
    }

    /// Finish with success
    pub fn finish_success(&self, message: &str) {
        let elapsed = self.elapsed();
        self.pb
            .set_style(ProgressStyle::default_spinner().template("{msg}").unwrap());
        self.pb.finish_with_message(format!(
            "{} {} (completed in {})",
            STATUS_SUCCESS,
            style(message).green(),
            HumanDuration(elapsed)
        ));
    }

    /// Finish with error
    pub fn finish_error(&self, message: &str) {
        self.pb
            .set_style(ProgressStyle::default_spinner().template("{msg}").unwrap());
        self.pb
            .finish_with_message(format!("{} {}", STATUS_ERROR, style(message).red()));
    }

    /// Finish and clear
    pub fn finish_clear(&self) {
        self.pb.finish_and_clear();
    }

    /// Get the underlying progress bar for advanced usage
    pub fn progress_bar(&self) -> &ProgressBar {
        &self.pb
    }
}

// =============================================================================
// STEP-BASED PROGRESS FOR MULTI-PHASE OPERATIONS
// =============================================================================

/// A single step in a multi-step operation
pub struct Step {
    pub name: String,
    pub weight: u64, // Relative weight for progress calculation
}

/// Track progress through multiple discrete steps with overall percentage
pub struct StepProgress {
    pb: ProgressBar,
    steps: Vec<Step>,
    current_step: usize,
    total_weight: u64,
    completed_weight: u64,
    start_time: Instant,
    step_start_time: Instant,
}

impl StepProgress {
    /// Create a new step progress tracker
    pub fn new(steps: Vec<(&str, u64)>) -> Self {
        let steps: Vec<Step> = steps
            .into_iter()
            .map(|(name, weight)| Step {
                name: name.to_string(),
                weight,
            })
            .collect();

        let total_weight: u64 = steps.iter().map(|s| s.weight).sum();

        let pb = if is_quiet() {
            ProgressBar::hidden()
        } else {
            let pb = ProgressBar::new(100);
            pb.set_style(
                ProgressStyle::default_bar()
                    .template(
                        "{msg}\n      [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}% ETA: {eta}",
                    )
                    .unwrap()
                    .progress_chars("█░-"),
            );
            if !steps.is_empty() {
                pb.set_message(format!("[1/{}] {}", steps.len(), steps[0].name));
            }
            pb
        };

        let now = Instant::now();
        Self {
            pb,
            steps,
            current_step: 0,
            total_weight,
            completed_weight: 0,
            start_time: now,
            step_start_time: now,
        }
    }

    /// Advance to the next step
    pub fn next_step(&mut self) {
        if self.current_step < self.steps.len() {
            self.completed_weight += self.steps[self.current_step].weight;
            self.current_step += 1;
            self.step_start_time = Instant::now();

            let percent = if self.total_weight > 0 {
                (self.completed_weight * 100) / self.total_weight
            } else {
                100
            };

            self.pb.set_position(percent);

            if self.current_step < self.steps.len() {
                self.pb.set_message(format!(
                    "[{}/{}] {}",
                    self.current_step + 1,
                    self.steps.len(),
                    self.steps[self.current_step].name
                ));
            }
        }
    }

    /// Update progress within the current step (0.0 - 1.0)
    pub fn set_step_progress(&mut self, progress: f64) {
        if self.current_step < self.steps.len() {
            let step_weight = self.steps[self.current_step].weight;
            let step_contribution = (step_weight as f64 * progress.clamp(0.0, 1.0)) as u64;

            let total_progress = self.completed_weight + step_contribution;
            let percent = if self.total_weight > 0 {
                (total_progress * 100) / self.total_weight
            } else {
                100
            };

            self.pb.set_position(percent);
        }
    }

    /// Get current step index (0-based)
    pub fn current_step_index(&self) -> usize {
        self.current_step
    }

    /// Get total number of steps
    pub fn total_steps(&self) -> usize {
        self.steps.len()
    }

    /// Get overall percentage (0-100)
    pub fn percentage(&self) -> u64 {
        if self.total_weight > 0 {
            (self.completed_weight * 100) / self.total_weight
        } else {
            100
        }
    }

    /// Get elapsed time
    pub fn elapsed(&self) -> Duration {
        self.start_time.elapsed()
    }

    /// Finish with success
    pub fn finish_success(&self, message: &str) {
        let elapsed = self.elapsed();
        self.pb
            .set_style(ProgressStyle::default_spinner().template("{msg}").unwrap());
        self.pb.finish_with_message(format!(
            "{} {} (completed in {})",
            STATUS_SUCCESS,
            style(message).green(),
            HumanDuration(elapsed)
        ));
    }

    /// Finish with error
    pub fn finish_error(&self, message: &str) {
        self.pb
            .set_style(ProgressStyle::default_spinner().template("{msg}").unwrap());
        self.pb
            .finish_with_message(format!("{} {}", STATUS_ERROR, style(message).red()));
    }
}

// =============================================================================
// CONVENIENCE FUNCTIONS FOR COMMON OPERATIONS
// =============================================================================

/// Create a progress bar for build operations with N files
pub fn build_progress(total_files: u64) -> ProgressTracker {
    ProgressTracker::new(total_files, &format!("Building {} files", total_files))
}

/// Create a progress bar for download operations
pub fn download_progress(total_bytes: u64, name: &str) -> ProgressTracker {
    ProgressTracker::new_bytes(total_bytes, &format!("Downloading {}", name))
}

/// Create a step progress for typical build pipeline
pub fn build_pipeline_progress() -> StepProgress {
    StepProgress::new(vec![
        ("Parsing configuration", 1),
        ("Resolving dependencies", 2),
        ("Compiling sources", 5),
        ("Linking", 2),
    ])
}

/// Create a step progress for install operations
pub fn install_pipeline_progress() -> StepProgress {
    StepProgress::new(vec![
        ("Checking dependencies", 1),
        ("Downloading packages", 3),
        ("Extracting files", 2),
        ("Configuring", 1),
        ("Finalizing installation", 1),
    ])
}

/// Create a step progress with custom steps
pub fn custom_step_progress(steps: Vec<(&str, u64)>) -> StepProgress {
    StepProgress::new(steps)
}

/// Format duration in a human-readable way
pub fn format_duration(duration: Duration) -> String {
    let secs = duration.as_secs();
    if secs < 60 {
        format!("{}s", secs)
    } else if secs < 3600 {
        format!("{}m {}s", secs / 60, secs % 60)
    } else {
        format!("{}h {}m", secs / 3600, (secs % 3600) / 60)
    }
}

/// Format bytes in a human-readable way
pub fn format_bytes(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;

    if bytes < KB {
        format!("{} B", bytes)
    } else if bytes < MB {
        format!("{:.1} KB", bytes as f64 / KB as f64)
    } else if bytes < GB {
        format!("{:.1} MB", bytes as f64 / MB as f64)
    } else {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_spinner_creation() {
        let pb = spinner("Testing...");
        thread::sleep(Duration::from_millis(500));
        finish_success(&pb, "Test complete!");
    }

    #[test]
    fn test_quiet_mode() {
        let pb = maybe_spinner(true, "Should be hidden");
        assert!(pb.is_hidden());
    }

    #[test]
    fn test_progress_tracker_percentage() {
        let mut tracker = ProgressTracker::new(100, "Test");
        assert_eq!(tracker.percentage(), 0.0);

        tracker.set_position(50);
        assert_eq!(tracker.percentage(), 50.0);

        tracker.set_position(100);
        assert_eq!(tracker.percentage(), 100.0);
        tracker.finish_clear();
    }

    #[test]
    fn test_progress_tracker_inc() {
        let mut tracker = ProgressTracker::new(10, "Test");
        tracker.inc(3);
        assert_eq!(tracker.percentage(), 30.0);

        tracker.inc(7);
        assert_eq!(tracker.percentage(), 100.0);
        tracker.finish_clear();
    }

    #[test]
    fn test_step_progress() {
        let mut steps = StepProgress::new(vec![("Step 1", 1), ("Step 2", 2), ("Step 3", 1)]);

        assert_eq!(steps.current_step_index(), 0);
        assert_eq!(steps.total_steps(), 3);
        assert_eq!(steps.percentage(), 0);

        steps.next_step();
        assert_eq!(steps.current_step_index(), 1);
        assert_eq!(steps.percentage(), 25); // 1 out of 4 total weight

        steps.next_step();
        assert_eq!(steps.current_step_index(), 2);
        assert_eq!(steps.percentage(), 75); // 3 out of 4 total weight

        steps.next_step();
        assert_eq!(steps.percentage(), 100);
    }

    #[test]
    fn test_step_progress_within_step() {
        let mut steps = StepProgress::new(vec![("Step 1", 2), ("Step 2", 2)]);

        // Set 50% progress within step 1 (weight 2 out of 4 total)
        steps.set_step_progress(0.5);
        // 50% of 2 = 1, 1/4 = 25%
        assert_eq!(steps.percentage(), 0); // percentage() uses completed_weight only
    }

    #[test]
    fn test_format_duration() {
        assert_eq!(format_duration(Duration::from_secs(30)), "30s");
        assert_eq!(format_duration(Duration::from_secs(90)), "1m 30s");
        assert_eq!(format_duration(Duration::from_secs(3700)), "1h 1m");
    }

    #[test]
    fn test_format_bytes() {
        assert_eq!(format_bytes(500), "500 B");
        assert_eq!(format_bytes(1024), "1.0 KB");
        assert_eq!(format_bytes(1536), "1.5 KB");
        assert_eq!(format_bytes(1048576), "1.0 MB");
        assert_eq!(format_bytes(1073741824), "1.00 GB");
    }

    #[test]
    fn test_progress_tracker_eta() {
        let mut tracker = ProgressTracker::new(100, "Test");

        // No progress yet, no ETA
        assert!(tracker.eta().is_none());

        // Simulate some progress
        thread::sleep(Duration::from_millis(100));
        tracker.set_position(10);

        // Should have an ETA now
        let eta = tracker.eta();
        assert!(eta.is_some());

        tracker.finish_clear();
    }
}
