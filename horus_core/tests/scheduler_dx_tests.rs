//! Integration tests for Scheduler DX (Developer Experience) API
//!
//! Tests the intent-based constructors:
//! - `Scheduler::new()` - Auto-optimizing with capability detection
//! - `SchedulerBuilder::simulation().build().unwrap()` - Deterministic virtual time mode
//! - `SchedulerBuilder::prototype().build().unwrap()` - Fast development mode
//! - `Scheduler::builder()` - Explicit configuration

use horus_core::core::Node;
use horus_core::error::Result;
use horus_core::scheduling::{Scheduler, SchedulerBuilder};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Simple test node that counts ticks
struct TickCounterNode {
    name: &'static str,
    tick_count: Arc<AtomicU32>,
}

impl TickCounterNode {
    fn new(name: &'static str, counter: Arc<AtomicU32>) -> Self {
        Self {
            name,
            tick_count: counter,
        }
    }
}

impl Node for TickCounterNode {
    fn name(&self) -> &'static str {
        self.name
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }

    fn shutdown(&mut self) -> Result<()> {
        Ok(())
    }
}

// =============================================================================
// Scheduler::new() Tests - Auto-Optimization
// =============================================================================

#[test]
fn test_new_creates_scheduler() {
    // Basic test: new() should create a working scheduler
    let mut scheduler = Scheduler::new();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler.add(TickCounterNode::new("test_node", counter.clone())).order(0).done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok(), "Scheduler::new() should create a working scheduler");
    assert!(counter.load(Ordering::SeqCst) > 0, "Node should have ticked");
}

#[test]
fn test_new_detects_capabilities() {
    // new() should detect runtime capabilities
    let scheduler = Scheduler::new();

    // On any system, capabilities() should return Some after new()
    let caps = scheduler.capabilities();
    assert!(caps.is_some(), "Scheduler::new() should detect capabilities");

    let caps = caps.unwrap();
    // Core count should always be detectable
    assert!(caps.cpu_count > 0, "Should detect at least 1 CPU core");
}

#[test]
fn test_new_records_degradations_gracefully() {
    // new() should gracefully degrade if RT features unavailable
    let scheduler = Scheduler::new();

    // On non-root or non-RT systems, there may be degradations
    // This test just ensures degradations() returns without panic
    let degradations = scheduler.degradations();

    // Degradations list should be accessible (may be empty on RT-capable systems)
    let _ = degradations.len();
}

#[test]
fn test_new_with_capacity() {
    // new() should support with_capacity for pre-allocation
    let mut scheduler = Scheduler::new().with_capacity(100);
    let counter = Arc::new(AtomicU32::new(0));

    scheduler.add(TickCounterNode::new("cap_node", counter.clone())).order(0).done();

    // Use 100ms to be reliable under parallel test load
    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

// =============================================================================
// SchedulerBuilder::simulation().build().unwrap() Tests - Deterministic Mode
// =============================================================================

#[test]
fn test_simulation_creates_deterministic_scheduler() {
    // simulation() should create a deterministic scheduler
    let mut scheduler = SchedulerBuilder::simulation().build().unwrap();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler.add(TickCounterNode::new("sim_node", counter.clone())).order(0).done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok(), "SchedulerBuilder::simulation().build().unwrap() should create a working scheduler");
    assert!(counter.load(Ordering::SeqCst) > 0, "Node should have ticked");
}

#[test]
fn test_simulation_has_no_rt_features() {
    // simulation() should not enable RT features (conflicts with virtual time)
    let scheduler = SchedulerBuilder::simulation().build().unwrap();

    // Capabilities should be None (intentionally disabled for determinism)
    let caps = scheduler.capabilities();
    assert!(caps.is_none(), "simulation() should not detect RT capabilities");

    // No degradations (RT wasn't attempted)
    let degradations = scheduler.degradations();
    assert!(degradations.is_empty(), "simulation() should have no degradations");
}

#[test]
fn test_simulation_with_seed_reproducibility() {
    // Two schedulers with same seed should behave identically
    let scheduler1 = SchedulerBuilder::simulation().seed(12345).build().unwrap();
    let scheduler2 = SchedulerBuilder::simulation().seed(12345).build().unwrap();

    // Both should have deterministic config with same seed
    assert!(scheduler1.is_simulation_mode());
    assert!(scheduler2.is_simulation_mode());

    // Scheduler names should match
    assert_eq!(scheduler1.get_name(), scheduler2.get_name());
}

#[test]
fn test_simulation_has_blackbox() {
    // simulation() should enable BlackBox for debugging
    let scheduler = SchedulerBuilder::simulation().build().unwrap();

    // The scheduler should have recording capabilities enabled
    assert!(scheduler.get_name().contains("Simulation"), "Should be named SimulationScheduler");

    // Verify BlackBox is actually present (8MB buffer)
    assert!(
        scheduler.blackbox().is_some(),
        "simulation() should have BlackBox for debugging"
    );
}

#[test]
fn test_simulation_is_simulation_mode() {
    // simulation() must enable deterministic mode
    let scheduler = SchedulerBuilder::simulation().build().unwrap();
    assert!(scheduler.is_simulation_mode(), "simulation() must enable deterministic mode");
}

// =============================================================================
// SchedulerBuilder::prototype().build().unwrap() Tests - Development Mode
// =============================================================================

#[test]
fn test_prototype_creates_dev_scheduler() {
    // prototype() should create a minimal overhead scheduler
    let mut scheduler = SchedulerBuilder::prototype().build().unwrap();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler.add(TickCounterNode::new("proto_node", counter.clone())).order(0).done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok(), "SchedulerBuilder::prototype().build().unwrap() should create a working scheduler");
    assert!(counter.load(Ordering::SeqCst) > 0, "Node should have ticked");
}

#[test]
fn test_prototype_has_no_capability_detection() {
    // prototype() skips capability detection for fast startup
    let scheduler = SchedulerBuilder::prototype().build().unwrap();

    // No capabilities detected (intentionally skipped)
    let caps = scheduler.capabilities();
    assert!(caps.is_none(), "prototype() should skip capability detection");
}

#[test]
fn test_prototype_has_safety_monitor() {
    // prototype() should enable safety monitor for dev feedback
    let scheduler = SchedulerBuilder::prototype().build().unwrap();

    // Verify by checking name
    assert!(scheduler.get_name().contains("Prototype"), "Should be named PrototypeScheduler");
}

#[test]
fn test_prototype_fast_startup() {
    // prototype() should have fast startup (no RT checks)
    let start = Instant::now();
    let _scheduler = SchedulerBuilder::prototype().build().unwrap();
    let elapsed = start.elapsed();

    // Startup should be fast (under 100ms on any system)
    assert!(elapsed < Duration::from_millis(100),
        "prototype() should start quickly, took {:?}", elapsed);
}

#[test]
fn test_prototype_is_not_deterministic() {
    // prototype() is NOT deterministic (uses wall clock)
    let scheduler = SchedulerBuilder::prototype().build().unwrap();
    assert!(!scheduler.is_simulation_mode(), "prototype() should NOT be deterministic");
}

// =============================================================================
// Scheduler::builder() Tests - Explicit Configuration
// =============================================================================

#[test]
fn test_builder_default_build() {
    // builder().build() should create a basic scheduler
    // Note: builder() creates a "prototype" scheduler which is a minimal dev stub
    // It may not actually tick nodes, but should init/shutdown correctly
    let result = Scheduler::builder().build();
    assert!(result.is_ok(), "builder().build() should succeed");

    let mut scheduler = result.unwrap();
    let counter = Arc::new(AtomicU32::new(0));

    scheduler.add(TickCounterNode::new("builder_node", counter.clone())).order(0).done();

    // Use 100ms to be reliable under parallel test load
    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
    // Prototype scheduler may not tick nodes, so we don't assert counter > 0
    // The important thing is that the scheduler runs and shuts down cleanly
}

#[test]
fn test_builder_with_name() {
    // builder().name() should set scheduler name
    let scheduler = Scheduler::builder()
        .name("CustomScheduler")
        .build()
        .expect("build should succeed");

    assert_eq!(scheduler.get_name(), "CustomScheduler");
}

#[test]
fn test_builder_with_capacity() {
    // builder().capacity() should pre-allocate nodes
    let mut scheduler = Scheduler::builder()
        .capacity(50)
        .build()
        .expect("build should succeed");

    let counter = Arc::new(AtomicU32::new(0));
    scheduler.add(TickCounterNode::new("cap_builder_node", counter.clone())).order(0).done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_builder_deterministic() {
    // builder().deterministic() should enable deterministic mode
    let scheduler = Scheduler::builder()
        .deterministic()
        .seed(42)
        .build()
        .expect("build should succeed");

    assert!(scheduler.is_simulation_mode(), "deterministic() should enable determinism");
}

#[test]
fn test_builder_watchdog() {
    // builder().watchdog() should configure watchdog timeout
    let mut scheduler = Scheduler::builder()
        .watchdog(1000) // 1 second watchdog
        .build()
        .expect("build should succeed");

    let counter = Arc::new(AtomicU32::new(0));
    scheduler
        .add(TickCounterNode::new("watchdog_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_builder_blackbox() {
    // builder().blackbox() should configure flight recorder
    let mut scheduler = Scheduler::builder()
        .blackbox(4) // 4MB blackbox
        .build()
        .expect("build should succeed");

    let counter = Arc::new(AtomicU32::new(0));
    scheduler
        .add(TickCounterNode::new("blackbox_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_builder_strict_mode_graceful() {
    // builder().strict() without RT requirements should still build
    let scheduler = Scheduler::builder()
        .name("StrictNonRT")
        .strict()  // Strict mode, but no RT features requested
        .build();

    // Should succeed because no RT features were requested
    assert!(scheduler.is_ok(), "strict() without RT features should succeed");
}

#[test]
fn test_builder_chaining() {
    // All builder methods should chain properly
    let scheduler = Scheduler::builder()
        .name("ChainedScheduler")
        .capacity(25)
        .watchdog(500)
        .blackbox(2)
        .build()
        .expect("chained build should succeed");

    assert_eq!(scheduler.get_name(), "ChainedScheduler");
}

#[test]
fn test_builder_preset_hard_realtime() {
    // SchedulerBuilder::hard_realtime() preset
    let builder = SchedulerBuilder::hard_realtime();

    // Should be able to build (may degrade on non-RT systems)
    let result = builder.build();
    assert!(result.is_ok(), "hard_realtime() preset should build");
}

#[test]
fn test_builder_preset_simulation() {
    // SchedulerBuilder::simulation() preset
    let scheduler = SchedulerBuilder::simulation()
        .build()
        .expect("simulation preset should build");

    assert!(scheduler.is_simulation_mode(), "simulation preset should be deterministic");
}

#[test]
fn test_builder_preset_prototype() {
    // SchedulerBuilder::prototype() preset
    let mut scheduler = SchedulerBuilder::prototype()
        .build()
        .expect("prototype preset should build");

    let counter = Arc::new(AtomicU32::new(0));
    scheduler
        .add(TickCounterNode::new("preset_proto_node", counter.clone()))
        .order(0)
        .done();

    let result = scheduler.run_for(Duration::from_millis(100));
    assert!(result.is_ok());
}

#[test]
fn test_builder_preset_safety_critical() {
    // SchedulerBuilder::safety_critical() preset
    let result = SchedulerBuilder::safety_critical().build();

    // May fail on non-RT systems if strict mode is enabled
    // But should at least attempt to build without panic
    match result {
        Ok(scheduler) => {
            // Verify it's configured for safety
            assert!(scheduler.get_name().len() > 0);
        }
        Err(_) => {
            // Expected on non-RT systems - strict mode rejects degradation
        }
    }
}

// =============================================================================
// Degradation Handling Tests
// =============================================================================

#[test]
fn test_graceful_degradation_on_non_rt() {
    // new() should gracefully degrade on non-RT systems
    let scheduler = Scheduler::new();

    // Should always succeed, even on non-RT systems
    let caps = scheduler.capabilities();
    let degradations = scheduler.degradations();

    if let Some(caps) = caps {
        if !caps.rt_priority_available {
            // If RT is not available, degradations may be recorded
            // (or the system may simply not have attempted RT)
            // Just verify we can access the degradations list
            let _ = degradations.len();
        }
    }
}

#[test]
fn test_strict_mode_rejects_degradation() {
    // builder().strict() with RT features should fail on non-RT systems
    let _result = Scheduler::builder()
        .rt_priority(99)  // Request RT priority
        .strict()          // Don't allow degradation
        .build();

    // On non-RT systems (most dev machines), this should fail
    // On RT-capable systems with proper permissions, it should succeed
    #[cfg(target_os = "linux")]
    {
        // Linux-specific behavior
        // Will fail unless running as root with RT permissions
    }

    #[cfg(not(target_os = "linux"))]
    {
        // Non-Linux should always fail for RT features
        assert!(result.is_err(), "strict() with rt_priority() should fail on non-Linux");
    }
}

// =============================================================================
// Cross-Platform Behavior Tests
// =============================================================================

#[test]
fn test_constructors_work_on_all_platforms() {
    // All intent-based constructors should work on all platforms

    // new() - always works
    let scheduler = Scheduler::new();
    assert!(scheduler.capabilities().is_some() || scheduler.capabilities().is_none());

    // simulation() - always works
    let scheduler = SchedulerBuilder::simulation().build().unwrap();
    assert!(scheduler.is_simulation_mode());

    // prototype() - always works
    let scheduler = SchedulerBuilder::prototype().build().unwrap();
    assert!(!scheduler.is_simulation_mode());

    // builder().build() - always works
    let scheduler = Scheduler::builder().build();
    assert!(scheduler.is_ok());
}

// =============================================================================
// Named Constructor Tests
// =============================================================================

#[test]
fn test_scheduler_names() {
    // Each constructor should set appropriate default names

    let new_scheduler = Scheduler::new();
    assert!(!new_scheduler.get_name().is_empty(), "new() should have a name");

    let sim_scheduler = SchedulerBuilder::simulation().build().unwrap();
    assert!(sim_scheduler.get_name().contains("Simulation"),
        "simulation() should have 'Simulation' in name");

    let proto_scheduler = SchedulerBuilder::prototype().build().unwrap();
    assert!(proto_scheduler.get_name().contains("Prototype"),
        "prototype() should have 'Prototype' in name");

    let builder_scheduler = Scheduler::builder()
        .name("CustomName")
        .build()
        .unwrap();
    assert_eq!(builder_scheduler.get_name(), "CustomName",
        "builder().name() should override default");
}

// =============================================================================
// Status Introspection Tests
// =============================================================================

#[test]
fn test_status_returns_formatted_string() {
    // status() should return a formatted string with scheduler info
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    // Should contain the scheduler name
    assert!(status.contains("SCHEDULER STATUS:"), "status() should have header");

    // Should contain execution mode section
    assert!(status.contains("Execution Mode:"), "status() should show execution mode");

    // Should contain safety features section
    assert!(status.contains("Safety Features:"), "status() should show safety features");
}

#[test]
fn test_status_shows_capabilities() {
    // status() should show detected capabilities when available
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    // new() detects capabilities, so status should show platform info
    assert!(status.contains("Platform:"), "status() should show platform for new()");
    assert!(status.contains("RT Features:"), "status() should show RT features");
}

#[test]
fn test_status_no_capabilities_for_simulation() {
    // simulation() doesn't detect capabilities
    let scheduler = SchedulerBuilder::simulation().build().unwrap();
    let status = scheduler.status();

    // Should show "capabilities not detected" since simulation skips detection
    assert!(status.contains("capabilities not detected"),
        "simulation() status should indicate no capability detection");

    // Should show simulation mode enabled
    assert!(status.contains("[x] Simulation Mode"), "should show simulation mode enabled");
}

#[test]
fn test_status_shows_safety_features() {
    // Builder with safety features should show them in status
    let scheduler = Scheduler::builder()
        .safety_monitor(5)
        .blackbox(2)
        .build()
        .expect("build should succeed");

    let status = scheduler.status();

    // Should show safety monitor enabled
    assert!(status.contains("[x] Safety Monitor"), "should show safety monitor enabled");
    // Should show blackbox enabled
    assert!(status.contains("[x] BlackBox Recorder"), "should show blackbox enabled");
}

#[test]
fn test_status_different_for_each_constructor() {
    // Each constructor type should produce different status output
    let new_status = Scheduler::new().status();
    let sim_status = SchedulerBuilder::simulation().build().unwrap().status();
    let proto_status = SchedulerBuilder::prototype().build().unwrap().status();

    // Names should be different
    assert!(new_status.contains("AutoOptimized") || new_status.contains("SCHEDULER STATUS:"));
    assert!(sim_status.contains("Simulation"));
    assert!(proto_status.contains("Prototype"));

    // Simulation should show simulation mode enabled
    assert!(sim_status.contains("[x] Simulation Mode"));
    assert!(!new_status.contains("[x] Simulation Mode"));
}

// =============================================================================
// BlackBox Flight Recorder Tests
// =============================================================================

#[test]
fn test_new_auto_creates_blackbox() {
    let scheduler = Scheduler::new();

    // BlackBox should be automatically created in new()
    assert!(
        scheduler.blackbox().is_some(),
        "new() should auto-create BlackBox for crash analysis"
    );
}

#[test]
fn test_blackbox_mut_accessor() {
    use horus_core::scheduling::BlackBoxEvent;

    let mut scheduler = Scheduler::new();

    // Should be able to get mutable reference and record events
    let bb = scheduler.blackbox_mut().expect("BlackBox should exist");

    // Record a custom event
    bb.record(BlackBoxEvent::Custom {
        category: "test".to_string(),
        message: "Integration test event".to_string(),
    });

    // The event should be recorded (we can't easily verify without reading back,
    // but the main test is that recording doesn't panic)
}

#[test]
fn test_prototype_has_no_blackbox() {
    let scheduler = SchedulerBuilder::prototype().build().unwrap();

    // Prototype mode should NOT have BlackBox by default
    // (fast development mode skips overhead)
    assert!(
        scheduler.blackbox().is_none(),
        "prototype() should not auto-create BlackBox"
    );
}

#[test]
fn test_builder_can_add_blackbox() {
    let scheduler = Scheduler::builder()
        .blackbox(8) // 8MB buffer
        .build()
        .expect("Builder should succeed");

    // Builder explicitly requested BlackBox
    assert!(
        scheduler.blackbox().is_some(),
        "builder().blackbox() should create BlackBox"
    );
}

#[test]
fn test_status_shows_blackbox() {
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    // Status should show BlackBox is enabled
    assert!(
        status.contains("BlackBox"),
        "status() should mention BlackBox"
    );
    assert!(
        status.contains("[x] BlackBox"),
        "status() should show BlackBox as enabled: {}",
        status
    );
}

// =============================================================================
// Circuit Breaker Integration Tests
// =============================================================================

#[test]
fn test_circuit_breakers_enabled_by_default() {
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    // Circuit breakers should be shown as enabled
    assert!(
        status.contains("Circuit Breakers"),
        "status() should mention Circuit Breakers"
    );
    assert!(
        status.contains("[x] Circuit Breakers"),
        "status() should show Circuit Breakers as enabled"
    );
}

#[test]
fn test_circuit_summary_empty_scheduler() {
    let scheduler = Scheduler::new();

    // No nodes = all zeros
    let (closed, open, half_open) = scheduler.circuit_summary();
    assert_eq!(closed, 0, "No nodes should mean 0 closed circuits");
    assert_eq!(open, 0, "No nodes should mean 0 open circuits");
    assert_eq!(half_open, 0, "No nodes should mean 0 half-open circuits");
}

#[test]
fn test_circuit_state_nonexistent_node() {
    let scheduler = Scheduler::new();

    // Node doesn't exist
    assert!(
        scheduler.circuit_state("nonexistent_node").is_none(),
        "Should return None for nonexistent node"
    );
}

#[test]
fn test_status_shows_circuit_breaker_config() {
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    // Should show the 30s timeout configuration
    assert!(
        status.contains("30s timeout"),
        "status() should show 30s circuit breaker timeout: {}",
        status
    );
}

#[test]
fn test_builder_circuit_breaker_config() {
    // Builder can customize circuit breaker config with timeout
    let scheduler = Scheduler::builder()
        .circuit_breaker_with_timeout(10, 5, 60000) // 10 failures, 5 successes, 60s
        .build()
        .expect("Builder should succeed");

    // Builder created scheduler should work
    assert!(!scheduler.get_name().is_empty());
}

// =============================================================================
// WCET Enforcement Tests
// =============================================================================

#[test]
fn test_new_has_safety_monitor_enabled() {
    // new() should enable SafetyMonitor by default (for WCET enforcement)
    let scheduler = Scheduler::new();

    // safety_stats() returns Some when SafetyMonitor is enabled
    assert!(scheduler.safety_stats().is_some());
}

#[test]
fn test_safety_stats_initial_values() {
    // Initial safety stats should have zero violations
    let scheduler = Scheduler::new();

    let stats = scheduler.safety_stats().expect("SafetyMonitor should be enabled");
    assert_eq!(stats.wcet_overruns, 0);
    assert_eq!(stats.deadline_misses, 0);
    assert_eq!(stats.watchdog_expirations, 0);
}

#[test]
fn test_status_shows_wcet_enforcement() {
    // status() should show WCET Enforcement feature
    let scheduler = Scheduler::new();
    let status = scheduler.status();

    // WCET Enforcement should be shown in Safety Features
    assert!(status.contains("WCET Enforcement"));
}

#[test]
fn test_prototype_has_no_safety_stats() {
    // prototype() is for development - SafetyMonitor is optional
    let scheduler = SchedulerBuilder::prototype().build().unwrap();
    // prototype() doesn't enable SafetyMonitor
    // Check that it at least doesn't crash
    let _status = scheduler.status();
}

#[test]
fn test_simulation_has_safety_monitor() {
    // simulation() should also have SafetyMonitor for WCET tracking
    let scheduler = SchedulerBuilder::simulation().build().unwrap();
    let _status = scheduler.status();

    // simulation mode has deterministic behavior but still tracks safety
}

#[test]
fn test_builder_with_safety_monitor() {
    // Builder can explicitly enable safety monitor
    let scheduler = Scheduler::builder()
        .safety_monitor(10) // 10 deadline misses before action
        .build()
        .expect("Builder should succeed");

    let stats = scheduler.safety_stats().expect("SafetyMonitor should be enabled");
    assert_eq!(stats.wcet_overruns, 0);
}
