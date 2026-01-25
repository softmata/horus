/// Example showing unified API vs convenience constructors
///
/// Both approaches work identically - pick based on preference:
/// - Builder pattern: Maximum flexibility
/// - Convenience constructors: Quick common patterns
use horus_core::scheduling::{config::SchedulerConfig, Scheduler};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== HORUS Scheduler API Patterns ===\n");

    // ========================================================================
    // Pattern 1: Builder Pattern (Unified API) - Maximum Flexibility
    // ========================================================================
    println!("Pattern 1: Builder Pattern (Recommended)");
    println!("-----------------------------------------");

    let scheduler = Scheduler::new()
        .with_config(SchedulerConfig::hard_realtime())
        .with_capacity(128)
        .disable_learning()
        .with_safety_monitor(3)
        .with_name("CustomRTScheduler");

    println!("[OK] Created with builder pattern");
    println!("  Config: hard_realtime preset");
    println!("  Capacity: 128 nodes pre-allocated");
    println!("  Learning: DISABLED");
    println!("  Safety: ENABLED (max 3 misses)\n");

    // OS integration (requires root/capabilities)
    match scheduler.set_os_priority(50) {
        Ok(_) => println!("[OK] Real-time priority: 50 (SCHED_FIFO)"),
        Err(e) => println!("[WARNING] RT priority failed (need root): {}", e),
    }

    match scheduler.pin_to_cpu(0) {
        Ok(_) => println!("[OK] Pinned to CPU core 0"),
        Err(e) => println!("[WARNING] CPU pinning failed: {}", e),
    }

    match scheduler.lock_memory() {
        Ok(_) => println!("[OK] Memory locked (no page faults)"),
        Err(e) => println!("[WARNING] Memory locking failed (need CAP_IPC_LOCK): {}", e),
    }

    println!();

    // ========================================================================
    // Pattern 2: Convenience Constructor - Quick Start
    // ========================================================================
    println!("Pattern 2: Hard Realtime Configuration");
    println!("--------------------------------------");

    let scheduler2 = Scheduler::new()
        .with_config(SchedulerConfig::hard_realtime())
        .disable_learning();

    println!("[OK] Created with hard_realtime() config");
    println!("  (Uses builder pattern with preset config)");
    println!("  Equivalent to Pattern 1\n");

    // Same OS integration
    let _ = scheduler2.set_os_priority(50);
    let _ = scheduler2.pin_to_cpu(0);
    let _ = scheduler2.lock_memory();

    // ========================================================================
    // Pattern 3: Custom Composition - Mix Features
    // ========================================================================
    println!("Pattern 3: Custom Composition");
    println!("-----------------------------");

    let mut config = SchedulerConfig::hard_realtime();
    config.timing.global_rate_hz = 2000.0; // Override to 2kHz

    let _scheduler3 = Scheduler::new()
        .with_config(config)
        .with_capacity(64)
        // Note: Learning is ENABLED (default) - mixed workload
        .with_safety_monitor(5) // More tolerant (5 misses)
        .with_name("MixedWorkload");

    println!("[OK] Custom composition:");
    println!("  Base: hard_realtime preset");
    println!("  Override: 2kHz rate (was 1kHz)");
    println!("  Learning: ENABLED (default)");
    println!("  Safety: 5 misses allowed\n");

    // ========================================================================
    // Pattern 4: Deterministic (No OS Features)
    // ========================================================================
    println!("Pattern 4: Deterministic (Simulation)");
    println!("--------------------------------------");

    let _scheduler4 = Scheduler::new()
        .disable_learning()
        .with_name("SimScheduler");

    println!("[OK] Deterministic scheduler:");
    println!("  Learning: DISABLED");
    println!("  No OS integration needed");
    println!("  Suitable for testing/simulation\n");

    // ========================================================================
    // Pattern 5: Standard (Default Behavior)
    // ========================================================================
    println!("Pattern 5: Standard (Default)");
    println!("------------------------------");

    let _scheduler5 = Scheduler::new();

    println!("[OK] Standard scheduler:");
    println!("  All defaults");
    println!("  Learning: ENABLED");
    println!("  Adapts to workload dynamically\n");

    println!("=== Comparison Summary ===");
    println!();
    println!("Builder Pattern:         Flexible, compose any features");
    println!("Convenience Constructor: Quick start, common patterns");
    println!("OS Integration:          Always explicit, requires permissions");
    println!("Configuration Presets:   Robot-specific optimizations");
    println!();
    println!("All patterns use the same backend - choose based on preference!");

    Ok(())
}
