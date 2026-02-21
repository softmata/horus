/// Example showing the HORUS Scheduler API patterns
///
/// The API uses progressive disclosure:
/// - `Scheduler::new()` — lightweight, no syscalls
/// - `SchedulerConfig` fields opt in to features: RT, BlackBox, safety monitor
/// - Presets bundle common configurations: `deploy()`, `safety_critical()`, `deterministic()`
use horus_core::scheduling::{config::SchedulerConfig, Scheduler};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== HORUS Scheduler API Patterns ===\n");

    // ========================================================================
    // Pattern 1: Lightweight (development/prototyping)
    // ========================================================================
    println!("Pattern 1: Lightweight (development)");
    println!("------------------------------------");

    let _scheduler = Scheduler::new(); // No syscalls, no allocations beyond struct

    println!("[OK] Lightweight scheduler — no RT, no BlackBox");
    println!("  Best for: rapid iteration, unit tests\n");

    // ========================================================================
    // Pattern 2: Production Deployment
    // ========================================================================
    println!("Pattern 2: Production Deployment");
    println!("--------------------------------");

    let scheduler = Scheduler::deploy(); // RT + BlackBox + profiling

    println!("[OK] Deploy preset — RT features + 16MB BlackBox");
    if let Some(caps) = scheduler.capabilities() {
        println!("  RT support: {}", caps.has_rt_support());
        println!("  Can lock memory: {}", caps.can_lock_memory());
    }
    for deg in scheduler.degradations() {
        println!("  [WARN] {}: {}", deg.feature, deg.reason);
    }
    println!();

    // ========================================================================
    // Pattern 3: Config — compose exactly what you need
    // ========================================================================
    println!("Pattern 3: Config Composition");
    println!("-----------------------------");

    let mut config = SchedulerConfig::standard();
    config.realtime.rt_scheduling_class = true;
    config.monitoring.black_box_enabled = true;
    config.monitoring.black_box_size_mb = 8;
    let _scheduler = Scheduler::new()
        .with_config(config)
        .tick_hz(1000.0)   // 1kHz control loop
        .with_name("CustomRobot");

    println!("[OK] Custom config composition");
    println!("  RT: enabled, BlackBox: 8MB, Rate: 1kHz\n");

    // ========================================================================
    // Pattern 4: Config Preset + extras
    // ========================================================================
    println!("Pattern 4: Config Preset + Extras");
    println!("---------------------------------");

    let mut config = SchedulerConfig::hard_realtime();
    config.realtime.safety_monitor = true;
    config.realtime.max_deadline_misses = 3;
    let _scheduler = Scheduler::new()
        .with_config(config)
        .with_capacity(128)
        .with_name("HardRT");

    println!("[OK] hard_realtime config preset + safety monitor");
    println!("  WCET enforcement, 10ms watchdog, panic on deadline miss\n");

    // ========================================================================
    // Pattern 5: Named Presets
    // ========================================================================
    println!("Pattern 5: Named Presets");
    println!("------------------------");

    let _s1 = Scheduler::safety_critical();
    println!("[OK] safety_critical — sequential, WCET, watchdog");

    let _s2 = Scheduler::high_performance();
    println!("[OK] high_performance — parallel, 10kHz");

    let _s3 = Scheduler::hard_realtime();
    println!("[OK] hard_realtime — strict deadlines");

    let _s4 = Scheduler::deterministic();
    println!("[OK] deterministic — reproducible execution\n");

    // ========================================================================
    // Pattern 6: Manual OS Integration (still available)
    // ========================================================================
    println!("Pattern 6: Manual OS Integration");
    println!("--------------------------------");

    let scheduler = Scheduler::new();

    match scheduler.set_os_priority(50) {
        Ok(_) => println!("[OK] RT priority 50 (SCHED_FIFO)"),
        Err(e) => println!("[SKIP] RT priority: {}", e),
    }

    match scheduler.pin_to_cpu(0) {
        Ok(_) => println!("[OK] Pinned to CPU 0"),
        Err(e) => println!("[SKIP] CPU pin: {}", e),
    }

    match scheduler.lock_memory() {
        Ok(_) => println!("[OK] Memory locked"),
        Err(e) => println!("[SKIP] Memory lock: {}", e),
    }

    println!();

    // ========================================================================
    // Summary
    // ========================================================================
    println!("=== API Summary ===");
    println!();
    println!("  Scheduler::new()              Lightweight, no syscalls");
    println!("  Scheduler::deploy()           RT + BlackBox (production)");
    println!("  Scheduler::safety_critical()  WCET + watchdog + sequential");
    println!("  Scheduler::high_performance() Parallel + 10kHz");
    println!("  Scheduler::hard_realtime()    Strict deadlines");
    println!("  Scheduler::deterministic()    Reproducible execution");
    println!();
    println!("  .with_config(C)   Apply a SchedulerConfig preset");
    println!("  .tick_hz(Hz)      Set global tick rate");
    println!("  .with_name(S)     Name the scheduler instance");
    println!("  .with_capacity(N) Pre-allocate node slots");

    Ok(())
}
