/// Example showing the HORUS Scheduler API patterns
///
/// The API uses progressive disclosure:
/// - `Scheduler::new()` — lightweight, no syscalls
/// - Builder methods opt in to features: `.realtime()`, `.with_blackbox()`, `.tick_hz()`
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
    // Pattern 3: Builder — compose exactly what you need
    // ========================================================================
    println!("Pattern 3: Builder Composition");
    println!("------------------------------");

    let _scheduler = Scheduler::new()
        .realtime()        // RT priority + memory lock + CPU pin
        .with_blackbox(8)  // 8MB flight recorder
        .tick_hz(1000.0)   // 1kHz control loop
        .with_name("CustomRobot");

    println!("[OK] Custom builder composition");
    println!("  RT: enabled, BlackBox: 8MB, Rate: 1kHz\n");

    // ========================================================================
    // Pattern 4: Config Preset (advanced)
    // ========================================================================
    println!("Pattern 4: Config Preset");
    println!("------------------------");

    let _scheduler = Scheduler::new()
        .with_config(SchedulerConfig::hard_realtime())
        .with_capacity(128)
        .with_safety_monitor(3)
        .with_name("HardRT");

    println!("[OK] hard_realtime config preset");
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
    println!("  .realtime()       Opt-in RT priority + memory lock + CPU pin");
    println!("  .with_blackbox(N) Opt-in N MB flight recorder");
    println!("  .tick_hz(Hz)      Set global tick rate");
    println!("  .with_config(C)   Apply a SchedulerConfig preset");

    Ok(())
}
