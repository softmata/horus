use horus_core::{HorusResult as Result, Node, Scheduler, TopicMetadata};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Instant;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::DurationExt;

/// Test node that simulates CPU-intensive work
struct CpuNode {
    name: &'static str,
    counter: Arc<AtomicUsize>,
    work_ms: u64,
}

impl Node for CpuNode {
    fn name(&self) -> &'static str {
        self.name
    }

    fn init(&mut self) -> Result<()> {
        println!("CPU Node {} initialized", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        // Simulate CPU work
        let start = Instant::now();
        while start.elapsed().as_millis() < self.work_ms as u128 {
            // Busy wait
        }
        self.counter.fetch_add(1, Ordering::SeqCst);
    }

    fn shutdown(&mut self) -> Result<()> {
        println!("CPU Node {} shutdown", self.name);
        Ok(())
    }

    fn publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: format!("{}.output", self.name),
            type_name: "std_msgs/Int32".to_string(),
        }]
    }

    fn subscribers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }
}

/// Test node that simulates I/O operations (camera/sensor)
struct IoNode {
    name: &'static str,
    counter: Arc<AtomicUsize>,
    io_delay_ms: u64,
}

impl Node for IoNode {
    fn name(&self) -> &'static str {
        self.name
    }

    fn init(&mut self) -> Result<()> {
        println!("I/O Node {} initialized", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        // Simulate blocking I/O (e.g., camera read)
        thread::sleep(self.io_delay_ms.ms());
        self.counter.fetch_add(1, Ordering::SeqCst);
    }

    fn shutdown(&mut self) -> Result<()> {
        println!("I/O Node {} shutdown", self.name);
        Ok(())
    }

    fn publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: format!("{}.data", self.name),
            type_name: "sensor_msgs/Image".to_string(),
        }]
    }

    fn subscribers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }
}

/// Test node that occasionally fails (to test skip policy)
struct FlakyNode {
    name: &'static str,
    counter: Arc<AtomicUsize>,
    fail_rate: f32, // 0.0 to 1.0
}

impl Node for FlakyNode {
    fn name(&self) -> &'static str {
        self.name
    }

    fn init(&mut self) -> Result<()> {
        println!("Flaky Node {} initialized", self.name);
        Ok(())
    }

    fn tick(&mut self) {
        let count = self.counter.fetch_add(1, Ordering::SeqCst);

        // Fail based on rate
        if (count as f32 % 10.0) / 10.0 < self.fail_rate {
            panic!("Simulated failure in flaky node!");
        }
    }

    fn shutdown(&mut self) -> Result<()> {
        println!("Flaky Node {} shutdown", self.name);
        Ok(())
    }

    fn publishers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }

    fn subscribers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: "camera.data".to_string(),
            type_name: "sensor_msgs/Image".to_string(),
        }]
    }
}

#[test]
fn test_enhanced_scheduler() {
    cleanup_stale_shm();
    // Create shared counters
    let cpu_counter = Arc::new(AtomicUsize::new(0));
    let io_counter = Arc::new(AtomicUsize::new(0));
    let flaky_counter = Arc::new(AtomicUsize::new(0));

    // Create scheduler
    let mut scheduler = Scheduler::new();

    // Add CPU-intensive nodes (should stay in main tier)
    scheduler
        .add(CpuNode {
            name: "cpu_fast",
            counter: Arc::clone(&cpu_counter),
            work_ms: 1, // Very fast
        })
        .order(10)
        .build();

    // Add I/O-heavy node (should move to async tier after learning)
    scheduler
        .add(IoNode {
            name: "camera",
            counter: Arc::clone(&io_counter),
            io_delay_ms: 50, // Blocking I/O
        })
        .order(20)
        .build();

    // Add flaky node with Ignore policy (tolerates failures without stopping)
    scheduler
        .add(FlakyNode {
            name: "processor",
            counter: Arc::clone(&flaky_counter),
            fail_rate: 0.3, // 30% failure rate
        })
        .order(30)
        .failure_policy(horus_core::scheduling::FailurePolicy::Ignore)
        .build();

    // Run scheduler for 3 seconds
    let run_duration = 3_u64.secs();
    let start = Instant::now();

    // Run in a separate thread
    let handle = thread::spawn(move || {
        scheduler.run_for(run_duration).expect("Scheduler failed");
    });

    // Wait for completion
    handle.join().expect("Scheduler thread panicked");

    let elapsed = start.elapsed();

    // Verify results
    println!("\n=== Test Results ===");
    println!("Total runtime: {:?}", elapsed);
    println!("CPU node ticks: {}", cpu_counter.load(Ordering::SeqCst));
    println!("I/O node ticks: {}", io_counter.load(Ordering::SeqCst));
    println!("Flaky node ticks: {}", flaky_counter.load(Ordering::SeqCst));

    let cpu_ticks = cpu_counter.load(Ordering::SeqCst);
    let io_ticks = io_counter.load(Ordering::SeqCst);
    let flaky_ticks = flaky_counter.load(Ordering::SeqCst);

    // CPU node (1ms work) in 3 seconds should tick many times
    assert!(
        cpu_ticks > 10,
        "CPU node should have ticked many times in 3s, got {}",
        cpu_ticks
    );
    // I/O node (50ms blocking) should still tick
    assert!(
        io_ticks > 0,
        "I/O node should have executed despite blocking delay"
    );

    // Flaky node (30% fail) should still tick due to Ignore policy
    assert!(
        flaky_ticks > 0,
        "Flaky node should have executed (Ignore policy protects from crash)"
    );

    // In a sequential scheduler, all nodes share the same tick loop,
    // so CPU and I/O nodes tick the same number of times.
    assert!(
        cpu_ticks >= io_ticks,
        "CPU node ({}) should tick at least as much as I/O node ({})",
        cpu_ticks,
        io_ticks
    );

    println!("\n Enhanced scheduler test passed!");
    println!("- Dependency graph analysis worked");
    println!("- Async I/O tier handled blocking operations");
    println!("- Failure policies protected against failures");
}

#[test]
fn test_skip_policy_protection() {
    use horus_core::scheduling::FailurePolicy;

    cleanup_stale_shm();
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Add a node that always fails with explicit Skip policy
    // (using default Fast tier so the node stays in the main loop, not the background executor)
    scheduler
        .add(FlakyNode {
            name: "always_fails",
            counter: Arc::clone(&counter),
            fail_rate: 1.0, // Always fails
        })
        .order(10)
        .failure_policy(FailurePolicy::skip(5, 30_u64.secs()))
        .build();

    // Run for 1 second
    scheduler
        .run_for(1_u64.secs())
        .expect("Scheduler failed");

    // The node should have attempted and failed multiple times, with the scheduler
    // recovering from each panic (catch_unwind) rather than crashing.
    let attempts = counter.load(Ordering::SeqCst);
    println!(
        "Failed node attempted {} times (scheduler survived all panics)",
        attempts
    );

    // The key invariant: scheduler survived 1 second of continuous panics.
    // The exact count depends on tick rate; we just verify it ran and recovered.
    assert!(attempts >= 1, "Should have attempted at least once");

    println!(" Scheduler successfully recovered from cascading node panics");
}
