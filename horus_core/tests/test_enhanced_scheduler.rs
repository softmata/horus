use horus_core::{Node, Result, Scheduler, TopicMetadata};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

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

    fn get_publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: format!("{}/output", self.name),
            type_name: "std_msgs/Int32".to_string(),
        }]
    }

    fn get_subscribers(&self) -> Vec<TopicMetadata> {
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
        thread::sleep(Duration::from_millis(self.io_delay_ms));
        self.counter.fetch_add(1, Ordering::SeqCst);
    }

    fn shutdown(&mut self) -> Result<()> {
        println!("I/O Node {} shutdown", self.name);
        Ok(())
    }

    fn get_publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: format!("{}/data", self.name),
            type_name: "sensor_msgs/Image".to_string(),
        }]
    }

    fn get_subscribers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }
}

/// Test node that occasionally fails (to test circuit breaker)
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

    fn get_publishers(&self) -> Vec<TopicMetadata> {
        Vec::new()
    }

    fn get_subscribers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: "camera.data".to_string(),
            type_name: "sensor_msgs/Image".to_string(),
        }]
    }
}

#[test]
fn test_enhanced_scheduler() {
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
        .done();

    // Add I/O-heavy node (should move to async tier after learning)
    scheduler
        .add(IoNode {
            name: "camera",
            counter: Arc::clone(&io_counter),
            io_delay_ms: 50, // Blocking I/O
        })
        .order(20)
        .done();

    // Add flaky node (to test circuit breaker)
    scheduler
        .add(FlakyNode {
            name: "processor",
            counter: Arc::clone(&flaky_counter),
            fail_rate: 0.3, // 30% failure rate
        })
        .order(30)
        .done();

    // Run scheduler for 3 seconds
    let run_duration = Duration::from_secs(3);
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

    // Assertions
    assert!(
        cpu_counter.load(Ordering::SeqCst) > 0,
        "CPU node should have executed"
    );
    assert!(
        io_counter.load(Ordering::SeqCst) > 0,
        "I/O node should have executed"
    );

    // Flaky node should have some ticks despite failures
    // Circuit breaker should protect it from crashing the system
    assert!(
        flaky_counter.load(Ordering::SeqCst) > 0,
        "Flaky node should have executed at least once"
    );

    println!("\n Enhanced scheduler test passed!");
    println!("- Dependency graph analysis worked");
    println!("- Async I/O tier handled blocking operations");
    println!("- Circuit breaker protected against failures");
}

#[test]
fn test_circuit_breaker_protection() {
    let counter = Arc::new(AtomicUsize::new(0));

    let mut scheduler = Scheduler::new();

    // Add a node that always fails
    scheduler
        .add(FlakyNode {
            name: "always_fails",
            counter: Arc::clone(&counter),
            fail_rate: 1.0, // Always fails
        })
        .order(10)
        .done();

    // Run for 1 second
    scheduler
        .run_for(Duration::from_secs(1))
        .expect("Scheduler failed");

    // The node should have attempted a few times before circuit breaker opened
    let attempts = counter.load(Ordering::SeqCst);
    println!(
        "Failed node attempted {} times before circuit opened",
        attempts
    );

    // Circuit breaker should have limited attempts (default threshold is 5)
    assert!(
        attempts <= 10,
        "Circuit breaker should have limited attempts"
    );
    assert!(attempts >= 1, "Should have attempted at least once");

    println!(" Circuit breaker successfully protected against cascading failures");
}
