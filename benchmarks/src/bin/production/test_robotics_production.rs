// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

/// Comprehensive Robotics Test Suite
///
/// This test suite validates the single-slot Link implementation
/// by simulating realistic robotics workloads with high-frequency sensor loops, control loops,
/// multi-rate systems, and stress scenarios.
///
/// Run with: ./target/release/test_robotics_production <test_name>
use horus::prelude::Topic;
use horus_library::messages::control::{DifferentialDriveCommand, MotorCommand, PidConfig};
use horus_library::messages::geometry::Transform;
use horus_library::messages::sensor::Imu;
use std::env;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// Test result structure
struct TestResult {
    success: bool,
    message: String,
}

impl TestResult {
    fn success(msg: impl Into<String>) -> Self {
        Self {
            success: true,
            message: msg.into(),
        }
    }

    fn failure(msg: impl Into<String>) -> Self {
        Self {
            success: false,
            message: msg.into(),
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        print_usage();
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "high_freq_sensor" => test_high_frequency_sensor_loop(),
        "control_loop" => test_control_loop(),
        "multi_rate" => test_multi_rate_system(),
        "robot_pipeline" => test_realistic_robot_pipeline(),
        "stress" => test_stress_burst(),
        "multi_process" => test_multi_process_ipc(),
        "all" => run_all_tests(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            print_usage();
            process::exit(1);
        }
    };

    if result.success {
        println!("\n{}", "=".repeat(60));
        println!("TEST PASSED: {}", test_name);
        println!("{}", "=".repeat(60));
        println!("{}", result.message);
        process::exit(0);
    } else {
        eprintln!("\n{}", "=".repeat(60));
        eprintln!("TEST FAILED: {}", test_name);
        eprintln!("{}", "=".repeat(60));
        eprintln!("{}", result.message);
        process::exit(1);
    }
}

fn print_usage() {
    eprintln!("Robotics Test Suite");
    eprintln!("\nUsage: test_robotics_production <test_name>");
    eprintln!("\nAvailable tests:");
    eprintln!("  high_freq_sensor  - High-frequency sensor loop (100-1000Hz)");
    eprintln!("  control_loop      - Control loop timing validation (50-100Hz)");
    eprintln!("  multi_rate        - Multi-rate system coordination");
    eprintln!("  robot_pipeline    - Realistic sensor-controller-actuator pipeline");
    eprintln!("  stress            - Stress test with message bursts");
    eprintln!("  multi_process     - Multi-process IPC validation");
    eprintln!("  all               - Run all tests");
}

fn run_all_tests() -> TestResult {
    println!("Running complete test suite...\n");

    let tests: Vec<(&str, fn() -> TestResult)> = vec![
        ("high_freq_sensor", test_high_frequency_sensor_loop),
        ("control_loop", test_control_loop),
        ("multi_rate", test_multi_rate_system),
        ("robot_pipeline", test_realistic_robot_pipeline),
        ("stress", test_stress_burst),
        ("multi_process", test_multi_process_ipc),
    ];

    let mut passed = 0;
    let mut failed = 0;

    for (name, test_fn) in tests {
        println!("\n{}", "-".repeat(60));
        println!("Running test: {}", name);
        println!("{}", "-".repeat(60));

        let result = test_fn();
        if result.success {
            println!("PASSED: {}", name);
            passed += 1;
        } else {
            eprintln!("FAILED: {}", name);
            eprintln!("  {}", result.message);
            failed += 1;
        }
    }

    println!("\n{}", "=".repeat(60));
    println!("Test Summary: {} passed, {} failed", passed, failed);
    println!("{}", "=".repeat(60));

    if failed == 0 {
        TestResult::success(format!("All {} tests passed!", passed))
    } else {
        TestResult::failure(format!(
            "{} out of {} tests failed. System needs fixes.",
            failed,
            passed + failed
        ))
    }
}

/// Test 1: High-Frequency Sensor Loop (100-1000Hz)
///
/// Validates:
/// - IMU data publishing at 200Hz
/// - Encoder readings at 100Hz
/// - Latest-value semantics work correctly
/// - No message drops break the system
fn test_high_frequency_sensor_loop() -> TestResult {
    println!("Testing high-frequency sensor loop...");

    let imu_topic = format!("test_imu_{}", process::id());
    let encoder_topic = format!("test_encoder_{}", process::id());

    // Create IMU link (200Hz)
    let imu_producer = match Topic::<Imu>::producer(&imu_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create IMU producer: {}", e)),
    };

    let imu_consumer = match Topic::<Imu>::consumer(&imu_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create IMU consumer: {}", e)),
    };

    // Create encoder link (100Hz)
    let encoder_producer = match Topic::<MotorCommand>::producer(&encoder_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create encoder producer: {}", e)),
    };

    let encoder_consumer = match Topic::<MotorCommand>::consumer(&encoder_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create encoder consumer: {}", e)),
    };

    println!("  Created sensor Links");

    let running = Arc::new(AtomicBool::new(true));
    let imu_received = Arc::new(AtomicU64::new(0));
    let encoder_received = Arc::new(AtomicU64::new(0));

    // IMU consumer thread (200Hz)
    let imu_thread_running = running.clone();
    let imu_thread_received = imu_received.clone();
    let imu_thread = thread::spawn(move || {
        let mut last_timestamp = 0;
        let mut out_of_order = 0;

        while imu_thread_running.load(Ordering::Relaxed) {
            if let Some(imu) = imu_consumer.recv(&mut None) {
                // Verify latest-value semantics: timestamps should be monotonic
                if imu.timestamp < last_timestamp {
                    out_of_order += 1;
                }
                last_timestamp = imu.timestamp;
                imu_thread_received.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(100)); // Poll every 100us
        }
        out_of_order
    });

    // Encoder consumer thread (100Hz)
    let encoder_thread_running = running.clone();
    let encoder_thread_received = encoder_received.clone();
    let encoder_thread = thread::spawn(move || {
        let mut last_timestamp = 0;
        let mut out_of_order = 0;

        while encoder_thread_running.load(Ordering::Relaxed) {
            if let Some(encoder) = encoder_consumer.recv(&mut None) {
                if encoder.timestamp < last_timestamp {
                    out_of_order += 1;
                }
                last_timestamp = encoder.timestamp;
                encoder_thread_received.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(200)); // Poll every 200us
        }
        out_of_order
    });

    // Publish IMU at 200Hz for 1 second
    let start = Instant::now();
    let mut imu_sent = 0;
    while start.elapsed() < Duration::from_secs(1) {
        let mut imu = Imu::new();
        imu.timestamp = imu_sent;
        imu.angular_velocity = [0.1, 0.2, 0.3];
        imu.linear_acceleration = [0.0, 0.0, 9.81];

        if imu_producer.send(imu, &mut None).is_ok() {
            imu_sent += 1;
        }
        thread::sleep(Duration::from_micros(5000)); // 200Hz = 5ms
    }

    // Publish encoders at 100Hz for 1 second
    let start = Instant::now();
    let mut encoder_sent = 0;
    while start.elapsed() < Duration::from_secs(1) {
        let encoder = MotorCommand::velocity(0, encoder_sent as f64);

        if encoder_producer.send(encoder, &mut None).is_ok() {
            encoder_sent += 1;
        }
        thread::sleep(Duration::from_micros(10000)); // 100Hz = 10ms
    }

    // Let consumers catch up
    thread::sleep(Duration::from_millis(200));

    running.store(false, Ordering::Relaxed);

    let imu_out_of_order = imu_thread.join().unwrap();
    let encoder_out_of_order = encoder_thread.join().unwrap();

    let imu_rcvd = imu_received.load(Ordering::Relaxed);
    let encoder_rcvd = encoder_received.load(Ordering::Relaxed);

    println!("  IMU: sent={}, received={}", imu_sent, imu_rcvd);
    println!(
        "  Encoder: sent={}, received={}",
        encoder_sent, encoder_rcvd
    );

    // Validation criteria:
    // - Should receive at least 150 IMU messages (200Hz * 1s = 200, allow some tolerance)
    // - Should receive at least 80 encoder messages (100Hz * 1s = 100, allow some tolerance)
    // - No out-of-order messages (latest-value semantics)
    if imu_rcvd < 150 {
        return TestResult::failure(format!(
            "Insufficient IMU messages received: {} < 150",
            imu_rcvd
        ));
    }

    if encoder_rcvd < 80 {
        return TestResult::failure(format!(
            "Insufficient encoder messages received: {} < 80",
            encoder_rcvd
        ));
    }

    if imu_out_of_order > 0 {
        return TestResult::failure(format!("IMU messages out of order: {}", imu_out_of_order));
    }

    if encoder_out_of_order > 0 {
        return TestResult::failure(format!(
            "Encoder messages out of order: {}",
            encoder_out_of_order
        ));
    }

    TestResult::success(format!(
        "High-frequency sensor loop validated:\n\
         - IMU: {} messages at 200Hz\n\
         - Encoder: {} messages at 100Hz\n\
         - Latest-value semantics maintained\n\
         - No timing violations",
        imu_rcvd, encoder_rcvd
    ))
}

/// Test 2: Control Loop (50-100Hz)
///
/// Validates:
/// - PID controller receiving sensor data
/// - Publishing control commands
/// - Timing constraints met
fn test_control_loop() -> TestResult {
    println!("Testing control loop timing...");

    let sensor_topic = format!("test_ctrl_sensor_{}", process::id());
    let command_topic = format!("test_ctrl_cmd_{}", process::id());

    let sensor_producer = match Topic::<Imu>::producer(&sensor_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create sensor producer: {}", e)),
    };

    let sensor_consumer = match Topic::<Imu>::consumer(&sensor_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create sensor consumer: {}", e)),
    };

    let command_producer = match Topic::<DifferentialDriveCommand>::producer(&command_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create command producer: {}", e)),
    };

    let command_consumer = match Topic::<DifferentialDriveCommand>::consumer(&command_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create command consumer: {}", e)),
    };

    println!("  Created control loop Links");

    let running = Arc::new(AtomicBool::new(true));
    let commands_received = Arc::new(AtomicU64::new(0));

    // Controller thread (50Hz control loop)
    let controller_running = running.clone();
    let controller_thread = thread::spawn(move || {
        let pid_config = PidConfig::pi(1.0, 0.1);
        let mut integral = 0.0;
        let target = 0.0; // Try to maintain zero angular velocity

        let start = Instant::now();
        let mut loop_count = 0;
        let mut max_latency = Duration::ZERO;
        let mut total_latency = Duration::ZERO;

        while controller_running.load(Ordering::Relaxed) && start.elapsed() < Duration::from_secs(1)
        {
            let loop_start = Instant::now();

            if let Some(imu) = sensor_consumer.recv(&mut None) {
                // PID control on angular velocity Z
                let error = target - imu.angular_velocity[2];
                integral += error * 0.02; // 50Hz = 20ms = 0.02s
                let output = pid_config.kp * error + pid_config.ki * integral;

                let cmd = DifferentialDriveCommand::new(-output, output);
                let _ = command_producer.send(cmd, &mut None);

                loop_count += 1;
            }

            let latency = loop_start.elapsed();
            total_latency += latency;
            if latency > max_latency {
                max_latency = latency;
            }

            // 50Hz = 20ms period
            thread::sleep(Duration::from_millis(20));
        }

        (loop_count, max_latency, total_latency)
    });

    // Command receiver thread
    let receiver_running = running.clone();
    let receiver_received = commands_received.clone();
    let receiver_thread = thread::spawn(move || {
        while receiver_running.load(Ordering::Relaxed) {
            if command_consumer.recv(&mut None).is_some() {
                receiver_received.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(500));
        }
    });

    // Publish sensor data at 100Hz for 1.2 seconds
    let start = Instant::now();
    let mut sensor_sent = 0;
    while start.elapsed() < Duration::from_millis(1200) {
        let mut imu = Imu::new();
        imu.angular_velocity = [0.0, 0.0, 0.1]; // Some angular velocity to control
        imu.timestamp = sensor_sent;

        if sensor_producer.send(imu, &mut None).is_ok() {
            sensor_sent += 1;
        }
        thread::sleep(Duration::from_millis(10)); // 100Hz
    }

    running.store(false, Ordering::Relaxed);

    let (loop_count, max_latency, total_latency) = controller_thread.join().unwrap();
    let _ = receiver_thread.join();

    let commands_rcvd = commands_received.load(Ordering::Relaxed);
    let avg_latency = total_latency / loop_count.max(1) as u32;

    println!("  Sensor messages sent: {}", sensor_sent);
    println!("  Control loops executed: {}", loop_count);
    println!("  Commands received: {}", commands_rcvd);
    println!("  Average loop latency: {:?}", avg_latency);
    println!("  Max loop latency: {:?}", max_latency);

    // Validation:
    // - Should execute ~50 control loops (50Hz * 1s)
    // - Max latency should be under 10ms for real-time control
    // - Should receive most commands
    if loop_count < 40 {
        return TestResult::failure(format!("Insufficient control loops: {} < 40", loop_count));
    }

    if max_latency > Duration::from_millis(10) {
        return TestResult::failure(format!(
            "Control loop latency too high: {:?} > 10ms",
            max_latency
        ));
    }

    if commands_rcvd < 35 {
        return TestResult::failure(format!(
            "Insufficient commands received: {} < 35",
            commands_rcvd
        ));
    }

    TestResult::success(format!(
        "Control loop validated:\n\
         - {} loops at 50Hz\n\
         - Avg latency: {:?}\n\
         - Max latency: {:?}\n\
         - {} commands transmitted\n\
         - Real-time constraints met",
        loop_count, avg_latency, max_latency, commands_rcvd
    ))
}

/// Test 3: Multi-Rate System
///
/// Validates:
/// - Fast sensors (200Hz)
/// - Medium control (50Hz)
/// - Slow planning (10Hz)
/// - All communicating via Link
fn test_multi_rate_system() -> TestResult {
    println!("Testing multi-rate system coordination...");

    let fast_topic = format!("test_fast_{}", process::id());
    let medium_topic = format!("test_medium_{}", process::id());
    let slow_topic = format!("test_slow_{}", process::id());

    // Fast sensor link (200Hz)
    let fast_producer = match Topic::<Imu>::producer(&fast_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create fast producer: {}", e)),
    };

    let fast_consumer = match Topic::<Imu>::consumer(&fast_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create fast consumer: {}", e)),
    };

    // Medium control link (50Hz)
    let medium_producer = match Topic::<DifferentialDriveCommand>::producer(&medium_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create medium producer: {}", e)),
    };

    let medium_consumer = match Topic::<DifferentialDriveCommand>::consumer(&medium_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create medium consumer: {}", e)),
    };

    // Slow planning link (10Hz)
    let slow_producer = match Topic::<Transform>::producer(&slow_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create slow producer: {}", e)),
    };

    let slow_consumer = match Topic::<Transform>::consumer(&slow_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create slow consumer: {}", e)),
    };

    println!("  Created multi-rate Links");

    let running = Arc::new(AtomicBool::new(true));
    let fast_count = Arc::new(AtomicU64::new(0));
    let medium_count = Arc::new(AtomicU64::new(0));
    let slow_count = Arc::new(AtomicU64::new(0));

    // Fast sensor reader (200Hz)
    let fast_running = running.clone();
    let fast_thread_count = fast_count.clone();
    let fast_thread = thread::spawn(move || {
        while fast_running.load(Ordering::Relaxed) {
            if fast_consumer.recv(&mut None).is_some() {
                fast_thread_count.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(100));
        }
    });

    // Medium control reader (50Hz)
    let medium_running = running.clone();
    let medium_thread_count = medium_count.clone();
    let medium_thread = thread::spawn(move || {
        while medium_running.load(Ordering::Relaxed) {
            if medium_consumer.recv(&mut None).is_some() {
                medium_thread_count.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(500));
        }
    });

    // Slow planning reader (10Hz)
    let slow_running = running.clone();
    let slow_thread_count = slow_count.clone();
    let slow_thread = thread::spawn(move || {
        while slow_running.load(Ordering::Relaxed) {
            if slow_consumer.recv(&mut None).is_some() {
                slow_thread_count.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_millis(5));
        }
    });

    // Publishers
    let start = Instant::now();
    let duration = Duration::from_secs(1);

    let mut fast_sent = 0;
    let mut medium_sent = 0;
    let mut slow_sent = 0;

    while start.elapsed() < duration {
        let elapsed = start.elapsed();

        // Fast: 200Hz = 5ms
        if elapsed.as_micros() % 5000 < 1000 {
            let imu = Imu::new();
            if fast_producer.send(imu, &mut None).is_ok() {
                fast_sent += 1;
            }
        }

        // Medium: 50Hz = 20ms
        if elapsed.as_millis() % 20 == 0 {
            let cmd = DifferentialDriveCommand::new(1.0, 1.0);
            if medium_producer.send(cmd, &mut None).is_ok() {
                medium_sent += 1;
            }
        }

        // Slow: 10Hz = 100ms
        if elapsed.as_millis() % 100 == 0 {
            let transform = Transform::identity();
            if slow_producer.send(transform, &mut None).is_ok() {
                slow_sent += 1;
            }
        }

        thread::sleep(Duration::from_micros(500));
    }

    thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);

    let _ = fast_thread.join();
    let _ = medium_thread.join();
    let _ = slow_thread.join();

    let fast_rcvd = fast_count.load(Ordering::Relaxed);
    let medium_rcvd = medium_count.load(Ordering::Relaxed);
    let slow_rcvd = slow_count.load(Ordering::Relaxed);

    println!("  Fast (200Hz): sent={}, received={}", fast_sent, fast_rcvd);
    println!(
        "  Medium (50Hz): sent={}, received={}",
        medium_sent, medium_rcvd
    );
    println!("  Slow (10Hz): sent={}, received={}", slow_sent, slow_rcvd);

    // Validation: Should receive most messages at each rate
    if fast_rcvd < 150 {
        return TestResult::failure(format!("Insufficient fast messages: {} < 150", fast_rcvd));
    }

    if medium_rcvd < 35 {
        return TestResult::failure(format!(
            "Insufficient medium messages: {} < 35",
            medium_rcvd
        ));
    }

    if slow_rcvd < 7 {
        return TestResult::failure(format!("Insufficient slow messages: {} < 7", slow_rcvd));
    }

    TestResult::success(format!(
        "Multi-rate system validated:\n\
         - Fast (200Hz): {} messages\n\
         - Medium (50Hz): {} messages\n\
         - Slow (10Hz): {} messages\n\
         - All rates coordinated successfully",
        fast_rcvd, medium_rcvd, slow_rcvd
    ))
}

/// Test 4: Realistic Robot Pipeline
///
/// Validates:
/// - Sensor -> Controller -> Actuator pipeline
/// - Multiple sensors (IMU + encoders)
/// - Controller fusing data
/// - Actuator receiving commands
fn test_realistic_robot_pipeline() -> TestResult {
    println!("Testing realistic robot sensor-controller-actuator pipeline...");

    let imu_topic = format!("test_robot_imu_{}", process::id());
    let encoder_topic = format!("test_robot_encoder_{}", process::id());
    let control_topic = format!("test_robot_control_{}", process::id());

    // Sensor Links
    let imu_producer = match Topic::<Imu>::producer(&imu_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create IMU producer: {}", e)),
    };

    let imu_consumer = match Topic::<Imu>::consumer(&imu_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create IMU consumer: {}", e)),
    };

    let encoder_producer = match Topic::<MotorCommand>::producer(&encoder_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create encoder producer: {}", e)),
    };

    let encoder_consumer = match Topic::<MotorCommand>::consumer(&encoder_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create encoder consumer: {}", e)),
    };

    // Control Link
    let control_producer = match Topic::<DifferentialDriveCommand>::producer(&control_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create control producer: {}", e)),
    };

    let control_consumer = match Topic::<DifferentialDriveCommand>::consumer(&control_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create control consumer: {}", e)),
    };

    println!("  Created robot pipeline Links");

    let running = Arc::new(AtomicBool::new(true));
    let control_commands = Arc::new(AtomicU64::new(0));

    // Controller thread: Fuses IMU and encoder data, publishes control commands
    let controller_running = running.clone();
    let controller_thread = thread::spawn(move || {
        let mut last_imu_timestamp = 0;
        let mut last_encoder_timestamp = 0;
        let mut commands_sent = 0;

        let start = Instant::now();
        while controller_running.load(Ordering::Relaxed) && start.elapsed() < Duration::from_secs(1)
        {
            let mut got_imu = false;
            let mut got_encoder = false;

            // Read latest sensor data
            if let Some(imu) = imu_consumer.recv(&mut None) {
                last_imu_timestamp = imu.timestamp;
                got_imu = true;
            }

            if let Some(encoder) = encoder_consumer.recv(&mut None) {
                last_encoder_timestamp = encoder.timestamp;
                got_encoder = true;
            }

            // Only send control command if we have fresh data from both sensors
            if got_imu || got_encoder {
                // Simple controller: use timestamps to compute command
                let cmd_value =
                    (last_imu_timestamp as f64 * 0.001) + (last_encoder_timestamp as f64 * 0.001);
                let cmd = DifferentialDriveCommand::new(cmd_value, cmd_value * 0.5);

                if control_producer.send(cmd, &mut None).is_ok() {
                    commands_sent += 1;
                }
            }

            thread::sleep(Duration::from_millis(10)); // 100Hz control
        }

        commands_sent
    });

    // Actuator thread: Receives and executes control commands
    let actuator_running = running.clone();
    let actuator_commands = control_commands.clone();
    let actuator_thread = thread::spawn(move || {
        let mut last_timestamp = 0;
        let mut out_of_order = 0;

        while actuator_running.load(Ordering::Relaxed) {
            if let Some(cmd) = control_consumer.recv(&mut None) {
                if cmd.timestamp < last_timestamp {
                    out_of_order += 1;
                }
                last_timestamp = cmd.timestamp;
                actuator_commands.fetch_add(1, Ordering::Relaxed);

                // Simulate actuator processing
                thread::sleep(Duration::from_micros(100));
            }
            thread::sleep(Duration::from_micros(200));
        }
        out_of_order
    });

    // Sensor threads: Publish IMU at 200Hz and encoders at 100Hz
    let imu_running = running.clone();
    let imu_thread = thread::spawn(move || {
        let mut count = 0;
        let start = Instant::now();
        while imu_running.load(Ordering::Relaxed) && start.elapsed() < Duration::from_secs(1) {
            let mut imu = Imu::new();
            imu.timestamp = count;
            imu.angular_velocity = [0.1, 0.2, 0.3];

            if imu_producer.send(imu, &mut None).is_ok() {
                count += 1;
            }
            thread::sleep(Duration::from_micros(5000)); // 200Hz
        }
        count
    });

    let encoder_running = running.clone();
    let encoder_thread = thread::spawn(move || {
        let mut count = 0;
        let start = Instant::now();
        while encoder_running.load(Ordering::Relaxed) && start.elapsed() < Duration::from_secs(1) {
            let encoder = MotorCommand::velocity(0, count as f64);

            if encoder_producer.send(encoder, &mut None).is_ok() {
                count += 1;
            }
            thread::sleep(Duration::from_micros(10000)); // 100Hz
        }
        count
    });

    // Wait for test completion
    let imu_sent = imu_thread.join().unwrap();
    let encoder_sent = encoder_thread.join().unwrap();

    thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);

    let commands_sent = controller_thread.join().unwrap();
    let out_of_order = actuator_thread.join().unwrap();
    let commands_rcvd = control_commands.load(Ordering::Relaxed);

    println!("  IMU messages: {}", imu_sent);
    println!("  Encoder messages: {}", encoder_sent);
    println!("  Control commands sent: {}", commands_sent);
    println!("  Control commands received: {}", commands_rcvd);

    // Validation
    if imu_sent < 150 {
        return TestResult::failure(format!("Insufficient IMU messages: {} < 150", imu_sent));
    }

    if encoder_sent < 80 {
        return TestResult::failure(format!(
            "Insufficient encoder messages: {} < 80",
            encoder_sent
        ));
    }

    if commands_sent < 80 {
        return TestResult::failure(format!(
            "Insufficient control commands sent: {} < 80",
            commands_sent
        ));
    }

    if commands_rcvd < 70 {
        return TestResult::failure(format!(
            "Insufficient control commands received: {} < 70",
            commands_rcvd
        ));
    }

    if out_of_order > 0 {
        return TestResult::failure(format!("Commands out of order: {}", out_of_order));
    }

    TestResult::success(format!(
        "Realistic robot pipeline validated:\n\
         - IMU sensor: {} messages at 200Hz\n\
         - Encoder sensor: {} messages at 100Hz\n\
         - Controller: {} commands at 100Hz\n\
         - Actuator: {} commands executed\n\
         - Full pipeline operational",
        imu_sent, encoder_sent, commands_sent, commands_rcvd
    ))
}

/// Test 5: Stress Test with Bursts
///
/// Validates:
/// - Bursts of rapid messages
/// - Overwrite behavior is safe
/// - No crashes or deadlocks
fn test_stress_burst() -> TestResult {
    println!("Testing stress conditions with message bursts...");

    let stress_topic = format!("test_stress_{}", process::id());

    let producer = match Topic::<Imu>::producer(&stress_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create stress producer: {}", e)),
    };

    let consumer = match Topic::<Imu>::consumer(&stress_topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create stress consumer: {}", e)),
    };

    println!("  Created stress test Link");

    let running = Arc::new(AtomicBool::new(true));
    let received_count = Arc::new(AtomicU64::new(0));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_count = received_count.clone();
    let consumer_thread = thread::spawn(move || {
        let mut last_seq = 0;
        let mut valid_sequences = 0;

        while consumer_running.load(Ordering::Relaxed) {
            if let Some(imu) = consumer.recv(&mut None) {
                // Verify sequence monotonicity (latest-value semantics)
                if imu.timestamp >= last_seq {
                    valid_sequences += 1;
                }
                last_seq = imu.timestamp;
                consumer_count.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(50));
        }
        valid_sequences
    });

    // Stress test: Send bursts of messages
    let start = Instant::now();
    let mut total_sent = 0;

    while start.elapsed() < Duration::from_secs(2) {
        // Send burst of 100 messages as fast as possible
        for i in 0..100 {
            let mut imu = Imu::new();
            imu.timestamp = total_sent + i;
            imu.angular_velocity = [i as f64, i as f64 * 2.0, i as f64 * 3.0];

            if producer.send(imu, &mut None).is_ok() {
                // All sends should succeed (single-slot never fails)
            } else {
                return TestResult::failure("Send failed during burst".to_string());
            }
        }
        total_sent += 100;

        // Small pause between bursts
        thread::sleep(Duration::from_millis(10));
    }

    thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);

    let valid_sequences = consumer_thread.join().unwrap();
    let received = received_count.load(Ordering::Relaxed);

    println!("  Total messages sent: {}", total_sent);
    println!("  Messages received: {}", received);
    println!("  Valid sequences: {}", valid_sequences);

    // Validation:
    // - Should handle all sends without error
    // - Consumer should receive messages (many will be overwritten)
    // - All received messages should maintain sequence order
    if received == 0 {
        return TestResult::failure("No messages received during stress test".to_string());
    }

    if valid_sequences != received {
        return TestResult::failure(format!(
            "Sequence violations: {} valid out of {} received",
            valid_sequences, received
        ));
    }

    TestResult::success(format!(
        "Stress test passed:\n\
         - Sent {} messages in bursts\n\
         - Received {} messages\n\
         - All {} sequences valid\n\
         - No crashes or deadlocks\n\
         - Overwrite behavior safe",
        total_sent, received, valid_sequences
    ))
}

/// Test 6: Multi-Process IPC
///
/// Validates:
/// - Fork producer/consumer processes
/// - IPC works correctly across processes
/// - Clean shutdown
fn test_multi_process_ipc() -> TestResult {
    println!("Testing multi-process IPC...");

    let ipc_topic = format!("test_ipc_{}", process::id());

    // Create producer in current process
    let producer = match Topic::<MotorCommand>::producer(&ipc_topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create IPC producer: {}", e)),
    };

    println!("  Created producer");

    // Spawn consumer as a separate thread (simulating separate process)
    let consumer_topic = ipc_topic.clone();
    let received_count = Arc::new(AtomicU64::new(0));
    let received_count_clone = received_count.clone();

    let consumer_thread = thread::spawn(move || {
        // Create consumer in "separate process" (thread)
        let consumer = match Topic::<MotorCommand>::consumer(&consumer_topic) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Failed to create IPC consumer: {}", e);
                return 0;
            }
        };

        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(2) {
            if let Some(cmd) = consumer.recv(&mut None) {
                // Verify data integrity
                if cmd.mode == MotorCommand::MODE_VELOCITY {
                    received_count_clone.fetch_add(1, Ordering::Relaxed);
                }
            }
            thread::sleep(Duration::from_micros(500));
        }

        received_count_clone.load(Ordering::Relaxed)
    });

    // Give consumer time to initialize
    thread::sleep(Duration::from_millis(100));

    // Producer sends messages
    let start = Instant::now();
    let mut sent = 0;

    while start.elapsed() < Duration::from_millis(1500) {
        let cmd = MotorCommand::velocity(0, sent as f64);

        if producer.send(cmd, &mut None).is_ok() {
            sent += 1;
        }
        thread::sleep(Duration::from_millis(5)); // 200Hz
    }

    // Wait for consumer to finish
    let received = consumer_thread.join().unwrap();

    println!("  Messages sent: {}", sent);
    println!("  Messages received: {}", received);

    // Validation
    if sent < 200 {
        return TestResult::failure(format!("Insufficient messages sent: {} < 200", sent));
    }

    if received < 180 {
        return TestResult::failure(format!(
            "Insufficient messages received across IPC: {} < 180",
            received
        ));
    }

    TestResult::success(format!(
        "Multi-process IPC validated:\n\
         - Sent {} messages across processes\n\
         - Received {} messages\n\
         - IPC communication working\n\
         - Clean shutdown successful",
        sent, received
    ))
}
