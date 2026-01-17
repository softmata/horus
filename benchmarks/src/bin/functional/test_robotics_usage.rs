// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

/// Robotics Usage Test Suite
/// Verifies typical robotics usage patterns work correctly
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::env;
use std::process;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  sensor_flow");
        eprintln!("  actuator_commands");
        eprintln!("  control_loop_1khz");
        eprintln!("  transforms");
        eprintln!("  state_machine");
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "sensor_flow" => test_sensor_flow(),
        "actuator_commands" => test_actuator_commands(),
        "control_loop_1khz" => test_control_loop_1khz(),
        "transforms" => test_transforms(),
        "state_machine" => test_state_machine(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    if result {
        println!(" Test passed: {}", test_name);
        process::exit(0);
    } else {
        eprintln!("[FAIL] Test failed: {}", test_name);
        process::exit(1);
    }
}

/// Test 1: Sensor Data Flow
fn test_sensor_flow() -> bool {
    println!("Testing sensor data flow pattern...");

    // Simulate Sensor  Processor  Command pipeline using CmdVel
    let sensor_topic = format!("test_sensor_{}", process::id());
    let processed_topic = format!("test_processed_{}", process::id());
    let cmd_topic = format!("test_cmd_{}", process::id());

    let sensor_pub = match Topic::<CmdVel>::new(&sensor_topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create sensor publisher: {}", e);
            return false;
        }
    };

    let sensor_sub = match Topic::<CmdVel>::new(&sensor_topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create sensor subscriber: {}", e);
            return false;
        }
    };

    let processed_pub = match Topic::<CmdVel>::new(&processed_topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create processed publisher: {}", e);
            return false;
        }
    };

    let processed_sub = match Topic::<CmdVel>::new(&processed_topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create processed subscriber: {}", e);
            return false;
        }
    };

    let cmd_pub = match Topic::<CmdVel>::new(&cmd_topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create command publisher: {}", e);
            return false;
        }
    };

    let cmd_sub = match Topic::<CmdVel>::new(&cmd_topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create command subscriber: {}", e);
            return false;
        }
    };

    println!("   Created sensor data pipeline (Sensor  Processor  Cmd)");

    thread::sleep(Duration::from_millis(100));

    // Spawn processor thread
    let processor_handle = thread::spawn(move || {
        let mut processed = 0;
        let start = Instant::now();
        while processed < 100 && start.elapsed() < Duration::from_secs(5) {
            if let Some(sensor) = sensor_sub.recv(&mut None) {
                let processed_msg = CmdVel::with_timestamp(
                    sensor.linear * 0.8,
                    sensor.angular * 0.9,
                    sensor.stamp_nanos,
                );
                if processed_pub.send(processed_msg, &mut None).is_ok() {
                    processed += 1;
                }
            } else {
                thread::sleep(Duration::from_micros(100));
            }
        }
        processed == 100
    });

    // Spawn controller thread
    let controller_handle = thread::spawn(move || {
        let mut processed = 0;
        let start = Instant::now();
        while processed < 100 && start.elapsed() < Duration::from_secs(5) {
            if let Some(msg) = processed_sub.recv(&mut None) {
                let cmd =
                    CmdVel::with_timestamp(msg.linear * 0.5, msg.angular * 0.3, msg.stamp_nanos);
                if cmd_pub.send(cmd, &mut None).is_ok() {
                    processed += 1;
                }
            } else {
                thread::sleep(Duration::from_micros(100));
            }
        }
        processed == 100
    });

    // Main thread: publish sensor data and receive commands
    let mut received_cmds = 0;
    for i in 0..100 {
        let sensor = CmdVel::with_timestamp(1.0 + (i as f32 * 0.01), 0.5, i);

        if let Err(e) = sensor_pub.send(sensor, &mut None) {
            eprintln!("Failed to publish sensor data {}: {:?}", i, e);
            return false;
        }

        if let Some(_cmd) = cmd_sub.recv(&mut None) {
            received_cmds += 1;
        }

        thread::sleep(Duration::from_micros(500));
    }

    // Wait for remaining commands
    let start = Instant::now();
    while received_cmds < 100 && start.elapsed() < Duration::from_secs(2) {
        if cmd_sub.recv(&mut None).is_some() {
            received_cmds += 1;
        } else {
            thread::sleep(Duration::from_micros(100));
        }
    }

    let processor_result = processor_handle.join().unwrap();
    let controller_result = controller_handle.join().unwrap();

    if processor_result && controller_result && received_cmds >= 95 {
        println!("   Processor handled 100 sensor messages");
        println!("   Controller processed 100 messages");
        println!("   Received {} command messages", received_cmds);
        true
    } else {
        eprintln!(
            "Pipeline incomplete: processor={}, controller={}, cmds={}",
            processor_result, controller_result, received_cmds
        );
        false
    }
}

/// Test 2: Actuator Commands
fn test_actuator_commands() -> bool {
    println!("Testing actuator command pattern...");

    let topic = format!("test_actuator_{}", process::id());

    // Create command link (high priority, low latency)
    let cmd_sender = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create command sender: {}", e);
            return false;
        }
    };

    let cmd_receiver = match Topic::<CmdVel>::new(&topic) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to create command receiver: {}", e);
            return false;
        }
    };

    println!("   Created actuator command Link");

    // Simulate motor driver receiving commands
    let driver_handle = thread::spawn(move || {
        let mut last_timestamp = 0;
        let mut received = 0;
        let start = Instant::now();

        while received < 500 && start.elapsed() < Duration::from_secs(5) {
            if let Some(cmd) = cmd_receiver.recv(&mut None) {
                // Verify commands are in order
                if cmd.stamp_nanos < last_timestamp {
                    eprintln!(
                        "Command out of order: {} < {}",
                        cmd.stamp_nanos, last_timestamp
                    );
                    return false;
                }
                last_timestamp = cmd.stamp_nanos;
                received += 1;

                // Simulate motor actuation delay
                thread::sleep(Duration::from_micros(10));
            }
        }
        received == 500
    });

    // Send commands at regular intervals
    for i in 0..500 {
        let cmd = CmdVel {
            linear: (i as f32 * 0.01).sin(),
            angular: (i as f32 * 0.02).cos(),
            stamp_nanos: i,
        };

        if let Err(e) = cmd_sender.send(cmd, &mut None) {
            eprintln!("Failed to send command {}: {:?}", i, e);
            return false;
        }

        thread::sleep(Duration::from_micros(50)); // 20kHz command rate
    }

    let driver_result = driver_handle.join().unwrap();

    if driver_result {
        println!("   Sent 500 actuator commands");
        println!("   All commands received in correct order");
        true
    } else {
        eprintln!("Motor driver did not receive all commands");
        false
    }
}

/// Test 3: Control Loop at 1kHz (simulated with simple loop)
fn test_control_loop_1khz() -> bool {
    println!("Testing 1kHz control loop pattern...");

    let topic = format!("test_control_{}", process::id());

    let sender = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create sender: {}", e);
            return false;
        }
    };

    let receiver = match Topic::<CmdVel>::new(&topic) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to create receiver: {}", e);
            return false;
        }
    };

    println!("   Created control loop communication channel");

    let counter = Arc::new(Mutex::new(0u32));
    let counter_clone = Arc::clone(&counter);

    // Spawn receiver thread (simulates control loop consumer)
    let recv_handle = thread::spawn(move || {
        let mut received = 0;
        let start = Instant::now();

        while received < 1000 && start.elapsed() < Duration::from_secs(2) {
            if receiver.recv(&mut None).is_some() {
                received += 1;
            }
        }

        *counter_clone.lock().unwrap() = received;
        received >= 900 // Allow some tolerance
    });

    // Send at 1kHz (1ms intervals)
    let start = Instant::now();
    for i in 0..1000 {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i,
        };

        if sender.send(msg, &mut None).is_err() {
            thread::sleep(Duration::from_micros(10)); // Back off slightly if buffer full
        }

        thread::sleep(Duration::from_millis(1)); // 1kHz target
    }
    let elapsed = start.elapsed();

    let recv_result = recv_handle.join().unwrap();
    let final_count = *counter.lock().unwrap();

    println!(
        "   Sent 1000 messages in {}ms (target: 1000ms)",
        elapsed.as_millis()
    );
    println!("   Received {} messages", final_count);

    if recv_result && final_count >= 900 {
        println!("   1kHz control loop pattern successful");
        true
    } else {
        eprintln!("Control loop pattern failed: received {}/1000", final_count);
        false
    }
}

/// Test 4: Transform Broadcasting
fn test_transforms() -> bool {
    println!("Testing broadcast pattern (simulated with CmdVel)...");

    let broadcast_topic = format!("test_broadcast_{}", process::id());

    let broadcaster = match Topic::<CmdVel>::new(&broadcast_topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create broadcaster: {}", e);
            return false;
        }
    };

    let listener = match Topic::<CmdVel>::new(&broadcast_topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create listener: {}", e);
            return false;
        }
    };

    println!("   Created broadcast Hub");

    thread::sleep(Duration::from_millis(100));

    // Broadcast 200 messages at 100Hz
    for i in 0..200 {
        let msg = CmdVel::with_timestamp((i as f32) * 0.01, (i as f32) * 0.02, i);

        if let Err(e) = broadcaster.send(msg, &mut None) {
            eprintln!("Failed to broadcast message {}: {:?}", i, e);
            return false;
        }
        thread::sleep(Duration::from_millis(10)); // 100Hz
    }

    println!("   Broadcast 200 messages");

    // Receive and verify
    let mut received = 0;
    let start = Instant::now();
    while received < 200 && start.elapsed() < Duration::from_secs(5) {
        if listener.recv(&mut None).is_some() {
            received += 1;
        } else {
            thread::sleep(Duration::from_micros(100));
        }
    }

    if received >= 190 {
        println!("   Received {} broadcasts", received);
        true
    } else {
        eprintln!("Only received {} out of 200 broadcasts", received);
        false
    }
}

/// Test 5: State Machine (simulated with message pattern)
fn test_state_machine() -> bool {
    println!("Testing state machine communication pattern...");

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum RobotState {
        Idle,
        Moving,
        Turning,
        Stopped,
    }

    let topic = format!("test_statemachine_{}", process::id());

    let state_pub = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create state publisher: {}", e);
            return false;
        }
    };

    let state_sub = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create state subscriber: {}", e);
            return false;
        }
    };

    println!("   Created state machine communication Hub");

    thread::sleep(Duration::from_millis(100));

    // Simulate state machine by publishing state transitions
    let mut state = RobotState::Idle;
    let mut transitions = 0;

    let pub_handle = thread::spawn(move || {
        for i in 0..50 {
            // Encode state as linear velocity
            let linear = match state {
                RobotState::Idle => 0.0,
                RobotState::Moving => 1.0,
                RobotState::Turning => 0.5,
                RobotState::Stopped => 0.0,
            };

            let msg = CmdVel {
                linear,
                angular: transitions as f32,
                stamp_nanos: i,
            };

            if state_pub.send(msg, &mut None).is_ok() {
                // Transition to next state
                state = match state {
                    RobotState::Idle => RobotState::Moving,
                    RobotState::Moving => RobotState::Turning,
                    RobotState::Turning => RobotState::Stopped,
                    RobotState::Stopped => RobotState::Idle,
                };
                transitions += 1;
            }

            thread::sleep(Duration::from_millis(10)); // 100Hz
        }
        transitions
    });

    // Receive state updates
    let mut received_transitions = 0;
    let start = Instant::now();
    while received_transitions < 50 && start.elapsed() < Duration::from_secs(2) {
        if state_sub.recv(&mut None).is_some() {
            received_transitions += 1;
        } else {
            thread::sleep(Duration::from_micros(100));
        }
    }

    let sent_transitions = pub_handle.join().unwrap();

    println!("   Sent {} state transitions", sent_transitions);
    println!("   Received {} state updates", received_transitions);

    if received_transitions >= 45 {
        println!("   State machine communication pattern successful");
        true
    } else {
        eprintln!(
            "Incomplete state machine communication: {}/50",
            received_transitions
        );
        false
    }
}
