// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

/// API Compatibility Test Suite
/// Verifies that core APIs remain functional after structural changes
use horus::prelude::{Scheduler, Topic};
use horus_library::messages::cmd_vel::CmdVel;
use std::env;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  node_creation");
        eprintln!("  hub_pubsub");
        eprintln!("  link_sendrecv");
        eprintln!("  scheduler");
        eprintln!("  message_types");
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "node_creation" => test_node_creation(),
        "hub_pubsub" => test_hub_pubsub(),
        "link_sendrecv" => test_link_sendrecv(),
        "scheduler" => test_scheduler(),
        "message_types" => test_message_types(),
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

/// Test 1: Basic IPC Creation
fn test_node_creation() -> bool {
    println!("Testing basic IPC creation...");

    // Create Hub instances (simulating nodes communicating)
    let topic = format!("test_basic_{}", process::id());

    let hub1 = match Topic::<CmdVel>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("Failed to create Hub 1: {:?}", e);
            return false;
        }
    };

    let hub2 = match Topic::<CmdVel>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("Failed to create Hub 2: {:?}", e);
            return false;
        }
    };

    println!("   Created Hub instances successfully");

    // Test multiple Hub instances
    let mut hubs = Vec::new();
    for i in 0..10 {
        let t = format!("test_multi_{}_{}", process::id(), i);
        match Topic::<CmdVel>::new(&t) {
            Ok(h) => hubs.push(h),
            Err(e) => {
                eprintln!("Failed to create Hub {}: {:?}", i, e);
                return false;
            }
        }
    }

    println!("   Created {} Hub instances", hubs.len());
    true
}

/// Test 2: Hub Pub/Sub
fn test_hub_pubsub() -> bool {
    println!("Testing Hub publish/subscribe...");

    let topic = format!("test_hub_{}", process::id());

    // Create publisher
    let publisher = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create publisher: {}", e);
            return false;
        }
    };
    println!("   Publisher created");

    // Create subscriber
    let subscriber = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create subscriber: {}", e);
            return false;
        }
    };
    println!("   Subscriber created");

    // Give subscriber time to register
    thread::sleep(Duration::from_millis(100));

    // Publish messages
    let test_msg = CmdVel {
        linear: 1.5,
        angular: 0.5,
        stamp_nanos: 12345,
    };

    for i in 0..10 {
        let mut msg = test_msg.clone();
        msg.stamp_nanos = i;
        if let Err(e) = publisher.send(msg, &mut None) {
            eprintln!("Failed to publish message {}: {:?}", i, e);
            return false;
        }
    }
    println!("   Published 10 messages");

    // Wait a bit for message propagation
    thread::sleep(Duration::from_millis(50));

    // Receive messages
    let mut received = 0;
    let start = Instant::now();
    while received < 10 && start.elapsed() < Duration::from_secs(2) {
        if let Some(_msg) = subscriber.recv(&mut None) {
            received += 1;
        } else {
            thread::sleep(Duration::from_micros(100));
        }
    }

    if received == 10 {
        println!("   Received all 10 messages");
        true
    } else {
        eprintln!("Only received {} out of 10 messages", received);
        false
    }
}

/// Test 3: Link Send/Recv
fn test_link_sendrecv() -> bool {
    println!("Testing Link send/receive...");

    let topic = format!("test_link_{}", process::id());

    // Create link (sender will be created implicitly)
    let sender = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create sender: {}", e);
            return false;
        }
    };
    println!("   Sender created");

    let receiver = match Topic::<CmdVel>::new(&topic) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to create receiver: {}", e);
            return false;
        }
    };
    println!("   Receiver created");

    // Test 1: Send and receive a single message
    let test_msg = CmdVel {
        linear: 2.0,
        angular: 1.0,
        stamp_nanos: 54321,
    };

    if let Err(e) = sender.send(test_msg.clone(), &mut None) {
        eprintln!("Failed to send message: {:?}", e);
        return false;
    }

    // Give time for message to propagate
    thread::sleep(Duration::from_micros(100));

    let received = receiver.recv(&mut None);
    if received.is_none() {
        eprintln!("Failed to receive message");
        return false;
    }
    println!("   Basic send/receive works");

    // Test 2: Latest-value semantics (single-slot overwrite)
    // Send multiple messages rapidly, consumer should get the LATEST value
    for i in 0..100 {
        let mut msg = test_msg.clone();
        msg.stamp_nanos = i;
        if let Err(e) = sender.send(msg, &mut None) {
            eprintln!("Failed to send message {}: {:?}", i, e);
            return false;
        }
    }
    println!("   Sent 100 messages");

    // Give time for last message to propagate
    thread::sleep(Duration::from_micros(100));

    // Consumer should get the LATEST value (stamp_nanos = 99)
    if let Some(msg) = receiver.recv(&mut None) {
        if msg.stamp_nanos != 99 {
            eprintln!("Expected latest value (99), got {}", msg.stamp_nanos);
            return false;
        }
        println!("   Latest-value semantics work correctly (got message 99)");
    } else {
        eprintln!("Failed to receive latest message");
        return false;
    }

    true
}

/// Test 4: Scheduler
fn test_scheduler() -> bool {
    println!("Testing Scheduler creation...");

    // Create scheduler
    let scheduler = Scheduler::new();
    println!("   Scheduler created");

    // Test that we can create multiple schedulers
    let _scheduler2 = Scheduler::new();
    println!("   Multiple schedulers can be created");

    // Note: Testing actual node registration and execution requires
    // implementing the Node trait, which is beyond the scope of this
    // simple API compatibility test. The scheduler API itself is verified
    // by successful compilation and instantiation.

    println!("   Scheduler API compatible");
    true
}

/// Test 5: Message Types
fn test_message_types() -> bool {
    println!("Testing message type compatibility...");

    // Test CmdVel
    let cmd_vel = CmdVel {
        linear: 1.0,
        angular: 0.5,
        stamp_nanos: 12345,
    };

    let topic = format!("test_types_{}", process::id());

    // Create Hub for CmdVel
    let pub_cmdvel = match Topic::<CmdVel>::new(&format!("{}_cmdvel", topic)) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create CmdVel publisher: {}", e);
            return false;
        }
    };

    let sub_cmdvel = match Topic::<CmdVel>::new(&format!("{}_cmdvel", topic)) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create CmdVel subscriber: {}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    // Publish and receive
    if let Err(e) = pub_cmdvel.send(cmd_vel, &mut None) {
        eprintln!("Failed to publish CmdVel: {:?}", e);
        return false;
    }

    thread::sleep(Duration::from_millis(20));

    match sub_cmdvel.recv(&mut None) {
        Some(msg) => {
            if msg.linear == cmd_vel.linear
                && msg.angular == cmd_vel.angular
                && msg.stamp_nanos == cmd_vel.stamp_nanos
            {
                println!("   CmdVel type works correctly");
            } else {
                eprintln!("CmdVel data corrupted");
                return false;
            }
        }
        None => {
            eprintln!("Failed to receive CmdVel");
            return false;
        }
    }

    println!("   All message types compatible");
    true
}
