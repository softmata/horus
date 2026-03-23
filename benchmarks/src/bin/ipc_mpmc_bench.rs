//! Cross-process MPMC benchmark: horus MpmcShm vs iceoryx2
//!
//! Uses fork() to create real separate processes sharing via SHM.
//! Run: cargo run --release -p horus_benchmarks --bin ipc_mpmc_bench --features iceoryx2 --no-default-features

use std::time::Instant;

use horus_core::communication::Topic;
use iceoryx2::prelude::*;
use serde::{Deserialize, Serialize};

const ITERATIONS: usize = 20_000;

fn main() {
    println!("==========================================================");
    println!("  Cross-Process MPMC: horus MpmcShm vs iceoryx2");
    println!("  {} iterations per publisher", ITERATIONS);
    println!("==========================================================\n");

    // --- horus: 2 publisher processes, 1 subscriber process ---
    println!("--- horus 2P/1S cross-process ---");
    bench_horus_xproc_mpmc(2, 1);

    println!("--- iceoryx2 2P/1S cross-process ---");
    bench_iox2_xproc_mpmc(2, 1);

    println!("--- horus 1P/2S cross-process ---");
    bench_horus_xproc_spmc(1, 2);

    println!("--- iceoryx2 1P/2S cross-process ---");
    bench_iox2_xproc_spmc(1, 2);
}

fn bench_horus_xproc_mpmc(num_pubs: usize, num_subs: usize) {
    // Fork publisher children
    let mut child_pids = Vec::new();
    for p in 0..num_pubs {
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            // Child publisher
            std::thread::sleep(std::time::Duration::from_millis(300));
            let topic: Topic<u64> = Topic::new("bench.horus.xproc.mpmc").expect("topic");
            for i in 0..ITERATIONS {
                topic.send((p * ITERATIONS + i) as u64);
            }
            std::process::exit(0);
        }
        child_pids.push(pid);
    }

    // Parent is subscriber
    std::thread::sleep(std::time::Duration::from_millis(200));
    let topic: Topic<u64> = Topic::new("bench.horus.xproc.mpmc").expect("topic");
    std::thread::sleep(std::time::Duration::from_millis(200));

    let total_expected = ITERATIONS * num_pubs;
    let mut received = 0;
    let mut latencies = Vec::new();
    let deadline = Instant::now() + std::time::Duration::from_secs(15);

    while received < total_expected && Instant::now() < deadline {
        let t0 = Instant::now();
        if topic.recv().is_some() {
            latencies.push(t0.elapsed().as_nanos() as u64);
            received += 1;
        } else {
            std::hint::spin_loop();
        }
    }

    // Wait for children
    for pid in child_pids {
        unsafe { libc::waitpid(pid, std::ptr::null_mut(), 0); }
    }

    if !latencies.is_empty() {
        latencies.sort();
        let n = latencies.len();
        let avg = latencies.iter().sum::<u64>() / n as u64;
        let med = latencies[n / 2];
        let p99 = latencies[(n as f64 * 0.99) as usize];
        println!("  recv={}/{}  avg={}ns  med={}ns  p99={}ns  min={}ns\n",
            n, total_expected, avg, med, p99, latencies[0]);
    } else {
        println!("  No messages received\n");
    }
}

fn bench_horus_xproc_spmc(num_pubs: usize, num_subs: usize) {
    // Fork subscriber children that each count messages
    let mut child_pids = Vec::new();
    for s in 0..num_subs {
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            // Child subscriber
            std::thread::sleep(std::time::Duration::from_millis(200));
            let topic: Topic<u64> = Topic::new("bench.horus.xproc.spmc").expect("topic");
            let mut count = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(10);
            while count < ITERATIONS && Instant::now() < deadline {
                if topic.recv().is_some() {
                    count += 1;
                } else {
                    std::hint::spin_loop();
                }
            }
            std::process::exit(0);
        }
        child_pids.push(pid);
    }

    // Parent is publisher + measurer
    std::thread::sleep(std::time::Duration::from_millis(400));
    let topic: Topic<u64> = Topic::new("bench.horus.xproc.spmc").expect("topic");

    let start = Instant::now();
    for i in 0..ITERATIONS {
        topic.send(i as u64);
    }
    let elapsed = start.elapsed();

    for pid in child_pids {
        unsafe { libc::waitpid(pid, std::ptr::null_mut(), 0); }
    }

    let throughput = ITERATIONS as f64 / elapsed.as_secs_f64();
    println!("  sent={}  time={:.1}ms  throughput={:.2}M/s\n",
        ITERATIONS, elapsed.as_millis(), throughput / 1e6);
}

fn bench_iox2_xproc_mpmc(num_pubs: usize, num_subs: usize) {
    let mut child_pids = Vec::new();
    for p in 0..num_pubs {
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            std::thread::sleep(std::time::Duration::from_millis(300));
            let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
            let service = node
                .service_builder(&"bench/iox2/xproc/mpmc".try_into().unwrap())
                .publish_subscribe::<u64>()
                .max_publishers(4)
                .max_subscribers(4)
                .enable_safe_overflow(true)
                .open_or_create()
                .expect("service");
            let publisher = service.publisher_builder().create().expect("pub");
            for i in 0..ITERATIONS {
                let _ = publisher.send_copy((p * ITERATIONS + i) as u64);
            }
            std::process::exit(0);
        }
        child_pids.push(pid);
    }

    // Parent subscriber
    std::thread::sleep(std::time::Duration::from_millis(200));
    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
    let service = node
        .service_builder(&"bench/iox2/xproc/mpmc".try_into().unwrap())
        .publish_subscribe::<u64>()
        .max_publishers(4)
        .max_subscribers(4)
        .enable_safe_overflow(true)
        .open_or_create()
        .expect("service");
    let subscriber = service.subscriber_builder().create().expect("sub");

    let total_expected = ITERATIONS * num_pubs;
    let mut received = 0;
    let mut latencies = Vec::new();
    let deadline = Instant::now() + std::time::Duration::from_secs(15);

    while received < total_expected && Instant::now() < deadline {
        let t0 = Instant::now();
        match subscriber.receive() {
            Ok(Some(_)) => {
                latencies.push(t0.elapsed().as_nanos() as u64);
                received += 1;
            }
            _ => { std::hint::spin_loop(); }
        }
    }

    for pid in child_pids {
        unsafe { libc::waitpid(pid, std::ptr::null_mut(), 0); }
    }

    if !latencies.is_empty() {
        latencies.sort();
        let n = latencies.len();
        let avg = latencies.iter().sum::<u64>() / n as u64;
        let med = latencies[n / 2];
        let p99 = latencies[(n as f64 * 0.99) as usize];
        println!("  recv={}/{}  avg={}ns  med={}ns  p99={}ns  min={}ns\n",
            n, total_expected, avg, med, p99, latencies[0]);
    } else {
        println!("  No messages received\n");
    }
}

fn bench_iox2_xproc_spmc(num_pubs: usize, num_subs: usize) {
    let mut child_pids = Vec::new();
    for s in 0..num_subs {
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            std::thread::sleep(std::time::Duration::from_millis(200));
            let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
            let service = node
                .service_builder(&"bench/iox2/xproc/spmc".try_into().unwrap())
                .publish_subscribe::<u64>()
                .max_publishers(4)
                .max_subscribers(4)
                .enable_safe_overflow(true)
                .open_or_create()
                .expect("service");
            let subscriber = service.subscriber_builder().create().expect("sub");
            let mut count = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(10);
            while count < ITERATIONS && Instant::now() < deadline {
                match subscriber.receive() {
                    Ok(Some(_)) => { count += 1; }
                    _ => { std::hint::spin_loop(); }
                }
            }
            std::process::exit(0);
        }
        child_pids.push(pid);
    }

    // Parent publisher
    std::thread::sleep(std::time::Duration::from_millis(400));
    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
    let service = node
        .service_builder(&"bench/iox2/xproc/spmc".try_into().unwrap())
        .publish_subscribe::<u64>()
        .max_publishers(4)
        .max_subscribers(4)
        .enable_safe_overflow(true)
        .open_or_create()
        .expect("service");
    let publisher = service.publisher_builder().create().expect("pub");

    let start = Instant::now();
    for i in 0..ITERATIONS {
        let _ = publisher.send_copy(i as u64);
    }
    let elapsed = start.elapsed();

    for pid in child_pids {
        unsafe { libc::waitpid(pid, std::ptr::null_mut(), 0); }
    }

    let throughput = ITERATIONS as f64 / elapsed.as_secs_f64();
    println!("  sent={}  time={:.1}ms  throughput={:.2}M/s\n",
        ITERATIONS, elapsed.as_millis(), throughput / 1e6);
}
