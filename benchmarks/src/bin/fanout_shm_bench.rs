//! Cross-process FanoutShm benchmark
//!
//! Tests ShmFanoutRing performance across real process boundaries using fork().
//! Measures latency and throughput for 2P/1S and 2P/2S cross-process MPMC.
//!
//! Run: cargo run --release -p horus_benchmarks --bin fanout_shm_bench --no-default-features

use std::time::Instant;

use horus_core::communication::Topic;

const ITERATIONS: usize = 50_000;

fn main() {
    // Clean stale SHM
    let _ = std::fs::remove_dir_all("/dev/shm/horus_default");

    println!("==========================================================");
    println!("  Cross-Process FanoutShm MPMC Benchmark");
    println!("  {} iterations per publisher", ITERATIONS);
    println!("==========================================================\n");

    println!("--- horus FanoutShm 2P/1S cross-process (latency) ---");
    bench_fanout_shm_mpmc(2, 1);

    // Clean between tests
    let _ = std::fs::remove_dir_all("/dev/shm/horus_default");
    std::thread::sleep(std::time::Duration::from_millis(100));

    println!("--- horus FanoutShm 1P/2S cross-process (throughput) ---");
    bench_fanout_shm_spmc(1, 2);

    // Clean between tests
    let _ = std::fs::remove_dir_all("/dev/shm/horus_default");
    std::thread::sleep(std::time::Duration::from_millis(100));

    println!("--- horus FanoutShm 2P/2S cross-process (latency) ---");
    bench_fanout_shm_mpmc(2, 2);
}

fn bench_fanout_shm_mpmc(num_pubs: usize, num_subs: usize) {
    // Fork subscriber children first so they're ready
    let mut sub_pids = Vec::new();
    for _ in 0..num_subs {
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            // Child subscriber
            std::thread::sleep(std::time::Duration::from_millis(200));
            let topic: Topic<u64> = Topic::new("bench.fanout.shm.mpmc").expect("topic");
            let mut count = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(15);
            while count < ITERATIONS * num_pubs && Instant::now() < deadline {
                if topic.recv().is_some() {
                    count += 1;
                } else {
                    topic.check_migration_now();
                    std::hint::spin_loop();
                }
            }
            std::process::exit(0);
        }
        sub_pids.push(pid);
    }

    // Fork publisher children
    let mut pub_pids = Vec::new();
    for p in 0..num_pubs {
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            // Child publisher — wait for subscribers to set up
            std::thread::sleep(std::time::Duration::from_millis(500));
            let topic: Topic<u64> = Topic::new("bench.fanout.shm.mpmc").expect("topic");
            std::thread::sleep(std::time::Duration::from_millis(300));
            topic.check_migration_now();
            std::thread::sleep(std::time::Duration::from_millis(100));

            for i in 0..ITERATIONS {
                topic.send((p * ITERATIONS + i) as u64);
            }
            std::process::exit(0);
        }
        pub_pids.push(pid);
    }

    // Parent: also subscribe and measure latency
    std::thread::sleep(std::time::Duration::from_millis(200));
    let topic: Topic<u64> = Topic::new("bench.fanout.shm.mpmc").expect("topic");
    std::thread::sleep(std::time::Duration::from_millis(600));
    topic.check_migration_now();

    let total_expected = ITERATIONS * num_pubs;
    let mut received = 0;
    let mut latencies = Vec::with_capacity(total_expected);
    let deadline = Instant::now() + std::time::Duration::from_secs(15);
    let mut last_check = Instant::now();

    while received < total_expected && Instant::now() < deadline {
        let t0 = Instant::now();
        if topic.recv().is_some() {
            latencies.push(t0.elapsed().as_nanos() as u64);
            received += 1;
        } else {
            std::hint::spin_loop();
            if last_check.elapsed() > std::time::Duration::from_millis(50) {
                topic.check_migration_now();
                last_check = Instant::now();
            }
        }
    }

    // Wait for all children
    for pid in pub_pids.iter().chain(sub_pids.iter()) {
        unsafe { libc::waitpid(*pid, std::ptr::null_mut(), 0); }
    }

    if !latencies.is_empty() {
        latencies.sort();
        let n = latencies.len();
        let avg = latencies.iter().sum::<u64>() / n as u64;
        let med = latencies[n / 2];
        let p99 = latencies[(n as f64 * 0.99) as usize];
        println!("  {}P/{}S  recv={}/{}  avg={}ns  med={}ns  p99={}ns  min={}ns\n",
            num_pubs, num_subs, n, total_expected, avg, med, p99, latencies[0]);
    } else {
        println!("  {}P/{}S  No messages received\n", num_pubs, num_subs);
    }
}

fn bench_fanout_shm_spmc(num_pubs: usize, num_subs: usize) {
    // Fork subscriber children
    let mut child_pids = Vec::new();
    for _ in 0..num_subs {
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            std::thread::sleep(std::time::Duration::from_millis(200));
            let topic: Topic<u64> = Topic::new("bench.fanout.shm.spmc").expect("topic");
            let mut count = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(10);
            while count < ITERATIONS && Instant::now() < deadline {
                if topic.recv().is_some() {
                    count += 1;
                } else {
                    topic.check_migration_now();
                    std::hint::spin_loop();
                }
            }
            std::process::exit(0);
        }
        child_pids.push(pid);
    }

    // Parent is publisher + measurer
    std::thread::sleep(std::time::Duration::from_millis(600));
    let topic: Topic<u64> = Topic::new("bench.fanout.shm.spmc").expect("topic");
    std::thread::sleep(std::time::Duration::from_millis(200));
    topic.check_migration_now();

    let start = Instant::now();
    for i in 0..ITERATIONS {
        topic.send(i as u64);
    }
    let elapsed = start.elapsed();

    for pid in child_pids {
        unsafe { libc::waitpid(pid, std::ptr::null_mut(), 0); }
    }

    let throughput = ITERATIONS as f64 / elapsed.as_secs_f64();
    println!("  {}P/{}S  sent={}  time={:.1}ms  throughput={:.2}M/s\n",
        num_pubs, num_subs, ITERATIONS, elapsed.as_millis(), throughput / 1e6);
}
