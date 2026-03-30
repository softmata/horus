//! horus Topic<T> vs iceoryx2 pub/sub latency benchmark
//!
//! Measures same-thread send/recv latency and throughput.
//! Payload sizes: 16B, 1KB, 4KB
//!
//! Run: cargo run --release -p horus_benchmarks --bin iceoryx2_comparison --features iceoryx2

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Instant;

use serde::{Deserialize, Serialize};

use horus_core::communication::Topic;
use iceoryx2::prelude::*;

// POD structs for horus (need Serialize/Deserialize for Topic<T> bounds,
// but horus auto-detects POD and skips actual serialization)
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct HorusData1K {
    #[serde(with = "serde_arrays")]
    data: [u64; 128],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct HorusData4K {
    #[serde(with = "serde_arrays")]
    data: [u64; 512],
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const WARMUP: usize = 1000;
const ITERATIONS: usize = 100_000;

// ---------------------------------------------------------------------------
// Stats
// ---------------------------------------------------------------------------

fn compute_stats(mut latencies: Vec<u64>) -> (u64, u64, u64, u64, u64) {
    latencies.sort_unstable();
    let n = latencies.len();
    let sum: u64 = latencies.iter().sum();
    let min = latencies[0];
    let avg = sum / n as u64;
    let median = latencies[n / 2];
    let p99 = latencies[(n as f64 * 0.99) as usize];
    let max = latencies[n - 1];
    (min, avg, median, p99, max)
}

fn print_row(label: &str, min: u64, avg: u64, median: u64, p99: u64, max: u64) {
    println!(
        "  {:<35} min={:>6}  avg={:>6}  med={:>6}  p99={:>7}  max={:>8}",
        label, min, avg, median, p99, max
    );
}

// ---------------------------------------------------------------------------
// horus benchmarks — use u64 as payload (simplest POD)
// ---------------------------------------------------------------------------

fn bench_horus_u64() -> (u64, u64, u64, u64, u64) {
    let topic: Topic<u64> = Topic::new("bench.horus.u64").expect("topic");

    for _ in 0..WARMUP {
        topic.send(42u64);
        let _ = topic.recv();
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for i in 0..ITERATIONS {
        let start = Instant::now();
        topic.send(i as u64);
        let _ = topic.recv();
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_horus_1kb() -> (u64, u64, u64, u64, u64) {
    let topic: Topic<HorusData1K> = Topic::new("bench.horus.1kb.pod").expect("topic");
    let msg = HorusData1K { data: [0u64; 128] };

    for _ in 0..WARMUP {
        topic.send(msg);
        let _ = topic.recv();
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for _ in 0..ITERATIONS {
        let start = Instant::now();
        topic.send(msg);
        let _ = topic.recv();
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_horus_4kb() -> (u64, u64, u64, u64, u64) {
    let topic: Topic<HorusData4K> = Topic::new("bench.horus.4kb.pod").expect("topic");
    let msg = HorusData4K { data: [0u64; 512] };

    for _ in 0..WARMUP {
        topic.send(msg);
        let _ = topic.recv();
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for _ in 0..ITERATIONS {
        let start = Instant::now();
        topic.send(msg);
        let _ = topic.recv();
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_horus_throughput_u64() -> f64 {
    let topic: Topic<u64> = Topic::new("bench.horus.tp.u64").expect("topic");
    // Drain as we go to avoid ring buffer full
    let n = 1_000_000usize;
    let start = Instant::now();
    for i in 0..n {
        topic.send(i as u64);
        // Drain periodically to avoid full ring
        if i % 8 == 7 {
            for _ in 0..8 {
                let _ = topic.recv();
            }
        }
    }
    n as f64 / start.elapsed().as_secs_f64()
}

// ---------------------------------------------------------------------------
// iceoryx2 benchmarks — use u64 as payload
// ---------------------------------------------------------------------------

fn bench_iox2_u64() -> (u64, u64, u64, u64, u64) {
    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");

    let service = node
        .service_builder(&"bench/iox2/u64".try_into().unwrap())
        .publish_subscribe::<u64>()
        .open_or_create()
        .expect("service");

    let publisher = service.publisher_builder().create().expect("pub");
    let subscriber = service.subscriber_builder().create().expect("sub");

    for _ in 0..WARMUP {
        publisher.send_copy(42u64).unwrap();
        while subscriber.receive().unwrap().is_none() {
            std::hint::spin_loop();
        }
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for i in 0..ITERATIONS {
        let start = Instant::now();
        publisher.send_copy(i as u64).unwrap();
        while subscriber.receive().unwrap().is_none() {
            std::hint::spin_loop();
        }
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_iox2_1kb() -> (u64, u64, u64, u64, u64) {
    #[repr(C)]
    #[derive(Debug, Clone, Copy)]
    struct Data1K {
        data: [u64; 128],
    }
    impl Default for Data1K {
        fn default() -> Self {
            Self { data: [0u64; 128] }
        }
    }
    unsafe impl ZeroCopySend for Data1K {}

    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");

    let service = node
        .service_builder(&"bench/iox2/1kb".try_into().unwrap())
        .publish_subscribe::<Data1K>()
        .open_or_create()
        .expect("service");

    let publisher = service.publisher_builder().create().expect("pub");
    let subscriber = service.subscriber_builder().create().expect("sub");
    let msg = Data1K::default();

    for _ in 0..WARMUP {
        publisher.send_copy(msg).unwrap();
        while subscriber.receive().unwrap().is_none() {
            std::hint::spin_loop();
        }
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for _ in 0..ITERATIONS {
        let start = Instant::now();
        publisher.send_copy(msg).unwrap();
        while subscriber.receive().unwrap().is_none() {
            std::hint::spin_loop();
        }
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_iox2_4kb() -> (u64, u64, u64, u64, u64) {
    #[repr(C)]
    #[derive(Debug, Clone, Copy)]
    struct Data4K {
        data: [u64; 512],
    }
    impl Default for Data4K {
        fn default() -> Self {
            Self { data: [0u64; 512] }
        }
    }
    unsafe impl ZeroCopySend for Data4K {}

    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");

    let service = node
        .service_builder(&"bench/iox2/4kb".try_into().unwrap())
        .publish_subscribe::<Data4K>()
        .open_or_create()
        .expect("service");

    let publisher = service.publisher_builder().create().expect("pub");
    let subscriber = service.subscriber_builder().create().expect("sub");
    let msg = Data4K::default();

    for _ in 0..WARMUP {
        publisher.send_copy(msg).unwrap();
        while subscriber.receive().unwrap().is_none() {
            std::hint::spin_loop();
        }
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for _ in 0..ITERATIONS {
        let start = Instant::now();
        publisher.send_copy(msg).unwrap();
        while subscriber.receive().unwrap().is_none() {
            std::hint::spin_loop();
        }
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_iox2_throughput_u64() -> f64 {
    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");

    let service = node
        .service_builder(&"bench/iox2/tp/u64".try_into().unwrap())
        .publish_subscribe::<u64>()
        .open_or_create()
        .expect("service");

    let publisher = service.publisher_builder().create().expect("pub");

    let n = 1_000_000usize;
    let start = Instant::now();
    for i in 0..n {
        publisher.send_copy(i as u64).unwrap();
    }
    n as f64 / start.elapsed().as_secs_f64()
}

// ---------------------------------------------------------------------------
// Cross-thread benchmarks (forces SHM/atomic backends)
// ---------------------------------------------------------------------------

fn bench_horus_cross_thread_u64() -> (u64, u64, u64, u64, u64) {
    let ready = Arc::new(AtomicBool::new(false));
    let done = Arc::new(AtomicBool::new(false));

    let ready_pub = ready.clone();
    let done_pub = done.clone();

    // Publisher thread
    let pub_handle = thread::spawn(move || {
        let topic: Topic<u64> = Topic::new("bench.horus.ct.u64").expect("topic");
        ready_pub.store(true, Ordering::Release);

        // Wait for subscriber to be ready
        while !ready_pub.load(Ordering::Acquire) {}
        std::thread::sleep(std::time::Duration::from_millis(50));

        for i in 0..(WARMUP + ITERATIONS) as u64 {
            topic.send(i);
            // Pace to avoid overwhelming ring buffer
            while topic.recv().is_none() {
                std::hint::spin_loop();
            }
        }
        done_pub.store(true, Ordering::Release);
    });

    // Subscriber thread — measures roundtrip on its own topic
    let sub_handle = thread::spawn(move || {
        let topic: Topic<u64> = Topic::new("bench.horus.ct.u64.sub").expect("topic");
        // Give publisher time to create topic first
        std::thread::sleep(std::time::Duration::from_millis(100));

        let mut latencies = Vec::with_capacity(ITERATIONS);

        // Use a separate ping-pong approach:
        // Publisher sends on topic A, subscriber reads from topic A
        // This forces cross-thread atomic path
        let pub_topic: Topic<u64> = Topic::new("bench.horus.ct.u64").expect("topic");

        for i in 0..(WARMUP + ITERATIONS) {
            let start = Instant::now();
            pub_topic.send(i as u64);
            while pub_topic.recv().is_none() {
                std::hint::spin_loop();
            }
            if i >= WARMUP {
                latencies.push(start.elapsed().as_nanos() as u64);
            }
        }

        latencies
    });

    // Actually — the above approach has both threads on same Topic which IS cross-thread.
    // But both threads call send+recv on their own Topic clone, which horus detects as
    // same-thread (each Topic instance is thread-local).
    //
    // For TRUE cross-thread: publisher sends, subscriber on different thread receives.
    // Let me use a proper ping-pong pattern.

    pub_handle.join().unwrap();
    let latencies = sub_handle.join().unwrap();

    if latencies.is_empty() {
        return (0, 0, 0, 0, 0);
    }
    compute_stats(latencies)
}

/// Cross-thread ping-pong: thread A sends on topic1, thread B receives on topic1,
/// thread B sends on topic2, thread A receives on topic2. Measures full roundtrip.
fn bench_horus_cross_thread_pingpong() -> (u64, u64, u64, u64, u64) {
    let barrier = Arc::new(std::sync::Barrier::new(2));

    let barrier_a = barrier.clone();
    let handle_a = thread::spawn(move || {
        let tx: Topic<u64> = Topic::new("bench.horus.pp.a2b").expect("topic");
        let rx: Topic<u64> = Topic::new("bench.horus.pp.b2a").expect("topic");

        barrier_a.wait();

        let mut latencies = Vec::with_capacity(ITERATIONS);

        for i in 0..(WARMUP + ITERATIONS) as u64 {
            let start = Instant::now();
            tx.send(i);
            // Wait for pong
            while rx.recv().is_none() {
                std::hint::spin_loop();
            }
            if i >= WARMUP as u64 {
                latencies.push(start.elapsed().as_nanos() as u64);
            }
        }

        latencies
    });

    let barrier_b = barrier.clone();
    let handle_b = thread::spawn(move || {
        let rx: Topic<u64> = Topic::new("bench.horus.pp.a2b").expect("topic");
        let tx: Topic<u64> = Topic::new("bench.horus.pp.b2a").expect("topic");

        barrier_b.wait();

        for _ in 0..(WARMUP + ITERATIONS) {
            // Wait for ping
            while rx.recv().is_none() {
                std::hint::spin_loop();
            }
            // Send pong
            tx.send(1);
        }
    });

    let latencies = handle_a.join().unwrap();
    handle_b.join().unwrap();

    if latencies.is_empty() {
        return (0, 0, 0, 0, 0);
    }
    // Divide by 2 for one-way latency
    let one_way: Vec<u64> = latencies.iter().map(|&l| l / 2).collect();
    compute_stats(one_way)
}

/// Cross-thread ping-pong for iceoryx2
fn bench_iox2_cross_thread_pingpong() -> (u64, u64, u64, u64, u64) {
    let barrier = Arc::new(std::sync::Barrier::new(2));

    let barrier_a = barrier.clone();
    let handle_a = thread::spawn(move || {
        let node = NodeBuilder::new().create::<ipc::Service>().expect("node");

        let svc_a2b = node
            .service_builder(&"bench/iox2/pp/a2b".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");
        let svc_b2a = node
            .service_builder(&"bench/iox2/pp/b2a".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");

        let publisher = svc_a2b.publisher_builder().create().expect("pub");
        let subscriber = svc_b2a.subscriber_builder().create().expect("sub");

        barrier_a.wait();

        let mut latencies = Vec::with_capacity(ITERATIONS);

        for i in 0..(WARMUP + ITERATIONS) as u64 {
            let start = Instant::now();
            publisher.send_copy(i).unwrap();
            // Wait for pong
            while subscriber.receive().unwrap().is_none() {
                std::hint::spin_loop();
            }
            if i >= WARMUP as u64 {
                latencies.push(start.elapsed().as_nanos() as u64);
            }
        }

        latencies
    });

    let barrier_b = barrier.clone();
    let handle_b = thread::spawn(move || {
        let node = NodeBuilder::new().create::<ipc::Service>().expect("node");

        let svc_a2b = node
            .service_builder(&"bench/iox2/pp/a2b".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");
        let svc_b2a = node
            .service_builder(&"bench/iox2/pp/b2a".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");

        let subscriber = svc_a2b.subscriber_builder().create().expect("sub");
        let publisher = svc_b2a.publisher_builder().create().expect("pub");

        barrier_b.wait();

        for _ in 0..(WARMUP + ITERATIONS) {
            while subscriber.receive().unwrap().is_none() {
                std::hint::spin_loop();
            }
            publisher.send_copy(1u64).unwrap();
        }
    });

    let latencies = handle_a.join().unwrap();
    handle_b.join().unwrap();

    if latencies.is_empty() {
        return (0, 0, 0, 0, 0);
    }
    let one_way: Vec<u64> = latencies.iter().map(|&l| l / 2).collect();
    compute_stats(one_way)
}

/// Cross-process benchmark using fork()
fn bench_cross_process() {
    println!("--- Cross-Process (fork) Ping-Pong ---");
    println!();

    // horus cross-process
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        // Child: pong responder
        let rx: Topic<u64> = Topic::new("bench.horus.xproc.a2b").expect("topic");
        let tx: Topic<u64> = Topic::new("bench.horus.xproc.b2a").expect("topic");
        std::thread::sleep(std::time::Duration::from_millis(200));

        for _ in 0..(WARMUP + ITERATIONS) {
            while rx.recv().is_none() {
                std::hint::spin_loop();
            }
            tx.send(1u64);
        }
        std::process::exit(0);
    } else {
        // Parent: ping sender + measurer
        let tx: Topic<u64> = Topic::new("bench.horus.xproc.a2b").expect("topic");
        let rx: Topic<u64> = Topic::new("bench.horus.xproc.b2a").expect("topic");
        std::thread::sleep(std::time::Duration::from_millis(300));

        let mut latencies = Vec::with_capacity(ITERATIONS);
        for i in 0..(WARMUP + ITERATIONS) as u64 {
            let start = Instant::now();
            tx.send(i);
            while rx.recv().is_none() {
                std::hint::spin_loop();
            }
            if i >= WARMUP as u64 {
                latencies.push(start.elapsed().as_nanos() as u64);
            }
        }

        // Wait for child
        unsafe {
            let mut status = 0;
            libc::waitpid(pid, &mut status, 0);
        }

        let one_way: Vec<u64> = latencies.iter().map(|&l| l / 2).collect();
        let (min, avg, med, p99, max) = compute_stats(one_way);
        print_row("horus  cross-process u64", min, avg, med, p99, max);
    }

    // iceoryx2 cross-process
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        // Child: pong responder
        std::thread::sleep(std::time::Duration::from_millis(200));
        let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
        let svc_a2b = node
            .service_builder(&"bench/iox2/xproc/a2b".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");
        let svc_b2a = node
            .service_builder(&"bench/iox2/xproc/b2a".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");

        let subscriber = svc_a2b.subscriber_builder().create().expect("sub");
        let publisher = svc_b2a.publisher_builder().create().expect("pub");

        for _ in 0..(WARMUP + ITERATIONS) {
            while subscriber.receive().unwrap().is_none() {
                std::hint::spin_loop();
            }
            publisher.send_copy(1u64).unwrap();
        }
        std::process::exit(0);
    } else {
        // Parent: ping sender + measurer
        std::thread::sleep(std::time::Duration::from_millis(300));
        let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
        let svc_a2b = node
            .service_builder(&"bench/iox2/xproc/a2b".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");
        let svc_b2a = node
            .service_builder(&"bench/iox2/xproc/b2a".try_into().unwrap())
            .publish_subscribe::<u64>()
            .open_or_create()
            .expect("service");

        let publisher = svc_a2b.publisher_builder().create().expect("pub");
        let subscriber = svc_b2a.subscriber_builder().create().expect("sub");

        let mut latencies = Vec::with_capacity(ITERATIONS);
        for i in 0..(WARMUP + ITERATIONS) as u64 {
            let start = Instant::now();
            publisher.send_copy(i).unwrap();
            while subscriber.receive().unwrap().is_none() {
                std::hint::spin_loop();
            }
            if i >= WARMUP as u64 {
                latencies.push(start.elapsed().as_nanos() as u64);
            }
        }

        unsafe {
            let mut status = 0;
            libc::waitpid(pid, &mut status, 0);
        }

        let one_way: Vec<u64> = latencies.iter().map(|&l| l / 2).collect();
        let (min, avg, med, p99, max) = compute_stats(one_way);
        print_row("iox2   cross-process u64", min, avg, med, p99, max);
    }

    println!();
}

// ---------------------------------------------------------------------------
// MPMC benchmarks — multiple publishers, multiple subscribers
// ---------------------------------------------------------------------------

fn bench_horus_mpmc(num_pubs: usize, num_subs: usize) -> (u64, u64, u64, u64, u64) {
    let msgs_per_pub = 10_000;
    let barrier = Arc::new(std::sync::Barrier::new(num_pubs + num_subs));
    let total_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));

    // Subscriber threads — each measures its own receive latency
    let mut sub_handles = Vec::new();
    for s in 0..num_subs {
        let barrier_s = barrier.clone();
        let total_sent_s = total_sent.clone();
        sub_handles.push(thread::spawn(move || {
            let topic: Topic<u64> = Topic::new("bench.horus.mpmc").expect("topic");
            barrier_s.wait();

            let mut latencies = Vec::new();
            let expected = msgs_per_pub * num_pubs;
            let mut received = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(30);

            while received < expected / num_subs && Instant::now() < deadline {
                let t0 = Instant::now();
                if topic.recv().is_some() {
                    latencies.push(t0.elapsed().as_nanos() as u64);
                    received += 1;
                } else {
                    std::hint::spin_loop();
                }
            }

            latencies
        }));
    }

    // Publisher threads
    let mut pub_handles = Vec::new();
    for p in 0..num_pubs {
        let barrier_p = barrier.clone();
        let total_sent_p = total_sent.clone();
        pub_handles.push(thread::spawn(move || {
            let topic: Topic<u64> = Topic::new("bench.horus.mpmc").expect("topic");
            barrier_p.wait();

            for i in 0..msgs_per_pub {
                topic.send((p * msgs_per_pub + i) as u64);
                total_sent_p.fetch_add(1, Ordering::Relaxed);
            }
        }));
    }

    for h in pub_handles {
        h.join().unwrap();
    }

    let mut all_latencies = Vec::new();
    for h in sub_handles {
        all_latencies.extend(h.join().unwrap());
    }

    if all_latencies.is_empty() {
        return (0, 0, 0, 0, 0);
    }
    compute_stats(all_latencies)
}

fn bench_iox2_mpmc(num_pubs: usize, num_subs: usize) -> (u64, u64, u64, u64, u64) {
    let msgs_per_pub = 10_000; // Reduced for MPMC (iceoryx2 is slow to set up)
    let barrier = Arc::new(std::sync::Barrier::new(num_pubs + num_subs));

    // Subscriber threads
    let mut sub_handles = Vec::new();
    for s in 0..num_subs {
        let barrier_s = barrier.clone();
        sub_handles.push(thread::spawn(move || {
            let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
            let service = node
                .service_builder(&"bench/iox2/mpmc".try_into().unwrap())
                .publish_subscribe::<u64>()
                .max_publishers(num_pubs + 1)
                .max_subscribers(num_subs + 1)
                .enable_safe_overflow(true)
                .open_or_create()
                .expect("service");

            let subscriber = service.subscriber_builder().create().expect("sub");
            barrier_s.wait();

            let mut latencies = Vec::new();
            let expected = msgs_per_pub * num_pubs;
            let mut received = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(30);

            while received < expected / num_subs && Instant::now() < deadline {
                let t0 = Instant::now();
                match subscriber.receive() {
                    Ok(Some(_)) => {
                        latencies.push(t0.elapsed().as_nanos() as u64);
                        received += 1;
                    }
                    _ => {
                        std::hint::spin_loop();
                    }
                }
            }

            latencies
        }));
    }

    // Publisher threads
    let mut pub_handles = Vec::new();
    for p in 0..num_pubs {
        let barrier_p = barrier.clone();
        pub_handles.push(thread::spawn(move || {
            let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
            let service = node
                .service_builder(&"bench/iox2/mpmc".try_into().unwrap())
                .publish_subscribe::<u64>()
                .max_publishers(num_pubs + 1)
                .max_subscribers(num_subs + 1)
                .enable_safe_overflow(true)
                .open_or_create()
                .expect("service");

            let publisher = service.publisher_builder().create().expect("pub");
            barrier_p.wait();

            for i in 0..msgs_per_pub {
                let _ = publisher.send_copy((p * msgs_per_pub + i) as u64);
            }
        }));
    }

    for h in pub_handles {
        h.join().unwrap();
    }

    let mut all_latencies = Vec::new();
    for h in sub_handles {
        all_latencies.extend(h.join().unwrap());
    }

    if all_latencies.is_empty() {
        return (0, 0, 0, 0, 0);
    }
    compute_stats(all_latencies)
}

// ---------------------------------------------------------------------------
// FanoutRing benchmark (contention-free MPMC)
// ---------------------------------------------------------------------------

fn bench_horus_fanout_mpmc(num_pubs: usize, num_subs: usize) -> (u64, u64, u64, u64, u64) {
    use horus_core::communication::topic::fanout::FanoutRing;
    use std::sync::Arc;

    let msgs_per_pub = 10_000;
    let ring = Arc::new(FanoutRing::<u64>::new(num_pubs + 1, num_subs + 1, 256));

    // Register endpoints
    let mut pub_ids = Vec::new();
    let mut sub_ids = Vec::new();
    for _ in 0..num_pubs {
        pub_ids.push(ring.register_publisher());
    }
    for _ in 0..num_subs {
        sub_ids.push(ring.register_subscriber());
    }

    let barrier = Arc::new(std::sync::Barrier::new(num_pubs + num_subs));

    // Subscriber threads — measure receive latency
    let mut sub_handles = Vec::new();
    for &sid in &sub_ids {
        let ring = ring.clone();
        let barrier = barrier.clone();
        sub_handles.push(thread::spawn(move || {
            barrier.wait();
            let mut latencies = Vec::new();
            let expected = msgs_per_pub * num_pubs;
            let mut received = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(30);

            while received < expected && Instant::now() < deadline {
                let t0 = Instant::now();
                if ring.recv_as(sid).is_some() {
                    latencies.push(t0.elapsed().as_nanos() as u64);
                    received += 1;
                } else {
                    std::hint::spin_loop();
                }
            }
            latencies
        }));
    }

    // Publisher threads
    let mut pub_handles = Vec::new();
    for &pid in &pub_ids {
        let ring = ring.clone();
        let barrier = barrier.clone();
        pub_handles.push(thread::spawn(move || {
            barrier.wait();
            for i in 0..msgs_per_pub {
                let _ = ring.send_as((pid * msgs_per_pub + i) as u64, pid);
            }
        }));
    }

    for h in pub_handles {
        h.join().unwrap();
    }

    let mut all_latencies = Vec::new();
    for h in sub_handles {
        all_latencies.extend(h.join().unwrap());
    }

    if all_latencies.is_empty() {
        return (0, 0, 0, 0, 0);
    }
    compute_stats(all_latencies)
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    println!("==========================================================");
    println!("  horus Topic<T> vs iceoryx2 — IPC Latency Benchmark");
    println!("  {} iterations, {} warmup", ITERATIONS, WARMUP);
    println!("==========================================================");
    println!();
    println!("  All times in nanoseconds (ns)");
    println!();

    // --- u64 (8 bytes) ---
    println!("--- 8B payload (u64) ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_u64();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_u64();
    print_row("horus  Topic<u64>", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   PubSub<u64>", imin, iavg, imed, ip99, imax);
    let winner8 = if havg <= iavg { "horus" } else { "iox2" };
    let ratio8 = if havg <= iavg {
        iavg as f64 / havg.max(1) as f64
    } else {
        havg as f64 / iavg.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", winner8, ratio8);

    // --- 1KB ---
    println!("--- 1KB payload ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_1kb();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_1kb();
    print_row("horus  Topic<[u64;128]>", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   PubSub<[u64;128]>", imin, iavg, imed, ip99, imax);
    let winner1k = if havg <= iavg { "horus" } else { "iox2" };
    let ratio1k = if havg <= iavg {
        iavg as f64 / havg.max(1) as f64
    } else {
        havg as f64 / iavg.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", winner1k, ratio1k);

    // --- 4KB ---
    println!("--- 4KB payload ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_4kb();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_4kb();
    print_row("horus  Topic<[u64;512]>", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   PubSub<[u64;512]>", imin, iavg, imed, ip99, imax);
    let winner4k = if havg <= iavg { "horus" } else { "iox2" };
    let ratio4k = if havg <= iavg {
        iavg as f64 / havg.max(1) as f64
    } else {
        havg as f64 / iavg.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", winner4k, ratio4k);

    // --- Cross-thread (forces atomic/SHM backend) ---
    println!("--- Cross-Thread Ping-Pong (one-way latency) ---");
    println!("  (Forces horus into SpscIntra atomic backend)");
    println!();
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_cross_thread_pingpong();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_cross_thread_pingpong();
    print_row("horus  cross-thread u64", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   cross-thread u64", imin, iavg, imed, ip99, imax);
    let winner_ct = if havg <= iavg { "horus" } else { "iox2" };
    let ratio_ct = if havg <= iavg {
        iavg as f64 / havg.max(1) as f64
    } else {
        havg as f64 / iavg.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", winner_ct, ratio_ct);

    // --- Cross-process (true IPC via SHM) ---
    bench_cross_process();

    // --- MPMC ---
    println!("--- MPMC: 2 publishers, 2 subscribers ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_mpmc(2, 2);
    let (imin, iavg, imed, ip99, imax) = bench_iox2_mpmc(2, 2);
    print_row("horus  2P/2S", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   2P/2S", imin, iavg, imed, ip99, imax);
    let w22 = if havg <= iavg { "horus" } else { "iox2" };
    let r22 = if havg <= iavg {
        iavg as f64 / havg.max(1) as f64
    } else {
        havg as f64 / iavg.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", w22, r22);

    println!("--- MPMC: 4 publishers, 4 subscribers ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_mpmc(4, 4);
    let (imin, iavg, imed, ip99, imax) = bench_iox2_mpmc(4, 4);
    print_row("horus  4P/4S", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   4P/4S", imin, iavg, imed, ip99, imax);
    let w44 = if havg <= iavg { "horus" } else { "iox2" };
    let r44 = if havg <= iavg {
        iavg as f64 / havg.max(1) as f64
    } else {
        havg as f64 / iavg.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", w44, r44);

    // --- FanoutRing (contention-free MPMC) ---
    println!("--- FanoutRing (NEW): 2 publishers, 2 subscribers ---");
    let (fmin, favg, fmed, fp99, fmax) = bench_horus_fanout_mpmc(2, 2);
    print_row("horus  fanout 2P/2S", fmin, favg, fmed, fp99, fmax);
    print_row("iox2   2P/2S (from above)", imin, iavg, imed, ip99, imax);
    let fw22 = if favg <= iavg { "horus-fanout" } else { "iox2" };
    let fr22 = if favg <= iavg {
        iavg as f64 / favg.max(1) as f64
    } else {
        favg as f64 / iavg.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", fw22, fr22);

    println!("--- FanoutRing (NEW): 4 publishers, 4 subscribers ---");
    let (fmin, favg, fmed, fp99, fmax) = bench_horus_fanout_mpmc(4, 4);
    let (imin4, iavg4, imed4, ip994, imax4) = bench_iox2_mpmc(4, 4);
    print_row("horus  OLD mpmc 4P/4S", hmin, havg, hmed, hp99, hmax);
    print_row("horus  fanout 4P/4S", fmin, favg, fmed, fp99, fmax);
    print_row("iox2   4P/4S", imin4, iavg4, imed4, ip994, imax4);
    let fw44 = if favg <= iavg4 {
        "horus-fanout"
    } else {
        "iox2"
    };
    let fr44 = if favg <= iavg4 {
        iavg4 as f64 / favg.max(1) as f64
    } else {
        favg as f64 / iavg4.max(1) as f64
    };
    println!("  Winner: {} ({:.1}x faster)\n", fw44, fr44);

    // 8P/4S removed — iceoryx2 hits connection limits and hangs

    // --- Throughput ---
    println!("--- Send-only throughput (msgs/sec) ---");
    let ht = bench_horus_throughput_u64();
    let it = bench_iox2_throughput_u64();
    println!("  horus:  {:.2}M msgs/sec", ht / 1e6);
    println!("  iox2:   {:.2}M msgs/sec", it / 1e6);
    let tw = if ht >= it { "horus" } else { "iox2" };
    let tr = if ht >= it { ht / it } else { it / ht };
    println!("  Winner: {} ({:.1}x)\n", tw, tr);
}

// Cross-process MPMC not benchmarked yet — FanoutRing is intra-process only.
// For cross-process MPMC, horus uses MpmcShm (SHM CAS path).
