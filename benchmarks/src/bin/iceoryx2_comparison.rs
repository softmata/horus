//! horus Topic<T> vs iceoryx2 pub/sub latency benchmark
//!
//! Measures same-thread send/recv latency and throughput.
//! Payload sizes: 8B, 1KB, 4KB
//!
//! Run: cargo run --release -p horus_benchmarks --bin iceoryx2_comparison --features iceoryx2
//!
//! # Topology, and why it decides the result
//!
//! The same-thread sections used to be structurally unfair to iceoryx2, in
//! horus's favour, by a large factor:
//!
//! - the horus arm called `send` and `recv` on **one** `Topic` handle. A single
//!   handle that does both acquires `role == Both`, which selects horus's
//!   same-instance fast path: an inlined write and read through
//!   `cached_data_ptr` with no dispatch, no epoch check, and no ring
//!   publication. Both halves hit the same L1 line the same thread just wrote.
//! - the iceoryx2 arm used a real publisher and a real subscriber and spun
//!   `while receive().is_none()`, i.e. the full queue path.
//!
//! Those are not the same measurement, and the horus half is not a path any
//! two-node system takes: a publisher and a subscriber are never the same
//! handle. The horus arms below now use a separate publisher handle and
//! subscriber handle and spin until the message is actually received, which is
//! the same shape as the iceoryx2 arm.
//!
//! Even so, **every "same-thread" section measures one thread writing and then
//! reading a ring it just wrote**, for both libraries. That is a lower bound on
//! IPC cost, not IPC cost: there is no cross-core cache-line transfer in it.
//! The cross-thread and cross-process sections are the ones that measure a real
//! transfer; prefer those when quoting a number.
//!
//! # `l / 2` is an estimate, not a one-way latency
//!
//! The ping-pong sections divide a round trip by two. That is only a one-way
//! latency if the two legs are symmetric. A stall on one leg is reported as
//! half a stall on each, so RTT/2 systematically **understates the tail** —
//! exactly the figure of merit here. The raw RTT max is printed alongside.

use std::sync::atomic::Ordering;
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

/// Compare two arms on the tail first, then the median.
///
/// Every section used to print `Winner: X (Ny faster)` decided purely on the
/// **mean**. This project ranks on worst-case and jitter: a change that
/// improves the mean and widens the tail is a regression, and a single-number
/// "winner" from the mean cannot express that. There is no winner line any
/// more — the three ratios are printed and disagreement between them is the
/// interesting result, not something to collapse.
///
/// Arguments are `(median, p99, max)` for each arm.
fn print_comparison(horus: (u64, u64, u64), iox2: (u64, u64, u64)) {
    let ratio = |h: u64, i: u64| {
        if h == 0 {
            f64::NAN
        } else {
            i as f64 / h as f64
        }
    };
    println!(
        "  iox2/horus ratio:  median={:.2}x  p99={:.2}x  max={:.2}x  (>1 favours horus)",
        ratio(horus.0, iox2.0),
        ratio(horus.1, iox2.1),
        ratio(horus.2, iox2.2),
    );
    if ratio(horus.1, iox2.1) < 1.0 && ratio(horus.0, iox2.0) > 1.0 {
        println!("  NOTE: horus wins the median and loses the p99 — a tail regression.");
    }
    println!();
}

// ---------------------------------------------------------------------------
// horus benchmarks — use u64 as payload (simplest POD)
// ---------------------------------------------------------------------------

/// Same-thread publisher-handle -> subscriber-handle round trip.
///
/// Two handles, not one. A single handle doing both send and recv takes horus's
/// `role == Both` same-instance fast path, which bypasses dispatch entirely and
/// is not a path any publisher/subscriber pair can reach; measuring it beside
/// iceoryx2's real queue path is what made the horus column look several times
/// faster than it is.
///
/// The recv spins until the message actually arrives, so a delivery failure
/// hangs rather than being recorded as a very fast sample. The old
/// `let _ = topic.recv();` discarded the result: had recv returned `None` every
/// time — a total failure to deliver — the benchmark would have reported its
/// best numbers ever.
fn bench_horus_u64() -> (u64, u64, u64, u64, u64) {
    let tx: Topic<u64> = Topic::new("bench.horus.u64").expect("topic");
    let rx: Topic<u64> = Topic::new("bench.horus.u64").expect("topic");

    for _ in 0..WARMUP {
        tx.send(42u64);
        while rx.recv().is_none() {
            std::hint::spin_loop();
        }
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for i in 0..ITERATIONS {
        let start = Instant::now();
        tx.send(i as u64);
        while rx.recv().is_none() {
            std::hint::spin_loop();
        }
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_horus_1kb() -> (u64, u64, u64, u64, u64) {
    let tx: Topic<HorusData1K> = Topic::new("bench.horus.1kb.pod").expect("topic");
    let rx: Topic<HorusData1K> = Topic::new("bench.horus.1kb.pod").expect("topic");
    let msg = HorusData1K { data: [0u64; 128] };

    for _ in 0..WARMUP {
        tx.send(msg);
        while rx.recv().is_none() {
            std::hint::spin_loop();
        }
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for _ in 0..ITERATIONS {
        let start = Instant::now();
        tx.send(msg);
        while rx.recv().is_none() {
            std::hint::spin_loop();
        }
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

fn bench_horus_4kb() -> (u64, u64, u64, u64, u64) {
    let tx: Topic<HorusData4K> = Topic::new("bench.horus.4kb.pod").expect("topic");
    let rx: Topic<HorusData4K> = Topic::new("bench.horus.4kb.pod").expect("topic");
    let msg = HorusData4K { data: [0u64; 512] };

    for _ in 0..WARMUP {
        tx.send(msg);
        while rx.recv().is_none() {
            std::hint::spin_loop();
        }
    }

    let mut latencies = Vec::with_capacity(ITERATIONS);
    for _ in 0..ITERATIONS {
        let start = Instant::now();
        tx.send(msg);
        while rx.recv().is_none() {
            std::hint::spin_loop();
        }
        latencies.push(start.elapsed().as_nanos() as u64);
    }

    compute_stats(latencies)
}

/// Messages per second for a send-then-drain-every-8 loop.
///
/// NOT a send-only rate. The horus arm has always drained 8 messages for every
/// 8 sends inside the timed region, while the iceoryx2 arm ran a bare send loop
/// with no subscriber attached — so the two arms were timing different loops
/// and the section was labelled "send-only throughput" for the one that was
/// not. `bench_iox2_throughput_u64` now drains identically.
fn bench_horus_throughput_u64() -> f64 {
    let tx: Topic<u64> = Topic::new("bench.horus.tp.u64").expect("topic");
    let rx: Topic<u64> = Topic::new("bench.horus.tp.u64").expect("topic");
    // Drain as we go to avoid ring buffer full
    let n = 1_000_000usize;
    let start = Instant::now();
    for i in 0..n {
        tx.send(i as u64);
        // Drain periodically to avoid full ring
        if i % 8 == 7 {
            for _ in 0..8 {
                let _ = rx.recv();
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

/// Same loop shape as `bench_horus_throughput_u64`: send, and drain 8 every 8.
///
/// This arm used to be a bare send loop with no subscriber attached at all,
/// which times a different amount of work than the horus arm it was printed
/// against — and with no subscriber, iceoryx2 has nowhere to deliver to, so it
/// was not even doing the same job. The subscriber and the drain make the two
/// arms comparable.
fn bench_iox2_throughput_u64() -> f64 {
    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");

    let service = node
        .service_builder(&"bench/iox2/tp/u64".try_into().unwrap())
        .publish_subscribe::<u64>()
        .open_or_create()
        .expect("service");

    let publisher = service.publisher_builder().create().expect("pub");
    let subscriber = service.subscriber_builder().create().expect("sub");

    let n = 1_000_000usize;
    let start = Instant::now();
    for i in 0..n {
        publisher.send_copy(i as u64).unwrap();
        if i % 8 == 7 {
            for _ in 0..8 {
                let _ = subscriber.receive();
            }
        }
    }
    n as f64 / start.elapsed().as_secs_f64()
}

// ---------------------------------------------------------------------------
// Cross-thread benchmarks (forces SHM/atomic backends)
// ---------------------------------------------------------------------------

// An earlier `bench_horus_cross_thread_u64` lived here. It had both threads
// calling send+recv on their own `Topic` instance, which horus resolves to the
// thread-local backend — so it measured the same-thread path twice under a
// "cross-thread" label, as its own trailing comment admitted. It was never
// wired into `main`; `bench_horus_cross_thread_pingpong` below is the correct
// replacement (A sends on one topic, B receives on it and replies on another,
// which is the only shape that forces the cross-thread SpscShm backend).

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
    // ROUND TRIP, not halved.
    //
    // This used to return `l / 2` per sample under a "one-way latency" label.
    // Halving is only valid if the two legs are symmetric; a stall on one leg
    // is reported as half a stall on each, so the halved p99 and max understate
    // the tail — the figure this project ranks on. The round trip is what was
    // actually observed, so that is what is reported.
    compute_stats(latencies)
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
    // Round trip, not halved — see `bench_horus_cross_thread_pingpong`.
    compute_stats(latencies)
}

/// Cross-process benchmark using fork()
fn bench_cross_process() {
    println!("--- Cross-Process (fork) Ping-Pong (ROUND TRIP, not halved) ---");
    println!("  The only section here that measures a real cross-address-space transfer.");
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

        // Round trip, not halved — see `bench_horus_cross_thread_pingpong`.
        let (min, avg, med, p99, max) = compute_stats(latencies);
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

        // Round trip, not halved — see `bench_horus_cross_thread_pingpong`.
        let (min, avg, med, p99, max) = compute_stats(latencies);
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
    for _ in 0..num_subs {
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
                } else if total_sent_s.load(Ordering::Acquire) >= expected {
                    // Every publisher has finished and the ring still came up
                    // empty, so this subscriber's `expected / num_subs` quota is
                    // never going to arrive: the MPMC split is not guaranteed
                    // even, and ring overflow can drop messages outright. Stop
                    // here instead of spinning out the full 30s deadline, which
                    // is what `total_sent` was counting for.
                    break;
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
                // Release pairs with the subscribers' Acquire load: once a
                // subscriber observes the count reach `expected`, every send
                // that produced it is already visible, so an empty ring really
                // does mean nothing more is coming.
                total_sent_p.fetch_add(1, Ordering::Release);
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
    // Mirrors the horus arm's `total_sent`. Without it, this arm had no early
    // exit: when overflow dropped messages (and `enable_safe_overflow(true)`
    // means it does), every subscriber spun out the full 30 s deadline while
    // the horus arm returned as soon as the publishers were done. Two arms with
    // different stopping rules are not a like-for-like comparison.
    let total_sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));

    // Subscriber threads
    let mut sub_handles = Vec::new();
    for _ in 0..num_subs {
        let barrier_s = barrier.clone();
        let total_sent_s = total_sent.clone();
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
                    _ if total_sent_s.load(Ordering::Acquire) >= expected => {
                        // Every publisher has finished and the queue came up
                        // empty: the rest of this subscriber's quota is not
                        // coming. Same stopping rule as the horus arm.
                        break;
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
        let total_sent_p = total_sent.clone();
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
                total_sent_p.fetch_add(1, Ordering::Release);
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
    println!("  Ranked on p99 and max first, median second. There is deliberately no");
    println!("  single 'winner' line: it used to be decided on the MEAN, which cannot");
    println!("  express a change that improves the median and widens the tail.");
    println!();

    // Stale SHM from a previous run of this binary would otherwise be reused:
    // the topic names below are fixed strings with no per-run suffix, so a
    // segment left behind by an earlier run — complete with its registered but
    // dead participants — is what the next run opens.
    let _ = std::fs::remove_dir_all(horus_core::memory::shm_topics_dir());

    // --- u64 (8 bytes) ---
    println!("--- 8B payload (u64), SAME THREAD: pub handle -> sub handle ---");
    println!("  (one thread writes then reads its own ring; no cross-core transfer)");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_u64();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_u64();
    print_row("horus  Topic<u64>", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   PubSub<u64>", imin, iavg, imed, ip99, imax);
    print_comparison((hmed, hp99, hmax), (imed, ip99, imax));

    // --- 1KB ---
    println!("--- 1KB payload, SAME THREAD ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_1kb();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_1kb();
    print_row("horus  Topic<[u64;128]>", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   PubSub<[u64;128]>", imin, iavg, imed, ip99, imax);
    print_comparison((hmed, hp99, hmax), (imed, ip99, imax));

    // --- 4KB ---
    println!("--- 4KB payload, SAME THREAD ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_4kb();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_4kb();
    print_row("horus  Topic<[u64;512]>", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   PubSub<[u64;512]>", imin, iavg, imed, ip99, imax);
    print_comparison((hmed, hp99, hmax), (imed, ip99, imax));

    // --- Cross-thread (forces atomic/SHM backend) ---
    println!("--- Cross-Thread Ping-Pong (ROUND TRIP, not halved) ---");
    println!("  A sends on topic1, B replies on topic2. Values are full round trips:");
    println!("  halving them would report a one-sided stall as half a stall on each leg.");
    println!("  Backend is chosen by Topic::new at runtime and is not verified here.");
    println!();
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_cross_thread_pingpong();
    let (imin, iavg, imed, ip99, imax) = bench_iox2_cross_thread_pingpong();
    print_row("horus  cross-thread u64 RTT", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   cross-thread u64 RTT", imin, iavg, imed, ip99, imax);
    print_comparison((hmed, hp99, hmax), (imed, ip99, imax));

    // --- Cross-process (true IPC via SHM) ---
    bench_cross_process();

    // --- MPMC ---
    println!("--- MPMC: 2 publishers, 2 subscribers ---");
    println!("  These are recv() CALL COSTS on an already-backlogged queue, not");
    println!("  publish-to-receive latencies: publishers are joined before the");
    println!("  subscribers are, so most receives return from a full ring. Both arms");
    println!("  are measured the same way, so they compare, but neither is a latency.");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_mpmc(2, 2);
    let (imin, iavg, imed, ip99, imax) = bench_iox2_mpmc(2, 2);
    print_row("horus  2P/2S recv cost", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   2P/2S recv cost", imin, iavg, imed, ip99, imax);
    print_comparison((hmed, hp99, hmax), (imed, ip99, imax));

    println!("--- MPMC: 4 publishers, 4 subscribers ---");
    let (hmin, havg, hmed, hp99, hmax) = bench_horus_mpmc(4, 4);
    let (imin, iavg, imed, ip99, imax) = bench_iox2_mpmc(4, 4);
    print_row("horus  4P/4S recv cost", hmin, havg, hmed, hp99, hmax);
    print_row("iox2   4P/4S recv cost", imin, iavg, imed, ip99, imax);
    print_comparison((hmed, hp99, hmax), (imed, ip99, imax));

    // --- Throughput ---
    println!("--- send + drain-8-every-8 loop throughput (msgs/sec) ---");
    println!("  Both arms run the same loop shape. This is not a send-only rate.");
    let ht = bench_horus_throughput_u64();
    let it = bench_iox2_throughput_u64();
    println!("  horus:  {:.2}M msgs/sec", ht / 1e6);
    println!("  iox2:   {:.2}M msgs/sec", it / 1e6);
    println!("  iox2/horus ratio: {:.2}x (<1 favours horus)\n", it / ht);
}

// Cross-process MPMC uses the SHM fan-out path (ShmFanoutRing / FanoutShm).
