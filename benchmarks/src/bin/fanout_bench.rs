//! Quick FanoutRing latency benchmark
//! Run: cargo run --release -p horus_benchmarks --bin fanout_bench --no-default-features

use std::sync::Arc;
use std::thread;
use std::time::Instant;

use horus_core::communication::topic::fanout::FanoutRing;

fn bench_fanout(np: usize, ns: usize) -> (u64, u64, u64, u64) {
    let msgs_per_pub = 50_000;
    let ring = Arc::new(FanoutRing::<u64>::new(np + 1, ns + 1, 256));

    let mut pub_ids = Vec::new();
    let mut sub_ids = Vec::new();
    for _ in 0..np {
        pub_ids.push(ring.register_publisher());
    }
    for _ in 0..ns {
        sub_ids.push(ring.register_subscriber());
    }

    let barrier = Arc::new(std::sync::Barrier::new(np + ns));

    let mut sub_handles = Vec::new();
    for &sid in &sub_ids {
        let ring = ring.clone();
        let barrier = barrier.clone();
        sub_handles.push(thread::spawn(move || {
            barrier.wait();
            let mut latencies = Vec::new();
            let expected = msgs_per_pub * np;
            let mut received = 0;
            let deadline = Instant::now() + std::time::Duration::from_secs(10);
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

    let mut pub_handles = Vec::new();
    for &pid in &pub_ids {
        let ring = ring.clone();
        let barrier = barrier.clone();
        pub_handles.push(thread::spawn(move || {
            barrier.wait();
            for i in 0..msgs_per_pub {
                let _ = ring.send_as(i as u64, pid);
            }
        }));
    }

    for h in pub_handles {
        h.join().unwrap();
    }
    let mut all: Vec<u64> = Vec::new();
    for h in sub_handles {
        all.extend(h.join().unwrap());
    }

    all.sort();
    let n = all.len();
    if n == 0 {
        return (0, 0, 0, 0);
    }
    let avg = all.iter().sum::<u64>() / n as u64;
    let med = all[n / 2];
    let p99 = all[(n as f64 * 0.99) as usize];
    let min = all[0];
    (min, avg, med, p99)
}

fn main() {
    println!("=== FanoutRing (contention-free MPMC) Latency ===");
    println!("   50,000 msgs/publisher, all times in ns\n");

    for (np, ns) in [(1, 1), (2, 2), (4, 4), (8, 4), (8, 8)] {
        let (min, avg, med, p99) = bench_fanout(np, ns);
        println!(
            "  {}P/{}S:  min={:>5}  avg={:>5}  med={:>5}  p99={:>6}",
            np, ns, min, avg, med, p99
        );
    }

    println!("\n=== Comparison Reference (from previous run) ===");
    println!("  Old MPMC 2P/2S:  avg=176ns  (CAS contention)");
    println!("  Old MPMC 4P/4S:  avg=502ns  (CAS contention)");
    println!("  iceoryx2 2P/2S:  avg=127ns");
    println!("  iceoryx2 4P/4S:  avg=140ns");
}
