//! A fast one-way latency probe for a single `Topic<T>` path.
//!
//! `all_paths_latency` takes ~10 minutes to sweep every backend, which makes it
//! useless as a feedback loop while optimising the hot path. This measures one
//! thing — cross-thread one-way latency on a POD topic — in about two seconds,
//! so a change can be measured as fast as it can be compiled.
//!
//! Run: `cargo run --release -p horus_benchmarks --bin topic_probe [reps]`
//!
//! # Method
//!
//! Offset-free by construction: producer and consumer are threads in one
//! process reading the same `CLOCK_MONOTONIC`, and the send timestamp travels
//! inside the payload, so no cross-clock comparison is needed.
//!
//! The producer waits for the consumer's ack before sending again, so every
//! sample is an UNLOADED receive. A free-running producer measures queueing
//! delay instead — which is how `Topic_cross_process`'s 5914ns came to be
//! published as a latency when it was 64 slots of backlog at 92ns each.
//!
//! Latencies are reported UNFILTERED. Tukey trimming is what let
//! `DeterminismMetrics.cv` report a good number no matter what the tail did.
//!
//! `--raw` prints one line per rep so a caller can do its own statistics.

use horus_core::communication::Topic;
use horus_robotics::messages::CmdVel;
use serde::{Deserialize, Serialize};

/// A payload with the send timestamp in its first word, so the one-way
/// measurement works the same way it does for `CmdVel`.
///
/// `Topic<T>` requires Serialize/Deserialize for its bounds, but a POD type
/// never takes the serde path.
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct Bulk<const N: usize> {
    ts: u64,
    #[serde(with = "serde_arrays")]
    data: [u64; N],
}

const WARMUP: usize = 20_000;
const SAMPLES: usize = 200_000;
/// Fewer samples for bulk payloads: a 4 KB ring at capacity 16 is 64 KB, and
/// the run is memory-bound rather than latency-bound past a point.
const SAMPLES_BULK: usize = 50_000;
/// Image frames allocate from a pool per send, so fewer samples.
const SAMPLES_IMG: usize = 20_000;
const WARMUP_IMG: usize = 2_000;

#[repr(C)]
struct Timespec {
    sec: i64,
    nsec: i64,
}

extern "C" {
    fn clock_gettime(clk: i32, tp: *mut Timespec) -> i32;
    fn sched_setaffinity(pid: i32, cpusetsize: usize, mask: *const u64) -> i32;
}

const CLOCK_MONOTONIC: i32 = 1;

#[inline(always)]
fn now_ns() -> u64 {
    let mut ts = Timespec { sec: 0, nsec: 0 };
    // SAFETY: `ts` is a valid, correctly-sized `struct timespec`.
    unsafe {
        clock_gettime(CLOCK_MONOTONIC, &mut ts);
    }
    (ts.sec as u64) * 1_000_000_000 + (ts.nsec as u64)
}

/// Pin the calling thread to `cpu`. Returns false if the kernel refused.
fn pin(cpu: usize) -> bool {
    let mut mask = [0u64; 16];
    mask[cpu / 64] = 1u64 << (cpu % 64);
    // SAFETY: `mask` is a 1024-bit cpu_set_t, the size glibc expects.
    unsafe { sched_setaffinity(0, core::mem::size_of::<[u64; 16]>(), mask.as_ptr()) == 0 }
}

fn pct(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

/// One rep on a bulk payload. Same method as `run`; separated because the
/// timestamp lives in a different field.
fn run_bulk<const N: usize>(rep: usize, cpus: (usize, usize)) -> Vec<u64>
where
    Bulk<N>: Serialize + for<'d> Deserialize<'d>,
{
    let name = format!("probe_b{}_{}_{}", N, std::process::id(), rep);
    let tx: Topic<Bulk<N>> = Topic::new(&name).expect("producer handle");
    let rx: Topic<Bulk<N>> = Topic::new(&name).expect("consumer handle");
    assert!(rx.recv().is_none(), "nothing published yet");

    let total = WARMUP + SAMPLES_BULK;
    let ack = std::sync::atomic::AtomicU64::new(0);
    if rep == 0 {
        eprintln!(
            "  payload {} B, backend: {} (producer), {} (consumer)",
            core::mem::size_of::<Bulk<N>>(),
            tx.backend_name(),
            rx.backend_name()
        );
    }

    std::thread::scope(|s| {
        let ack = &ack;
        let consumer = s.spawn(move || {
            pin(cpus.1);
            let mut out: Vec<u64> = Vec::with_capacity(total);
            let mut seen = 0u64;
            while (seen as usize) < total {
                if let Some(msg) = rx.recv() {
                    out.push(now_ns().wrapping_sub(msg.ts));
                    seen += 1;
                    ack.store(seen, std::sync::atomic::Ordering::Release);
                } else {
                    std::hint::spin_loop();
                }
            }
            out.split_off(WARMUP)
        });

        pin(cpus.0);
        let mut send_ns: Vec<u64> = Vec::with_capacity(total);
        for i in 0..total {
            let t0 = now_ns();
            let msg = Bulk::<N> {
                ts: t0,
                data: [i as u64; N],
            };
            tx.send(msg);
            send_ns.push(now_ns().wrapping_sub(t0));
            let want = i as u64 + 1;
            while ack.load(std::sync::atomic::Ordering::Acquire) < want {
                std::hint::spin_loop();
            }
        }
        let mut sv = send_ns.split_off(WARMUP);
        sv.sort_unstable();
        if rep == 0 {
            eprintln!(
                "  unloaded send() p50={}ns p99={}ns",
                pct(&sv, 0.50),
                pct(&sv, 0.99)
            );
        }
        consumer.join().expect("consumer thread")
    })
}

/// One rep on the tensor-backed `Image` path — HORUS's actual answer for bulk
/// payloads.
///
/// `Image` is `{ descriptor, pool: Arc<TensorPool> }`: the pixels live in a
/// shared-memory pool and the topic carries only the 224-byte descriptor. So
/// publishing is genuinely zero-copy and the latency should be flat in frame
/// size — which is the property worth measuring, and the reason a
/// `Topic<[u64; 512]>` row is not the right comparison against a loan/publish
/// API. Sending 4 KB by value copies 4 KB; this does not.
///
/// The timestamp travels in the descriptor's own `timestamp_ns` field.
fn run_image(rep: usize, w: u32, h: u32, cpus: (usize, usize)) -> Vec<u64> {
    use horus_core::memory::Image;
    use horus_core::types::ImageEncoding;

    let name = format!("probe_img{}x{}_{}_{}", w, h, std::process::id(), rep);
    let tx: Topic<Image> = Topic::new(&name).expect("producer handle");
    let rx: Topic<Image> = Topic::new(&name).expect("consumer handle");
    assert!(rx.recv().is_none(), "nothing published yet");

    let total = WARMUP_IMG + SAMPLES_IMG;
    let ack = std::sync::atomic::AtomicU64::new(0);
    if rep == 0 {
        let probe = Image::new(w, h, ImageEncoding::Mono8).expect("alloc probe");
        eprintln!(
            "  {}x{} Mono8 = {} B of pixels, descriptor on the wire; backend: {}",
            w,
            h,
            probe.nbytes(),
            tx.backend_name()
        );
    }

    std::thread::scope(|s| {
        let ack = &ack;
        let consumer = s.spawn(move || {
            pin(cpus.1);
            let mut out: Vec<u64> = Vec::with_capacity(total);
            let mut seen = 0u64;
            while (seen as usize) < total {
                if let Some(img) = rx.recv() {
                    out.push(now_ns().wrapping_sub(img.timestamp_ns()));
                    seen += 1;
                    ack.store(seen, std::sync::atomic::Ordering::Release);
                } else {
                    std::hint::spin_loop();
                }
            }
            out.split_off(WARMUP_IMG)
        });

        pin(cpus.0);
        // Allocate ONCE, outside the loop, and clone per send.
        //
        // `Image::new` zero-initialises the pixel buffer, so allocating inside
        // the loop measures a memset of the whole frame: with a fresh 1920x1080
        // frame per iteration this reported 82us, which is the cost of clearing
        // 2 MB, not of publishing anything. Cloning bumps the tensor refcount
        // and copies only the descriptor, which is what a camera pipeline
        // reusing its buffers actually does — and what makes the transport cost
        // visible.
        let frame = Image::new(w, h, ImageEncoding::Mono8).expect("alloc frame");
        frame.data_mut()[0] = 1;
        for i in 0..total {
            let mut img = frame.clone();
            // Touch one pixel through the shared buffer so the send cannot be
            // optimised away.
            img.data_mut()[0] = i as u8;
            img.set_timestamp_ns(now_ns());
            tx.send(img);
            let want = i as u64 + 1;
            while ack.load(std::sync::atomic::Ordering::Acquire) < want {
                std::hint::spin_loop();
            }
        }
        consumer.join().expect("consumer thread")
    })
}

/// One rep. Returns the sorted per-message one-way latencies in ns.
fn run(rep: usize, cpus: (usize, usize)) -> Vec<u64> {
    let name = format!("probe_{}_{}", std::process::id(), rep);
    let tx: Topic<CmdVel> = Topic::new(&name).expect("producer handle");
    let rx: Topic<CmdVel> = Topic::new(&name).expect("consumer handle");

    // Seat the consumer's cursor before anything is published: a broadcast
    // backend joins at the head, so a handle that first calls recv() after the
    // sends would legitimately see nothing.
    assert!(rx.recv().is_none(), "nothing published yet");

    let total = WARMUP + SAMPLES;
    let ack = std::sync::atomic::AtomicU64::new(0);

    // Report the backend actually selected. Which dispatch path this exercises
    // decides what the number means, and a benchmark that does not say cannot
    // be compared against a run that chose differently — `all_paths_latency`
    // has a row whose selected backend does not match the one its label claims.
    // Read before the handles are moved into the threads.
    if rep == 0 {
        eprintln!(
            "  backend: {} (producer), {} (consumer)",
            tx.backend_name(),
            rx.backend_name()
        );
    }

    std::thread::scope(|s| {
        let ack = &ack;
        let consumer = s.spawn(move || {
            pin(cpus.1);
            let mut out: Vec<u64> = Vec::with_capacity(total);
            let mut seen = 0u64;
            while (seen as usize) < total {
                if let Some(msg) = rx.recv() {
                    out.push(now_ns().wrapping_sub(msg.timestamp_ns));
                    seen += 1;
                    ack.store(seen, std::sync::atomic::Ordering::Release);
                } else {
                    std::hint::spin_loop();
                }
            }
            out.split_off(WARMUP)
        });

        pin(cpus.0);
        // The send timestamp is also the start of the send-cost measurement, so
        // splitting the one-way time into "enqueue" and "transfer + receive"
        // costs one extra clock read per message and no extra apparatus. Doing
        // it here rather than in a free-running loop matters: with the producer
        // paced by the ack, the ring never fills, so this is unloaded enqueue
        // cost. An unthrottled producer measures the backpressure re-read of
        // `header.tail` — a line the consumer owns — on nearly every send, and
        // reports ~95ns instead of the truth.
        let mut send_ns: Vec<u64> = Vec::with_capacity(total);
        for i in 0..total {
            let t0 = now_ns();
            let msg = CmdVel {
                timestamp_ns: t0,
                linear: 1.0,
                angular: 0.5,
            };
            tx.send(msg);
            send_ns.push(now_ns().wrapping_sub(t0));
            let want = i as u64 + 1;
            while ack.load(std::sync::atomic::Ordering::Acquire) < want {
                std::hint::spin_loop();
            }
        }
        let mut sv = send_ns.split_off(WARMUP);
        sv.sort_unstable();
        if rep == 0 {
            eprintln!(
                "  unloaded send() p50={}ns p99={}ns (inside the one-way figure below)",
                pct(&sv, 0.50),
                pct(&sv, 0.99)
            );
        }
        consumer.join().expect("consumer thread")
    })
}

/// Cost of the `send()` call alone, with a consumer draining so the ring never
/// fills. This is NOT a latency — it is the producer-side enqueue cost, and the
/// distinction matters: `all_paths_latency` labels these rows `[send]` and they
/// have been quoted as though they were end-to-end times.
fn send_only(reps: usize, cpus: (usize, usize)) {
    println!();
    println!("  send() call cost alone (NOT a latency — enqueue cost only)");
    println!("  rep      p50      p99    p99.9");
    for rep in 0..reps {
        let name = format!("probe_s_{}_{}", std::process::id(), rep);
        let tx: Topic<CmdVel> = Topic::new(&name).expect("producer handle");
        let rx: Topic<CmdVel> = Topic::new(&name).expect("consumer handle");
        assert!(rx.recv().is_none(), "nothing published yet");

        let stop = std::sync::atomic::AtomicBool::new(false);
        let total = WARMUP + SAMPLES;
        std::thread::scope(|s| {
            let stop = &stop;
            let drain = s.spawn(move || {
                pin(cpus.1);
                let mut n = 0u64;
                while !stop.load(std::sync::atomic::Ordering::Relaxed) {
                    if rx.recv().is_some() {
                        n += 1;
                    } else {
                        std::hint::spin_loop();
                    }
                }
                n
            });
            pin(cpus.0);
            let mut out: Vec<u64> = Vec::with_capacity(total);
            for i in 0..total {
                let msg = CmdVel {
                    timestamp_ns: 0,
                    linear: 1.0,
                    angular: i as f32,
                };
                let t0 = now_ns();
                tx.send(msg);
                out.push(now_ns().wrapping_sub(t0));
            }
            stop.store(true, std::sync::atomic::Ordering::Relaxed);
            drain.join().expect("drain thread");
            let mut v = out.split_off(WARMUP);
            v.sort_unstable();
            println!(
                "  {:>3}  {:>7}  {:>7}  {:>7}",
                rep,
                pct(&v, 0.50),
                pct(&v, 0.99),
                pct(&v, 0.999)
            );
        });
    }
}

/// Cost of allocating a pooled frame and returning it — `Image::new` plus the
/// drop that scrubs the slot. NOT a transport figure.
///
/// Worth measuring separately because a pipeline that allocates per capture puts
/// this on its critical path, and it is far larger than publishing: the scrub
/// alone is ~45us for a 1080p frame. The transport for the same frame is ~300ns.
fn alloc_only(reps: usize, w: u32, h: u32) {
    use horus_core::memory::Image;
    use horus_core::types::ImageEncoding;

    let probe = Image::new(w, h, ImageEncoding::Mono8).expect("alloc");
    println!();
    println!(
        "  Image::new + drop, {}x{} Mono8 = {} B (NOT a transport figure)",
        w,
        h,
        probe.nbytes()
    );
    drop(probe);
    println!("  rep      p50      p99      max");
    for _ in 0..reps {
        let n = 2_000usize;
        let mut v: Vec<u64> = Vec::with_capacity(n);
        for _ in 0..n {
            let t0 = now_ns();
            let img = Image::new(w, h, ImageEncoding::Mono8).expect("alloc");
            drop(img);
            v.push(now_ns().wrapping_sub(t0));
        }
        v.sort_unstable();
        println!(
            "       {:>7}  {:>7}  {:>7}",
            pct(&v, 0.50),
            pct(&v, 0.99),
            v[v.len() - 1]
        );
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let raw = args.iter().any(|a| a == "--raw");
    let send_only_mode = args.iter().any(|a| a == "--send-only");
    let alloc_dims: Option<(u32, u32)> = args
        .iter()
        .position(|a| a == "--alloc")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| {
            let (w, h) = s.split_once('x')?;
            Some((w.parse().ok()?, h.parse().ok()?))
        });
    let image_dims: Option<(u32, u32)> = args
        .iter()
        .position(|a| a == "--image")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| {
            let (w, h) = s.split_once('x')?;
            Some((w.parse().ok()?, h.parse().ok()?))
        });
    let bulk_bytes: Option<usize> = args
        .iter()
        .position(|a| a == "--bulk")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok());
    let reps: usize = args
        .iter()
        .skip(1)
        .find(|a| !a.starts_with("--"))
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);

    let ncpu = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(2);
    // 0 and 2 rather than 0 and 1: on the common enumeration those are distinct
    // physical cores, where 0/1 can be an SMT sibling pair sharing an L1 — which
    // would remove the very coherence traffic this measures.
    let cpus = if ncpu >= 4 { (0usize, 2usize) } else { (0, 1) };

    let clock_probe = {
        let t0 = now_ns();
        let mut acc = 0u64;
        for _ in 0..100_000 {
            acc = acc.wrapping_add(now_ns());
        }
        let d = (now_ns() - t0) as f64 / 100_000.0;
        std::hint::black_box(acc);
        d
    };

    println!(
        "topic_probe: Topic<CmdVel> ({} B), cross-thread one-way, {SAMPLES} samples x {reps} reps",
        core::mem::size_of::<CmdVel>()
    );
    println!("clock overhead ~{clock_probe:.1}ns (inside every sample); cpus {cpus:?}; unfiltered");
    println!();

    if send_only_mode {
        send_only(reps, cpus);
        return;
    }

    if let Some((w, h)) = alloc_dims {
        alloc_only(reps, w, h);
        return;
    }

    if let Some((w, h)) = image_dims {
        println!("  rep      p50      p99    p99.9      max");
        for rep in 0..reps {
            let mut v = run_image(rep, w, h, cpus);
            v.sort_unstable();
            println!(
                "  {:>3}  {:>7}  {:>7}  {:>7}  {:>7}",
                rep,
                pct(&v, 0.50),
                pct(&v, 0.99),
                pct(&v, 0.999),
                v[v.len() - 1]
            );
        }
        return;
    }

    if let Some(bytes) = bulk_bytes {
        println!("  rep      p50      p99    p99.9      max");
        for rep in 0..reps {
            let mut v = match bytes {
                1024 => run_bulk::<128>(rep, cpus),
                4096 => run_bulk::<512>(rep, cpus),
                _ => {
                    eprintln!("--bulk accepts 1024 or 4096");
                    return;
                }
            };
            v.sort_unstable();
            println!(
                "  {:>3}  {:>7}  {:>7}  {:>7}  {:>7}",
                rep,
                pct(&v, 0.50),
                pct(&v, 0.99),
                pct(&v, 0.999),
                v[v.len() - 1]
            );
        }
        return;
    }

    let mut medians = Vec::new();
    println!("  rep      p50      p99    p99.9      max");
    for rep in 0..reps {
        let mut v = run(rep, cpus);
        v.sort_unstable();
        medians.push(pct(&v, 0.50));
        println!(
            "  {:>3}  {:>7}  {:>7}  {:>7}  {:>7}",
            rep,
            pct(&v, 0.50),
            pct(&v, 0.99),
            pct(&v, 0.999),
            v[v.len() - 1]
        );
        if raw {
            eprintln!("raw rep{rep} p50={}", pct(&v, 0.50));
        }
    }

    medians.sort_unstable();
    let best = medians[0];
    let med_of_med = medians[medians.len() / 2];
    println!();
    // The median of per-rep medians is the headline: it is robust to one rep
    // landing on a noisy scheduling window, which a mean is not. `best` is
    // reported beside it because on a shared, powersave-governed box the
    // fastest rep is the one least contaminated by other load.
    println!("  median-of-medians {med_of_med}ns   best-rep {best}ns");
}
