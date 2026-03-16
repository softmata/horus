//! Cross-Platform Performance Benchmarks for horus_sys
//!
//! Measures latency of platform-divergent operations to catch performance
//! regressions and quantify platform differences.
//!
//! Run: `cargo bench -p horus_sys --bench parity_bench`

use criterion::{criterion_group, criterion_main, Criterion};
use std::time::Duration;

/// Benchmark SHM region create + drop cycle.
///
/// Expected: <100us on Linux (tmpfs), <200us on macOS (shm_open), <500us on Windows (pagefile).
fn bench_shm_create_drop(c: &mut Criterion) {
    let mut counter = 0u64;
    c.bench_function("shm_create_drop_4kb", |b| {
        b.iter(|| {
            counter += 1;
            let name = format!("bench_shm_{}", counter);
            let region = horus_sys::shm::ShmRegion::new(&name, 4096).unwrap();
            drop(region);
        })
    });
}

/// Benchmark TopicMeta write + read roundtrip.
///
/// Expected: <50us (JSON serialize + file write + file read + deserialize).
fn bench_topic_meta_roundtrip(c: &mut Criterion) {
    let mut counter = 0u64;
    c.bench_function("topic_meta_write_read", |b| {
        b.iter(|| {
            counter += 1;
            let name = format!("bench_meta_{}", counter);
            horus_sys::shm::write_topic_meta(&name, 4096).unwrap();
            let metas = horus_sys::shm::list_topic_metas();
            std::hint::black_box(&metas);
            horus_sys::shm::remove_topic_meta(&name);
        })
    });
}

/// Benchmark list_topic_metas with N existing topics.
///
/// Setup: write 50 .meta files, then benchmark listing them.
fn bench_list_50_topic_metas(c: &mut Criterion) {
    // Setup: create 50 topic metas
    for i in 0..50 {
        horus_sys::shm::write_topic_meta(&format!("bench_list_{}", i), 1024 * (i + 1)).unwrap();
    }

    c.bench_function("list_50_topic_metas", |b| {
        b.iter(|| {
            let metas = horus_sys::shm::list_topic_metas();
            std::hint::black_box(metas.len());
        })
    });

    // Teardown
    for i in 0..50 {
        horus_sys::shm::remove_topic_meta(&format!("bench_list_{}", i));
    }
}

/// Benchmark find_nodes with N presence files.
///
/// Setup: write 20 presence files (with current PID so liveness passes).
fn bench_find_nodes_20(c: &mut Criterion) {
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    std::fs::create_dir_all(&nodes_dir).unwrap();

    // Setup: 20 presence files
    for i in 0..20 {
        let presence = serde_json::json!({
            "name": format!("bench_node_{}", i),
            "pid": std::process::id(),
            "publishers": [{"topic_name": format!("topic_{}", i), "type_name": "BenchMsg"}],
            "subscribers": [],
            "tick_count": i * 100,
        });
        std::fs::write(
            nodes_dir.join(format!("bench_node_{}.json", i)),
            serde_json::to_string(&presence).unwrap(),
        )
        .unwrap();
    }

    c.bench_function("find_nodes_20", |b| {
        b.iter(|| {
            let nodes = horus_sys::discover::find_nodes();
            std::hint::black_box(nodes.len());
        })
    });

    // Teardown
    for i in 0..20 {
        let _ = std::fs::remove_file(nodes_dir.join(format!("bench_node_{}.json", i)));
    }
}

/// Benchmark process_info for current process.
///
/// Measures per-platform process introspection cost.
fn bench_process_info(c: &mut Criterion) {
    let pid = std::process::id();
    c.bench_function("process_info_current", |b| {
        b.iter(|| {
            let info = horus_sys::discover::process_info(pid).unwrap();
            std::hint::black_box(&info);
        })
    });
}

/// Benchmark is_process_alive check.
fn bench_is_alive(c: &mut Criterion) {
    let pid = std::process::id();
    c.bench_function("is_process_alive", |b| {
        b.iter(|| {
            std::hint::black_box(horus_sys::discover::is_process_alive(pid));
        })
    });
}

/// Benchmark monotonic_ns timing call.
fn bench_monotonic_ns(c: &mut Criterion) {
    c.bench_function("monotonic_ns", |b| {
        b.iter(|| {
            std::hint::black_box(horus_sys::time::monotonic_ns());
        })
    });
}

/// Benchmark namespace generation (cached path).
fn bench_shm_namespace(c: &mut Criterion) {
    // Warm up the cache
    let _ = horus_sys::shm::shm_namespace();

    c.bench_function("shm_namespace_cached", |b| {
        b.iter(|| {
            std::hint::black_box(horus_sys::shm::shm_namespace());
        })
    });
}

criterion_group! {
    name = shm_benches;
    config = Criterion::default()
        .sample_size(100)
        .measurement_time(Duration::from_secs(5));
    targets = bench_shm_create_drop, bench_topic_meta_roundtrip, bench_list_50_topic_metas
}

criterion_group! {
    name = discover_benches;
    config = Criterion::default()
        .sample_size(50)
        .measurement_time(Duration::from_secs(3));
    targets = bench_find_nodes_20, bench_process_info, bench_is_alive
}

criterion_group! {
    name = timing_benches;
    config = Criterion::default()
        .sample_size(200)
        .measurement_time(Duration::from_secs(3));
    targets = bench_monotonic_ns, bench_shm_namespace
}

criterion_main!(shm_benches, discover_benches, timing_benches);
