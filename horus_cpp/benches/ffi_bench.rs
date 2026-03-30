//! Criterion benchmarks for C++ FFI overhead.

use criterion::{black_box, criterion_group, criterion_main, Criterion};

use horus_cpp::*;

fn bench_scheduler_new_destroy(c: &mut Criterion) {
    c.bench_function("scheduler_new+destroy", |b| {
        b.iter(|| {
            let sched = scheduler_new();
            black_box(&sched);
            drop(sched);
        });
    });
}

fn bench_tick_once_empty(c: &mut Criterion) {
    let mut sched = scheduler_new();
    c.bench_function("tick_once (empty)", |b| {
        b.iter(|| {
            scheduler_tick_once(black_box(&mut sched)).ok();
        });
    });
}

fn bench_tick_once_1_node(c: &mut Criterion) {
    let mut sched = scheduler_new();
    extern "C" fn noop() {}
    let mut builder = node_builder_new("bench_node");
    node_builder_set_tick(&mut builder, noop);
    node_builder_build(builder, &mut sched).unwrap();

    c.bench_function("tick_once (1 node)", |b| {
        b.iter(|| {
            scheduler_tick_once(black_box(&mut sched)).ok();
        });
    });
}

fn bench_topic_send_cmd_vel(c: &mut Criterion) {
    let topic = format!("bench.cmd_vel.{}", std::process::id());
    let pub_ = publisher_cmd_vel_new(&topic).unwrap();
    let msg = horus_library::CmdVel {
        timestamp_ns: 0,
        linear: 0.5,
        angular: 0.1,
    };

    c.bench_function("topic_send CmdVel", |b| {
        b.iter(|| {
            publisher_cmd_vel_send(&pub_, black_box(msg));
        });
    });
}

fn bench_topic_send_recv_cmd_vel(c: &mut Criterion) {
    let topic = format!("bench.roundtrip.{}", std::process::id());
    let pub_ = publisher_cmd_vel_new(&topic).unwrap();
    let sub = subscriber_cmd_vel_new(&topic).unwrap();
    let msg = horus_library::CmdVel {
        timestamp_ns: 0,
        linear: 0.5,
        angular: 0.1,
    };

    c.bench_function("topic_send+recv CmdVel", |b| {
        b.iter(|| {
            publisher_cmd_vel_send(&pub_, black_box(msg));
            let _ = black_box(subscriber_cmd_vel_recv(&sub));
        });
    });
}

fn bench_abi_version(c: &mut Criterion) {
    c.bench_function("get_abi_version (bare FFI call)", |b| {
        b.iter(|| {
            black_box(get_abi_version());
        });
    });
}

criterion_group!(
    benches,
    bench_abi_version,
    bench_scheduler_new_destroy,
    bench_tick_once_empty,
    bench_tick_once_1_node,
    bench_topic_send_cmd_vel,
    bench_topic_send_recv_cmd_vel,
);
criterion_main!(benches);
