#![allow(dead_code)]
//! RT Readiness Report test — generates the customer-facing RT report.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test rt_readiness -- --ignored --nocapture

use horus_core::scheduling::rt_report::RtReport;
use std::time::Duration;

#[test]
#[ignore]
fn generate_rt_readiness_report() {
    println!("\nRunning RT readiness assessment (3 second benchmark)...\n");
    let report = RtReport::generate(Duration::from_secs(3));
    report.print();

    // Basic sanity checks
    assert!(
        report.jitter_samples > 1000,
        "Should have >1000 jitter samples"
    );
    assert!(
        report.ipc_latency_ns > 0.0,
        "IPC latency should be measured"
    );
    assert!(
        report.ipc_throughput_msg_per_sec > 1000.0,
        "IPC should do >1000 msg/sec"
    );

    println!("\nGrade: {:?}", report.grade);
    println!("Production ready: {}", report.is_production_ready());
}
