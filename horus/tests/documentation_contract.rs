use horus::prelude::*;

#[test]
fn unified_prelude_exposes_documented_robotics_and_rt_types() {
    let command = CmdVel::new(1.0, 0.5);
    assert_eq!(command.linear, 1.0);
    assert_eq!(command.angular, 0.5);

    let scan = LaserScan::default();
    assert_eq!(scan.ranges.len(), 360);

    let config = RtConfig::builder()
        .memory_locked(false)
        .scheduler(RtScheduler::Normal)
        .priority(1)
        .build();
    let _ = config;
}
