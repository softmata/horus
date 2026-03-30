// Multi-Node Pipeline — HORUS C++ Example
//
// Demonstrates a 3-node pipeline: sensor → processor → actuator.
// All nodes run in a single scheduler with execution ordering.
//
// Build: horus build
// Run:   horus run

#include <horus/horus.hpp>
using namespace horus::literals;

int main() {
    auto sched = horus::Scheduler()
        .tick_rate(100_hz)
        .name("robot_pipeline")
        .prefer_rt();

    // ── Topics ───────────────────────────────────────────────────────
    auto scan_pub = sched.advertise<horus::msg::LaserScan>("lidar.scan");
    auto scan_sub = sched.subscribe<horus::msg::LaserScan>("lidar.scan");
    auto cmd_pub  = sched.advertise<horus::msg::CmdVel>("cmd_vel");
    auto cmd_sub  = sched.subscribe<horus::msg::CmdVel>("cmd_vel");
    auto hb_pub   = sched.advertise<horus::msg::Heartbeat>("system.heartbeat");

    // ── Node 1: Sensor (highest priority) ────────────────────────────
    sched.add("lidar_driver")
        .rate(100_hz)
        .order(0)  // runs first
        .tick([&] {
            auto scan = scan_pub.loan();
            // Simulate LiDAR data
            for (int i = 0; i < 360; ++i) {
                scan->ranges[i] = 2.0f + 0.5f * static_cast<float>(i % 10);
            }
            scan->angle_min = 0.0f;
            scan->angle_max = 6.28f;
            scan_pub.publish(std::move(scan));
        })
        .build();

    // ── Node 2: Processor (medium priority) ──────────────────────────
    sched.add("nav_controller")
        .rate(50_hz)
        .order(10)
        .budget(5_ms)
        .on_miss(horus::Miss::Skip)
        .tick([&] {
            auto scan = scan_sub.recv();
            if (!scan) return;

            // Compute velocity from scan
            float front_range = scan->ranges[0];
            auto cmd = cmd_pub.loan();
            cmd->linear  = front_range > 1.0f ? 0.3f : 0.0f;
            cmd->angular = front_range > 1.0f ? 0.0f : 0.5f;
            cmd_pub.publish(std::move(cmd));
        })
        .build();

    // ── Node 3: Actuator (lowest priority) ───────────────────────────
    sched.add("motor_driver")
        .rate(100_hz)
        .order(20)
        .tick([&] {
            auto cmd = cmd_sub.recv();
            if (!cmd) return;
            // In a real robot, this would send to motor controller
            // cmd->linear, cmd->angular → PWM signals
        })
        .build();

    // ── Node 4: Diagnostics (background) ─────────────────────────────
    sched.add("diagnostics")
        .rate(1_hz)
        .order(200)  // background priority
        .tick([&] {
            auto hb = hb_pub.loan();
            hb->alive = 1;
            hb->sequence += 1;
            hb_pub.publish(std::move(hb));
        })
        .build();

    sched.spin();
    return 0;
}
