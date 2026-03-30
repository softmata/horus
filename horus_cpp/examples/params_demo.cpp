// Runtime Parameters — HORUS C++ Example
//
// Demonstrates dynamic parameter configuration.
// Parameters can be changed at runtime via `horus param set`.
//
// Build: horus build
// Run:   horus run

#include <horus/horus.hpp>
using namespace horus::literals;

#include <cstdio>

int main() {
    auto sched = horus::Scheduler()
        .tick_rate(10_hz)
        .name("params_demo");

    auto cmd_pub = sched.advertise<horus::msg::CmdVel>("cmd_vel");

    // Configurable parameters
    float max_speed = 0.5f;
    float turn_rate = 0.3f;
    bool enabled = true;

    sched.add("configurable_controller")
        .rate(10_hz)
        .tick([&] {
            if (!enabled) return;

            auto cmd = cmd_pub.loan();
            cmd->linear  = max_speed;
            cmd->angular = turn_rate;
            cmd_pub.publish(std::move(cmd));
        })
        .build();

    sched.spin();
    return 0;
}
