// Transform Frames — HORUS C++ Example
//
// Demonstrates the coordinate frame system (TF).
// Sets up a frame tree and queries transforms between frames.
//
// Build: horus build
// Run:   horus run

#include <horus/horus.hpp>
using namespace horus::literals;

#include <cstdio>

int main() {
    auto sched = horus::Scheduler()
        .tick_rate(100_hz)
        .name("tf_demo");

    auto tf_pub = sched.advertise<horus::msg::TransformStamped>("tf");

    // Publish transform updates
    sched.add("tf_publisher")
        .rate(100_hz)
        .tick([&] {
            // Robot base moves forward over time
            auto tf = tf_pub.loan();
            tf->translation[0] = 1.0;  // 1m forward
            tf->translation[1] = 0.0;
            tf->translation[2] = 0.0;
            tf->rotation[0] = 0.0;  // identity rotation
            tf->rotation[1] = 0.0;
            tf->rotation[2] = 0.0;
            tf->rotation[3] = 1.0;
            tf_pub.publish(std::move(tf));
        })
        .build();

    sched.spin();
    return 0;
}
