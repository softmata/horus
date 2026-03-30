// Obstacle Avoidance Controller — HORUS C++ Example
//
// Subscribes to LiDAR scan, publishes velocity commands.
// The canonical "hello world" of robotics.
//
// Build: horus build
// Run:   horus run

#include <horus/horus.hpp>
using namespace horus::literals;

int main() {
    horus::Scheduler sched;
    sched.tick_rate(100_hz)
         .name("obstacle_avoidance");

    // Controller node with tick callback
    sched.add("controller")
        .tick([&] {
            // In a real robot, this would subscribe to LiDAR and publish CmdVel.
            // For now, just demonstrate the node runs.
        })
        .build();

    // Run for a few ticks then stop
    for (int i = 0; i < 10; ++i) {
        sched.tick_once();
    }
    sched.stop();
    return 0;
}
