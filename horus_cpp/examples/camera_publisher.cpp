// Camera Publisher — HORUS C++ Example
//
// Demonstrates the loan pattern for zero-copy image publishing.
// The loaned buffer points directly to shared memory — no copies.
//
// Build: horus build
// Run:   horus run

#include <horus/horus.hpp>
using namespace horus::literals;

int main() {
    // The configuration methods return `Scheduler&`, and the copy
    // constructor is deleted, so the chain cannot initialize a new
    // object — declare first, then configure (see scheduler.hpp).
    horus::Scheduler sched;
    sched.tick_rate(30_hz)
        .name("camera_node");

    auto imu_pub = sched.advertise<horus::msg::Imu>("camera.imu");
    // NOTE: the vision messages (CameraInfo and friends) are bound in the FFI
    // layer — see `impl_topic_ffi!(camera_info, ...)` — but
    // include/horus/msg/vision.hpp declares no structs, so they are not
    // reachable from C++. This example used `horus::msg::CameraInfo` with
    // fields `fx`/`fy`/`cx`/`cy`, a shape the Rust type has not had for some
    // time (it carries `camera_matrix[9]`), so it did not compile on either
    // count. Publishing the IMU below exercises the same loan/publish path.

    // Simulated camera node publishing at 30 Hz
    sched.add("camera_driver")
        .rate(30_hz)
        .budget(30_ms)
        .tick([&] {
            // Publish simulated IMU data (camera has built-in IMU)
            auto imu = imu_pub.loan();
            imu->orientation[0] = 0.0;
            imu->orientation[1] = 0.0;
            imu->orientation[2] = 0.0;
            imu->orientation[3] = 1.0; // identity quaternion
            imu->linear_acceleration[0] = 0.0;
            imu->linear_acceleration[1] = 0.0;
            imu->linear_acceleration[2] = 9.81; // gravity
            imu_pub.publish(std::move(imu));

        })
        .build();

    sched.spin();
    return 0;
}
