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
    auto sched = horus::Scheduler()
        .tick_rate(30_hz)
        .name("camera_node");

    auto imu_pub = sched.advertise<horus::msg::Imu>("camera.imu");
    auto info_pub = sched.advertise<horus::msg::CameraInfo>("camera.info");

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

            // Publish camera intrinsics (once per frame)
            auto info = info_pub.loan();
            info->width = 1920;
            info->height = 1080;
            info->fx = 600.0;
            info->fy = 600.0;
            info->cx = 960.0;
            info->cy = 540.0;
            info_pub.publish(std::move(info));
        })
        .build();

    sched.spin();
    return 0;
}
