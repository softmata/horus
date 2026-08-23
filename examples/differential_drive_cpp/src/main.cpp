// Differential-drive robot controller — C++ port of the `differential_drive`
// example.
//
// Same robot, same topics, same two nodes as examples/differential_drive:
// a 50 Hz driver publishing CmdVel in a square pattern and a 10 Hz safety
// monitor watching the commands go past. Read them side by side to see what
// carries across the language boundary and what does not.
//
// Build: horus build
// Run:   horus run

#include <horus/horus.hpp>

#include <cmath>
#include <cstdio>

using namespace horus::literals;

// The scheduler ticks at 50 Hz, so every duration below is expressed in ticks.
// Kept as named constants rather than inline literals because the two phase
// lengths are the only numbers a reader is likely to want to change.
constexpr uint32_t TICK_HZ = 50;
constexpr uint32_t FORWARD_TICKS = 100;       // 2 s at 50 Hz
constexpr uint32_t TURN_TICKS = 79;           // ~1.57 rad at 1.0 rad/s
constexpr uint32_t RUN_TICKS = 30 * TICK_HZ;  // stop after 30 s, ~2 squares

/// Drives the robot in a square pattern: forward -> turn -> forward -> turn.
///
/// The Rust example ends with `scheduler.run_for(30_u64.secs())`. The C++
/// `Scheduler` has no `run_for`: `spin()` runs until something calls `stop()`.
/// So the driver counts its own ticks and stops the scheduler itself, which is
/// also how you would wire a real "finish the mission and exit" node.
class SquareDriver : public horus::Node {
public:
    explicit SquareDriver(horus::Scheduler& sched) : Node("SquareDriver"), sched_(&sched) {
        cmd_pub_ = advertise<horus::msg::CmdVel>("cmd_vel");
        odom_sub_ = subscribe<horus::msg::Odometry>("odom");
    }

    void tick() override {
        ++phase_ticks_;
        ++total_ticks_;

        horus::msg::CmdVel cmd{};
        if (phase_ == Phase::Forward) {
            if (phase_ticks_ >= FORWARD_TICKS) {
                phase_ = Phase::Turning;
                phase_ticks_ = 0;
            }
            cmd.linear = 0.3f;   // m/s
            cmd.angular = 0.0f;  // rad/s
        } else {
            if (phase_ticks_ >= TURN_TICKS) {
                phase_ = Phase::Forward;
                phase_ticks_ = 0;
            }
            cmd.linear = 0.0f;
            cmd.angular = 1.0f;
        }
        cmd_pub_->send(cmd);

        // Log odometry every 50 ticks, when the simulator is publishing it.
        if (phase_ticks_ % 50 == 0) {
            // `recv()` hands back `optional<BorrowedSample<T>>`, so reaching a
            // field takes two arrows: one through the optional, one through the
            // borrowed sample. Same shape as the other C++ examples.
            if (auto odom = odom_sub_->recv()) {
                char line[160];
                std::snprintf(line, sizeof(line), "pos=(%.2f, %.2f), theta=%.2f, phase=%s",
                              (*odom)->pose.x, (*odom)->pose.y, (*odom)->pose.theta,
                              phase_ == Phase::Forward ? "forward" : "turning");
                horus::log::info(name(), line);
            }
        }

        if (total_ticks_ >= RUN_TICKS) {
            horus::log::info(name(), "30 s elapsed — stopping");
            sched_->stop();
        }
    }

    // Called by the safety monitor path, not by `tick`: a stopped robot is the
    // safe state for a differential drive.
    void enter_safe_state() override {
        cmd_pub_->send(horus::msg::CmdVel{});
    }

private:
    enum class Phase { Forward, Turning };

    horus::Scheduler* sched_;
    horus::Publisher<horus::msg::CmdVel>* cmd_pub_;
    horus::Subscriber<horus::msg::Odometry>* odom_sub_;
    Phase phase_ = Phase::Forward;
    uint32_t phase_ticks_ = 0;
    uint32_t total_ticks_ = 0;
};

/// Watches the velocity commands and complains when they leave the envelope.
class SafetyNode : public horus::Node {
public:
    SafetyNode() : Node("SafetyNode") {
        cmd_sub_ = subscribe<horus::msg::CmdVel>("cmd_vel");
    }

    void tick() override {
        auto cmd = cmd_sub_->recv();
        if (!cmd)
            return;

        const float linear = (*cmd)->linear;
        const float angular = (*cmd)->angular;
        if (std::fabs(linear) > MAX_LINEAR || std::fabs(angular) > MAX_ANGULAR) {
            char line[128];
            std::snprintf(line, sizeof(line), "SAFETY: velocity exceeds limits! lin=%.2f ang=%.2f",
                          linear, angular);
            horus::log::warn(name(), line);
        }
    }

private:
    static constexpr float MAX_LINEAR = 1.0f;   // m/s
    static constexpr float MAX_ANGULAR = 3.0f;  // rad/s

    horus::Subscriber<horus::msg::CmdVel>* cmd_sub_;
};

int main() {
    // Declare first, then configure. The configuration methods return
    // `Scheduler&` and the copy constructor is deleted, so
    // `auto sched = horus::Scheduler().tick_rate(...)` does not compile.
    horus::Scheduler sched;
    sched.tick_rate(50_hz).name("differential_drive_cpp");

    SquareDriver driver(sched);
    SafetyNode safety;

    sched.add(driver).order(0).rate(50_hz).build();
    sched.add(safety).order(10).rate(10_hz).build();

    sched.spin();
    return 0;
}
