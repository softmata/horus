// Tests Publisher<T>/Subscriber<T> for ALL 11 message types.
// Verifies each type can be created, sent, received, and fields preserved.
//
// Compile: g++ -std=c++17 -fext-numeric-literals -I horus_cpp/include \
//          -o cpp_all_types_test tests/cpp_all_types_test.cpp \
//          -L target/debug -lhorus_cpp -lpthread -ldl -lm

#include <horus/horus.hpp>
#include <cstdio>
#include <cmath>
#include <cstring>

static int pass_count = 0;
static int fail_count = 0;

#define CHECK(cond, name) do { \
    if (cond) { std::printf("[PASS] %s\n", name); pass_count++; } \
    else { std::printf("[FAIL] %s (line %d)\n", name, __LINE__); fail_count++; } \
} while(0)
#define CHECK_NEAR(a, b, eps, name) CHECK(std::fabs(double(a) - double(b)) < (eps), name)

// ─── Test each type: create pub+sub, send, recv, verify fields ──────────────

void test_cmd_vel() {
    std::printf("\n=== CmdVel ===\n");
    horus::Publisher<horus::msg::CmdVel>  pub("alltype.cmd_vel");
    horus::Subscriber<horus::msg::CmdVel> sub("alltype.cmd_vel");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::CmdVel msg{}; msg.timestamp_ns = 1; msg.linear = 0.5f; msg.angular = -0.1f;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) { CHECK_NEAR(r->get()->linear, 0.5f, 0.001f, "linear"); }
}

void test_laser_scan() {
    std::printf("\n=== LaserScan ===\n");
    horus::Publisher<horus::msg::LaserScan>  pub("alltype.laser");
    horus::Subscriber<horus::msg::LaserScan> sub("alltype.laser");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::LaserScan msg{}; msg.ranges[0] = 1.5f; msg.ranges[359] = 3.0f;
    msg.angle_min = -1.57f; msg.angle_max = 1.57f; msg.timestamp_ns = 2;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) {
        CHECK_NEAR(r->get()->ranges[0], 1.5f, 0.001f, "ranges[0]");
        CHECK_NEAR(r->get()->ranges[359], 3.0f, 0.001f, "ranges[359]");
        CHECK_NEAR(r->get()->angle_min, -1.57f, 0.01f, "angle_min");
    }
}

void test_imu() {
    std::printf("\n=== Imu ===\n");
    horus::Publisher<horus::msg::Imu>  pub("alltype.imu");
    horus::Subscriber<horus::msg::Imu> sub("alltype.imu");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::Imu msg{};
    msg.orientation[3] = 1.0; // w=1 (identity)
    msg.linear_acceleration[2] = 9.81;
    msg.timestamp_ns = 3;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) {
        CHECK_NEAR(r->get()->orientation[3], 1.0, 1e-10, "qw");
        CHECK_NEAR(r->get()->linear_acceleration[2], 9.81, 1e-10, "accel_z");
    }
}

void test_odometry() {
    std::printf("\n=== Odometry ===\n");
    horus::Publisher<horus::msg::Odometry>  pub("alltype.odom");
    horus::Subscriber<horus::msg::Odometry> sub("alltype.odom");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::Odometry msg{};
    msg.pose.x = 1.0; msg.pose.y = 2.0; msg.pose.theta = 0.5;
    msg.twist.linear[0] = 0.3;
    msg.timestamp_ns = 4;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) {
        CHECK_NEAR(r->get()->pose.x, 1.0, 1e-10, "pose.x");
        CHECK_NEAR(r->get()->twist.linear[0], 0.3, 1e-10, "twist.vx");
    }
}

void test_joint_state() {
    std::printf("\n=== JointState ===\n");
    horus::Publisher<horus::msg::JointState>  pub("alltype.joints");
    horus::Subscriber<horus::msg::JointState> sub("alltype.joints");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::JointState msg{};
    msg.joint_count = 3;
    msg.positions[0] = 1.0; msg.positions[1] = 2.0; msg.positions[2] = 3.0;
    msg.velocities[0] = 0.1;
    std::strncpy(reinterpret_cast<char*>(msg.names[0]), "shoulder", 31);
    msg.timestamp_ns = 5;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) {
        CHECK(r->get()->joint_count == 3, "3 joints");
        CHECK_NEAR(r->get()->positions[0], 1.0, 1e-10, "pos[0]");
        CHECK_NEAR(r->get()->positions[2], 3.0, 1e-10, "pos[2]");
    }
}

void test_twist() {
    std::printf("\n=== Twist ===\n");
    horus::Publisher<horus::msg::Twist>  pub("alltype.twist");
    horus::Subscriber<horus::msg::Twist> sub("alltype.twist");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::Twist msg{};
    msg.linear[0] = 1.0; msg.linear[1] = 2.0; msg.linear[2] = 3.0;
    msg.angular[0] = 0.1; msg.angular[1] = 0.2; msg.angular[2] = 0.3;
    msg.timestamp_ns = 6;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) {
        CHECK_NEAR(r->get()->linear[0], 1.0, 1e-10, "vx");
        CHECK_NEAR(r->get()->angular[2], 0.3, 1e-10, "wz");
    }
}

void test_pose2d() {
    std::printf("\n=== Pose2D ===\n");
    horus::Publisher<horus::msg::Pose2D>  pub("alltype.pose");
    horus::Subscriber<horus::msg::Pose2D> sub("alltype.pose");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::Pose2D msg{}; msg.x = 10.0; msg.y = 20.0; msg.theta = 1.57;
    msg.timestamp_ns = 7;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) { CHECK_NEAR(r->get()->x, 10.0, 1e-10, "x"); CHECK_NEAR(r->get()->theta, 1.57, 1e-10, "theta"); }
}

void test_transform_stamped() {
    std::printf("\n=== TransformStamped ===\n");
    horus::Publisher<horus::msg::TransformStamped>  pub("alltype.tf");
    horus::Subscriber<horus::msg::TransformStamped> sub("alltype.tf");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::TransformStamped msg{};
    msg.translation[0] = 1.0; msg.translation[1] = 2.0; msg.translation[2] = 3.0;
    msg.rotation[3] = 1.0; // w=1
    msg.timestamp_ns = 8;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) { CHECK_NEAR(r->get()->translation[0], 1.0, 1e-10, "tx"); }
}

void test_nav_goal() {
    std::printf("\n=== NavGoal ===\n");
    horus::Publisher<horus::msg::NavGoal>  pub("alltype.nav");
    horus::Subscriber<horus::msg::NavGoal> sub("alltype.nav");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::NavGoal msg{};
    msg.target_x = 5.0; msg.target_y = 3.0; msg.target_theta = 1.0;
    msg.tolerance = 0.1f; msg.timestamp_ns = 9;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) { CHECK_NEAR(r->get()->target_x, 5.0, 1e-10, "x"); CHECK_NEAR(r->get()->tolerance, 0.1f, 0.001f, "tol"); }
}

void test_heartbeat() {
    std::printf("\n=== Heartbeat ===\n");
    horus::Publisher<horus::msg::Heartbeat>  pub("alltype.hb");
    horus::Subscriber<horus::msg::Heartbeat> sub("alltype.hb");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::Heartbeat msg{};
    msg.sequence = 42; msg.alive = true; msg.cpu_usage = 50.0f;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) { CHECK(r->get()->sequence == 42, "seq"); CHECK(r->get()->alive, "alive"); }
}

void test_emergency_stop() {
    std::printf("\n=== EmergencyStop ===\n");
    horus::Publisher<horus::msg::EmergencyStop>  pub("alltype.estop");
    horus::Subscriber<horus::msg::EmergencyStop> sub("alltype.estop");
    CHECK(pub.is_valid() && sub.is_valid(), "create");
    horus::msg::EmergencyStop msg{};
    msg.engaged = 1; msg.auto_reset = 0;
    pub.send(msg);
    auto r = sub.recv();
    CHECK(r.has_value(), "recv");
    if (r) { CHECK(r->get()->engaged == 1, "engaged"); CHECK(r->get()->auto_reset == 0, "no auto_reset"); }
}

int main() {
    std::printf("╔══════════════════════════════════════════════════╗\n");
    std::printf("║  All 11 Message Types — C++ Pub/Sub Test         ║\n");
    std::printf("╚══════════════════════════════════════════════════╝\n");

    test_cmd_vel();
    test_laser_scan();
    test_imu();
    test_odometry();
    test_joint_state();
    test_twist();
    test_pose2d();
    test_transform_stamped();
    test_nav_goal();
    test_heartbeat();
    test_emergency_stop();

    std::printf("\n╔══════════════════════════════════════════════════╗\n");
    std::printf("║  Results: %2d passed, %d failed                   ║\n", pass_count, fail_count);
    std::printf("╚══════════════════════════════════════════════════╝\n");
    return fail_count > 0 ? 1 : 0;
}
