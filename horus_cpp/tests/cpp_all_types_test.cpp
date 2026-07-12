// Tests Publisher<T>/Subscriber<T> for ALL 11 message types.
// Verifies each type can be created, sent, received, and fields preserved.
// Converted to GoogleTest.

#include <horus/horus.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

TEST(AllTypes, CmdVel) {
    horus::Publisher<horus::msg::CmdVel>  pub("alltype.cmd_vel");
    horus::Subscriber<horus::msg::CmdVel> sub("alltype.cmd_vel");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::CmdVel msg{};
    msg.timestamp_ns = 1; msg.linear = 0.5f; msg.angular = -0.1f;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->linear, 0.5f, 0.001f);
}

TEST(AllTypes, LaserScan) {
    horus::Publisher<horus::msg::LaserScan>  pub("alltype.laser");
    horus::Subscriber<horus::msg::LaserScan> sub("alltype.laser");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::LaserScan msg{};
    msg.ranges[0] = 1.5f; msg.ranges[359] = 3.0f;
    msg.angle_min = -1.57f; msg.angle_max = 1.57f;
    msg.timestamp_ns = 2;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->ranges[0], 1.5f, 0.001f);
    EXPECT_NEAR(r->get()->ranges[359], 3.0f, 0.001f);
    EXPECT_NEAR(r->get()->angle_min, -1.57f, 0.01f);
}

TEST(AllTypes, Imu) {
    horus::Publisher<horus::msg::Imu>  pub("alltype.imu");
    horus::Subscriber<horus::msg::Imu> sub("alltype.imu");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::Imu msg{};
    msg.orientation[3] = 1.0;
    msg.linear_acceleration[2] = 9.81;
    msg.timestamp_ns = 3;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->orientation[3], 1.0, 1e-10);
    EXPECT_NEAR(r->get()->linear_acceleration[2], 9.81, 1e-10);
}

TEST(AllTypes, Odometry) {
    horus::Publisher<horus::msg::Odometry>  pub("alltype.odom");
    horus::Subscriber<horus::msg::Odometry> sub("alltype.odom");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::Odometry msg{};
    msg.pose.x = 1.0; msg.pose.y = 2.0; msg.pose.theta = 0.5;
    msg.twist.linear[0] = 0.3;
    msg.timestamp_ns = 4;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->pose.x, 1.0, 1e-10);
    EXPECT_NEAR(r->get()->twist.linear[0], 0.3, 1e-10);
}

TEST(AllTypes, JointState) {
    horus::Publisher<horus::msg::JointState>  pub("alltype.joints");
    horus::Subscriber<horus::msg::JointState> sub("alltype.joints");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::JointState msg{};
    msg.joint_count = 3;
    msg.positions[0] = 1.0;
    msg.positions[1] = 2.0;
    msg.positions[2] = 3.0;
    msg.velocities[0] = 0.1;
    std::strncpy(reinterpret_cast<char*>(msg.names[0]), "shoulder", 31);
    msg.timestamp_ns = 5;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->get()->joint_count, 3u);
    EXPECT_NEAR(r->get()->positions[0], 1.0, 1e-10);
    EXPECT_NEAR(r->get()->positions[2], 3.0, 1e-10);
}

TEST(AllTypes, Twist) {
    horus::Publisher<horus::msg::Twist>  pub("alltype.twist");
    horus::Subscriber<horus::msg::Twist> sub("alltype.twist");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::Twist msg{};
    msg.linear[0] = 1.0; msg.linear[1] = 2.0; msg.linear[2] = 3.0;
    msg.angular[0] = 0.1; msg.angular[1] = 0.2; msg.angular[2] = 0.3;
    msg.timestamp_ns = 6;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->linear[0], 1.0, 1e-10);
    EXPECT_NEAR(r->get()->angular[2], 0.3, 1e-10);
}

TEST(AllTypes, Pose2D) {
    horus::Publisher<horus::msg::Pose2D>  pub("alltype.pose");
    horus::Subscriber<horus::msg::Pose2D> sub("alltype.pose");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::Pose2D msg{};
    msg.x = 10.0; msg.y = 20.0; msg.theta = 1.57;
    msg.timestamp_ns = 7;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->x, 10.0, 1e-10);
    EXPECT_NEAR(r->get()->theta, 1.57, 1e-10);
}

TEST(AllTypes, TransformStamped) {
    horus::Publisher<horus::msg::TransformStamped>  pub("alltype.tf");
    horus::Subscriber<horus::msg::TransformStamped> sub("alltype.tf");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::TransformStamped msg{};
    msg.translation[0] = 1.0; msg.translation[1] = 2.0; msg.translation[2] = 3.0;
    msg.rotation[3] = 1.0;
    msg.timestamp_ns = 8;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->translation[0], 1.0, 1e-10);
}

TEST(AllTypes, NavGoal) {
    horus::Publisher<horus::msg::NavGoal>  pub("alltype.nav");
    horus::Subscriber<horus::msg::NavGoal> sub("alltype.nav");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::NavGoal msg{};
    msg.target_x = 5.0; msg.target_y = 3.0; msg.target_theta = 1.0;
    msg.tolerance = 0.1f;
    msg.timestamp_ns = 9;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR(r->get()->target_x, 5.0, 1e-10);
    EXPECT_NEAR(r->get()->tolerance, 0.1f, 0.001f);
}

TEST(AllTypes, Heartbeat) {
    horus::Publisher<horus::msg::Heartbeat>  pub("alltype.hb");
    horus::Subscriber<horus::msg::Heartbeat> sub("alltype.hb");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::Heartbeat msg{};
    msg.sequence = 42; msg.alive = true; msg.cpu_usage = 50.0f;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->get()->sequence, 42u);
    EXPECT_TRUE(r->get()->alive);
}

TEST(AllTypes, EmergencyStop) {
    horus::Publisher<horus::msg::EmergencyStop>  pub("alltype.estop");
    horus::Subscriber<horus::msg::EmergencyStop> sub("alltype.estop");
    ASSERT_TRUE(pub.is_valid() && sub.is_valid());
    horus::msg::EmergencyStop msg{};
    msg.engaged = 1; msg.auto_reset = 0;
    pub.send(msg);
    auto r = sub.recv();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->get()->engaged, 1);
    EXPECT_EQ(r->get()->auto_reset, 0);
}
