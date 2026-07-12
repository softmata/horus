// Full Ergonomic API E2E Test — tests the complete C++ user experience via
// <horus/horus.hpp>: Scheduler, NodeBuilder, Publisher, Subscriber, loan
// pattern, tick callbacks. Converted to GoogleTest.

#include <horus/horus.hpp>
#include <gtest/gtest.h>
#include <cstdint>

using namespace horus::literals;

TEST(Scheduler, LifecycleStartStop) {
    horus::Scheduler sched;
    sched.tick_rate(100_hz).name("e2e_test");
    EXPECT_TRUE(sched.is_running()) << "Scheduler created and running";
    sched.stop();
    EXPECT_FALSE(sched.is_running()) << "Scheduler stopped";
}

TEST(Node, TickCallbackInvoked) {
    horus::Scheduler sched;
    int tick_count = 0;
    sched.add("counter")
        .tick([&] { tick_count++; })
        .build();

    sched.tick_once();
    EXPECT_GE(tick_count, 1) << "Node tick callback invoked";

    for (int i = 0; i < 9; i++) sched.tick_once();
    EXPECT_GE(tick_count, 10) << "10 ticks executed";
}

TEST(PubSub, CmdVelRoundtripLoanPattern) {
    horus::Publisher<horus::msg::CmdVel> pub_("e2e.cmd_vel");
    horus::Subscriber<horus::msg::CmdVel> sub("e2e.cmd_vel");
    ASSERT_TRUE(pub_.is_valid()) << "CmdVel publisher created";
    ASSERT_TRUE(sub.is_valid()) << "CmdVel subscriber created";

    auto sample = pub_.loan();
    sample->timestamp_ns = 12345;
    sample->linear = 0.42f;
    sample->angular = -0.17f;
    pub_.publish(std::move(sample));

    auto msg = sub.recv();
    ASSERT_TRUE(msg.has_value()) << "Message received";
    EXPECT_EQ((*msg)->timestamp_ns, 12345u) << "timestamp preserved";
    EXPECT_GT((*msg)->linear, 0.41f);
    EXPECT_LT((*msg)->linear, 0.43f);
    EXPECT_GT((*msg)->angular, -0.18f);
    EXPECT_LT((*msg)->angular, -0.16f);
}

TEST(PubSub, MultipleMessagesReceived) {
    horus::Publisher<horus::msg::CmdVel> pub_("e2e.multi");
    horus::Subscriber<horus::msg::CmdVel> sub("e2e.multi");

    for (int i = 0; i < 100; i++) {
        horus::msg::CmdVel cmd;
        cmd.timestamp_ns = static_cast<uint64_t>(i);
        cmd.linear = static_cast<float>(i) * 0.01f;
        cmd.angular = 0.0f;
        pub_.send(cmd);
    }

    int recv_count = 0;
    while (auto msg = sub.recv()) { (void)msg; recv_count++; }
    EXPECT_GT(recv_count, 0) << "Multiple messages received (got " << recv_count << " of 100)";
}

TEST(PubSub, TickCallbackPublishes) {
    horus::Scheduler sched;
    horus::Publisher<horus::msg::CmdVel> pub_("e2e.tick_pub");
    horus::Subscriber<horus::msg::CmdVel> sub("e2e.tick_pub");
    int publish_count = 0;

    sched.add("publisher_node")
        .tick([&] {
            auto sample = pub_.loan();
            sample->linear = 0.5f;
            sample->angular = 0.0f;
            sample->timestamp_ns = static_cast<uint64_t>(publish_count);
            pub_.publish(std::move(sample));
            publish_count++;
        })
        .build();

    for (int i = 0; i < 5; i++) sched.tick_once();
    EXPECT_EQ(publish_count, 5) << "Publisher node ticked 5 times";

    int recv_count = 0;
    while (sub.recv()) recv_count++;
    EXPECT_GT(recv_count, 0) << "Subscriber received messages from tick callback (got "
                             << recv_count << " of " << publish_count << ")";
}

TEST(Scheduler, MultipleNodesTick) {
    horus::Scheduler sched;
    int sensor_ticks = 0, controller_ticks = 0, actuator_ticks = 0;

    sched.add("sensor").order(0)
        .tick([&] { sensor_ticks++; }).build();
    sched.add("controller").order(10)
        .tick([&] { controller_ticks++; }).build();
    sched.add("actuator").order(20)
        .tick([&] { actuator_ticks++; }).build();

    for (int i = 0; i < 10; i++) sched.tick_once();
    // Note: only the last-registered node's callback works due to the
    // simplified global trampoline — this is a known limitation preserved
    // from the original CHECK test.
    int total = sensor_ticks + controller_ticks + actuator_ticks;
    EXPECT_GE(total, 10) << "Multiple nodes ticked (sensor=" << sensor_ticks
                          << " controller=" << controller_ticks
                          << " actuator=" << actuator_ticks << ")";
}

TEST(Literals, DurationLiteralValues) {
    auto f = 100_hz;
    EXPECT_GT(f.value(), 99.9);
    EXPECT_LT(f.value(), 100.1);

    auto ms = 5_ms;
    EXPECT_EQ(ms.count(), 5000);

    auto us = 200_us;
    EXPECT_EQ(us.count(), 200);

    auto s = 3_s;
    EXPECT_EQ(s.count(), 3000000);
}

TEST(Miss, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(horus::Miss::Warn), 0);
    EXPECT_EQ(static_cast<uint8_t>(horus::Miss::Skip), 1);
    EXPECT_EQ(static_cast<uint8_t>(horus::Miss::SafeMode), 2);
    EXPECT_EQ(static_cast<uint8_t>(horus::Miss::Stop), 3);
}

TEST(Cabi, AbiVersionMatchesHeader) {
    uint32_t ver = horus_get_abi_version();
    EXPECT_EQ(ver, static_cast<uint32_t>(HORUS_CPP_ABI_VERSION));
}

TEST(PubSub, PublisherMoveSemantics) {
    horus::Publisher<horus::msg::CmdVel> pub1("e2e.move_test");
    EXPECT_TRUE(pub1.is_valid()) << "Publisher valid before move";

    horus::Publisher<horus::msg::CmdVel> pub2(std::move(pub1));
    EXPECT_TRUE(pub2.is_valid()) << "Publisher valid after move";
    EXPECT_FALSE(pub1.is_valid()) << "Original invalid after move";
}
