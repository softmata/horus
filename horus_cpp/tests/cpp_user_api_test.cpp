// Full User API Test — tests the ERGONOMIC C++ headers as a real user would.
// Tests all horus::* wrapper classes from <horus/horus.hpp>.
// Converted to GoogleTest.

#include <horus/horus.hpp>
#include <gtest/gtest.h>
#include <atomic>
#include <cmath>
#include <cstring>

// ═══════════════════════════════════════════════════════════════════════════
// 1. Scheduler + Multi-Node Pipeline
// ═══════════════════════════════════════════════════════════════════════════

namespace {
std::atomic<int> sensor_ticks{0};
std::atomic<int> controller_ticks{0};
std::atomic<int> actuator_ticks{0};
}

TEST(UserApi, SchedulerMultiNodePipeline) {
    using namespace horus::literals;
    horus::Scheduler sched;
    sched.tick_rate(100_hz);
    sched.name("user_api_test");
    EXPECT_TRUE(sched.is_running());

    sensor_ticks = 0; controller_ticks = 0; actuator_ticks = 0;

    sched.add("sensor").order(0)
        .tick([] { sensor_ticks++; }).build();
    sched.add("controller").order(10)
        .tick([] { controller_ticks++; }).build();
    sched.add("actuator").order(20)
        .tick([] { actuator_ticks++; }).build();

    EXPECT_EQ(sched.node_list().size(), 3u);

    for (int i = 0; i < 10; i++) sched.tick_once();

    EXPECT_GE(sensor_ticks, 1);
    EXPECT_GE(controller_ticks, 1);
    EXPECT_GE(actuator_ticks, 1);
    EXPECT_EQ(sched.get_name(), std::string("user_api_test"));

    sched.stop();
    EXPECT_FALSE(sched.is_running());
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Publisher + Subscriber (CmdVel)
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, PubSubCmdVelLoanAndSend) {
    horus::Publisher<horus::msg::CmdVel> pub("user_api.cmd_vel");
    horus::Subscriber<horus::msg::CmdVel> sub("user_api.cmd_vel");
    ASSERT_TRUE(pub.is_valid());
    ASSERT_TRUE(sub.is_valid());

    {
        auto sample = pub.loan();
        sample->timestamp_ns = 42;
        sample->linear = 1.5f;
        sample->angular = -0.3f;
        pub.publish(std::move(sample));
    }
    auto msg = sub.recv();
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->get()->timestamp_ns, 42u);
    EXPECT_NEAR(msg->get()->linear, 1.5f, 0.001f);
    EXPECT_NEAR(msg->get()->angular, -0.3f, 0.001f);

    horus::msg::CmdVel direct{};
    direct.timestamp_ns = 100;
    direct.linear = 2.0f;
    direct.angular = 0.5f;
    pub.send(direct);

    auto msg2 = sub.recv();
    ASSERT_TRUE(msg2.has_value());
    EXPECT_EQ(msg2->get()->timestamp_ns, 100u);

    for (int i = 0; i < 5; i++) {
        horus::msg::CmdVel m{};
        m.timestamp_ns = static_cast<uint64_t>(200 + i);
        m.linear = static_cast<float>(i);
        pub.send(m);
    }
    int recv_count = 0;
    for (int i = 0; i < 10; i++) {
        auto m = sub.recv();
        if (m) recv_count++;
    }
    EXPECT_GE(recv_count, 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. TensorPool + Image + PointCloud + Tensor
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, PerceptionPoolImagePointcloudTensor) {
    horus::TensorPool pool(9960, 16 * 1024 * 1024, 64);
    ASSERT_TRUE(bool(pool));

    horus::Image cam(pool, 640, 480, horus::Encoding::Rgb8);
    ASSERT_TRUE(bool(cam));
    EXPECT_EQ(cam.width(), 640u);
    EXPECT_EQ(cam.height(), 480u);
    EXPECT_EQ(cam.data_size(), static_cast<size_t>(640 * 480 * 3));

    horus::Image depth(pool, 640, 480, horus::Encoding::Gray8);
    EXPECT_TRUE(bool(depth));

    horus::PointCloud lidar(pool, 10000, 3);
    ASSERT_TRUE(bool(lidar));
    EXPECT_EQ(lidar.num_points(), 10000u);

    uint64_t shape[] = {1, 3, 224, 224};
    horus::Tensor nn(pool, shape, 4, horus::Dtype::F32);
    ASSERT_TRUE(bool(nn));
    EXPECT_EQ(nn.nbytes(), static_cast<uint64_t>(1 * 3 * 224 * 224 * 4));

    float* data = reinterpret_cast<float*>(nn.data());
    data[0] = 3.14f;
    EXPECT_NEAR(data[0], 3.14f, 0.001f);

    EXPECT_GT(pool.stats().allocated, 0u);
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Runtime Parameters
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, RuntimeParams) {
    horus::Params params;
    params.set("speed", 1.5);
    params.set("count", int64_t(42));
    params.set("enabled", true);
    params.set("name", "atlas");

    EXPECT_NEAR(params.get<double>("speed", 0.0), 1.5, 1e-10);
    EXPECT_EQ(params.get<int64_t>("count", 0), 42);
    EXPECT_TRUE(params.get<bool>("enabled", false));
    EXPECT_EQ(params.get<std::string>("name", ""), "atlas");
    EXPECT_NEAR(params.get<double>("missing", 99.0), 99.0, 1e-10);
    EXPECT_TRUE(params.has("speed"));
    EXPECT_FALSE(params.has("nope"));
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. TransformFrame
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, TransformFrameRegisterUpdateLookup) {
    horus::TransformFrame tf;
    tf.register_frame("world");
    tf.register_frame("base_link", "world");
    tf.register_frame("lidar", "base_link");
    tf.register_frame("camera", "base_link");

    tf.update("base_link", {1.0, 0.0, 0.0}, {0, 0, 0, 1}, 1000);
    tf.update("lidar", {0.0, 0.0, 0.5}, {0, 0, 0, 1}, 1000);
    tf.update("camera", {0.1, 0.0, 0.3}, {0, 0, 0, 1}, 1000);

    auto t = tf.lookup("base_link", "world");
    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(t->translation[0], 1.0, 1e-10);

    EXPECT_TRUE(tf.can_transform("lidar", "world"));
    EXPECT_TRUE(tf.can_transform("camera", "world"));
    EXPECT_FALSE(tf.can_transform("world", "nonexistent"));
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Service Client/Server
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, ServiceCreateAndMove) {
    horus::ServiceClient client("user_api.svc");
    EXPECT_TRUE(bool(client));

    horus::ServiceServer server("user_api.svc");
    EXPECT_TRUE(bool(server));

    auto c2 = std::move(client);
    EXPECT_TRUE(bool(c2));
    EXPECT_FALSE(bool(client));
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Action Client/Server
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, ActionSendGoalAndCancel) {
    horus::ActionClient client("user_api.nav");
    ASSERT_TRUE(bool(client));

    auto goal = client.send_goal(R"({"x": 5.0})");
    ASSERT_TRUE(bool(goal));
    EXPECT_NE(goal.id(), 0u); // ids are process-unique, not necessarily 1
    EXPECT_EQ(goal.status(), horus::GoalStatus::Pending);
    EXPECT_TRUE(goal.is_active());

    // Publishes a cancel over the topic; does not change the local handle status.
    goal.cancel();

    auto g2 = client.send_goal("{}");
    EXPECT_NE(g2.id(), 0u);
    EXPECT_NE(g2.id(), goal.id()); // ids are unique

    horus::ActionServer server("user_api.nav");
    EXPECT_TRUE(bool(server));
    EXPECT_FALSE(server.is_ready());
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Multi-Node Different Configs (RT + Compute + BestEffort)
// ═══════════════════════════════════════════════════════════════════════════

namespace {
std::atomic<int> rt_ticks{0};
std::atomic<int> compute_ticks{0};
std::atomic<int> be_ticks{0};
}

TEST(UserApi, MultiConfigNodes) {
    using namespace horus::literals;
    horus::Scheduler sched;
    sched.name("multi_cfg");

    rt_ticks = 0; compute_ticks = 0; be_ticks = 0;

    sched.add("rt_motor")
        .budget(5_ms).deadline(8_ms)
        .on_miss(horus::Miss::SafeMode).order(0)
        .tick([] { rt_ticks++; }).build();
    sched.add("planner")
        .compute().order(50)
        .tick([] { compute_ticks++; }).build();
    sched.add("logger")
        .order(100)
        .tick([] { be_ticks++; }).build();

    EXPECT_EQ(sched.node_list().size(), 3u);

    for (int i = 0; i < 10; i++) sched.tick_once();

    EXPECT_GE(rt_ticks, 1);
    EXPECT_GE(compute_ticks, 1);
    EXPECT_GE(be_ticks, 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Two Schedulers, Shared Topic
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, TwoSchedulersSharedTopic) {
    horus::Scheduler sched1;
    sched1.name("sched_a");
    horus::Scheduler sched2;
    sched2.name("sched_b");

    horus::Publisher<horus::msg::CmdVel> pub("two_sched.cmdvel");
    horus::Subscriber<horus::msg::CmdVel> sub("two_sched.cmdvel");

    static std::atomic<bool> published{false};
    static std::atomic<bool> received{false};
    published = false; received = false;

    sched1.add("pub_node").order(0)
        .tick([&] {
            if (!published) {
                horus::msg::CmdVel m{}; m.timestamp_ns = 77; m.linear = 3.14f;
                pub.send(m);
                published = true;
            }
        }).build();

    sched2.add("sub_node").order(0)
        .tick([&] {
            auto m = sub.recv();
            if (m) received = true;
        }).build();

    sched1.tick_once();
    sched2.tick_once();

    EXPECT_TRUE(published.load());
    EXPECT_TRUE(received.load());
}

// ═══════════════════════════════════════════════════════════════════════════
// 10. Duration Literals
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, DurationLiterals) {
    using namespace horus::literals;
    auto f = 100_hz;
    EXPECT_NEAR(f.value(), 100.0, 0.001);
    EXPECT_EQ(f.period().count(), 10000);
    EXPECT_EQ((5_ms).count(), 5000);
    EXPECT_EQ((200_us).count(), 200);
    EXPECT_EQ((3_s).count(), 3000000);
    EXPECT_EQ(f.budget_default().count(), 8000);
    EXPECT_EQ(f.deadline_default().count(), 9500);
}

// ═══════════════════════════════════════════════════════════════════════════
// 11. Move Semantics for All Types
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, MoveSemanticsAllTypes) {
    horus::TensorPool p1(9950, 1024*1024, 16);
    auto p2 = std::move(p1);
    EXPECT_TRUE(bool(p2));
    EXPECT_FALSE(bool(p1));

    horus::Image i1(p2, 100, 100, horus::Encoding::Rgb8);
    auto i2 = std::move(i1);
    EXPECT_TRUE(bool(i2));

    horus::PointCloud pc1(p2, 100, 3);
    auto pc2 = std::move(pc1);
    EXPECT_TRUE(bool(pc2));

    horus::Params pr1;
    pr1.set("x", 1.0);
    auto pr2 = std::move(pr1);
    EXPECT_TRUE(pr2.has("x"));

    horus::TransformFrame tf1;
    tf1.register_frame("world");
    auto tf2 = std::move(tf1);
    (void)tf2;

    horus::ActionClient ac("move_test.act");
    auto g1 = ac.send_goal("{}");
    auto g2 = std::move(g1);
    EXPECT_TRUE(bool(g2));
    EXPECT_FALSE(bool(g1));

    horus::Publisher<horus::msg::CmdVel> pub1("move_test.pub");
    auto pub2 = std::move(pub1);
    EXPECT_TRUE(pub2.is_valid());
    EXPECT_FALSE(pub1.is_valid());

    horus::Subscriber<horus::msg::CmdVel> sub1("move_test.sub");
    auto sub2 = std::move(sub1);
    EXPECT_TRUE(sub2.is_valid());
    EXPECT_FALSE(sub1.is_valid());
}

// ═══════════════════════════════════════════════════════════════════════════
// 12. Struct-Based Node (like Rust `impl Node`)
// ═══════════════════════════════════════════════════════════════════════════

class MotorController : public horus::Node {
public:
    MotorController() : Node("motor_ctrl") {
        cmd_sub_ = subscribe<horus::msg::CmdVel>("node_test.cmd");
        motor_pub_ = advertise<horus::msg::CmdVel>("node_test.motor");
    }

    void tick() override {
        tick_count_++;
        auto msg = cmd_sub_->recv();
        if (msg) {
            recv_count_++;
            horus::msg::CmdVel out{};
            out.linear = msg->get()->linear * 0.5f;
            motor_pub_->send(out);
        }
    }

    void init() override { init_called_ = true; }
    void enter_safe_state() override { safe_called_ = true; }

    int tick_count_ = 0;
    int recv_count_ = 0;
    bool init_called_ = false;
    bool safe_called_ = false;

private:
    horus::Subscriber<horus::msg::CmdVel>* cmd_sub_;
    horus::Publisher<horus::msg::CmdVel>*  motor_pub_;
};

TEST(UserApi, StructBasedNode) {
    horus::Scheduler sched;
    MotorController ctrl;

    EXPECT_EQ(ctrl.name(), "motor_ctrl");
    EXPECT_EQ(ctrl.publishers().size(), 1u);
    EXPECT_EQ(ctrl.subscriptions().size(), 1u);

    horus::Publisher<horus::msg::CmdVel> cmd_pub("node_test.cmd");
    horus::msg::CmdVel cmd{}; cmd.linear = 2.0f;
    cmd_pub.send(cmd);

    sched.add(ctrl).order(0).build();

    for (int i = 0; i < 5; i++) sched.tick_once();

    EXPECT_GE(ctrl.tick_count_, 5);
    EXPECT_GE(ctrl.recv_count_, 1);

    ctrl.init();
    EXPECT_TRUE(ctrl.init_called_);
    ctrl.enter_safe_state();
    EXPECT_TRUE(ctrl.safe_called_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 13. LambdaNode
// ═══════════════════════════════════════════════════════════════════════════

TEST(UserApi, LambdaNode) {
    horus::Scheduler sched;

    static int lambda_ticks = 0;
    static int lambda_recvs = 0;
    lambda_ticks = 0; lambda_recvs = 0;

    horus::LambdaNode sensor("lambda_sensor");
    sensor.pub<horus::msg::CmdVel>("lambda.data")
          .on_tick([](horus::LambdaNode& self) {
              lambda_ticks++;
              self.send<horus::msg::CmdVel>("lambda.data",
                                            horus::msg::CmdVel{0, 1.0f, 0.0f});
          });

    horus::LambdaNode processor("lambda_processor");
    processor.sub<horus::msg::CmdVel>("lambda.data")
             .on_tick([](horus::LambdaNode& self) {
                 auto msg = self.recv<horus::msg::CmdVel>("lambda.data");
                 if (msg) lambda_recvs++;
             });

    EXPECT_EQ(sensor.name(), "lambda_sensor");
    EXPECT_EQ(sensor.publishers().size(), 1u);
    EXPECT_EQ(processor.subscriptions().size(), 1u);

    sched.add(sensor).order(0).build();
    sched.add(processor).order(10).build();

    for (int i = 0; i < 5; i++) sched.tick_once();

    EXPECT_GE(lambda_ticks, 5);
    EXPECT_GE(lambda_recvs, 1);
}
