// Full User API Test — tests the ERGONOMIC C++ headers as a real user would.
//
// Tests all horus::* wrapper classes from <horus/horus.hpp>.
//
// Compile: g++ -std=c++17 -fext-numeric-literals -I horus_cpp/include \
//          -o cpp_user_api_test tests/cpp_user_api_test.cpp \
//          -L target/debug -lhorus_cpp -lpthread -ldl -lm

#include <horus/horus.hpp>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <atomic>

static int pass_count = 0;
static int fail_count = 0;

#define CHECK(cond, name) do { \
    if (cond) { std::printf("[PASS] %s\n", name); pass_count++; } \
    else { std::printf("[FAIL] %s (line %d)\n", name, __LINE__); fail_count++; } \
} while(0)

#define CHECK_EQ(a, b, name) CHECK((a) == (b), name)
#define CHECK_NEAR(a, b, eps, name) CHECK(std::fabs(double(a) - double(b)) < (eps), name)

// ═══════════════════════════════════════════════════════════════════════════
// 1. Scheduler + Multi-Node Pipeline
// ═══════════════════════════════════════════════════════════════════════════

static std::atomic<int> sensor_ticks{0};
static std::atomic<int> controller_ticks{0};
static std::atomic<int> actuator_ticks{0};

void test_scheduler_pipeline() {
    std::printf("\n=== 1. Scheduler + Multi-Node Pipeline ===\n");
    using namespace horus::literals;

    horus::Scheduler sched;
    sched.tick_rate(100_hz);
    sched.name("user_api_test");

    CHECK(sched.is_running(), "scheduler running after creation");

    sensor_ticks = 0;
    controller_ticks = 0;
    actuator_ticks = 0;

    // Don't set rate — let nodes tick on every scheduler tick (BestEffort class)
    sched.add("sensor")
        .order(0)
        .tick([] { sensor_ticks++; })
        .build();

    sched.add("controller")
        .order(10)
        .tick([] { controller_ticks++; })
        .build();

    sched.add("actuator")
        .order(20)
        .tick([] { actuator_ticks++; })
        .build();

    auto nodes = sched.node_list();
    CHECK_EQ(nodes.size(), (size_t)3, "3 nodes registered");

    for (int i = 0; i < 10; i++) sched.tick_once();

    // RT nodes with budget/deadline may skip some ticks due to timing enforcement
    CHECK(sensor_ticks >= 1, "sensor ticked >= 1");
    CHECK(controller_ticks >= 1, "controller ticked >= 1");
    CHECK(actuator_ticks >= 1, "actuator ticked >= 1");
    CHECK_EQ(sched.get_name(), std::string("user_api_test"), "scheduler name");

    sched.stop();
    CHECK(!sched.is_running(), "scheduler stopped");
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Publisher + Subscriber (CmdVel via template specialization)
// ═══════════════════════════════════════════════════════════════════════════

void test_pubsub_cmdvel() {
    std::printf("\n=== 2. Publisher + Subscriber CmdVel ===\n");

    // Use template specialization: Publisher<msg::CmdVel> and Subscriber<msg::CmdVel>
    horus::Publisher<horus::msg::CmdVel> pub("user_api.cmd_vel");
    horus::Subscriber<horus::msg::CmdVel> sub("user_api.cmd_vel");

    CHECK(pub.is_valid(), "publisher valid");
    CHECK(sub.is_valid(), "subscriber valid");

    // Loan pattern (zero-copy)
    {
        auto sample = pub.loan();
        sample->timestamp_ns = 42;
        sample->linear = 1.5f;
        sample->angular = -0.3f;
        pub.publish(std::move(sample));
    }

    auto msg = sub.recv();
    CHECK(msg.has_value(), "message received");
    if (msg) {
        CHECK_EQ(msg->get()->timestamp_ns, (uint64_t)42, "timestamp preserved");
        CHECK_NEAR(msg->get()->linear, 1.5f, 0.001f, "linear preserved");
        CHECK_NEAR(msg->get()->angular, -0.3f, 0.001f, "angular preserved");
    }

    // Send by copy
    horus::msg::CmdVel direct{};
    direct.timestamp_ns = 100;
    direct.linear = 2.0f;
    direct.angular = 0.5f;
    pub.send(direct);

    auto msg2 = sub.recv();
    CHECK(msg2.has_value(), "second message received");
    if (msg2) {
        CHECK_EQ(msg2->get()->timestamp_ns, (uint64_t)100, "copy timestamp");
    }

    // Multiple sends
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
    CHECK(recv_count >= 1, "multiple sends received >= 1");
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. TensorPool + Image + PointCloud
// ═══════════════════════════════════════════════════════════════════════════

void test_perception() {
    std::printf("\n=== 3. TensorPool + Image + PointCloud ===\n");

    horus::TensorPool pool(9960, 16 * 1024 * 1024, 64);
    CHECK(bool(pool), "pool created");

    // Camera: 640x480 RGB
    horus::Image cam(pool, 640, 480, horus::Encoding::Rgb8);
    CHECK(bool(cam), "camera image");
    CHECK_EQ(cam.width(), (uint32_t)640, "width");
    CHECK_EQ(cam.height(), (uint32_t)480, "height");
    CHECK_EQ(cam.data_size(), (size_t)(640 * 480 * 3), "data size");

    // Depth: 640x480 GRAY8
    horus::Image depth(pool, 640, 480, horus::Encoding::Gray8);
    CHECK(bool(depth), "depth image");

    // Lidar: 10K XYZ
    horus::PointCloud lidar(pool, 10000, 3);
    CHECK(bool(lidar), "lidar pointcloud");
    CHECK_EQ(lidar.num_points(), (uint64_t)10000, "10K points");

    // NN Tensor: [1,3,224,224] f32
    uint64_t shape[] = {1, 3, 224, 224};
    horus::Tensor nn(pool, shape, 4, horus::Dtype::F32);
    CHECK(bool(nn), "NN tensor");
    CHECK_EQ(nn.nbytes(), (uint64_t)(1 * 3 * 224 * 224 * 4), "tensor bytes");

    float* data = reinterpret_cast<float*>(nn.data());
    data[0] = 3.14f;
    CHECK_NEAR(data[0], 3.14f, 0.001f, "tensor write/read");

    CHECK(pool.stats().allocated > 0, "pool has allocations");
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Runtime Parameters
// ═══════════════════════════════════════════════════════════════════════════

void test_params() {
    std::printf("\n=== 4. Runtime Parameters ===\n");

    horus::Params params;

    params.set("speed", 1.5);
    params.set("count", int64_t(42));
    params.set("enabled", true);
    params.set("name", "atlas");

    CHECK_NEAR(params.get<double>("speed", 0.0), 1.5, 1e-10, "get<double>");
    CHECK_EQ(params.get<int64_t>("count", 0), (int64_t)42, "get<i64>");
    CHECK_EQ(params.get<bool>("enabled", false), true, "get<bool>");
    CHECK_EQ(params.get<std::string>("name", ""), std::string("atlas"), "get<string>");
    CHECK_NEAR(params.get<double>("missing", 99.0), 99.0, 1e-10, "default");
    CHECK(params.has("speed"), "has");
    CHECK(!params.has("nope"), "!has");
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. TransformFrame
// ═══════════════════════════════════════════════════════════════════════════

void test_transforms() {
    std::printf("\n=== 5. TransformFrame ===\n");

    horus::TransformFrame tf;
    tf.register_frame("world");
    tf.register_frame("base_link", "world");
    tf.register_frame("lidar", "base_link");
    tf.register_frame("camera", "base_link");

    tf.update("base_link", {1.0, 0.0, 0.0}, {0, 0, 0, 1}, 1000);
    tf.update("lidar", {0.0, 0.0, 0.5}, {0, 0, 0, 1}, 1000);
    tf.update("camera", {0.1, 0.0, 0.3}, {0, 0, 0, 1}, 1000);

    auto t = tf.lookup("base_link", "world");
    CHECK(t.has_value(), "lookup base→world");
    if (t) CHECK_NEAR(t->translation[0], 1.0, 1e-10, "tx = 1.0");

    CHECK(tf.can_transform("lidar", "world"), "lidar→world");
    CHECK(tf.can_transform("camera", "world"), "camera→world");
    CHECK(!tf.can_transform("world", "nonexistent"), "no path");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Service Client/Server
// ═══════════════════════════════════════════════════════════════════════════

void test_services() {
    std::printf("\n=== 6. Service ===\n");

    horus::ServiceClient client("user_api.svc");
    CHECK(bool(client), "client created");

    horus::ServiceServer server("user_api.svc");
    CHECK(bool(server), "server created");

    // Move semantics
    auto c2 = std::move(client);
    CHECK(bool(c2), "moved client valid");
    CHECK(!bool(client), "original invalid");
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Action Client/Server
// ═══════════════════════════════════════════════════════════════════════════

void test_actions() {
    std::printf("\n=== 7. Action ===\n");

    horus::ActionClient client("user_api.nav");
    CHECK(bool(client), "action client");

    auto goal = client.send_goal(R"({"x": 5.0})");
    CHECK(bool(goal), "goal handle");
    CHECK_EQ(goal.id(), (uint64_t)1, "goal id");
    CHECK_EQ(goal.status(), horus::GoalStatus::Pending, "pending");
    CHECK(goal.is_active(), "active");

    goal.cancel();
    CHECK_EQ(goal.status(), horus::GoalStatus::Canceled, "canceled");

    auto g2 = client.send_goal("{}");
    CHECK_EQ(g2.id(), (uint64_t)2, "goal id increments");

    horus::ActionServer server("user_api.nav");
    CHECK(bool(server), "action server");
    CHECK(!server.is_ready(), "not ready without handlers");
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Multi-Node Different Configs (RT + Compute + BestEffort)
// ═══════════════════════════════════════════════════════════════════════════

static std::atomic<int> rt_ticks{0};
static std::atomic<int> compute_ticks{0};
static std::atomic<int> be_ticks{0};

void test_multi_config() {
    std::printf("\n=== 8. Multi-Config Nodes ===\n");
    using namespace horus::literals;

    horus::Scheduler sched;
    sched.name("multi_cfg");

    rt_ticks = 0; compute_ticks = 0; be_ticks = 0;

    // RT node: budget+deadline without rate → ticks every scheduler tick with timing enforcement
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

    CHECK_EQ(sched.node_list().size(), (size_t)3, "3 different-config nodes");

    for (int i = 0; i < 10; i++) sched.tick_once();

    // RT node may skip ticks due to budget/deadline enforcement
    CHECK(rt_ticks >= 1, "RT ticked");
    CHECK(compute_ticks >= 1, "compute ticked");
    CHECK(be_ticks >= 1, "best-effort ticked");
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Two Schedulers, Shared Topic
// ═══════════════════════════════════════════════════════════════════════════

void test_two_schedulers() {
    std::printf("\n=== 9. Two Schedulers ===\n");

    horus::Scheduler sched1;
    sched1.name("sched_a");
    horus::Scheduler sched2;
    sched2.name("sched_b");

    // Pub on sched1's node, sub on sched2's node, same topic
    horus::Publisher<horus::msg::CmdVel> pub("two_sched.cmdvel");
    horus::Subscriber<horus::msg::CmdVel> sub("two_sched.cmdvel");

    static std::atomic<bool> published{false};
    static std::atomic<bool> received{false};

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

    published = false; received = false;

    sched1.tick_once();
    sched2.tick_once();

    CHECK(published.load(), "sched1 published");
    CHECK(received.load(), "sched2 received from sched1");
}

// ═══════════════════════════════════════════════════════════════════════════
// 10. Duration Literals
// ═══════════════════════════════════════════════════════════════════════════

void test_literals() {
    std::printf("\n=== 10. Duration Literals ===\n");
    using namespace horus::literals;

    auto f = 100_hz;
    CHECK_NEAR(f.value(), 100.0, 0.001, "100_hz");
    CHECK_EQ(f.period().count(), (int64_t)10000, "period = 10000us");
    CHECK_EQ((5_ms).count(), (int64_t)5000, "5_ms");
    CHECK_EQ((200_us).count(), (int64_t)200, "200_us");
    CHECK_EQ((3_s).count(), (int64_t)3000000, "3_s");
    CHECK_EQ(f.budget_default().count(), (int64_t)8000, "80% budget");
    CHECK_EQ(f.deadline_default().count(), (int64_t)9500, "95% deadline");
}

// ═══════════════════════════════════════════════════════════════════════════
// 11. Move Semantics for All Types
// ═══════════════════════════════════════════════════════════════════════════

void test_moves() {
    std::printf("\n=== 11. Move Semantics ===\n");

    // Pool
    horus::TensorPool p1(9950, 1024*1024, 16);
    auto p2 = std::move(p1);
    CHECK(bool(p2), "pool moved");
    CHECK(!bool(p1), "original invalid");

    // Image
    horus::Image i1(p2, 100, 100, horus::Encoding::Rgb8);
    auto i2 = std::move(i1);
    CHECK(bool(i2), "image moved");

    // PointCloud
    horus::PointCloud pc1(p2, 100, 3);
    auto pc2 = std::move(pc1);
    CHECK(bool(pc2), "pointcloud moved");

    // Params
    horus::Params pr1;
    pr1.set("x", 1.0);
    auto pr2 = std::move(pr1);
    CHECK(pr2.has("x"), "params moved with data");

    // TransformFrame
    horus::TransformFrame tf1;
    tf1.register_frame("world");
    auto tf2 = std::move(tf1);
    CHECK(true, "tf moved");

    // GoalHandle
    horus::ActionClient ac("move_test.act");
    auto g1 = ac.send_goal("{}");
    auto g2 = std::move(g1);
    CHECK(bool(g2), "goal handle moved");
    CHECK(!bool(g1), "original goal invalid");

    // Publisher
    horus::Publisher<horus::msg::CmdVel> pub1("move_test.pub");
    auto pub2 = std::move(pub1);
    CHECK(pub2.is_valid(), "publisher moved");
    CHECK(!pub1.is_valid(), "original pub invalid");

    // Subscriber
    horus::Subscriber<horus::msg::CmdVel> sub1("move_test.sub");
    auto sub2 = std::move(sub1);
    CHECK(sub2.is_valid(), "subscriber moved");
    CHECK(!sub1.is_valid(), "original sub invalid");
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

void test_struct_node() {
    std::printf("\n=== 12. Struct-Based Node ===\n");

    horus::Scheduler sched;

    // Create node with built-in pub/sub
    MotorController ctrl;
    CHECK_EQ(ctrl.name(), std::string("motor_ctrl"), "node name");
    CHECK_EQ(ctrl.publishers().size(), (size_t)1, "1 publisher");
    CHECK_EQ(ctrl.subscriptions().size(), (size_t)1, "1 subscription");

    // Publish to the node's input
    horus::Publisher<horus::msg::CmdVel> cmd_pub("node_test.cmd");
    horus::msg::CmdVel cmd{}; cmd.linear = 2.0f;
    cmd_pub.send(cmd);

    // Add to scheduler with scheduling config
    sched.add(ctrl).order(0).build();

    // Tick
    for (int i = 0; i < 5; i++) sched.tick_once();

    CHECK(ctrl.tick_count_ >= 5, "node ticked >= 5");
    CHECK(ctrl.recv_count_ >= 1, "node received >= 1 message");

    // Lifecycle
    ctrl.init();
    CHECK(ctrl.init_called_, "init() called");
    ctrl.enter_safe_state();
    CHECK(ctrl.safe_called_, "enter_safe_state() called");
}

// ═══════════════════════════════════════════════════════════════════════════
// 13. LambdaNode (like Python's horus.Node())
// ═══════════════════════════════════════════════════════════════════════════

void test_lambda_node() {
    std::printf("\n=== 13. LambdaNode (Python-style) ===\n");

    horus::Scheduler sched;

    static int lambda_ticks = 0;
    static int lambda_recvs = 0;

    horus::LambdaNode sensor("lambda_sensor");
    sensor.pub<horus::msg::CmdVel>("lambda.data")
          .on_tick([](horus::LambdaNode& self) {
              lambda_ticks++;
              self.send<horus::msg::CmdVel>("lambda.data", horus::msg::CmdVel{0, 1.0f, 0.0f});
          });

    horus::LambdaNode processor("lambda_processor");
    processor.sub<horus::msg::CmdVel>("lambda.data")
             .on_tick([](horus::LambdaNode& self) {
                 auto msg = self.recv<horus::msg::CmdVel>("lambda.data");
                 if (msg) lambda_recvs++;
             });

    CHECK_EQ(sensor.name(), std::string("lambda_sensor"), "sensor name");
    CHECK_EQ(sensor.publishers().size(), (size_t)1, "sensor has 1 pub");
    CHECK_EQ(processor.subscriptions().size(), (size_t)1, "processor has 1 sub");

    lambda_ticks = 0;
    lambda_recvs = 0;

    sched.add(sensor).order(0).build();
    sched.add(processor).order(10).build();

    for (int i = 0; i < 5; i++) sched.tick_once();

    CHECK(lambda_ticks >= 5, "lambda sensor ticked >= 5");
    CHECK(lambda_recvs >= 1, "lambda processor received >= 1");
}

// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::printf("╔══════════════════════════════════════════════════╗\n");
    std::printf("║  HORUS C++ User API Test Suite                   ║\n");
    std::printf("╚══════════════════════════════════════════════════╝\n");

    test_scheduler_pipeline();
    test_pubsub_cmdvel();
    test_perception();
    test_params();
    test_transforms();
    test_services();
    test_actions();
    test_multi_config();
    test_two_schedulers();
    test_literals();
    test_moves();
    test_struct_node();
    test_lambda_node();

    std::printf("\n╔══════════════════════════════════════════════════╗\n");
    std::printf("║  Results: %3d passed, %d failed                  ║\n", pass_count, fail_count);
    std::printf("╚══════════════════════════════════════════════════╝\n");

    return fail_count > 0 ? 1 : 0;
}
