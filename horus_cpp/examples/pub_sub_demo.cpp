// Pub/Sub Demo — HORUS C++ Ergonomic API
//
// Demonstrates Publisher<CmdVel> and Subscriber<CmdVel> with the
// loan pattern through the full Rust SHM stack.
//
// Compile: g++ -std=c++17 -I include -o pub_sub_demo examples/pub_sub_demo.cpp -L ../target/debug -lhorus_cpp -lpthread -ldl -lm
// Run:     LD_LIBRARY_PATH=../target/debug ./pub_sub_demo

#include <horus/horus.hpp>
#include <cstdio>

using namespace horus::literals;

int main() {
    std::printf("=== HORUS C++ Pub/Sub Demo ===\n\n");

    // Create scheduler
    horus::Scheduler sched;
    sched.tick_rate(100_hz).name("pub_sub_demo");

    // Create publisher and subscriber on the same topic
    horus::Publisher<horus::msg::CmdVel> pub_("demo.cmd_vel");
    horus::Subscriber<horus::msg::CmdVel> sub("demo.cmd_vel");

    if (!pub_.is_valid()) {
        std::printf("[FAIL] Publisher creation failed\n");
        return 1;
    }
    if (!sub.is_valid()) {
        std::printf("[FAIL] Subscriber creation failed\n");
        return 1;
    }
    std::printf("[PASS] Publisher and Subscriber created\n");

    // Test 1: Send via loan pattern
    {
        auto sample = pub_.loan();
        sample->timestamp_ns = 42;
        sample->linear = 0.5f;
        sample->angular = -0.1f;
        pub_.publish(std::move(sample));
        std::printf("[PASS] Published via loan pattern\n");
    }

    // Test 2: Receive
    {
        auto msg = sub.recv();
        if (msg) {
            std::printf("[PASS] Received: timestamp=%lu linear=%.2f angular=%.2f\n",
                (*msg)->timestamp_ns, (*msg)->linear, (*msg)->angular);

            // Verify data integrity
            bool ok = ((*msg)->timestamp_ns == 42)
                   && ((*msg)->linear > 0.49f && (*msg)->linear < 0.51f)
                   && ((*msg)->angular > -0.11f && (*msg)->angular < -0.09f);
            std::printf("[%s] Data integrity check\n", ok ? "PASS" : "FAIL");
        } else {
            std::printf("[FAIL] No message received\n");
        }
    }

    // Test 3: Send via copy
    {
        horus::msg::CmdVel cmd;
        cmd.timestamp_ns = 99;
        cmd.linear = 1.0f;
        cmd.angular = 0.0f;
        pub_.send(cmd);

        auto msg = sub.recv();
        if (msg && (*msg)->timestamp_ns == 99 && (*msg)->linear > 0.99f) {
            std::printf("[PASS] Send-by-copy roundtrip\n");
        } else {
            std::printf("[FAIL] Send-by-copy roundtrip\n");
        }
    }

    // Test 4: Empty recv
    {
        auto msg = sub.recv();
        if (!msg) {
            std::printf("[PASS] Empty recv returns nullopt\n");
        } else {
            std::printf("[FAIL] Expected nullopt\n");
        }
    }

    // Test 5: has_msg
    {
        pub_.send(horus::msg::CmdVel{0, 0.0f, 0.0f});
        if (sub.has_msg()) {
            std::printf("[PASS] has_msg() detects pending message\n");
            sub.recv(); // consume
        } else {
            std::printf("[FAIL] has_msg() missed pending message\n");
        }
    }

    std::printf("\n=== Done ===\n");
    return 0;
}
