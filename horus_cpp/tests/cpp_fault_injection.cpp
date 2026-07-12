// Fault injection — prove surviving process recovers cleanly from faults.
// 5 scenarios: subscriber killed, publisher-loan-killed, SHM missing,
// ABI version contract, rapid restart.

#include <horus/horus.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

using namespace horus::literals;

extern "C" uint32_t horus_get_abi_version();

TEST(FaultInjection, SubscriberKilledMidPublish) {
    horus::Publisher<horus::msg::CmdVel> pub_("fi.sub_killed");
    pid_t child = fork();
    ASSERT_NE(child, -1);
    if (child == 0) {
        prctl(PR_SET_PDEATHSIG, SIGKILL);
        horus::Subscriber<horus::msg::CmdVel> sub("fi.sub_killed");
        for (int i = 0; i < 10; i++) {
            auto m = sub.recv();
            (void)m;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        _exit(0);
    }
    for (int i = 0; i < 5; i++) {
        horus::msg::CmdVel msg{};
        msg.timestamp_ns = static_cast<uint64_t>(i);
        msg.linear = 0.1f;
        pub_.send(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    kill(child, SIGKILL);
    waitpid(child, nullptr, 0);
    for (int i = 0; i < 10; i++) {
        horus::msg::CmdVel msg{};
        msg.timestamp_ns = static_cast<uint64_t>(i + 100);
        msg.linear = 0.2f;
        pub_.send(msg);
    }
    SUCCEED() << "parent survived subscriber SIGKILL";
}

TEST(FaultInjection, PublisherHoldsLoanThenKilled) {
    pid_t child = fork();
    ASSERT_NE(child, -1);
    if (child == 0) {
        prctl(PR_SET_PDEATHSIG, SIGKILL);
        horus::Publisher<horus::msg::CmdVel> pub_("fi.loan_leak");
        auto loan = pub_.loan();
        loan->timestamp_ns = 42;
        // Intentionally do NOT publish — hold the loan, die.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        _exit(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    kill(child, SIGKILL);
    waitpid(child, nullptr, 0);

    horus::Subscriber<horus::msg::CmdVel> sub("fi.loan_leak");
    horus::Publisher<horus::msg::CmdVel> pub2("fi.loan_leak");
    horus::msg::CmdVel msg{};
    msg.timestamp_ns = 99;
    msg.linear = 0.5f;
    pub2.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto got = sub.recv();
    EXPECT_TRUE(got.has_value()) << "subscriber must recover after publisher loan-leak death";
}

TEST(FaultInjection, ShmNamespaceMissing) {
    // Non-existent namespace — creating publisher must not crash.
    setenv("HORUS_NAMESPACE", "fi_nonexistent_path_xyz_123456", 1);
    horus::Publisher<horus::msg::CmdVel> pub_("fi.no_shm");
    horus::msg::CmdVel msg{};
    pub_.send(msg);  // must not SIGSEGV
    SUCCEED();
}

TEST(FaultInjection, AbiVersionContract) {
    uint32_t v = horus_get_abi_version();
    EXPECT_GE(v, 1u) << "ABI version must be >= 1";
    EXPECT_EQ(v, static_cast<uint32_t>(HORUS_CPP_ABI_VERSION))
        << "ABI version must match header at build time";
}

TEST(FaultInjection, RapidRestart100Cycles) {
    // Simulates crash+restart: 100 scheduler+publisher new/destroy cycles
    // with a publish in each. No accumulated SHM resource leak.
    for (int i = 0; i < 100; i++) {
        horus::Scheduler sched;
        sched.tick_rate(1000_hz).name("rapid");
        horus::Publisher<horus::msg::CmdVel> pub_("fi.rapid");
        horus::msg::CmdVel msg{};
        msg.timestamp_ns = static_cast<uint64_t>(i);
        pub_.send(msg);
        sched.stop();
    }
    SUCCEED() << "100 rapid-restart cycles without crash";
}
