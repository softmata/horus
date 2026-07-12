// Soak test — sustained publish/tick cycle with memory and handle monitoring.
// Default duration 2 hours (override via HORUS_SOAK_DURATION_SEC env var).
//
// Samples VmRSS, thread count, open-fd count every 60s. Baseline at t+300s
// (allow initial ramp). Fails if RSS grew > 10%, thread delta > 2, fd delta
// > 2 from baseline to end.

#include <horus/horus.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <string>
#include <thread>
#include <unistd.h>

using namespace horus::literals;

namespace {

long read_vmrss_kb() {
    std::ifstream f("/proc/self/status");
    std::string key;
    while (f >> key) {
        if (key == "VmRSS:") {
            long v;
            f >> v;
            return v;
        }
        std::string rest;
        std::getline(f, rest);
    }
    return 0;
}

int count_entries(const char* path) {
    DIR* d = opendir(path);
    if (!d) return 0;
    int n = 0;
    while (readdir(d)) n++;
    closedir(d);
    return n - 2;  // drop "." and ".."
}

int count_threads() { return count_entries("/proc/self/task"); }
int count_fds()     { return count_entries("/proc/self/fd"); }

}  // namespace

TEST(Soak, SustainedPubSubBoundedMemory) {
    const char* dur_env = std::getenv("HORUS_SOAK_DURATION_SEC");
    int duration_sec = dur_env ? std::atoi(dur_env) : 7200;
    ASSERT_GT(duration_sec, 60) << "duration must be > 60s";

    horus::Scheduler sched;
    sched.tick_rate(100_hz).name("soak");
    horus::Publisher<horus::msg::CmdVel> pub_("soak.cmd");
    horus::Subscriber<horus::msg::CmdVel> sub("soak.cmd");

    int ticks = 0;
    sched.add("soak_node").tick([&] {
        auto loan = pub_.loan();
        loan->timestamp_ns = static_cast<uint64_t>(ticks);
        loan->linear = 0.01f * (ticks % 100);
        loan->angular = 0.0f;
        pub_.publish(std::move(loan));
        auto msg = sub.recv();
        (void)msg;
        ticks++;
    }).build();

    auto t0 = std::chrono::steady_clock::now();
    long rss_t300 = 0, rss_last = 0, rss_max = 0;
    int threads_t300 = 0, threads_last = 0;
    int fds_t300 = 0, fds_last = 0;
    long last_sample_at = -1;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - t0).count();
        if (elapsed >= duration_sec) break;

        sched.tick_once();
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        if (elapsed > 0 && elapsed % 60 == 0 && elapsed != last_sample_at) {
            last_sample_at = elapsed;
            rss_last = read_vmrss_kb();
            threads_last = count_threads();
            fds_last = count_fds();
            if (rss_last > rss_max) rss_max = rss_last;
            std::fprintf(stderr,
                "[soak t+%lds] rss=%ldkB threads=%d fds=%d ticks=%d\n",
                static_cast<long>(elapsed), rss_last, threads_last, fds_last, ticks);
            if (elapsed == 300) {
                rss_t300 = rss_last;
                threads_t300 = threads_last;
                fds_t300 = fds_last;
            }
        }
    }

    ASSERT_GT(rss_t300, 0) << "soak duration too short to establish baseline (< 5 min)";
    double growth_pct = 100.0 * (rss_last - rss_t300) / rss_t300;
    std::fprintf(stderr,
        "[soak summary] rss_t300=%ldkB rss_end=%ldkB rss_max=%ldkB growth=%.2f%% ticks=%d\n",
        rss_t300, rss_last, rss_max, growth_pct, ticks);
    EXPECT_LT(growth_pct, 10.0) << "RSS grew > 10% after warm-up (memory leak)";
    EXPECT_LE(threads_last - threads_t300, 2) << "thread count grew unexpectedly";
    EXPECT_LE(fds_last - fds_t300, 2) << "fd count grew unexpectedly";
}
