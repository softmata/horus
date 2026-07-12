# horus_cpp

C++ bindings for the HORUS robotics framework — zero-copy shared memory IPC,
deterministic scheduling, real-time safety via CXX FFI.

## Production Testing

horus_cpp targets production-critical robotics. Every commit runs:

- 139 Rust unit tests, 61 C++ gtest tests, 4 libFuzzer targets (60 s each), 4 proptest properties, 1 loom model.
- Four sanitizers: **ASan, TSan, UBSan, Valgrind** (all CI-gated).
- Code coverage enforced: **≥ 80% Rust** / **≥ 70% C++** via `cargo llvm-cov`.
- Nightly: 2-hour soak test, 5 fault-injection scenarios, 30-min long-run fuzz per target.

See [TESTING.md](./TESTING.md) for the full test matrix and how to run
everything locally.

## Quick Start

```cpp
#include <horus/horus.hpp>
using namespace horus::literals;

int main() {
    horus::Scheduler sched;
    sched.tick_rate(100_hz).prefer_rt();
    auto lidar = sched.subscribe<horus::msg::LaserScan>("lidar.scan");
    auto cmd   = sched.advertise<horus::msg::CmdVel>("cmd_vel");

    sched.add("controller")
        .rate(50_hz)
        .budget(5_ms)
        .on_miss(horus::Miss::Skip)
        .tick([&] {
            auto scan = lidar.recv();
            if (!scan) return;
            auto out = cmd.loan();
            out->linear  = scan->get()->ranges[0] > 0.5f ? 0.3f : 0.0f;
            out->angular = scan->get()->ranges[0] > 0.5f ? 0.0f : 0.5f;
            cmd.publish(std::move(out));
        })
        .build();

    sched.spin();
}
```

## Build

See the root [CLAUDE.md](../CLAUDE.md) for workspace-wide build commands.

## License

Apache-2.0. See [LICENSE](../LICENSE) in the repo root.
