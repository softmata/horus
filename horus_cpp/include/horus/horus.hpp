// HORUS C++ API — single include for everything.
//
// Usage:
//   #include <horus/horus.hpp>
//   using namespace horus::literals;
//
//   int main() {
//       horus::Scheduler sched;
//       sched.tick_rate(100_hz).prefer_rt();
//
//       auto lidar = sched.subscribe<horus::msg::LaserScan>("lidar.scan");
//       auto cmd   = sched.advertise<horus::msg::CmdVel>("cmd_vel");
//
//       sched.add("controller")
//           .rate(50_hz)
//           .budget(5_ms)
//           .on_miss(horus::Miss::Skip)
//           .tick([&] {
//               auto scan = lidar.recv();
//               if (!scan) return;
//               auto out = cmd.loan();
//               out->linear  = scan->get()->ranges[0] > 0.5f ? 0.3f : 0.0f;
//               out->angular = scan->get()->ranges[0] > 0.5f ? 0.0f : 0.5f;
//               cmd.publish(std::move(out));
//           })
//           .build();
//
//       sched.spin();
//   }
#pragma once

// C API (must come first — implementations depend on it)
#include "horus_c.h"

// Core API declarations
#include "duration.hpp"
#include "error.hpp"
#include "scheduler.hpp"
#include "topic.hpp"
#include "node.hpp"

// Message types
#include "messages.hpp"

// Pool-backed types (Image, PointCloud, Tensor)
#include "pool.hpp"

// Runtime parameters
#include "params.hpp"

// RPC: Services and Actions
#include "service.hpp"
#include "action.hpp"

// Coordinate transforms
#include "transform.hpp"

// Logging and flight recorder
#include "log.hpp"
#include "blackbox.hpp"

// Implementations (header-only, inline)
#include "impl/scheduler_impl.hpp"
#include "impl/topic_impl.hpp"

// Bring literals into scope by default
namespace horus {
    using namespace literals;
}
