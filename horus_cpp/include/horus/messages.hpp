// HORUS C++ Message Types — single include for all message categories.
//
// Usage:
//   #include <horus/messages.hpp>
//   horus::msg::CmdVel cmd{.timestamp_ns = 0, .linear = 0.5f, .angular = 0.1f};
//
// All structs are #[repr(C)] compatible with Rust for zero-copy SHM IPC.
#pragma once

#include "msg/geometry.hpp"
#include "msg/sensor.hpp"
#include "msg/control.hpp"
#include "msg/diagnostics.hpp"
#include "msg/navigation.hpp"
#include "msg/detection.hpp"
#include "msg/vision.hpp"
#include "msg/force.hpp"
#include "msg/time.hpp"
#include "msg/input.hpp"
