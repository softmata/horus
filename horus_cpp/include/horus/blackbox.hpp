// HORUS BlackBox — flight recorder for post-mortem crash analysis.
//
// Usage:
//   horus::blackbox::record("safety", "Emergency stop triggered by obstacle");
//   horus::blackbox::record("controller", "PID output saturated");
//
// Events are visible in `horus blackbox` CLI output.
#pragma once

#include "horus_c.h"
#include <string>

namespace horus {
namespace blackbox {

inline void record(const char* category, const char* message) {
    horus_blackbox_record(category, message);
}

inline void record(const std::string& category, const std::string& message) {
    horus_blackbox_record(category.c_str(), message.c_str());
}

} // namespace blackbox
} // namespace horus
