// HORUS Logging — send log messages visible in `horus log` CLI.
//
// Usage:
//   horus::log::info("controller", "Starting motor control at 100Hz");
//   horus::log::warn("sensor", "IMU calibration drift detected");
//   horus::log::error("safety", "Emergency stop triggered");
#pragma once

#include "horus_c.h"
#include <string>

namespace horus {
namespace log {

inline void info(const char* node, const char* msg)  { horus_log(0, node, msg); }
inline void warn(const char* node, const char* msg)  { horus_log(1, node, msg); }
inline void error(const char* node, const char* msg) { horus_log(2, node, msg); }

inline void info(const std::string& node, const std::string& msg)  { horus_log(0, node.c_str(), msg.c_str()); }
inline void warn(const std::string& node, const std::string& msg)  { horus_log(1, node.c_str(), msg.c_str()); }
inline void error(const std::string& node, const std::string& msg) { horus_log(2, node.c_str(), msg.c_str()); }

} // namespace log
} // namespace horus
