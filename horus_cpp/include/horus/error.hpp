// HORUS C++ Error Types.
//
// Maps Rust's HorusError to C++ error handling patterns.
// CXX translates Rust Result::Err into C++ exceptions.
// These types provide structured error information.
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace horus {

/// Deadline miss policy — matches Rust Miss enum.
enum class Miss : uint8_t {
    Warn     = 0,  ///< Log warning, continue
    Skip     = 1,  ///< Skip this tick
    SafeMode = 2,  ///< Enter safe state
    Stop     = 3,  ///< Stop scheduler
};

/// HORUS error — thrown by FFI functions on failure.
///
/// C++ receives this when a Rust function returns Err.
/// CXX automatically converts Rust errors to C++ exceptions.
class Error : public std::runtime_error {
public:
    explicit Error(const std::string& msg) : std::runtime_error(msg) {}
    explicit Error(const char* msg) : std::runtime_error(msg) {}
};

} // namespace horus
