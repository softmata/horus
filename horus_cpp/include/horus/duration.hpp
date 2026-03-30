// HORUS C++ Duration & Frequency — user-defined literals.
//
// Matches Rust's DurationExt: 100_hz, 10_ms, 200_us
//
// Usage:
//   using namespace horus::literals;
//   auto freq = 100_hz;      // Frequency(100.0)
//   auto dur  = 5_ms;        // std::chrono::microseconds(5000)
//   auto us   = 200_us;      // std::chrono::microseconds(200)
#pragma once

#include <chrono>
#include <cstdint>

namespace horus {

/// Frequency in Hertz.
class Frequency {
public:
    constexpr explicit Frequency(double hz) : hz_(hz) {}

    /// Get the frequency value in Hz.
    constexpr double value() const { return hz_; }

    /// Get the period (1/freq) as microseconds.
    constexpr std::chrono::microseconds period() const {
        return std::chrono::microseconds(
            static_cast<int64_t>(1'000'000.0 / hz_));
    }

    /// Default budget (80% of period).
    constexpr std::chrono::microseconds budget_default() const {
        return std::chrono::microseconds(
            static_cast<int64_t>(800'000.0 / hz_));
    }

    /// Default deadline (95% of period).
    constexpr std::chrono::microseconds deadline_default() const {
        return std::chrono::microseconds(
            static_cast<int64_t>(950'000.0 / hz_));
    }

private:
    double hz_;
};

/// Duration type alias for convenience.
using Duration = std::chrono::microseconds;

namespace literals {

/// Frequency literal: `100_hz`
constexpr Frequency operator""_hz(unsigned long long hz) {
    return Frequency(static_cast<double>(hz));
}

/// Milliseconds literal: `10_ms`
constexpr std::chrono::microseconds operator""_ms(unsigned long long ms) {
    return std::chrono::microseconds(ms * 1000);
}

/// Microseconds literal: `200_us`
constexpr std::chrono::microseconds operator""_us(unsigned long long us) {
    return std::chrono::microseconds(us);
}

/// Nanoseconds literal: `500_ns`
constexpr std::chrono::nanoseconds operator""_ns(unsigned long long ns) {
    return std::chrono::nanoseconds(ns);
}

/// Seconds literal: `5_s`
constexpr std::chrono::microseconds operator""_s(unsigned long long s) {
    return std::chrono::microseconds(s * 1'000'000);
}

} // namespace literals
} // namespace horus
