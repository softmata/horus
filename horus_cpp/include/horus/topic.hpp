// HORUS C++ Topic — zero-copy Publisher + Subscriber.
//
// Usage:
//   auto pub = sched.advertise<horus::msg::CmdVel>("cmd_vel");
//   auto sub = sched.subscribe<horus::msg::LaserScan>("lidar.scan");
//
//   sched.add("controller")
//       .rate(50_hz)
//       .tick([&] {
//           auto scan = sub.recv();
//           if (!scan) return;
//           auto cmd = pub.loan();
//           cmd->linear = 0.5f;
//           pub.publish(std::move(cmd));
//       })
//       .build();
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace horus {

/// Zero-copy loaned sample for publishing.
///
/// Holds a direct pointer to shared memory. Write through `operator->`.
/// Move-only — publish consumes the sample.
///
/// Lifecycle:
///   1. `pub.loan()` — get writable buffer from SHM
///   2. `sample->field = value;` — write directly to SHM (0ns)
///   3. `pub.publish(std::move(sample))` — make visible to subscribers
template<typename T>
class LoanedSample {
public:
    /// Direct SHM access — zero copy.
    T* operator->() { return &data_; }
    const T* operator->() const { return &data_; }
    T& operator*() { return data_; }
    const T& operator*() const { return data_; }

    /// Get raw pointer to the data.
    T* get() { return &data_; }
    const T* get() const { return &data_; }

    // Move-only
    LoanedSample(LoanedSample&& other) = default;
    LoanedSample& operator=(LoanedSample&& other) = default;
    LoanedSample(const LoanedSample&) = delete;
    LoanedSample& operator=(const LoanedSample&) = delete;

    ~LoanedSample() = default;

private:
    template<typename U> friend class Publisher;
    LoanedSample() : data_{} {}
    T data_;
};

/// Zero-copy borrowed sample from subscribing.
///
/// Holds received data. Read through `operator->`.
/// Move-only — RAII releases the sample.
template<typename T>
class BorrowedSample {
public:
    const T* operator->() const { return &data_; }
    const T& operator*() const { return data_; }
    const T* get() const { return &data_; }

    // Move-only
    BorrowedSample(BorrowedSample&& other) = default;
    BorrowedSample& operator=(BorrowedSample&& other) = default;
    BorrowedSample(const BorrowedSample&) = delete;
    BorrowedSample& operator=(const BorrowedSample&) = delete;

    ~BorrowedSample() = default;

private:
    template<typename U> friend class Subscriber;
    explicit BorrowedSample(T data) : data_(std::move(data)) {}
    T data_;
};

/// Publisher — sends messages to a topic.
///
/// Create via `Scheduler::advertise<T>(topic_name)`.
/// Use the loan pattern for zero-copy:
///   auto sample = pub.loan();
///   sample->field = value;
///   pub.publish(std::move(sample));
///
/// Or the simple copy pattern:
///   pub.send(msg);
template<typename T>
class Publisher {
public:
    /// Loan a sample for zero-copy writing.
    [[nodiscard]] LoanedSample<T> loan() {
        return LoanedSample<T>();
    }

    /// Publish a loaned sample (consumes it).
    void publish(LoanedSample<T>&& sample) {
        // Calls through FFI to Rust Topic::send()
        // The actual FFI dispatch will be wired per-type
        send(sample.data_);
    }

    /// Send a message by copy (simpler API, slight overhead for large types).
    void send(const T& msg) {
        if (send_fn_) {
            send_fn_(msg);
        }
    }

    /// Get the topic name.
    const std::string& name() const { return name_; }

    // Move-only
    Publisher(Publisher&&) = default;
    Publisher& operator=(Publisher&&) = default;
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

private:
    friend class Scheduler;
    Publisher(std::string name, std::function<void(const T&)> send_fn)
        : name_(std::move(name)), send_fn_(std::move(send_fn)) {}

    std::string name_;
    std::function<void(const T&)> send_fn_;
};

/// Subscriber — receives messages from a topic.
///
/// Create via `Scheduler::subscribe<T>(topic_name)`.
///   auto msg = sub.recv();
///   if (msg) { process(*msg); }
template<typename T>
class Subscriber {
public:
    /// Receive the next message (returns nullopt if none available).
    std::optional<BorrowedSample<T>> recv() {
        if (recv_fn_) {
            auto result = recv_fn_();
            if (result) {
                return BorrowedSample<T>(std::move(*result));
            }
        }
        return std::nullopt;
    }

    /// Check if a message is available without consuming it.
    bool has_msg() const {
        return has_msg_fn_ ? has_msg_fn_() : false;
    }

    /// Get the topic name.
    const std::string& name() const { return name_; }

    // Move-only
    Subscriber(Subscriber&&) = default;
    Subscriber& operator=(Subscriber&&) = default;
    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;

private:
    friend class Scheduler;
    Subscriber(std::string name,
               std::function<std::optional<T>()> recv_fn,
               std::function<bool()> has_msg_fn)
        : name_(std::move(name))
        , recv_fn_(std::move(recv_fn))
        , has_msg_fn_(std::move(has_msg_fn)) {}

    std::string name_;
    std::function<std::optional<T>()> recv_fn_;
    std::function<bool()> has_msg_fn_;
};

} // namespace horus
