// HORUS Topic Implementation — CmdVel specialization via C API.
//
// Each message type gets a template specialization of Publisher/Subscriber
// that calls the type-specific C API functions.
#pragma once

#include "../horus_c.h"
#include "../topic.hpp"
#include "../msg/control.hpp"

#include <string>
#include <optional>

namespace horus {

// ─── CmdVel Publisher Specialization ─────────────────────────────────────────

template<>
class Publisher<msg::CmdVel> {
public:
    explicit Publisher(std::string_view topic_name) {
        std::string s(topic_name);
        inner_ = horus_publisher_cmd_vel_new(s.c_str());
        name_ = std::string(topic_name);
    }

    ~Publisher() {
        if (inner_) horus_publisher_cmd_vel_destroy(inner_);
    }

    // Move-only
    Publisher(Publisher&& other) noexcept : inner_(other.inner_), name_(std::move(other.name_)) {
        other.inner_ = nullptr;
    }
    Publisher& operator=(Publisher&& other) noexcept {
        if (this != &other) {
            if (inner_) horus_publisher_cmd_vel_destroy(inner_);
            inner_ = other.inner_;
            name_ = std::move(other.name_);
            other.inner_ = nullptr;
        }
        return *this;
    }
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

    /// Loan a sample for zero-copy writing.
    [[nodiscard]] LoanedSample<msg::CmdVel> loan() {
        return LoanedSample<msg::CmdVel>();
    }

    /// Publish a loaned sample.
    void publish(LoanedSample<msg::CmdVel>&& sample) {
        send(*sample);
    }

    /// Send by copy.
    void send(const msg::CmdVel& msg) {
        if (!inner_) return;
        HorusCmdVel c_msg;
        c_msg.timestamp_ns = msg.timestamp_ns;
        c_msg.linear = msg.linear;
        c_msg.angular = msg.angular;
        horus_publisher_cmd_vel_send(inner_, &c_msg);
    }

    const std::string& name() const { return name_; }
    bool is_valid() const { return inner_ != nullptr; }

private:
    HorusPublisher* inner_ = nullptr;
    std::string name_;
};

// ─── CmdVel Subscriber Specialization ────────────────────────────────────────

template<>
class Subscriber<msg::CmdVel> {
public:
    explicit Subscriber(std::string_view topic_name) {
        std::string s(topic_name);
        inner_ = horus_subscriber_cmd_vel_new(s.c_str());
        name_ = std::string(topic_name);
    }

    ~Subscriber() {
        if (inner_) horus_subscriber_cmd_vel_destroy(inner_);
    }

    // Move-only
    Subscriber(Subscriber&& other) noexcept : inner_(other.inner_), name_(std::move(other.name_)) {
        other.inner_ = nullptr;
    }
    Subscriber& operator=(Subscriber&& other) noexcept {
        if (this != &other) {
            if (inner_) horus_subscriber_cmd_vel_destroy(inner_);
            inner_ = other.inner_;
            name_ = std::move(other.name_);
            other.inner_ = nullptr;
        }
        return *this;
    }
    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;

    /// Receive next message.
    std::optional<BorrowedSample<msg::CmdVel>> recv() {
        if (!inner_) return std::nullopt;
        HorusCmdVel c_msg;
        if (horus_subscriber_cmd_vel_recv(inner_, &c_msg)) {
            msg::CmdVel msg;
            msg.timestamp_ns = c_msg.timestamp_ns;
            msg.linear = c_msg.linear;
            msg.angular = c_msg.angular;
            return BorrowedSample<msg::CmdVel>(msg);
        }
        return std::nullopt;
    }

    /// Check if message available.
    bool has_msg() const {
        if (!inner_) return false;
        return horus_subscriber_cmd_vel_has_msg(inner_);
    }

    const std::string& name() const { return name_; }
    bool is_valid() const { return inner_ != nullptr; }

private:
    HorusSubscriber* inner_ = nullptr;
    std::string name_;
};

} // namespace horus
