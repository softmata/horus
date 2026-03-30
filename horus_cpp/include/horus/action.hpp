// Action client/server — long-running tasks with progress feedback.
//
// Usage:
//   auto client = horus::ActionClient("navigate");
//   auto goal = client.send_goal(R"({"x": 5.0, "y": 3.0})");
//   while (goal.is_active()) { /* wait for result */ }
#pragma once

#include "horus_c.h"
#include <cstdint>
#include <string>
#include <utility>

namespace horus {

// ── GoalStatus ─────────────────────────────────────────────────────────────

enum class GoalStatus : uint8_t {
    Pending   = HORUS_GOAL_PENDING,
    Active    = HORUS_GOAL_ACTIVE,
    Succeeded = HORUS_GOAL_SUCCEEDED,
    Aborted   = HORUS_GOAL_ABORTED,
    Canceled  = HORUS_GOAL_CANCELED,
    Rejected  = HORUS_GOAL_REJECTED,
};

// ── GoalHandle ─────────────────────────────────────────────────────────────

class GoalHandle {
public:
    ~GoalHandle() { if (handle_) horus_goal_handle_destroy(handle_); }

    // Move only
    GoalHandle(GoalHandle&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    GoalHandle& operator=(GoalHandle&& o) noexcept {
        if (this != &o) { if (handle_) horus_goal_handle_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    GoalHandle(const GoalHandle&) = delete;
    GoalHandle& operator=(const GoalHandle&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    GoalStatus status() const { return handle_ ? static_cast<GoalStatus>(horus_goal_handle_status(handle_)) : GoalStatus::Rejected; }
    uint64_t id() const { return handle_ ? horus_goal_handle_id(handle_) : 0; }
    bool is_active() const { return handle_ ? horus_goal_handle_is_active(handle_) : false; }
    void cancel() { if (handle_) horus_action_client_cancel(handle_); }

private:
    friend class ActionClient;
    explicit GoalHandle(HorusGoalHandle* h) : handle_(h) {}
    HorusGoalHandle* handle_;
};

// ── ActionClient ───────────────────────────────────────────────────────────

class ActionClient {
public:
    explicit ActionClient(const char* name)
        : handle_(horus_action_client_new(name)) {}

    explicit ActionClient(const std::string& name)
        : ActionClient(name.c_str()) {}

    ~ActionClient() { if (handle_) horus_action_client_destroy(handle_); }

    // Move only
    ActionClient(ActionClient&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    ActionClient& operator=(ActionClient&& o) noexcept {
        if (this != &o) { if (handle_) horus_action_client_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    ActionClient(const ActionClient&) = delete;
    ActionClient& operator=(const ActionClient&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    GoalHandle send_goal(const char* goal_json) {
        return GoalHandle(horus_action_client_send_goal(handle_, goal_json));
    }

    GoalHandle send_goal(const std::string& goal_json) {
        return send_goal(goal_json.c_str());
    }

private:
    HorusActionClient* handle_;
};

// ── ActionServer ───────────────────────────────────────────────────────────

class ActionServer {
public:
    explicit ActionServer(const char* name)
        : handle_(horus_action_server_new(name)) {}

    explicit ActionServer(const std::string& name)
        : ActionServer(name.c_str()) {}

    ~ActionServer() { if (handle_) horus_action_server_destroy(handle_); }

    // Move only
    ActionServer(ActionServer&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    ActionServer& operator=(ActionServer&& o) noexcept {
        if (this != &o) { if (handle_) horus_action_server_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    ActionServer(const ActionServer&) = delete;
    ActionServer& operator=(const ActionServer&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    using AcceptHandler  = uint8_t(*)(const uint8_t*, size_t);
    using ExecuteHandler = void(*)(uint64_t, const uint8_t*, size_t);

    void set_accept_handler(AcceptHandler handler) {
        if (handle_) horus_action_server_set_accept_handler(handle_, handler);
    }

    void set_execute_handler(ExecuteHandler handler) {
        if (handle_) horus_action_server_set_execute_handler(handle_, handler);
    }

    bool is_ready() const {
        return handle_ ? horus_action_server_is_ready(handle_) : false;
    }

private:
    HorusActionServer* handle_;
};

} // namespace horus
