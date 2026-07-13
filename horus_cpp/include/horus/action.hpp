// Action client/server — long-running tasks with progress feedback + cancel.
//
// Server usage:
//   void execute(HorusActionGoalHandle* h, const uint8_t* goal, size_t len) {
//       horus::ActionGoalHandle g(h);
//       while (!g.is_cancel_requested()) {
//           g.publish_feedback(R"({"progress":0.5})");
//           /* ... do work ... */
//       }
//       g.succeed(R"({"ok":true})");
//   }
//   auto server = horus::ActionServer("navigate");
//   server.set_accept_handler(accept);
//   server.set_execute_handler(execute);
//   while (running) { server.process(); /* sleep a little */ }
//
// Client usage:
//   auto client = horus::ActionClient("navigate");
//   auto goal = client.send_goal(R"({"x":5.0,"y":3.0})");
//   // ... optionally client.poll_feedback(...) ...
//   client.cancel(goal.id());  // request cancellation
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

// ── ActionGoalHandle (server side) ─────────────────────────────────────────
//
// Non-owning wrapper around the handle passed to the execute handler. The
// underlying handle lives on the goal thread's stack and is valid ONLY for the
// duration of the execute call — do not stash it and call succeed()/
// publish_feedback() afterward (use-after-free). Same contract as ROS.

class ActionGoalHandle {
public:
    explicit ActionGoalHandle(HorusActionGoalHandle* h) : handle_(h) {}

    uint64_t id() const { return handle_ ? horus_action_goal_id(handle_) : 0; }

    // True once a cancel has been requested for this goal. Poll this in the
    // execute loop and exit promptly (call cancel()) when it returns true.
    bool is_cancel_requested() const {
        return handle_ ? horus_action_goal_is_cancel_requested(handle_) : false;
    }

    // Publish progress feedback (JSON). Returns false if oversize (>3968 bytes).
    bool publish_feedback(const char* feedback_json) {
        return handle_ ? horus_action_goal_publish_feedback(handle_, feedback_json) : false;
    }
    bool publish_feedback(const std::string& feedback_json) {
        return publish_feedback(feedback_json.c_str());
    }

    // Finish the goal. Exactly one of these takes effect (first call wins); if
    // the handler returns without finishing, the goal completes as Aborted.
    void succeed(const char* result_json)  { if (handle_) horus_action_goal_succeed(handle_, result_json); }
    void abort(const char* result_json)    { if (handle_) horus_action_goal_abort(handle_, result_json); }
    void canceled(const char* result_json) { if (handle_) horus_action_goal_canceled(handle_, result_json); }
    void succeed(const std::string& r)  { succeed(r.c_str()); }
    void abort(const std::string& r)    { abort(r.c_str()); }
    void canceled(const std::string& r) { canceled(r.c_str()); }

private:
    HorusActionGoalHandle* handle_;
};

// ── GoalHandle (client side) ───────────────────────────────────────────────

class GoalHandle {
public:
    ~GoalHandle() { if (handle_) horus_goal_handle_destroy(handle_); }

    // Move only
    GoalHandle(GoalHandle&& o) noexcept : handle_(o.handle_), client_(o.client_) { o.handle_ = nullptr; }
    GoalHandle& operator=(GoalHandle&& o) noexcept {
        if (this != &o) { if (handle_) horus_goal_handle_destroy(handle_); handle_ = o.handle_; client_ = o.client_; o.handle_ = nullptr; }
        return *this;
    }
    GoalHandle(const GoalHandle&) = delete;
    GoalHandle& operator=(const GoalHandle&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    GoalStatus status() const { return handle_ ? static_cast<GoalStatus>(horus_goal_handle_status(handle_)) : GoalStatus::Rejected; }
    uint64_t id() const { return handle_ ? horus_goal_handle_id(handle_) : 0; }
    bool is_active() const { return handle_ ? horus_goal_handle_is_active(handle_) : false; }

    // Request cancellation over the cancel topic (needs the owning client alive).
    void cancel() { if (client_) horus_action_client_cancel(client_, id()); }

private:
    friend class ActionClient;
    GoalHandle(HorusGoalHandle* h, const HorusActionClient* c) : handle_(h), client_(c) {}
    HorusGoalHandle* handle_;
    const HorusActionClient* client_;
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
        return GoalHandle(horus_action_client_send_goal(handle_, goal_json), handle_);
    }
    GoalHandle send_goal(const std::string& goal_json) {
        return send_goal(goal_json.c_str());
    }

    // Request cancellation of a goal by id.
    void cancel(uint64_t goal_id) {
        if (handle_) horus_action_client_cancel(handle_, goal_id);
    }

    // Non-blocking. Returns true and fills out_* if feedback is available.
    bool poll_feedback(uint64_t& out_goal_id, std::string& out_json) {
        if (!handle_) return false;
        char buf[4096];
        int32_t n = horus_action_client_poll_feedback(
            handle_, &out_goal_id, reinterpret_cast<uint8_t*>(buf), sizeof(buf));
        if (n < 0) return false;
        out_json.assign(buf, static_cast<size_t>(n));
        return true;
    }

    // Non-blocking. Returns true once a result for goal_id is available.
    bool poll_result(uint64_t goal_id, GoalStatus& out_status, std::string& out_json) {
        if (!handle_) return false;
        uint8_t status = 0;
        char buf[4096];
        int32_t n = horus_action_client_poll_result(
            handle_, goal_id, &status, reinterpret_cast<uint8_t*>(buf), sizeof(buf));
        if (n < 0) return false;
        out_status = static_cast<GoalStatus>(status);
        out_json.assign(buf, static_cast<size_t>(n));
        return true;
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
    // Runs on a dedicated thread per goal. Wrap the raw handle in ActionGoalHandle.
    using ExecuteHandler = void(*)(HorusActionGoalHandle*, const uint8_t*, size_t);

    void set_accept_handler(AcceptHandler handler) {
        if (handle_) horus_action_server_set_accept_handler(handle_, handler);
    }

    void set_execute_handler(ExecuteHandler handler) {
        if (handle_) horus_action_server_set_execute_handler(handle_, handler);
    }

    bool is_ready() const {
        return handle_ ? horus_action_server_is_ready(handle_) : false;
    }

    // Drive the server once: publish feedback/results, accept/dispatch new goals,
    // and deliver cancellations. Call repeatedly from the SAME thread that
    // constructed the server.
    void process() {
        if (handle_) horus_action_server_process(handle_);
    }

private:
    HorusActionServer* handle_;
};

} // namespace horus
