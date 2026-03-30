// HORUS Node — first-class node with built-in pub/sub, lifecycle, and state.
//
// Matches Rust `impl Node` and Python `horus.Node()` parity.
//
// Usage (struct-based — like Rust):
//
//   class Controller : public horus::Node {
//   public:
//       Controller() : Node("controller") {
//           lidar_ = subscribe<msg::CmdVel>("lidar.scan");
//           cmd_   = advertise<msg::CmdVel>("motor.cmd");
//       }
//
//       void tick() override {
//           auto scan = lidar_->recv();
//           if (!scan) return;
//           auto cmd = cmd_->loan();
//           cmd->linear = scan->get()->linear > 0.5f ? 0.3f : 0.0f;
//           cmd_->publish(std::move(cmd));
//       }
//
//       void init() override { printf("Controller ready\n"); }
//       void enter_safe_state() override { /* stop motors */ }
//
//   private:
//       Subscriber<msg::CmdVel>* lidar_;
//       Publisher<msg::CmdVel>*  cmd_;
//   };
//
// Usage (lambda-based — like Python):
//
//   auto node = horus::LambdaNode("controller")
//       .sub<msg::CmdVel>("lidar.scan")
//       .pub<msg::CmdVel>("motor.cmd")
//       .on_tick([](horus::LambdaNode& self) {
//           auto scan = self.recv<msg::CmdVel>("lidar.scan");
//           if (!scan) return;
//           self.send("motor.cmd", msg::CmdVel{0, 0.3f, 0.0f});
//       });
#pragma once

#include "horus_c.h"
#include "topic.hpp"
#include "msg/control.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace horus {

// ═══════════════════════════════════════════════════════════════════════════
// Node base class — struct-based, like Rust's `impl Node for T`
// ═══════════════════════════════════════════════════════════════════════════

class Node {
public:
    explicit Node(std::string_view name) : name_(name) {}
    virtual ~Node() = default;

    // ── Lifecycle (override in subclass) ────────────────────────────
    virtual void tick() = 0;
    virtual void init() {}
    virtual void enter_safe_state() {}
    virtual void on_shutdown() {}

    // ── Identity ────────────────────────────────────────────────────
    const std::string& name() const { return name_; }

    // ── Pub/Sub creation (call in constructor) ──────────────────────

    template<typename T>
    Publisher<T>* advertise(std::string_view topic) {
        auto p = std::make_shared<Publisher<T>>(topic);
        auto raw = p.get();
        owned_.push_back(p);  // prevent destruction
        pub_names_.push_back(std::string(topic));
        return raw;
    }

    template<typename T>
    Subscriber<T>* subscribe(std::string_view topic) {
        auto s = std::make_shared<Subscriber<T>>(topic);
        auto raw = s.get();
        owned_.push_back(s);  // prevent destruction
        sub_names_.push_back(std::string(topic));
        return raw;
    }

    // ── Introspection ───────────────────────────────────────────────
    const std::vector<std::string>& publishers() const { return pub_names_; }
    const std::vector<std::string>& subscriptions() const { return sub_names_; }

private:
    std::string name_;
    std::vector<std::shared_ptr<void>> owned_;  // type-erased ownership
    std::vector<std::string> pub_names_;
    std::vector<std::string> sub_names_;
};

// ═══════════════════════════════════════════════════════════════════════════
// LambdaNode — declarative node like Python's horus.Node()
// ═══════════════════════════════════════════════════════════════════════════

class LambdaNode {
public:
    explicit LambdaNode(std::string_view name) : name_(name) {}

    // ── Builder pattern ─────────────────────────────────────────────

    template<typename T>
    LambdaNode& pub(std::string_view topic) {
        auto p = std::make_shared<Publisher<T>>(topic);
        typed_pubs_[std::string(topic)] = p;
        pub_names_.push_back(std::string(topic));
        return *this;
    }

    template<typename T>
    LambdaNode& sub(std::string_view topic) {
        auto s = std::make_shared<Subscriber<T>>(topic);
        typed_subs_[std::string(topic)] = s;
        sub_names_.push_back(std::string(topic));
        return *this;
    }

    LambdaNode& on_tick(std::function<void(LambdaNode&)> fn) {
        tick_fn_ = std::move(fn);
        return *this;
    }

    LambdaNode& on_init(std::function<void(LambdaNode&)> fn) {
        init_fn_ = std::move(fn);
        return *this;
    }

    // ── Runtime API (call from tick) ────────────────────────────────

    /// Send a message by copy to a named topic.
    template<typename T>
    void send(const std::string& topic, const T& msg) {
        auto it = typed_pubs_.find(topic);
        if (it != typed_pubs_.end()) {
            auto pub = std::static_pointer_cast<Publisher<T>>(it->second);
            pub->send(msg);
        }
    }

    /// Receive from a named topic.
    template<typename T>
    std::optional<BorrowedSample<T>> recv(const std::string& topic) {
        auto it = typed_subs_.find(topic);
        if (it != typed_subs_.end()) {
            auto sub = std::static_pointer_cast<Subscriber<T>>(it->second);
            return sub->recv();
        }
        return std::nullopt;
    }

    /// Check if message available on topic.
    template<typename T>
    bool has_msg(const std::string& topic) {
        auto it = typed_subs_.find(topic);
        if (it != typed_subs_.end()) {
            auto sub = std::static_pointer_cast<Subscriber<T>>(it->second);
            return sub->has_msg();
        }
        return false;
    }

    // ── Identity ────────────────────────────────────────────────────
    const std::string& name() const { return name_; }
    const std::vector<std::string>& publishers() const { return pub_names_; }
    const std::vector<std::string>& subscriptions() const { return sub_names_; }

    // ── Internal (used by Scheduler) ────────────────────────────────
    void call_tick() { if (tick_fn_) tick_fn_(*this); }
    void call_init() { if (init_fn_) init_fn_(*this); }

private:
    std::string name_;
    std::function<void(LambdaNode&)> tick_fn_;
    std::function<void(LambdaNode&)> init_fn_;

    // Type-erased pub/sub storage (shared_ptr<void> with known cast target)
    std::unordered_map<std::string, std::shared_ptr<void>> typed_pubs_;
    std::unordered_map<std::string, std::shared_ptr<void>> typed_subs_;
    std::vector<std::string> pub_names_;
    std::vector<std::string> sub_names_;
};

} // namespace horus
