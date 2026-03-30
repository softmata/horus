// TransformFrame — coordinate frame system (TF tree).
//
// Usage:
//   auto tf = horus::TransformFrame();
//   tf.register_frame("world", "");
//   tf.register_frame("base_link", "world");
//   tf.update("base_link", {1.0, 2.0, 0.0}, {0, 0, 0, 1}, timestamp);
//   auto [pos, rot] = tf.lookup("base_link", "world");
#pragma once

#include "horus_c.h"
#include <array>
#include <cstdint>
#include <optional>
#include <utility>

namespace horus {

struct Transform {
    std::array<double, 3> translation;  // [x, y, z]
    std::array<double, 4> rotation;     // [qx, qy, qz, qw]
};

class TransformFrame {
public:
    TransformFrame() : handle_(horus_transform_frame_new()) {}
    explicit TransformFrame(size_t max_frames)
        : handle_(horus_transform_frame_with_capacity(max_frames)) {}

    ~TransformFrame() { if (handle_) horus_transform_frame_destroy(handle_); }

    // Move only
    TransformFrame(TransformFrame&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    TransformFrame& operator=(TransformFrame&& o) noexcept {
        if (this != &o) { if (handle_) horus_transform_frame_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    TransformFrame(const TransformFrame&) = delete;
    TransformFrame& operator=(const TransformFrame&) = delete;

    /// Register a frame. Pass nullptr or "" for parent to create root frame.
    /// Returns frame ID or -1 on error.
    int register_frame(const char* name, const char* parent = nullptr) {
        return horus_transform_frame_register(handle_, name, parent);
    }

    /// Update a frame's transform.
    bool update(const char* frame, const std::array<double, 3>& pos,
                const std::array<double, 4>& rot, uint64_t timestamp_ns) {
        return horus_transform_frame_update(handle_, frame,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            timestamp_ns) == 0;
    }

    /// Look up transform between frames.
    std::optional<Transform> lookup(const char* source, const char* target) const {
        double out[7];
        if (horus_transform_frame_lookup(handle_, source, target, out) != 0)
            return std::nullopt;
        return Transform{
            {out[0], out[1], out[2]},
            {out[3], out[4], out[5], out[6]}
        };
    }

    /// Check if path exists between frames.
    bool can_transform(const char* source, const char* target) const {
        return horus_transform_frame_can_transform(handle_, source, target);
    }

private:
    HorusTransformFrame* handle_;
};

} // namespace horus
