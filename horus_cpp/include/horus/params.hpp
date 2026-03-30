// RuntimeParams — dynamic configuration for nodes.
//
// Usage:
//   auto params = horus::Params();
//   params.set("max_speed", 1.5);
//   params.set("enabled", true);
//   params.set("name", "robot1");
//   double speed = params.get<double>("max_speed", 0.0);
#pragma once

#include "horus_c.h"
#include <cstdint>
#include <string>
#include <optional>
#include <utility>

namespace horus {

class Params {
public:
    Params() : handle_(horus_params_new()) {}
    ~Params() { if (handle_) horus_params_destroy(handle_); }

    // Move only
    Params(Params&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    Params& operator=(Params&& o) noexcept {
        if (this != &o) { if (handle_) horus_params_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    Params(const Params&) = delete;
    Params& operator=(const Params&) = delete;

    // ── Typed getters ──────────────────────────────────────────────────

    std::optional<double> get_f64(const char* key) const {
        double v;
        return horus_params_get_f64(handle_, key, &v) ? std::optional<double>(v) : std::nullopt;
    }

    std::optional<int64_t> get_i64(const char* key) const {
        int64_t v;
        return horus_params_get_i64(handle_, key, &v) ? std::optional<int64_t>(v) : std::nullopt;
    }

    std::optional<bool> get_bool(const char* key) const {
        bool v;
        return horus_params_get_bool(handle_, key, &v) ? std::optional<bool>(v) : std::nullopt;
    }

    std::optional<std::string> get_string(const char* key) const {
        char buf[1024];
        int len = horus_params_get_string(handle_, key, reinterpret_cast<char*>(buf), sizeof(buf));
        if (len < 0) return std::nullopt;
        return std::string(buf, static_cast<size_t>(len));
    }

    // ── Typed setters ──────────────────────────────────────────────────

    bool set(const char* key, double value)      { return horus_params_set_f64(handle_, key, value) == 0; }
    bool set(const char* key, int64_t value)     { return horus_params_set_i64(handle_, key, value) == 0; }
    bool set(const char* key, int value)         { return horus_params_set_i64(handle_, key, static_cast<int64_t>(value)) == 0; }
    bool set(const char* key, bool value)        { return horus_params_set_bool(handle_, key, value) == 0; }
    bool set(const char* key, const char* value) { return horus_params_set_string(handle_, key, value) == 0; }
    bool set(const char* key, const std::string& value) { return horus_params_set_string(handle_, key, value.c_str()) == 0; }

    // ── Convenience get with default ───────────────────────────────────

    template<typename T> T get(const char* key, T default_val) const;

    bool has(const char* key) const { return horus_params_has(handle_, key); }

private:
    HorusParams* handle_;
};

// Template specializations for get<T>
template<> inline double Params::get<double>(const char* key, double def) const {
    return get_f64(key).value_or(def);
}
template<> inline int64_t Params::get<int64_t>(const char* key, int64_t def) const {
    return get_i64(key).value_or(def);
}
template<> inline int Params::get<int>(const char* key, int def) const {
    return static_cast<int>(get_i64(key).value_or(def));
}
template<> inline bool Params::get<bool>(const char* key, bool def) const {
    return get_bool(key).value_or(def);
}
template<> inline std::string Params::get<std::string>(const char* key, std::string def) const {
    return get_string(key).value_or(std::move(def));
}

} // namespace horus
