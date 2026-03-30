// Service client/server — request/response RPC.
//
// Usage:
//   // Client side
//   auto client = horus::ServiceClient("add_two_ints");
//   auto response = client.call(R"({"a": 3, "b": 4})", 1000ms);
//
//   // Server side
//   auto server = horus::ServiceServer("add_two_ints");
//   server.set_handler([](const uint8_t* req, size_t len, uint8_t* res, size_t* res_len) -> bool {
//       // Parse request, compute response, write to res
//       return true;
//   });
#pragma once

#include "horus_c.h"
#include <cstdint>
#include <string>
#include <optional>
#include <chrono>
#include <utility>

namespace horus {

class ServiceClient {
public:
    explicit ServiceClient(const char* name)
        : handle_(horus_service_client_new(name)) {}

    explicit ServiceClient(const std::string& name)
        : ServiceClient(name.c_str()) {}

    ~ServiceClient() { if (handle_) horus_service_client_destroy(handle_); }

    // Move only
    ServiceClient(ServiceClient&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    ServiceClient& operator=(ServiceClient&& o) noexcept {
        if (this != &o) { if (handle_) horus_service_client_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    ServiceClient(const ServiceClient&) = delete;
    ServiceClient& operator=(const ServiceClient&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    /// Call service with JSON request. Returns JSON response or empty on error/timeout.
    std::optional<std::string> call(const char* request_json, std::chrono::microseconds timeout) {
        char buf[4096];
        int len = horus_service_client_call(
            handle_, request_json,
            static_cast<uint64_t>(timeout.count()),
            reinterpret_cast<char*>(buf), sizeof(buf));
        if (len < 0) return std::nullopt;
        return std::string(buf, static_cast<size_t>(len));
    }

    std::optional<std::string> call(const std::string& request_json, std::chrono::microseconds timeout) {
        return call(request_json.c_str(), timeout);
    }

private:
    HorusServiceClient* handle_;
};

class ServiceServer {
public:
    explicit ServiceServer(const char* name)
        : handle_(horus_service_server_new(name)) {}

    explicit ServiceServer(const std::string& name)
        : ServiceServer(name.c_str()) {}

    ~ServiceServer() { if (handle_) horus_service_server_destroy(handle_); }

    // Move only
    ServiceServer(ServiceServer&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    ServiceServer& operator=(ServiceServer&& o) noexcept {
        if (this != &o) { if (handle_) horus_service_server_destroy(handle_); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    ServiceServer(const ServiceServer&) = delete;
    ServiceServer& operator=(const ServiceServer&) = delete;

    explicit operator bool() const { return handle_ != nullptr; }

    using Handler = bool(*)(const uint8_t*, size_t, uint8_t*, size_t*);

    void set_handler(Handler handler) {
        if (handle_) horus_service_server_set_handler(handle_, handler);
    }

private:
    HorusServiceServer* handle_;
};

} // namespace horus
