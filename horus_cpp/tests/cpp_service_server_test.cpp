// End-to-end C++ service test: a server driven by process() answers a client
// call. Build (headless): link libhorus_cpp.a with -lpthread -ldl. Returns 0 on
// success.
#include <horus/service.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <unistd.h>

// Echo handler: response = request.
static bool echo(const uint8_t* req, size_t req_len, uint8_t* res, size_t* res_len) {
    std::memcpy(res, req, req_len);
    *res_len = req_len;
    return true;
}

int main() {
    const std::string name = "cpp_e2e_svc." + std::to_string(::getpid());
    horus::ServiceServer server(name);
    server.set_handler(echo);
    horus::ServiceClient client(name);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Run the blocking call on a thread; drive the server from main.
    std::atomic<bool> done{false};
    std::string response;
    bool ok = false;
    std::thread caller([&] {
        auto r = client.call(R"({"ping":7})", std::chrono::seconds(3));
        if (r) { response = *r; ok = true; }
        done.store(true);
    });

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (!done.load()) {
        server.process();
        if (std::chrono::steady_clock::now() > deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    caller.join();

    if (!ok) { std::fprintf(stderr, "FAIL: service call not answered\n"); return 1; }
    if (response.find("ping") == std::string::npos) { std::fprintf(stderr, "FAIL: response did not echo request: %s\n", response.c_str()); return 1; }
    std::printf("OK: C++ service server answers a client call (%s)\n", response.c_str());
    return 0;
}
