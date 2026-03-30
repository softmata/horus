// Comprehensive C++ API test — exercises ALL C API functions from C++.
// Tests: TensorPool, Image, PointCloud, Params, TransformFrame, Service, Action.
//
// Compile: g++ -std=c++17 -I horus_cpp/include -o cpp_full_api_test tests/cpp_full_api_test.cpp \
//          -L target/debug -lhorus_cpp -lpthread -ldl -lm
// Run:     LD_LIBRARY_PATH=target/debug ./cpp_full_api_test

#include <horus/horus_c.h>
#include <cstdio>
#include <cstring>
#include <cmath>

static int pass_count = 0;
static int fail_count = 0;

#define CHECK(cond, name) do { \
    if (cond) { std::printf("[PASS] %s\n", name); pass_count++; } \
    else { std::printf("[FAIL] %s\n", name); fail_count++; } \
} while(0)

#define CHECK_EQ(a, b, name) CHECK((a) == (b), name)
#define CHECK_NEAR(a, b, eps, name) CHECK(std::fabs((a) - (b)) < (eps), name)

// ── TensorPool ──────────────────────────────────────────────────────────────

void test_tensor_pool() {
    std::printf("\n=== TensorPool ===\n");

    HorusTensorPool* pool = horus_tensor_pool_new(9990, 4 * 1024 * 1024, 64);
    CHECK(pool != nullptr, "pool created");

    size_t alloc = 0, used = 0, free_bytes = 0;
    int rc = horus_tensor_pool_stats(pool, &alloc, &used, &free_bytes);
    CHECK_EQ(rc, 0, "stats returns 0");
    CHECK_EQ(alloc, (size_t)0, "0 allocated initially");
    CHECK(free_bytes > 0, "free bytes > 0");

    // Allocate tensor [10, 20, 3] u8
    uint64_t shape[] = {10, 20, 3};
    HorusTensor* tensor = horus_tensor_alloc(pool, shape, 3, HORUS_DTYPE_U8);
    CHECK(tensor != nullptr, "tensor allocated");
    CHECK_EQ(horus_tensor_nbytes(tensor), (uint64_t)(10 * 20 * 3), "tensor nbytes = 600");

    uint8_t* ptr = horus_tensor_data_ptr(pool, tensor);
    CHECK(ptr != nullptr, "tensor data_ptr non-null");

    // Write and read data
    ptr[0] = 42;
    ptr[599] = 99;
    CHECK_EQ(ptr[0], (uint8_t)42, "data write/read [0]");
    CHECK_EQ(ptr[599], (uint8_t)99, "data write/read [599]");

    horus_tensor_release(pool, tensor);
    horus_tensor_destroy(tensor);
    horus_tensor_pool_destroy(pool);
    CHECK(true, "pool destroyed without crash");
}

// ── Image ───────────────────────────────────────────────────────────────────

void test_image() {
    std::printf("\n=== Image ===\n");

    HorusTensorPool* pool = horus_tensor_pool_new(9991, 4 * 1024 * 1024, 64);
    CHECK(pool != nullptr, "pool for image");

    HorusImage* img = horus_image_new(pool, 640, 480, HORUS_ENC_RGB8);
    CHECK(img != nullptr, "image created");
    CHECK_EQ(horus_image_width(img), (uint32_t)640, "width = 640");
    CHECK_EQ(horus_image_height(img), (uint32_t)480, "height = 480");
    CHECK_EQ(horus_image_data_size(img), (size_t)(640 * 480 * 3), "data_size = 921600");

    // RGBA8
    HorusImage* rgba = horus_image_new(pool, 320, 240, HORUS_ENC_RGBA8);
    CHECK(rgba != nullptr, "RGBA8 image created");
    CHECK_EQ(horus_image_data_size(rgba), (size_t)(320 * 240 * 4), "RGBA data_size");

    // GRAY8
    HorusImage* gray = horus_image_new(pool, 100, 100, HORUS_ENC_GRAY8);
    CHECK(gray != nullptr, "GRAY8 image created");
    CHECK_EQ(horus_image_data_size(gray), (size_t)(100 * 100 * 1), "GRAY data_size");

    horus_image_destroy(img);
    horus_image_destroy(rgba);
    horus_image_destroy(gray);
    horus_tensor_pool_destroy(pool);
}

// ── PointCloud ──────────────────────────────────────────────────────────────

void test_pointcloud() {
    std::printf("\n=== PointCloud ===\n");

    HorusTensorPool* pool = horus_tensor_pool_new(9992, 4 * 1024 * 1024, 64);

    // XYZ points
    HorusPointCloud* pc = horus_pointcloud_new(pool, 1000, 3);
    CHECK(pc != nullptr, "pointcloud XYZ created");
    CHECK_EQ(horus_pointcloud_num_points(pc), (uint64_t)1000, "1000 points");
    CHECK_EQ(horus_pointcloud_fields(pc), (uint32_t)3, "3 fields (XYZ)");

    // XYZI points
    HorusPointCloud* pc4 = horus_pointcloud_new(pool, 500, 4);
    CHECK(pc4 != nullptr, "pointcloud XYZI created");
    CHECK_EQ(horus_pointcloud_fields(pc4), (uint32_t)4, "4 fields (XYZI)");

    horus_pointcloud_destroy(pc);
    horus_pointcloud_destroy(pc4);
    horus_tensor_pool_destroy(pool);
}

// ── Params ──────────────────────────────────────────────────────────────────

void test_params() {
    std::printf("\n=== Params ===\n");

    HorusParams* p = horus_params_new();
    CHECK(p != nullptr, "params created");

    // f64
    CHECK_EQ(horus_params_set_f64(p, "speed", 1.5), 0, "set f64");
    double fv;
    CHECK_EQ(horus_params_get_f64(p, "speed", &fv), 1, "get f64 found");
    CHECK_NEAR(fv, 1.5, 1e-10, "f64 value = 1.5");

    // i64
    CHECK_EQ(horus_params_set_i64(p, "count", 42), 0, "set i64");
    int64_t iv;
    CHECK_EQ(horus_params_get_i64(p, "count", &iv), 1, "get i64 found");
    CHECK_EQ(iv, (int64_t)42, "i64 value = 42");

    // bool
    CHECK_EQ(horus_params_set_bool(p, "enabled", true), 0, "set bool");
    bool bv;
    CHECK_EQ(horus_params_get_bool(p, "enabled", &bv), 1, "get bool found");
    CHECK(bv == true, "bool value = true");

    // string
    CHECK_EQ(horus_params_set_string(p, "name", "robot1"), 0, "set string");
    char buf[64];
    int len = horus_params_get_string(p, "name", (char*)buf, 64);
    CHECK_EQ(len, 6, "string length = 6");
    CHECK(std::strcmp(buf, "robot1") == 0, "string value = robot1");

    // has / missing
    CHECK(horus_params_has(p, "speed"), "has 'speed'");
    CHECK(!horus_params_has(p, "missing"), "!has 'missing'");

    double missing;
    CHECK_EQ(horus_params_get_f64(p, "nonexistent", &missing), 0, "get missing returns 0");

    horus_params_destroy(p);
}

// ── TransformFrame ──────────────────────────────────────────────────────────

void test_transform() {
    std::printf("\n=== TransformFrame ===\n");

    HorusTransformFrame* tf = horus_transform_frame_new();
    CHECK(tf != nullptr, "tf created");

    // Register frames
    int world_id = horus_transform_frame_register(tf, "world", nullptr);
    CHECK(world_id >= 0, "register world");

    int base_id = horus_transform_frame_register(tf, "base_link", "world");
    CHECK(base_id >= 0, "register base_link");

    int cam_id = horus_transform_frame_register(tf, "camera", "base_link");
    CHECK(cam_id >= 0, "register camera");

    // Update transform: base_link at (1, 2, 3) with identity rotation
    int rc = horus_transform_frame_update(tf, "base_link",
        1.0, 2.0, 3.0,       // translation
        0.0, 0.0, 0.0, 1.0,  // quaternion (identity)
        1000);
    CHECK_EQ(rc, 0, "update base_link");

    // Update camera: (0.1, 0, 0.5) from base_link
    rc = horus_transform_frame_update(tf, "camera",
        0.1, 0.0, 0.5,
        0.0, 0.0, 0.0, 1.0,
        1000);
    CHECK_EQ(rc, 0, "update camera");

    // Lookup base_link → world
    double out[7];
    rc = horus_transform_frame_lookup(tf, "base_link", "world", out);
    CHECK_EQ(rc, 0, "lookup base_link→world");
    CHECK_NEAR(out[0], 1.0, 1e-10, "tx = 1.0");
    CHECK_NEAR(out[1], 2.0, 1e-10, "ty = 2.0");
    CHECK_NEAR(out[2], 3.0, 1e-10, "tz = 3.0");

    // Can transform
    CHECK(horus_transform_frame_can_transform(tf, "base_link", "world"), "can_transform base→world");
    CHECK(horus_transform_frame_can_transform(tf, "camera", "world"), "can_transform camera→world");
    CHECK(!horus_transform_frame_can_transform(tf, "world", "nonexistent"), "!can_transform to nonexistent");

    horus_transform_frame_destroy(tf);
}

// ── Service ─────────────────────────────────────────────────────────────────

void test_service() {
    std::printf("\n=== Service ===\n");

    HorusServiceClient* client = horus_service_client_new("test_full_api.svc");
    CHECK(client != nullptr, "service client created");

    HorusServiceServer* server = horus_service_server_new("test_full_api.svc");
    CHECK(server != nullptr, "service server created");

    horus_service_client_destroy(client);
    horus_service_server_destroy(server);
    CHECK(true, "service destroyed without crash");
}

// ── Action ──────────────────────────────────────────────────────────────────

void test_action() {
    std::printf("\n=== Action ===\n");

    HorusActionClient* client = horus_action_client_new("test_full_api.action");
    CHECK(client != nullptr, "action client created");

    // Send goal
    HorusGoalHandle* goal = horus_action_client_send_goal(client, "{\"x\": 5.0}");
    CHECK(goal != nullptr, "goal handle created");
    CHECK_EQ(horus_goal_handle_id(goal), (uint64_t)1, "goal id = 1");
    CHECK_EQ(horus_goal_handle_status(goal), (uint8_t)HORUS_GOAL_PENDING, "status = Pending");
    CHECK(horus_goal_handle_is_active(goal), "goal is active");

    // Cancel
    horus_action_client_cancel(goal);
    CHECK_EQ(horus_goal_handle_status(goal), (uint8_t)HORUS_GOAL_CANCELED, "status = Canceled");
    CHECK(!horus_goal_handle_is_active(goal), "goal not active after cancel");

    // Server
    HorusActionServer* server = horus_action_server_new("test_full_api.action");
    CHECK(server != nullptr, "action server created");
    CHECK(!horus_action_server_is_ready(server), "server not ready (no handlers)");

    horus_goal_handle_destroy(goal);
    horus_action_client_destroy(client);
    horus_action_server_destroy(server);
    CHECK(true, "action destroyed without crash");
}

// ── Null Safety ─────────────────────────────────────────────────────────────

void test_null_safety() {
    std::printf("\n=== Null Safety ===\n");

    // Pool
    horus_tensor_pool_destroy(nullptr);
    CHECK_EQ(horus_tensor_pool_stats(nullptr, nullptr, nullptr, nullptr), -1, "null pool stats");
    CHECK(horus_tensor_alloc(nullptr, nullptr, 0, 0) == nullptr, "null tensor alloc");
    CHECK(horus_tensor_data_ptr(nullptr, nullptr) == nullptr, "null tensor data_ptr");
    CHECK_EQ(horus_tensor_nbytes(nullptr), (uint64_t)0, "null tensor nbytes");

    // Image
    CHECK(horus_image_new(nullptr, 0, 0, 0) == nullptr, "null image new");
    CHECK_EQ(horus_image_width(nullptr), (uint32_t)0, "null image width");

    // PointCloud
    CHECK(horus_pointcloud_new(nullptr, 0, 0) == nullptr, "null pc new");
    CHECK_EQ(horus_pointcloud_num_points(nullptr), (uint64_t)0, "null pc points");

    // Params
    CHECK(!horus_params_has(nullptr, nullptr), "null params has");

    // Action
    CHECK(horus_action_client_send_goal(nullptr, nullptr) == nullptr, "null goal send");
    CHECK(!horus_goal_handle_is_active(nullptr), "null goal active");
    CHECK(!horus_action_server_is_ready(nullptr), "null server ready");

    // Transform
    CHECK_EQ(horus_transform_frame_register(nullptr, nullptr, nullptr), -1, "null tf register");
    CHECK(!horus_transform_frame_can_transform(nullptr, nullptr, nullptr), "null tf can_transform");

    CHECK(true, "null safety — no crashes");
}

int main() {
    std::printf("=== HORUS C++ Full API Test ===\n");

    test_tensor_pool();
    test_image();
    test_pointcloud();
    test_params();
    test_transform();
    test_service();
    test_action();
    test_null_safety();

    std::printf("\n╔══════════════════════════════════════╗\n");
    std::printf("║  Results: %d passed, %d failed        ║\n", pass_count, fail_count);
    std::printf("╚══════════════════════════════════════╝\n");

    return fail_count > 0 ? 1 : 0;
}
