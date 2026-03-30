/* HORUS C API — callable from C and C++.
 *
 * This header declares all #[no_mangle] extern "C" functions exported by
 * libhorus_cpp. Link against libhorus_cpp.so (or .a) to use.
 *
 * GENERATED from horus_cpp/src/c_api.rs — keep in sync.
 */
#ifndef HORUS_C_H
#define HORUS_C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── ABI Version ─────────────────────────────────────────────────────────── */

#define HORUS_CPP_ABI_VERSION 1

uint32_t horus_get_abi_version(void);

/* ── Opaque Types ────────────────────────────────────────────────────────── */

typedef struct HorusScheduler HorusScheduler;
typedef struct HorusNodeBuilder HorusNodeBuilder;
typedef struct HorusPublisher HorusPublisher;
typedef struct HorusSubscriber HorusSubscriber;

/* ── Scheduler ───────────────────────────────────────────────────────────── */

HorusScheduler* horus_scheduler_new(void);
void            horus_scheduler_destroy(HorusScheduler* sched);
void            horus_scheduler_tick_rate(HorusScheduler* sched, double hz);
void            horus_scheduler_name(HorusScheduler* sched, const char* name);
void            horus_scheduler_prefer_rt(HorusScheduler* sched);
void            horus_scheduler_require_rt(HorusScheduler* sched);
void            horus_scheduler_deterministic(HorusScheduler* sched, bool enabled);
void            horus_scheduler_verbose(HorusScheduler* sched, bool enabled);
void            horus_scheduler_watchdog(HorusScheduler* sched, uint64_t timeout_us);
void            horus_scheduler_blackbox(HorusScheduler* sched, size_t size_mb);
void            horus_scheduler_enable_network(HorusScheduler* sched);
void            horus_scheduler_stop(const HorusScheduler* sched);
bool            horus_scheduler_is_running(const HorusScheduler* sched);
bool            horus_scheduler_has_full_rt(const HorusScheduler* sched);
int             horus_scheduler_get_name(const HorusScheduler* sched, char* buf, size_t buf_len);
int             horus_scheduler_tick_once(HorusScheduler* sched);
uint32_t        horus_scheduler_node_count(const HorusScheduler* sched);
int             horus_scheduler_node_name_at(const HorusScheduler* sched, uint32_t index, char* buf, size_t buf_len);

/* ── Node Builder ────────────────────────────────────────────────────────── */

HorusNodeBuilder* horus_node_builder_new(const char* name);
void              horus_node_builder_rate(HorusNodeBuilder* builder, double hz);
void              horus_node_builder_budget(HorusNodeBuilder* builder, uint64_t budget_us);
void              horus_node_builder_deadline(HorusNodeBuilder* builder, uint64_t deadline_us);
void              horus_node_builder_on_miss(HorusNodeBuilder* builder, uint8_t policy);
void              horus_node_builder_compute(HorusNodeBuilder* builder);
void              horus_node_builder_order(HorusNodeBuilder* builder, uint32_t order);
void              horus_node_builder_set_tick(HorusNodeBuilder* builder, void (*callback)(void));
void              horus_node_builder_async_io(HorusNodeBuilder* builder);
void              horus_node_builder_on_topic(HorusNodeBuilder* builder, const char* topic);
void              horus_node_builder_pin_core(HorusNodeBuilder* builder, size_t cpu_id);
void              horus_node_builder_priority(HorusNodeBuilder* builder, int32_t prio);
void              horus_node_builder_watchdog(HorusNodeBuilder* builder, uint64_t timeout_us);
int               horus_node_builder_build(HorusNodeBuilder* builder, HorusScheduler* sched);

/* ── CmdVel Topic ────────────────────────────────────────────────────────── */

typedef struct {
    uint64_t timestamp_ns;
    float    linear;   /* m/s */
    float    angular;  /* rad/s */
} HorusCmdVel;

HorusPublisher*  horus_publisher_cmd_vel_new(const char* topic_name);
void             horus_publisher_cmd_vel_destroy(HorusPublisher* pub_);
void             horus_publisher_cmd_vel_send(const HorusPublisher* pub_, const HorusCmdVel* msg);

HorusSubscriber* horus_subscriber_cmd_vel_new(const char* topic_name);
void             horus_subscriber_cmd_vel_destroy(HorusSubscriber* sub);
int              horus_subscriber_cmd_vel_recv(const HorusSubscriber* sub, HorusCmdVel* out);
bool             horus_subscriber_cmd_vel_has_msg(const HorusSubscriber* sub);

/* ── JsonWireMessage Topic (for services/actions) ────────────────────────── */

typedef struct {
    uint8_t  data[3968];
    uint32_t data_len;
    uint64_t msg_id;
    uint8_t  msg_type;  /* 0=request, 1=response_ok, 2=response_err, 3=feedback, 4=goal, 5=result */
    uint8_t  _padding[11];
} HorusJsonWireMsg;

HorusPublisher*  horus_publisher_json_wire_new(const char* topic_name);
void             horus_publisher_json_wire_destroy(HorusPublisher* pub_);
void             horus_publisher_json_wire_send(const HorusPublisher* pub_, const HorusJsonWireMsg* msg);

HorusSubscriber* horus_subscriber_json_wire_new(const char* topic_name);
void             horus_subscriber_json_wire_destroy(HorusSubscriber* sub);
int              horus_subscriber_json_wire_recv(const HorusSubscriber* sub, HorusJsonWireMsg* out);

/* ── Miss Policy Constants ───────────────────────────────────────────────── */

#define HORUS_MISS_WARN      0
#define HORUS_MISS_SKIP      1
#define HORUS_MISS_SAFE_MODE 2
#define HORUS_MISS_STOP      3

/* ── TensorPool ─────────────────────────────────────────────────────────── */

typedef struct HorusTensorPool   HorusTensorPool;
typedef struct HorusTensor       HorusTensor;
typedef struct HorusImage        HorusImage;
typedef struct HorusPointCloud   HorusPointCloud;

/* dtype: 0=f32, 1=f64, 2=u8, 3=i32 */
#define HORUS_DTYPE_F32  0
#define HORUS_DTYPE_F64  1
#define HORUS_DTYPE_U8   2
#define HORUS_DTYPE_I32  3

/* encoding: 0=RGB8, 1=RGBA8, 2=GRAY8, 3=BGR8 */
#define HORUS_ENC_RGB8   0
#define HORUS_ENC_RGBA8  1
#define HORUS_ENC_GRAY8  2
#define HORUS_ENC_BGR8   3

HorusTensorPool* horus_tensor_pool_new(uint32_t pool_id, size_t pool_size_bytes, size_t max_slots);
void             horus_tensor_pool_destroy(HorusTensorPool* pool);
int              horus_tensor_pool_stats(const HorusTensorPool* pool, size_t* out_allocated, size_t* out_used, size_t* out_free);

HorusTensor*     horus_tensor_alloc(const HorusTensorPool* pool, const uint64_t* shape, size_t ndim, uint8_t dtype);
uint8_t*         horus_tensor_data_ptr(const HorusTensorPool* pool, const HorusTensor* tensor);
uint64_t         horus_tensor_nbytes(const HorusTensor* tensor);
void             horus_tensor_release(const HorusTensorPool* pool, const HorusTensor* tensor);
void             horus_tensor_destroy(HorusTensor* tensor);

HorusImage*      horus_image_new(const HorusTensorPool* pool, uint32_t width, uint32_t height, uint8_t encoding);
uint32_t         horus_image_width(const HorusImage* img);
uint32_t         horus_image_height(const HorusImage* img);
size_t           horus_image_data_size(const HorusImage* img);
void             horus_image_destroy(HorusImage* img);

HorusPointCloud* horus_pointcloud_new(const HorusTensorPool* pool, uint32_t num_points, uint32_t fields_per_point);
uint64_t         horus_pointcloud_num_points(const HorusPointCloud* pc);
uint32_t         horus_pointcloud_fields(const HorusPointCloud* pc);
void             horus_pointcloud_destroy(HorusPointCloud* pc);

/* ── RuntimeParams ──────────────────────────────────────────────────────── */

typedef struct HorusParams HorusParams;

HorusParams* horus_params_new(void);
void         horus_params_destroy(HorusParams* params);
int          horus_params_get_f64(const HorusParams* params, const char* key, double* out);
int          horus_params_set_f64(const HorusParams* params, const char* key, double value);
int          horus_params_get_i64(const HorusParams* params, const char* key, int64_t* out);
int          horus_params_set_i64(const HorusParams* params, const char* key, int64_t value);
int          horus_params_get_bool(const HorusParams* params, const char* key, bool* out);
int          horus_params_set_bool(const HorusParams* params, const char* key, bool value);
int          horus_params_get_string(const HorusParams* params, const char* key, char* buf, size_t buf_len);
int          horus_params_set_string(const HorusParams* params, const char* key, const char* value);
bool         horus_params_has(const HorusParams* params, const char* key);

/* ── Service ────────────────────────────────────────────────────────────── */

typedef struct HorusServiceClient HorusServiceClient;
typedef struct HorusServiceServer HorusServiceServer;

HorusServiceClient* horus_service_client_new(const char* name);
void                horus_service_client_destroy(HorusServiceClient* client);
int                 horus_service_client_call(const HorusServiceClient* client, const char* request_json, uint64_t timeout_us, char* out_buf, size_t out_buf_len);

HorusServiceServer* horus_service_server_new(const char* name);
void                horus_service_server_destroy(HorusServiceServer* server);
void                horus_service_server_set_handler(HorusServiceServer* server, bool (*handler)(const uint8_t*, size_t, uint8_t*, size_t*));

/* ── Action ─────────────────────────────────────────────────────────────── */

typedef struct HorusActionClient HorusActionClient;
typedef struct HorusActionServer HorusActionServer;
typedef struct HorusGoalHandle   HorusGoalHandle;

/* Goal status: 0=Pending, 1=Active, 2=Succeeded, 3=Aborted, 4=Canceled, 5=Rejected */
#define HORUS_GOAL_PENDING    0
#define HORUS_GOAL_ACTIVE     1
#define HORUS_GOAL_SUCCEEDED  2
#define HORUS_GOAL_ABORTED    3
#define HORUS_GOAL_CANCELED   4
#define HORUS_GOAL_REJECTED   5

HorusActionClient* horus_action_client_new(const char* name);
void               horus_action_client_destroy(HorusActionClient* client);
HorusGoalHandle*   horus_action_client_send_goal(const HorusActionClient* client, const char* goal_json);
void               horus_action_client_cancel(HorusGoalHandle* handle);

uint8_t            horus_goal_handle_status(const HorusGoalHandle* handle);
uint64_t           horus_goal_handle_id(const HorusGoalHandle* handle);
bool               horus_goal_handle_is_active(const HorusGoalHandle* handle);
void               horus_goal_handle_destroy(HorusGoalHandle* handle);

HorusActionServer* horus_action_server_new(const char* name);
void               horus_action_server_destroy(HorusActionServer* server);
void               horus_action_server_set_accept_handler(HorusActionServer* server, uint8_t (*handler)(const uint8_t*, size_t));
void               horus_action_server_set_execute_handler(HorusActionServer* server, void (*handler)(uint64_t, const uint8_t*, size_t));
bool               horus_action_server_is_ready(const HorusActionServer* server);

/* ── TransformFrame ─────────────────────────────────────────────────────── */

typedef struct HorusTransformFrame HorusTransformFrame;

HorusTransformFrame* horus_transform_frame_new(void);
HorusTransformFrame* horus_transform_frame_with_capacity(size_t max_frames);
void                 horus_transform_frame_destroy(HorusTransformFrame* tf);
int                  horus_transform_frame_register(const HorusTransformFrame* tf, const char* name, const char* parent);
int                  horus_transform_frame_update(const HorusTransformFrame* tf, const char* frame_name, double tx, double ty, double tz, double qx, double qy, double qz, double qw, uint64_t timestamp_ns);
int                  horus_transform_frame_lookup(const HorusTransformFrame* tf, const char* source, const char* target, double* out);
bool                 horus_transform_frame_can_transform(const HorusTransformFrame* tf, const char* source, const char* target);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HORUS_C_H */
