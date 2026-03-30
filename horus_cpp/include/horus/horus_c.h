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

/* ── Miss Policy Constants ───────────────────────────────────────────────── */

#define HORUS_MISS_WARN      0
#define HORUS_MISS_SKIP      1
#define HORUS_MISS_SAFE_MODE 2
#define HORUS_MISS_STOP      3

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HORUS_C_H */
