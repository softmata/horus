# HORUS DX Overhaul Proposal

**Goal:** Make HORUS genuinely easier to use than ROS2 for beginners, while keeping expert-grade performance.

HORUS already beats ROS2 on latency (87ns vs 50us), memory safety (Rust), and setup (1 command vs 10+). But API ergonomics — the thing beginners interact with every day — has gaps that undercut the "gentle learning curve" promise in the README. This proposal fixes that.

---

## Priority 1: Things That Frustrate the First Month

### 1.1 Add Structured Error Context

**The Problem:** Errors are created with `format!()` strings that throw away the original error's type and backtrace:

```rust
// Current — loses the io::Error's kind, source, backtrace
let contents = std::fs::read_to_string(path)
    .map_err(|e| HorusError::config(format!("Failed to read config file: {}", e)))?;

// Another example — loses the plugin error chain
let plugin = self.get_plugin(plugin_id).map_err(|e| {
    HorusError::driver(format!("Failed to get plugin '{}': {}", plugin_id, e))
})?;
```

A beginner sees `"Config error: Failed to read config file: No such file or directory"` and has to guess which file. The original `io::Error` had `.path()` context but it's gone.

**ROS2 comparison:** ROS2 errors are equally bad (C++ exceptions with string messages). HORUS can do much better with Rust's error ecosystem.

**Fix:** Add `.context()` extension and structured error fields:

```rust
// AFTER — preserves source error, adds context
let contents = std::fs::read_to_string(path)
    .horus_context(|| format!("Reading config file: {}", path.display()))?;

// Error display now shows the chain:
// Config error: Reading config file: /etc/horus/config.toml
//   Caused by: No such file or directory (os error 2)
```

Implementation: Add a `source: Option<Box<dyn Error>>` field to relevant `HorusError` variants. Add a `HorusContext` extension trait on `Result<T, E>` that wraps errors while preserving the source chain. This is the pattern `anyhow` and `eyre` use — proven ergonomic.

---

### 1.2 Clean Up the `HorusError::Other` Catch-All

**The Problem:** `HorusError::Other(String)` with blanket `From<String>` and `From<&str>` implementations means any string silently becomes an error:

```rust
// This compiles and creates a useless error
let err: HorusError = "something went wrong".into();
```

Library code uses `Other` as an escape hatch instead of adding proper variants. Over time, more errors become unstructured strings.

**Fix:** Remove `From<String>` and `From<&str>` impls. Replace `Other(String)` with `Internal { message: String, file: &'static str, line: u32 }` populated by a macro:

```rust
// Forces structured errors in library code
return Err(HorusError::config("Invalid bitrate"));  // Good — specific variant

// For truly internal/unexpected errors
return Err(horus_internal!("Unexpected state: {:?}", state));
// Expands to: HorusError::Internal { message: "...", file: "src/foo.rs", line: 42 }
```

---

### 1.3 Simplify Send/Receive API Variants

**The Problem:** `Topic<T>` has 6 send methods and 6 receive methods:

```rust
topic.send(msg)          // basic
topic.send_tracked(msg)  // with metrics
topic.send_logged(msg)   // with logging
topic.recv()             // basic
topic.recv_tracked()     // with metrics
topic.recv_logged()      // with logging
```

A beginner doesn't know which one to use. The answer is usually "just use `send`/`recv`" but the other variants clutter autocomplete and documentation.

**ROS2 comparison:** ROS2 has `publish()` and subscription callbacks. One method each. Metrics and logging are configured on the middleware, not per-call.

**Fix:** Make `send`/`recv` the only public methods. Move tracking and logging to topic-level configuration:

```rust
// Configure once
let topic = Topic::new("sensor_data")?
    .with_metrics(true)
    .with_logging(LogLevel::Debug);

// Use simply
topic.send(msg)?;
let data = topic.recv();

// Metrics and logging happen automatically based on config
```

---

## Priority 2: Polish That Sets HORUS Apart

### 2.1 Rename `Topic::new` → `Topic::create` for Pub/Sub Clarity

Small rename that reads better in beginner code:

```rust
// Current — "new" doesn't convey communication semantics
let topic = Topic::new("cmd_vel")?;

// Proposed — clearer intent
let publisher = Topic::publish("cmd_vel")?;
let subscriber = Topic::subscribe("cmd_vel")?;

// For bidirectional (current pattern still works)
let topic = Topic::create("sensor_data")?;
```

This makes the publish/subscribe pattern explicit at the call site, which is how every tutorial will use it.

### 2.2 Add Semantic Priority Names to NodeBuilder

```rust
// Current — what does order(0) mean? Lower = first? Higher = first?
scheduler.add(my_node).order(0).done();

// Proposed — keep .order() for experts, add semantic names
scheduler.add(sensor_node).high_priority().done();    // order(10)
scheduler.add(control_node).normal_priority().done();  // order(100)
scheduler.add(logger_node).low_priority().done();      // order(200)
scheduler.add(monitor_node).background().done();       // order(900)
```

---

## What NOT to Change

Some things that look like issues but shouldn't be "fixed":

1. **`repr(C)` on message types** — Required for zero-copy shared memory. The perf benefit is worth the constraint. Document it; don't remove it.

2. **NodeConfig's 8 fields** — The `NodeBuilder` fluent API already handles this well. The underlying struct complexity is appropriate for what it configures.

3. **The 23 `HorusError` variants** — Having specific error variants is good. The issue is the `Other` catch-all and string-based context, not the number of variants.

4. **The action! macro** — It's already well-designed and matches ROS2's action concept while being more ergonomic. Leave it.

---

## Migration Strategy

All changes can be done incrementally. None require a flag day.

| Change | Breaking? | Migration Path |
|--------|-----------|----------------|
| main.rs split | No | Internal refactor. CLI behavior unchanged. |
| Error context | No | Additive. Existing error creation unchanged. |
| `Other` removal | Yes | Replace `"string".into()` with specific variant. Grep-able. |
| Send/recv simplification | Yes | Config-based metrics. Old methods → deprecated then removed. |
| `Topic::publish`/`subscribe` | No | Additive aliases. `Topic::new` stays. |
| Priority names | No | Additive. `.order()` stays. |

**Breaking changes: 2 out of 6.** Both are mechanical search-replace migrations.

---

## The ROS2 Comparison Test

For each change, the question is: "Would a ROS2 user switching to HORUS find this easier?"

| Feature | ROS2 Today | HORUS Today | HORUS After This Proposal |
|---------|-----------|-------------|--------------------------|
| Timestamp clarity | `msg.header.stamp.sec` + `.nanosec` | `msg.timestamp_ns` (unambiguous) ✅ | `msg.timestamp_ns` (unambiguous) ✅ |
| Transform types | `Transform` vs `TransformStamped` (clear) | `Transform` vs `TransformStamped` (clear) ✅ | `Transform` vs `TransformStamped` (clear) ✅ |
| Topic type safety | None (strings everywhere) | `topics!` macro + `Topic::publish/subscribe` ✅ | `topics!` macro + `Topic::publish/subscribe` ✅ |
| Error messages | C++ exceptions (opaque) | String-formatted (no chain) | Chained context (source preserved) |
| CLI codebase | Modular (rclcpp_components) | 6,442-line monolith | Modular command handlers |

**HORUS wins on 5 of 5 categories after these changes** (vs 3 of 5 today, DX-wise). Combined with the existing latency, safety, and setup advantages, this makes the "easier than ROS2" claim defensible.
