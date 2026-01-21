# Changelog

All notable changes to HORUS will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2026-01-21

### Breaking Changes

#### Zero-Overhead IPC Architecture

HORUS v0.3.0 introduces a **zero-overhead IPC architecture** that separates introspection from the hot path. This is a breaking API change.

**Topic API Changes:**

| Operation | v0.2.x (Old) | v0.3.0+ (New) |
|-----------|--------------|---------------|
| Send | `topic.send(msg, &mut ctx)?` | `topic.send(msg)?` |
| Receive | `topic.recv(&mut ctx)` | `topic.recv()` |
| Send with logging | N/A | `topic.send_logged(msg)?` |
| Receive with logging | N/A | `topic.recv_logged()` |

**Return Type Changes:**

| Method | v0.2.x | v0.3.0+ |
|--------|--------|---------|
| `send()` | `HorusResult<()>` | `Result<(), T>` |
| `recv()` | `Option<T>` | `Option<T>` (unchanged) |

**Migration Example:**

```rust
// Before (v0.2.x)
fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
    self.pub.send(data, &mut ctx).ok();
    if let Some(msg) = self.sub.recv(&mut ctx) {
        // process
    }
}

// After (v0.3.0+)
fn tick(&mut self) {
    self.pub.send(data).ok();
    if let Some(msg) = self.sub.recv() {
        // process
    }
}
```

**Why This Change?**

The old API embedded introspection (timing, logging) in every IPC operation, adding 170-640ns overhead even when no monitoring was active. The new architecture:

- **Zero-overhead by default**: `send()` and `recv()` have no introspection overhead
- **Opt-in introspection**: Use `send_logged()` / `recv_logged()` when needed
- **External monitoring**: Tools like `horus monitor` and `horus topic echo` work without affecting performance

See the [Migration Guide](/docs/development/migration-zero-overhead) for detailed migration instructions.

### Added

- `send()` method with zero introspection overhead (~167ns for MpmcShm)
- `recv()` method with zero introspection overhead
- `send_logged()` method for opt-in introspection (requires `T: LogSummary`)
- `recv_logged()` method for opt-in introspection (requires `T: LogSummary`)
- `LogSummary` trait and derive macro for message introspection
- CI benchmark regression tests with absolute latency thresholds:
  - MpmcShm: 600ns max
  - SpscShm: 200ns max
  - MpmcIntra: 100ns max
  - SpscIntra: 50ns max
- External introspection tools work without affecting publisher performance

### Changed

- `Topic::send()` now returns `Result<(), T>` instead of `HorusResult<()>`
  - On buffer full, the original message is returned so you can retry
- All example nodes updated to new zero-overhead API
- All horus_library built-in nodes updated to new API
- Documentation updated with new API examples

### Removed

- `send(msg, ctx)` method (replaced by `send(msg)`)
- `recv(ctx)` method (replaced by `recv()`)
- Embedded introspection in hot path (moved to opt-in `*_logged()` methods)

### Performance

| Backend | v0.2.x | v0.3.0+ | Improvement |
|---------|--------|---------|-------------|
| MpmcShm | ~450-800ns | ~167ns | 2.7-4.8x faster |
| SpscShm | ~200-350ns | ~85ns | 2.4-4.1x faster |
| MpmcIntra | ~180-300ns | ~80ns | 2.2-3.7x faster |
| SpscIntra | ~80-150ns | ~25ns | 3.2-6.0x faster |

## [0.2.0] - Previous Release

See git history for changes prior to v0.3.0.
