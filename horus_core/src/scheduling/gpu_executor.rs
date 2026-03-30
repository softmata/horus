//! GPU executor — runs GPU nodes on a dedicated thread with CUDA streams.
//!
//! Each GPU node gets its own CUDA stream. Per tick cycle:
//! 1. **Launch phase**: All ready GPU nodes call tick() (non-blocking kernel launches)
//! 2. **Sync phase**: cudaDeviceSynchronize or per-stream sync
//! 3. **Result phase**: Record metrics via SharedMonitors
//!
//! # Architecture
//!
//! ```text
//!  GPU Thread (owns all GPU nodes + streams)
//!  ┌────────────────────────────────────────────┐
//!  │  CUDA context set on thread                │
//!  │  loop:                                     │
//!  │    for each ready node:                    │
//!  │      set gpu_stream in tick context        │
//!  │      record start event on stream[i]       │
//!  │      node.tick()  // launches GPU kernels  │
//!  │      record end event on stream[i]         │
//!  │    synchronize all streams                 │
//!  │    for each completed node:                │
//!  │      measure GPU duration from events      │
//!  │      report to monitors                    │
//!  │    sleep until next tick period             │
//!  └────────────────────────────────────────────┘
//! ```

use std::panic::AssertUnwindSafe;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::gpu::cuda_ffi::{self, CudaContext};
use crate::gpu::stream::{CudaEvent, CudaStream};
use crate::terminal::print_line;

use super::primitives::NodeRunner;
use super::types::{RegisteredNode, SharedMonitors};

/// GPU executor — manages CUDA streams and ticks GPU nodes.
///
/// Follows the same contract as ComputeExecutor/AsyncExecutor:
/// - `start()` spawns a thread and returns the executor handle
/// - `stop()` joins the thread and returns the nodes
pub(crate) struct GpuExecutor {
    handle: Option<std::thread::JoinHandle<Vec<RegisteredNode>>>,
}

impl GpuExecutor {
    /// Start the GPU executor with the given nodes.
    ///
    /// Creates one CUDA stream per node. The executor thread runs
    /// a tick loop that launches kernels and synchronizes each cycle.
    pub fn start(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> crate::error::HorusResult<Self> {
        nodes.sort_by_key(|n| n.priority);

        let handle = std::thread::Builder::new()
            .name("horus-gpu".to_string())
            .spawn(move || Self::gpu_thread_main(nodes, running, tick_period, monitors))
            .map_err(|e| {
                crate::error::HorusError::Node(crate::error::NodeError::InitFailed {
                    node: "gpu-executor".into(),
                    reason: format!("Failed to spawn GPU thread: {}", e),
                })
            })?;

        Ok(Self {
            handle: Some(handle),
        })
    }

    /// Stop the GPU executor and reclaim its nodes.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        self.handle
            .take()
            .expect("GPU thread handle already consumed")
            .join()
            .expect("GPU thread panicked")
    }

    /// Main function for the GPU executor thread.
    fn gpu_thread_main(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> Vec<RegisteredNode> {
        // Initialize CUDA context on this thread
        let ctx = match Self::init_cuda() {
            Ok(ctx) => ctx,
            Err(e) => {
                print_line(&format!(
                    "[GPU] Failed to initialize CUDA: {}. GPU nodes will not execute.",
                    e
                ));
                return nodes;
            }
        };

        // Create one stream per node
        let mut streams: Vec<Option<CudaStream>> = Vec::with_capacity(nodes.len());
        for node in &nodes {
            match CudaStream::new() {
                Ok(s) => streams.push(Some(s)),
                Err(e) => {
                    print_line(&format!(
                        "[GPU] Failed to create stream for node '{}': {}",
                        node.name, e
                    ));
                    streams.push(None);
                }
            }
        }

        print_line(&format!(
            "[GPU] Started with {} nodes, {} streams, tick period {:?}",
            nodes.len(),
            streams.iter().filter(|s| s.is_some()).count(),
            tick_period,
        ));

        while running.load(Ordering::Relaxed) {
            let loop_start = Instant::now();

            // === Phase 1: Classify ready nodes ===
            let mut ready_indices = Vec::new();
            for (i, node) in nodes.iter().enumerate() {
                if !node.initialized || node.is_stopped || node.is_paused {
                    continue;
                }
                if streams[i].is_none() {
                    continue; // No stream for this node
                }

                // Check control flags (pause/kill from CLI)
                if monitors.node_controls.is_stopped(node.name.as_ref()) {
                    continue;
                }
                if monitors.node_controls.is_paused(node.name.as_ref()) {
                    continue;
                }

                // Per-node rate limiting
                if let Some(rate_hz) = node.rate_hz {
                    if let Some(last_tick) = node.last_tick {
                        let elapsed = loop_start.duration_since(last_tick).as_secs_f64();
                        if rate_hz > 0.0 && elapsed < 1.0 / rate_hz {
                            continue;
                        }
                    }
                }

                ready_indices.push(i);
            }

            if ready_indices.is_empty() {
                let elapsed = loop_start.elapsed();
                if elapsed < tick_period {
                    std::thread::sleep(tick_period - elapsed);
                }
                continue;
            }

            // === Phase 2: Launch — tick all ready nodes ===
            // Start events for GPU-side timing
            let mut start_events: Vec<Option<CudaEvent>> = (0..nodes.len()).map(|_| None).collect();
            let mut end_events: Vec<Option<CudaEvent>> = (0..nodes.len()).map(|_| None).collect();

            let now = Instant::now();
            for &i in &ready_indices {
                if nodes[i].rate_hz.is_some() {
                    nodes[i].last_tick = Some(now);
                }
                if let Some(ref mut ctx) = nodes[i].context {
                    ctx.start_tick();
                }
                if let Some(ref mut recorder) = nodes[i].recorder {
                    recorder.begin_tick(0);
                }

                let stream = streams[i].as_ref().unwrap();

                // Record start event
                start_events[i] = CudaEvent::new().ok().and_then(|e| {
                    e.record(stream).ok()?;
                    Some(e)
                });

                // Inject stream into thread-local TickContext so nodes can
                // access it via horus::gpu_stream().
                crate::core::tick_context::set_gpu_stream(stream.raw());

                // Tick the node — kernels launch asynchronously on the stream
                let tick_result = std::panic::catch_unwind(AssertUnwindSafe(|| {
                    nodes[i].node.tick();
                }));

                // Clear stream from context
                crate::core::tick_context::clear_gpu_stream();

                // Record end event
                end_events[i] = CudaEvent::new().ok().and_then(|e| {
                    e.record(stream).ok()?;
                    Some(e)
                });

                // Handle tick panic immediately (mark node stopped)
                if let Err(panic_err) = tick_result {
                    let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                        format!("[GPU] Node '{}' panicked: {}", nodes[i].name, s)
                    } else if let Some(s) = panic_err.downcast_ref::<String>() {
                        format!("[GPU] Node '{}' panicked: {}", nodes[i].name, s)
                    } else {
                        format!("[GPU] Node '{}' panicked (unknown)", nodes[i].name)
                    };
                    print_line(&error_msg);
                    nodes[i].node.on_error(&error_msg);
                    nodes[i].is_stopped = true;
                }
            }

            // === Phase 3: Synchronize all streams ===
            // Use per-stream sync (not device-wide) to allow future overlap
            for &i in &ready_indices {
                if nodes[i].is_stopped {
                    continue;
                }
                if let Some(ref stream) = streams[i] {
                    if let Err(e) = stream.synchronize() {
                        print_line(&format!(
                            "[GPU] Stream sync failed for node '{}': {}",
                            nodes[i].name, e
                        ));
                    }
                }
            }

            // === Phase 4: Record metrics ===
            for &i in &ready_indices {
                if nodes[i].is_stopped {
                    continue;
                }

                // Compute GPU-side duration from events (more accurate than wall clock)
                let gpu_duration = match (&end_events[i], &start_events[i]) {
                    (Some(end), Some(start)) => end.elapsed_since(start).unwrap_or(Duration::ZERO),
                    _ => Duration::ZERO,
                };

                // Use GPU duration for metrics if available, otherwise wall clock
                let duration = if gpu_duration > Duration::ZERO {
                    gpu_duration
                } else {
                    now.elapsed() // Fallback
                };

                // Record stats
                if let Some(ref mut stats) = nodes[i].rt_stats {
                    stats.record_execution(duration);
                }

                if let Ok(mut profiler) = monitors.profiler.try_lock() {
                    profiler.record(&nodes[i].name, duration);
                }

                if let Some(ref mut recorder) = nodes[i].recorder {
                    recorder.end_tick(duration.as_nanos() as u64);
                }

                monitors.update_registry(&nodes[i], duration.as_nanos() as u64);

                if let Some(ref mut ctx) = nodes[i].context {
                    ctx.record_tick();
                }
            }

            // Sleep until next tick period
            let elapsed = loop_start.elapsed();
            if elapsed < tick_period {
                std::thread::sleep(tick_period - elapsed);
            }
        }

        // Cleanup: drop streams before returning nodes
        drop(streams);
        let _ = ctx; // Keep context alive until here

        print_line(&format!(
            "[GPU] Stopped ({} nodes returning to scheduler)",
            nodes.len()
        ));

        nodes
    }

    /// Initialize CUDA on this thread.
    fn init_cuda() -> Result<CudaContext, String> {
        cuda_ffi::init().map_err(|e| format!("cuInit: {}", e))?;
        let dev = cuda_ffi::device_get(0).map_err(|e| format!("cuDeviceGet: {}", e))?;
        let ctx = CudaContext::new(dev).map_err(|e| format!("cuCtxCreate: {}", e))?;
        ctx.set_current()
            .map_err(|e| format!("cuCtxSetCurrent: {}", e))?;
        Ok(ctx)
    }
}
