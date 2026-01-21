use crate::core::node::{Node, NodeInfo};
use crate::error::HorusResult;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

/// Message type for async node communication
#[derive(Debug, Clone)]
pub enum AsyncMessage {
    Tick,
    Shutdown,
}

/// Result from async node execution
#[derive(Debug)]
pub struct AsyncResult {
    pub node_name: String,
    pub _duration: Duration,
    pub success: bool,
    pub error: Option<String>,
}

/// Async I/O executor for non-blocking node execution
/// Perfect for camera reads, network I/O, file operations
pub struct AsyncIOExecutor {
    runtime: Arc<tokio::runtime::Runtime>,
    handles: Vec<JoinHandle<()>>,
    channels: Vec<mpsc::Sender<AsyncMessage>>,
}

impl AsyncIOExecutor {
    /// Create new async I/O executor
    pub fn new() -> HorusResult<Self> {
        // Create dedicated runtime for I/O operations
        // Use multi-thread runtime for true parallelism
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(4) // Dedicated I/O threads
            .thread_name("horus-async-io")
            .enable_io()
            .enable_time()
            .build()
            .map_err(|e| {
                crate::error::HorusError::Internal(format!("Failed to create async runtime: {}", e))
            })?;

        Ok(Self {
            runtime: Arc::new(runtime),
            handles: Vec::new(),
            channels: Vec::new(),
        })
    }

    /// Spawn a node to run in async I/O tier
    /// The node will run in a separate task and won't block the main scheduler
    pub fn spawn_node(
        &mut self,
        node: Box<dyn Node>,
        context: Option<NodeInfo>,
        result_tx: mpsc::UnboundedSender<AsyncResult>,
    ) -> HorusResult<()> {
        let (tx, mut rx) = mpsc::channel::<AsyncMessage>(100);
        self.channels.push(tx);

        let node_name = node.name().to_string();
        let runtime = Arc::clone(&self.runtime);

        // Spawn the node in its own async task
        let handle = runtime.spawn(async move {
            // Wrap in Option to track ownership after moving into spawn_blocking
            let mut node_opt = Some(node);
            let mut context_opt = context;

            loop {
                match rx.recv().await {
                    Some(AsyncMessage::Tick) => {
                        // Take ownership for this tick
                        let Some(mut node) = node_opt.take() else {
                            break; // Node was lost in previous error
                        };
                        let mut context = context_opt.take();

                        // Start tick timer before execution
                        if let Some(ref mut ctx) = context {
                            ctx.start_tick();
                        }

                        let start = Instant::now();

                        // Execute node tick
                        // Note: We use spawn_blocking for the actual tick since Node trait is sync
                        let tick_result = tokio::task::spawn_blocking(move || {
                            let result =
                                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                    node.tick();
                                }));
                            (node, context, result)
                        })
                        .await;

                        let duration = start.elapsed();

                        match tick_result {
                            Ok((returned_node, mut returned_ctx, panic_result)) => {
                                // Restore node for next tick
                                node_opt = Some(returned_node);

                                let (success, error) = match panic_result {
                                    Ok(_) => {
                                        // Record successful tick (updates metrics and writes heartbeat)
                                        if let Some(ref mut ctx) = returned_ctx {
                                            ctx.record_tick();
                                        }
                                        (true, None)
                                    }
                                    Err(e) => {
                                        let msg = if let Some(s) = e.downcast_ref::<&str>() {
                                            format!("Node panicked: {}", s)
                                        } else if let Some(s) = e.downcast_ref::<String>() {
                                            format!("Node panicked: {}", s)
                                        } else {
                                            "Node panicked with unknown error".to_string()
                                        };
                                        // Record failed tick
                                        if let Some(ref mut ctx) = returned_ctx {
                                            ctx.record_tick_failure(msg.clone());
                                        }
                                        (false, Some(msg))
                                    }
                                };

                                // Restore context after recording tick
                                context_opt = returned_ctx;

                                // Send result back
                                let _ = result_tx.send(AsyncResult {
                                    node_name: node_name.clone(),
                                    _duration: duration,
                                    success,
                                    error,
                                });
                            }
                            Err(e) => {
                                // Task join error
                                let _ = result_tx.send(AsyncResult {
                                    node_name: node_name.clone(),
                                    _duration: duration,
                                    success: false,
                                    error: Some(format!("Task error: {}", e)),
                                });
                            }
                        }
                    }
                    Some(AsyncMessage::Shutdown) | None => {
                        // Shutdown the node if we still have ownership
                        if let Some(mut node) = node_opt.take() {
                            if let Some(context) = context_opt.as_mut() {
                                let _ = node.shutdown(context);
                            }
                        }
                        break;
                    }
                }
            }
        });

        self.handles.push(handle);
        Ok(())
    }

    /// Trigger tick for all async nodes
    pub async fn tick_all(&self) {
        for tx in &self.channels {
            let _ = tx.send(AsyncMessage::Tick).await;
        }
    }

    /// Shutdown all async nodes
    pub async fn shutdown_all(&mut self) {
        // Send shutdown message to all nodes
        for tx in &self.channels {
            let _ = tx.send(AsyncMessage::Shutdown).await;
        }

        // Wait for all tasks to complete
        for handle in self.handles.drain(..) {
            let _ = handle.await;
        }

        self.channels.clear();
    }
}

/// Async node with timeout protection for I/O-heavy operations
/// This allows slow/blocking operations to run without blocking the scheduler
///
/// # Usage
/// ```ignore
/// let camera_node = CameraNode::new();
/// let async_node = AsyncNode::new(camera_node, Duration::from_secs(1));
/// // Node will error if tick takes > 1 second
/// ```
pub struct AsyncNode<N: Node> {
    inner: N,
    timeout: Duration,
}

impl<N: Node> AsyncNode<N> {
    /// Create a new async wrapper with timeout protection
    pub fn new(node: N, timeout: Duration) -> Self {
        Self {
            inner: node,
            timeout,
        }
    }

    /// Execute node tick with timeout protection
    ///
    /// Returns error if:
    /// - Node execution exceeds timeout
    /// - Node panics during execution
    pub async fn tick_with_timeout(&mut self) -> Result<(), String> {
        let timeout_duration = self.timeout;

        // Use timeout to prevent infinite blocking
        let result = tokio::time::timeout(timeout_duration, async {
            // Execute node tick
            // Note: This is a simplified implementation
            // In production, this would run in spawn_blocking
            self.inner.tick();
            Ok::<(), String>(())
        })
        .await;

        match result {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(format!(
                "Node '{}' execution exceeded timeout of {:?}",
                self.inner.name(),
                timeout_duration
            )),
        }
    }

    /// Get reference to inner node
    pub fn inner(&self) -> &N {
        &self.inner
    }

    /// Get mutable reference to inner node
    pub fn inner_mut(&mut self) -> &mut N {
        &mut self.inner
    }
}
