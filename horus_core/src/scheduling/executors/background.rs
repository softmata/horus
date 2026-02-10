//! Background executor for low-priority node execution
//!
//! Runs background-tier nodes in a dedicated low-priority thread,
//! preventing them from interfering with real-time control loops.

use crate::core::node::{Node, NodeInfo};
use crate::error::HorusResult;
use crate::horus_internal;
use crossbeam::channel::{bounded, Receiver, Sender, TryRecvError};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// Message type for background node communication
#[derive(Debug)]
enum BackgroundMessage {
    /// Trigger a tick for the node
    Tick,
    /// Shutdown the node
    Shutdown,
}

/// Result from background node execution
#[derive(Debug)]
pub struct BackgroundResult {
    pub node_name: String,
    pub duration: Duration,
    pub success: bool,
    pub error: Option<String>,
}

/// Background executor for low-priority node execution
///
/// Runs nodes in a dedicated thread with:
/// - Lower OS thread priority (nice value)
/// - Rate limiting to prevent CPU hogging
/// - Non-blocking communication with main scheduler
pub struct BackgroundExecutor {
    /// Worker thread handles (one per node)
    worker_handles: Vec<JoinHandle<()>>,
    /// Channels to send messages to background nodes
    message_txs: Vec<Sender<BackgroundMessage>>,
    /// Channel to receive results from background nodes
    result_rx: Receiver<BackgroundResult>,
    /// Channel sender for spawning new nodes
    result_tx: Sender<BackgroundResult>,
    /// Running flag
    running: Arc<AtomicBool>,
    /// Node names for tracking
    node_names: Vec<String>,
}

impl BackgroundExecutor {
    /// Create a new background executor
    pub fn new() -> HorusResult<Self> {
        let (result_tx, result_rx) = bounded(1024);
        let running = Arc::new(AtomicBool::new(true));

        Ok(Self {
            worker_handles: Vec::new(),
            message_txs: Vec::new(),
            result_rx,
            result_tx,
            running,
            node_names: Vec::new(),
        })
    }

    /// Spawn a node to run in background tier
    ///
    /// The node will run in a dedicated low-priority thread
    pub fn spawn_node(
        &mut self,
        node: Box<dyn Node>,
        context: Option<NodeInfo>,
    ) -> HorusResult<()> {
        let node_name = node.name().to_string();
        let (msg_tx, msg_rx) = bounded::<BackgroundMessage>(64);
        let result_tx = self.result_tx.clone();
        let running = Arc::clone(&self.running);

        self.message_txs.push(msg_tx);
        self.node_names.push(node_name.clone());

        // Spawn dedicated thread for this background node
        let handle = thread::Builder::new()
            .name(format!("horus-bg-{}", node_name))
            .spawn(move || {
                // Try to set lower thread priority (best effort)
                #[cfg(unix)]
                {
                    unsafe {
                        // Set nice value to 10 (lower priority)
                        libc::nice(10);
                    }
                }

                let mut node = node;
                let mut context = context;

                while running.load(Ordering::Relaxed) {
                    match msg_rx.try_recv() {
                        Ok(BackgroundMessage::Tick) => {
                            // Start tick timer
                            if let Some(ref mut ctx) = context {
                                ctx.start_tick();
                            }

                            let start = Instant::now();

                            // Execute node tick with panic catching
                            let result =
                                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                    node.tick();
                                }));

                            let duration = start.elapsed();

                            let (success, error) = match result {
                                Ok(_) => {
                                    if let Some(ref mut ctx) = context {
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
                                    if let Some(ref mut ctx) = context {
                                        ctx.record_tick_failure(msg.clone());
                                    }
                                    (false, Some(msg))
                                }
                            };

                            // Send result back (non-blocking)
                            let _ = result_tx.try_send(BackgroundResult {
                                node_name: node_name.clone(),
                                duration,
                                success,
                                error,
                            });
                        }
                        Ok(BackgroundMessage::Shutdown) => {
                            // Shutdown the node
                            let _ = node.shutdown();
                            break;
                        }
                        Err(TryRecvError::Empty) => {
                            // No message, sleep briefly to avoid busy-waiting
                            // Background nodes don't need to run at high frequency
                            thread::sleep(Duration::from_millis(10));
                        }
                        Err(TryRecvError::Disconnected) => {
                            // Channel closed, shutdown
                            break;
                        }
                    }
                }
            })
            .map_err(|e| horus_internal!("Failed to spawn background thread: {}", e))?;

        self.worker_handles.push(handle);

        println!(
            "[Background] Spawned node '{}' in low-priority thread",
            self.node_names.last().unwrap()
        );

        Ok(())
    }

    /// Trigger tick for all background nodes
    pub fn tick_all(&self) {
        for tx in &self.message_txs {
            // Non-blocking send - if the node is busy, skip this tick
            let _ = tx.try_send(BackgroundMessage::Tick);
        }
    }

    /// Get results from background nodes (non-blocking)
    pub fn poll_results(&self) -> Vec<BackgroundResult> {
        let mut results = Vec::new();
        while let Ok(result) = self.result_rx.try_recv() {
            results.push(result);
        }
        results
    }

    /// Shutdown all background nodes
    pub fn shutdown(&mut self) {
        self.running.store(false, Ordering::SeqCst);

        // Send shutdown message to all nodes
        for tx in &self.message_txs {
            let _ = tx.send(BackgroundMessage::Shutdown);
        }

        // Wait for worker threads to finish (with timeout)
        for handle in self.worker_handles.drain(..) {
            let _ = handle.join();
        }

        self.message_txs.clear();
        self.node_names.clear();
    }

    /// Get number of background nodes
    pub fn node_count(&self) -> usize {
        self.node_names.len()
    }

    /// Get names of background nodes
    pub fn node_names(&self) -> &[String] {
        &self.node_names
    }
}

impl Default for BackgroundExecutor {
    fn default() -> Self {
        Self::new().expect("Failed to create background executor")
    }
}

impl Drop for BackgroundExecutor {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_background_executor_creation() {
        let executor = BackgroundExecutor::new();
        assert!(executor.is_ok());
    }
}
