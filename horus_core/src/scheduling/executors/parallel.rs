//! Parallel executor module for thread pool configuration
//!
//! This module provides thread pool configuration for the scheduler.

/// Parallel executor configuration for thread pool settings
#[derive(Debug)]
pub struct ParallelExecutor {
    /// Number of worker threads (defaults to CPU count)
    num_threads: usize,
    /// CPU cores to pin threads to (optional)
    cpu_cores: Option<Vec<usize>>,
}

impl ParallelExecutor {
    /// Create new parallel executor with automatic thread count
    pub fn new() -> Self {
        let num_threads = num_cpus::get().max(1);
        Self {
            num_threads,
            cpu_cores: None,
        }
    }

    /// Set the maximum number of threads to use
    pub fn set_max_threads(&mut self, num_threads: usize) {
        self.num_threads = num_threads.max(1);
    }

    /// Set specific CPU cores to pin threads to
    pub fn set_cpu_cores(&mut self, cores: Vec<usize>) {
        if !cores.is_empty() {
            self.cpu_cores = Some(cores);
        }
    }
}

impl Default for ParallelExecutor {
    fn default() -> Self {
        Self::new()
    }
}
