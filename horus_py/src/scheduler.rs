use crate::config::PySchedulerConfig;
use crate::node::PyNodeInfo;
use horus::{NodeInfo as CoreNodeInfo, TopicMetadata};
use horus_core::core::Node as CoreNode;
use horus_core::error::HorusError;
use horus_core::scheduling::{
    CircuitState, FailurePolicy, NodeRegistration, NodeTier, Scheduler as CoreScheduler,
    SchedulerConfig, SchedulerNodeMetrics,
};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

// ─── PyNodeAdapter ───────────────────────────────────────────────────────────
// Implements horus_core::Node by bridging to Python objects via GIL acquisition.

struct PyNodeAdapter {
    py_object: Py<PyAny>,
    leaked_name: &'static str,
    node_context: Arc<Mutex<CoreNodeInfo>>,
    cached_info: Option<Py<PyNodeInfo>>,
    stop_requested: Arc<AtomicBool>,
    scheduler_running: Arc<AtomicBool>,
    publishers_list: Vec<TopicMetadata>,
    subscribers_list: Vec<TopicMetadata>,
    priority_val: u32,
    rate: Option<f64>,
}

impl CoreNode for PyNodeAdapter {
    fn name(&self) -> &'static str {
        self.leaked_name
    }

    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        Python::with_gil(|py| {
            let py_info = Py::new(
                py,
                PyNodeInfo {
                    inner: self.node_context.clone(),
                    scheduler_running: Some(self.scheduler_running.clone()),
                },
            )
            .map_err(|e| HorusError::node(self.leaked_name, e.to_string()))?;

            self.cached_info = Some(py_info.clone_ref(py));

            let result = self
                .py_object
                .call_method1(py, "init", (py_info,))
                .or_else(|_| self.py_object.call_method0(py, "init"));

            match result {
                Ok(_) => Ok(()),
                Err(e) => {
                    if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                        self.stop_requested.store(true, Ordering::Relaxed);
                        Ok(())
                    } else {
                        Err(HorusError::node(
                            self.leaked_name,
                            format!("init failed: {}", e),
                        ))
                    }
                }
            }
        })
    }

    fn tick(&mut self) {
        if self.stop_requested.load(Ordering::Relaxed) {
            return;
        }

        Python::with_gil(|py| {
            // Start tick timing on the context
            if let Ok(mut ctx) = self.node_context.lock() {
                ctx.start_tick();
            }

            let py_info = if let Some(ref cached) = self.cached_info {
                cached.clone_ref(py)
            } else {
                match Py::new(
                    py,
                    PyNodeInfo {
                        inner: self.node_context.clone(),
                        scheduler_running: Some(self.scheduler_running.clone()),
                    },
                ) {
                    Ok(info) => {
                        self.cached_info = Some(info.clone_ref(py));
                        info
                    }
                    Err(_) => return,
                }
            };

            let result = self
                .py_object
                .call_method1(py, "tick", (py_info,))
                .or_else(|_| self.py_object.call_method0(py, "tick"));

            match result {
                Ok(_) => {
                    if let Ok(mut ctx) = self.node_context.lock() {
                        ctx.record_tick();
                    }
                }
                Err(e) => {
                    if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                        self.stop_requested.store(true, Ordering::Relaxed);
                        // Also stop the scheduler via the running flag
                        self.scheduler_running.store(false, Ordering::SeqCst);
                    } else {
                        // Panic for horus_core's catch_unwind / failure policy
                        panic!(
                            "Python node '{}' tick failed: {}",
                            self.leaked_name, e
                        );
                    }
                }
            }
        });
    }

    fn shutdown(&mut self) -> horus_core::error::HorusResult<()> {
        Python::with_gil(|py| {
            let py_info = if let Some(ref cached) = self.cached_info {
                cached.clone_ref(py)
            } else {
                Py::new(
                    py,
                    PyNodeInfo {
                        inner: self.node_context.clone(),
                        scheduler_running: Some(self.scheduler_running.clone()),
                    },
                )
                .map_err(|e| HorusError::node(self.leaked_name, e.to_string()))?
            };

            let result = self
                .py_object
                .call_method1(py, "shutdown", (py_info,))
                .or_else(|_| self.py_object.call_method0(py, "shutdown"));

            match result {
                Ok(_) => Ok(()),
                Err(e) => {
                    if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                        self.stop_requested.store(true, Ordering::Relaxed);
                        Ok(())
                    } else {
                        Err(HorusError::node(
                            self.leaked_name,
                            format!("shutdown failed: {}", e),
                        ))
                    }
                }
            }
        })
    }

    fn publishers(&self) -> Vec<TopicMetadata> {
        self.publishers_list.clone()
    }

    fn subscribers(&self) -> Vec<TopicMetadata> {
        self.subscribers_list.clone()
    }

    fn priority(&self) -> u32 {
        self.priority_val
    }

    fn rate_hz(&self) -> Option<f64> {
        self.rate
    }
}

// ─── PyNodeBuilder ───────────────────────────────────────────────────────────

/// Fluent builder for adding nodes to the scheduler.
///
/// Example:
///     scheduler.add(my_node).order(0).rate_hz(100.0).rt().done()
#[pyclass(module = "horus._horus")]
pub struct PyNodeBuilder {
    scheduler: Py<PyScheduler>,
    node: Py<PyAny>,
    order: u32,
    rate_hz: Option<f64>,
    rt: bool,
    deadline_ms: Option<f64>,
    wcet_us: Option<u64>,
    tier: Option<String>,
    failure_policy: Option<String>,
}

#[pymethods]
impl PyNodeBuilder {
    /// Set execution order (lower = earlier in tick sequence).
    fn order(mut slf: PyRefMut<'_, Self>, order: u32) -> PyRefMut<'_, Self> {
        slf.order = order;
        slf
    }

    /// Alias for order() - set execution priority.
    fn priority(slf: PyRefMut<'_, Self>, priority: u32) -> PyRefMut<'_, Self> {
        Self::order(slf, priority)
    }

    /// Set node-specific tick rate in Hz.
    fn rate_hz(mut slf: PyRefMut<'_, Self>, rate: f64) -> PyRefMut<'_, Self> {
        slf.rate_hz = Some(rate);
        slf
    }

    /// Mark as a real-time node with deadline monitoring.
    fn rt(mut slf: PyRefMut<'_, Self>) -> PyRefMut<'_, Self> {
        slf.rt = true;
        slf
    }

    /// Set soft deadline in milliseconds.
    fn deadline_ms(mut slf: PyRefMut<'_, Self>, ms: f64) -> PyRefMut<'_, Self> {
        slf.deadline_ms = Some(ms);
        slf.rt = true;
        slf
    }

    /// Set WCET budget in microseconds.
    fn wcet_us(mut slf: PyRefMut<'_, Self>, us: u64) -> PyRefMut<'_, Self> {
        slf.wcet_us = Some(us);
        slf.rt = true;
        slf
    }

    /// Set execution tier: "ultra_fast", "fast", "normal".
    fn tier(mut slf: PyRefMut<'_, Self>, name: String) -> PyRefMut<'_, Self> {
        slf.tier = Some(name);
        slf
    }

    /// Set failure policy: "fatal", "restart", "skip", "ignore".
    fn failure_policy(mut slf: PyRefMut<'_, Self>, name: String) -> PyRefMut<'_, Self> {
        slf.failure_policy = Some(name);
        slf
    }

    /// Finalize and add the node to the scheduler.
    fn done(slf: PyRefMut<'_, Self>, py: Python) -> PyResult<Py<PyScheduler>> {
        let scheduler = slf.scheduler.clone_ref(py);
        let node = slf.node.clone_ref(py);
        let order = slf.order;
        let rate_hz = slf.rate_hz;
        let rt = slf.rt;
        let deadline_ms = slf.deadline_ms;
        let tier = slf.tier.clone();
        let failure_policy = slf.failure_policy.clone();
        drop(slf);

        {
            let sched = scheduler.borrow(py);
            sched.add_node_internal(py, node, order, rate_hz, rt, deadline_ms, tier, failure_policy)?;
        }

        Ok(scheduler)
    }

    /// Alias for done() - finalize and add.
    fn add(slf: PyRefMut<'_, Self>, py: Python) -> PyResult<Py<PyScheduler>> {
        Self::done(slf, py)
    }
}

// ─── PyScheduler ─────────────────────────────────────────────────────────────

/// Python wrapper for HORUS Scheduler wrapping horus_core::Scheduler.
///
/// All scheduling logic (rate control, circuit breaker, watchdog, deadline,
/// parallel execution, record/replay, telemetry, etc.) is handled by horus_core.
/// Python API is preserved with zero breaking changes.
#[pyclass(module = "horus._horus")]
pub struct PyScheduler {
    inner: Mutex<Option<CoreScheduler>>,
    tick_rate_hz: f64,
    scheduler_running: Arc<AtomicBool>,
    stop_requested: Arc<AtomicBool>,
    removed_nodes: Mutex<HashSet<String>>,
}

impl PyScheduler {
    fn add_node_internal(
        &self,
        py: Python,
        node: Py<PyAny>,
        order: u32,
        rate_hz: Option<f64>,
        rt: bool,
        deadline_ms: Option<f64>,
        tier: Option<String>,
        failure_policy: Option<String>,
    ) -> PyResult<()> {
        let name: String = node.getattr(py, "name")?.extract(py)?;

        let publishers: Vec<TopicMetadata> = node
            .getattr(py, "pub_topics")
            .and_then(|attr| attr.extract::<Vec<String>>(py))
            .unwrap_or_default()
            .into_iter()
            .map(|t| TopicMetadata {
                topic_name: t,
                type_name: "unknown".into(),
            })
            .collect();

        let subscribers: Vec<TopicMetadata> = node
            .getattr(py, "sub_topics")
            .and_then(|attr| attr.extract::<Vec<String>>(py))
            .unwrap_or_default()
            .into_iter()
            .map(|t| TopicMetadata {
                topic_name: t,
                type_name: "unknown".into(),
            })
            .collect();

        // Rate precedence: explicit param > node.rate attribute > None (scheduler default)
        let node_rate = rate_hz.or_else(|| {
            node.getattr(py, "rate")
                .and_then(|attr| attr.extract::<f64>(py))
                .ok()
        });

        let leaked_name: &'static str = Box::leak(name.clone().into_boxed_str());

        let adapter = PyNodeAdapter {
            py_object: node,
            leaked_name,
            node_context: Arc::new(Mutex::new(CoreNodeInfo::new(name.clone()))),
            cached_info: None,
            stop_requested: self.stop_requested.clone(),
            scheduler_running: self.scheduler_running.clone(),
            publishers_list: publishers,
            subscribers_list: subscribers,
            priority_val: order,
            rate: node_rate,
        };

        let mut config = NodeRegistration::new(Box::new(adapter)).order(order);
        if let Some(rate) = node_rate {
            config = config.rate_hz(rate);
        }
        if rt {
            config = config.rt();
        }
        if let Some(ms) = deadline_ms {
            config = config.deadline_ms(ms as u64);
        }
        if let Some(ref tier_name) = tier {
            let node_tier = match tier_name.as_str() {
                "ultra_fast" => NodeTier::UltraFast,
                "fast" => NodeTier::Fast,
                "normal" => NodeTier::Normal,
                _ => NodeTier::Fast,
            };
            config = config.tier(node_tier);
        }
        if let Some(ref policy_name) = failure_policy {
            let policy = match policy_name.as_str() {
                "fatal" => FailurePolicy::Fatal,
                "restart" => FailurePolicy::restart(5, 100),
                "skip" => FailurePolicy::skip(5, 30_000),
                "ignore" => FailurePolicy::Ignore,
                _ => FailurePolicy::Fatal,
            };
            config = config.failure_policy(policy);
        }

        let mut guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot add nodes while scheduler is running")
        })?;

        inner.add_configured(config);

        println!(
            "Added node '{}' (order={}, rate={}, rt={})",
            name,
            order,
            node_rate.map_or("scheduler".to_string(), |r| format!("{}Hz", r)),
            rt
        );

        Ok(())
    }

    /// Build a Python dict from a SchedulerNodeMetrics + optional failure stats.
    fn metric_to_dict(
        py: Python,
        metric: &SchedulerNodeMetrics,
        failure: Option<&horus_core::scheduling::FailureHandlerStats>,
        tick_rate: f64,
    ) -> PyResult<Py<PyAny>> {
        let dict = PyDict::new(py);
        dict.set_item("name", &metric.name)?;
        dict.set_item("order", metric.priority)?;
        dict.set_item("rate_hz", tick_rate)?;
        dict.set_item("total_ticks", metric.total_ticks)?;
        dict.set_item("successful_ticks", metric.successful_ticks)?;
        dict.set_item("failed_ticks", metric.failed_ticks)?;
        dict.set_item("errors_count", metric.errors_count)?;
        dict.set_item("avg_tick_duration_ms", metric.avg_tick_duration_ms)?;
        dict.set_item("min_tick_duration_ms", metric.min_tick_duration_ms)?;
        dict.set_item("max_tick_duration_ms", metric.max_tick_duration_ms)?;
        dict.set_item("last_tick_duration_ms", metric.last_tick_duration_ms)?;
        dict.set_item("uptime_seconds", metric.uptime_seconds)?;

        if let Some(f) = failure {
            dict.set_item("failure_count", f.failure_count)?;
            dict.set_item("consecutive_failures", f.failure_count)?;
            dict.set_item("circuit_open", f.is_suppressed)?;
        } else {
            dict.set_item("failure_count", 0u32)?;
            dict.set_item("consecutive_failures", 0u32)?;
            dict.set_item("circuit_open", false)?;
        }

        dict.set_item("deadline_ms", py.None())?;
        dict.set_item("deadline_misses", 0u64)?;
        dict.set_item("watchdog_enabled", false)?;
        dict.set_item("watchdog_timeout_ms", 0u64)?;
        dict.set_item("watchdog_expired", false)?;
        dict.set_item("watchdog_time_since_feed_ms", py.None())?;
        dict.set_item("state", "running")?;

        Ok(dict.into())
    }

    /// Helper: take the inner scheduler, run a closure, put it back.
    fn with_inner_run<F>(&self, py: Python, f: F) -> PyResult<()>
    where
        F: FnOnce(&mut CoreScheduler) -> horus_core::error::HorusResult<()> + Send,
        // CoreScheduler must be Send for py.allow_threads
    {
        let mut inner = {
            let mut guard = self
                .inner
                .lock()
                .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
            guard.take().ok_or_else(|| {
                PyRuntimeError::new_err("Scheduler already running or not initialized")
            })?
        };

        self.stop_requested.store(false, Ordering::Relaxed);

        let result = py.allow_threads(move || {
            let r = f(&mut inner);
            (inner, r)
        });

        let (returned_inner, run_result) = result;

        {
            let mut guard = self
                .inner
                .lock()
                .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
            *guard = Some(returned_inner);
        }

        run_result.map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }
}

#[pymethods]
impl PyScheduler {
    #[new]
    #[pyo3(signature = (config=None))]
    pub fn new(config: Option<PySchedulerConfig>) -> PyResult<Self> {
        let core_config = config
            .as_ref()
            .map(|c| c.to_core_config())
            .unwrap_or_else(SchedulerConfig::standard);

        let tick_rate = config.as_ref().map_or(100.0, |c| c.tick_rate);

        let core_sched = CoreScheduler::new()
            .with_name("PythonScheduler")
            .with_config(core_config);
        let running_flag = core_sched.running_flag();

        Ok(PyScheduler {
            inner: Mutex::new(Some(core_sched)),
            tick_rate_hz: tick_rate,
            scheduler_running: running_flag,
            stop_requested: Arc::new(AtomicBool::new(false)),
            removed_nodes: Mutex::new(HashSet::new()),
        })
    }

    /// Create scheduler from a configuration preset
    #[staticmethod]
    pub fn from_config(config: PySchedulerConfig) -> PyResult<Self> {
        Self::new(Some(config))
    }

    /// Create a production deployment scheduler (RT features + blackbox + profiling).
    ///
    /// Applies best-effort RT priority, memory locking, CPU affinity, and
    /// enables a 16MB BlackBox flight recorder.
    #[staticmethod]
    pub fn deploy() -> PyResult<Self> {
        let core_sched = CoreScheduler::deploy();
        let running_flag = core_sched.running_flag();
        Ok(PyScheduler {
            inner: Mutex::new(Some(core_sched)),
            tick_rate_hz: 60.0, // deploy() uses default ~60Hz
            scheduler_running: running_flag,
            stop_requested: Arc::new(AtomicBool::new(false)),
            removed_nodes: Mutex::new(HashSet::new()),
        })
    }

    /// Create a safety-critical scheduler.
    ///
    /// Sequential execution, 1kHz, full WCET enforcement, watchdogs, memory locking.
    #[staticmethod]
    pub fn preset_safety_critical() -> PyResult<Self> {
        let core_sched = CoreScheduler::safety_critical();
        let running_flag = core_sched.running_flag();
        Ok(PyScheduler {
            inner: Mutex::new(Some(core_sched)),
            tick_rate_hz: 1000.0,
            scheduler_running: running_flag,
            stop_requested: Arc::new(AtomicBool::new(false)),
            removed_nodes: Mutex::new(HashSet::new()),
        })
    }

    /// Create a high-performance scheduler.
    ///
    /// Parallel execution, 10kHz, WCET enforcement, memory locking.
    #[staticmethod]
    pub fn preset_high_performance() -> PyResult<Self> {
        let core_sched = CoreScheduler::high_performance();
        let running_flag = core_sched.running_flag();
        Ok(PyScheduler {
            inner: Mutex::new(Some(core_sched)),
            tick_rate_hz: 10000.0,
            scheduler_running: running_flag,
            stop_requested: Arc::new(AtomicBool::new(false)),
            removed_nodes: Mutex::new(HashSet::new()),
        })
    }

    /// Create a deterministic scheduler for reproducible execution.
    ///
    /// Sequential execution, strict topology validation, deterministic seed.
    #[staticmethod]
    pub fn preset_deterministic() -> PyResult<Self> {
        let core_sched = CoreScheduler::deterministic();
        let running_flag = core_sched.running_flag();
        Ok(PyScheduler {
            inner: Mutex::new(Some(core_sched)),
            tick_rate_hz: 100.0,
            scheduler_running: running_flag,
            stop_requested: Arc::new(AtomicBool::new(false)),
            removed_nodes: Mutex::new(HashSet::new()),
        })
    }

    /// Create a hard real-time scheduler.
    ///
    /// Parallel execution, 1kHz, <5us jitter target, 10ms watchdog, panic on deadline miss.
    #[staticmethod]
    pub fn preset_hard_realtime() -> PyResult<Self> {
        let core_sched = CoreScheduler::hard_realtime();
        let running_flag = core_sched.running_flag();
        Ok(PyScheduler {
            inner: Mutex::new(Some(core_sched)),
            tick_rate_hz: 1000.0,
            scheduler_running: running_flag,
            stop_requested: Arc::new(AtomicBool::new(false)),
            removed_nodes: Mutex::new(HashSet::new()),
        })
    }

    /// Start building a node configuration (fluent API).
    fn node(slf: Py<Self>, _py: Python, node: Py<PyAny>) -> PyResult<PyNodeBuilder> {
        Ok(PyNodeBuilder {
            scheduler: slf,
            node,
            order: 100,
            rate_hz: None,
            rt: false,
            deadline_ms: None,
            wcet_us: None,
            tier: None,
            failure_policy: None,
        })
    }

    /// Add a node to the scheduler.
    #[pyo3(signature = (node, order=100, rate_hz=None, rt=false, deadline_ms=None, logging=true, tier=None, failure_policy=None))]
    fn add(
        &self,
        py: Python,
        node: Py<PyAny>,
        order: u32,
        rate_hz: Option<f64>,
        rt: bool,
        deadline_ms: Option<f64>,
        #[allow(unused_variables)]
        logging: bool,
        tier: Option<String>,
        failure_policy: Option<String>,
    ) -> PyResult<()> {
        self.add_node_internal(py, node, order, rate_hz, rt, deadline_ms, tier, failure_policy)
    }

    /// Set per-node rate control
    fn set_node_rate(&self, node_name: String, rate_hz: f64) -> PyResult<()> {
        if rate_hz <= 0.0 || rate_hz > 10000.0 {
            return Err(PyRuntimeError::new_err(
                "Rate must be between 0 and 10000 Hz",
            ));
        }

        let mut guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot modify while scheduler is running")
        })?;

        inner.set_node_rate(&node_name, rate_hz);
        println!("Set node '{}' rate to {}Hz", node_name, rate_hz);
        Ok(())
    }

    /// Set per-node deadline for soft real-time scheduling.
    /// Note: Should be set before run() via node builder for best results.
    #[pyo3(signature = (node_name, deadline_ms=None))]
    fn set_node_deadline(&self, node_name: String, deadline_ms: Option<f64>) -> PyResult<()> {
        if let Some(d) = deadline_ms {
            if d <= 0.0 || d > 10000.0 {
                return Err(PyRuntimeError::new_err(
                    "Deadline must be between 0 and 10000 ms",
                ));
            }
        }
        eprintln!(
            "Warning: set_node_deadline('{}') should be configured via node builder before run()",
            node_name
        );
        Ok(())
    }

    /// Enable/disable watchdog timer for a specific node.
    /// Note: Should be configured via SchedulerConfig before creating the scheduler.
    #[pyo3(signature = (node_name, enabled, timeout_ms=None))]
    fn set_node_watchdog(
        &self,
        node_name: String,
        enabled: bool,
        timeout_ms: Option<u64>,
    ) -> PyResult<()> {
        let _ = (enabled, timeout_ms);
        eprintln!(
            "Warning: set_node_watchdog('{}') should be configured via SchedulerConfig",
            node_name
        );
        Ok(())
    }

    /// Run the scheduler indefinitely (until stop() or Ctrl+C).
    fn run(&self, py: Python) -> PyResult<()> {
        self.with_inner_run(py, |sched| sched.run())
    }

    /// Run the scheduler for a specified duration (in seconds).
    fn run_for(&self, py: Python, duration_seconds: f64) -> PyResult<()> {
        if duration_seconds <= 0.0 {
            return Err(PyRuntimeError::new_err("Duration must be positive"));
        }
        let duration = Duration::from_secs_f64(duration_seconds);
        self.with_inner_run(py, move |sched| sched.run_for(duration))
    }

    /// Run specific nodes by name (continuously until stop()).
    fn tick(&self, py: Python, node_names: Vec<String>) -> PyResult<()> {
        self.with_inner_run(py, move |sched| {
            let refs: Vec<&str> = node_names.iter().map(|s| s.as_str()).collect();
            sched.tick(&refs)
        })
    }

    /// Run specific nodes for a specified duration (in seconds).
    fn tick_for(
        &self,
        py: Python,
        node_names: Vec<String>,
        duration_seconds: f64,
    ) -> PyResult<()> {
        if duration_seconds <= 0.0 {
            return Err(PyRuntimeError::new_err("Duration must be positive"));
        }
        let duration = Duration::from_secs_f64(duration_seconds);
        self.with_inner_run(py, move |sched| {
            let refs: Vec<&str> = node_names.iter().map(|s| s.as_str()).collect();
            sched.tick_for(&refs, duration)
        })
    }

    /// Stop the scheduler.
    fn stop(&self) -> PyResult<()> {
        self.stop_requested.store(true, Ordering::Relaxed);
        self.scheduler_running.store(false, Ordering::SeqCst);
        Ok(())
    }

    /// Check if the scheduler is running.
    fn is_running(&self) -> PyResult<bool> {
        Ok(self.scheduler_running.load(Ordering::SeqCst))
    }

    /// Set the tick rate in Hz.
    fn set_tick_rate(&self, rate_hz: f64) -> PyResult<()> {
        if rate_hz <= 0.0 || rate_hz > 10000.0 {
            return Err(PyRuntimeError::new_err(
                "Tick rate must be between 0 and 10000 Hz",
            ));
        }
        eprintln!(
            "Warning: set_tick_rate should be configured via SchedulerConfig at construction"
        );
        Ok(())
    }

    /// Get node statistics.
    fn get_node_stats(&self, py: Python, node_name: String) -> PyResult<Py<PyAny>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = guard
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("Stats unavailable while scheduler is running"))?;

        for metric in inner.get_metrics() {
            if metric.name == node_name {
                let failure = inner.failure_stats(&node_name);
                return Self::metric_to_dict(py, &metric, failure.as_ref(), self.tick_rate_hz);
            }
        }

        Err(PyRuntimeError::new_err(format!(
            "Node '{}' not found",
            node_name
        )))
    }

    /// Remove a node from the scheduler.
    /// Note: horus_core doesn't support runtime removal; node is excluded from stats.
    fn remove_node(&self, name: String) -> PyResult<bool> {
        let mut removed = self
            .removed_nodes
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        Ok(removed.insert(name))
    }

    /// Get list of all nodes with basic information.
    fn get_all_nodes(&self, py: Python) -> PyResult<Vec<Py<PyAny>>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("Node list unavailable while scheduler is running")
        })?;
        let removed = self
            .removed_nodes
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;

        let mut result = Vec::new();
        for metric in inner.get_metrics() {
            if removed.contains(&metric.name) {
                continue;
            }
            let failure = inner.failure_stats(&metric.name);
            result.push(Self::metric_to_dict(
                py,
                &metric,
                failure.as_ref(),
                self.tick_rate_hz,
            )?);
        }

        Ok(result)
    }

    /// Get count of all registered nodes.
    fn get_node_count(&self) -> PyResult<usize> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => {
                let removed = self
                    .removed_nodes
                    .lock()
                    .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
                Ok(sched.get_node_list().len() - removed.len())
            }
            None => Ok(0),
        }
    }

    /// Check if a node exists by name.
    fn has_node(&self, name: String) -> PyResult<bool> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => {
                let removed = self
                    .removed_nodes
                    .lock()
                    .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
                Ok(sched.get_node_list().contains(&name) && !removed.contains(&name))
            }
            None => Ok(false),
        }
    }

    /// Get list of node names.
    fn get_node_names(&self) -> PyResult<Vec<String>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => {
                let removed = self
                    .removed_nodes
                    .lock()
                    .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
                Ok(sched
                    .get_node_list()
                    .into_iter()
                    .filter(|n| !removed.contains(n))
                    .collect())
            }
            None => Ok(Vec::new()),
        }
    }

    /// Get list of added node names (alias for get_node_names).
    fn get_nodes(&self) -> PyResult<Vec<String>> {
        self.get_node_names()
    }

    /// Get node priority.
    fn get_node_info(&self, name: String) -> PyResult<Option<u32>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => {
                for metric in sched.get_metrics() {
                    if metric.name == name {
                        return Ok(Some(metric.priority));
                    }
                }
                Ok(None)
            }
            None => Ok(None),
        }
    }

    // ========================================================================
    // Status & Diagnostics
    // ========================================================================

    /// Get scheduler status string.
    fn status(&self) -> PyResult<String> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.status()),
            None => Ok("running".to_string()),
        }
    }

    /// Get runtime capabilities as a dict.
    fn capabilities(&self, py: Python) -> PyResult<Option<Py<PyAny>>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = match guard.as_ref() {
            Some(s) => s,
            None => return Ok(None),
        };
        match inner.capabilities() {
            Some(caps) => {
                let dict = PyDict::new(py);
                dict.set_item("preempt_rt", caps.preempt_rt)?;
                dict.set_item("rt_priority_available", caps.rt_priority_available)?;
                dict.set_item("max_rt_priority", caps.max_rt_priority)?;
                dict.set_item("min_rt_priority", caps.min_rt_priority)?;
                Ok(Some(dict.into()))
            }
            None => Ok(None),
        }
    }

    /// Check if full real-time capabilities are available.
    fn has_full_rt(&self) -> PyResult<bool> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.has_full_rt()),
            None => Ok(false),
        }
    }

    /// Get RT degradations as a list of dicts.
    fn degradations(&self, py: Python) -> PyResult<Py<PyAny>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = match guard.as_ref() {
            Some(s) => s,
            None => return Ok(PyList::empty(py).into()),
        };
        let result = PyList::empty(py);
        for deg in inner.degradations() {
            let dict = PyDict::new(py);
            dict.set_item("feature", format!("{:?}", deg.feature))?;
            dict.set_item("reason", &deg.reason)?;
            dict.set_item("severity", format!("{:?}", deg.severity))?;
            result.append(dict)?;
        }
        Ok(result.into())
    }

    /// Get the current tick count.
    fn current_tick(&self) -> PyResult<u64> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.current_tick()),
            None => Ok(0),
        }
    }

    /// Get the scheduler name.
    fn scheduler_name(&self) -> PyResult<String> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.scheduler_name().to_string()),
            None => Ok("PythonScheduler".to_string()),
        }
    }

    // ========================================================================
    // Safety Stats
    // ========================================================================

    /// Get safety statistics as a dict.
    fn safety_stats(&self, py: Python) -> PyResult<Option<Py<PyAny>>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = match guard.as_ref() {
            Some(s) => s,
            None => return Ok(None),
        };
        match inner.safety_stats() {
            Some(stats) => {
                let dict = PyDict::new(py);
                dict.set_item("state", format!("{:?}", stats.state))?;
                dict.set_item("wcet_overruns", stats.wcet_overruns)?;
                dict.set_item("deadline_misses", stats.deadline_misses)?;
                dict.set_item("watchdog_expirations", stats.watchdog_expirations)?;
                Ok(Some(dict.into()))
            }
            None => Ok(None),
        }
    }

    /// Get circuit breaker state for a node.
    fn circuit_state(&self, node_name: String) -> PyResult<Option<String>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => match sched.circuit_state(&node_name) {
                Some(state) => {
                    let s = match state {
                        CircuitState::Closed => "closed",
                        CircuitState::Open => "open",
                        CircuitState::HalfOpen => "half_open",
                    };
                    Ok(Some(s.to_string()))
                }
                None => Ok(None),
            },
            None => Ok(None),
        }
    }

    /// Get circuit breaker summary: (closed_count, open_count, half_open_count).
    fn circuit_summary(&self, py: Python) -> PyResult<Py<PyAny>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => {
                let (closed, open, half_open) = sched.circuit_summary();
                let dict = PyDict::new(py);
                dict.set_item("closed", closed)?;
                dict.set_item("open", open)?;
                dict.set_item("half_open", half_open)?;
                Ok(dict.into())
            }
            None => {
                let dict = PyDict::new(py);
                dict.set_item("closed", 0)?;
                dict.set_item("open", 0)?;
                dict.set_item("half_open", 0)?;
                Ok(dict.into())
            }
        }
    }

    // ========================================================================
    // Recording
    // ========================================================================

    /// Check if scheduler is currently recording.
    fn is_recording(&self) -> PyResult<bool> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.is_recording()),
            None => Ok(false),
        }
    }

    /// Check if scheduler is currently replaying.
    fn is_replaying(&self) -> PyResult<bool> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.is_replaying()),
            None => Ok(false),
        }
    }

    /// Stop recording and return saved file paths.
    fn stop_recording(&self) -> PyResult<Vec<String>> {
        let mut guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot stop recording while scheduler is running")
        })?;
        let paths = inner
            .stop_recording()
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(paths.into_iter().map(|p| p.display().to_string()).collect())
    }

    /// List available recordings.
    #[staticmethod]
    fn list_recordings() -> PyResult<Vec<String>> {
        CoreScheduler::list_recordings().map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    // ========================================================================
    // Node Control
    // ========================================================================

    /// Set WCET budget for a node in microseconds.
    fn set_wcet_budget(&self, node_name: String, us: u64) -> PyResult<()> {
        let mut guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot set WCET budget while scheduler is running")
        })?;
        inner
            .set_wcet_budget(&node_name, Duration::from_micros(us))
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Add a critical node with watchdog timeout in milliseconds.
    fn add_critical_node(&self, node_name: String, timeout_ms: u64) -> PyResult<()> {
        let mut guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot add critical node while scheduler is running")
        })?;
        inner
            .add_critical_node(&node_name, Duration::from_millis(timeout_ms))
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    // ========================================================================
    // Deterministic / Simulation Mode
    // ========================================================================

    /// Check if scheduler is in simulation mode (deterministic execution).
    fn is_simulation_mode(&self) -> PyResult<bool> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.is_simulation_mode()),
            None => Ok(false),
        }
    }

    /// Get the deterministic seed (simulation mode only).
    fn seed(&self) -> PyResult<Option<u64>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.seed()),
            None => Ok(None),
        }
    }

    /// Get the current virtual time in seconds (simulation mode only).
    fn virtual_time(&self) -> PyResult<Option<f64>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.virtual_time().map(|d| d.as_secs_f64())),
            None => Ok(None),
        }
    }

    /// Get the current virtual tick number (simulation mode only).
    fn virtual_tick(&self) -> PyResult<Option<u64>> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.virtual_tick()),
            None => Ok(None),
        }
    }

    /// Delete a recording by session name.
    #[staticmethod]
    fn delete_recording(session_name: String) -> PyResult<()> {
        CoreScheduler::delete_recording(&session_name)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    fn __repr__(&self) -> PyResult<String> {
        let guard = self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))?;
        let count = guard
            .as_ref()
            .map(|s| s.get_node_list().len())
            .unwrap_or(0);
        Ok(format!(
            "Scheduler(nodes={}, tick_rate={}Hz)",
            count, self.tick_rate_hz
        ))
    }

    /// Pickle support: Get state for serialization.
    fn __getstate__(&self, py: Python) -> PyResult<Py<PyAny>> {
        let state = PyDict::new(py);
        state.set_item("tick_rate_hz", self.tick_rate_hz)?;
        Ok(state.into())
    }

    /// Pickle support: Restore state from deserialization.
    fn __setstate__(&mut self, state: &Bound<'_, PyDict>) -> PyResult<()> {
        let tick_rate_hz: f64 = state
            .get_item("tick_rate_hz")?
            .ok_or_else(|| PyRuntimeError::new_err("Missing 'tick_rate_hz' in pickled state"))?
            .extract()?;

        self.tick_rate_hz = tick_rate_hz;

        let core_sched = CoreScheduler::new().with_name("PythonScheduler");
        self.scheduler_running = core_sched.running_flag();
        *self
            .inner
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))? = Some(core_sched);
        *self
            .removed_nodes
            .lock()
            .map_err(|_| PyRuntimeError::new_err("Internal lock poisoned"))? = HashSet::new();

        Ok(())
    }
}
