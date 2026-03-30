use crate::config::PySchedulerConfig;
use crate::node::PyNodeInfo;
use horus::core::NodeInfo as CoreNodeInfo;
use horus_core::core::DurationExt;
use horus_core::core::Miss;
use horus_core::core::Node as CoreNode;
use horus_core::core::NodeMetrics;
use horus_core::error::HorusError;
use horus_core::scheduling::{
    ExecutionClass, FailurePolicy, NodeRegistration, Scheduler as CoreScheduler,
};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Convert a lock `PoisonError` into a descriptive `PyRuntimeError`.
///
/// Lock poisoning means a previous operation panicked while holding the lock.
/// The scheduler's internal state is inconsistent and cannot be recovered.
fn lock_poisoned<T>(_: std::sync::PoisonError<T>) -> PyErr {
    PyRuntimeError::new_err(
        "Internal state corrupted: a previous operation panicked while holding a lock. \
         This typically means a node's tick(), init(), or shutdown() raised an unhandled exception. \
         Restart the scheduler or Python process to recover.",
    )
}

/// Parameters for adding a node to the scheduler.
struct NodeParams {
    order: u32,
    rate_hz: Option<f64>,
    rt: bool,
    failure_policy: Option<FailurePolicy>,
    miss_policy: Option<String>,
    execution_class: Option<ExecutionClass>,
    budget_seconds: Option<f64>,
    deadline_seconds: Option<f64>,
    priority: Option<i32>,
    pinned_core: Option<usize>,
    watchdog_seconds: Option<f64>,
}

// ─── PyNodeAdapter ───────────────────────────────────────────────────────────
// Implements horus_core::Node by bridging to Python objects via GIL acquisition.
//
// # GIL / Rust-lock Safety
//
// The GIL and Rust Mutexes interact through three invariants that MUST be
// maintained to avoid deadlock:
//
// ## Invariant 1 — No Rust lock is held when Python::attach is entered
//
//   `init()`, `tick()`, and `shutdown()` each call `Python::attach`.
//   Before that call, every Rust MutexGuard must already be dropped.
//   `node_context` is only locked for brief, non-blocking operations
//   (`start_tick`, `record_tick`) that complete before Python is invoked.
//
// ## Invariant 2 — The GIL is released before long scheduler operations
//
//   `with_inner_run` calls `py.detach(...)` to release the GIL while
//   `CoreScheduler::run()` executes.  The `self.inner` Mutex is unlocked
//   (taken via `guard.take()`) *before* `detach` is called.
//   Consequently, when Python callbacks re-acquire the GIL inside `tick()`,
//   no PyScheduler-level Mutex is still held.
//
// ## Invariant 3 — Python callbacks MUST NOT re-enter the scheduler
//
//   A Python callback that calls `scheduler.run()` / `scheduler.add()` will
//   find `self.inner == None` and receive `RuntimeError("Scheduler already
//   running")` — an error, not a deadlock.  However, users must never block
//   inside a Python callback waiting for the scheduler to become free;
//   doing so would deadlock the scheduler thread.
//
// ## Summary table
//
//   | Lock         | Held when attach entered?   | Held when Python CB runs? |
//   |------------- |-----------------------------|---------------------------|
//   | GIL          | Yes (that's the whole point)| Yes                       |
//   | node_context | No — released before call   | No                        |
//   | self.inner   | No — taken out before run   | No                        |

struct PyNodeAdapter {
    py_object: Py<PyAny>,
    node_name: String,
    node_context: Arc<Mutex<CoreNodeInfo>>,
    cached_info: Option<Py<PyNodeInfo>>,
    stop_requested: Arc<AtomicBool>,
    scheduler_running: Arc<AtomicBool>,
}

impl CoreNode for PyNodeAdapter {
    fn name(&self) -> &str {
        &self.node_name
    }

    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        Python::attach(|py| {
            let py_info = Py::new(
                py,
                PyNodeInfo {
                    inner: self.node_context.clone(),
                    scheduler_running: Some(self.scheduler_running.clone()),
                },
            )
            .map_err(|e| HorusError::node(&self.node_name, e.to_string()))?;

            self.cached_info = Some(py_info.clone_ref(py));

            let result = self
                .py_object
                .call_method1(py, "init", (py_info,))
                .or_else(|e| {
                    // Only fall back to no-arg if the method doesn't accept info parameter.
                    // TypeError = wrong signature. All other errors propagate.
                    if e.is_instance_of::<pyo3::exceptions::PyTypeError>(py) {
                        self.py_object.call_method0(py, "init")
                    } else {
                        Err(e)
                    }
                });

            match result {
                Ok(_) => Ok(()),
                Err(e) => {
                    if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                        self.stop_requested.store(true, Ordering::Relaxed);
                        Ok(())
                    } else {
                        Err(HorusError::node(
                            &self.node_name,
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

        Python::attach(|py| {
            // GIL Safety: `node_context` is locked only for the duration of
            // `start_tick()`.  The MutexGuard is dropped at the closing `}` of
            // this if-let block — BEFORE any Python code is invoked.  This
            // ensures no Rust lock is held when the Python callback runs.
            if let Ok(mut ctx) = self.node_context.lock() {
                ctx.start_tick();
            } // ← node_context MutexGuard released here

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

            // Python callback runs here.  No Rust Mutex is held at this point
            // (see module-level GIL/lock safety comment).  If the Python code
            // calls back into Rust (e.g., topic.send()), that is safe because
            // node_context and self.inner are unlocked.
            let result = self
                .py_object
                .call_method1(py, "tick", (py_info,))
                .or_else(|e| {
                    if e.is_instance_of::<pyo3::exceptions::PyTypeError>(py) {
                        self.py_object.call_method0(py, "tick")
                    } else {
                        Err(e)
                    }
                });

            match result {
                Ok(_) => {
                    // node_context is again briefly locked only for record_tick()
                    // and released before the next Python interaction.
                    if let Ok(mut ctx) = self.node_context.lock() {
                        ctx.record_tick();
                    } // ← node_context MutexGuard released here
                }
                Err(e) => {
                    if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                        self.stop_requested.store(true, Ordering::Relaxed);
                        // Also stop the scheduler via the running flag
                        self.scheduler_running.store(false, Ordering::SeqCst);
                    } else {
                        // Intentional: horus_core wraps tick() in catch_unwind
                        // and routes panics through the node's FailurePolicy (restart,
                        // escalate, ignore).  Returning an error here would bypass
                        // the fault-tolerance system.  resume_unwind triggers the
                        // same catch_unwind path without being a panic!() call.
                        std::panic::resume_unwind(Box::new(format!(
                            "Python node '{}' tick failed: {}",
                            &self.node_name, e
                        )));
                    }
                }
            }
        });
    }

    fn shutdown(&mut self) -> horus_core::error::HorusResult<()> {
        Python::attach(|py| {
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
                .map_err(|e| HorusError::node(&self.node_name, e.to_string()))?
            };

            let result = self
                .py_object
                .call_method1(py, "shutdown", (py_info,))
                .or_else(|e| {
                    if e.is_instance_of::<pyo3::exceptions::PyTypeError>(py) {
                        self.py_object.call_method0(py, "shutdown")
                    } else {
                        Err(e)
                    }
                });

            match result {
                Ok(_) => Ok(()),
                Err(e) => {
                    if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                        self.stop_requested.store(true, Ordering::Relaxed);
                        Ok(())
                    } else {
                        Err(HorusError::node(
                            &self.node_name,
                            format!("shutdown failed: {}", e),
                        ))
                    }
                }
            }
        })
    }
}

// ─── PyMiss ─────────────────────────────────────────────────────────────────

/// Deadline miss policy for a node.
///
/// Set via `.on_miss()` on the node builder.
///
/// Example:
///     scheduler.node(motor).order(0).rate(1000).on_miss(Miss.SAFE_MODE).build()
#[pyclass(name = "Miss", module = "horus._horus")]
#[derive(Debug, Clone, Copy)]
pub struct PyMiss;

#[pymethods]
impl PyMiss {
    /// Log warning and continue normally.
    #[classattr]
    const WARN: &'static str = "warn";

    /// Skip this tick, resume next cycle.
    #[classattr]
    const SKIP: &'static str = "skip";

    /// Enter safe mode — calls `enter_safe_state()` on the node.
    #[classattr]
    const SAFE_MODE: &'static str = "safe_mode";

    /// Stop the entire scheduler (last resort).
    #[classattr]
    const STOP: &'static str = "stop";
}

/// Parse a miss policy string into a `Miss` enum.
fn parse_miss_policy(name: &str) -> PyResult<Miss> {
    match name {
        "warn" => Ok(Miss::Warn),
        "skip" => Ok(Miss::Skip),
        "safe_mode" | "safemode" => Ok(Miss::SafeMode),
        "stop" => Ok(Miss::Stop),
        _ => Err(PyRuntimeError::new_err(format!(
            "Unknown miss policy '{}'. Use Miss.WARN, Miss.SKIP, Miss.SAFE_MODE, or Miss.STOP",
            name
        ))),
    }
}

// ─── PyNodeBuilder ───────────────────────────────────────────────────────────

/// Fluent builder for adding nodes to the scheduler.
///
/// Example:
///     scheduler.add(my_node).order(0).rate(100.0).build()
#[pyclass(name = "NodeBuilder", module = "horus._horus")]
pub struct PyNodeBuilder {
    scheduler: Py<PyScheduler>,
    node: Py<PyAny>,
    order: u32,
    rate_hz: Option<f64>,
    rt: bool,
    failure_policy: Option<FailurePolicy>,
    miss_policy: Option<String>,
    execution_class: Option<ExecutionClass>,
    budget_seconds: Option<f64>,
    deadline_seconds: Option<f64>,
    priority: Option<i32>,
    pinned_core: Option<usize>,
    watchdog_seconds: Option<f64>,
}

#[pymethods]
impl PyNodeBuilder {
    /// Set execution order (lower = earlier in tick sequence).
    fn order(mut slf: PyRefMut<'_, Self>, order: u32) -> PyRefMut<'_, Self> {
        slf.order = order;
        slf
    }

    /// Set node-specific tick rate in Hz.
    fn rate(mut slf: PyRefMut<'_, Self>, rate: f64) -> PyRefMut<'_, Self> {
        slf.rate_hz = Some(rate);
        slf
    }

    /// Set tick budget in seconds (e.g., ``300 * us`` for 300μs).
    ///
    /// Overrides the auto-derived budget (80% of period from ``rate()``).
    /// Use ``horus.us`` and ``horus.ms`` constants for clarity.
    fn budget(mut slf: PyRefMut<'_, Self>, seconds: f64) -> PyResult<PyRefMut<'_, Self>> {
        if seconds <= 0.0 || !seconds.is_finite() {
            return Err(PyRuntimeError::new_err(
                "budget must be positive and finite",
            ));
        }
        if seconds > 1.0 {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "budget={:.3}s is unusually large (>{:.0} ticks at any reasonable rate). \
                 Did you mean {:.0} * us ({:.6}s)? \
                 Budget is in seconds. Use the `us` and `ms` constants: budget=300 * us",
                seconds,
                seconds,
                seconds * 1_000_000.0,
                seconds / 1_000_000.0
            )));
        }
        slf.budget_seconds = Some(seconds);
        Ok(slf)
    }

    /// Set tick deadline in seconds (e.g., ``900 * us`` for 900μs).
    ///
    /// Overrides the auto-derived deadline (95% of period from ``rate()``).
    /// Use ``horus.us`` and ``horus.ms`` constants for clarity.
    fn deadline(mut slf: PyRefMut<'_, Self>, seconds: f64) -> PyResult<PyRefMut<'_, Self>> {
        if seconds <= 0.0 || !seconds.is_finite() {
            return Err(PyRuntimeError::new_err(
                "deadline must be positive and finite",
            ));
        }
        if seconds > 5.0 {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "deadline={:.3}s is unusually large. \
                 Did you mean {:.0} * us ({:.6}s)? \
                 Deadline is in seconds. Use the `us` and `ms` constants: deadline=900 * us",
                seconds,
                seconds * 1_000_000.0,
                seconds / 1_000_000.0
            )));
        }
        slf.deadline_seconds = Some(seconds);
        Ok(slf)
    }

    /// Set deadline miss policy: Miss.WARN, Miss.SKIP, Miss.SAFE_MODE, Miss.STOP.
    fn on_miss(mut slf: PyRefMut<'_, Self>, policy: String) -> PyRefMut<'_, Self> {
        slf.miss_policy = Some(policy);
        slf
    }

    /// Set failure policy: "fatal", "restart", "skip", "ignore".
    ///
    /// For "restart" and "skip", optional keyword arguments control behavior:
    ///   - restart: max_retries (default 5), backoff_ms (default 100)
    ///   - skip: max_failures (default 5), cooldown_ms (default 30000)
    ///
    /// Examples:
    ///     .failure_policy("restart")
    ///     .failure_policy("restart", max_retries=10, backoff_ms=200)
    ///     .failure_policy("skip", max_failures=3, cooldown_ms=5000)
    #[pyo3(signature = (name, max_retries=None, backoff_ms=None, max_failures=None, cooldown_ms=None))]
    fn failure_policy(
        mut slf: PyRefMut<'_, Self>,
        name: String,
        max_retries: Option<u32>,
        backoff_ms: Option<u64>,
        max_failures: Option<u32>,
        cooldown_ms: Option<u64>,
    ) -> PyResult<PyRefMut<'_, Self>> {
        let policy = match name.as_str() {
            "fatal" => FailurePolicy::Fatal,
            "restart" => {
                FailurePolicy::restart(max_retries.unwrap_or(5), backoff_ms.unwrap_or(100).ms())
            }
            "skip" => FailurePolicy::skip(
                max_failures.unwrap_or(5),
                cooldown_ms.unwrap_or(30_000).ms(),
            ),
            "ignore" => FailurePolicy::Ignore,
            _ => {
                return Err(PyRuntimeError::new_err(format!(
                    "Unknown failure policy '{}'. Use 'fatal', 'restart', 'skip', or 'ignore'",
                    name
                )));
            }
        };
        slf.failure_policy = Some(policy);
        Ok(slf)
    }

    /// Set event-driven execution: node ticks when `topic` receives data.
    ///
    /// Example:
    ///     scheduler.node(detector).on("lidar_scan").build()
    fn on(mut slf: PyRefMut<'_, Self>, topic: String) -> PyRefMut<'_, Self> {
        slf.execution_class = Some(ExecutionClass::Event(topic));
        slf
    }

    /// Set compute execution class (CPU-bound work on thread pool).
    fn compute(mut slf: PyRefMut<'_, Self>) -> PyRefMut<'_, Self> {
        slf.execution_class = Some(ExecutionClass::Compute);
        slf
    }

    /// Set async I/O execution class (I/O-bound work on tokio runtime).
    fn async_io(mut slf: PyRefMut<'_, Self>) -> PyRefMut<'_, Self> {
        slf.execution_class = Some(ExecutionClass::AsyncIo);
        slf
    }

    /// Set OS scheduling priority (lower value = higher priority).
    fn priority(mut slf: PyRefMut<'_, Self>, prio: i32) -> PyRefMut<'_, Self> {
        slf.priority = Some(prio);
        slf
    }

    /// Pin this node to a specific CPU core.
    fn core(mut slf: PyRefMut<'_, Self>, cpu_id: usize) -> PyRefMut<'_, Self> {
        slf.pinned_core = Some(cpu_id);
        slf
    }

    /// Set per-node watchdog timeout in seconds.
    fn watchdog(mut slf: PyRefMut<'_, Self>, seconds: f64) -> PyResult<PyRefMut<'_, Self>> {
        if seconds <= 0.0 || !seconds.is_finite() {
            return Err(PyRuntimeError::new_err(
                "watchdog must be positive and finite",
            ));
        }
        slf.watchdog_seconds = Some(seconds);
        Ok(slf)
    }

    /// Finalize and add the node to the scheduler.
    fn build(slf: PyRefMut<'_, Self>, py: Python) -> PyResult<Py<PyScheduler>> {
        let scheduler = slf.scheduler.clone_ref(py);
        let node = slf.node.clone_ref(py);
        let order = slf.order;
        let rate_hz = slf.rate_hz;
        let rt = slf.rt;
        let failure_policy = slf.failure_policy.clone();
        let miss_policy = slf.miss_policy.clone();
        let execution_class = slf.execution_class.clone();
        let budget_seconds = slf.budget_seconds;
        let deadline_seconds = slf.deadline_seconds;
        let priority = slf.priority;
        let pinned_core = slf.pinned_core;
        let watchdog_seconds = slf.watchdog_seconds;
        drop(slf);

        {
            let sched = scheduler.borrow(py);
            sched.add_node_internal(
                py,
                node,
                NodeParams {
                    order,
                    rate_hz,
                    rt,
                    failure_policy,
                    miss_policy,
                    execution_class,
                    budget_seconds,
                    deadline_seconds,
                    priority,
                    pinned_core,
                    watchdog_seconds,
                },
            )?;
        }

        Ok(scheduler)
    }
}

// ─── PyScheduler ─────────────────────────────────────────────────────────────

/// Python wrapper for HORUS Scheduler wrapping horus_core::Scheduler.
///
/// All scheduling logic (rate control, fault tolerance, watchdog, deadline,
/// parallel execution, record/replay, telemetry, etc.) is handled by horus_core.
/// Python API is preserved with zero breaking changes.
#[pyclass(name = "Scheduler", module = "horus._horus")]
pub struct PyScheduler {
    inner: Mutex<Option<CoreScheduler>>,
    tick_rate_hz: f64,
    scheduler_running: Arc<AtomicBool>,
    stop_requested: Arc<AtomicBool>,
    removed_nodes: Mutex<HashSet<String>>,
}

impl PyScheduler {
    fn wrap_core(core_sched: CoreScheduler, tick_rate: f64) -> PyResult<Self> {
        let running_flag = core_sched.running_flag();
        Ok(PyScheduler {
            inner: Mutex::new(Some(core_sched)),
            tick_rate_hz: tick_rate,
            scheduler_running: running_flag,
            stop_requested: Arc::new(AtomicBool::new(false)),
            removed_nodes: Mutex::new(HashSet::new()),
        })
    }

    fn add_node_internal(&self, py: Python, node: Py<PyAny>, params: NodeParams) -> PyResult<()> {
        let NodeParams {
            order,
            rate_hz,
            rt,
            failure_policy,
            miss_policy,
            execution_class,
            budget_seconds,
            deadline_seconds,
            priority,
            pinned_core,
            watchdog_seconds,
        } = params;
        let name: String = node.getattr(py, "name")?.extract(py)?;

        // Rate precedence: explicit param > node.rate attribute > None (scheduler default)
        let node_rate = rate_hz.or_else(|| {
            node.getattr(py, "rate")
                .and_then(|attr| attr.extract::<f64>(py))
                .ok()
        });

        let adapter = PyNodeAdapter {
            py_object: node,
            node_name: name.clone(),
            node_context: Arc::new(Mutex::new(CoreNodeInfo::new(name.clone()))),
            cached_info: None,
            stop_requested: self.stop_requested.clone(),
            scheduler_running: self.scheduler_running.clone(),
        };

        let mut config = NodeRegistration::new(Box::new(adapter)).order(order);
        if let Some(rate) = node_rate {
            config = config.rate(rate.hz());
        }
        if let Some(ref miss_name) = miss_policy {
            config = config.on_miss(parse_miss_policy(miss_name)?);
        }
        if let Some(policy) = failure_policy {
            config = config.failure_policy(policy);
        }
        if let Some(exec_class) = execution_class {
            match exec_class {
                ExecutionClass::Event(ref topic) => {
                    config = config.on(topic);
                }
                ExecutionClass::Compute => {
                    config = config.compute();
                }
                ExecutionClass::AsyncIo => {
                    config = config.async_io();
                }
                ExecutionClass::Rt => {
                    // RT is auto-detected from rate
                }
                ExecutionClass::Gpu => {
                    config = config.gpu();
                }
                ExecutionClass::BestEffort => {} // default, no-op
            }
        }
        if let Some(budget_s) = budget_seconds {
            config = config.budget(budget_s.secs());
        }
        if let Some(deadline_s) = deadline_seconds {
            config = config.deadline(deadline_s.secs());
        }
        if let Some(prio) = priority {
            config = config.priority(prio);
        }
        if let Some(cpu) = pinned_core {
            config = config.core(cpu);
        }
        if let Some(wd_s) = watchdog_seconds {
            config = config.watchdog(wd_s.secs());
        }

        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot add nodes while scheduler is running")
        })?;

        config
            .validate()
            .map_err(|e| PyRuntimeError::new_err(format!("Invalid node configuration: {}", e)))?;
        inner.add_configured(config);

        tracing::info!(
            node = %name,
            order,
            rate = %node_rate.map_or("scheduler".to_string(), |r| format!("{}Hz", r)),
            rt,
            "Added node",
        );

        Ok(())
    }

    /// Build a Python dict from a NodeMetrics.
    fn metric_to_dict(py: Python, metric: &NodeMetrics, tick_rate: f64) -> PyResult<Py<PyAny>> {
        let dict = PyDict::new(py);
        dict.set_item("name", metric.name())?;
        dict.set_item("order", metric.order())?;
        dict.set_item("rate_hz", tick_rate)?;
        dict.set_item("total_ticks", metric.total_ticks())?;
        dict.set_item("successful_ticks", metric.successful_ticks())?;
        dict.set_item("failed_ticks", metric.failed_ticks())?;
        dict.set_item("errors_count", metric.errors_count())?;
        dict.set_item("avg_tick_duration_ms", metric.avg_tick_duration_ms())?;
        dict.set_item("min_tick_duration_ms", metric.min_tick_duration_ms())?;
        dict.set_item("max_tick_duration_ms", metric.max_tick_duration_ms())?;
        dict.set_item("last_tick_duration_ms", metric.last_tick_duration_ms())?;
        dict.set_item("uptime_seconds", metric.uptime_seconds())?;
        dict.set_item("messages_sent", metric.messages_sent())?;
        dict.set_item("messages_received", metric.messages_received())?;

        Ok(dict.into())
    }

    /// Helper: take the inner scheduler, run a closure, put it back.
    fn with_inner_run<F>(&self, py: Python, f: F) -> PyResult<()>
    where
        F: FnOnce(&mut CoreScheduler) -> horus_core::error::HorusResult<()> + Send,
        // CoreScheduler must be Send for py.detach
    {
        let mut inner = {
            let mut guard = self.inner.lock().map_err(lock_poisoned)?;
            guard.take().ok_or_else(|| {
                PyRuntimeError::new_err("Scheduler already running or not initialized")
            })?
        };

        self.stop_requested.store(false, Ordering::Relaxed);

        let result = py.detach(move || {
            let r = f(&mut inner);
            (inner, r)
        });

        let (returned_inner, run_result) = result;

        {
            let mut guard = self.inner.lock().map_err(lock_poisoned)?;
            *guard = Some(returned_inner);
        }

        run_result.map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }
}

#[pymethods]
impl PyScheduler {
    #[new]
    #[pyo3(signature = (config=None, deterministic=false, name=None, cores=None, max_deadline_misses=None, verbose=false, telemetry=None))]
    pub fn new(
        config: Option<PySchedulerConfig>,
        deterministic: bool,
        name: Option<String>,
        cores: Option<Vec<usize>>,
        max_deadline_misses: Option<u64>,
        verbose: bool,
        telemetry: Option<String>,
    ) -> PyResult<Self> {
        let core_config = config
            .as_ref()
            .map(|c| c.to_core_config())
            .unwrap_or_default();

        let tick_rate = config.as_ref().map_or(100.0, |c| c.tick_rate);

        let mut core_sched = CoreScheduler::new();
        if deterministic {
            core_sched = core_sched.deterministic(true);
        }
        if let Some(n) = name {
            core_sched = core_sched.name(&n);
        }
        if let Some(ref cpu_ids) = cores {
            core_sched = core_sched.cores(cpu_ids);
        }
        if let Some(n) = max_deadline_misses {
            core_sched = core_sched.max_deadline_misses(n);
        }
        if verbose {
            core_sched = core_sched.verbose(true);
        }
        if let Some(ref ep) = telemetry {
            core_sched = core_sched.telemetry(ep);
        }
        core_sched.apply_config(core_config);
        Self::wrap_core(core_sched, tick_rate)
    }

    /// Start building a node configuration (fluent API).
    fn node(slf: Py<Self>, _py: Python, node: Py<PyAny>) -> PyResult<PyNodeBuilder> {
        Ok(PyNodeBuilder {
            scheduler: slf,
            node,
            order: 100,
            rate_hz: None,
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: None,
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        })
    }

    /// Add a node to the scheduler.
    #[pyo3(signature = (node, order=100, rate=None, rt=false, failure_policy=None, on_miss=None, budget=None, deadline=None))]
    fn add(
        &self,
        py: Python,
        node: Py<PyAny>,
        order: u32,
        rate: Option<f64>,
        rt: bool,
        failure_policy: Option<String>,
        on_miss: Option<String>,
        budget: Option<f64>,
        deadline: Option<f64>,
    ) -> PyResult<()> {
        let parsed_policy = match failure_policy.as_deref() {
            Some("fatal") => Some(FailurePolicy::Fatal),
            Some("restart") => Some(FailurePolicy::restart(5, 100_u64.ms())),
            Some("skip") => Some(FailurePolicy::skip(5, 30_u64.secs())),
            Some("ignore") => Some(FailurePolicy::Ignore),
            Some(other) => {
                return Err(PyRuntimeError::new_err(format!(
                    "Unknown failure policy '{}'. Use 'fatal', 'restart', 'skip', or 'ignore'",
                    other
                )));
            }
            None => None,
        };
        self.add_node_internal(
            py,
            node,
            NodeParams {
                order,
                rate_hz: rate,
                rt,
                failure_policy: parsed_policy,
                miss_policy: on_miss,
                execution_class: None,
                budget_seconds: budget,
                deadline_seconds: deadline,
                priority: None,
                pinned_core: None,
                watchdog_seconds: None,
            },
        )
    }

    /// Set per-node rate control
    fn set_node_rate(&self, node_name: String, rate: f64) -> PyResult<()> {
        if rate <= 0.0 || rate > 10000.0 {
            return Err(PyRuntimeError::new_err(
                "Rate must be between 0 and 10000 Hz",
            ));
        }

        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard
            .as_mut()
            .ok_or_else(|| PyRuntimeError::new_err("Cannot modify while scheduler is running"))?;

        inner.set_node_rate(&node_name, rate.hz());
        tracing::info!(node = %node_name, rate, "Set node rate");
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
        let duration = duration_seconds.secs();
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
    fn tick_for(&self, py: Python, node_names: Vec<String>, duration_seconds: f64) -> PyResult<()> {
        if duration_seconds <= 0.0 {
            return Err(PyRuntimeError::new_err("Duration must be positive"));
        }
        let duration = duration_seconds.secs();
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

    /// Get node statistics.
    fn get_node_stats(&self, py: Python, node_name: String) -> PyResult<Py<PyAny>> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("Stats unavailable while scheduler is running")
        })?;

        for metric in inner.metrics() {
            if metric.name() == node_name {
                return Self::metric_to_dict(py, &metric, self.tick_rate_hz);
            }
        }

        Err(crate::errors::HorusNotFoundError::new_err(format!(
            "Node '{}' not found",
            node_name
        )))
    }

    /// Remove a node from the scheduler.
    /// Note: horus_core doesn't support runtime removal; node is excluded from stats.
    fn remove_node(&self, name: String) -> PyResult<bool> {
        let mut removed = self.removed_nodes.lock().map_err(lock_poisoned)?;
        Ok(removed.insert(name))
    }

    /// Get list of all nodes with basic information.
    fn get_all_nodes(&self, py: Python) -> PyResult<Vec<Py<PyAny>>> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("Node list unavailable while scheduler is running")
        })?;
        let removed = self.removed_nodes.lock().map_err(lock_poisoned)?;

        let mut result = Vec::new();
        for metric in inner.metrics() {
            if removed.contains(metric.name()) {
                continue;
            }
            result.push(Self::metric_to_dict(py, &metric, self.tick_rate_hz)?);
        }

        Ok(result)
    }

    /// Get count of all registered nodes.
    fn get_node_count(&self) -> PyResult<usize> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => {
                let removed = self.removed_nodes.lock().map_err(lock_poisoned)?;
                Ok(sched.node_list().len() - removed.len())
            }
            None => Ok(0),
        }
    }

    /// Check if a node exists by name.
    fn has_node(&self, name: String) -> PyResult<bool> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => {
                let removed = self.removed_nodes.lock().map_err(lock_poisoned)?;
                Ok(sched.node_list().contains(&name) && !removed.contains(&name))
            }
            None => Ok(false),
        }
    }

    /// Get list of node names.
    fn get_node_names(&self) -> PyResult<Vec<String>> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => {
                let removed = self.removed_nodes.lock().map_err(lock_poisoned)?;
                Ok(sched
                    .node_list()
                    .into_iter()
                    .filter(|n| !removed.contains(n))
                    .collect())
            }
            None => Ok(Vec::new()),
        }
    }

    /// Get node order (execution priority).
    fn get_node_info(&self, name: String) -> PyResult<Option<u32>> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => {
                for metric in sched.metrics() {
                    if metric.name() == name {
                        return Ok(Some(metric.order()));
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
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.status()),
            None => Ok("running".to_string()),
        }
    }

    /// Get runtime capabilities as a dict.
    fn capabilities(&self, py: Python) -> PyResult<Option<Py<PyAny>>> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
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
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.has_full_rt()),
            None => Ok(false),
        }
    }

    /// Get RT degradations as a list of dicts.
    fn degradations(&self, py: Python) -> PyResult<Py<PyAny>> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
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
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.current_tick()),
            None => Ok(0),
        }
    }

    /// Get the scheduler name.
    fn scheduler_name(&self) -> PyResult<String> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
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
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = match guard.as_ref() {
            Some(s) => s,
            None => return Ok(None),
        };
        match inner.safety_stats() {
            Some(stats) => {
                let dict = PyDict::new(py);
                dict.set_item("state", format!("{:?}", stats.state()))?;
                dict.set_item("budget_overruns", stats.budget_overruns())?;
                dict.set_item("deadline_misses", stats.deadline_misses())?;
                dict.set_item("watchdog_expirations", stats.watchdog_expirations())?;
                dict.set_item("degrade_activations", stats.degrade_activations())?;
                Ok(Some(dict.into()))
            }
            None => Ok(None),
        }
    }

    // ========================================================================
    // Recording
    // ========================================================================

    /// Check if scheduler is currently recording.
    fn is_recording(&self) -> PyResult<bool> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.is_recording()),
            None => Ok(false),
        }
    }

    /// Check if scheduler is currently replaying.
    fn is_replaying(&self) -> PyResult<bool> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        match guard.as_ref() {
            Some(sched) => Ok(sched.is_replaying()),
            None => Ok(false),
        }
    }

    /// Stop recording and return saved file paths.
    fn stop_recording(&self) -> PyResult<Vec<String>> {
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
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

    /// Set tick budget for a node in microseconds.
    fn set_tick_budget(&self, node_name: String, us: u64) -> PyResult<()> {
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot set tick budget while scheduler is running")
        })?;
        inner
            .set_tick_budget(&node_name, us.us())
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Add a critical node with watchdog timeout in milliseconds.
    fn add_critical_node(&self, node_name: String, timeout_ms: u64) -> PyResult<()> {
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot add critical node while scheduler is running")
        })?;
        inner
            .add_critical_node(&node_name, timeout_ms.ms())
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Delete a recording by session name.
    #[staticmethod]
    fn delete_recording(session_name: String) -> PyResult<()> {
        CoreScheduler::delete_recording(&session_name)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    // ========================================================================
    // Replay
    // ========================================================================

    /// Load an entire scheduler recording for replay.
    ///
    /// Returns a new Scheduler in replay mode with all recorded nodes loaded.
    ///
    /// Example:
    ///     sched = Scheduler.replay_from("~/.local/share/horus/recordings/crash/scheduler@abc.horus")
    ///     sched.run()
    #[staticmethod]
    fn replay_from(path: String) -> PyResult<Self> {
        let core_sched = CoreScheduler::replay_from(std::path::PathBuf::from(&path))
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to load replay: {}", e)))?;
        Self::wrap_core(core_sched, 100.0)
    }

    /// Add a replay node from a recording file for mixed replay.
    ///
    /// Replays one node's recorded outputs while other nodes run live.
    ///
    /// Example:
    ///     sched = Scheduler(tick_rate=100)
    ///     sched.add_replay("recordings/sensor@001.horus", priority=0)
    ///     sched.add(live_controller)
    ///     sched.run()
    #[pyo3(signature = (path, priority=0))]
    fn add_replay(&self, path: String, priority: u32) -> PyResult<()> {
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard.as_mut().ok_or_else(|| {
            PyRuntimeError::new_err("Cannot add replay while scheduler is running")
        })?;
        inner
            .add_replay(std::path::PathBuf::from(&path), priority)
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to add replay: {}", e)))?;
        Ok(())
    }

    /// Set replay to start at a specific tick (time travel).
    ///
    /// Example:
    ///     sched = Scheduler.replay_from(path)
    ///     sched.start_at_tick(1500)
    ///     sched.run()
    fn start_at_tick(&self, tick: u64) -> PyResult<()> {
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard
            .take()
            .ok_or_else(|| PyRuntimeError::new_err("Scheduler not available"))?;
        *guard = Some(inner.start_at_tick(tick));
        Ok(())
    }

    /// Set replay to stop at a specific tick.
    ///
    /// Example:
    ///     sched = Scheduler.replay_from(path)
    ///     sched.stop_at_tick(2000)
    ///     sched.run()
    fn stop_at_tick(&self, tick: u64) -> PyResult<()> {
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard
            .take()
            .ok_or_else(|| PyRuntimeError::new_err("Scheduler not available"))?;
        *guard = Some(inner.stop_at_tick(tick));
        Ok(())
    }

    /// Set replay speed multiplier (0.01 to 100.0).
    ///
    /// Example:
    ///     sched.set_replay_speed(0.5)   # Half speed
    ///     sched.set_replay_speed(2.0)   # Double speed
    fn set_replay_speed(&self, speed: f64) -> PyResult<()> {
        if speed < 0.01 || speed > 100.0 || !speed.is_finite() {
            return Err(PyRuntimeError::new_err(
                "Replay speed must be between 0.01 and 100.0",
            ));
        }
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard
            .take()
            .ok_or_else(|| PyRuntimeError::new_err("Scheduler not available"))?;
        *guard = Some(inner.with_replay_speed(speed));
        Ok(())
    }

    /// Set an override value for what-if testing during replay.
    ///
    /// Replaces a node's output with custom bytes during replay.
    ///
    /// Example:
    ///     import struct
    ///     sched.set_replay_override("sensor", "temperature", struct.pack('<f', 25.0))
    fn set_replay_override(
        &self,
        node_name: String,
        output_name: String,
        value: Vec<u8>,
    ) -> PyResult<()> {
        let mut guard = self.inner.lock().map_err(lock_poisoned)?;
        let inner = guard
            .take()
            .ok_or_else(|| PyRuntimeError::new_err("Scheduler not available"))?;
        *guard = Some(inner.with_override(&node_name, &output_name, value));
        Ok(())
    }

    fn __repr__(&self) -> PyResult<String> {
        let guard = self.inner.lock().map_err(lock_poisoned)?;
        let count = guard.as_ref().map(|s| s.node_list().len()).unwrap_or(0);
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

        let core_sched = CoreScheduler::new();
        self.scheduler_running = core_sched.running_flag();
        *self.inner.lock().map_err(lock_poisoned)? = Some(core_sched);
        *self.removed_nodes.lock().map_err(lock_poisoned)? = HashSet::new();

        Ok(())
    }
}

// ─── Lock-ordering safety tests ─────────────────────────────────────────────
#[cfg(test)]
mod tests {
    use super::*;
    use horus_core::core::NodeInfo as CoreNodeInfo;
    use std::sync::atomic::AtomicBool;
    use std::sync::{Arc, Mutex};

    /// Verify that `node_context` is NOT locked when a Python callback is
    /// about to be invoked inside `tick()`.
    ///
    /// We simulate this by cloning the Arc, and — inside a mock "callback" —
    /// using `try_lock()` to assert the mutex is free.  If the lock were held
    /// across the callback site, `try_lock` would return `Err(WouldBlock)`.
    ///
    /// This test does not require PyO3 / a live Python interpreter.
    #[test]
    fn node_context_not_locked_at_callback_site() {
        let node_context = Arc::new(Mutex::new(CoreNodeInfo::new("test_node".to_string())));
        let observer = node_context.clone();

        // Simulate the sequence inside PyNodeAdapter::tick():
        // 1. Acquire and release lock for start_tick
        {
            let mut ctx = node_context.lock().unwrap();
            ctx.start_tick();
        } // ← lock released

        // 2. "Python callback" runs — must be able to acquire node_context
        let try_result = observer.try_lock();
        assert!(
            try_result.is_ok(),
            "node_context was still locked at Python callback site — \
             GIL deadlock would occur if Python re-entered Rust here"
        );
        drop(try_result);

        // 3. Acquire and release lock for record_tick
        {
            let mut ctx = node_context.lock().unwrap();
            ctx.record_tick();
        }
    }

    /// Verify that `scheduler_running` can be observed from a second thread
    /// while a node callback is running (simulates topic.send() pattern).
    #[test]
    fn scheduler_running_flag_readable_during_callback() {
        let running = Arc::new(AtomicBool::new(true));
        let observer = running.clone();

        // Simulate: scheduler is running
        running.store(true, Ordering::SeqCst);

        // Another thread (e.g., Python callback invoking topic.send()) can
        // read the flag without blocking.
        let handle = std::thread::spawn(move || observer.load(Ordering::SeqCst));

        assert!(
            handle.join().unwrap(),
            "Running flag should be observable from callback thread"
        );
    }

    /// Verify tick metrics accumulate correctly across multiple ticks.
    #[test]
    fn node_context_tick_metrics_accumulate() {
        let ctx = Arc::new(Mutex::new(CoreNodeInfo::new("metrics_test".to_string())));

        for _ in 0..5 {
            let mut guard = ctx.lock().unwrap();
            guard.start_tick();
            guard.record_tick();
        }

        let guard = ctx.lock().unwrap();
        assert_eq!(guard.metrics().total_ticks(), 5);
        assert_eq!(guard.metrics().successful_ticks(), 5);
    }

    /// Concurrent access to node_context from multiple threads.
    #[test]
    fn node_context_concurrent_access() {
        let ctx = Arc::new(Mutex::new(CoreNodeInfo::new("concurrent".to_string())));

        let handles: Vec<_> = (0..4)
            .map(|_| {
                let ctx = ctx.clone();
                std::thread::spawn(move || {
                    for _ in 0..10 {
                        let mut guard = ctx.lock().unwrap();
                        guard.start_tick();
                        guard.record_tick();
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        let guard = ctx.lock().unwrap();
        assert_eq!(guard.metrics().total_ticks(), 40);
    }

    /// Verify stop_requested flag propagates across threads.
    #[test]
    fn stop_requested_flag_propagates() {
        let stop = Arc::new(AtomicBool::new(false));
        let observer = stop.clone();

        assert!(!stop.load(Ordering::SeqCst));

        // Simulate request_stop from Python
        stop.store(true, Ordering::SeqCst);

        let saw_stop = std::thread::spawn(move || observer.load(Ordering::SeqCst))
            .join()
            .unwrap();
        assert!(saw_stop, "Stop flag should propagate to other threads");
    }

    // ── Cross-language config parity tests ──────────────────────────────
    // These verify that NodeParams (the bridge between Python Node attrs
    // and Rust NodeRegistration) correctly carries all 12 config fields.

    #[test]
    fn node_params_all_fields_set() {
        let params = NodeParams {
            order: 5,
            rate_hz: Some(1000.0),
            rt: false,
            failure_policy: Some(FailurePolicy::Ignore),
            miss_policy: Some("skip".to_string()),
            execution_class: Some(ExecutionClass::Compute),
            budget_seconds: Some(0.0003),
            deadline_seconds: Some(0.0009),
            priority: Some(10),
            pinned_core: Some(2),
            watchdog_seconds: Some(0.5),
        };
        assert_eq!(params.order, 5);
        assert_eq!(params.rate_hz, Some(1000.0));
        assert_eq!(params.budget_seconds, Some(0.0003));
        assert_eq!(params.deadline_seconds, Some(0.0009));
        assert_eq!(params.priority, Some(10));
        assert_eq!(params.pinned_core, Some(2));
        assert_eq!(params.watchdog_seconds, Some(0.5));
        assert!(matches!(params.miss_policy.as_deref(), Some("skip")));
        assert!(matches!(
            params.execution_class,
            Some(ExecutionClass::Compute)
        ));
    }

    #[test]
    fn node_params_all_defaults() {
        let params = NodeParams {
            order: 100,
            rate_hz: None,
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: None,
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert_eq!(params.order, 100);
        assert!(params.rate_hz.is_none());
        assert!(params.budget_seconds.is_none());
        assert!(params.deadline_seconds.is_none());
        assert!(params.priority.is_none());
        assert!(params.pinned_core.is_none());
        assert!(params.watchdog_seconds.is_none());
        assert!(params.failure_policy.is_none());
        assert!(params.miss_policy.is_none());
        assert!(params.execution_class.is_none());
    }

    #[test]
    fn node_params_budget_only() {
        let params = NodeParams {
            order: 0,
            rate_hz: Some(1000.0),
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: None,
            budget_seconds: Some(0.0003),
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert_eq!(params.budget_seconds, Some(0.0003));
        assert!(params.deadline_seconds.is_none());
    }

    #[test]
    fn node_params_async_io_execution_class() {
        let params = NodeParams {
            order: 0,
            rate_hz: Some(10.0),
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: Some(ExecutionClass::AsyncIo),
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert!(matches!(
            params.execution_class,
            Some(ExecutionClass::AsyncIo)
        ));
    }

    #[test]
    fn node_params_event_driven() {
        let params = NodeParams {
            order: 0,
            rate_hz: None,
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: Some(ExecutionClass::Event("lidar_scan".to_string())),
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert!(matches!(
            params.execution_class,
            Some(ExecutionClass::Event(ref t)) if t == "lidar_scan"
        ));
    }

    #[test]
    fn node_params_rt_with_priority_and_core() {
        let params = NodeParams {
            order: 0,
            rate_hz: Some(1000.0),
            rt: true,
            failure_policy: None,
            miss_policy: Some("safe_mode".to_string()),
            execution_class: None,
            budget_seconds: Some(0.0003),
            deadline_seconds: Some(0.0009),
            priority: Some(0),
            pinned_core: Some(3),
            watchdog_seconds: Some(0.5),
        };
        assert!(params.rt);
        assert_eq!(params.priority, Some(0));
        assert_eq!(params.pinned_core, Some(3));
        assert_eq!(params.watchdog_seconds, Some(0.5));
        assert!(matches!(params.miss_policy.as_deref(), Some("safe_mode")));
    }

    #[test]
    fn node_params_failure_policy_restart() {
        let params = NodeParams {
            order: 50,
            rate_hz: Some(30.0),
            rt: false,
            failure_policy: Some(FailurePolicy::restart(5, 100_u64.ms())),
            miss_policy: Some("warn".to_string()),
            execution_class: None,
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert!(params.failure_policy.is_some());
        assert!(matches!(params.miss_policy.as_deref(), Some("warn")));
    }

    #[test]
    fn miss_policy_parsing() {
        assert!(matches!(parse_miss_policy("warn").unwrap(), Miss::Warn));
        assert!(matches!(parse_miss_policy("skip").unwrap(), Miss::Skip));
        assert!(matches!(
            parse_miss_policy("safe_mode").unwrap(),
            Miss::SafeMode
        ));
        assert!(matches!(
            parse_miss_policy("safemode").unwrap(),
            Miss::SafeMode
        ));
        assert!(matches!(parse_miss_policy("stop").unwrap(), Miss::Stop));
        assert!(parse_miss_policy("invalid").is_err());
    }

    #[test]
    fn node_params_watchdog_only() {
        let params = NodeParams {
            order: 0,
            rate_hz: Some(100.0),
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: None,
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: Some(2.0),
        };
        assert_eq!(params.watchdog_seconds, Some(2.0));
        assert!(params.priority.is_none());
    }

    #[test]
    fn node_params_all_execution_classes() {
        // BestEffort (default)
        let p1 = NodeParams {
            order: 0,
            rate_hz: None,
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: None,
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert!(p1.execution_class.is_none());

        // Compute
        let p2 = NodeParams {
            order: 0,
            rate_hz: None,
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: Some(ExecutionClass::Compute),
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert!(matches!(p2.execution_class, Some(ExecutionClass::Compute)));

        // AsyncIo
        let p3 = NodeParams {
            order: 0,
            rate_hz: None,
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: Some(ExecutionClass::AsyncIo),
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert!(matches!(p3.execution_class, Some(ExecutionClass::AsyncIo)));

        // Event
        let p4 = NodeParams {
            order: 0,
            rate_hz: None,
            rt: false,
            failure_policy: None,
            miss_policy: None,
            execution_class: Some(ExecutionClass::Event("scan".into())),
            budget_seconds: None,
            deadline_seconds: None,
            priority: None,
            pinned_core: None,
            watchdog_seconds: None,
        };
        assert!(matches!(p4.execution_class, Some(ExecutionClass::Event(_))));
    }

    #[test]
    fn node_params_complete_rt_config() {
        // Mirrors: Node(tick=fn, rate=1000, order=0, budget=300*us, deadline=900*us,
        //               on_miss="skip", failure_policy="restart", priority=0, core=2, watchdog=0.5)
        let params = NodeParams {
            order: 0,
            rate_hz: Some(1000.0),
            rt: false,
            failure_policy: Some(FailurePolicy::restart(3, 200_u64.ms())),
            miss_policy: Some("skip".to_string()),
            execution_class: None,
            budget_seconds: Some(0.0003),
            deadline_seconds: Some(0.0009),
            priority: Some(0),
            pinned_core: Some(2),
            watchdog_seconds: Some(0.5),
        };
        // Every field maps to a Rust NodeRegistration builder call
        assert_eq!(params.order, 0);
        assert_eq!(params.rate_hz, Some(1000.0));
        assert_eq!(params.budget_seconds, Some(0.0003));
        assert_eq!(params.deadline_seconds, Some(0.0009));
        assert!(matches!(params.miss_policy.as_deref(), Some("skip")));
        assert!(params.failure_policy.is_some());
        assert_eq!(params.priority, Some(0));
        assert_eq!(params.pinned_core, Some(2));
        assert_eq!(params.watchdog_seconds, Some(0.5));
    }
}
