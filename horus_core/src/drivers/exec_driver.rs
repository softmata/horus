//! Exec driver — wraps an external binary as a HORUS Node.
//!
//! Any program that publishes to HORUS SHM topics is a valid driver,
//! regardless of language (C++, Python, Go, shell script). The ExecDriver
//! launches it as a subprocess, monitors health, and handles crash recovery.

use std::collections::HashMap;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use super::params::NodeParams;

/// Configuration for an exec driver.
#[derive(Debug, Clone)]
pub struct ExecDriverConfig {
    /// Path to the binary to launch.
    pub path: PathBuf,
    /// CLI arguments to pass.
    pub args: Vec<String>,
    /// Extra environment variables (HORUS_PARAM_* for driver config).
    pub env: HashMap<String, String>,
    /// Maximum restart attempts before giving up. Default: 3.
    pub max_retries: u32,
    /// Base restart delay in milliseconds (doubles each retry). Default: 1000.
    pub restart_delay_ms: u64,
    /// Shutdown timeout in milliseconds before SIGKILL. Default: 5000.
    pub shutdown_timeout_ms: u64,
}

impl Default for ExecDriverConfig {
    fn default() -> Self {
        Self {
            path: PathBuf::new(),
            args: Vec::new(),
            env: HashMap::new(),
            max_retries: 3,
            restart_delay_ms: 1000,
            shutdown_timeout_ms: 5000,
        }
    }
}

/// Exec driver that wraps a subprocess as a HORUS Node.
///
/// Lifecycle:
/// - `init()`: launches the subprocess
/// - `tick()`: checks if process is alive, restarts on crash
/// - `shutdown()`: sends SIGTERM, waits, then SIGKILL
pub struct ExecDriver {
    config: ExecDriverConfig,
    child: Option<Child>,
    name: String,
    restart_count: u32,
    started: bool,
    /// When the next restart attempt becomes due. `tick()` used to `sleep()` the
    /// backoff inline, which blocks the whole executor thread — every RT node
    /// sharing that thread misses its deadline for the full delay, up to 30 s.
    /// The backoff is a deadline now, so `tick()` always returns promptly.
    next_restart_at: Option<Instant>,
}

impl ExecDriver {
    /// Create a new exec driver with the given name and config.
    pub fn new(name: impl Into<String>, config: ExecDriverConfig) -> Self {
        Self {
            config,
            child: None,
            name: name.into(),
            restart_count: 0,
            started: false,
            next_restart_at: None,
        }
    }

    /// Create from a path string, args, and NodeParams (parsed from horus.toml).
    pub fn from_config(
        path: &str,
        args: Vec<String>,
        params: &NodeParams,
    ) -> crate::error::Result<Self> {
        let max_retries = params.get_or("max_retries", 3u32);
        let restart_delay_ms = params.get_or("restart_delay_ms", 1000u64);
        let shutdown_timeout_ms = params.get_or("shutdown_timeout_ms", 5000u64);

        let mut env = HashMap::new();
        for key in params.keys() {
            if !matches!(
                key,
                "max_retries" | "restart_delay_ms" | "shutdown_timeout_ms"
            ) {
                // Convert any TOML value to string for env var
                let val = if let Ok(s) = params.get::<String>(key) {
                    Some(s)
                } else if let Ok(i) = params.get::<i64>(key) {
                    Some(i.to_string())
                } else if let Ok(f) = params.get::<f64>(key) {
                    Some(f.to_string())
                } else if let Ok(b) = params.get::<bool>(key) {
                    Some(b.to_string())
                } else {
                    // Arrays and complex types: use raw TOML representation
                    params.raw(key).map(|v| v.to_string())
                };
                if let Some(val) = val {
                    env.insert(format!("HORUS_PARAM_{}", key.to_uppercase()), val);
                }
            }
        }

        let name = std::path::Path::new(path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("exec_driver")
            .to_string();

        Ok(Self::new(
            &name,
            ExecDriverConfig {
                path: PathBuf::from(path),
                args,
                env,
                max_retries,
                restart_delay_ms,
                shutdown_timeout_ms,
            },
        ))
    }

    /// Create from NodeParams (parsed from horus.toml).
    pub fn from_params(name: &str, path: PathBuf, params: &NodeParams) -> Self {
        let args: Vec<String> = params.get::<Vec<String>>("args").unwrap_or_default();
        let max_retries = params.get_or("max_retries", 3u32);
        let restart_delay_ms = params.get_or("restart_delay_ms", 1000u64);
        let shutdown_timeout_ms = params.get_or("shutdown_timeout_ms", 5000u64);

        // Pass all driver params as HORUS_PARAM_* env vars
        let mut env = HashMap::new();
        env.insert("HORUS_DRIVER_NAME".to_string(), name.to_string());
        for key in params.keys() {
            if key != "args"
                && key != "max_retries"
                && key != "restart_delay_ms"
                && key != "shutdown_timeout_ms"
            {
                if let Ok(val) = params.get::<String>(key) {
                    env.insert(format!("HORUS_PARAM_{}", key.to_uppercase()), val);
                }
            }
        }

        Self::new(
            name,
            ExecDriverConfig {
                path,
                args,
                env,
                max_retries,
                restart_delay_ms,
                shutdown_timeout_ms,
            },
        )
    }

    /// Launch the subprocess.
    fn launch(&mut self) -> std::io::Result<()> {
        let mut cmd = Command::new(&self.config.path);
        cmd.args(&self.config.args)
            .envs(&self.config.env)
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit());

        self.child = Some(cmd.spawn()?);
        self.started = true;

        let pid = self.child.as_ref().map(|c| c.id()).unwrap_or(0);
        tracing::info!(
            "Exec driver '{}' launched (PID: {}, binary: {})",
            self.name,
            pid,
            self.config.path.display(),
        );
        Ok(())
    }

    /// Check if the subprocess is still running.
    fn check_health(&mut self) -> bool {
        if let Some(ref mut child) = self.child {
            match child.try_wait() {
                Ok(Some(status)) => {
                    tracing::warn!("Exec driver '{}' exited with {}", self.name, status);
                    self.child = None;
                    false
                }
                Ok(None) => true, // still running
                Err(e) => {
                    tracing::error!("Exec driver '{}' health check failed: {}", self.name, e);
                    false
                }
            }
        } else {
            false
        }
    }
}

impl crate::core::Node for ExecDriver {
    fn name(&self) -> &str {
        &self.name
    }

    fn init(&mut self) -> crate::error::Result<()> {
        self.launch().map_err(|e| {
            crate::error::Error::Config(crate::error::ConfigError::Other(format!(
                "exec driver '{}' failed to launch: {}",
                self.name, e
            )))
        })
    }

    fn tick(&mut self) {
        if !self.started {
            return;
        }
        if self.check_health() {
            return;
        }

        // A restart is already scheduled: return immediately until it is due.
        // This used to be `std::thread::sleep(delay)` right here, on the shared
        // executor thread, so a flapping device stalled every co-scheduled node
        // for up to 30 s at a time.
        if let Some(at) = self.next_restart_at {
            if Instant::now() < at {
                return;
            }
            self.next_restart_at = None;
            if let Err(e) = self.launch() {
                tracing::error!("Exec driver '{}' restart failed: {}", self.name, e);
            }
            return;
        }

        if self.restart_count < self.config.max_retries {
            self.restart_count += 1;
            // Clamp the shift BEFORE multiplying. `max_retries` and
            // `restart_delay_ms` both come straight from horus.toml, so the old
            // `restart_delay_ms * 2u64.pow(restart_count - 1)` overflowed for a
            // large-but-plausible `max_retries`: a panic inside tick() in debug,
            // and in release a wrap to ~0 that turned the backoff into an
            // unthrottled respawn loop — the exact opposite of the 30 s cap the
            // comment promised, because `.min(30_000)` was applied too late.
            let shift = (self.restart_count - 1).min(20);
            let delay = self
                .config
                .restart_delay_ms
                .saturating_mul(1u64 << shift)
                .min(30_000);
            tracing::warn!(
                "Exec driver '{}' crashed, restarting (attempt {}/{}, delay {}ms)",
                self.name,
                self.restart_count,
                self.config.max_retries,
                delay,
            );
            self.next_restart_at = Some(Instant::now() + Duration::from_millis(delay));
        }
    }

    fn shutdown(&mut self) -> crate::error::Result<()> {
        // No pending restart survives shutdown.
        self.next_restart_at = None;
        if let Some(ref mut child) = self.child {
            tracing::info!(
                "Shutting down exec driver '{}' (PID: {})...",
                self.name,
                child.id()
            );

            // Send SIGTERM (Unix) or kill (Windows)
            #[cfg(unix)]
            {
                unsafe {
                    libc::kill(child.id() as i32, libc::SIGTERM);
                }
            }
            #[cfg(not(unix))]
            {
                let _ = child.kill();
            }

            // Wait for graceful shutdown with timeout
            let deadline = Instant::now() + Duration::from_millis(self.config.shutdown_timeout_ms);
            loop {
                match child.try_wait() {
                    Ok(Some(_)) => break,
                    Ok(None) => {
                        if Instant::now() > deadline {
                            tracing::warn!(
                                "Exec driver '{}' didn't exit in time, force killing",
                                self.name
                            );
                            let _ = child.kill();
                            let _ = child.wait();
                            break;
                        }
                        std::thread::sleep(Duration::from_millis(50));
                    }
                    Err(_) => break,
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Node;

    #[test]
    fn test_exec_driver_config_defaults() {
        let config = ExecDriverConfig::default();
        assert_eq!(config.max_retries, 3);
        assert_eq!(config.restart_delay_ms, 1000);
        assert_eq!(config.shutdown_timeout_ms, 5000);
        assert!(config.args.is_empty());
        assert!(config.env.is_empty());
    }

    #[test]
    fn test_exec_driver_new() {
        let config = ExecDriverConfig {
            path: PathBuf::from("./my_driver"),
            args: vec!["--port".to_string(), "8080".to_string()],
            ..Default::default()
        };
        let driver = ExecDriver::new("test_driver", config);
        assert_eq!(driver.name(), "test_driver");
        assert!(!driver.started);
        assert_eq!(driver.restart_count, 0);
    }

    #[test]
    fn test_exec_driver_from_params() {
        let mut params_map = HashMap::new();
        params_map.insert(
            "port".to_string(),
            toml::Value::String("/dev/ttyUSB0".to_string()),
        );
        params_map.insert("max_retries".to_string(), toml::Value::Integer(5));
        let params = NodeParams::new(params_map);

        let driver = ExecDriver::from_params("lidar", PathBuf::from("./lidar_driver"), &params);
        assert_eq!(driver.name(), "lidar");
        assert_eq!(driver.config.max_retries, 5);
        assert!(driver.config.env.contains_key("HORUS_PARAM_PORT"));
    }

    #[test]
    fn test_exec_driver_health_check_no_child() {
        let config = ExecDriverConfig::default();
        let mut driver = ExecDriver::new("test", config);
        assert!(!driver.check_health());
    }

    #[test]
    fn test_exec_driver_restart_count_limits() {
        let config = ExecDriverConfig {
            max_retries: 2,
            ..Default::default()
        };
        let mut driver = ExecDriver::new("test", config);
        driver.started = true;
        // Simulate crash detection without real subprocess
        assert_eq!(driver.restart_count, 0);
        driver.restart_count = 2;
        // At max retries, tick should not increment further
        assert!(driver.restart_count >= driver.config.max_retries);
    }

    /// Regression: `tick()` must never sleep the executor thread, and the 30 s
    /// cap must hold for any `max_retries` an operator can put in horus.toml.
    /// The old code computed `restart_delay_ms * 2u64.pow(restart_count - 1)`
    /// and only THEN applied `.min(30_000)`, so the multiply overflowed (panic
    /// in debug, wrap-to-zero in release) well before the cap could apply.
    #[test]
    fn restart_backoff_is_non_blocking_and_cannot_overflow() {
        let config = ExecDriverConfig {
            path: PathBuf::from("/nonexistent/horus_exec_driver_test_binary"),
            max_retries: 100,
            restart_delay_ms: 1000,
            ..Default::default()
        };
        let mut driver = ExecDriver::new("test", config);
        driver.started = true;
        // 1000 * 2^79 in the old expression — far past u64::MAX.
        driver.restart_count = 80;

        let start = Instant::now();
        driver.tick();
        assert!(
            start.elapsed() < Duration::from_millis(100),
            "tick must schedule the backoff, not sleep through it"
        );
        assert_eq!(driver.restart_count, 81);
        let at = driver
            .next_restart_at
            .expect("a crashed driver must have a restart scheduled");
        assert!(
            at <= Instant::now() + Duration::from_millis(30_000),
            "backoff must stay capped at 30s regardless of retry count"
        );

        // Not yet due: the next tick returns without stacking another attempt.
        driver.tick();
        assert_eq!(
            driver.restart_count, 81,
            "a pending restart must not accumulate further attempts"
        );
    }
}
