//! Coverage gap tests for presence and service resilient calls.
//!
//! Tests untested accessors on NodePresence (health_status, tick_count,
//! error_count, services, actions) and ServiceClient::call_resilient /
//! call_resilient_with.

#[cfg(test)]
mod tests {
    use horus_core::core::presence::NodePresence;
    use horus_core::core::DurationExt;
    
    use horus_core::error::RetryConfig;
    use horus_core::service;
    use horus_core::services::{ServiceClient, ServiceError, ServiceServerBuilder};

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 1: NodePresence accessor tests (via JSON deserialization)
    // ═══════════════════════════════════════════════════════════════════════════
    //
    // NodePresence::new is pub(crate), so integration tests construct via
    // serde_json deserialization to test the public accessor surface.

    fn make_presence_json(
        health_status: Option<&str>,
        tick_count: u64,
        error_count: u32,
        services: &[&str],
        actions: &[&str],
    ) -> String {
        let health = match health_status {
            Some(s) => format!("\"{}\"", s),
            None => "null".to_string(),
        };
        let svc: Vec<String> = services.iter().map(|s| format!("\"{}\"", s)).collect();
        let act: Vec<String> = actions.iter().map(|s| format!("\"{}\"", s)).collect();
        format!(
            r#"{{
                "name": "test_node",
                "pid": 12345,
                "scheduler": "default",
                "publishers": [],
                "subscribers": [],
                "start_time": 1000000,
                "priority": 10,
                "rate_hz": 100.0,
                "pid_start_time": 0,
                "health_status": {},
                "tick_count": {},
                "error_count": {},
                "services": [{}],
                "actions": [{}]
            }}"#,
            health,
            tick_count,
            error_count,
            svc.join(", "),
            act.join(", "),
        )
    }

    #[test]
    fn test_presence_health_status_some() {
        let json = make_presence_json(Some("Healthy"), 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.health_status(), Some("Healthy"));
    }

    #[test]
    fn test_presence_health_status_none() {
        let json = make_presence_json(None, 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.health_status(), None);
    }

    #[test]
    fn test_presence_health_status_warning() {
        let json = make_presence_json(Some("Warning"), 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.health_status(), Some("Warning"));
    }

    #[test]
    fn test_presence_health_status_critical() {
        let json = make_presence_json(Some("Critical"), 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.health_status(), Some("Critical"));
    }

    #[test]
    fn test_presence_tick_count_zero() {
        let json = make_presence_json(None, 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.tick_count(), 0);
    }

    #[test]
    fn test_presence_tick_count_large() {
        let json = make_presence_json(None, 999_999_999, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.tick_count(), 999_999_999);
    }

    #[test]
    fn test_presence_tick_count_max() {
        let json = make_presence_json(None, u64::MAX, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.tick_count(), u64::MAX);
    }

    #[test]
    fn test_presence_error_count_zero() {
        let json = make_presence_json(None, 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.error_count(), 0);
    }

    #[test]
    fn test_presence_error_count_nonzero() {
        let json = make_presence_json(None, 0, 42, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.error_count(), 42);
    }

    #[test]
    fn test_presence_error_count_max() {
        let json = make_presence_json(None, 0, u32::MAX, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.error_count(), u32::MAX);
    }

    #[test]
    fn test_presence_services_empty() {
        let json = make_presence_json(None, 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert!(presence.services().is_empty());
    }

    #[test]
    fn test_presence_services_populated() {
        let json = make_presence_json(None, 0, 0, &["set_params", "get_status"], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.services().len(), 2);
        assert_eq!(presence.services()[0], "set_params");
        assert_eq!(presence.services()[1], "get_status");
    }

    #[test]
    fn test_presence_actions_empty() {
        let json = make_presence_json(None, 0, 0, &[], &[]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert!(presence.actions().is_empty());
    }

    #[test]
    fn test_presence_actions_populated() {
        let json = make_presence_json(None, 0, 0, &[], &["navigate", "pick_place"]);
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.actions().len(), 2);
        assert_eq!(presence.actions()[0], "navigate");
        assert_eq!(presence.actions()[1], "pick_place");
    }

    #[test]
    fn test_presence_all_fields_populated() {
        let json = make_presence_json(
            Some("Error"),
            50000,
            17,
            &["reset", "calibrate"],
            &["move_to", "grasp"],
        );
        let presence: NodePresence = serde_json::from_str(&json).unwrap();
        assert_eq!(presence.health_status(), Some("Error"));
        assert_eq!(presence.tick_count(), 50000);
        assert_eq!(presence.error_count(), 17);
        assert_eq!(presence.services(), &["reset", "calibrate"]);
        assert_eq!(presence.actions(), &["move_to", "grasp"]);
        // Also verify existing accessors still work
        assert_eq!(presence.name(), "test_node");
        assert_eq!(presence.pid(), 12345);
        assert_eq!(presence.priority(), 10);
    }

    #[test]
    fn test_presence_defaults_when_fields_missing() {
        // Simulate an old-format presence file that lacks the new fields.
        // serde(default) should fill in zeros/empty vecs.
        let json = r#"{
            "name": "legacy_node",
            "pid": 1,
            "scheduler": null,
            "publishers": [],
            "subscribers": [],
            "start_time": 1000,
            "priority": 0,
            "rate_hz": null,
            "pid_start_time": 0
        }"#;
        let presence: NodePresence = serde_json::from_str(json).unwrap();
        assert_eq!(presence.health_status(), None);
        assert_eq!(presence.tick_count(), 0);
        assert_eq!(presence.error_count(), 0);
        assert!(presence.services().is_empty());
        assert!(presence.actions().is_empty());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 2: ServiceClient::call_resilient / call_resilient_with tests
    // ═══════════════════════════════════════════════════════════════════════════

    // Unique service types for resilient call tests (avoid topic collisions)
    service! { CgResilientBasic     { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { CgResilientCustom    { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { CgResilientTimeout   { request { }                response { } } }
    service! { CgResilientPermErr   { request { should_fail: bool, value: i32 } response { result: i32 } } }
    service! { CgResilientRetry     { request { a: i64, b: i64 } response { sum: i64 } } }

    #[test]
    fn test_call_resilient_success() {
        let _server = ServiceServerBuilder::<CgResilientBasic>::new()
            .on_request(|req| Ok(CgResilientBasicResponse { sum: req.a + req.b }))
            .poll_interval(1_u64.ms())
            .build()
            .unwrap();

        std::thread::sleep(20_u64.ms());

        let mut client = ServiceClient::<CgResilientBasic>::new().unwrap();
        let resp = client.call_resilient(CgResilientBasicRequest { a: 10, b: 20 }, 2_u64.secs());

        assert!(resp.is_ok(), "call_resilient should succeed: {:?}", resp);
        assert_eq!(resp.unwrap().sum, 30);
    }

    #[test]
    fn test_call_resilient_with_custom_config() {
        let _server = ServiceServerBuilder::<CgResilientCustom>::new()
            .on_request(|req| Ok(CgResilientCustomResponse { sum: req.a + req.b }))
            .poll_interval(1_u64.ms())
            .build()
            .unwrap();

        std::thread::sleep(20_u64.ms());

        let config = RetryConfig::new(5, 5_u64.ms());
        let mut client = ServiceClient::<CgResilientCustom>::new().unwrap();
        let resp = client.call_resilient_with(
            CgResilientCustomRequest { a: 7, b: 8 },
            2_u64.secs(),
            config,
        );

        assert!(
            resp.is_ok(),
            "call_resilient_with should succeed: {:?}",
            resp
        );
        assert_eq!(resp.unwrap().sum, 15);
    }

    #[test]
    fn test_call_resilient_timeout_no_server() {
        // No server running — call_resilient should eventually time out
        // after retrying. Use short timeout + minimal retries to keep fast.
        let config = RetryConfig::new(1, 5_u64.ms());
        let mut client = ServiceClient::<CgResilientTimeout>::new().unwrap();
        let result = client.call_resilient_with(CgResilientTimeoutRequest {}, 30_u64.ms(), config);

        assert!(
            matches!(result, Err(ServiceError::Timeout)),
            "expected Timeout after retries, got {:?}",
            result
        );
    }

    #[test]
    fn test_call_resilient_permanent_error_not_retried() {
        // ServiceFailed is a permanent error — should propagate immediately
        // without retrying.
        let _server = ServiceServerBuilder::<CgResilientPermErr>::new()
            .on_request(|req| {
                if req.should_fail {
                    Err("permanent failure".to_string())
                } else {
                    Ok(CgResilientPermErrResponse {
                        result: req.value * 2,
                    })
                }
            })
            .poll_interval(1_u64.ms())
            .build()
            .unwrap();

        std::thread::sleep(20_u64.ms());

        let config = RetryConfig::new(5, 5_u64.ms());
        let mut client = ServiceClient::<CgResilientPermErr>::new().unwrap();
        let result = client.call_resilient_with(
            CgResilientPermErrRequest {
                should_fail: true,
                value: 0,
            },
            2_u64.secs(),
            config,
        );

        assert!(result.is_err());
        match result {
            Err(ServiceError::ServiceFailed(msg)) => {
                assert!(
                    msg.contains("permanent failure"),
                    "expected permanent failure message, got: {}",
                    msg
                );
            }
            other => panic!("expected ServiceFailed, got {:?}", other),
        }
    }

    #[test]
    fn test_call_resilient_default_config() {
        // Verify call_resilient uses default RetryConfig (3 retries, 10ms backoff).
        // With a running server, it should succeed on first attempt.
        let _server = ServiceServerBuilder::<CgResilientRetry>::new()
            .on_request(|req| Ok(CgResilientRetryResponse { sum: req.a + req.b }))
            .poll_interval(1_u64.ms())
            .build()
            .unwrap();

        std::thread::sleep(20_u64.ms());

        let mut client = ServiceClient::<CgResilientRetry>::new().unwrap();
        let resp = client.call_resilient(CgResilientRetryRequest { a: 100, b: 200 }, 2_u64.secs());

        assert!(resp.is_ok());
        assert_eq!(resp.unwrap().sum, 300);
    }

    #[test]
    fn test_retry_config_accessors() {
        let config = RetryConfig::new(5, 50_u64.ms())
            .with_max_backoff(500_u64.ms())
            .with_multiplier(3.0);

        assert_eq!(config.max_retries(), 5);
        assert_eq!(config.initial_backoff(), 50_u64.ms());
        assert_eq!(config.max_backoff(), 500_u64.ms());
        assert!((config.backoff_multiplier() - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_retry_config_default() {
        let config = RetryConfig::default();
        assert_eq!(config.max_retries(), 3);
        assert_eq!(config.initial_backoff(), 10_u64.ms());
        assert!((config.backoff_multiplier() - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_service_error_is_transient() {
        assert!(ServiceError::Timeout.is_transient());
        assert!(ServiceError::Transport("err".to_string()).is_transient());
        assert!(!ServiceError::ServiceFailed("err".to_string()).is_transient());
        assert!(!ServiceError::NoServer.is_transient());
    }
}
