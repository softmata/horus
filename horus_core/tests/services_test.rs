//! Comprehensive tests for the HORUS service system.
//!
//! Covers: service!() macro, ServiceRequest/ServiceResponse types,
//! ServiceClient, AsyncServiceClient, ServiceServerBuilder, and full
//! client-server roundtrip scenarios.

#[cfg(test)]
mod tests {
    use horus_core::service;
    use horus_core::services::{
        AsyncServiceClient, ServiceClient, ServiceError, ServiceInfo, ServiceRequest,
        ServiceResponse, ServiceServerBuilder,
    };
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;
    use std::time::Duration;

    // ─── Test service definitions ───────────────────────────────────────────

    // We define multiple services using the service!() macro.
    // Each test uses a unique topic name by constructing services with
    // runtime-unique names to prevent cross-test interference.

    // Each service used in roundtrip tests gets a UNIQUE name to prevent
    // topic collisions when tests run in parallel. Services used only for
    // struct/type tests can share names since they never create topics.

    service! {
        /// Simple addition service for type-level / macro tests only.
        AddTwoInts {
            request { a: i64, b: i64 }
            response { sum: i64 }
        }
    }

    service! {
        /// Service with empty fields (edge case).
        PingMacroTest {
            request { }
            response { }
        }
    }

    // ─── Unique services for roundtrip tests ─────────────────────────────────
    // Each roundtrip test needs its own service type to avoid topic name conflicts.

    service! { RtAddBasic        { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtEcho            { request { message: String } response { echoed: String } } }
    service! { RtMultiCall       { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtFallible        { request { should_fail: bool, value: i32 } response { result: i32 } } }
    service! { RtCallOptional    { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtCallOptErr      { request { should_fail: bool, value: i32 } response { result: i32 } } }
    service! { RtAsyncWait       { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtAsyncPoll       { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtNoHandler       { request { } response { } } }
    service! { RtDropServer      { request { } response { } } }
    service! { RtLargePayload    { request { message: String } response { echoed: String } } }
    service! { RtNegative        { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtSharedState     { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtAsyncTimeout    { request { } response { } } }
    service! { RtAsyncExpired    { request { } response { } } }
    service! { RtAsyncWaitTo     { request { } response { } } }
    service! { RtClientTimeout   { request { } response { } } }
    service! { RtClientOptTo     { request { } response { } } }
    service! { RtClientCreate    { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtClientPoll      { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtAsyncCreate     { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtAsyncCreatePoll { request { message: String } response { echoed: String } } }
    service! { RtServerDefault   { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtServerHandler   { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtServerPoll      { request { a: i64, b: i64 } response { sum: i64 } } }
    service! { RtServerStop      { request { message: String } response { echoed: String } } }
    service! { RtServerDrop      { request { } response { } } }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 1: service!() macro and type generation tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_service_macro_generates_request_struct() {
        let req = AddTwoIntsRequest { a: 10, b: 20 };
        assert_eq!(req.a, 10);
        assert_eq!(req.b, 20);
    }

    #[test]
    fn test_service_macro_generates_response_struct() {
        let resp = AddTwoIntsResponse { sum: 42 };
        assert_eq!(resp.sum, 42);
    }

    #[test]
    fn test_service_macro_generates_service_trait_impl() {
        use horus_core::services::Service;

        // name() should convert CamelCase to snake_case
        assert_eq!(AddTwoInts::name(), "add_two_ints");
        assert_eq!(RtEcho::name(), "rt_echo");
        assert_eq!(RtFallible::name(), "rt_fallible");
        assert_eq!(PingMacroTest::name(), "ping_macro_test");
    }

    #[test]
    fn test_service_macro_topic_names() {
        use horus_core::services::Service;

        assert_eq!(AddTwoInts::request_topic(), "add_two_ints/request");
        assert_eq!(AddTwoInts::response_topic(), "add_two_ints/response");
    }

    #[test]
    fn test_service_macro_type_names() {
        use horus_core::services::Service;

        let req_type = AddTwoInts::request_type_name();
        let res_type = AddTwoInts::response_type_name();
        assert!(req_type.contains("AddTwoIntsRequest"));
        assert!(res_type.contains("AddTwoIntsResponse"));
    }

    #[test]
    fn test_service_macro_request_clone_and_debug() {
        let req = AddTwoIntsRequest { a: 1, b: 2 };
        let cloned = req.clone();
        assert_eq!(cloned.a, 1);
        assert_eq!(cloned.b, 2);

        let debug = format!("{:?}", req);
        assert!(debug.contains("AddTwoIntsRequest"));
        assert!(debug.contains("1"));
        assert!(debug.contains("2"));
    }

    #[test]
    fn test_service_macro_serde_roundtrip_request() {
        let req = AddTwoIntsRequest { a: 100, b: -50 };
        let serialized = serde_json::to_string(&req).unwrap();
        let deserialized: AddTwoIntsRequest = serde_json::from_str(&serialized).unwrap();
        assert_eq!(deserialized.a, 100);
        assert_eq!(deserialized.b, -50);
    }

    #[test]
    fn test_service_macro_serde_roundtrip_response() {
        let resp = AddTwoIntsResponse { sum: 42 };
        let serialized = serde_json::to_string(&resp).unwrap();
        let deserialized: AddTwoIntsResponse = serde_json::from_str(&serialized).unwrap();
        assert_eq!(deserialized.sum, 42);
    }

    #[test]
    fn test_service_macro_string_fields() {
        let req = RtEchoRequest {
            message: "hello world".to_string(),
        };
        let serialized = serde_json::to_string(&req).unwrap();
        let deserialized: RtEchoRequest = serde_json::from_str(&serialized).unwrap();
        assert_eq!(deserialized.message, "hello world");
    }

    #[test]
    fn test_service_macro_empty_fields() {
        // PingMacroTest has empty request/response — should compile and work
        let _req = PingMacroTestRequest {};
        let _resp = PingMacroTestResponse {};
        let debug = format!("{:?}", _req);
        assert!(debug.contains("PingMacroTestRequest"));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 2: ServiceRequest / ServiceResponse wire type tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_service_request_fields() {
        let req = ServiceRequest {
            request_id: 42,
            payload: AddTwoIntsRequest { a: 1, b: 2 },
        };
        assert_eq!(req.request_id, 42);
        assert_eq!(req.payload.a, 1);
        assert_eq!(req.payload.b, 2);
    }

    #[test]
    fn test_service_request_clone() {
        let req = ServiceRequest {
            request_id: 1,
            payload: "hello".to_string(),
        };
        let cloned = req.clone();
        assert_eq!(cloned.request_id, 1);
        assert_eq!(cloned.payload, "hello");
    }

    #[test]
    fn test_service_request_serde_roundtrip() {
        let req = ServiceRequest {
            request_id: 99,
            payload: AddTwoIntsRequest { a: 7, b: 8 },
        };
        let bytes = serde_json::to_vec(&req).unwrap();
        let decoded: ServiceRequest<AddTwoIntsRequest> = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded.request_id, 99);
        assert_eq!(decoded.payload.a, 7);
        assert_eq!(decoded.payload.b, 8);
    }

    #[test]
    fn test_service_response_success() {
        let resp = ServiceResponse::success(10, AddTwoIntsResponse { sum: 42 });
        assert_eq!(resp.request_id, 10);
        assert!(resp.ok);
        assert!(resp.payload.is_some());
        assert_eq!(resp.payload.unwrap().sum, 42);
        assert!(resp.error.is_none());
    }

    #[test]
    fn test_service_response_failure() {
        let resp = ServiceResponse::<AddTwoIntsResponse>::failure(10, "division by zero");
        assert_eq!(resp.request_id, 10);
        assert!(!resp.ok);
        assert!(resp.payload.is_none());
        assert_eq!(resp.error.as_deref(), Some("division by zero"));
    }

    #[test]
    fn test_service_response_serde_roundtrip_success() {
        let resp = ServiceResponse::success(5, "result".to_string());
        let bytes = serde_json::to_vec(&resp).unwrap();
        let decoded: ServiceResponse<String> = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded.request_id, 5);
        assert!(decoded.ok);
        assert_eq!(decoded.payload.as_deref(), Some("result"));
    }

    #[test]
    fn test_service_response_serde_roundtrip_failure() {
        let resp = ServiceResponse::<String>::failure(5, "boom");
        let bytes = serde_json::to_vec(&resp).unwrap();
        let decoded: ServiceResponse<String> = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded.request_id, 5);
        assert!(!decoded.ok);
        assert!(decoded.payload.is_none());
        assert_eq!(decoded.error.as_deref(), Some("boom"));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 3: ServiceError tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_service_error_timeout_display() {
        let err = ServiceError::Timeout;
        assert_eq!(format!("{}", err), "service call timed out");
    }

    #[test]
    fn test_service_error_service_failed_display() {
        let err = ServiceError::ServiceFailed("handler crashed".to_string());
        assert!(format!("{}", err).contains("handler crashed"));
    }

    #[test]
    fn test_service_error_no_server_display() {
        let err = ServiceError::NoServer;
        assert!(format!("{}", err).contains("no server"));
    }

    #[test]
    fn test_service_error_transport_display() {
        let err = ServiceError::Transport("connection lost".to_string());
        assert!(format!("{}", err).contains("connection lost"));
    }

    #[test]
    fn test_service_error_is_std_error() {
        let err = ServiceError::Timeout;
        let _: &dyn std::error::Error = &err;
    }

    #[test]
    fn test_service_error_clone() {
        let err = ServiceError::ServiceFailed("oops".to_string());
        let cloned = err.clone();
        assert_eq!(format!("{}", cloned), format!("{}", err));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 4: ServiceInfo metadata tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_service_info_creation() {
        let info = ServiceInfo {
            name: "add_two_ints".to_string(),
            request_type: "AddTwoIntsRequest".to_string(),
            response_type: "AddTwoIntsResponse".to_string(),
            servers: 1,
            clients: 3,
        };
        assert_eq!(info.name, "add_two_ints");
        assert_eq!(info.servers, 1);
        assert_eq!(info.clients, 3);
    }

    #[test]
    fn test_service_info_serde_roundtrip() {
        let info = ServiceInfo {
            name: "echo".to_string(),
            request_type: "EchoRequest".to_string(),
            response_type: "EchoResponse".to_string(),
            servers: 0,
            clients: 2,
        };
        let json = serde_json::to_string(&info).unwrap();
        let decoded: ServiceInfo = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.name, "echo");
        assert_eq!(decoded.servers, 0);
        assert_eq!(decoded.clients, 2);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 5: ServiceClient creation tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_service_client_creation() {
        let client = ServiceClient::<RtClientCreate>::new();
        assert!(client.is_ok(), "ServiceClient::new() should succeed");
    }

    #[test]
    fn test_service_client_with_custom_poll_interval() {
        let client = ServiceClient::<RtClientPoll>::with_poll_interval(Duration::from_micros(500));
        assert!(client.is_ok());
    }

    #[test]
    fn test_async_service_client_creation() {
        let client = AsyncServiceClient::<RtAsyncCreate>::new();
        assert!(client.is_ok(), "AsyncServiceClient::new() should succeed");
    }

    #[test]
    fn test_async_service_client_with_custom_poll_interval() {
        let client =
            AsyncServiceClient::<RtAsyncCreatePoll>::with_poll_interval(Duration::from_micros(100));
        assert!(client.is_ok());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 6: ServiceServer creation and lifecycle tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_server_builder_default() {
        let builder = ServiceServerBuilder::<RtServerDefault>::new();
        let server = builder.build();
        assert!(server.is_ok());
        let server = server.unwrap();
        assert_eq!(server.name, "rt_server_default");
    }

    #[test]
    fn test_server_builder_with_handler() {
        let server = ServiceServerBuilder::<RtServerHandler>::new()
            .on_request(|req| Ok(RtServerHandlerResponse { sum: req.a + req.b }))
            .build();
        assert!(server.is_ok());
    }

    #[test]
    fn test_server_builder_with_poll_interval() {
        let server = ServiceServerBuilder::<RtServerPoll>::new()
            .on_request(|req| Ok(RtServerPollResponse { sum: req.a + req.b }))
            .poll_interval(Duration::from_millis(10))
            .build();
        assert!(server.is_ok());
    }

    #[test]
    fn test_server_stop() {
        let server = ServiceServerBuilder::<RtServerStop>::new()
            .on_request(|req| {
                Ok(RtServerStopResponse {
                    echoed: req.message,
                })
            })
            .build()
            .unwrap();
        server.stop();
    }

    #[test]
    fn test_server_drop_shuts_down() {
        let server = ServiceServerBuilder::<RtServerDrop>::new()
            .on_request(|_req| Ok(RtServerDropResponse {}))
            .build()
            .unwrap();
        drop(server);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 7: Client timeout behavior
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_client_call_timeout_no_server() {
        let mut client = ServiceClient::<RtClientTimeout>::new().unwrap();
        let result = client.call(RtClientTimeoutRequest {}, Duration::from_millis(50));
        assert!(
            matches!(result, Err(ServiceError::Timeout)),
            "expected Timeout, got {:?}",
            result
        );
    }

    #[test]
    fn test_client_call_optional_timeout_returns_none() {
        let mut client = ServiceClient::<RtClientOptTo>::new().unwrap();
        let result = client.call_optional(RtClientOptToRequest {}, Duration::from_millis(50));
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 8: Full client-server roundtrip tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_roundtrip_add_two_ints() {
        let _server = ServiceServerBuilder::<RtAddBasic>::new()
            .on_request(|req| Ok(RtAddBasicResponse { sum: req.a + req.b }))
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtAddBasic>::new().unwrap();
        let resp = client.call(RtAddBasicRequest { a: 3, b: 4 }, Duration::from_secs(2));

        assert!(resp.is_ok(), "roundtrip should succeed: {:?}", resp);
        assert_eq!(resp.unwrap().sum, 7);
    }

    #[test]
    fn test_roundtrip_echo_string() {
        let _server = ServiceServerBuilder::<RtEcho>::new()
            .on_request(|req| {
                Ok(RtEchoResponse {
                    echoed: req.message.clone(),
                })
            })
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtEcho>::new().unwrap();
        let resp = client.call(
            RtEchoRequest {
                message: "hello horus".to_string(),
            },
            Duration::from_secs(2),
        );

        assert!(resp.is_ok(), "echo roundtrip should succeed: {:?}", resp);
        assert_eq!(resp.unwrap().echoed, "hello horus");
    }

    #[test]
    fn test_roundtrip_multiple_calls() {
        let _server = ServiceServerBuilder::<RtMultiCall>::new()
            .on_request(|req| Ok(RtMultiCallResponse { sum: req.a + req.b }))
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtMultiCall>::new().unwrap();

        for i in 0..5 {
            let resp = client.call(
                RtMultiCallRequest { a: i, b: i * 10 },
                Duration::from_secs(2),
            );
            assert!(resp.is_ok(), "call {} should succeed", i);
            assert_eq!(resp.unwrap().sum, i + i * 10);
        }
    }

    #[test]
    fn test_roundtrip_server_returns_error() {
        let _server = ServiceServerBuilder::<RtFallible>::new()
            .on_request(|req| {
                if req.should_fail {
                    Err("intentional failure".to_string())
                } else {
                    Ok(RtFallibleResponse {
                        result: req.value * 2,
                    })
                }
            })
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtFallible>::new().unwrap();

        // Successful call
        let resp = client.call(
            RtFallibleRequest {
                should_fail: false,
                value: 21,
            },
            Duration::from_secs(2),
        );
        assert!(resp.is_ok());
        assert_eq!(resp.unwrap().result, 42);

        // Failing call
        let resp = client.call(
            RtFallibleRequest {
                should_fail: true,
                value: 0,
            },
            Duration::from_secs(2),
        );
        assert!(resp.is_err());
        match resp {
            Err(ServiceError::ServiceFailed(msg)) => {
                assert!(msg.contains("intentional failure"));
            }
            other => unreachable!("expected ServiceFailed, got {:?}", other),
        }
    }

    #[test]
    fn test_roundtrip_call_optional_success() {
        let _server = ServiceServerBuilder::<RtCallOptional>::new()
            .on_request(|req| Ok(RtCallOptionalResponse { sum: req.a + req.b }))
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtCallOptional>::new().unwrap();
        let resp = client.call_optional(
            RtCallOptionalRequest { a: 10, b: 20 },
            Duration::from_secs(2),
        );
        assert!(resp.is_ok());
        let inner = resp.unwrap();
        assert!(inner.is_some());
        assert_eq!(inner.unwrap().sum, 30);
    }

    #[test]
    fn test_roundtrip_call_optional_propagates_service_error() {
        let _server = ServiceServerBuilder::<RtCallOptErr>::new()
            .on_request(|_req| Err("forced error".to_string()))
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtCallOptErr>::new().unwrap();
        let resp = client.call_optional(
            RtCallOptErrRequest {
                should_fail: true,
                value: 0,
            },
            Duration::from_secs(2),
        );
        assert!(resp.is_err());
        assert!(
            matches!(resp, Err(ServiceError::ServiceFailed(_))),
            "expected ServiceFailed, got {:?}",
            resp
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 9: AsyncServiceClient tests
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_async_client_call_and_wait() {
        let _server = ServiceServerBuilder::<RtAsyncWait>::new()
            .on_request(|req| Ok(RtAsyncWaitResponse { sum: req.a + req.b }))
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = AsyncServiceClient::<RtAsyncWait>::new().unwrap();
        let pending = client.call_async(RtAsyncWaitRequest { a: 5, b: 6 }, Duration::from_secs(2));

        let resp = pending.wait();
        assert!(resp.is_ok(), "async wait should succeed: {:?}", resp);
        assert_eq!(resp.unwrap().sum, 11);
    }

    #[test]
    fn test_async_client_check_polling() {
        let _server = ServiceServerBuilder::<RtAsyncPoll>::new()
            .on_request(|req| Ok(RtAsyncPollResponse { sum: req.a + req.b }))
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = AsyncServiceClient::<RtAsyncPoll>::new().unwrap();
        let mut pending = client.call_async(
            RtAsyncPollRequest { a: 100, b: 200 },
            Duration::from_secs(2),
        );

        let mut result = None;
        for _ in 0..200 {
            match pending.check() {
                Ok(Some(resp)) => {
                    result = Some(resp);
                    break;
                }
                Ok(None) => {
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(e) => panic!("unexpected error during check: {:?}", e),
            }
        }

        assert!(
            result.is_some(),
            "should have received response via polling"
        );
        assert_eq!(result.unwrap().sum, 300);
    }

    #[test]
    fn test_async_client_timeout_on_check() {
        let mut client = AsyncServiceClient::<RtAsyncTimeout>::new().unwrap();
        let mut pending = client.call_async(RtAsyncTimeoutRequest {}, Duration::from_millis(50));

        std::thread::sleep(Duration::from_millis(100));

        let result = pending.check();
        assert!(
            matches!(result, Err(ServiceError::Timeout)),
            "expected Timeout, got {:?}",
            result
        );
    }

    #[test]
    fn test_async_client_is_expired() {
        let mut client = AsyncServiceClient::<RtAsyncExpired>::new().unwrap();
        let pending = client.call_async(RtAsyncExpiredRequest {}, Duration::from_millis(30));

        assert!(!pending.is_expired(), "should not be expired immediately");

        std::thread::sleep(Duration::from_millis(50));

        assert!(pending.is_expired(), "should be expired after timeout");
    }

    #[test]
    fn test_async_client_wait_timeout() {
        let mut client = AsyncServiceClient::<RtAsyncWaitTo>::new().unwrap();
        let pending = client.call_async(RtAsyncWaitToRequest {}, Duration::from_millis(50));

        let result = pending.wait();
        assert!(
            matches!(result, Err(ServiceError::Timeout)),
            "expected Timeout, got {:?}",
            result
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 10: Server without handler (default behavior)
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_server_no_handler_returns_error() {
        let _server = ServiceServerBuilder::<RtNoHandler>::new()
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtNoHandler>::new().unwrap();
        let resp = client.call(RtNoHandlerRequest {}, Duration::from_secs(2));

        assert!(resp.is_err());
        match resp {
            Err(ServiceError::ServiceFailed(msg)) => {
                assert!(msg.contains("no handler registered"));
            }
            other => unreachable!("expected ServiceFailed, got {:?}", other),
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Section 11: Edge cases
    // ═══════════════════════════════════════════════════════════════════════════

    #[test]
    fn test_server_drop_during_client_call() {
        let server = ServiceServerBuilder::<RtDropServer>::new()
            .on_request(|_req| {
                std::thread::sleep(Duration::from_millis(100));
                Ok(RtDropServerResponse {})
            })
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        drop(server);

        let mut client = ServiceClient::<RtDropServer>::new().unwrap();
        let result = client.call(RtDropServerRequest {}, Duration::from_millis(100));
        assert!(result.is_err());
    }

    #[test]
    fn test_large_payload_roundtrip() {
        let _server = ServiceServerBuilder::<RtLargePayload>::new()
            .on_request(|req| {
                Ok(RtLargePayloadResponse {
                    echoed: req.message.clone(),
                })
            })
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let large_message = "x".repeat(10_000);
        let mut client = ServiceClient::<RtLargePayload>::new().unwrap();
        let resp = client.call(
            RtLargePayloadRequest {
                message: large_message.clone(),
            },
            Duration::from_secs(5),
        );

        assert!(resp.is_ok(), "large payload roundtrip should work");
        assert_eq!(resp.unwrap().echoed.len(), 10_000);
    }

    #[test]
    fn test_negative_values_roundtrip() {
        let _server = ServiceServerBuilder::<RtNegative>::new()
            .on_request(|req| Ok(RtNegativeResponse { sum: req.a + req.b }))
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtNegative>::new().unwrap();
        let resp = client.call(
            RtNegativeRequest {
                a: i64::MIN / 2,
                b: i64::MIN / 2,
            },
            Duration::from_secs(2),
        );
        assert!(resp.is_ok());
        assert_eq!(resp.unwrap().sum, i64::MIN);
    }

    #[test]
    fn test_handler_with_shared_state() {
        let call_count = Arc::new(AtomicU32::new(0));
        let count_clone = call_count.clone();

        let _server = ServiceServerBuilder::<RtSharedState>::new()
            .on_request(move |req| {
                count_clone.fetch_add(1, Ordering::SeqCst);
                Ok(RtSharedStateResponse { sum: req.a + req.b })
            })
            .poll_interval(Duration::from_millis(1))
            .build()
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));

        let mut client = ServiceClient::<RtSharedState>::new().unwrap();
        for _ in 0..3 {
            let _ = client.call(RtSharedStateRequest { a: 1, b: 1 }, Duration::from_secs(2));
        }

        assert!(
            call_count.load(Ordering::SeqCst) >= 3,
            "handler should have been called at least 3 times"
        );
    }
}
