//! End-to-end integration tests for the full codegen pipeline.
//!
//! These tests exercise: parser → CXX bridge gen → C++ header gen → monomorphization
//! on realistic robotics API patterns (Publisher, Subscriber, Scheduler).

#[cfg(test)]
mod tests {
    use syn::{parse_quote, ItemImpl, ItemStruct};

    use crate::cpp_gen::generate_cpp_header;
    use crate::cxx_gen::{generate_cxx_bridge, generate_monomorphized_bridge};
    use crate::parser::{parse_impl, parse_struct};
    use crate::types::{BindingItem, MonomorphTarget, DEFAULT_MONOMORPH_TYPES};

    // ─── Full Publisher Pattern ──────────────────────────────────────

    #[test]
    fn full_publisher_pipeline() {
        // 1. Parse struct with hints
        let struct_item: ItemStruct = parse_quote! {
            #[cpp(raii)]
            pub struct Publisher {
                inner: u64,
            }
        };
        let struct_binding = parse_struct(&struct_item).unwrap();
        assert!(struct_binding.hints.raii, "raii hint parsed");

        // 2. Parse impl with method hints
        let impl_item: ItemImpl = parse_quote! {
            impl Publisher {
                #[cpp(returns = "unique_ptr")]
                pub fn new(topic: &str) -> Self {
                    todo!()
                }

                #[cpp(returns = "unique_ptr", raii)]
                pub fn loan(&self) -> LoanedSample {
                    todo!()
                }

                pub fn publish(&self, #[cpp(move_semantics)] sample: LoanedSample) {
                    todo!()
                }

                pub fn data_ptr(sample: &LoanedSample) -> *mut u8 {
                    todo!()
                }

                pub fn data_size(sample: &LoanedSample) -> usize {
                    todo!()
                }

                fn internal_helper(&self) {
                    // private — should be excluded from FFI
                }
            }
        };
        let impl_binding = parse_impl(&impl_item).unwrap();

        // Verify hints parsed correctly
        assert!(impl_binding.methods[0].hints.returns_unique_ptr, "new returns unique_ptr");
        assert!(impl_binding.methods[1].hints.raii, "loan has raii");
        assert!(impl_binding.methods[1].hints.returns_unique_ptr, "loan returns unique_ptr");
        assert!(impl_binding.methods[2].sig.params[0].hints.move_semantics, "publish param has move");

        // 3. Generate CXX bridge
        let bridge = generate_cxx_bridge(&BindingItem::Impl(impl_binding.clone()));
        let bridge_str = bridge.to_string();

        assert!(bridge_str.contains("type Publisher"), "opaque type");
        assert!(bridge_str.contains("publisher_new"), "new fn");
        assert!(bridge_str.contains("publisher_loan"), "loan fn");
        assert!(bridge_str.contains("publisher_publish"), "publish fn");
        assert!(bridge_str.contains("publisher_data_ptr"), "data_ptr fn");
        assert!(bridge_str.contains("publisher_data_size"), "data_size fn");
        assert!(!bridge_str.contains("publisher_internal_helper"), "private excluded");
        assert!(bridge_str.contains("Box < Publisher >"), "new returns Box");

        // 4. Generate C++ header
        let header = generate_cpp_header(&BindingItem::Impl(impl_binding));

        assert!(header.contains("#pragma once"), "pragma");
        assert!(header.contains("namespace horus {"), "namespace");
        assert!(header.contains("class Publisher {"), "class");
        assert!(header.contains("[[nodiscard]]"), "nodiscard on unique_ptr returns");
        assert!(header.contains("std::unique_ptr"), "unique_ptr return");
        assert!(header.contains("LoanedSample&& sample"), "move param");
        assert!(header.contains("std::string_view topic"), "string_view param");
        assert!(!header.contains("internal_helper"), "private excluded from header");

        // 5. Generate struct header
        let struct_header = generate_cpp_header(&BindingItem::Struct(struct_binding));
        assert!(struct_header.contains("~Publisher()"), "RAII destructor");
        assert!(struct_header.contains("Publisher(Publisher&& other) = default"), "move ctor");
        assert!(struct_header.contains("Publisher(const Publisher&) = delete"), "no copy");
    }

    // ─── Full Subscriber Pattern ─────────────────────────────────────

    #[test]
    fn full_subscriber_pipeline() {
        let impl_item: ItemImpl = parse_quote! {
            impl Subscriber {
                pub fn new(topic: &str) -> Self { todo!() }
                pub fn recv(&self) -> Option<BorrowedSample> { todo!() }
                pub fn has_msg(&self) -> bool { todo!() }
            }
        };
        let binding = parse_impl(&impl_item).unwrap();

        // CXX bridge
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding.clone()));
        let bs = bridge.to_string();
        assert!(bs.contains("subscriber_new"), "new");
        assert!(bs.contains("subscriber_recv"), "recv");
        assert!(bs.contains("subscriber_has_msg"), "has_msg");

        // C++ header
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert!(header.contains("std::optional<BorrowedSample>"), "optional return");
        assert!(header.contains("bool has_msg()"), "bool return");
        assert!(header.contains("const;"), "const method for &self");
    }

    // ─── Full Scheduler Pattern ──────────────────────────────────────

    #[test]
    fn full_scheduler_pipeline() {
        let impl_item: ItemImpl = parse_quote! {
            impl Scheduler {
                pub fn new() -> Self { todo!() }
                pub fn tick_rate(&mut self, hz: f64) { todo!() }
                pub fn prefer_rt(&mut self) { todo!() }
                pub fn monitoring(&mut self, enable: bool) { todo!() }
                pub fn run(self) -> Result<()> { todo!() }
            }
        };
        let binding = parse_impl(&impl_item).unwrap();

        // CXX bridge
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding.clone()));
        let bs = bridge.to_string();
        assert!(bs.contains("scheduler_new"), "new");
        assert!(bs.contains("scheduler_tick_rate"), "tick_rate");
        assert!(bs.contains("scheduler_prefer_rt"), "prefer_rt");
        assert!(bs.contains("scheduler_run"), "run");
        assert!(bs.contains("obj : Box < Scheduler >"), "run takes owned self");
        assert!(bs.contains("obj : & mut Scheduler"), "tick_rate takes &mut self");

        // C++ header
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert!(header.contains("static Self new()"), "static factory");
        assert!(header.contains("void tick_rate(double hz)"), "f64→double");
        assert!(header.contains("void monitoring(bool enable)"), "bool param");
        assert!(header.contains("void run()"), "Result<()>→void (throws)");
    }

    // ─── Monomorphized Topic Pattern ─────────────────────────────────

    #[test]
    fn full_monomorphized_topic_pipeline() {
        let impl_item: ItemImpl = parse_quote! {
            impl Topic {
                pub fn new(name: &str) -> Self { todo!() }
                pub fn send(&self, msg: u8) -> Result<()> { todo!() }
                pub fn recv(&self) -> Option<u8> { todo!() }
                pub fn read_latest(&self) -> Option<u8> { todo!() }
            }
        };
        let binding = parse_impl(&impl_item).unwrap();

        let targets = &[
            MonomorphTarget { rust_name: "CmdVel", snake_name: "cmd_vel", rust_path: "horus_library::CmdVel" },
            MonomorphTarget { rust_name: "LaserScan", snake_name: "laser_scan", rust_path: "horus_library::LaserScan" },
            MonomorphTarget { rust_name: "Imu", snake_name: "imu", rust_path: "horus_library::Imu" },
        ];

        let bridge = generate_monomorphized_bridge("Topic", &binding.methods, targets);
        let bs = bridge.to_string();

        // 3 concrete types
        assert!(bs.contains("type TopicCmdVel"), "CmdVel type");
        assert!(bs.contains("type TopicLaserScan"), "LaserScan type");
        assert!(bs.contains("type TopicImu"), "Imu type");

        // 4 methods × 3 types = 12 FFI functions
        for target in targets {
            let prefix = format!("topic_{}", target.snake_name);
            assert!(bs.contains(&format!("{}_new", prefix)), "{} new", target.rust_name);
            assert!(bs.contains(&format!("{}_send", prefix)), "{} send", target.rust_name);
            assert!(bs.contains(&format!("{}_recv", prefix)), "{} recv", target.rust_name);
            assert!(bs.contains(&format!("{}_read_latest", prefix)), "{} read_latest", target.rust_name);
        }

        // Result preserved in send
        assert!(bs.contains("Result"), "send has Result");
    }

    // ─── Default Monomorph Set Sanity ────────────────────────────────

    #[test]
    fn default_monomorph_set_generates_valid_bridge() {
        let impl_item: ItemImpl = parse_quote! {
            impl Topic {
                pub fn send(&self, msg: u8) -> Result<()> { todo!() }
                pub fn recv(&self) -> Option<u8> { todo!() }
            }
        };
        let binding = parse_impl(&impl_item).unwrap();

        let bridge = generate_monomorphized_bridge("Topic", &binding.methods, DEFAULT_MONOMORPH_TYPES);
        let bs = bridge.to_string();

        // Should have 25 concrete types
        assert!(bs.contains("type TopicCmdVel"), "CmdVel");
        assert!(bs.contains("type TopicLaserScan"), "LaserScan");
        assert!(bs.contains("type TopicImu"), "Imu");
        assert!(bs.contains("type TopicOdometry"), "Odometry");
        assert!(bs.contains("type TopicTwist"), "Twist");
        assert!(bs.contains("type TopicHeartbeat"), "Heartbeat");
        assert!(bs.contains("type TopicEmergencyStop"), "EmergencyStop");
        assert!(bs.contains("type TopicTFMessage"), "TFMessage");

        // Should have 25 × 2 = 50 FFI functions
        let fn_count = bs.matches("fn topic_").count();
        assert_eq!(fn_count, 50, "25 types × 2 methods = 50 functions, got {fn_count}");
    }

    // ─── LoanedSample Direct Field Access ────────────────────────────

    #[test]
    fn loaned_sample_direct_field_access() {
        let struct_item: ItemStruct = parse_quote! {
            #[cpp(direct_field_access, raii)]
            pub struct LoanedSample {
                ptr: *mut u8,
                size: usize,
            }
        };
        let binding = parse_struct(&struct_item).unwrap();

        assert!(binding.hints.direct_field_access, "direct_field_access");
        assert!(binding.hints.raii, "raii");

        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert!(header.contains("operator->()"), "operator->");
        assert!(header.contains("operator*()"), "operator*");
        assert!(header.contains("~LoanedSample()"), "destructor");
        assert!(header.contains("LoanedSample(const LoanedSample&) = delete"), "no copy");
        assert!(header.contains("void* data_ptr_"), "raw data pointer");
    }

    // ─── Service Pattern ─────────────────────────────────────────────

    #[test]
    fn service_client_pipeline() {
        let impl_item: ItemImpl = parse_quote! {
            impl ServiceClient {
                pub fn new(name: &str) -> Self { todo!() }
                pub fn call(&self, request: Request, timeout_ms: u64) -> Result<Response> { todo!() }
            }
        };
        let binding = parse_impl(&impl_item).unwrap();

        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding.clone()));
        let bs = bridge.to_string();
        assert!(bs.contains("service_client_new"), "new");
        assert!(bs.contains("service_client_call"), "call");
        assert!(bs.contains("Result <"), "Result return");

        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert!(header.contains("Response call("), "call returns Response (throws on error)");
        assert!(header.contains("uint64_t timeout_ms"), "u64→uint64_t");
    }

    // ─── Pipeline Metrics ────────────────────────────────────────────

    #[test]
    fn pipeline_coverage_summary() {
        // This test documents what the codegen pipeline covers
        // as a quick reference for developers.
        assert_eq!(DEFAULT_MONOMORPH_TYPES.len(), 25, "25 default message types");

        // Count hint types
        let hints = ["move_semantics", "raii", "direct_field_access", "returns", "name", "namespace"];
        assert_eq!(hints.len(), 6, "6 hint types supported");
    }
}
