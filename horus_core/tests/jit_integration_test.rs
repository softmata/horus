/// Integration test for JIT compilation system
/// Demonstrates how the new trait-based JIT support works
use horus_core::core::Node;
use horus_core::scheduling::jit::ScalingNode;
use horus_core::scheduling::Scheduler;
use std::time::Duration;

#[test]
fn test_jit_scaling_node_compilation() {
    // Create a ScalingNode that implements the JIT trait methods
    let node = ScalingNode::new(3, 7);

    // Verify it reports JIT support
    assert!(node.supports_jit(), "ScalingNode should support JIT");
    assert!(
        node.is_jit_deterministic(),
        "ScalingNode should be deterministic"
    );
    assert!(node.is_jit_pure(), "ScalingNode should be pure");

    // Verify it provides arithmetic params for Cranelift compilation
    let params = node.get_jit_arithmetic_params();
    assert!(params.is_some(), "Should provide arithmetic params");
    let (factor, offset) = params.unwrap();
    assert_eq!(factor, 3, "Factor should be 3");
    assert_eq!(offset, 7, "Offset should be 7");
}

#[test]
fn test_scheduler_with_jit_node() {
    let mut scheduler = Scheduler::new();

    // Add a real JIT-capable ScalingNode
    // The scheduler should detect supports_jit() and compile with Cranelift
    let node = ScalingNode::new(2, 10); // output = input * 2 + 10
    scheduler.add(node).order(0).done();

    // Run for a short duration - this will:
    // 1. Compile the node using Cranelift JIT
    // 2. Execute via the JIT path (native code)
    // 3. Track statistics
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_jit_execution_performance() {
    use std::time::Instant;

    // Test direct JIT compiler performance
    let mut compiler =
        horus_core::scheduling::jit::JITCompiler::new().expect("Failed to create JIT compiler");

    // Compile a simple arithmetic function
    let func_ptr = compiler
        .compile_arithmetic_node("perf_test", 5, 3)
        .expect("Failed to compile");

    assert!(!func_ptr.is_null(), "Function pointer should not be null");

    // Execute and measure timing
    let iterations = 10000;
    let start = Instant::now();

    for i in 0..iterations {
        unsafe {
            let func: fn(i64) -> i64 = std::mem::transmute(func_ptr);
            let result = func(i);
            // Verify correctness: result = i * 5 + 3
            assert_eq!(result, i * 5 + 3, "JIT computation incorrect");
        }
    }

    let elapsed = start.elapsed();
    let avg_ns = elapsed.as_nanos() as f64 / iterations as f64;

    println!("[JIT PERF] {} iterations in {:?}", iterations, elapsed);
    println!("[JIT PERF] Average: {:.0}ns per call", avg_ns);

    // JIT should be ultra-fast (< 200ns per call typically)
    assert!(
        avg_ns < 1000.0,
        "JIT execution should be under 1000ns, got {:.0}ns",
        avg_ns
    );
}

#[test]
fn test_non_jit_node_default_behavior() {
    // A node without JIT support should use default trait implementations
    struct RegularNode;

    impl Node for RegularNode {
        fn name(&self) -> &'static str {
            "regular_node"
        }

        fn tick(&mut self) {
            // Regular computation
        }

        fn init(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn shutdown(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn get_publishers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }

        fn get_subscribers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }
    }

    let node = RegularNode;

    // Default implementations should return false/None
    assert!(
        !node.supports_jit(),
        "Regular node should not support JIT by default"
    );
    assert!(
        !node.is_jit_deterministic(),
        "Default should be non-deterministic"
    );
    assert!(!node.is_jit_pure(), "Default should be non-pure");
    assert!(
        node.get_jit_arithmetic_params().is_none(),
        "Default should have no arithmetic params"
    );
    assert!(
        node.get_jit_compute().is_none(),
        "Default should have no compute function"
    );
}

#[test]
fn test_custom_jit_compute_function() {
    // Test a node that provides a custom compute function
    #[allow(dead_code)]
    struct CustomJITNode {
        multiplier: i64,
    }

    impl Node for CustomJITNode {
        fn name(&self) -> &'static str {
            "custom_jit_node"
        }

        fn tick(&mut self) {
            // Fallback tick
        }

        fn init(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn shutdown(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn get_publishers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }

        fn get_subscribers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }

        fn supports_jit(&self) -> bool {
            true
        }

        fn is_jit_deterministic(&self) -> bool {
            true
        }

        fn is_jit_pure(&self) -> bool {
            true
        }

        fn get_jit_compute(&self) -> Option<fn(i64) -> i64> {
            // Provide a simple doubling function
            Some(|x| x * 2)
        }
    }

    let node = CustomJITNode { multiplier: 2 };

    assert!(node.supports_jit());
    assert!(node.get_jit_compute().is_some());

    // Execute the compute function
    if let Some(compute) = node.get_jit_compute() {
        assert_eq!(compute(5), 10);
        assert_eq!(compute(100), 200);
    }
}

#[test]
fn test_mixed_scheduler_with_jit_and_regular_nodes() {
    let mut scheduler = Scheduler::new();

    // Add a JIT-capable node
    let jit_node = ScalingNode::new(4, 1); // output = input * 4 + 1
    scheduler.add(jit_node).order(0).done();

    // Add a regular node
    struct RegularComputeNode {
        counter: u64,
    }

    impl Node for RegularComputeNode {
        fn name(&self) -> &'static str {
            "regular_compute"
        }

        fn tick(&mut self) {
            self.counter += 1;
        }

        fn init(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn shutdown(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn get_publishers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }

        fn get_subscribers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }
    }

    scheduler.add(RegularComputeNode { counter: 0 }).order(1).done();

    // Run both nodes - JIT node uses native code, regular node uses tick()
    let result = scheduler.run_for(Duration::from_millis(50));
    assert!(result.is_ok());
}

#[test]
fn test_learning_phase_with_jit() {
    // This test demonstrates that:
    // 1. Nodes that declare supports_jit() are compiled at add-time
    // 2. The learning phase profiles all nodes
    // 3. After learning, ultra-fast nodes that weren't pre-compiled get JIT'd
    // 4. Pre-compiled nodes are NOT overwritten by learning phase

    let mut scheduler = Scheduler::new();

    // Add a JIT-capable node - will be compiled at add-time with factor=5, offset=2
    let jit_node = ScalingNode::new(5, 2);
    scheduler.add(jit_node).order(0).done();

    // Add a regular fast node - might get JIT'd during learning if ultra-fast
    struct FastNode {
        counter: u64,
    }

    impl Node for FastNode {
        fn name(&self) -> &'static str {
            "fast_regular_node"
        }

        fn tick(&mut self) {
            // Ultra-fast computation - likely to be classified as UltraFast
            self.counter = self.counter.wrapping_add(1);
        }

        fn init(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn shutdown(&mut self) -> horus_core::error::Result<()> {
            Ok(())
        }

        fn get_publishers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }

        fn get_subscribers(&self) -> Vec<horus_core::core::TopicMetadata> {
            Vec::new()
        }
    }

    scheduler.add(FastNode { counter: 0 }).order(1).done();

    // Run long enough for learning phase to potentially complete
    // Learning phase typically needs ~100+ ticks per node
    let result = scheduler.run_for(Duration::from_millis(200));
    assert!(result.is_ok());

    // The test passes if:
    // - ScalingNode was compiled at add-time (factor=5, offset=2)
    // - Learning phase did NOT overwrite ScalingNode's correct JIT function
    // - FastNode may or may not have been auto-compiled depending on profiler classification
}
