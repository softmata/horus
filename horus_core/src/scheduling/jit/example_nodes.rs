use super::dataflow::{BinaryOp, DataflowExpr, DataflowNode};
/// Example DataflowNode implementations for testing and demonstration
/// These are internal nodes used by the scheduler for JIT compilation testing
use crate::core::Node;
use crate::error::HorusResult;

/// Simple scaling node: output = input * scale + offset
/// This is a pure, deterministic node suitable for JIT compilation
pub struct ScalingNode {
    scale: i64,
    offset: i64,
    current_input: i64,
    current_output: i64,
}

impl ScalingNode {
    pub fn new(scale: i64, offset: i64) -> Self {
        Self {
            scale,
            offset,
            current_input: 0,
            current_output: 0,
        }
    }
}

impl Node for ScalingNode {
    fn name(&self) -> &'static str {
        "jit_scaling_test"
    }

    fn tick(&mut self) {
        // Simple arithmetic: output = input * scale + offset
        self.current_output = self.current_input * self.scale + self.offset;
        self.current_input += 1; // Increment for next tick
    }

    fn init(&mut self) -> HorusResult<()> {
        Ok(())
    }

    fn shutdown(&mut self) -> HorusResult<()> {
        Ok(())
    }

    fn get_publishers(&self) -> Vec<crate::core::TopicMetadata> {
        Vec::new()
    }

    fn get_subscribers(&self) -> Vec<crate::core::TopicMetadata> {
        Vec::new()
    }

    // ==================== JIT Compilation Support ====================
    /// This node supports JIT compilation
    fn supports_jit(&self) -> bool {
        true
    }

    /// This node is deterministic (same input always produces same output)
    fn is_jit_deterministic(&self) -> bool {
        true
    }

    /// This node is pure (no side effects)
    fn is_jit_pure(&self) -> bool {
        true
    }

    /// Get JIT parameters for Cranelift compilation: output = input * factor + offset
    fn get_jit_arithmetic_params(&self) -> Option<(i64, i64)> {
        Some((self.scale, self.offset))
    }
}

impl DataflowNode for ScalingNode {
    fn get_dataflow_expr(&self) -> Option<DataflowExpr> {
        // Represents: (input * scale) + offset
        Some(DataflowExpr::BinOp {
            op: BinaryOp::Add,
            left: Box::new(DataflowExpr::BinOp {
                op: BinaryOp::Mul,
                left: Box::new(DataflowExpr::Input("value".to_string())),
                right: Box::new(DataflowExpr::Const(self.scale)),
            }),
            right: Box::new(DataflowExpr::Const(self.offset)),
        })
    }

    fn is_deterministic(&self) -> bool {
        true
    }

    fn is_pure(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scaling_node_arithmetic() {
        let mut node = ScalingNode::new(2, 1);

        // First tick: 0 * 2 + 1 = 1
        node.tick();
        assert_eq!(node.current_output, 1);

        // Second tick: 1 * 2 + 1 = 3
        node.tick();
        assert_eq!(node.current_output, 3);

        // Third tick: 2 * 2 + 1 = 5
        node.tick();
        assert_eq!(node.current_output, 5);
    }

    #[test]
    fn test_dataflow_expr_generation() {
        let node = ScalingNode::new(3, 7);
        let expr = node.get_dataflow_expr();
        assert!(expr.is_some());

        // Verify it's deterministic and pure
        assert!(node.is_deterministic());
        assert!(node.is_pure());
    }
}
