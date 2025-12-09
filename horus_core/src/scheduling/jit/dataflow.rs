use super::compiler::JITCompiler;
use crate::core::Node;
use std::time::Instant;

/// Trait for nodes that can describe their computation as a dataflow expression.
///
/// Nodes implementing this trait can express their computation as a `DataflowExpr` AST,
/// which can then be compiled to native code via `JITCompiler::compile_dataflow_expr()`.
///
/// This is an optional trait - nodes that don't implement it will use the standard
/// tick-based execution. Implementing this enables automatic JIT compilation for
/// deterministic computations.
#[allow(dead_code)] // Public API - implemented by user nodes, not used internally
pub trait DataflowNode: Node {
    /// Get the dataflow computation as a simple expression
    /// Returns None if too complex for JIT
    fn get_dataflow_expr(&self) -> Option<DataflowExpr>;

    /// Check if this node is deterministic (same input = same output)
    fn is_deterministic(&self) -> bool {
        true
    }

    /// Check if this node has no side effects
    fn is_pure(&self) -> bool {
        true
    }
}

/// Simple expression AST for describing node computations.
///
/// Can be compiled to native code via `JITCompiler::compile_dataflow_expr()`.
///
/// # Example
/// ```ignore
/// use horus_core::scheduling::jit::{DataflowExpr, BinaryOp, JITCompiler};
///
/// // Build AST: input * 3 + 7
/// let expr = DataflowExpr::BinOp {
///     op: BinaryOp::Add,
///     left: Box::new(DataflowExpr::BinOp {
///         op: BinaryOp::Mul,
///         left: Box::new(DataflowExpr::Input("x".into())),
///         right: Box::new(DataflowExpr::Const(3)),
///     }),
///     right: Box::new(DataflowExpr::Const(7)),
/// };
///
/// let mut compiler = JITCompiler::new()?;
/// let func_ptr = compiler.compile_dataflow_expr("my_func", &expr)?;
/// ```
#[derive(Debug, Clone)]
pub enum DataflowExpr {
    /// Constant value
    Const(i64),

    /// Input variable
    Input(String),

    /// Binary operation
    BinOp {
        op: BinaryOp,
        left: Box<DataflowExpr>,
        right: Box<DataflowExpr>,
    },

    /// Unary operation
    UnaryOp {
        op: UnaryOp,
        expr: Box<DataflowExpr>,
    },
}

/// Binary operations for dataflow expressions
#[derive(Debug, Clone, Copy)]
pub enum BinaryOp {
    Add,
    Sub,
    Mul,
    Div,
    Mod,
    And,
    Or,
    Xor,
}

/// Unary operations for dataflow expressions
#[derive(Debug, Clone, Copy)]
pub enum UnaryOp {
    Neg,
    Not,
    Abs,
}

/// A compiled dataflow graph that runs at native speed
pub struct CompiledDataflow {
    /// Name of the compiled dataflow
    pub name: String,

    /// Compiled function pointer
    pub func_ptr: *const u8,

    /// Execution statistics
    pub exec_count: u64,
    pub total_ns: u64,
}

// Safety: The compiled function pointer points to read-only JIT code
// which is safe to access from multiple threads
unsafe impl Send for CompiledDataflow {}
unsafe impl Sync for CompiledDataflow {}

impl CompiledDataflow {
    /// Create a new compiled dataflow from a dataflow expression
    ///
    /// # Arguments
    /// * `name` - Name for the compiled function
    /// * `expr` - The dataflow expression to compile
    ///
    /// # Returns
    /// A compiled dataflow that can execute native code, or an error if compilation fails
    pub fn new(name: &str, expr: &DataflowExpr) -> Result<Self, String> {
        let mut compiler = JITCompiler::new()?;
        let func_ptr = compiler.compile_dataflow_expr(name, expr)?;

        Ok(Self {
            name: name.to_string(),
            func_ptr,
            exec_count: 0,
            total_ns: 0,
        })
    }

    /// Create a compiled dataflow for a simple arithmetic operation: output = input * multiplier + addend
    ///
    /// # Arguments
    /// * `name` - Name for the compiled function
    /// * `multiplier` - The multiplier value
    /// * `addend` - The value to add
    pub fn new_arithmetic(name: &str, multiplier: i64, addend: i64) -> Result<Self, String> {
        let mut compiler = JITCompiler::new()?;
        let func_ptr = compiler.compile_arithmetic_node(name, multiplier, addend)?;

        Ok(Self {
            name: name.to_string(),
            func_ptr,
            exec_count: 0,
            total_ns: 0,
        })
    }

    /// Execute the compiled dataflow with given inputs
    ///
    /// # Panics
    /// Panics if the compiled function pointer is null (JIT compilation failed).
    /// Use `try_execute` for error handling instead of panics.
    pub fn execute(&mut self, input: i64) -> i64 {
        let start = Instant::now();

        // Execute the compiled function - panic if null (JIT failed)
        let result = if !self.func_ptr.is_null() {
            // Cast to function pointer and execute
            unsafe {
                let func: fn(i64) -> i64 = std::mem::transmute(self.func_ptr);
                func(input)
            }
        } else {
            panic!(
                "JIT compilation failed for '{}': cannot execute without compiled function. \
                Use try_execute() to handle this case gracefully.",
                self.name
            );
        };

        let elapsed_ns = start.elapsed().as_nanos() as u64;
        self.exec_count += 1;
        self.total_ns += elapsed_ns;

        result
    }

    /// Try to execute the compiled dataflow, returning an error if JIT compilation failed
    pub fn try_execute(&mut self, input: i64) -> Result<i64, String> {
        if self.func_ptr.is_null() {
            return Err(format!(
                "JIT compilation failed for '{}': no compiled function available",
                self.name
            ));
        }

        let start = Instant::now();
        let result = unsafe {
            let func: fn(i64) -> i64 = std::mem::transmute(self.func_ptr);
            func(input)
        };

        let elapsed_ns = start.elapsed().as_nanos() as u64;
        self.exec_count += 1;
        self.total_ns += elapsed_ns;

        Ok(result)
    }

    /// Check if this dataflow has a valid compiled function
    pub fn is_compiled(&self) -> bool {
        !self.func_ptr.is_null()
    }

    /// Create a stats-only CompiledDataflow without actual JIT compilation
    ///
    /// Use this when you want to track execution statistics for a node that
    /// doesn't support JIT compilation. The `execute()` method will panic if called
    /// on a stats-only dataflow - use `try_execute()` or check `is_compiled()` first.
    pub fn new_stats_only(name: &str) -> Self {
        Self {
            name: name.to_string(),
            func_ptr: std::ptr::null(),
            exec_count: 0,
            total_ns: 0,
        }
    }

    /// Record execution time for stats tracking (when not using JIT)
    pub fn record_execution(&mut self, elapsed_ns: u64) {
        self.exec_count += 1;
        self.total_ns += elapsed_ns;
    }

    /// Get average execution time in nanoseconds
    pub fn avg_exec_ns(&self) -> f64 {
        if self.exec_count == 0 {
            0.0
        } else {
            self.total_ns as f64 / self.exec_count as f64
        }
    }

    /// Check if this dataflow is performing well (< 100ns average)
    pub fn is_fast_enough(&self) -> bool {
        self.avg_exec_ns() < 100.0
    }
}
