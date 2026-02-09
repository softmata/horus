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

    /// Create a builder for constructing complex dataflow graphs.
    ///
    /// The builder provides a fluent API for defining dataflow computations
    /// that can be compiled to native code.
    ///
    /// # Example
    /// ```ignore
    /// use horus_core::scheduling::jit::CompiledDataflow;
    ///
    /// let dataflow = CompiledDataflow::builder()
    ///     .input("sensor1")
    ///     .input("sensor2")
    ///     .add("sensor1", "sensor2", "sum")
    ///     .multiply("sum", "gain", "scaled")
    ///     .add("scaled", "offset", "output")
    ///     .build()?;
    ///
    /// let result = dataflow.execute(42);
    /// ```
    pub fn builder() -> DataflowBuilder {
        DataflowBuilder::new()
    }
}

/// Builder for constructing complex dataflow graphs.
///
/// Provides a fluent API for defining dataflow computations that can be
/// compiled to native code using Cranelift JIT.
///
/// # Example
/// ```ignore
/// use horus_core::scheduling::jit::CompiledDataflow;
///
/// // Simple scaling: output = input * 2 + 10
/// let dataflow = CompiledDataflow::builder()
///     .name("scaling")
///     .constant("scale", 2)
///     .constant("offset", 10)
///     .multiply("input", "scale", "scaled")
///     .add("scaled", "offset", "output")
///     .output("output")
///     .build()?;
///
/// // Multi-sensor fusion: output = (sensor1 + sensor2) / 2
/// let fusion = CompiledDataflow::builder()
///     .name("sensor_fusion")
///     .input("sensor1")
///     .input("sensor2")
///     .constant("divisor", 2)
///     .add("sensor1", "sensor2", "sum")
///     .divide("sum", "divisor", "output")
///     .output("output")
///     .build()?;
/// ```
#[derive(Debug, Clone)]
pub struct DataflowBuilder {
    /// Name of the dataflow
    name: String,
    /// Input variable names
    inputs: Vec<String>,
    /// Constants (name -> value)
    constants: std::collections::HashMap<String, i64>,
    /// Operations in order
    operations: Vec<DataflowOp>,
    /// Output variable name
    output: Option<String>,
}

/// Internal operation representation for the builder
#[derive(Debug, Clone)]
struct DataflowOp {
    op_type: DataflowOpType,
    left: String,
    right: String,
    output: String,
}

#[derive(Debug, Clone)]
enum DataflowOpType {
    Add,
    Sub,
    Mul,
    Div,
    Mod,
    And,
    Or,
    Xor,
    Neg, // Unary: output = -left
    Abs, // Unary: output = |left|
}

impl DataflowBuilder {
    /// Create a new dataflow builder
    pub fn new() -> Self {
        Self {
            name: "dataflow".to_string(),
            inputs: vec!["input".to_string()], // Default input
            constants: std::collections::HashMap::new(),
            operations: Vec::new(),
            output: None,
        }
    }

    /// Set the name of the dataflow
    pub fn name(mut self, name: &str) -> Self {
        self.name = name.to_string();
        self
    }

    /// Add an input variable
    ///
    /// By default, there is one input called "input". Call this to add
    /// additional named inputs or to change the input name.
    pub fn input(mut self, name: &str) -> Self {
        if !self.inputs.contains(&name.to_string()) {
            self.inputs.push(name.to_string());
        }
        self
    }

    /// Clear default inputs (call before adding custom inputs)
    pub fn no_default_input(mut self) -> Self {
        self.inputs.clear();
        self
    }

    /// Add a constant value
    pub fn constant(mut self, name: &str, value: i64) -> Self {
        self.constants.insert(name.to_string(), value);
        self
    }

    /// Add two values: output = left + right
    pub fn add(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Add,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Subtract two values: output = left - right
    pub fn subtract(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Sub,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Multiply two values: output = left * right
    pub fn multiply(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Mul,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Divide two values: output = left / right
    pub fn divide(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Div,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Modulo two values: output = left % right
    pub fn modulo(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Mod,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Bitwise AND: output = left & right
    pub fn bitwise_and(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::And,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Bitwise OR: output = left | right
    pub fn bitwise_or(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Or,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Bitwise XOR: output = left ^ right
    pub fn bitwise_xor(mut self, left: &str, right: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Xor,
            left: left.to_string(),
            right: right.to_string(),
            output: output.to_string(),
        });
        self
    }

    /// Negate a value: output = -input
    pub fn negate(mut self, input: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Neg,
            left: input.to_string(),
            right: String::new(), // Unused for unary ops
            output: output.to_string(),
        });
        self
    }

    /// Absolute value: output = |input|
    pub fn abs(mut self, input: &str, output: &str) -> Self {
        self.operations.push(DataflowOp {
            op_type: DataflowOpType::Abs,
            left: input.to_string(),
            right: String::new(), // Unused for unary ops
            output: output.to_string(),
        });
        self
    }

    /// Set the output variable name
    ///
    /// If not called, the output of the last operation is used.
    pub fn output(mut self, name: &str) -> Self {
        self.output = Some(name.to_string());
        self
    }

    /// Build the dataflow expression AST
    fn build_expr(&self) -> Result<DataflowExpr, String> {
        if self.operations.is_empty() {
            // No operations - just return the first input
            if self.inputs.is_empty() {
                return Err("No inputs or operations defined".to_string());
            }
            return Ok(DataflowExpr::Input(self.inputs[0].clone()));
        }

        // Track computed values
        let mut values: std::collections::HashMap<String, DataflowExpr> =
            std::collections::HashMap::new();

        // Add inputs
        for input in &self.inputs {
            values.insert(input.clone(), DataflowExpr::Input(input.clone()));
        }

        // Add constants
        for (name, value) in &self.constants {
            values.insert(name.clone(), DataflowExpr::Const(*value));
        }

        // Process operations
        for op in &self.operations {
            let left_expr = values
                .get(&op.left)
                .cloned()
                .ok_or_else(|| format!("Unknown variable: {}", op.left))?;

            let result_expr = match op.op_type {
                DataflowOpType::Neg => DataflowExpr::UnaryOp {
                    op: UnaryOp::Neg,
                    expr: Box::new(left_expr),
                },
                DataflowOpType::Abs => DataflowExpr::UnaryOp {
                    op: UnaryOp::Abs,
                    expr: Box::new(left_expr),
                },
                _ => {
                    // Binary operation
                    let right_expr = values
                        .get(&op.right)
                        .cloned()
                        .ok_or_else(|| format!("Unknown variable: {}", op.right))?;

                    let binary_op = match op.op_type {
                        DataflowOpType::Add => BinaryOp::Add,
                        DataflowOpType::Sub => BinaryOp::Sub,
                        DataflowOpType::Mul => BinaryOp::Mul,
                        DataflowOpType::Div => BinaryOp::Div,
                        DataflowOpType::Mod => BinaryOp::Mod,
                        DataflowOpType::And => BinaryOp::And,
                        DataflowOpType::Or => BinaryOp::Or,
                        DataflowOpType::Xor => BinaryOp::Xor,
                        _ => unreachable!(),
                    };

                    DataflowExpr::BinOp {
                        op: binary_op,
                        left: Box::new(left_expr),
                        right: Box::new(right_expr),
                    }
                }
            };

            values.insert(op.output.clone(), result_expr);
        }

        // Get output expression
        let output_name = self.output.clone().unwrap_or_else(|| {
            // Use the output of the last operation
            self.operations.last().map(|op| op.output.clone()).unwrap()
        });

        values
            .remove(&output_name)
            .ok_or_else(|| format!("Output variable not found: {}", output_name))
    }

    /// Build and compile the dataflow to native code
    pub fn build(self) -> Result<CompiledDataflow, String> {
        let expr = self.build_expr()?;
        CompiledDataflow::new(&self.name, &expr)
    }

    /// Build without compiling (returns the expression AST)
    pub fn build_expr_only(self) -> Result<DataflowExpr, String> {
        self.build_expr()
    }
}

impl Default for DataflowBuilder {
    fn default() -> Self {
        Self::new()
    }
}
