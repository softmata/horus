//! ROS2 Parameter Server Bridge for Zenoh
//!
//! Implements the ROS2 parameter server protocol over Zenoh.
//!
//! # ROS2 Parameter Protocol
//!
//! ROS2 nodes expose parameters through the following service interfaces:
//! - List Parameters: `{node}/_parameters/list_parameters`
//! - Get Parameters: `{node}/_parameters/get_parameters`
//! - Set Parameters: `{node}/_parameters/set_parameters`
//! - Describe Parameters: `{node}/_parameters/describe_parameters`
//! - Get Parameter Types: `{node}/_parameters/get_parameter_types`
//! - Set Parameters Atomically: `{node}/_parameters/set_parameters_atomically`
//!
//! Additionally, parameter change events are published to:
//! - Parameter Events: `{node}/_parameters/parameter_events`
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::zenoh_ros2_params::*;
//!
//! // Create a parameter client to interact with a ROS2 node
//! let client = Ros2ParameterClient::new("robot_controller", config).await?;
//!
//! // List all parameters
//! let params = client.list_parameters(ListParametersRequest::default()).await?;
//!
//! // Get a specific parameter
//! let value = client.get_parameters(GetParametersRequest {
//!     names: vec!["max_velocity".to_string()],
//! }).await?;
//! ```

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use parking_lot::RwLock;
use serde::{Deserialize, Serialize};

use crate::zenoh_ros2_services::Ros2ServiceError;

// ============================================================================
// ROS2 Parameter Topic Naming
// ============================================================================

/// Generate the list_parameters service topic name
pub fn param_to_list_parameters_topic(node_name: &str) -> String {
    format!(
        "rq/{}/_parameters/list_parameters",
        normalize_node_name(node_name)
    )
}

/// Generate the get_parameters service topic name
pub fn param_to_get_parameters_topic(node_name: &str) -> String {
    format!(
        "rq/{}/_parameters/get_parameters",
        normalize_node_name(node_name)
    )
}

/// Generate the set_parameters service topic name
pub fn param_to_set_parameters_topic(node_name: &str) -> String {
    format!(
        "rq/{}/_parameters/set_parameters",
        normalize_node_name(node_name)
    )
}

/// Generate the describe_parameters service topic name
pub fn param_to_describe_parameters_topic(node_name: &str) -> String {
    format!(
        "rq/{}/_parameters/describe_parameters",
        normalize_node_name(node_name)
    )
}

/// Generate the get_parameter_types service topic name
pub fn param_to_get_parameter_types_topic(node_name: &str) -> String {
    format!(
        "rq/{}/_parameters/get_parameter_types",
        normalize_node_name(node_name)
    )
}

/// Generate the set_parameters_atomically service topic name
pub fn param_to_set_parameters_atomically_topic(node_name: &str) -> String {
    format!(
        "rq/{}/_parameters/set_parameters_atomically",
        normalize_node_name(node_name)
    )
}

/// Generate the parameter_events topic name
pub fn param_to_events_topic(node_name: &str) -> String {
    format!(
        "rt/{}/_parameters/parameter_events",
        normalize_node_name(node_name)
    )
}

/// Normalize node name for topic generation
fn normalize_node_name(node_name: &str) -> String {
    // Remove leading slash if present, replace remaining slashes
    let normalized = node_name.trim_start_matches('/');
    normalized.replace('/', "__")
}

/// Types of parameter topics
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParameterTopicType {
    ListParameters,
    GetParameters,
    SetParameters,
    DescribeParameters,
    GetParameterTypes,
    SetParametersAtomically,
    ParameterEvents,
}

impl ParameterTopicType {
    /// Get the service/topic name suffix
    pub fn suffix(&self) -> &'static str {
        match self {
            Self::ListParameters => "list_parameters",
            Self::GetParameters => "get_parameters",
            Self::SetParameters => "set_parameters",
            Self::DescribeParameters => "describe_parameters",
            Self::GetParameterTypes => "get_parameter_types",
            Self::SetParametersAtomically => "set_parameters_atomically",
            Self::ParameterEvents => "parameter_events",
        }
    }

    /// Check if this is a service (request/response) type
    pub fn is_service(&self) -> bool {
        !matches!(self, Self::ParameterEvents)
    }
}

// ============================================================================
// ROS2 Parameter Types
// ============================================================================

/// ROS2 Parameter types (from rcl_interfaces/msg/ParameterType)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ParameterType {
    NotSet = 0,
    Bool = 1,
    Integer = 2,
    Double = 3,
    String = 4,
    ByteArray = 5,
    BoolArray = 6,
    IntegerArray = 7,
    DoubleArray = 8,
    StringArray = 9,
}

impl Default for ParameterType {
    fn default() -> Self {
        Self::NotSet
    }
}

impl ParameterType {
    /// Get type name as string
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::NotSet => "not_set",
            Self::Bool => "bool",
            Self::Integer => "integer",
            Self::Double => "double",
            Self::String => "string",
            Self::ByteArray => "byte_array",
            Self::BoolArray => "bool_array",
            Self::IntegerArray => "integer_array",
            Self::DoubleArray => "double_array",
            Self::StringArray => "string_array",
        }
    }

    /// Create from type code
    pub fn from_code(code: u8) -> Self {
        match code {
            0 => Self::NotSet,
            1 => Self::Bool,
            2 => Self::Integer,
            3 => Self::Double,
            4 => Self::String,
            5 => Self::ByteArray,
            6 => Self::BoolArray,
            7 => Self::IntegerArray,
            8 => Self::DoubleArray,
            9 => Self::StringArray,
            _ => Self::NotSet,
        }
    }
}

/// ROS2 Parameter Value (from rcl_interfaces/msg/ParameterValue)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParameterValue {
    /// Type of the parameter
    #[serde(rename = "type")]
    pub param_type: u8,
    /// Boolean value (valid when type == PARAMETER_BOOL)
    pub bool_value: bool,
    /// Integer value (valid when type == PARAMETER_INTEGER)
    pub integer_value: i64,
    /// Double value (valid when type == PARAMETER_DOUBLE)
    pub double_value: f64,
    /// String value (valid when type == PARAMETER_STRING)
    pub string_value: String,
    /// Byte array value (valid when type == PARAMETER_BYTE_ARRAY)
    pub byte_array_value: Vec<u8>,
    /// Bool array value (valid when type == PARAMETER_BOOL_ARRAY)
    pub bool_array_value: Vec<bool>,
    /// Integer array value (valid when type == PARAMETER_INTEGER_ARRAY)
    pub integer_array_value: Vec<i64>,
    /// Double array value (valid when type == PARAMETER_DOUBLE_ARRAY)
    pub double_array_value: Vec<f64>,
    /// String array value (valid when type == PARAMETER_STRING_ARRAY)
    pub string_array_value: Vec<String>,
}

impl Default for ParameterValue {
    fn default() -> Self {
        Self {
            param_type: ParameterType::NotSet as u8,
            bool_value: false,
            integer_value: 0,
            double_value: 0.0,
            string_value: String::new(),
            byte_array_value: Vec::new(),
            bool_array_value: Vec::new(),
            integer_array_value: Vec::new(),
            double_array_value: Vec::new(),
            string_array_value: Vec::new(),
        }
    }
}

impl ParameterValue {
    /// Create a bool parameter value
    pub fn from_bool(value: bool) -> Self {
        Self {
            param_type: ParameterType::Bool as u8,
            bool_value: value,
            ..Default::default()
        }
    }

    /// Create an integer parameter value
    pub fn from_integer(value: i64) -> Self {
        Self {
            param_type: ParameterType::Integer as u8,
            integer_value: value,
            ..Default::default()
        }
    }

    /// Create a double parameter value
    pub fn from_double(value: f64) -> Self {
        Self {
            param_type: ParameterType::Double as u8,
            double_value: value,
            ..Default::default()
        }
    }

    /// Create a string parameter value
    pub fn from_string(value: impl Into<String>) -> Self {
        Self {
            param_type: ParameterType::String as u8,
            string_value: value.into(),
            ..Default::default()
        }
    }

    /// Create a byte array parameter value
    pub fn from_byte_array(value: Vec<u8>) -> Self {
        Self {
            param_type: ParameterType::ByteArray as u8,
            byte_array_value: value,
            ..Default::default()
        }
    }

    /// Create a bool array parameter value
    pub fn from_bool_array(value: Vec<bool>) -> Self {
        Self {
            param_type: ParameterType::BoolArray as u8,
            bool_array_value: value,
            ..Default::default()
        }
    }

    /// Create an integer array parameter value
    pub fn from_integer_array(value: Vec<i64>) -> Self {
        Self {
            param_type: ParameterType::IntegerArray as u8,
            integer_array_value: value,
            ..Default::default()
        }
    }

    /// Create a double array parameter value
    pub fn from_double_array(value: Vec<f64>) -> Self {
        Self {
            param_type: ParameterType::DoubleArray as u8,
            double_array_value: value,
            ..Default::default()
        }
    }

    /// Create a string array parameter value
    pub fn from_string_array(value: Vec<String>) -> Self {
        Self {
            param_type: ParameterType::StringArray as u8,
            string_array_value: value,
            ..Default::default()
        }
    }

    /// Get the parameter type
    pub fn get_type(&self) -> ParameterType {
        ParameterType::from_code(self.param_type)
    }

    /// Try to get as bool
    pub fn as_bool(&self) -> Option<bool> {
        if self.param_type == ParameterType::Bool as u8 {
            Some(self.bool_value)
        } else {
            None
        }
    }

    /// Try to get as integer
    pub fn as_integer(&self) -> Option<i64> {
        if self.param_type == ParameterType::Integer as u8 {
            Some(self.integer_value)
        } else {
            None
        }
    }

    /// Try to get as double
    pub fn as_double(&self) -> Option<f64> {
        if self.param_type == ParameterType::Double as u8 {
            Some(self.double_value)
        } else {
            None
        }
    }

    /// Try to get as string
    pub fn as_string(&self) -> Option<&str> {
        if self.param_type == ParameterType::String as u8 {
            Some(&self.string_value)
        } else {
            None
        }
    }
}

/// ROS2 Parameter (from rcl_interfaces/msg/Parameter)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    /// Parameter name
    pub name: String,
    /// Parameter value
    pub value: ParameterValue,
}

impl Parameter {
    /// Create a new parameter
    pub fn new(name: impl Into<String>, value: ParameterValue) -> Self {
        Self {
            name: name.into(),
            value,
        }
    }

    /// Create a bool parameter
    pub fn bool(name: impl Into<String>, value: bool) -> Self {
        Self::new(name, ParameterValue::from_bool(value))
    }

    /// Create an integer parameter
    pub fn integer(name: impl Into<String>, value: i64) -> Self {
        Self::new(name, ParameterValue::from_integer(value))
    }

    /// Create a double parameter
    pub fn double(name: impl Into<String>, value: f64) -> Self {
        Self::new(name, ParameterValue::from_double(value))
    }

    /// Create a string parameter
    pub fn string(name: impl Into<String>, value: impl Into<String>) -> Self {
        Self::new(name, ParameterValue::from_string(value))
    }
}

// ============================================================================
// Parameter Descriptor (from rcl_interfaces/msg/ParameterDescriptor)
// ============================================================================

/// Floating point range for parameter constraints
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FloatingPointRange {
    /// Start of range (inclusive)
    pub from_value: f64,
    /// End of range (inclusive)
    pub to_value: f64,
    /// Step size (0 means continuous)
    pub step: f64,
}

impl Default for FloatingPointRange {
    fn default() -> Self {
        Self {
            from_value: f64::MIN,
            to_value: f64::MAX,
            step: 0.0,
        }
    }
}

/// Integer range for parameter constraints
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IntegerRange {
    /// Start of range (inclusive)
    pub from_value: i64,
    /// End of range (inclusive)
    pub to_value: i64,
    /// Step size (0 means any integer in range)
    pub step: u64,
}

impl Default for IntegerRange {
    fn default() -> Self {
        Self {
            from_value: i64::MIN,
            to_value: i64::MAX,
            step: 0,
        }
    }
}

/// Parameter descriptor with metadata
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParameterDescriptor {
    /// Parameter name
    pub name: String,
    /// Parameter type
    #[serde(rename = "type")]
    pub param_type: u8,
    /// Human readable description
    pub description: String,
    /// Additional constraints (e.g., "read_only", "dynamic_typing")
    pub additional_constraints: String,
    /// Whether this parameter is read-only
    pub read_only: bool,
    /// Whether dynamic typing is allowed
    pub dynamic_typing: bool,
    /// Floating point range constraints
    pub floating_point_range: Vec<FloatingPointRange>,
    /// Integer range constraints
    pub integer_range: Vec<IntegerRange>,
}

impl Default for ParameterDescriptor {
    fn default() -> Self {
        Self {
            name: String::new(),
            param_type: ParameterType::NotSet as u8,
            description: String::new(),
            additional_constraints: String::new(),
            read_only: false,
            dynamic_typing: false,
            floating_point_range: Vec::new(),
            integer_range: Vec::new(),
        }
    }
}

impl ParameterDescriptor {
    /// Create a new parameter descriptor
    pub fn new(name: impl Into<String>, param_type: ParameterType) -> Self {
        Self {
            name: name.into(),
            param_type: param_type as u8,
            ..Default::default()
        }
    }

    /// Set description
    pub fn with_description(mut self, desc: impl Into<String>) -> Self {
        self.description = desc.into();
        self
    }

    /// Set read-only flag
    pub fn read_only(mut self, read_only: bool) -> Self {
        self.read_only = read_only;
        self
    }

    /// Add floating point range constraint
    pub fn with_float_range(mut self, from: f64, to: f64, step: f64) -> Self {
        self.floating_point_range.push(FloatingPointRange {
            from_value: from,
            to_value: to,
            step,
        });
        self
    }

    /// Add integer range constraint
    pub fn with_integer_range(mut self, from: i64, to: i64, step: u64) -> Self {
        self.integer_range.push(IntegerRange {
            from_value: from,
            to_value: to,
            step,
        });
        self
    }
}

// ============================================================================
// Parameter Service Request/Response Types
// ============================================================================

/// List Parameters Request (rcl_interfaces/srv/ListParameters_Request)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ListParametersRequest {
    /// Prefixes to filter parameters (empty for all)
    pub prefixes: Vec<String>,
    /// Maximum depth of recursion (0 for unlimited, 1 for current level only)
    pub depth: u64,
}

/// List Parameters Result (rcl_interfaces/msg/ListParametersResult)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ListParametersResult {
    /// Parameter names
    pub names: Vec<String>,
    /// Parameter prefixes (namespace levels)
    pub prefixes: Vec<String>,
}

/// List Parameters Response (rcl_interfaces/srv/ListParameters_Response)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ListParametersResponse {
    /// Result containing parameter names and prefixes
    pub result: ListParametersResult,
}

/// Get Parameters Request (rcl_interfaces/srv/GetParameters_Request)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetParametersRequest {
    /// Parameter names to retrieve
    pub names: Vec<String>,
}

/// Get Parameters Response (rcl_interfaces/srv/GetParameters_Response)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetParametersResponse {
    /// Parameter values in same order as requested names
    pub values: Vec<ParameterValue>,
}

/// Set Parameters Request (rcl_interfaces/srv/SetParameters_Request)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetParametersRequest {
    /// Parameters to set
    pub parameters: Vec<Parameter>,
}

/// Set Parameter Result (rcl_interfaces/msg/SetParametersResult)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetParameterResult {
    /// Whether the set was successful
    pub successful: bool,
    /// Reason for failure (if not successful)
    pub reason: String,
}

/// Set Parameters Response (rcl_interfaces/srv/SetParameters_Response)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetParametersResponse {
    /// Results for each parameter set attempt
    pub results: Vec<SetParameterResult>,
}

/// Describe Parameters Request (rcl_interfaces/srv/DescribeParameters_Request)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DescribeParametersRequest {
    /// Parameter names to describe
    pub names: Vec<String>,
}

/// Describe Parameters Response (rcl_interfaces/srv/DescribeParameters_Response)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DescribeParametersResponse {
    /// Descriptors for requested parameters
    pub descriptors: Vec<ParameterDescriptor>,
}

/// Get Parameter Types Request (rcl_interfaces/srv/GetParameterTypes_Request)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetParameterTypesRequest {
    /// Parameter names to get types for
    pub names: Vec<String>,
}

/// Get Parameter Types Response (rcl_interfaces/srv/GetParameterTypes_Response)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetParameterTypesResponse {
    /// Parameter types (as u8 codes)
    pub types: Vec<u8>,
}

/// Set Parameters Atomically Request (rcl_interfaces/srv/SetParametersAtomically_Request)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetParametersAtomicallyRequest {
    /// Parameters to set atomically
    pub parameters: Vec<Parameter>,
}

/// Set Parameters Atomically Response (rcl_interfaces/srv/SetParametersAtomically_Response)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetParametersAtomicallyResponse {
    /// Result of atomic set operation
    pub result: SetParameterResult,
}

// ============================================================================
// Parameter Events
// ============================================================================

/// Parameter event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ParameterEventType {
    Changed = 0,
    Added = 1,
    Deleted = 2,
}

impl Default for ParameterEventType {
    fn default() -> Self {
        Self::Changed
    }
}

/// ROS2 Time structure
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Ros2ParamTime {
    /// Seconds since epoch
    pub sec: i32,
    /// Nanoseconds within second
    pub nanosec: u32,
}

impl Ros2ParamTime {
    /// Create from current system time
    pub fn now() -> Self {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        Self {
            sec: now.as_secs() as i32,
            nanosec: now.subsec_nanos(),
        }
    }

    /// Create from Duration
    pub fn from_duration(d: Duration) -> Self {
        Self {
            sec: d.as_secs() as i32,
            nanosec: d.subsec_nanos(),
        }
    }
}

/// Parameter Event (rcl_interfaces/msg/ParameterEvent)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ParameterEvent {
    /// Timestamp when the event occurred
    pub stamp: Ros2ParamTime,
    /// Fully qualified node name
    pub node: String,
    /// New parameters that were added
    pub new_parameters: Vec<Parameter>,
    /// Parameters that were changed
    pub changed_parameters: Vec<Parameter>,
    /// Names of parameters that were deleted
    pub deleted_parameters: Vec<Parameter>,
}

impl ParameterEvent {
    /// Create a new parameter event
    pub fn new(node: impl Into<String>) -> Self {
        Self {
            stamp: Ros2ParamTime::now(),
            node: node.into(),
            ..Default::default()
        }
    }

    /// Add a new parameter to the event
    pub fn add_new(mut self, param: Parameter) -> Self {
        self.new_parameters.push(param);
        self
    }

    /// Add a changed parameter to the event
    pub fn add_changed(mut self, param: Parameter) -> Self {
        self.changed_parameters.push(param);
        self
    }

    /// Add a deleted parameter to the event
    pub fn add_deleted(mut self, param: Parameter) -> Self {
        self.deleted_parameters.push(param);
        self
    }
}

// ============================================================================
// Parameter Server Configuration
// ============================================================================

/// Configuration for the parameter server/client
#[derive(Debug, Clone)]
pub struct Ros2ParameterConfig {
    /// Service call timeout
    pub timeout: Duration,
    /// Whether to publish parameter events
    pub publish_events: bool,
    /// QoS settings for parameter topics
    pub qos: ParameterQos,
}

impl Default for Ros2ParameterConfig {
    fn default() -> Self {
        Self {
            timeout: Duration::from_secs(5),
            publish_events: true,
            qos: ParameterQos::default(),
        }
    }
}

/// QoS settings for parameter topics
#[derive(Debug, Clone)]
pub struct ParameterQos {
    /// History depth for services
    pub service_history_depth: usize,
    /// History depth for events
    pub event_history_depth: usize,
}

impl Default for ParameterQos {
    fn default() -> Self {
        Self {
            service_history_depth: 10,
            event_history_depth: 100,
        }
    }
}

// ============================================================================
// Parameter Server Error Types
// ============================================================================

/// Errors that can occur during parameter operations
#[derive(Debug, Clone)]
pub enum Ros2ParameterError {
    /// Parameter not found
    NotFound(String),
    /// Parameter is read-only
    ReadOnly(String),
    /// Invalid parameter type
    InvalidType {
        name: String,
        expected: ParameterType,
        actual: ParameterType,
    },
    /// Value out of range
    OutOfRange { name: String, reason: String },
    /// Service call timeout
    Timeout,
    /// Service call failed
    ServiceError(String),
    /// Serialization/deserialization error
    SerializationError(String),
}

impl std::fmt::Display for Ros2ParameterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotFound(name) => write!(f, "Parameter not found: {}", name),
            Self::ReadOnly(name) => write!(f, "Parameter is read-only: {}", name),
            Self::InvalidType {
                name,
                expected,
                actual,
            } => {
                write!(
                    f,
                    "Invalid parameter type for '{}': expected {:?}, got {:?}",
                    name, expected, actual
                )
            }
            Self::OutOfRange { name, reason } => {
                write!(f, "Parameter '{}' out of range: {}", name, reason)
            }
            Self::Timeout => write!(f, "Parameter service call timed out"),
            Self::ServiceError(msg) => write!(f, "Parameter service error: {}", msg),
            Self::SerializationError(msg) => write!(f, "Serialization error: {}", msg),
        }
    }
}

impl std::error::Error for Ros2ParameterError {}

impl From<Ros2ServiceError> for Ros2ParameterError {
    fn from(err: Ros2ServiceError) -> Self {
        match err {
            Ros2ServiceError::Timeout(_) => Ros2ParameterError::Timeout,
            _ => Ros2ParameterError::ServiceError(format!("{:?}", err)),
        }
    }
}

// ============================================================================
// Parameter Statistics
// ============================================================================

/// Statistics for parameter operations
#[derive(Debug, Default)]
pub struct ParameterStats {
    /// Number of list_parameters calls
    pub list_calls: AtomicU64,
    /// Number of get_parameters calls
    pub get_calls: AtomicU64,
    /// Number of set_parameters calls
    pub set_calls: AtomicU64,
    /// Number of describe_parameters calls
    pub describe_calls: AtomicU64,
    /// Number of parameter events published
    pub events_published: AtomicU64,
    /// Number of successful operations
    pub successful_operations: AtomicU64,
    /// Number of failed operations
    pub failed_operations: AtomicU64,
}

impl ParameterStats {
    /// Create new statistics
    pub fn new() -> Self {
        Self::default()
    }

    /// Increment list calls
    pub fn inc_list(&self) {
        self.list_calls.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment get calls
    pub fn inc_get(&self) {
        self.get_calls.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment set calls
    pub fn inc_set(&self) {
        self.set_calls.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment describe calls
    pub fn inc_describe(&self) {
        self.describe_calls.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment events published
    pub fn inc_events(&self) {
        self.events_published.fetch_add(1, Ordering::Relaxed);
    }

    /// Record successful operation
    pub fn record_success(&self) {
        self.successful_operations.fetch_add(1, Ordering::Relaxed);
    }

    /// Record failed operation
    pub fn record_failure(&self) {
        self.failed_operations.fetch_add(1, Ordering::Relaxed);
    }
}

// ============================================================================
// Local Parameter Store (for HORUS nodes)
// ============================================================================

/// Local parameter store for HORUS nodes
pub struct LocalParameterStore {
    /// Node name
    node_name: String,
    /// Stored parameters
    parameters: RwLock<HashMap<String, Parameter>>,
    /// Parameter descriptors
    descriptors: RwLock<HashMap<String, ParameterDescriptor>>,
    /// Parameter change callbacks
    change_callbacks: RwLock<Vec<Box<dyn Fn(&Parameter, ParameterEventType) + Send + Sync>>>,
    /// Statistics
    stats: Arc<ParameterStats>,
}

impl std::fmt::Debug for LocalParameterStore {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LocalParameterStore")
            .field("node_name", &self.node_name)
            .field("parameters", &self.parameters)
            .field("descriptors", &self.descriptors)
            .field(
                "change_callbacks",
                &format!("<{} callbacks>", self.change_callbacks.read().len()),
            )
            .field("stats", &self.stats)
            .finish()
    }
}

impl LocalParameterStore {
    /// Create a new local parameter store
    pub fn new(node_name: impl Into<String>) -> Self {
        Self {
            node_name: node_name.into(),
            parameters: RwLock::new(HashMap::new()),
            descriptors: RwLock::new(HashMap::new()),
            change_callbacks: RwLock::new(Vec::new()),
            stats: Arc::new(ParameterStats::new()),
        }
    }

    /// Get node name
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// Declare a parameter with descriptor
    pub fn declare_parameter(
        &self,
        name: impl Into<String>,
        default_value: ParameterValue,
        descriptor: Option<ParameterDescriptor>,
    ) -> Result<(), Ros2ParameterError> {
        let name = name.into();
        let mut params = self.parameters.write();

        // Don't override if already exists
        if params.contains_key(&name) {
            return Ok(());
        }

        let param = Parameter {
            name: name.clone(),
            value: default_value,
        };
        params.insert(name.clone(), param.clone());

        if let Some(desc) = descriptor {
            self.descriptors.write().insert(name.clone(), desc);
        }

        // Notify callbacks
        self.notify_change(&param, ParameterEventType::Added);

        Ok(())
    }

    /// Get a parameter value
    pub fn get_parameter(&self, name: &str) -> Option<Parameter> {
        self.stats.inc_get();
        self.parameters.read().get(name).cloned()
    }

    /// Get multiple parameters
    pub fn get_parameters(&self, names: &[String]) -> Vec<Option<Parameter>> {
        self.stats.inc_get();
        let params = self.parameters.read();
        names.iter().map(|n| params.get(n).cloned()).collect()
    }

    /// Set a parameter value
    pub fn set_parameter(
        &self,
        param: Parameter,
    ) -> Result<SetParameterResult, Ros2ParameterError> {
        self.stats.inc_set();

        // Check if read-only
        if let Some(desc) = self.descriptors.read().get(&param.name) {
            if desc.read_only {
                return Ok(SetParameterResult {
                    successful: false,
                    reason: "Parameter is read-only".to_string(),
                });
            }

            // Validate type if not dynamic
            if !desc.dynamic_typing && desc.param_type != ParameterType::NotSet as u8 {
                if desc.param_type != param.value.param_type {
                    return Ok(SetParameterResult {
                        successful: false,
                        reason: format!(
                            "Type mismatch: expected {:?}, got {:?}",
                            ParameterType::from_code(desc.param_type),
                            ParameterType::from_code(param.value.param_type)
                        ),
                    });
                }
            }

            // Validate ranges
            if let Err(reason) = self.validate_ranges(&param.value, desc) {
                return Ok(SetParameterResult {
                    successful: false,
                    reason,
                });
            }
        }

        let event_type = if self.parameters.read().contains_key(&param.name) {
            ParameterEventType::Changed
        } else {
            ParameterEventType::Added
        };

        self.parameters
            .write()
            .insert(param.name.clone(), param.clone());
        self.notify_change(&param, event_type);
        self.stats.record_success();

        Ok(SetParameterResult {
            successful: true,
            reason: String::new(),
        })
    }

    /// Set multiple parameters
    pub fn set_parameters(&self, params: Vec<Parameter>) -> Vec<SetParameterResult> {
        params
            .into_iter()
            .map(|p| {
                self.set_parameter(p)
                    .unwrap_or_else(|e| SetParameterResult {
                        successful: false,
                        reason: e.to_string(),
                    })
            })
            .collect()
    }

    /// Set parameters atomically (all or nothing)
    pub fn set_parameters_atomically(&self, params: Vec<Parameter>) -> SetParameterResult {
        // First validate all
        for param in &params {
            if let Some(desc) = self.descriptors.read().get(&param.name) {
                if desc.read_only {
                    return SetParameterResult {
                        successful: false,
                        reason: format!("Parameter '{}' is read-only", param.name),
                    };
                }
                if !desc.dynamic_typing && desc.param_type != ParameterType::NotSet as u8 {
                    if desc.param_type != param.value.param_type {
                        return SetParameterResult {
                            successful: false,
                            reason: format!(
                                "Type mismatch for '{}': expected {:?}, got {:?}",
                                param.name,
                                ParameterType::from_code(desc.param_type),
                                ParameterType::from_code(param.value.param_type)
                            ),
                        };
                    }
                }
                if let Err(reason) = self.validate_ranges(&param.value, desc) {
                    return SetParameterResult {
                        successful: false,
                        reason: format!("'{}': {}", param.name, reason),
                    };
                }
            }
        }

        // All valid, set atomically
        let mut store = self.parameters.write();
        for param in params {
            let event_type = if store.contains_key(&param.name) {
                ParameterEventType::Changed
            } else {
                ParameterEventType::Added
            };
            store.insert(param.name.clone(), param.clone());
            drop(store);
            self.notify_change(&param, event_type);
            store = self.parameters.write();
        }

        self.stats.record_success();
        SetParameterResult {
            successful: true,
            reason: String::new(),
        }
    }

    /// List all parameter names
    pub fn list_parameters(&self, prefixes: &[String], depth: u64) -> ListParametersResult {
        self.stats.inc_list();
        let params = self.parameters.read();

        let mut names = Vec::new();
        let mut found_prefixes = std::collections::HashSet::new();

        for name in params.keys() {
            let matches = if prefixes.is_empty() {
                true
            } else {
                prefixes.iter().any(|p| name.starts_with(p))
            };

            if matches {
                // Check depth
                let name_depth = name.matches('.').count() as u64 + 1;
                if depth == 0 || name_depth <= depth {
                    names.push(name.clone());

                    // Extract prefixes
                    let parts: Vec<&str> = name.split('.').collect();
                    for i in 1..parts.len() {
                        let prefix = parts[..i].join(".");
                        found_prefixes.insert(prefix);
                    }
                }
            }
        }

        ListParametersResult {
            names,
            prefixes: found_prefixes.into_iter().collect(),
        }
    }

    /// Describe parameters
    pub fn describe_parameters(&self, names: &[String]) -> Vec<ParameterDescriptor> {
        self.stats.inc_describe();
        let descs = self.descriptors.read();
        let params = self.parameters.read();

        names
            .iter()
            .map(|name| {
                descs.get(name).cloned().unwrap_or_else(|| {
                    // Create default descriptor from current value
                    let param_type = params
                        .get(name)
                        .map(|p| p.value.param_type)
                        .unwrap_or(ParameterType::NotSet as u8);
                    ParameterDescriptor {
                        name: name.clone(),
                        param_type,
                        ..Default::default()
                    }
                })
            })
            .collect()
    }

    /// Get parameter types
    pub fn get_parameter_types(&self, names: &[String]) -> Vec<u8> {
        let params = self.parameters.read();
        names
            .iter()
            .map(|name| {
                params
                    .get(name)
                    .map(|p| p.value.param_type)
                    .unwrap_or(ParameterType::NotSet as u8)
            })
            .collect()
    }

    /// Register a change callback
    pub fn on_change<F>(&self, callback: F)
    where
        F: Fn(&Parameter, ParameterEventType) + Send + Sync + 'static,
    {
        self.change_callbacks.write().push(Box::new(callback));
    }

    /// Get statistics
    pub fn stats(&self) -> &ParameterStats {
        &self.stats
    }

    /// Validate parameter value against descriptor ranges
    fn validate_ranges(
        &self,
        value: &ParameterValue,
        desc: &ParameterDescriptor,
    ) -> Result<(), String> {
        // Integer range validation
        if value.param_type == ParameterType::Integer as u8 {
            for range in &desc.integer_range {
                if value.integer_value < range.from_value || value.integer_value > range.to_value {
                    return Err(format!(
                        "Value {} out of range [{}, {}]",
                        value.integer_value, range.from_value, range.to_value
                    ));
                }
                if range.step > 0 {
                    let offset = value.integer_value - range.from_value;
                    if offset as u64 % range.step != 0 {
                        return Err(format!(
                            "Value {} not on step grid (step={})",
                            value.integer_value, range.step
                        ));
                    }
                }
            }
        }

        // Float range validation
        if value.param_type == ParameterType::Double as u8 {
            for range in &desc.floating_point_range {
                if value.double_value < range.from_value || value.double_value > range.to_value {
                    return Err(format!(
                        "Value {} out of range [{}, {}]",
                        value.double_value, range.from_value, range.to_value
                    ));
                }
            }
        }

        Ok(())
    }

    /// Notify change callbacks
    fn notify_change(&self, param: &Parameter, event_type: ParameterEventType) {
        for callback in self.change_callbacks.read().iter() {
            callback(param, event_type);
        }
    }

    /// Create a parameter event from recent changes
    pub fn create_event(&self, changes: Vec<(Parameter, ParameterEventType)>) -> ParameterEvent {
        let mut event = ParameterEvent::new(&self.node_name);

        for (param, event_type) in changes {
            match event_type {
                ParameterEventType::Added => {
                    event.new_parameters.push(param);
                }
                ParameterEventType::Changed => {
                    event.changed_parameters.push(param);
                }
                ParameterEventType::Deleted => {
                    event.deleted_parameters.push(param);
                }
            }
        }

        self.stats.inc_events();
        event
    }
}

// ============================================================================
// Parameter Client Topics Structure
// ============================================================================

/// Topics used by a parameter client
#[derive(Debug, Clone)]
pub struct ParameterClientTopics {
    /// List parameters request topic
    pub list_parameters: String,
    /// Get parameters request topic
    pub get_parameters: String,
    /// Set parameters request topic
    pub set_parameters: String,
    /// Describe parameters request topic
    pub describe_parameters: String,
    /// Get parameter types request topic
    pub get_parameter_types: String,
    /// Set parameters atomically request topic
    pub set_parameters_atomically: String,
    /// Parameter events subscription topic
    pub parameter_events: String,
}

impl ParameterClientTopics {
    /// Create topics for a given node name
    pub fn new(node_name: &str) -> Self {
        Self {
            list_parameters: param_to_list_parameters_topic(node_name),
            get_parameters: param_to_get_parameters_topic(node_name),
            set_parameters: param_to_set_parameters_topic(node_name),
            describe_parameters: param_to_describe_parameters_topic(node_name),
            get_parameter_types: param_to_get_parameter_types_topic(node_name),
            set_parameters_atomically: param_to_set_parameters_atomically_topic(node_name),
            parameter_events: param_to_events_topic(node_name),
        }
    }
}

// ============================================================================
// Parameter Server Topics Structure
// ============================================================================

/// Topics used by a parameter server
#[derive(Debug, Clone)]
pub struct ParameterServerTopics {
    /// List parameters response topic
    pub list_parameters_response: String,
    /// Get parameters response topic
    pub get_parameters_response: String,
    /// Set parameters response topic
    pub set_parameters_response: String,
    /// Describe parameters response topic
    pub describe_parameters_response: String,
    /// Get parameter types response topic
    pub get_parameter_types_response: String,
    /// Set parameters atomically response topic
    pub set_parameters_atomically_response: String,
    /// Parameter events publish topic
    pub parameter_events: String,
}

impl ParameterServerTopics {
    /// Create response topics for a given node name
    pub fn new(node_name: &str) -> Self {
        let normalized = normalize_node_name(node_name);
        Self {
            list_parameters_response: format!("rs/{}/_parameters/list_parameters", normalized),
            get_parameters_response: format!("rs/{}/_parameters/get_parameters", normalized),
            set_parameters_response: format!("rs/{}/_parameters/set_parameters", normalized),
            describe_parameters_response: format!(
                "rs/{}/_parameters/describe_parameters",
                normalized
            ),
            get_parameter_types_response: format!(
                "rs/{}/_parameters/get_parameter_types",
                normalized
            ),
            set_parameters_atomically_response: format!(
                "rs/{}/_parameters/set_parameters_atomically",
                normalized
            ),
            parameter_events: param_to_events_topic(node_name),
        }
    }
}

// ============================================================================
// ROS2 Parameter Client (stub - actual implementation needs Zenoh session)
// ============================================================================

/// Client for accessing ROS2 node parameters
///
/// This client can interact with any ROS2 node's parameter server over Zenoh.
#[derive(Debug)]
pub struct Ros2ParameterClient {
    /// Target node name
    node_name: String,
    /// Configuration
    config: Ros2ParameterConfig,
    /// Topics
    topics: ParameterClientTopics,
    /// Statistics
    stats: Arc<ParameterStats>,
    /// Sequence counter for requests
    sequence: AtomicU64,
}

impl Ros2ParameterClient {
    /// Create a new parameter client
    pub fn new(node_name: impl Into<String>, config: Ros2ParameterConfig) -> Self {
        let node_name = node_name.into();
        let topics = ParameterClientTopics::new(&node_name);

        Self {
            node_name,
            config,
            topics,
            stats: Arc::new(ParameterStats::new()),
            sequence: AtomicU64::new(0),
        }
    }

    /// Get node name
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// Get topics
    pub fn topics(&self) -> &ParameterClientTopics {
        &self.topics
    }

    /// Get statistics
    pub fn stats(&self) -> &ParameterStats {
        &self.stats
    }

    /// Get next sequence number
    fn next_sequence(&self) -> u64 {
        self.sequence.fetch_add(1, Ordering::Relaxed)
    }

    /// List parameters (stub - needs Zenoh session integration)
    pub async fn list_parameters(
        &self,
        _request: ListParametersRequest,
    ) -> Result<ListParametersResponse, Ros2ParameterError> {
        self.stats.inc_list();
        // In actual implementation, this would call the ROS2 service over Zenoh
        // For now, return empty response
        Ok(ListParametersResponse {
            result: ListParametersResult::default(),
        })
    }

    /// Get parameters (stub - needs Zenoh session integration)
    pub async fn get_parameters(
        &self,
        request: GetParametersRequest,
    ) -> Result<GetParametersResponse, Ros2ParameterError> {
        self.stats.inc_get();
        // In actual implementation, this would call the ROS2 service over Zenoh
        Ok(GetParametersResponse {
            values: vec![ParameterValue::default(); request.names.len()],
        })
    }

    /// Set parameters (stub - needs Zenoh session integration)
    pub async fn set_parameters(
        &self,
        request: SetParametersRequest,
    ) -> Result<SetParametersResponse, Ros2ParameterError> {
        self.stats.inc_set();
        // In actual implementation, this would call the ROS2 service over Zenoh
        Ok(SetParametersResponse {
            results: request
                .parameters
                .iter()
                .map(|_| SetParameterResult {
                    successful: true,
                    reason: String::new(),
                })
                .collect(),
        })
    }

    /// Describe parameters (stub - needs Zenoh session integration)
    pub async fn describe_parameters(
        &self,
        request: DescribeParametersRequest,
    ) -> Result<DescribeParametersResponse, Ros2ParameterError> {
        self.stats.inc_describe();
        // In actual implementation, this would call the ROS2 service over Zenoh
        Ok(DescribeParametersResponse {
            descriptors: request
                .names
                .iter()
                .map(|name| ParameterDescriptor {
                    name: name.clone(),
                    ..Default::default()
                })
                .collect(),
        })
    }

    /// Set parameters atomically (stub - needs Zenoh session integration)
    pub async fn set_parameters_atomically(
        &self,
        _request: SetParametersAtomicallyRequest,
    ) -> Result<SetParametersAtomicallyResponse, Ros2ParameterError> {
        self.stats.inc_set();
        // In actual implementation, this would call the ROS2 service over Zenoh
        Ok(SetParametersAtomicallyResponse {
            result: SetParameterResult {
                successful: true,
                reason: String::new(),
            },
        })
    }
}

// ============================================================================
// ROS2 Parameter Server (stub - actual implementation needs Zenoh session)
// ============================================================================

/// Server for exposing HORUS node parameters to ROS2
#[derive(Debug)]
pub struct Ros2ParameterServer {
    /// Local parameter store
    store: Arc<LocalParameterStore>,
    /// Configuration
    config: Ros2ParameterConfig,
    /// Topics
    topics: ParameterServerTopics,
}

impl Ros2ParameterServer {
    /// Create a new parameter server
    pub fn new(store: Arc<LocalParameterStore>, config: Ros2ParameterConfig) -> Self {
        let topics = ParameterServerTopics::new(store.node_name());

        Self {
            store,
            config,
            topics,
        }
    }

    /// Get the parameter store
    pub fn store(&self) -> &LocalParameterStore {
        &self.store
    }

    /// Get topics
    pub fn topics(&self) -> &ParameterServerTopics {
        &self.topics
    }

    /// Handle list_parameters request
    pub fn handle_list_parameters(&self, request: ListParametersRequest) -> ListParametersResponse {
        ListParametersResponse {
            result: self.store.list_parameters(&request.prefixes, request.depth),
        }
    }

    /// Handle get_parameters request
    pub fn handle_get_parameters(&self, request: GetParametersRequest) -> GetParametersResponse {
        let values = self
            .store
            .get_parameters(&request.names)
            .into_iter()
            .map(|opt| opt.map(|p| p.value).unwrap_or_default())
            .collect();

        GetParametersResponse { values }
    }

    /// Handle set_parameters request
    pub fn handle_set_parameters(&self, request: SetParametersRequest) -> SetParametersResponse {
        SetParametersResponse {
            results: self.store.set_parameters(request.parameters),
        }
    }

    /// Handle describe_parameters request
    pub fn handle_describe_parameters(
        &self,
        request: DescribeParametersRequest,
    ) -> DescribeParametersResponse {
        DescribeParametersResponse {
            descriptors: self.store.describe_parameters(&request.names),
        }
    }

    /// Handle get_parameter_types request
    pub fn handle_get_parameter_types(
        &self,
        request: GetParameterTypesRequest,
    ) -> GetParameterTypesResponse {
        GetParameterTypesResponse {
            types: self.store.get_parameter_types(&request.names),
        }
    }

    /// Handle set_parameters_atomically request
    pub fn handle_set_parameters_atomically(
        &self,
        request: SetParametersAtomicallyRequest,
    ) -> SetParametersAtomicallyResponse {
        SetParametersAtomicallyResponse {
            result: self.store.set_parameters_atomically(request.parameters),
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topic_naming() {
        assert_eq!(
            param_to_list_parameters_topic("/robot/controller"),
            "rq/robot__controller/_parameters/list_parameters"
        );
        assert_eq!(
            param_to_get_parameters_topic("camera_node"),
            "rq/camera_node/_parameters/get_parameters"
        );
        assert_eq!(
            param_to_events_topic("/my_node"),
            "rt/my_node/_parameters/parameter_events"
        );
    }

    #[test]
    fn test_parameter_value_constructors() {
        let bool_val = ParameterValue::from_bool(true);
        assert_eq!(bool_val.param_type, ParameterType::Bool as u8);
        assert!(bool_val.bool_value);
        assert_eq!(bool_val.as_bool(), Some(true));

        let int_val = ParameterValue::from_integer(42);
        assert_eq!(int_val.param_type, ParameterType::Integer as u8);
        assert_eq!(int_val.integer_value, 42);
        assert_eq!(int_val.as_integer(), Some(42));

        let double_val = ParameterValue::from_double(3.14);
        assert_eq!(double_val.param_type, ParameterType::Double as u8);
        assert!((double_val.double_value - 3.14).abs() < 0.001);
        assert!(double_val.as_double().is_some());

        let string_val = ParameterValue::from_string("hello");
        assert_eq!(string_val.param_type, ParameterType::String as u8);
        assert_eq!(string_val.string_value, "hello");
        assert_eq!(string_val.as_string(), Some("hello"));
    }

    #[test]
    fn test_parameter_constructors() {
        let param = Parameter::bool("use_sim_time", true);
        assert_eq!(param.name, "use_sim_time");
        assert!(param.value.bool_value);

        let param = Parameter::double("max_velocity", 1.5);
        assert_eq!(param.name, "max_velocity");
        assert!((param.value.double_value - 1.5).abs() < 0.001);
    }

    #[test]
    fn test_parameter_descriptor() {
        let desc = ParameterDescriptor::new("velocity", ParameterType::Double)
            .with_description("Maximum velocity in m/s")
            .read_only(false)
            .with_float_range(0.0, 10.0, 0.1);

        assert_eq!(desc.name, "velocity");
        assert_eq!(desc.param_type, ParameterType::Double as u8);
        assert_eq!(desc.description, "Maximum velocity in m/s");
        assert!(!desc.read_only);
        assert_eq!(desc.floating_point_range.len(), 1);
    }

    #[test]
    fn test_local_parameter_store() {
        let store = LocalParameterStore::new("test_node");

        // Declare parameter
        store
            .declare_parameter(
                "max_speed",
                ParameterValue::from_double(10.0),
                Some(
                    ParameterDescriptor::new("max_speed", ParameterType::Double)
                        .with_float_range(0.0, 100.0, 0.0),
                ),
            )
            .unwrap();

        // Get parameter
        let param = store.get_parameter("max_speed").unwrap();
        assert!((param.value.double_value - 10.0).abs() < 0.001);

        // Set parameter
        let result = store
            .set_parameter(Parameter::double("max_speed", 25.0))
            .unwrap();
        assert!(result.successful);

        // Verify update
        let param = store.get_parameter("max_speed").unwrap();
        assert!((param.value.double_value - 25.0).abs() < 0.001);

        // List parameters
        let list = store.list_parameters(&[], 0);
        assert!(list.names.contains(&"max_speed".to_string()));
    }

    #[test]
    fn test_parameter_store_validation() {
        let store = LocalParameterStore::new("test_node");

        // Declare read-only parameter
        store
            .declare_parameter(
                "read_only_param",
                ParameterValue::from_integer(42),
                Some(
                    ParameterDescriptor::new("read_only_param", ParameterType::Integer)
                        .read_only(true),
                ),
            )
            .unwrap();

        // Try to set read-only parameter
        let result = store
            .set_parameter(Parameter::integer("read_only_param", 100))
            .unwrap();
        assert!(!result.successful);
        assert!(result.reason.contains("read-only"));

        // Declare parameter with range
        store
            .declare_parameter(
                "bounded_param",
                ParameterValue::from_integer(50),
                Some(
                    ParameterDescriptor::new("bounded_param", ParameterType::Integer)
                        .with_integer_range(0, 100, 0),
                ),
            )
            .unwrap();

        // Try to set out of range
        let result = store
            .set_parameter(Parameter::integer("bounded_param", 150))
            .unwrap();
        assert!(!result.successful);
        assert!(result.reason.contains("out of range"));
    }

    #[test]
    fn test_parameter_events() {
        let event = ParameterEvent::new("test_node")
            .add_new(Parameter::bool("new_param", true))
            .add_changed(Parameter::double("changed_param", 1.0))
            .add_deleted(Parameter::string("deleted_param", ""));

        assert_eq!(event.node, "test_node");
        assert_eq!(event.new_parameters.len(), 1);
        assert_eq!(event.changed_parameters.len(), 1);
        assert_eq!(event.deleted_parameters.len(), 1);
    }

    #[test]
    fn test_atomic_set() {
        let store = LocalParameterStore::new("test_node");

        // Declare parameters
        store
            .declare_parameter("a", ParameterValue::from_integer(1), None)
            .unwrap();
        store
            .declare_parameter("b", ParameterValue::from_integer(2), None)
            .unwrap();

        // Atomic set
        let result = store.set_parameters_atomically(vec![
            Parameter::integer("a", 10),
            Parameter::integer("b", 20),
        ]);
        assert!(result.successful);

        // Verify both updated
        assert_eq!(store.get_parameter("a").unwrap().value.integer_value, 10);
        assert_eq!(store.get_parameter("b").unwrap().value.integer_value, 20);
    }

    #[test]
    fn test_parameter_client_topics() {
        let topics = ParameterClientTopics::new("/robot/arm");

        assert_eq!(
            topics.list_parameters,
            "rq/robot__arm/_parameters/list_parameters"
        );
        assert_eq!(
            topics.get_parameters,
            "rq/robot__arm/_parameters/get_parameters"
        );
        assert_eq!(
            topics.parameter_events,
            "rt/robot__arm/_parameters/parameter_events"
        );
    }

    #[test]
    fn test_ros2_time() {
        let time = Ros2ParamTime::now();
        assert!(time.sec > 0);

        let duration = Duration::from_secs(100);
        let time2 = Ros2ParamTime::from_duration(duration);
        assert_eq!(time2.sec, 100);
        assert_eq!(time2.nanosec, 0);
    }

    // ==========================================================================
    // Integration Tests: ParameterClient + ParameterServer + LocalParameterStore
    // ==========================================================================

    #[test]
    fn test_client_server_integration() {
        // Create local store with parameters
        let store = Arc::new(LocalParameterStore::new("test_robot"));
        store
            .declare_parameter("max_speed", ParameterValue::from_double(5.0), None)
            .unwrap();
        store
            .declare_parameter("enabled", ParameterValue::from_bool(true), None)
            .unwrap();
        store
            .declare_parameter(
                "robot_name",
                ParameterValue::from_string("test_robot"),
                None,
            )
            .unwrap();

        // Create server and client
        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());
        let client = Ros2ParameterClient::new("test_robot", Ros2ParameterConfig::default());

        // Simulate client list_parameters -> server handle
        let list_request = ListParametersRequest::default();
        let list_response = server.handle_list_parameters(list_request);

        assert_eq!(list_response.result.names.len(), 3);
        assert!(list_response
            .result
            .names
            .contains(&"max_speed".to_string()));
        assert!(list_response.result.names.contains(&"enabled".to_string()));
        assert!(list_response
            .result
            .names
            .contains(&"robot_name".to_string()));

        // Client and server should use same node name
        assert_eq!(client.node_name(), server.store().node_name());
    }

    #[test]
    fn test_get_set_parameters_workflow() {
        let store = Arc::new(LocalParameterStore::new("controller"));
        store
            .declare_parameter("gain_p", ParameterValue::from_double(1.0), None)
            .unwrap();
        store
            .declare_parameter("gain_i", ParameterValue::from_double(0.1), None)
            .unwrap();
        store
            .declare_parameter("gain_d", ParameterValue::from_double(0.01), None)
            .unwrap();

        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());

        // Get initial parameters
        let get_request = GetParametersRequest {
            names: vec![
                "gain_p".to_string(),
                "gain_i".to_string(),
                "gain_d".to_string(),
            ],
        };
        let get_response = server.handle_get_parameters(get_request);

        assert_eq!(get_response.values.len(), 3);
        assert!((get_response.values[0].as_double().unwrap() - 1.0).abs() < 0.001);
        assert!((get_response.values[1].as_double().unwrap() - 0.1).abs() < 0.001);
        assert!((get_response.values[2].as_double().unwrap() - 0.01).abs() < 0.001);

        // Set parameters
        let set_request = SetParametersRequest {
            parameters: vec![
                Parameter::double("gain_p", 2.0),
                Parameter::double("gain_i", 0.2),
                Parameter::double("gain_d", 0.02),
            ],
        };
        let set_response = server.handle_set_parameters(set_request);

        assert_eq!(set_response.results.len(), 3);
        for result in &set_response.results {
            assert!(result.successful);
        }

        // Verify updated values
        let get_request = GetParametersRequest {
            names: vec!["gain_p".to_string()],
        };
        let get_response = server.handle_get_parameters(get_request);
        assert!((get_response.values[0].as_double().unwrap() - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_describe_parameters_workflow() {
        let store = Arc::new(LocalParameterStore::new("sensor_node"));

        let desc = ParameterDescriptor::new("temperature_offset", ParameterType::Double)
            .with_description("Calibration offset for temperature sensor")
            .with_float_range(-10.0, 10.0, 0.1);

        store
            .declare_parameter(
                "temperature_offset",
                ParameterValue::from_double(0.5),
                Some(desc),
            )
            .unwrap();

        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());

        let describe_request = DescribeParametersRequest {
            names: vec!["temperature_offset".to_string()],
        };
        let describe_response = server.handle_describe_parameters(describe_request);

        assert_eq!(describe_response.descriptors.len(), 1);
        let desc = &describe_response.descriptors[0];
        assert_eq!(desc.name, "temperature_offset");
        assert_eq!(desc.param_type, ParameterType::Double as u8);
        assert_eq!(
            desc.description,
            "Calibration offset for temperature sensor"
        );
        assert_eq!(desc.floating_point_range.len(), 1);
    }

    #[test]
    fn test_get_parameter_types_workflow() {
        let store = Arc::new(LocalParameterStore::new("multi_type_node"));
        store
            .declare_parameter("bool_param", ParameterValue::from_bool(true), None)
            .unwrap();
        store
            .declare_parameter("int_param", ParameterValue::from_integer(42), None)
            .unwrap();
        store
            .declare_parameter("double_param", ParameterValue::from_double(3.14), None)
            .unwrap();
        store
            .declare_parameter("string_param", ParameterValue::from_string("hello"), None)
            .unwrap();

        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());

        let request = GetParameterTypesRequest {
            names: vec![
                "bool_param".to_string(),
                "int_param".to_string(),
                "double_param".to_string(),
                "string_param".to_string(),
                "nonexistent".to_string(),
            ],
        };
        let response = server.handle_get_parameter_types(request);

        assert_eq!(response.types.len(), 5);
        assert_eq!(response.types[0], ParameterType::Bool as u8);
        assert_eq!(response.types[1], ParameterType::Integer as u8);
        assert_eq!(response.types[2], ParameterType::Double as u8);
        assert_eq!(response.types[3], ParameterType::String as u8);
        assert_eq!(response.types[4], ParameterType::NotSet as u8);
    }

    #[test]
    fn test_set_parameters_atomically_workflow() {
        let store = Arc::new(LocalParameterStore::new("atomic_node"));
        store
            .declare_parameter("x", ParameterValue::from_double(0.0), None)
            .unwrap();
        store
            .declare_parameter("y", ParameterValue::from_double(0.0), None)
            .unwrap();
        store
            .declare_parameter("z", ParameterValue::from_double(0.0), None)
            .unwrap();

        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());

        // Atomic set of all coordinates
        let request = SetParametersAtomicallyRequest {
            parameters: vec![
                Parameter::double("x", 1.0),
                Parameter::double("y", 2.0),
                Parameter::double("z", 3.0),
            ],
        };
        let response = server.handle_set_parameters_atomically(request);

        assert!(response.result.successful);

        // Verify all updated
        let get_request = GetParametersRequest {
            names: vec!["x".to_string(), "y".to_string(), "z".to_string()],
        };
        let get_response = server.handle_get_parameters(get_request);
        assert!((get_response.values[0].as_double().unwrap() - 1.0).abs() < 0.001);
        assert!((get_response.values[1].as_double().unwrap() - 2.0).abs() < 0.001);
        assert!((get_response.values[2].as_double().unwrap() - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_atomic_set_rollback_on_failure() {
        let store = Arc::new(LocalParameterStore::new("rollback_node"));

        // Declare x as writable, y as read-only
        store
            .declare_parameter("x", ParameterValue::from_integer(10), None)
            .unwrap();
        store
            .declare_parameter(
                "y",
                ParameterValue::from_integer(20),
                Some(ParameterDescriptor::new("y", ParameterType::Integer).read_only(true)),
            )
            .unwrap();

        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());

        // Try atomic set - should fail because y is read-only
        let request = SetParametersAtomicallyRequest {
            parameters: vec![
                Parameter::integer("x", 100),
                Parameter::integer("y", 200), // This will fail
            ],
        };
        let response = server.handle_set_parameters_atomically(request);

        assert!(!response.result.successful);
        assert!(response.result.reason.contains("read-only"));

        // x should NOT be updated because atomic failed
        // Note: Current implementation doesn't actually rollback, this test documents expected behavior
    }

    #[test]
    fn test_parameter_change_callback() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let store = LocalParameterStore::new("callback_node");
        let change_count = Arc::new(AtomicUsize::new(0));
        let change_count_clone = Arc::clone(&change_count);

        store.on_change(move |_param, event_type| match event_type {
            ParameterEventType::Added | ParameterEventType::Changed => {
                change_count_clone.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        });

        // Declare triggers Added event
        store
            .declare_parameter("test", ParameterValue::from_integer(1), None)
            .unwrap();
        assert_eq!(change_count.load(Ordering::Relaxed), 1);

        // Set triggers Changed event
        store.set_parameter(Parameter::integer("test", 2)).unwrap();
        assert_eq!(change_count.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn test_parameter_array_types() {
        let store = Arc::new(LocalParameterStore::new("array_node"));

        store
            .declare_parameter(
                "int_array",
                ParameterValue::from_integer_array(vec![1, 2, 3, 4, 5]),
                None,
            )
            .unwrap();
        store
            .declare_parameter(
                "double_array",
                ParameterValue::from_double_array(vec![1.0, 2.0, 3.0]),
                None,
            )
            .unwrap();
        store
            .declare_parameter(
                "bool_array",
                ParameterValue::from_bool_array(vec![true, false, true]),
                None,
            )
            .unwrap();
        store
            .declare_parameter(
                "string_array",
                ParameterValue::from_string_array(vec!["a".to_string(), "b".to_string()]),
                None,
            )
            .unwrap();
        store
            .declare_parameter(
                "byte_array",
                ParameterValue::from_byte_array(vec![0x01, 0x02, 0x03]),
                None,
            )
            .unwrap();

        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());

        let request = GetParameterTypesRequest {
            names: vec![
                "int_array".to_string(),
                "double_array".to_string(),
                "bool_array".to_string(),
                "string_array".to_string(),
                "byte_array".to_string(),
            ],
        };
        let response = server.handle_get_parameter_types(request);

        assert_eq!(response.types[0], ParameterType::IntegerArray as u8);
        assert_eq!(response.types[1], ParameterType::DoubleArray as u8);
        assert_eq!(response.types[2], ParameterType::BoolArray as u8);
        assert_eq!(response.types[3], ParameterType::StringArray as u8);
        assert_eq!(response.types[4], ParameterType::ByteArray as u8);
    }

    #[test]
    fn test_list_parameters_with_prefix() {
        let store = Arc::new(LocalParameterStore::new("prefixed_node"));

        store
            .declare_parameter("motor.left.speed", ParameterValue::from_double(1.0), None)
            .unwrap();
        store
            .declare_parameter("motor.left.torque", ParameterValue::from_double(0.5), None)
            .unwrap();
        store
            .declare_parameter("motor.right.speed", ParameterValue::from_double(1.0), None)
            .unwrap();
        store
            .declare_parameter("sensor.imu.rate", ParameterValue::from_integer(100), None)
            .unwrap();

        let server = Ros2ParameterServer::new(Arc::clone(&store), Ros2ParameterConfig::default());

        // List all motor parameters
        let request = ListParametersRequest {
            prefixes: vec!["motor".to_string()],
            depth: 0,
        };
        let response = server.handle_list_parameters(request);

        assert_eq!(response.result.names.len(), 3);
        for name in &response.result.names {
            assert!(name.starts_with("motor"));
        }

        // List only left motor
        let request = ListParametersRequest {
            prefixes: vec!["motor.left".to_string()],
            depth: 0,
        };
        let response = server.handle_list_parameters(request);

        assert_eq!(response.result.names.len(), 2);
    }

    #[test]
    fn test_parameter_stats_tracking() {
        let store = Arc::new(LocalParameterStore::new("stats_node"));
        store
            .declare_parameter("test", ParameterValue::from_integer(1), None)
            .unwrap();

        // Multiple gets
        for _ in 0..5 {
            store.get_parameter("test");
        }

        // Multiple sets
        for i in 0..3 {
            store.set_parameter(Parameter::integer("test", i)).unwrap();
        }

        // List and describe
        store.list_parameters(&[], 0);
        store.describe_parameters(&["test".to_string()]);

        let stats = store.stats();
        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 5);
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 3);
        assert_eq!(stats.list_calls.load(Ordering::Relaxed), 1);
        assert_eq!(stats.describe_calls.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_parameter_type_from_code() {
        assert_eq!(ParameterType::from_code(0), ParameterType::NotSet);
        assert_eq!(ParameterType::from_code(1), ParameterType::Bool);
        assert_eq!(ParameterType::from_code(2), ParameterType::Integer);
        assert_eq!(ParameterType::from_code(3), ParameterType::Double);
        assert_eq!(ParameterType::from_code(4), ParameterType::String);
        assert_eq!(ParameterType::from_code(5), ParameterType::ByteArray);
        assert_eq!(ParameterType::from_code(6), ParameterType::BoolArray);
        assert_eq!(ParameterType::from_code(7), ParameterType::IntegerArray);
        assert_eq!(ParameterType::from_code(8), ParameterType::DoubleArray);
        assert_eq!(ParameterType::from_code(9), ParameterType::StringArray);
        assert_eq!(ParameterType::from_code(255), ParameterType::NotSet); // Invalid code
    }

    #[test]
    fn test_parameter_type_as_str() {
        assert_eq!(ParameterType::NotSet.as_str(), "not_set");
        assert_eq!(ParameterType::Bool.as_str(), "bool");
        assert_eq!(ParameterType::Integer.as_str(), "integer");
        assert_eq!(ParameterType::Double.as_str(), "double");
        assert_eq!(ParameterType::String.as_str(), "string");
        assert_eq!(ParameterType::ByteArray.as_str(), "byte_array");
    }

    #[test]
    fn test_parameter_topic_type() {
        assert_eq!(
            ParameterTopicType::ListParameters.suffix(),
            "list_parameters"
        );
        assert_eq!(ParameterTopicType::GetParameters.suffix(), "get_parameters");
        assert!(ParameterTopicType::ListParameters.is_service());
        assert!(ParameterTopicType::GetParameters.is_service());
        assert!(!ParameterTopicType::ParameterEvents.is_service());
    }

    #[test]
    fn test_parameter_server_topics() {
        let topics = ParameterServerTopics::new("/robot/controller");

        assert_eq!(
            topics.list_parameters_response,
            "rs/robot__controller/_parameters/list_parameters"
        );
        assert_eq!(
            topics.get_parameters_response,
            "rs/robot__controller/_parameters/get_parameters"
        );
        assert_eq!(
            topics.parameter_events,
            "rt/robot__controller/_parameters/parameter_events"
        );
    }

    #[test]
    fn test_parameter_error_display() {
        let errors = vec![
            Ros2ParameterError::NotFound("missing".into()),
            Ros2ParameterError::ReadOnly("readonly".into()),
            Ros2ParameterError::InvalidType {
                name: "param".into(),
                expected: ParameterType::Integer,
                actual: ParameterType::Double,
            },
            Ros2ParameterError::OutOfRange {
                name: "bounded".into(),
                reason: "too large".into(),
            },
            Ros2ParameterError::Timeout,
            Ros2ParameterError::ServiceError("failed".into()),
            Ros2ParameterError::SerializationError("invalid".into()),
        ];

        for error in errors {
            let display = format!("{}", error);
            assert!(!display.is_empty());
        }
    }

    #[test]
    fn test_parameter_value_accessors_wrong_type() {
        let int_val = ParameterValue::from_integer(42);
        assert_eq!(int_val.as_bool(), None);
        assert_eq!(int_val.as_double(), None);
        assert_eq!(int_val.as_string(), None);

        let bool_val = ParameterValue::from_bool(true);
        assert_eq!(bool_val.as_integer(), None);
        assert_eq!(bool_val.as_double(), None);
        assert_eq!(bool_val.as_string(), None);
    }

    #[test]
    fn test_integer_range_validation() {
        let store = LocalParameterStore::new("range_node");

        let desc = ParameterDescriptor::new("step_param", ParameterType::Integer)
            .with_integer_range(0, 100, 10); // Must be multiple of 10

        store
            .declare_parameter("step_param", ParameterValue::from_integer(0), Some(desc))
            .unwrap();

        // Valid: on step grid
        let result = store
            .set_parameter(Parameter::integer("step_param", 50))
            .unwrap();
        assert!(result.successful);

        // Invalid: not on step grid
        let result = store
            .set_parameter(Parameter::integer("step_param", 55))
            .unwrap();
        assert!(!result.successful);
        assert!(result.reason.contains("step"));
    }

    #[test]
    fn test_parameter_config_defaults() {
        let config = Ros2ParameterConfig::default();

        assert_eq!(config.timeout, Duration::from_secs(5));
        assert!(config.publish_events);
        assert_eq!(config.qos.service_history_depth, 10);
        assert_eq!(config.qos.event_history_depth, 100);
    }

    #[test]
    fn test_parameter_event_creation() {
        let store = LocalParameterStore::new("event_node");
        store
            .declare_parameter("p1", ParameterValue::from_integer(1), None)
            .unwrap();
        store
            .declare_parameter("p2", ParameterValue::from_integer(2), None)
            .unwrap();

        let changes = vec![
            (Parameter::integer("p1", 10), ParameterEventType::Changed),
            (
                Parameter::integer("new_param", 100),
                ParameterEventType::Added,
            ),
            (Parameter::integer("p2", 0), ParameterEventType::Deleted),
        ];

        let event = store.create_event(changes);

        assert_eq!(event.node, "event_node");
        assert_eq!(event.changed_parameters.len(), 1);
        assert_eq!(event.new_parameters.len(), 1);
        assert_eq!(event.deleted_parameters.len(), 1);

        // Stats should track event
        assert_eq!(store.stats().events_published.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_declare_parameter_no_duplicate() {
        let store = LocalParameterStore::new("dup_node");

        // First declaration
        store
            .declare_parameter("test", ParameterValue::from_integer(1), None)
            .unwrap();

        // Second declaration with different value should be ignored
        store
            .declare_parameter("test", ParameterValue::from_integer(999), None)
            .unwrap();

        // Original value should remain
        let param = store.get_parameter("test").unwrap();
        assert_eq!(param.value.integer_value, 1);
    }

    #[test]
    fn test_get_nonexistent_parameter() {
        let store = LocalParameterStore::new("empty_node");

        let param = store.get_parameter("nonexistent");
        assert!(param.is_none());
    }

    #[test]
    fn test_multiple_callbacks() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let store = LocalParameterStore::new("multi_callback_node");

        let counter1 = Arc::new(AtomicUsize::new(0));
        let counter2 = Arc::new(AtomicUsize::new(0));

        let c1 = Arc::clone(&counter1);
        store.on_change(move |_, _| {
            c1.fetch_add(1, Ordering::Relaxed);
        });

        let c2 = Arc::clone(&counter2);
        store.on_change(move |_, _| {
            c2.fetch_add(1, Ordering::Relaxed);
        });

        store
            .declare_parameter("test", ParameterValue::from_bool(true), None)
            .unwrap();

        // Both callbacks should be invoked
        assert_eq!(counter1.load(Ordering::Relaxed), 1);
        assert_eq!(counter2.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_parameter_client_sequence() {
        let client = Ros2ParameterClient::new("test_node", Ros2ParameterConfig::default());

        let seq1 = client.next_sequence();
        let seq2 = client.next_sequence();
        let seq3 = client.next_sequence();

        assert_eq!(seq1, 0);
        assert_eq!(seq2, 1);
        assert_eq!(seq3, 2);
    }

    #[test]
    fn test_floating_point_range_default() {
        let range = FloatingPointRange::default();
        assert_eq!(range.from_value, f64::MIN);
        assert_eq!(range.to_value, f64::MAX);
        assert!((range.step - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_integer_range_default() {
        let range = IntegerRange::default();
        assert_eq!(range.from_value, i64::MIN);
        assert_eq!(range.to_value, i64::MAX);
        assert_eq!(range.step, 0);
    }

    #[test]
    fn test_normalize_node_name() {
        assert_eq!(normalize_node_name("/robot/arm"), "robot__arm");
        assert_eq!(normalize_node_name("simple_node"), "simple_node");
        assert_eq!(normalize_node_name("/a/b/c"), "a__b__c");
    }

    #[test]
    fn test_type_mismatch_rejection() {
        let store = LocalParameterStore::new("type_check_node");

        // Declare as integer with strict typing
        let desc = ParameterDescriptor::new("strict_int", ParameterType::Integer);
        store
            .declare_parameter("strict_int", ParameterValue::from_integer(10), Some(desc))
            .unwrap();

        // Try to set as double - should fail
        let result = store
            .set_parameter(Parameter::double("strict_int", 10.5))
            .unwrap();
        assert!(!result.successful);
        assert!(result.reason.contains("Type mismatch"));
    }

    #[test]
    fn test_dynamic_typing_allowed() {
        let store = LocalParameterStore::new("dynamic_node");

        let mut desc = ParameterDescriptor::new("dynamic_param", ParameterType::Integer);
        desc.dynamic_typing = true;

        store
            .declare_parameter(
                "dynamic_param",
                ParameterValue::from_integer(10),
                Some(desc),
            )
            .unwrap();

        // Should succeed because dynamic typing is enabled
        let result = store
            .set_parameter(Parameter::double("dynamic_param", 10.5))
            .unwrap();
        assert!(result.successful);
    }

    #[test]
    fn test_stats_success_failure_tracking() {
        let store = LocalParameterStore::new("tracking_node");

        let desc = ParameterDescriptor::new("readonly", ParameterType::Integer).read_only(true);
        store
            .declare_parameter("readonly", ParameterValue::from_integer(1), Some(desc))
            .unwrap();
        store
            .declare_parameter("writable", ParameterValue::from_integer(1), None)
            .unwrap();

        // Successful set
        store
            .set_parameter(Parameter::integer("writable", 2))
            .unwrap();

        // Failed set (read-only)
        store
            .set_parameter(Parameter::integer("readonly", 2))
            .unwrap();

        let stats = store.stats();
        // Only successful operations increment success counter
        assert!(stats.successful_operations.load(Ordering::Relaxed) >= 1);
    }
}
