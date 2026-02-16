/// Runtime profiler for tracking node execution statistics
pub mod profiler;

/// Node tier annotations for scheduling priority hints
pub mod profile;

pub use profile::NodeTier;
pub use profiler::RuntimeProfiler;
