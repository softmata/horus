//! Node Processor Trait - Enables hybrid nodes with user-injectable logic
//!
//! This module provides the `Processor` trait and related types that allow
//! HORUS nodes to act as both hardware drivers AND customizable processing pipelines.
//!
//! # Design Goals
//!
//! 1. **Zero-config default**: Nodes work out-of-box as pure drivers
//! 2. **Optional customization**: Users can inject processing logic via closures or traits
//! 3. **Type-safe**: Compile-time guarantees for processor compatibility
//! 4. **Minimal overhead**: No runtime cost when using defaults (monomorphization)
//! 5. **Ergonomic API**: Simple closure-based or trait-based hooks
//!
//! # Usage Examples
//!
//! ```rust,ignore
//! use horus_library::nodes::*;
//!
//! // 1. Simple driver mode (default - no processing)
//! let camera = CameraNode::new()?;
//!
//! // 2. With inline closure processing
//! let camera = CameraNode::new()?
//!     .with_processor(|frame: Image| {
//!         // Apply custom filter
//!         apply_grayscale(frame)
//!     });
//!
//! // 3. With typed processor struct
//! struct MyVisionPipeline { threshold: u8 }
//! impl Processor<Image> for MyVisionPipeline {
//!     fn process(&mut self, input: Image) -> Image {
//!         edge_detect(input, self.threshold)
//!     }
//! }
//! let camera = CameraNode::new()?
//!     .with_processor(MyVisionPipeline { threshold: 50 });
//!
//! // 4. Builder pattern with pipeline stages
//! let camera = CameraNode::builder()
//!     .resolution(1280, 720)
//!     .fps(30.0)
//!     .pipe(GrayscaleFilter)
//!     .pipe(EdgeDetector::new(50))
//!     .on_output(|frame| println!("Got frame: {}x{}", frame.width, frame.height))
//!     .build()?;
//!
//! // 5. Motor with command preprocessing
//! let motor = DcMotorNode::new()?
//!     .with_processor(|cmd: PwmCommand| {
//!         // Apply velocity ramping
//!         ramp_velocity(cmd, 0.1)
//!     });
//! ```

use std::marker::PhantomData;

/// Core processor trait for user-injectable logic
///
/// Implement this trait to create reusable processing components that can be
/// plugged into any compatible HORUS node.
///
/// # Type Parameters
/// - `I`: Input type (what the node receives/captures)
/// - `O`: Output type (what gets published), defaults to same as input
pub trait Processor<I, O = I>: Send + 'static {
    /// Process input and produce output
    ///
    /// This method is called for each piece of data the node handles.
    /// Return `Some(output)` to publish, or `None` to skip this frame.
    fn process(&mut self, input: I) -> Option<O>;

    /// Optional: Called once when the node starts
    fn on_start(&mut self) {}

    /// Optional: Called once when the node shuts down
    fn on_shutdown(&mut self) {}

    /// Optional: Called on each tick, even if no data is available
    fn on_tick(&mut self) {}
}

/// Default pass-through processor (no-op)
///
/// This is used when no custom processor is specified, providing zero-overhead
/// default behavior via monomorphization.
#[derive(Debug, Clone, Copy, Default)]
pub struct PassThrough<T>(PhantomData<T>);

impl<T> PassThrough<T> {
    pub fn new() -> Self {
        Self(PhantomData)
    }
}

impl<T: Send + 'static> Processor<T> for PassThrough<T> {
    #[inline(always)]
    fn process(&mut self, input: T) -> Option<T> {
        Some(input)
    }
}

/// Closure-based processor wrapper
///
/// Allows using simple closures as processors without implementing the trait.
pub struct ClosureProcessor<I, O, F>
where
    F: FnMut(I) -> O + Send + 'static,
{
    func: F,
    _phantom: PhantomData<(I, O)>,
}

impl<I, O, F> ClosureProcessor<I, O, F>
where
    F: FnMut(I) -> O + Send + 'static,
{
    pub fn new(func: F) -> Self {
        Self {
            func,
            _phantom: PhantomData,
        }
    }
}

impl<I, O, F> Processor<I, O> for ClosureProcessor<I, O, F>
where
    I: Send + 'static,
    O: Send + 'static,
    F: FnMut(I) -> O + Send + 'static,
{
    fn process(&mut self, input: I) -> Option<O> {
        Some((self.func)(input))
    }
}

/// Optional closure processor (can filter/skip outputs)
pub struct FilterProcessor<I, O, F>
where
    F: FnMut(I) -> Option<O> + Send + 'static,
{
    func: F,
    _phantom: PhantomData<(I, O)>,
}

impl<I, O, F> FilterProcessor<I, O, F>
where
    F: FnMut(I) -> Option<O> + Send + 'static,
{
    pub fn new(func: F) -> Self {
        Self {
            func,
            _phantom: PhantomData,
        }
    }
}

impl<I, O, F> Processor<I, O> for FilterProcessor<I, O, F>
where
    I: Send + 'static,
    O: Send + 'static,
    F: FnMut(I) -> Option<O> + Send + 'static,
{
    fn process(&mut self, input: I) -> Option<O> {
        (self.func)(input)
    }
}

/// Pipeline processor - chains multiple processors together
pub struct Pipeline<I, M, O, P1, P2>
where
    P1: Processor<I, M>,
    P2: Processor<M, O>,
{
    first: P1,
    second: P2,
    _phantom: PhantomData<(I, M, O)>,
}

impl<I, M, O, P1, P2> Pipeline<I, M, O, P1, P2>
where
    P1: Processor<I, M>,
    P2: Processor<M, O>,
{
    pub fn new(first: P1, second: P2) -> Self {
        Self {
            first,
            second,
            _phantom: PhantomData,
        }
    }
}

impl<I, M, O, P1, P2> Processor<I, O> for Pipeline<I, M, O, P1, P2>
where
    I: Send + 'static,
    M: Send + 'static,
    O: Send + 'static,
    P1: Processor<I, M>,
    P2: Processor<M, O>,
{
    fn process(&mut self, input: I) -> Option<O> {
        self.first
            .process(input)
            .and_then(|mid| self.second.process(mid))
    }

    fn on_start(&mut self) {
        self.first.on_start();
        self.second.on_start();
    }

    fn on_shutdown(&mut self) {
        self.first.on_shutdown();
        self.second.on_shutdown();
    }

    fn on_tick(&mut self) {
        self.first.on_tick();
        self.second.on_tick();
    }
}

/// Tap processor - runs a side-effect without modifying the data
pub struct Tap<T, F>
where
    F: FnMut(&T) + Send + 'static,
{
    func: F,
    _phantom: PhantomData<T>,
}

impl<T, F> Tap<T, F>
where
    F: FnMut(&T) + Send + 'static,
{
    pub fn new(func: F) -> Self {
        Self {
            func,
            _phantom: PhantomData,
        }
    }
}

impl<T, F> Processor<T> for Tap<T, F>
where
    T: Send + 'static,
    F: FnMut(&T) + Send + 'static,
{
    fn process(&mut self, input: T) -> Option<T> {
        (self.func)(&input);
        Some(input)
    }
}

/// Extension trait for chaining processors
pub trait ProcessorExt<I, O>: Processor<I, O> + Sized {
    /// Chain another processor after this one
    fn pipe<O2, P>(self, next: P) -> Pipeline<I, O, O2, Self, P>
    where
        P: Processor<O, O2>,
    {
        Pipeline::new(self, next)
    }

    /// Add a tap (side-effect) that doesn't modify the output
    fn tap<F>(self, func: F) -> Pipeline<I, O, O, Self, Tap<O, F>>
    where
        F: FnMut(&O) + Send + 'static,
        O: Send + 'static,
    {
        Pipeline::new(self, Tap::new(func))
    }

    /// Map the output to a different type
    fn map<O2, F>(self, func: F) -> Pipeline<I, O, O2, Self, ClosureProcessor<O, O2, F>>
    where
        F: FnMut(O) -> O2 + Send + 'static,
        O: Send + 'static,
        O2: Send + 'static,
    {
        Pipeline::new(self, ClosureProcessor::new(func))
    }

    /// Filter outputs (return None to skip)
    fn filter_map<O2, F>(self, func: F) -> Pipeline<I, O, O2, Self, FilterProcessor<O, O2, F>>
    where
        F: FnMut(O) -> Option<O2> + Send + 'static,
        O: Send + 'static,
        O2: Send + 'static,
    {
        Pipeline::new(self, FilterProcessor::new(func))
    }
}

// Implement ProcessorExt for all Processors
impl<I, O, P: Processor<I, O>> ProcessorExt<I, O> for P {}

/// Helper function to create a processor from a closure
pub fn processor<I, O, F>(func: F) -> ClosureProcessor<I, O, F>
where
    F: FnMut(I) -> O + Send + 'static,
{
    ClosureProcessor::new(func)
}

/// Helper function to create a filtering processor from a closure
pub fn filter<I, O, F>(func: F) -> FilterProcessor<I, O, F>
where
    F: FnMut(I) -> Option<O> + Send + 'static,
{
    FilterProcessor::new(func)
}

/// Helper function to create a tap (side-effect only)
pub fn tap<T, F>(func: F) -> Tap<T, F>
where
    F: FnMut(&T) + Send + 'static,
{
    Tap::new(func)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_passthrough() {
        let mut p = PassThrough::<i32>::new();
        assert_eq!(p.process(42), Some(42));
    }

    #[test]
    fn test_closure_processor() {
        let mut p = ClosureProcessor::new(|x: i32| x * 2);
        assert_eq!(p.process(21), Some(42));
    }

    #[test]
    fn test_filter_processor() {
        let mut p = FilterProcessor::new(|x: i32| if x > 0 { Some(x) } else { None });
        assert_eq!(p.process(5), Some(5));
        assert_eq!(p.process(-5), None);
    }

    #[test]
    fn test_pipeline() {
        let mut p = Pipeline::new(
            ClosureProcessor::new(|x: i32| x * 2),
            ClosureProcessor::new(|x: i32| x + 1),
        );
        assert_eq!(p.process(10), Some(21)); // (10 * 2) + 1 = 21
    }

    #[test]
    fn test_processor_ext_pipe() {
        let mut p = processor(|x: i32| x * 2).pipe(processor(|x: i32| x + 1));
        assert_eq!(p.process(10), Some(21));
    }

    #[test]
    fn test_processor_ext_map() {
        let mut p = processor(|x: i32| x * 2).map(|x| x.to_string());
        assert_eq!(p.process(21), Some("42".to_string()));
    }

    #[test]
    fn test_tap() {
        let mut p = Tap::new(|x: &i32| {
            // In real code, this would log or record metrics
            assert!(*x > 0);
        });

        assert_eq!(p.process(42), Some(42));
    }
}
