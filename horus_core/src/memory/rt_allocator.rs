//! RT-aware allocator for detecting heap allocations during real-time ticks.
//!
//! When a node has `.no_alloc()` set, the RT executor calls [`enter_rt_context`]
//! before `tick()` and [`leave_rt_context`] after. If the global allocator is
//! [`RtAwareAllocator`], any `alloc()` call during the RT context panics with
//! a clear message identifying the offending node.
//!
//! # Setup
//!
//! Users opt in by adding one line to their `main.rs`:
//!
//! ```rust,ignore
//! #[global_allocator]
//! static ALLOC: horus_core::memory::RtAwareAllocator = horus_core::memory::RtAwareAllocator;
//! ```
//!
//! Without this line, `.no_alloc()` is a no-op — the context functions still
//! toggle the thread-local flag but the system allocator doesn't check it.
//!
//! # Zero overhead
//!
//! - Without `#[global_allocator]`: zero overhead (system allocator, no checks)
//! - With `#[global_allocator]` but no `.no_alloc()` nodes: one thread-local read per alloc (~1ns)
//! - With `.no_alloc()` active: same 1ns check, panics on allocation

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

thread_local! {
    /// Whether the current thread is in an RT tick context where allocations are forbidden.
    static RT_ALLOC_CONTEXT: Cell<bool> = const { Cell::new(false) };
    /// Name of the node currently being ticked (for panic messages).
    static RT_NODE_NAME: Cell<Option<&'static str>> = const { Cell::new(None) };
}

/// Enter the RT allocation-free context for the named node.
///
/// Called by the RT executor before `tick()` when the node has `.no_alloc()`.
/// Any heap allocation after this call (and before [`leave_rt_context`]) will
/// panic if [`RtAwareAllocator`] is the global allocator.
pub fn enter_rt_context(node_name: &str) {
    // Leak the name to get 'static lifetime. This happens once per node per
    // executor startup — not per tick. The leaked memory is reclaimed when
    // the process exits.
    let name: &'static str = Box::leak(node_name.to_string().into_boxed_str());
    RT_NODE_NAME.with(|c| c.set(Some(name)));
    RT_ALLOC_CONTEXT.with(|c| c.set(true));
}

/// Leave the RT allocation-free context.
///
/// Called by the RT executor after `tick()`. Allocations are allowed again.
pub fn leave_rt_context() {
    RT_ALLOC_CONTEXT.with(|c| c.set(false));
}

/// Check if the current thread is in an RT allocation-free context.
pub fn is_rt_context() -> bool {
    RT_ALLOC_CONTEXT.with(|c| c.get())
}

/// Global allocator that panics on heap allocation during RT tick context.
///
/// Delegates all allocations to the system allocator. When [`enter_rt_context`]
/// has been called on the current thread, any `alloc()` call panics with a
/// message identifying the node.
///
/// # Usage
///
/// ```rust,ignore
/// #[global_allocator]
/// static ALLOC: horus_core::memory::RtAwareAllocator = horus_core::memory::RtAwareAllocator;
/// ```
pub struct RtAwareAllocator;

unsafe impl GlobalAlloc for RtAwareAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if RT_ALLOC_CONTEXT.with(|c| c.get()) {
            let name = RT_NODE_NAME.with(|c| c.get()).unwrap_or("<unknown>");
            // Write directly to stderr to avoid allocating in the panic message path.
            // The panic machinery itself may allocate, but the message reaches the user.
            eprintln!(
                "\n\x1b[1;31mHEAP ALLOCATION IN RT TICK!\x1b[0m\n\
                 Node '{}' allocated memory during tick().\n\
                 This causes unpredictable latency in real-time code.\n\
                 \n\
                 Fix: Remove Vec::push(), String::from(), format!(), Box::new(),\n\
                 or any other heap allocation from {0}::tick().\n\
                 \n\
                 To allow allocations (prototyping), remove .no_alloc() from the node builder.\n",
                name,
            );
            panic!(
                "Heap allocation in RT tick of '{}'. Remove allocations from tick() or remove .no_alloc().",
                name
            );
        }
        System.alloc(layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if RT_ALLOC_CONTEXT.with(|c| c.get()) {
            let name = RT_NODE_NAME.with(|c| c.get()).unwrap_or("<unknown>");
            panic!(
                "Heap allocation (zeroed) in RT tick of '{}'. Remove allocations from tick().",
                name
            );
        }
        System.alloc_zeroed(layout)
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if RT_ALLOC_CONTEXT.with(|c| c.get()) {
            let name = RT_NODE_NAME.with(|c| c.get()).unwrap_or("<unknown>");
            panic!(
                "Heap reallocation in RT tick of '{}'. Remove allocations from tick().",
                name
            );
        }
        System.realloc(ptr, layout, new_size)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rt_context_enter_leave() {
        assert!(!is_rt_context());
        enter_rt_context("test_node");
        assert!(is_rt_context());
        leave_rt_context();
        assert!(!is_rt_context());
    }

    #[test]
    fn test_rt_context_default_off() {
        assert!(!is_rt_context());
    }
}
