//! # HORUS Macros
//!
//! Procedural macros for the HORUS robotics framework.
//!
//! This crate provides derive macros and function-like macros to reduce
//! boilerplate and improve the developer experience when building HORUS applications.
//!
//! ## Available Macros
//!
//! - `node!` - Generate Node trait implementation with automatic topic registration
//! - `LogSummary` - Derive macro for efficient log output
//!
//! ## Safety
//!
//! These macros generate safe code and use proper error handling with `HorusError`.
//! All generated code follows Rust safety guidelines and avoids undefined behavior.

use proc_macro::TokenStream;

mod log_summary;
mod node;

/// Generate a HORUS node implementation with automatic topic registration.
///
/// # Example
///
/// ```rust,ignore
/// use horus_macros::node;
/// use horus::prelude::*;
///
/// node! {
///     CameraNode {
///         pub {
///             image: Image -> "camera.image",
///             status: Status -> "camera.status",
///         }
///
///         sub {
///             command: Command -> "camera.command",
///         }
///
///         data {
///             frame_count: u32 = 0,
///             buffer: Vec<u8> = Vec::new(),
///         }
///
///         tick(_ctx) {
///             // Process one message per tick (bounded execution time)
///             if let Some(cmd) = self.command.recv() {
///                 // Process command
///             }
///             self.frame_count += 1;
///             let img = self.capture_frame();
///             self.image.send(img);
///         }
///     }
/// }
/// ```
///
/// This generates:
/// - Complete struct definition with Topic fields
/// - `new()` constructor that creates all Topics
/// - `Node` trait implementation
/// - `Default` trait implementation
/// - Automatic snake_case node naming
///
/// # Sections
///
/// - `pub {}` - Publishers (optional, can be empty)
/// - `sub {}` - Subscribers (optional, can be empty)
/// - `data {}` - Internal state fields (optional)
/// - `tick {}` - Main update logic (required)
/// - `init(ctx) {}` - Initialization (optional)
/// - `shutdown(ctx) {}` - Cleanup (optional)
/// - `impl {}` - Additional methods (optional)
#[proc_macro]
pub fn node(input: TokenStream) -> TokenStream {
    node::impl_node_macro(input)
}

/// Derive the `LogSummary` trait for a type.
///
/// Generates a default implementation that uses `Debug` formatting:
///
/// ```rust,ignore
/// #[derive(Debug, LogSummary)]
/// pub struct MyMessage {
///     pub x: f32,
///     pub y: f32,
/// }
///
/// // Equivalent to:
/// // impl LogSummary for MyMessage {
/// //     fn log_summary(&self) -> String {
/// //         format!("{:?}", self)
/// //     }
/// // }
/// ```
///
/// For types that need custom log output (e.g., large data like images or
/// point clouds), implement `LogSummary` manually instead of deriving.
#[proc_macro_derive(LogSummary)]
pub fn derive_log_summary(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);
    TokenStream::from(log_summary::derive_log_summary(input))
}
