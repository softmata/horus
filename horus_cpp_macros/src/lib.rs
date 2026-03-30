//! # HORUS C++ Macros
//!
//! Procedural macros for generating C++ bindings from annotated Rust code.
//!
//! ## `#[horus_api]`
//!
//! Attribute macro that marks a struct or impl block for C++ binding generation.
//! When applied, it:
//! 1. Parses the Rust type/method signatures
//! 2. Generates a CXX bridge (`#[cxx::bridge]` module)
//! 3. Generates idiomatic C++ headers (.hpp files)
//!
//! ## Annotation Hints
//!
//! Fine-tune the generated C++ API with `#[cpp(...)]` attributes:
//!
//! - `#[cpp(move)]` — parameter uses `T&&` (move semantics) in C++
//! - `#[cpp(direct_field_access)]` — generates `operator->` instead of getters
//! - `#[cpp(returns = "unique_ptr")]` — wraps return in `std::unique_ptr<T>`
//! - `#[cpp(raii)]` — destructor calls Rust Drop
//!
//! ## Example
//!
//! ```rust,ignore
//! use horus_cpp_macros::horus_api;
//!
//! #[horus_api]
//! pub struct Publisher {
//!     // ...
//! }
//!
//! #[horus_api]
//! impl Publisher {
//!     #[cpp(returns = "unique_ptr")]
//!     pub fn new(topic: &str) -> Self { /* ... */ }
//!
//!     #[cpp(returns = "unique_ptr", raii)]
//!     pub fn loan(&self) -> LoanedSample { /* ... */ }
//!
//!     #[cpp(move)]
//!     pub fn publish(&self, sample: LoanedSample) { /* ... */ }
//! }
//! ```

use proc_macro::TokenStream;

mod parser;
mod types;
mod cxx_gen;
mod cpp_gen;

#[cfg(test)]
mod integration_tests;

/// Marks a struct or impl block for C++ binding generation.
///
/// When applied to a `struct`, collects field information for the C++ type.
/// When applied to an `impl` block, collects method signatures for the C++ class.
///
/// The generated code is emitted during the build step via `horus_cpp`'s build.rs,
/// not inline — this macro currently acts as a metadata collector that passes
/// through the original Rust code unchanged while recording binding metadata
/// to a compile-time registry.
///
/// # On Structs
///
/// ```rust,ignore
/// #[horus_api]
/// #[cpp(direct_field_access)]
/// pub struct LoanedSample {
///     ptr: *mut u8,
///     size: usize,
/// }
/// ```
///
/// # On Impl Blocks
///
/// ```rust,ignore
/// #[horus_api]
/// impl Publisher {
///     #[cpp(returns = "unique_ptr")]
///     pub fn new(topic: &str) -> Self { ... }
/// }
/// ```
#[proc_macro_attribute]
pub fn horus_api(attr: TokenStream, item: TokenStream) -> TokenStream {
    match parser::parse_horus_api(attr.into(), item.into()) {
        Ok(output) => output.into(),
        Err(err) => err.to_compile_error().into(),
    }
}
