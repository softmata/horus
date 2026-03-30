//! CXX bridge code generator.
//!
//! Transforms `BindingItem` metadata into `#[cxx::bridge]` Rust code.
//! The generated bridge defines the FFI boundary between Rust and C++.
//!
//! ## Naming Convention
//!
//! Methods are mangled as `{type_snake}_{method_name}`:
//! - `Publisher::new()` → `fn publisher_new(...)`
//! - `Publisher::loan()` → `fn publisher_loan(...)`
//!
//! ## Type Mapping
//!
//! | Rust | CXX Bridge |
//! |------|-----------|
//! | `Self` / owned return | `Box<TypeName>` |
//! | `&self` | `obj: &TypeName` |
//! | `&mut self` | `obj: &mut TypeName` |
//! | `&str` | `&str` |
//! | `String` | `String` |
//! | `Result<T>` | `Result<T>` (CXX maps to C++ exception) |
//! | `Option<T>` | Nullable via separate has/get pattern |
//! | `*mut u8` | `*mut u8` (raw pointer for zero-copy) |

#![allow(dead_code)]

use proc_macro2::{Ident, Span, TokenStream};
use quote::{format_ident, quote};
use syn::Type;

use crate::types::{
    BindingItem, ImplBinding, MethodBinding, MethodSig, MonomorphTarget, Receiver, StructBinding,
};

/// Generate CXX bridge code from binding metadata.
///
/// Returns a TokenStream containing a complete `#[cxx::bridge]` module
/// with `extern "Rust"` declarations for all types and methods.
pub fn generate_cxx_bridge(item: &BindingItem) -> TokenStream {
    match item {
        BindingItem::Struct(s) => generate_struct_bridge(s),
        BindingItem::Impl(i) => generate_impl_bridge(i),
    }
}

/// Generate bridge for a struct: declares an opaque Rust type.
fn generate_struct_bridge(binding: &StructBinding) -> TokenStream {
    let type_name = &binding.name;

    quote! {
        #[cxx::bridge]
        mod ffi {
            extern "Rust" {
                type #type_name;
            }
        }
    }
}

/// Generate bridge for an impl block: declares FFI functions for each method.
fn generate_impl_bridge(binding: &ImplBinding) -> TokenStream {
    let type_name = extract_type_ident(&binding.self_ty);
    let type_snake = to_snake_case(&type_name.to_string());

    let mut fn_decls = Vec::new();

    for method in &binding.methods {
        // Only generate FFI for public methods
        if !is_pub(&method.vis) {
            continue;
        }
        if let Some(decl) = generate_method_ffi(&type_name, &type_snake, method) {
            fn_decls.push(decl);
        }
    }

    if fn_decls.is_empty() {
        return TokenStream::new();
    }

    quote! {
        #[cxx::bridge]
        mod ffi {
            extern "Rust" {
                type #type_name;
                #(#fn_decls)*
            }
        }
    }
}

// ─── Monomorphized Bridge Generation ─────────────────────────────────────────

/// Generate monomorphized CXX bridge for a generic impl.
///
/// For each `MonomorphTarget`, replaces the generic type parameter with the
/// concrete type and generates FFI functions with the type name baked into
/// the function name.
///
/// Example: `impl<T> Publisher<T>` + target `CmdVel` →
///   `fn publisher_cmd_vel_new(topic: &str) -> Box<PublisherCmdVel>;`
///   `fn publisher_cmd_vel_send(obj: &PublisherCmdVel, msg: CmdVel) -> Result<()>;`
pub fn generate_monomorphized_bridge(
    base_type_name: &str,
    methods: &[MethodBinding],
    targets: &[MonomorphTarget],
) -> TokenStream {
    let mut all_types = Vec::new();
    let mut all_fns = Vec::new();

    for target in targets {
        let concrete_name = format!("{}{}", base_type_name, target.rust_name);
        let concrete_ident = Ident::new(&concrete_name, Span::call_site());
        let type_snake = to_snake_case(base_type_name);
        let prefix = format!("{}_{}", type_snake, target.snake_name);

        // Declare the concrete opaque type
        all_types.push(quote! { type #concrete_ident; });

        // Generate FFI functions for each public method
        for method in methods {
            if !is_pub(&method.vis) {
                continue;
            }
            if let Some(decl) =
                generate_monomorphized_method_ffi(&concrete_ident, &prefix, method, target)
            {
                all_fns.push(decl);
            }
        }
    }

    if all_types.is_empty() {
        return TokenStream::new();
    }

    quote! {
        #[cxx::bridge]
        mod ffi {
            extern "Rust" {
                #(#all_types)*
                #(#all_fns)*
            }
        }
    }
}

/// Generate a single monomorphized FFI function.
fn generate_monomorphized_method_ffi(
    concrete_type: &Ident,
    prefix: &str,
    method: &MethodBinding,
    _target: &MonomorphTarget,
) -> Option<TokenStream> {
    let method_name = &method.name;
    let ffi_name = format_ident!("{}_{}", prefix, method_name);
    let sig = &method.sig;

    let mut params = Vec::new();

    match &sig.receiver {
        Receiver::Ref => params.push(quote! { obj: &#concrete_type }),
        Receiver::RefMut => params.push(quote! { obj: &mut #concrete_type }),
        Receiver::Owned => params.push(quote! { obj: Box<#concrete_type> }),
        Receiver::None => {}
    }

    for param in &sig.params {
        let name = &param.name;
        let ty = &param.ty;
        params.push(quote! { #name: #ty });
    }

    let ret = match &sig.return_ty {
        None => quote! {},
        Some(ty) => {
            let mapped = if is_self_type(ty) {
                quote! { Box<#concrete_type> }
            } else {
                quote! { #ty }
            };
            if sig.is_result {
                quote! { -> Result<#mapped> }
            } else {
                quote! { -> #mapped }
            }
        }
    };

    Some(quote! { fn #ffi_name(#(#params),*) #ret; })
}

/// Generate a single FFI function declaration for a method.
fn generate_method_ffi(
    type_name: &Ident,
    type_snake: &str,
    method: &MethodBinding,
) -> Option<TokenStream> {
    let method_name = &method.name;
    let ffi_name = format_ident!("{}_{}", type_snake, method_name);
    let sig = &method.sig;

    // Build parameter list
    let mut params = Vec::new();

    // Add receiver as explicit parameter
    match &sig.receiver {
        Receiver::Ref => {
            params.push(quote! { obj: &#type_name });
        }
        Receiver::RefMut => {
            params.push(quote! { obj: &mut #type_name });
        }
        Receiver::Owned => {
            params.push(quote! { obj: Box<#type_name> });
        }
        Receiver::None => {} // static — no self param
    }

    // Add regular parameters
    for param in &sig.params {
        let name = &param.name;
        let ty = map_param_type(&param.ty);
        params.push(quote! { #name: #ty });
    }

    // Build return type
    let ret = generate_return_type(type_name, sig);

    Some(quote! {
        fn #ffi_name(#(#params),*) #ret;
    })
}

/// Map a parameter type for CXX compatibility.
/// Most types pass through unchanged — CXX handles &str, String, primitives natively.
fn map_param_type(ty: &Type) -> TokenStream {
    quote! { #ty }
}

/// Generate the return type for a CXX FFI function.
fn generate_return_type(type_name: &Ident, sig: &MethodSig) -> TokenStream {
    match &sig.return_ty {
        None => quote! {},
        Some(ty) => {
            let mapped = map_return_inner(type_name, ty);
            if sig.is_result {
                quote! { -> Result<#mapped> }
            } else {
                quote! { -> #mapped }
            }
        }
    }
}

/// Map the inner return type. `Self` becomes `Box<TypeName>`.
fn map_return_inner(type_name: &Ident, ty: &Type) -> TokenStream {
    if is_self_type(ty) {
        quote! { Box<#type_name> }
    } else {
        quote! { #ty }
    }
}

/// Check if a type is `Self`.
fn is_self_type(ty: &Type) -> bool {
    if let Type::Path(tp) = ty {
        if let Some(seg) = tp.path.segments.last() {
            return seg.ident == "Self";
        }
    }
    false
}

/// Check if visibility is `pub` (any variant).
fn is_pub(vis: &syn::Visibility) -> bool {
    !matches!(vis, syn::Visibility::Inherited)
}

/// Extract the type identifier from a syn::Type (handles simple paths).
fn extract_type_ident(ty: &Type) -> Ident {
    if let Type::Path(tp) = ty {
        if let Some(seg) = tp.path.segments.last() {
            return seg.ident.clone();
        }
    }
    Ident::new("Unknown", Span::call_site())
}

/// Convert PascalCase to snake_case.
fn to_snake_case(s: &str) -> String {
    let mut result = String::with_capacity(s.len() + 4);
    for (i, ch) in s.chars().enumerate() {
        if ch.is_uppercase() {
            if i > 0 {
                result.push('_');
            }
            result.push(ch.to_ascii_lowercase());
        } else {
            result.push(ch);
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::{parse_impl, parse_struct};
    use crate::types::BindingItem;
    use syn::{parse_quote, ItemImpl, ItemStruct};

    fn assert_contains(ts: &TokenStream, needle: &str) {
        let s = ts.to_string();
        assert!(
            s.contains(needle),
            "Expected output to contain \"{needle}\"\nActual:\n{s}"
        );
    }

    fn assert_not_contains(ts: &TokenStream, needle: &str) {
        let s = ts.to_string();
        assert!(
            !s.contains(needle),
            "Expected output NOT to contain \"{needle}\"\nActual:\n{s}"
        );
    }

    #[test]
    fn struct_bridge_declares_opaque_type() {
        let item: ItemStruct = parse_quote! {
            pub struct Publisher {
                inner: u64,
            }
        };
        let binding = parse_struct(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Struct(binding));
        assert_contains(&bridge, "type Publisher");
        assert_contains(&bridge, "extern \"Rust\"");
    }

    #[test]
    fn impl_bridge_generates_ffi_functions() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn new(topic: &str) -> Self { todo!() }
                pub fn loan(&self) -> LoanedSample { todo!() }
                pub fn publish(&self, sample: LoanedSample) { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        let s = bridge.to_string();

        // Check function names are mangled
        assert!(
            s.contains("publisher_new"),
            "missing publisher_new in:\n{s}"
        );
        assert!(
            s.contains("publisher_loan"),
            "missing publisher_loan in:\n{s}"
        );
        assert!(
            s.contains("publisher_publish"),
            "missing publisher_publish in:\n{s}"
        );

        // Check type is declared
        assert_contains(&bridge, "type Publisher");
    }

    #[test]
    fn ref_self_becomes_ref_param() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                pub fn bar(&self) -> u32 { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        assert_contains(&bridge, "obj : & Foo");
    }

    #[test]
    fn mut_self_becomes_mut_ref_param() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                pub fn mutate(&mut self) { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        assert_contains(&bridge, "obj : & mut Foo");
    }

    #[test]
    fn owned_self_becomes_box_param() {
        let item: ItemImpl = parse_quote! {
            impl Builder {
                pub fn build(self) -> Node { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        assert_contains(&bridge, "obj : Box < Builder >");
    }

    #[test]
    fn static_method_no_self_param() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn new(topic: &str) -> Self { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        let s = bridge.to_string();
        // Should have topic param but no obj param
        assert!(s.contains("topic"), "missing topic param in:\n{s}");
        assert!(
            !s.contains("obj :"),
            "static method should not have obj param:\n{s}"
        );
    }

    #[test]
    fn self_return_becomes_box() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn new(topic: &str) -> Self { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        assert_contains(&bridge, "Box < Publisher >");
    }

    #[test]
    fn result_return_preserved() {
        let item: ItemImpl = parse_quote! {
            impl Service {
                pub fn call(&self, req: Request) -> Result<Response> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        assert_contains(&bridge, "Result <");
    }

    #[test]
    fn void_return_no_arrow() {
        let item: ItemImpl = parse_quote! {
            impl Node {
                pub fn tick(&mut self) {}
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        let s = bridge.to_string();
        // Should NOT have -> in the function signature for void return
        assert!(
            !s.contains("->"),
            "void method should not have return type:\n{s}"
        );
    }

    #[test]
    fn private_methods_excluded() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                pub fn public_method(&self) {}
                fn private_method(&self) {}
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        assert_contains(&bridge, "foo_public_method");
        assert_not_contains(&bridge, "foo_private_method");
    }

    #[test]
    fn snake_case_conversion() {
        assert_eq!(to_snake_case("Publisher"), "publisher");
        assert_eq!(to_snake_case("LoanedSample"), "loaned_sample");
        assert_eq!(to_snake_case("TransformFrame"), "transform_frame");
        assert_eq!(to_snake_case("A"), "a");
        assert_eq!(to_snake_case("ABC"), "a_b_c");
    }

    #[test]
    fn empty_impl_no_output() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                fn only_private(&self) {}
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        assert!(bridge.is_empty(), "empty impl should produce no bridge");
    }

    #[test]
    fn complete_publisher_pattern() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn new(topic: &str) -> Self { todo!() }
                pub fn loan(&self) -> LoanedSample { todo!() }
                pub fn publish(&self, sample: LoanedSample) { todo!() }
                pub fn data_ptr(sample: &LoanedSample) -> *mut u8 { todo!() }
                pub fn data_size(sample: &LoanedSample) -> usize { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        let s = bridge.to_string();

        // Verify complete Publisher loan-pattern bridge
        assert!(s.contains("publisher_new"), "new: {s}");
        assert!(s.contains("publisher_loan"), "loan: {s}");
        assert!(s.contains("publisher_publish"), "publish: {s}");
        assert!(s.contains("publisher_data_ptr"), "data_ptr: {s}");
        assert!(s.contains("publisher_data_size"), "data_size: {s}");
        assert!(s.contains("Box < Publisher >"), "new returns Box: {s}");
    }

    // ─── Result/Option Edge Cases ────────────────────────────────────

    #[test]
    fn result_unit_generates_result_unit() {
        let item: ItemImpl = parse_quote! {
            impl Topic {
                pub fn send(&self, msg: CmdVel) -> Result<()> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        let s = bridge.to_string();
        assert!(s.contains("Result"), "should have Result in:\n{s}");
    }

    #[test]
    fn result_self_generates_result_box() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn try_new(topic: &str) -> Result<Self> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        let s = bridge.to_string();
        assert!(
            s.contains("Result < Box < Publisher >"),
            "Result<Box<Publisher>> in:\n{s}"
        );
    }

    #[test]
    fn option_in_cxx_bridge_passes_through() {
        let item: ItemImpl = parse_quote! {
            impl Subscriber {
                pub fn recv(&self) -> Option<Message> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_cxx_bridge(&BindingItem::Impl(binding));
        let s = bridge.to_string();
        assert!(
            s.contains("Message"),
            "return type should contain Message:\n{s}"
        );
    }

    #[test]
    fn void_result_in_cpp_header() {
        let item: ItemImpl = parse_quote! {
            impl Topic {
                pub fn send(&self, msg: CmdVel) -> Result<()> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = crate::cpp_gen::generate_cpp_header(&BindingItem::Impl(binding));
        assert!(
            header.contains("void send"),
            "void return for Result<()>:\n{header}"
        );
    }

    // ─── Monomorphization Tests ──────────────────────────────────────

    #[test]
    fn monomorph_publisher_cmd_vel() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn new(topic: &str) -> Self { todo!() }
                pub fn send(&self, msg: CmdVel) -> Result<()> { todo!() }
                pub fn recv(&self) -> Option<CmdVel> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let targets = &[MonomorphTarget {
            rust_name: "CmdVel",
            snake_name: "cmd_vel",
            rust_path: "horus_library::CmdVel",
        }];
        let bridge = generate_monomorphized_bridge("Publisher", &binding.methods, targets);
        let s = bridge.to_string();

        assert!(
            s.contains("type PublisherCmdVel"),
            "concrete type decl:\n{s}"
        );
        assert!(s.contains("publisher_cmd_vel_new"), "new fn:\n{s}");
        assert!(s.contains("publisher_cmd_vel_send"), "send fn:\n{s}");
        assert!(s.contains("publisher_cmd_vel_recv"), "recv fn:\n{s}");
        assert!(
            s.contains("Box < PublisherCmdVel >"),
            "new returns box:\n{s}"
        );
    }

    #[test]
    fn monomorph_multiple_types() {
        let item: ItemImpl = parse_quote! {
            impl Topic {
                pub fn send(&self, msg: u8) -> Result<()> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let targets = &[
            MonomorphTarget {
                rust_name: "CmdVel",
                snake_name: "cmd_vel",
                rust_path: "horus_library::CmdVel",
            },
            MonomorphTarget {
                rust_name: "LaserScan",
                snake_name: "laser_scan",
                rust_path: "horus_library::LaserScan",
            },
            MonomorphTarget {
                rust_name: "Imu",
                snake_name: "imu",
                rust_path: "horus_library::Imu",
            },
        ];
        let bridge = generate_monomorphized_bridge("Topic", &binding.methods, targets);
        let s = bridge.to_string();

        // All 3 concrete types declared
        assert!(s.contains("type TopicCmdVel"), "CmdVel type:\n{s}");
        assert!(s.contains("type TopicLaserScan"), "LaserScan type:\n{s}");
        assert!(s.contains("type TopicImu"), "Imu type:\n{s}");

        // All 3 send functions
        assert!(s.contains("topic_cmd_vel_send"), "CmdVel send:\n{s}");
        assert!(s.contains("topic_laser_scan_send"), "LaserScan send:\n{s}");
        assert!(s.contains("topic_imu_send"), "Imu send:\n{s}");
    }

    #[test]
    fn monomorph_private_excluded() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                pub fn public_fn(&self) {}
                fn private_fn(&self) {}
            }
        };
        let binding = parse_impl(&item).unwrap();
        let targets = &[MonomorphTarget {
            rust_name: "Bar",
            snake_name: "bar",
            rust_path: "Bar",
        }];
        let bridge = generate_monomorphized_bridge("Foo", &binding.methods, targets);
        let s = bridge.to_string();

        assert!(s.contains("foo_bar_public_fn"), "public:\n{s}");
        assert!(
            !s.contains("foo_bar_private_fn"),
            "private should be excluded:\n{s}"
        );
    }

    #[test]
    fn monomorph_empty_targets() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                pub fn bar(&self) {}
            }
        };
        let binding = parse_impl(&item).unwrap();
        let bridge = generate_monomorphized_bridge("Foo", &binding.methods, &[]);
        assert!(bridge.is_empty());
    }

    #[test]
    fn default_monomorph_set_has_25_types() {
        use crate::types::DEFAULT_MONOMORPH_TYPES;
        assert_eq!(DEFAULT_MONOMORPH_TYPES.len(), 25);
    }

    #[test]
    fn default_monomorph_types_have_consistent_naming() {
        use crate::types::DEFAULT_MONOMORPH_TYPES;
        for target in DEFAULT_MONOMORPH_TYPES {
            // Snake name should be lowercase
            assert_eq!(
                target.snake_name,
                target.snake_name.to_lowercase(),
                "{} snake_name should be lowercase",
                target.rust_name
            );
            // Rust path should start with horus_library::
            assert!(
                target.rust_path.starts_with("horus_library::"),
                "{} rust_path should start with horus_library::",
                target.rust_name
            );
        }
    }
}
