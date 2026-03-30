//! C++ header generator.
//!
//! Transforms `BindingItem` metadata into idiomatic C++ header files (.hpp).
//! Applies `CppHints` to control the ergonomic surface:
//!
//! | Hint | C++ Output |
//! |------|-----------|
//! | `direct_field_access` | `T* operator->()` |
//! | `raii` | destructor releases Rust Box |
//! | `returns_unique_ptr` | `std::unique_ptr<T>` return |
//! | `move_semantics` | `T&&` param, deleted copy ctor |
//! | `name` | custom class name |
//! | `namespace` | custom namespace |

use crate::types::{
    BindingItem, CppHints, ImplBinding, MethodBinding, MethodSig, Receiver, StructBinding,
};

/// Generate C++ header content from binding metadata.
///
/// Returns a complete .hpp file as a String, ready to write to disk.
pub fn generate_cpp_header(item: &BindingItem) -> String {
    match item {
        BindingItem::Struct(s) => generate_struct_header(s),
        BindingItem::Impl(i) => generate_impl_header(i),
    }
}

/// Generate a header for a struct binding (class with inner Box).
fn generate_struct_header(binding: &StructBinding) -> String {
    let default_name = binding.name.to_string();
    let class_name = binding.hints.cpp_name.as_deref().unwrap_or(&default_name);
    let namespace = binding.hints.cpp_namespace.as_deref().unwrap_or("horus");

    let mut out = String::with_capacity(1024);

    // File header
    write_file_header(&mut out);

    // Namespace open
    writeln_str(&mut out, &format!("namespace {} {{", namespace));
    out.push('\n');

    // Class declaration
    writeln_str(&mut out, &format!("class {} {{", class_name));
    writeln_str(&mut out, "public:");

    // Default constructor (protected — use factory methods)
    writeln_str(&mut out, &format!("    {}() = delete;", class_name));

    // If RAII: destructor
    if binding.hints.raii {
        writeln_str(&mut out, &format!("    ~{}();", class_name));
    }

    // If direct_field_access: operator->
    if binding.hints.direct_field_access {
        out.push('\n');
        writeln_str(
            &mut out,
            "    // Direct field access (zero-copy SHM pointer)",
        );
        writeln_str(&mut out, "    auto* operator->() { return data_ptr_; }");
        writeln_str(
            &mut out,
            "    const auto* operator->() const { return data_ptr_; }",
        );
        writeln_str(&mut out, "    auto& operator*() { return *data_ptr_; }");
        writeln_str(
            &mut out,
            "    const auto& operator*() const { return *data_ptr_; }",
        );
    }

    // Move-only: delete copy, default move
    if binding.hints.move_semantics || binding.hints.raii {
        out.push('\n');
        writeln_str(&mut out, "    // Move-only (no copying)");
        writeln_str(
            &mut out,
            &format!("    {}({}&& other) = default;", class_name, class_name),
        );
        writeln_str(
            &mut out,
            &format!(
                "    {}& operator=({}&& other) = default;",
                class_name, class_name
            ),
        );
        writeln_str(
            &mut out,
            &format!("    {}(const {}&) = delete;", class_name, class_name),
        );
        writeln_str(
            &mut out,
            &format!(
                "    {}& operator=(const {}&) = delete;",
                class_name, class_name
            ),
        );
    }

    // Private section
    out.push('\n');
    writeln_str(&mut out, "private:");
    writeln_str(
        &mut out,
        &format!("    rust::Box<detail::{}Inner> inner_;", class_name),
    );
    if binding.hints.direct_field_access {
        writeln_str(&mut out, "    void* data_ptr_;");
    }

    // Close class
    writeln_str(&mut out, "};");
    out.push('\n');

    // Namespace close
    writeln_str(&mut out, &format!("}} // namespace {}", namespace));

    out
}

/// Generate a header for an impl binding (methods on a class).
fn generate_impl_header(binding: &ImplBinding) -> String {
    let type_name = extract_type_name(&binding.self_ty);

    let mut out = String::with_capacity(2048);

    // File header
    write_file_header(&mut out);

    // Namespace open
    writeln_str(&mut out, "namespace horus {");
    out.push('\n');

    // Forward declare the class
    writeln_str(&mut out, &format!("class {} {{", type_name));
    writeln_str(&mut out, "public:");

    // Generate method declarations
    for method in &binding.methods {
        if !is_pub_vis(&method.vis) {
            continue;
        }
        let decl = generate_method_decl(&type_name, method);
        if !decl.is_empty() {
            writeln_str(&mut out, &format!("    {}", decl));
        }
    }

    // Private section with inner Box
    out.push('\n');
    writeln_str(&mut out, "private:");
    writeln_str(
        &mut out,
        &format!("    rust::Box<detail::{}Inner> inner_;", type_name),
    );

    // Close class
    writeln_str(&mut out, "};");
    out.push('\n');

    // Namespace close
    writeln_str(&mut out, "} // namespace horus");

    out
}

/// Generate a single method declaration.
fn generate_method_decl(class_name: &str, method: &MethodBinding) -> String {
    let name = method.name.to_string();
    let sig = &method.sig;
    let hints = &method.hints;

    // Build parameter list
    let params: Vec<String> = sig
        .params
        .iter()
        .map(|p| {
            let ty = type_to_cpp(&p.ty);
            let name = p.name.to_string();
            if p.hints.move_semantics {
                format!("{}&& {}", ty, name)
            } else {
                format!("{} {}", ty, name)
            }
        })
        .collect();
    let params_str = params.join(", ");

    // Build return type
    let ret = if let Some(ref ty) = sig.return_ty {
        let cpp_ty = type_to_cpp(ty);
        if hints.returns_unique_ptr {
            format!("std::unique_ptr<{}>", cpp_ty)
        } else if sig.is_option {
            format!("std::optional<{}>", cpp_ty)
        } else if sig.is_result {
            // Result maps to return-or-throw in C++
            cpp_ty
        } else {
            cpp_ty
        }
    } else {
        "void".to_string()
    };

    // Attributes
    let mut attrs = Vec::new();
    if hints.returns_unique_ptr {
        attrs.push("[[nodiscard]]");
    }
    if sig.receiver == Receiver::Ref {
        // const method (if not returning mutable ref)
    }

    let attrs_str = if attrs.is_empty() {
        String::new()
    } else {
        format!("{} ", attrs.join(" "))
    };

    // Const qualifier for &self methods
    let const_qual = if sig.receiver == Receiver::Ref {
        " const"
    } else {
        ""
    };

    // Static keyword for no-self methods
    match sig.receiver {
        Receiver::None => {
            format!("{}static {} {}({});", attrs_str, ret, name, params_str)
        }
        _ => {
            format!(
                "{}{} {}({}){};",
                attrs_str, ret, name, params_str, const_qual
            )
        }
    }
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Write the standard file header (pragma once, includes).
fn write_file_header(out: &mut String) {
    writeln_str(out, "// GENERATED by horus_cpp_macros — do not edit");
    writeln_str(out, "#pragma once");
    out.push('\n');
    writeln_str(out, "#include <cstddef>");
    writeln_str(out, "#include <cstdint>");
    writeln_str(out, "#include <memory>");
    writeln_str(out, "#include <optional>");
    writeln_str(out, "#include <string_view>");
    writeln_str(out, "#include \"rust/cxx.h\"");
    out.push('\n');
}

/// Map a Rust type to its C++ equivalent (simplified).
fn type_to_cpp(ty: &syn::Type) -> String {
    let s = quote_type_to_string(ty);
    match s.as_str() {
        "Self" => "Self".to_string(), // Placeholder — caller resolves
        "bool" => "bool".to_string(),
        "u8" => "uint8_t".to_string(),
        "u16" => "uint16_t".to_string(),
        "u32" => "uint32_t".to_string(),
        "u64" => "uint64_t".to_string(),
        "i8" => "int8_t".to_string(),
        "i16" => "int16_t".to_string(),
        "i32" => "int32_t".to_string(),
        "i64" => "int64_t".to_string(),
        "f32" => "float".to_string(),
        "f64" => "double".to_string(),
        "usize" => "size_t".to_string(),
        "String" => "rust::String".to_string(),
        "& str" => "std::string_view".to_string(),
        "* mut u8" => "uint8_t*".to_string(),
        "* const u8" => "const uint8_t*".to_string(),
        "()" => "void".to_string(),
        other => other.to_string(),
    }
}

/// Convert a syn::Type to a string representation.
fn quote_type_to_string(ty: &syn::Type) -> String {
    use quote::ToTokens;
    ty.to_token_stream().to_string()
}

/// Extract type name from syn::Type.
fn extract_type_name(ty: &syn::Type) -> String {
    if let syn::Type::Path(tp) = ty {
        if let Some(seg) = tp.path.segments.last() {
            return seg.ident.to_string();
        }
    }
    "Unknown".to_string()
}

fn is_pub_vis(vis: &syn::Visibility) -> bool {
    !matches!(vis, syn::Visibility::Inherited)
}

fn writeln_str(out: &mut String, s: &str) {
    out.push_str(s);
    out.push('\n');
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::{parse_impl, parse_struct};
    use crate::types::BindingItem;
    use syn::{parse_quote, ItemImpl, ItemStruct};

    fn assert_header_contains(header: &str, needle: &str) {
        assert!(
            header.contains(needle),
            "Expected header to contain \"{needle}\"\nActual:\n{header}"
        );
    }

    fn assert_header_not_contains(header: &str, needle: &str) {
        assert!(
            !header.contains(needle),
            "Expected header NOT to contain \"{needle}\"\nActual:\n{header}"
        );
    }

    #[test]
    fn struct_header_has_pragma_once() {
        let item: ItemStruct = parse_quote! {
            pub struct Foo { val: u32 }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "#pragma once");
        assert_header_contains(&header, "// GENERATED");
    }

    #[test]
    fn struct_header_has_includes() {
        let item: ItemStruct = parse_quote! {
            pub struct Foo { val: u32 }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "#include <memory>");
        assert_header_contains(&header, "#include <optional>");
        assert_header_contains(&header, "#include \"rust/cxx.h\"");
    }

    #[test]
    fn struct_header_has_namespace() {
        let item: ItemStruct = parse_quote! {
            pub struct Foo { val: u32 }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "namespace horus {");
        assert_header_contains(&header, "} // namespace horus");
    }

    #[test]
    fn struct_header_has_class() {
        let item: ItemStruct = parse_quote! {
            pub struct Publisher { inner: u64 }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "class Publisher {");
        assert_header_contains(&header, "Publisher() = delete;");
    }

    #[test]
    fn direct_field_access_generates_operator_arrow() {
        let item: ItemStruct = parse_quote! {
            #[cpp(direct_field_access)]
            pub struct LoanedSample {
                ptr: *mut u8,
                size: usize,
            }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "operator->()");
        assert_header_contains(&header, "operator*()");
        assert_header_contains(&header, "void* data_ptr_;");
    }

    #[test]
    fn raii_generates_destructor() {
        let item: ItemStruct = parse_quote! {
            #[cpp(raii)]
            pub struct SampleGuard { inner: u64 }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "~SampleGuard();");
    }

    #[test]
    fn raii_makes_move_only() {
        let item: ItemStruct = parse_quote! {
            #[cpp(raii)]
            pub struct Guard { inner: u64 }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "Guard(Guard&& other) = default;");
        assert_header_contains(&header, "Guard(const Guard&) = delete;");
    }

    #[test]
    fn custom_name_and_namespace() {
        let item: ItemStruct = parse_quote! {
            #[cpp(name = "MyPub", namespace = "horus::detail")]
            pub struct InternalPublisher { inner: u64 }
        };
        let binding = parse_struct(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Struct(binding));
        assert_header_contains(&header, "class MyPub {");
        assert_header_contains(&header, "namespace horus::detail {");
        assert_header_contains(&header, "} // namespace horus::detail");
    }

    #[test]
    fn impl_header_generates_methods() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn new(topic: &str) -> Self { todo!() }
                pub fn loan(&self) -> LoanedSample { todo!() }
                pub fn publish(&self, sample: LoanedSample) { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert_header_contains(&header, "class Publisher {");
        assert_header_contains(&header, "static Self new(std::string_view topic);");
        assert_header_contains(&header, "LoanedSample loan()");
        assert_header_contains(&header, "void publish(LoanedSample sample)");
    }

    #[test]
    fn returns_unique_ptr_hint() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                #[cpp(returns = "unique_ptr")]
                pub fn new(topic: &str) -> Self { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert_header_contains(&header, "std::unique_ptr<Self>");
        assert_header_contains(&header, "[[nodiscard]]");
    }

    #[test]
    fn option_return_becomes_optional() {
        let item: ItemImpl = parse_quote! {
            impl Subscriber {
                pub fn recv(&self) -> Option<Message> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert_header_contains(&header, "std::optional<Message>");
    }

    #[test]
    fn const_method_for_ref_self() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                pub fn get_value(&self) -> u32 { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert_header_contains(&header, "const;");
    }

    #[test]
    fn static_method_keyword() {
        let item: ItemImpl = parse_quote! {
            impl Factory {
                pub fn create(name: &str) -> Self { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert_header_contains(&header, "static ");
    }

    #[test]
    fn private_methods_excluded() {
        let item: ItemImpl = parse_quote! {
            impl Foo {
                pub fn visible(&self) {}
                fn hidden(&self) {}
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert_header_contains(&header, "visible");
        assert_header_not_contains(&header, "hidden");
    }

    #[test]
    fn primitive_type_mapping() {
        assert_eq!(type_to_cpp(&syn::parse_str("u32").unwrap()), "uint32_t");
        assert_eq!(type_to_cpp(&syn::parse_str("f64").unwrap()), "double");
        assert_eq!(type_to_cpp(&syn::parse_str("bool").unwrap()), "bool");
        assert_eq!(type_to_cpp(&syn::parse_str("usize").unwrap()), "size_t");
        assert_eq!(type_to_cpp(&syn::parse_str("*mut u8").unwrap()), "uint8_t*");
    }

    #[test]
    fn ref_str_maps_to_string_view() {
        assert_eq!(
            type_to_cpp(&syn::parse_str("&str").unwrap()),
            "std::string_view"
        );
    }

    #[test]
    fn move_semantics_param() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn publish(&self, #[cpp(move_semantics)] sample: LoanedSample) { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));
        assert_header_contains(&header, "LoanedSample&& sample");
    }

    #[test]
    fn complete_publisher_header() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                #[cpp(returns = "unique_ptr")]
                pub fn new(topic: &str) -> Self { todo!() }

                #[cpp(returns = "unique_ptr", raii)]
                pub fn loan(&self) -> LoanedSample { todo!() }

                pub fn publish(&self, #[cpp(move_semantics)] sample: LoanedSample) { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        let header = generate_cpp_header(&BindingItem::Impl(binding));

        assert_header_contains(&header, "#pragma once");
        assert_header_contains(&header, "namespace horus {");
        assert_header_contains(&header, "class Publisher {");
        assert_header_contains(
            &header,
            "[[nodiscard]] static std::unique_ptr<Self> new(std::string_view topic);",
        );
        assert_header_contains(
            &header,
            "[[nodiscard]] std::unique_ptr<LoanedSample> loan()",
        );
        assert_header_contains(&header, "void publish(LoanedSample&& sample)");
    }
}
