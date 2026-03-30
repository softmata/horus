//! Parser for `#[horus_api]` attribute macro.
//!
//! Handles two cases:
//! 1. `#[horus_api]` on a struct — extracts fields and hints
//! 2. `#[horus_api]` on an impl block — extracts methods and hints
//!
//! In both cases, the original Rust code is passed through unchanged
//! (with `#[cpp(...)]` attributes stripped to avoid confusing rustc).

use proc_macro2::TokenStream;
use quote::{quote, ToTokens};
use syn::{
    parse2, Attribute, FnArg, GenericArgument, ImplItem, Item, ItemImpl, ItemStruct, Meta, Pat,
    PathArguments, Result, ReturnType, Type, TypePath,
};

use crate::types::{
    CppHints, FieldBinding, ImplBinding, MethodBinding, MethodSig, ParamBinding, Receiver,
    StructBinding,
};

// ─── Entry Point ─────────────────────────────────────────────────────────────

/// Entry point: parse the annotated item, extract metadata, strip #[cpp(...)],
/// and pass through the cleaned code.
pub fn parse_horus_api(attr: TokenStream, item: TokenStream) -> Result<TokenStream> {
    let parsed: Item = parse2(item)?;

    match parsed {
        Item::Struct(mut s) => {
            // Parse struct-level hints from the #[horus_api] attr args
            let struct_hints = parse_cpp_hints_from_tokenstream(&attr)?;
            let _binding = parse_struct_with_hints(&s, struct_hints)?;
            // Strip #[cpp(...)] from struct and field attributes
            strip_cpp_attrs_from_struct(&mut s);
            Ok(s.to_token_stream())
        }
        Item::Impl(mut i) => {
            let _binding = parse_impl_with_hints(&i)?;
            // Strip #[cpp(...)] from methods
            strip_cpp_attrs_from_impl(&mut i);
            Ok(i.to_token_stream())
        }
        _ => Err(syn::Error::new_spanned(
            proc_macro2::TokenStream::new(),
            "#[horus_api] can only be applied to `struct` definitions or `impl` blocks.\n\
             Help: move this attribute to the struct or impl you want to expose to C++.\n\
             Example:\n  #[horus_api]\n  pub struct MyType { ... }\n\
             or:\n  #[horus_api]\n  impl MyType { ... }",
        )),
    }
}

// ─── Struct Parsing ──────────────────────────────────────────────────────────

/// Extract metadata from a struct definition with hints.
pub fn parse_struct(item: &ItemStruct) -> Result<StructBinding> {
    parse_struct_with_hints(item, CppHints::default())
}

fn parse_struct_with_hints(item: &ItemStruct, struct_hints: CppHints) -> Result<StructBinding> {
    let mut fields = Vec::new();

    match &item.fields {
        syn::Fields::Named(named) => {
            for f in &named.named {
                let name = f.ident.clone().ok_or_else(|| {
                    syn::Error::new_spanned(
                        f,
                        "tuple struct fields are not supported by #[horus_api]",
                    )
                })?;
                let hints = parse_cpp_hints_from_attrs(&f.attrs)?;
                fields.push(FieldBinding {
                    name,
                    ty: f.ty.clone(),
                    vis: f.vis.clone(),
                    hints,
                });
            }
        }
        syn::Fields::Unit => {}
        syn::Fields::Unnamed(_) => {
            return Err(syn::Error::new_spanned(
                item,
                "#[horus_api] does not support tuple structs.\n\
                 Help: use named fields instead.\n\
                 Change: `pub struct Foo(u32, u32);`\n\
                 To:     `pub struct Foo { pub a: u32, pub b: u32 }`",
            ));
        }
    }

    // Merge struct-level hints from outer attrs
    let mut hints = parse_cpp_hints_from_attrs(&item.attrs)?;
    merge_hints(&mut hints, &struct_hints);

    Ok(StructBinding {
        name: item.ident.clone(),
        generics: item.generics.clone(),
        fields,
        hints,
        vis: item.vis.clone(),
    })
}

// ─── Impl Parsing ────────────────────────────────────────────────────────────

/// Extract metadata from an impl block with hints.
pub fn parse_impl(item: &ItemImpl) -> Result<ImplBinding> {
    parse_impl_with_hints(item)
}

fn parse_impl_with_hints(item: &ItemImpl) -> Result<ImplBinding> {
    let mut methods = Vec::new();

    for impl_item in &item.items {
        if let ImplItem::Fn(method) = impl_item {
            let hints = parse_cpp_hints_from_attrs(&method.attrs)?;
            let sig = parse_method_sig(&method.sig)?;
            methods.push(MethodBinding {
                name: method.sig.ident.clone(),
                sig,
                hints,
                vis: method.vis.clone(),
            });
        }
    }

    Ok(ImplBinding {
        self_ty: (*item.self_ty).clone(),
        generics: item.generics.clone(),
        methods,
    })
}

// ─── Method Signature Parsing ────────────────────────────────────────────────

fn parse_method_sig(sig: &syn::Signature) -> Result<MethodSig> {
    let receiver = parse_receiver(sig);
    let params = parse_params(sig)?;
    let (return_ty, is_result, is_option) = parse_return_type(&sig.output);

    Ok(MethodSig {
        receiver,
        params,
        return_ty,
        is_result,
        is_option,
    })
}

fn parse_receiver(sig: &syn::Signature) -> Receiver {
    match sig.inputs.first() {
        Some(FnArg::Receiver(recv)) => {
            if recv.reference.is_some() {
                if recv.mutability.is_some() {
                    Receiver::RefMut
                } else {
                    Receiver::Ref
                }
            } else {
                Receiver::Owned
            }
        }
        _ => Receiver::None,
    }
}

fn parse_params(sig: &syn::Signature) -> Result<Vec<ParamBinding>> {
    let mut params = Vec::new();

    for arg in &sig.inputs {
        if let FnArg::Typed(pat_type) = arg {
            let name = match &*pat_type.pat {
                Pat::Ident(pat_ident) => pat_ident.ident.clone(),
                _ => {
                    return Err(syn::Error::new_spanned(
                        &pat_type.pat,
                        "#[horus_api] requires simple parameter names, not patterns.\n\
                         Help: change destructuring patterns to a single identifier.\n\
                         Change: `fn foo((a, b): (u32, u32))`\n\
                         To:     `fn foo(pair: (u32, u32))`",
                    ));
                }
            };
            let hints = parse_cpp_hints_from_attrs(&pat_type.attrs)?;
            params.push(ParamBinding {
                name,
                ty: (*pat_type.ty).clone(),
                hints,
            });
        }
    }

    Ok(params)
}

fn parse_return_type(ret: &ReturnType) -> (Option<Type>, bool, bool) {
    match ret {
        ReturnType::Default => (None, false, false),
        ReturnType::Type(_, ty) => {
            if let Some(inner) = unwrap_type_wrapper(ty, "Result") {
                (Some(inner), true, false)
            } else if let Some(inner) = unwrap_type_wrapper(ty, "HorusResult") {
                (Some(inner), true, false)
            } else if let Some(inner) = unwrap_type_wrapper(ty, "Option") {
                (Some(inner), false, true)
            } else {
                (Some((**ty).clone()), false, false)
            }
        }
    }
}

fn unwrap_type_wrapper(ty: &Type, wrapper_name: &str) -> Option<Type> {
    if let Type::Path(TypePath { path, .. }) = ty {
        let segment = path.segments.last()?;
        if segment.ident == wrapper_name {
            if let PathArguments::AngleBracketed(args) = &segment.arguments {
                if let Some(GenericArgument::Type(inner)) = args.args.first() {
                    return Some(inner.clone());
                }
            }
        }
    }
    None
}

// ─── #[cpp(...)] Hint Parsing ────────────────────────────────────────────────

const KNOWN_HINTS: &[&str] = &[
    "move_semantics",
    "raii",
    "direct_field_access",
    "returns",
    "name",
    "namespace",
];

/// Parse `#[cpp(...)]` hints from a list of syn Attributes.
pub fn parse_cpp_hints_from_attrs(attrs: &[Attribute]) -> Result<CppHints> {
    let mut hints = CppHints::default();

    for attr in attrs {
        if !attr.path().is_ident("cpp") {
            continue;
        }
        parse_single_cpp_attr(attr, &mut hints)?;
    }

    Ok(hints)
}

/// Parse #[cpp(...)] hints from a raw TokenStream (for the outer attr arg).
fn parse_cpp_hints_from_tokenstream(tokens: &TokenStream) -> Result<CppHints> {
    if tokens.is_empty() {
        return Ok(CppHints::default());
    }
    // Parse as: direct_field_access, move, raii, returns = "unique_ptr"
    // Wrap in a dummy attribute for syn parsing
    let dummy: Attribute = syn::parse_quote!(#[cpp(#tokens)]);
    let mut hints = CppHints::default();
    parse_single_cpp_attr(&dummy, &mut hints)?;
    Ok(hints)
}

/// Parse a single `#[cpp(...)]` attribute into hints.
fn parse_single_cpp_attr(attr: &Attribute, hints: &mut CppHints) -> Result<()> {
    attr.parse_nested_meta(|meta| {
        if meta.path.is_ident("move_semantics") {
            hints.move_semantics = true;
            Ok(())
        } else if meta.path.is_ident("raii") {
            hints.raii = true;
            Ok(())
        } else if meta.path.is_ident("direct_field_access") {
            hints.direct_field_access = true;
            Ok(())
        } else if meta.path.is_ident("returns") {
            // #[cpp(returns = "unique_ptr")]
            let value: syn::LitStr = meta.value()?.parse()?;
            match value.value().as_str() {
                "unique_ptr" => hints.returns_unique_ptr = true,
                other => {
                    return Err(meta.error(format!(
                        "unknown #[cpp(returns)] value \"{other}\", expected \"unique_ptr\""
                    )));
                }
            }
            Ok(())
        } else if meta.path.is_ident("name") {
            let value: syn::LitStr = meta.value()?.parse()?;
            hints.cpp_name = Some(value.value());
            Ok(())
        } else if meta.path.is_ident("namespace") {
            let value: syn::LitStr = meta.value()?.parse()?;
            hints.cpp_namespace = Some(value.value());
            Ok(())
        } else {
            let path = meta.path.to_token_stream().to_string();
            Err(meta.error(format!(
                "unknown #[cpp] hint \"{path}\", expected one of: {}",
                KNOWN_HINTS.join(", ")
            )))
        }
    })
}

// ─── Attribute Stripping ─────────────────────────────────────────────────────

/// Returns true if an attribute is `#[cpp(...)]`.
fn is_cpp_attr(attr: &Attribute) -> bool {
    attr.path().is_ident("cpp")
}

/// Strip all `#[cpp(...)]` attributes from a struct and its fields.
fn strip_cpp_attrs_from_struct(item: &mut ItemStruct) {
    item.attrs.retain(|a| !is_cpp_attr(a));
    if let syn::Fields::Named(ref mut fields) = item.fields {
        for f in &mut fields.named {
            f.attrs.retain(|a| !is_cpp_attr(a));
        }
    }
}

/// Strip all `#[cpp(...)]` attributes from an impl block and its methods.
fn strip_cpp_attrs_from_impl(item: &mut ItemImpl) {
    item.attrs.retain(|a| !is_cpp_attr(a));
    for impl_item in &mut item.items {
        if let ImplItem::Fn(method) = impl_item {
            method.attrs.retain(|a| !is_cpp_attr(a));
        }
    }
}

// ─── Hint Merging ────────────────────────────────────────────────────────────

/// Merge `other` hints into `target` (other wins on conflict).
fn merge_hints(target: &mut CppHints, other: &CppHints) {
    if other.direct_field_access {
        target.direct_field_access = true;
    }
    if other.move_semantics {
        target.move_semantics = true;
    }
    if other.returns_unique_ptr {
        target.returns_unique_ptr = true;
    }
    if other.raii {
        target.raii = true;
    }
    if other.cpp_name.is_some() {
        target.cpp_name.clone_from(&other.cpp_name);
    }
    if other.cpp_namespace.is_some() {
        target.cpp_namespace.clone_from(&other.cpp_namespace);
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use syn::parse_quote;

    // ─── Struct Parsing ──────────────────────────────────────────────────

    #[test]
    fn parse_simple_struct() {
        let item: ItemStruct = parse_quote! {
            pub struct Publisher {
                inner: TopicPublisher,
                name: String,
            }
        };
        let binding = parse_struct(&item).unwrap();
        assert_eq!(binding.name, "Publisher");
        assert_eq!(binding.fields.len(), 2);
        assert_eq!(binding.fields[0].name, "inner");
        assert_eq!(binding.fields[1].name, "name");
    }

    #[test]
    fn parse_unit_struct() {
        let item: ItemStruct = parse_quote! { pub struct EmptyMarker; };
        let binding = parse_struct(&item).unwrap();
        assert_eq!(binding.name, "EmptyMarker");
        assert_eq!(binding.fields.len(), 0);
    }

    #[test]
    fn parse_tuple_struct_rejected() {
        let item: ItemStruct = parse_quote! { pub struct Wrapper(u32, u32); };
        assert!(parse_struct(&item).is_err());
    }

    #[test]
    fn parse_generic_struct() {
        let item: ItemStruct = parse_quote! {
            pub struct Topic<T: Message> {
                data: T,
                count: usize,
            }
        };
        let binding = parse_struct(&item).unwrap();
        assert_eq!(binding.name, "Topic");
        assert!(!binding.generics.params.is_empty());
        assert_eq!(binding.fields.len(), 2);
    }

    // ─── Impl Parsing ────────────────────────────────────────────────────

    #[test]
    fn parse_impl_with_methods() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                pub fn new(topic: &str) -> Self { todo!() }
                pub fn loan(&self) -> LoanedSample { todo!() }
                pub fn publish(&self, sample: LoanedSample) { todo!() }
                fn private_helper(&mut self) { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        assert_eq!(binding.methods.len(), 4);
        assert_eq!(binding.methods[0].sig.receiver, Receiver::None);
        assert_eq!(binding.methods[1].sig.receiver, Receiver::Ref);
        assert_eq!(binding.methods[2].sig.params.len(), 1);
        assert_eq!(binding.methods[3].sig.receiver, Receiver::RefMut);
    }

    #[test]
    fn parse_result_return_type() {
        let item: ItemImpl = parse_quote! {
            impl Service {
                pub fn call(&self, req: Request) -> Result<Response> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        assert!(binding.methods[0].sig.is_result);
        assert!(!binding.methods[0].sig.is_option);
    }

    #[test]
    fn parse_option_return_type() {
        let item: ItemImpl = parse_quote! {
            impl Subscriber {
                pub fn recv(&self) -> Option<Message> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        assert!(binding.methods[0].sig.is_option);
    }

    #[test]
    fn parse_horus_result_return_type() {
        let item: ItemImpl = parse_quote! {
            impl Topic {
                pub fn send(&self, msg: CmdVel) -> HorusResult<()> { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        assert!(binding.methods[0].sig.is_result);
    }

    #[test]
    fn parse_owned_receiver() {
        let item: ItemImpl = parse_quote! {
            impl Builder {
                pub fn build(self) -> Node { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        assert_eq!(binding.methods[0].sig.receiver, Receiver::Owned);
    }

    #[test]
    fn parse_no_return() {
        let item: ItemImpl = parse_quote! {
            impl Node {
                pub fn tick(&mut self) {}
            }
        };
        let binding = parse_impl(&item).unwrap();
        assert!(binding.methods[0].sig.return_ty.is_none());
    }

    // ─── #[cpp(...)] Hints ───────────────────────────────────────────────

    #[test]
    fn hint_move_semantics() {
        // `move` is a Rust keyword, so in attributes we use `move_semantics`
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(move_semantics)])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert!(hints.move_semantics);
        assert!(!hints.raii);
    }

    #[test]
    fn hint_raii() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(raii)])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert!(hints.raii);
    }

    #[test]
    fn hint_direct_field_access() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(direct_field_access)])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert!(hints.direct_field_access);
    }

    #[test]
    fn hint_returns_unique_ptr() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(returns = "unique_ptr")])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert!(hints.returns_unique_ptr);
    }

    #[test]
    fn hint_custom_name() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(name = "MyPublisher")])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert_eq!(hints.cpp_name, Some("MyPublisher".to_string()));
    }

    #[test]
    fn hint_custom_namespace() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(namespace = "horus::detail")])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert_eq!(hints.cpp_namespace, Some("horus::detail".to_string()));
    }

    #[test]
    fn hint_multiple_combined() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(raii, returns = "unique_ptr")])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert!(hints.raii);
        assert!(hints.returns_unique_ptr);
        assert!(!hints.move_semantics);
    }

    #[test]
    fn hint_unknown_rejected() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(foobar)])];
        let result = parse_cpp_hints_from_attrs(&attrs);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("unknown #[cpp] hint"));
        assert!(err.contains("expected one of"));
    }

    #[test]
    fn hint_invalid_returns_value() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[cpp(returns = "shared_ptr")])];
        let result = parse_cpp_hints_from_attrs(&attrs);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("unique_ptr"));
    }

    #[test]
    fn non_cpp_attrs_ignored() {
        let attrs: Vec<Attribute> = vec![parse_quote!(#[derive(Debug)]), parse_quote!(#[repr(C)])];
        let hints = parse_cpp_hints_from_attrs(&attrs).unwrap();
        assert!(!hints.has_any());
    }

    #[test]
    fn no_attrs_returns_default() {
        let hints = parse_cpp_hints_from_attrs(&[]).unwrap();
        assert!(!hints.has_any());
    }

    // ─── Struct-Level Hints ──────────────────────────────────────────────

    #[test]
    fn struct_level_hints_from_attrs() {
        let item: ItemStruct = parse_quote! {
            #[cpp(direct_field_access)]
            pub struct LoanedSample {
                ptr: *mut u8,
                size: usize,
            }
        };
        let binding = parse_struct(&item).unwrap();
        assert!(binding.hints.direct_field_access);
    }

    // ─── Method-Level Hints ──────────────────────────────────────────────

    #[test]
    fn method_level_hints() {
        let item: ItemImpl = parse_quote! {
            impl Publisher {
                #[cpp(returns = "unique_ptr")]
                pub fn new(topic: &str) -> Self { todo!() }

                #[cpp(raii, returns = "unique_ptr")]
                pub fn loan(&self) -> LoanedSample { todo!() }
            }
        };
        let binding = parse_impl(&item).unwrap();
        assert!(binding.methods[0].hints.returns_unique_ptr);
        assert!(!binding.methods[0].hints.raii);
        assert!(binding.methods[1].hints.returns_unique_ptr);
        assert!(binding.methods[1].hints.raii);
    }

    // ─── Attribute Stripping ─────────────────────────────────────────────

    #[test]
    fn cpp_attrs_stripped_from_struct_output() {
        let input: TokenStream = quote! {
            #[cpp(direct_field_access)]
            pub struct Foo {
                pub x: f32,
            }
        };
        let output = parse_horus_api(TokenStream::new(), input).unwrap();
        let output_str = output.to_string();
        assert!(!output_str.contains("cpp"));
        assert!(output_str.contains("pub struct Foo"));
        assert!(output_str.contains("x : f32"));
    }

    #[test]
    fn cpp_attrs_stripped_from_impl_output() {
        let input: TokenStream = quote! {
            impl Foo {
                #[cpp(returns = "unique_ptr")]
                pub fn new() -> Self { todo!() }
            }
        };
        let output = parse_horus_api(TokenStream::new(), input).unwrap();
        let output_str = output.to_string();
        assert!(!output_str.contains("cpp"));
        assert!(output_str.contains("pub fn new"));
    }

    #[test]
    fn non_cpp_attrs_preserved() {
        let input: TokenStream = quote! {
            #[derive(Debug, Clone)]
            #[repr(C)]
            pub struct Bar {
                pub val: u32,
            }
        };
        let output = parse_horus_api(TokenStream::new(), input).unwrap();
        let output_str = output.to_string();
        assert!(output_str.contains("derive"));
        assert!(output_str.contains("repr"));
    }

    // ─── Identity Transform ──────────────────────────────────────────────

    #[test]
    fn rejects_non_struct_non_impl() {
        let input: TokenStream = quote! { pub fn standalone() {} };
        assert!(parse_horus_api(TokenStream::new(), input).is_err());
    }

    #[test]
    fn rejects_enum() {
        let input: TokenStream = quote! { pub enum Foo { A, B } };
        assert!(parse_horus_api(TokenStream::new(), input).is_err());
    }
}
