//! Message macro implementation
//!
//! Provides the `message!` macro for easy message type definitions with automatic
//! zero-copy optimization and String field conversion.

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
    parse::{Parse, ParseStream},
    punctuated::Punctuated,
    token::Comma,
    Attribute, Field, Ident, Lit, Meta, Result, Token, Type,
};

/// Parse either tuple-style or struct-style message definition
pub enum MessageInput {
    /// Tuple-style: `Position = (f32, f32)`
    Tuple { name: Ident, types: Vec<Type> },
    /// Struct-style: `MyMessage { x: u8, y: u8 }`
    Struct {
        name: Ident,
        fields: Vec<FieldInfo>,
    },
}

/// Field information including attributes
pub struct FieldInfo {
    pub name: Ident,
    pub ty: Type,
    pub attrs: Vec<Attribute>,
}

impl Parse for MessageInput {
    fn parse(input: ParseStream) -> Result<Self> {
        let name: Ident = input.parse()?;

        // Check if it's tuple-style (with =) or struct-style (with {)
        if input.peek(Token![=]) {
            // Tuple-style: Position = (f32, f32)
            input.parse::<Token![=]>()?;

            let content;
            syn::parenthesized!(content in input);

            let types: Punctuated<Type, Comma> =
                content.parse_terminated(Type::parse, Token![,])?;
            let types: Vec<Type> = types.into_iter().collect();

            Ok(MessageInput::Tuple { name, types })
        } else {
            // Struct-style: MyMessage { x: u8, y: u8 }
            let content;
            syn::braced!(content in input);

            let fields: Punctuated<Field, Comma> =
                content.parse_terminated(Field::parse_named, Token![,])?;

            let fields: Vec<FieldInfo> = fields
                .into_iter()
                .map(|f| FieldInfo {
                    name: f.ident.unwrap(),
                    ty: f.ty,
                    attrs: f.attrs,
                })
                .collect();

            Ok(MessageInput::Struct { name, fields })
        }
    }
}

/// Generate the complete message implementation
pub fn generate_message(input: MessageInput) -> TokenStream {
    match input {
        MessageInput::Tuple { name, types } => generate_tuple_message(name, types),
        MessageInput::Struct { name, fields } => generate_struct_message(name, fields),
    }
}

/// Generate a tuple-style message
fn generate_tuple_message(name: Ident, types: Vec<Type>) -> TokenStream {
    let field_list = types.iter().map(|ty| {
        quote! { pub #ty }
    });

    // Check if all types are Pod-compatible
    let is_pod = types.iter().all(is_pod_type);
    let zero_copy_impl = if is_pod {
        generate_zero_copy_impl(&name)
    } else {
        quote! {}
    };

    quote! {
        #[derive(Debug, Clone, ::horus::serde::Serialize, ::horus::serde::Deserialize)]
        #[repr(C)]
        pub struct #name(#(#field_list),*);

        impl ::horus::core::LogSummary for #name {
            fn log_summary(&self) -> ::std::string::String {
                format!("{:?}", self)
            }
        }

        #zero_copy_impl
    }
}

/// Generate a struct-style message with named fields
fn generate_struct_message(name: Ident, fields: Vec<FieldInfo>) -> TokenStream {
    // Process fields: detect String types and convert to FixedString<N>
    let processed_fields: Vec<_> = fields
        .iter()
        .map(|f| process_field(f))
        .collect();

    // Internal struct fields (String becomes FixedString<N>)
    let internal_field_defs = processed_fields.iter().map(|pf| {
        let field_name = &pf.field_name;
        let internal_type = &pf.internal_type;
        quote! { pub #field_name: #internal_type }
    });

    // Check if all fields are Pod-compatible
    let is_pod = processed_fields.iter().all(|pf| pf.is_pod);

    // Generate constructor
    let constructor = generate_constructor(&name, &processed_fields);

    // Generate string accessors for String fields
    let string_accessors = generate_string_accessors(&processed_fields);

    // Generate zero-copy methods if Pod-compatible
    let zero_copy_impl = if is_pod {
        generate_zero_copy_impl(&name)
    } else {
        quote! {}
    };

    quote! {
        #[derive(Debug, Clone, ::horus::serde::Serialize, ::horus::serde::Deserialize)]
        #[repr(C)]
        pub struct #name {
            #(#internal_field_defs),*
        }

        impl ::horus::core::LogSummary for #name {
            fn log_summary(&self) -> ::std::string::String {
                format!("{:?}", self)
            }
        }

        impl #name {
            #constructor
            #(#string_accessors)*
        }

        #zero_copy_impl
    }
}

/// Processed field information
struct ProcessedField {
    field_name: Ident,
    internal_type: Type,
    is_string: bool,
    is_pod: bool,
    max_len: Option<usize>,
}

/// Process a field: detect String and convert to FixedString<N>
fn process_field(field: &FieldInfo) -> ProcessedField {
    let is_string = is_string_type(&field.ty);

    if is_string {
        // Extract max_len from #[max_len = N] attribute (default 64)
        let max_len = extract_max_len(&field.attrs).unwrap_or(64);

        // Convert String to FixedString<N>
        let internal_type: Type = syn::parse_quote! {
            ::horus::core::FixedString<#max_len>
        };

        ProcessedField {
            field_name: field.name.clone(),
            internal_type,
            is_string: true,
            is_pod: true,  // FixedString is Pod
            max_len: Some(max_len),
        }
    } else {
        ProcessedField {
            field_name: field.name.clone(),
            internal_type: field.ty.clone(),
            is_string: false,
            is_pod: is_pod_type(&field.ty),
            max_len: None,
        }
    }
}

/// Generate constructor that accepts String for string fields
fn generate_constructor(_name: &Ident, fields: &[ProcessedField]) -> TokenStream {
    let params = fields.iter().map(|pf| {
        let field_name = &pf.field_name;
        if pf.is_string {
            quote! { #field_name: impl AsRef<str> }
        } else {
            let ty = &pf.internal_type;
            quote! { #field_name: #ty }
        }
    });

    let assignments = fields.iter().map(|pf| {
        let field_name = &pf.field_name;
        if pf.is_string {
            quote! {
                #field_name: ::horus::core::FixedString::from_str(#field_name.as_ref())
            }
        } else {
            quote! { #field_name }
        }
    });

    quote! {
        pub fn new(#(#params),*) -> Self {
            Self {
                #(#assignments),*
            }
        }
    }
}

/// Generate string accessor methods for String fields
fn generate_string_accessors(fields: &[ProcessedField]) -> Vec<TokenStream> {
    fields
        .iter()
        .filter(|pf| pf.is_string)
        .flat_map(|pf| {
            let field_name = &pf.field_name;
            let setter_name = format_ident!("set_{}", field_name);

            vec![
                // Getter: name() -> &str
                quote! {
                    #[inline]
                    pub fn #field_name(&self) -> &str {
                        self.#field_name.as_str()
                    }
                },
                // Setter: set_name(s: impl AsRef<str>)
                quote! {
                    #[inline]
                    pub fn #setter_name(&mut self, value: impl AsRef<str>) {
                        self.#field_name = ::horus::core::FixedString::from_str(value.as_ref());
                    }
                },
            ]
        })
        .collect()
}

/// Generate zero-copy implementation (SIZE, as_bytes, from_bytes)
fn generate_zero_copy_impl(name: &Ident) -> TokenStream {
    quote! {
        // SAFETY: All fields are Pod-compatible (primitives, arrays, or FixedString)
        unsafe impl ::horus::core::bytemuck::Pod for #name {}
        unsafe impl ::horus::core::bytemuck::Zeroable for #name {}

        impl #name {
            /// Size of this message in bytes (compile-time constant)
            pub const SIZE: usize = ::std::mem::size_of::<Self>();

            /// Convert to raw bytes (zero-copy)
            #[inline]
            pub fn as_bytes(&self) -> &[u8] {
                ::horus::core::bytemuck::bytes_of(self)
            }

            /// Create from raw bytes (zero-copy)
            ///
            /// Returns None if bytes are too short.
            #[inline]
            pub fn from_bytes(bytes: &[u8]) -> Option<&Self> {
                if bytes.len() >= Self::SIZE {
                    Some(::horus::core::bytemuck::from_bytes(&bytes[..Self::SIZE]))
                } else {
                    None
                }
            }

            /// Create from raw bytes, copying to ensure alignment
            #[inline]
            pub fn from_bytes_copy(bytes: &[u8]) -> Option<Self> {
                if bytes.len() >= Self::SIZE {
                    let mut result: Self = unsafe { ::std::mem::zeroed() };
                    let dst = ::horus::core::bytemuck::bytes_of_mut(&mut result);
                    dst.copy_from_slice(&bytes[..Self::SIZE]);
                    Some(result)
                } else {
                    None
                }
            }
        }
    }
}

/// Check if a type is String
fn is_string_type(ty: &Type) -> bool {
    if let Type::Path(type_path) = ty {
        if let Some(segment) = type_path.path.segments.last() {
            return segment.ident == "String";
        }
    }
    false
}

/// Extract max_len attribute value
fn extract_max_len(attrs: &[Attribute]) -> Option<usize> {
    for attr in attrs {
        if let Meta::NameValue(nv) = &attr.meta {
            if nv.path.is_ident("max_len") {
                if let syn::Expr::Lit(expr_lit) = &nv.value {
                    if let Lit::Int(lit_int) = &expr_lit.lit {
                        return lit_int.base10_parse().ok();
                    }
                }
            }
        }
    }
    None
}

/// Check if a type is Pod-compatible (primitives, arrays, FixedString)
fn is_pod_type(ty: &Type) -> bool {
    match ty {
        Type::Path(type_path) => {
            if let Some(segment) = type_path.path.segments.last() {
                let ident = &segment.ident;
                // Primitives
                if matches!(
                    ident.to_string().as_str(),
                    "u8" | "u16" | "u32" | "u64" | "u128"
                        | "i8" | "i16" | "i32" | "i64" | "i128"
                        | "f32" | "f64"
                        | "bool"
                ) {
                    return true;
                }
                // FixedString<N> is Pod
                if ident == "FixedString" {
                    return true;
                }
            }
            false
        }
        Type::Array(type_array) => {
            // Arrays of Pod are Pod
            is_pod_type(&type_array.elem)
        }
        _ => false,
    }
}
