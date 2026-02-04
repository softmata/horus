//! ROS2 Message macro implementation
//!
//! Provides the `ros2_msg!` macro for generating HORUS-compatible message types
//! from ROS2 message definitions with CDR serialization support.
//!
//! # Example
//!
//! ```rust,ignore
//! // Import a well-known ROS2 message type
//! ros2_msg!(geometry_msgs::Twist);
//! ros2_msg!(sensor_msgs::Imu);
//!
//! // Define a custom ROS2-compatible message inline
//! ros2_msg! {
//!     my_msgs::RobotState {
//!         std_msgs::Header header,
//!         geometry_msgs::Pose pose,
//!         geometry_msgs::Twist velocity,
//!         float64 battery_level,
//!     }
//! }
//! ```

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
    parse::{Parse, ParseStream},
    Ident, Result, Token,
};

/// Input for ros2_msg! macro
pub enum Ros2MsgInput {
    /// Import existing type: `geometry_msgs::Twist`
    Import { package: Ident, name: Ident },
    /// Define custom message inline
    Define {
        package: Ident,
        name: Ident,
        fields: Vec<Ros2Field>,
    },
}

/// A field in a ROS2 message definition
pub struct Ros2Field {
    /// Field type (can be primitive or nested message)
    pub field_type: Ros2Type,
    /// Field name
    pub name: Ident,
    /// Is this an array?
    pub is_array: bool,
    /// Fixed array size (None for unbounded)
    pub array_size: Option<usize>,
}

/// ROS2 type reference
pub enum Ros2Type {
    /// Primitive type (float64, int32, string, etc.)
    Primitive(Ident),
    /// Nested message type (package::Name)
    Message { package: Ident, name: Ident },
}

impl Parse for Ros2MsgInput {
    fn parse(input: ParseStream) -> Result<Self> {
        // Parse package name
        let package: Ident = input.parse()?;
        input.parse::<Token![::]>()?;
        let name: Ident = input.parse()?;

        // Check if it's just an import or a full definition
        if input.peek(syn::token::Brace) {
            // Full definition: my_msgs::RobotState { ... }
            let content;
            syn::braced!(content in input);

            let mut fields = Vec::new();
            while !content.is_empty() {
                fields.push(parse_ros2_field(&content)?);
                // Consume trailing comma or newline
                let _ = content.parse::<Token![,]>();
            }

            Ok(Ros2MsgInput::Define {
                package,
                name,
                fields,
            })
        } else {
            // Just an import
            Ok(Ros2MsgInput::Import { package, name })
        }
    }
}

/// Parse a ROS2 field: `float64 x` or `geometry_msgs::Point position`
fn parse_ros2_field(input: ParseStream) -> Result<Ros2Field> {
    // Parse type (can be `float64` or `geometry_msgs::Point`)
    let first_ident: Ident = input.parse()?;

    let field_type = if input.peek(Token![::]) {
        // Nested message type: geometry_msgs::Point
        input.parse::<Token![::]>()?;
        let type_name: Ident = input.parse()?;
        Ros2Type::Message {
            package: first_ident,
            name: type_name,
        }
    } else {
        // Primitive type
        Ros2Type::Primitive(first_ident)
    };

    // Check for array syntax
    let (is_array, array_size) = if input.peek(syn::token::Bracket) {
        let content;
        syn::bracketed!(content in input);
        if content.is_empty() {
            (true, None) // Unbounded array
        } else {
            let size: syn::LitInt = content.parse()?;
            (true, Some(size.base10_parse()?)) // Fixed-size array
        }
    } else {
        (false, None)
    };

    // Parse field name
    let name: Ident = input.parse()?;

    Ok(Ros2Field {
        field_type,
        name,
        is_array,
        array_size,
    })
}

/// Generate code for ros2_msg! macro
pub fn generate_ros2_msg(input: Ros2MsgInput) -> TokenStream {
    match input {
        Ros2MsgInput::Import { package, name } => generate_import(package, name),
        Ros2MsgInput::Define {
            package,
            name,
            fields,
        } => generate_definition(package, name, fields),
    }
}

/// Generate re-export for well-known ROS2 message types
fn generate_import(package: Ident, name: Ident) -> TokenStream {
    // Map ROS2 package names to horus_ros2_msgs modules
    let module = package_to_module(&package);

    quote! {
        pub use ::horus_ros2_msgs::#module::#name;

        impl ::horus::core::LogSummary for #name {
            fn log_summary(&self) -> ::std::string::String {
                format!("{:?}", self)
            }
        }
    }
}

/// Generate a custom ROS2-compatible message definition
fn generate_definition(package: Ident, name: Ident, fields: Vec<Ros2Field>) -> TokenStream {
    // Generate field definitions
    let field_defs = fields.iter().map(|f| {
        let field_name = &f.name;
        let rust_type = ros2_type_to_rust(&f.field_type, f.is_array, f.array_size);
        quote! { pub #field_name: #rust_type }
    });

    // Generate constructor parameters
    let constructor_params = fields.iter().map(|f| {
        let field_name = &f.name;
        let rust_type = ros2_type_to_rust(&f.field_type, f.is_array, f.array_size);
        quote! { #field_name: #rust_type }
    });

    // Generate constructor body
    let constructor_fields = fields.iter().map(|f| {
        let field_name = &f.name;
        quote! { #field_name }
    });

    // Generate LogSummary field formatting
    let log_fields = fields.iter().map(|f| {
        let field_name = &f.name;
        let name_str = field_name.to_string();
        quote! { format!("{}: {:?}", #name_str, self.#field_name) }
    });

    // Check if we can derive Copy (no arrays, strings, or nested non-Copy types)
    let is_copy = fields
        .iter()
        .all(|f| !f.is_array && is_copy_type(&f.field_type));
    let copy_derive = if is_copy {
        quote! { Copy, }
    } else {
        quote! {}
    };

    // Create module name from package
    let _module_name = format_ident!("{}", package);

    quote! {
        /// ROS2-compatible message: #package::#name
        #[derive(Debug, Clone, #copy_derive Default, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct #name {
            #(#field_defs),*
        }

        impl #name {
            /// Create a new #name message
            pub fn new(#(#constructor_params),*) -> Self {
                Self {
                    #(#constructor_fields),*
                }
            }
        }

        impl ::horus::core::LogSummary for #name {
            fn log_summary(&self) -> ::std::string::String {
                let parts: Vec<String> = vec![
                    #(#log_fields),*
                ];
                format!("{}({})", stringify!(#name), parts.join(", "))
            }
        }
    }
}

/// Convert package name to Rust module name
fn package_to_module(package: &Ident) -> Ident {
    // ROS2 packages use snake_case which is already valid Rust
    package.clone()
}

/// Convert ROS2 type to Rust type
fn ros2_type_to_rust(
    ros2_type: &Ros2Type,
    is_array: bool,
    array_size: Option<usize>,
) -> TokenStream {
    let base_type = match ros2_type {
        Ros2Type::Primitive(prim) => {
            let type_str = prim.to_string();
            match type_str.as_str() {
                "bool" => quote! { bool },
                "byte" | "uint8" => quote! { u8 },
                "char" => quote! { u8 },
                "float32" => quote! { f32 },
                "float64" => quote! { f64 },
                "int8" => quote! { i8 },
                "int16" => quote! { i16 },
                "int32" => quote! { i32 },
                "int64" => quote! { i64 },
                "uint16" => quote! { u16 },
                "uint32" => quote! { u32 },
                "uint64" => quote! { u64 },
                "string" => quote! { ::std::string::String },
                _ => {
                    // Assume it's a local type reference
                    quote! { #prim }
                }
            }
        }
        Ros2Type::Message { package, name } => {
            let module = package_to_module(package);
            quote! { ::horus_ros2_msgs::#module::#name }
        }
    };

    match (is_array, array_size) {
        (false, _) => base_type,
        (true, None) => quote! { Vec<#base_type> },
        (true, Some(size)) => quote! { [#base_type; #size] },
    }
}

/// Check if a ROS2 type can derive Copy
fn is_copy_type(ros2_type: &Ros2Type) -> bool {
    match ros2_type {
        Ros2Type::Primitive(prim) => {
            let type_str = prim.to_string();
            // String is not Copy
            !matches!(type_str.as_str(), "string")
        }
        Ros2Type::Message { .. } => {
            // Nested messages are typically not Copy (they may contain strings/vecs)
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_package_to_module() {
        let pkg = format_ident!("geometry_msgs");
        assert_eq!(package_to_module(&pkg).to_string(), "geometry_msgs");
    }
}
