//! Macro for defining HORUS service types.
//!
//! The [`service!`] macro provides a declarative way to define service types
//! with their Request and Response types.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::service;
//!
//! service! {
//!     /// Add two integers
//!     AddTwoInts {
//!         request {
//!             a: i64,
//!             b: i64,
//!         }
//!         response {
//!             sum: i64,
//!         }
//!     }
//! }
//! // Generated types:
//! // - AddTwoIntsRequest { a: i64, b: i64 }
//! // - AddTwoIntsResponse { sum: i64 }
//! // - struct AddTwoInts;  (implements Service)
//! ```
//!
//! # Generated Code
//!
//! For a service named `AddTwoInts`, the macro generates:
//! - `AddTwoIntsRequest` — the request payload struct
//! - `AddTwoIntsResponse` — the response payload struct
//! - `AddTwoInts` — the marker type implementing [`Service`]
//!
//! [`Service`]: crate::services::Service

/// Macro for defining HORUS service types.
///
/// See the module documentation for usage examples.
#[macro_export]
macro_rules! service {
    (
        $(#[$svc_meta:meta])*
        $svc_name:ident {
            request {
                $(
                    $(#[$req_field_meta:meta])*
                    $req_field:ident : $req_type:ty
                ),* $(,)?
            }
            response {
                $(
                    $(#[$res_field_meta:meta])*
                    $res_field:ident : $res_type:ty
                ),* $(,)?
            }
        }
    ) => {
        $crate::paste::paste! {
            // ===== Request type =====
            $(#[$svc_meta])*
            #[doc = concat!("Request type for the `", stringify!($svc_name), "` service.")]
            #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
            pub struct [<$svc_name Request>] {
                $(
                    $(#[$req_field_meta])*
                    pub $req_field: $req_type,
                )*
            }

            // ===== Response type =====
            #[doc = concat!("Response type for the `", stringify!($svc_name), "` service.")]
            #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
            pub struct [<$svc_name Response>] {
                $(
                    $(#[$res_field_meta])*
                    pub $res_field: $res_type,
                )*
            }

            // ===== Service marker type =====
            #[doc = concat!("Service marker type for `", stringify!($svc_name), "`.")]
            #[doc = ""]
            #[doc = "This is a zero-sized marker type that implements the `Service` trait."]
            $(#[$svc_meta])*
            pub struct $svc_name;

            impl $crate::services::Service for $svc_name {
                type Request = [<$svc_name Request>];
                type Response = [<$svc_name Response>];

                fn name() -> &'static str {
                    // Convert CamelCase → snake_case at runtime, leaked for 'static lifetime.
                    // This runs once per service type due to the `once_cell` guard.
                    {
                        use std::sync::OnceLock;
                        static NAME: OnceLock<&'static str> = OnceLock::new();
                        NAME.get_or_init(|| {
                            fn to_snake(s: &str) -> String {
                                let mut out = String::new();
                                for (i, c) in s.chars().enumerate() {
                                    if c.is_uppercase() {
                                        if i > 0 { out.push('_'); }
                                        out.push(c.to_ascii_lowercase());
                                    } else {
                                        out.push(c);
                                    }
                                }
                                out
                            }
                            Box::leak(to_snake(stringify!($svc_name)).into_boxed_str())
                        })
                    }
                }
            }
        }
    };
}
