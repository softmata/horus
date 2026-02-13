use proc_macro2::TokenStream;
use quote::quote;
use syn::DeriveInput;

pub fn derive_log_summary(input: DeriveInput) -> TokenStream {
    let name = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();

    quote! {
        impl #impl_generics ::horus_core::core::LogSummary for #name #ty_generics #where_clause {
            fn log_summary(&self) -> ::std::string::String {
                format!("{:?}", self)
            }
        }
    }
}
