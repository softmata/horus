use proc_macro::TokenStream;
use quote::quote;
use syn::{
    braced, parenthesized,
    parse::{Parse, ParseStream},
    parse_macro_input, Block, Error, Expr, Ident, Result, Token, Type,
};

/// Represents a topic definition in pub/sub sections
struct TopicDef {
    name: Ident,
    _colon: Token![:],
    ty: Type,
    _arrow: Token![->], // Both pub and sub use -> (section keyword differentiates them)
    topic: Expr,        // The topic string
}

/// Represents a field in the data section
struct DataField {
    name: Ident,
    _colon: Token![:],
    ty: Type,
    default: Option<(Token![=], Expr)>,
}

/// Publisher section
struct PubSection {
    _pub_token: Ident, // "pub" keyword
    fields: Vec<TopicDef>,
}

/// Subscriber section
struct SubSection {
    _sub_token: Ident, // "sub" keyword
    fields: Vec<TopicDef>,
}

/// Data section for internal state
struct DataSection {
    _data_token: Ident, // "data" keyword
    fields: Vec<DataField>,
}

/// Tick implementation
struct TickSection {
    _tick_token: Ident, // "tick" keyword
    _ctx_arg: Option<Ident>, // Parsed but not used - tick() no longer takes ctx
    body: Block,
}

/// Optional init implementation
struct InitSection {
    _init_token: Ident, // "init" keyword
    ctx_arg: Option<Ident>,
    body: Block,
}

/// Optional shutdown implementation
struct ShutdownSection {
    _shutdown_token: Ident, // "shutdown" keyword
    ctx_arg: Option<Ident>,
    body: Block,
}

/// Optional impl block for additional methods
struct ImplSection {
    _impl_token: Token![impl],
    body: Block,
}

/// Optional rate section for preferred execution rate
struct RateSection {
    _rate_token: Ident, // "rate" keyword
    rate_hz: Expr,      // The rate value in Hz
}

/// Optional explicit name section for custom node naming
struct NameSection {
    _name_token: Ident, // "name" keyword
    _colon: Token![:],
    name_value: syn::LitStr, // The quoted string like "FlightController"
}

/// The complete node definition
struct NodeDef {
    name: Ident,
    name_section: Option<NameSection>, // Optional explicit node name override
    pub_section: Option<PubSection>,
    sub_section: Option<SubSection>,
    data_section: Option<DataSection>,
    tick_section: TickSection, // Required
    init_section: Option<InitSection>,
    shutdown_section: Option<ShutdownSection>,
    impl_section: Option<ImplSection>,
    rate_section: Option<RateSection>, // Optional preferred rate
}

impl Parse for NodeDef {
    fn parse(input: ParseStream) -> Result<Self> {
        // Parse node name
        let name: Ident = input.parse()?;

        let content;
        braced!(content in input);

        let mut name_section = None;
        let mut pub_section = None;
        let mut sub_section = None;
        let mut data_section = None;
        let mut tick_section = None;
        let mut init_section = None;
        let mut shutdown_section = None;
        let mut impl_section = None;
        let mut rate_section = None;

        // Parse sections in any order
        while !content.is_empty() {
            let lookahead = content.lookahead1();

            if lookahead.peek(Token![pub]) {
                let pub_token: Token![pub] = content.parse()?;
                if pub_section.is_some() {
                    return Err(Error::new(pub_token.span, "Duplicate 'pub' section"));
                }
                // Create a fake ident for the parse function
                let section_name = Ident::new("pub", pub_token.span);
                pub_section = Some(parse_pub_section(&content, section_name)?);
            } else if lookahead.peek(Ident) {
                let section_name: Ident = content.parse()?;
                let section_str = section_name.to_string();

                match section_str.as_str() {
                    "sub" => {
                        if sub_section.is_some() {
                            return Err(Error::new(section_name.span(), "Duplicate 'sub' section"));
                        }
                        sub_section = Some(parse_sub_section(&content, section_name)?);
                    }
                    "data" => {
                        if data_section.is_some() {
                            return Err(Error::new(
                                section_name.span(),
                                "Duplicate 'data' section",
                            ));
                        }
                        data_section = Some(parse_data_section(&content, section_name)?);
                    }
                    "tick" => {
                        if tick_section.is_some() {
                            return Err(Error::new(
                                section_name.span(),
                                "Duplicate 'tick' section",
                            ));
                        }
                        tick_section = Some(parse_tick_section(&content, section_name)?);
                    }
                    "init" => {
                        if init_section.is_some() {
                            return Err(Error::new(
                                section_name.span(),
                                "Duplicate 'init' section",
                            ));
                        }
                        init_section = Some(parse_init_section(&content, section_name)?);
                    }
                    "shutdown" => {
                        if shutdown_section.is_some() {
                            return Err(Error::new(
                                section_name.span(),
                                "Duplicate 'shutdown' section",
                            ));
                        }
                        shutdown_section = Some(parse_shutdown_section(&content, section_name)?);
                    }
                    "rate" => {
                        if rate_section.is_some() {
                            return Err(Error::new(
                                section_name.span(),
                                "Duplicate 'rate' section",
                            ));
                        }
                        rate_section = Some(parse_rate_section(&content, section_name)?);
                    }
                    "name" => {
                        if name_section.is_some() {
                            return Err(Error::new(
                                section_name.span(),
                                "Duplicate 'name' section",
                            ));
                        }
                        name_section = Some(parse_name_section(&content, section_name)?);
                    }
                    _ => {
                        return Err(Error::new(section_name.span(),
                            format!("Unknown section '{}'. Expected: pub, sub, data, tick, init, shutdown, rate, name, or impl", section_str)));
                    }
                }
            } else if lookahead.peek(Token![impl]) {
                if impl_section.is_some() {
                    return Err(Error::new(content.span(), "Duplicate 'impl' section"));
                }
                impl_section = Some(parse_impl_section(&content)?);
            } else {
                return Err(lookahead.error());
            }
        }

        // Ensure tick section exists (required)
        let tick_section = tick_section
            .ok_or_else(|| Error::new(name.span(), "Missing required 'tick' section"))?;

        Ok(NodeDef {
            name,
            name_section,
            pub_section,
            sub_section,
            data_section,
            tick_section,
            init_section,
            shutdown_section,
            impl_section,
            rate_section,
        })
    }
}

// Parse helper functions for each section type
fn parse_pub_section(input: ParseStream, pub_token: Ident) -> Result<PubSection> {
    let content;
    braced!(content in input);

    let mut fields = Vec::new();

    while !content.is_empty() {
        let name: Ident = content.parse()?;
        let colon: Token![:] = content.parse()?;
        let ty: Type = content.parse()?;
        let arrow: Token![->] = content.parse()?;
        let topic: Expr = content.parse()?;

        fields.push(TopicDef {
            name,
            _colon: colon,
            ty,
            _arrow: arrow,
            topic,
        });

        if content.peek(Token![,]) {
            content.parse::<Token![,]>()?;
        }
    }

    Ok(PubSection {
        _pub_token: pub_token,
        fields,
    })
}

fn parse_sub_section(input: ParseStream, sub_token: Ident) -> Result<SubSection> {
    let content;
    braced!(content in input);

    let mut fields = Vec::new();

    while !content.is_empty() {
        let name: Ident = content.parse()?;
        let colon: Token![:] = content.parse()?;
        let ty: Type = content.parse()?;
        let arrow: Token![->] = content.parse()?;
        let topic: Expr = content.parse()?;

        fields.push(TopicDef {
            name,
            _colon: colon,
            ty,
            _arrow: arrow,
            topic,
        });

        if content.peek(Token![,]) {
            content.parse::<Token![,]>()?;
        }
    }

    Ok(SubSection {
        _sub_token: sub_token,
        fields,
    })
}

fn parse_data_section(input: ParseStream, data_token: Ident) -> Result<DataSection> {
    let content;
    braced!(content in input);

    let mut fields = Vec::new();

    while !content.is_empty() {
        let name: Ident = content.parse()?;
        let colon: Token![:] = content.parse()?;
        let ty: Type = content.parse()?;

        let default = if content.peek(Token![=]) {
            let eq: Token![=] = content.parse()?;
            let expr: Expr = content.parse()?;
            Some((eq, expr))
        } else {
            None
        };

        fields.push(DataField {
            name,
            _colon: colon,
            ty,
            default,
        });

        if content.peek(Token![,]) {
            content.parse::<Token![,]>()?;
        }
    }

    Ok(DataSection {
        _data_token: data_token,
        fields,
    })
}

fn parse_tick_section(input: ParseStream, tick_token: Ident) -> Result<TickSection> {
    // Check for optional (ctx) argument - parsed for backwards compatibility but not used
    let _ctx_arg = if input.peek(syn::token::Paren) {
        let args;
        parenthesized!(args in input);
        Some(args.parse::<Ident>()?)
    } else {
        None
    };

    let body: Block = input.parse()?;

    Ok(TickSection {
        _tick_token: tick_token,
        _ctx_arg,
        body,
    })
}

fn parse_init_section(input: ParseStream, init_token: Ident) -> Result<InitSection> {
    let ctx_arg = if input.peek(syn::token::Paren) {
        let args;
        parenthesized!(args in input);
        Some(args.parse::<Ident>()?)
    } else {
        None
    };

    let body: Block = input.parse()?;

    Ok(InitSection {
        _init_token: init_token,
        ctx_arg,
        body,
    })
}

fn parse_shutdown_section(input: ParseStream, shutdown_token: Ident) -> Result<ShutdownSection> {
    let ctx_arg = if input.peek(syn::token::Paren) {
        let args;
        parenthesized!(args in input);
        Some(args.parse::<Ident>()?)
    } else {
        None
    };

    let body: Block = input.parse()?;

    Ok(ShutdownSection {
        _shutdown_token: shutdown_token,
        ctx_arg,
        body,
    })
}

fn parse_impl_section(input: ParseStream) -> Result<ImplSection> {
    let impl_token: Token![impl] = input.parse()?;
    let body: Block = input.parse()?;

    Ok(ImplSection {
        _impl_token: impl_token,
        body,
    })
}

fn parse_rate_section(input: ParseStream, rate_token: Ident) -> Result<RateSection> {
    // Parse rate value: rate 60.0 or rate 100
    let rate_hz: Expr = input.parse()?;

    Ok(RateSection {
        _rate_token: rate_token,
        rate_hz,
    })
}

fn parse_name_section(input: ParseStream, name_token: Ident) -> Result<NameSection> {
    // Parse: name: "CustomName"
    let colon: Token![:] = input.parse()?;
    let name_value: syn::LitStr = input.parse()?;

    // Optional trailing comma
    if input.peek(Token![,]) {
        input.parse::<Token![,]>()?;
    }

    Ok(NameSection {
        _name_token: name_token,
        _colon: colon,
        name_value,
    })
}

pub fn impl_node_macro(input: TokenStream) -> TokenStream {
    let node_def = parse_macro_input!(input as NodeDef);

    let struct_name = &node_def.name;

    // Use explicit name if provided, otherwise auto-generate from struct name
    let node_name_str = if let Some(ref name_section) = node_def.name_section {
        name_section.name_value.value()
    } else {
        to_snake_case(&struct_name.to_string())
    };

    // Generate struct fields
    let mut struct_fields = Vec::new();

    // Add publisher fields
    if let Some(ref pub_section) = node_def.pub_section {
        for topic in &pub_section.fields {
            let name = &topic.name;
            let ty = &topic.ty;
            struct_fields.push(quote! {
                #name: horus_core::communication::Topic<#ty>
            });
        }
    }

    // Add subscriber fields
    if let Some(ref sub_section) = node_def.sub_section {
        for topic in &sub_section.fields {
            let name = &topic.name;
            let ty = &topic.ty;
            struct_fields.push(quote! {
                #name: horus_core::communication::Topic<#ty>
            });
        }
    }

    // Add data fields
    if let Some(ref data_section) = node_def.data_section {
        for field in &data_section.fields {
            let name = &field.name;
            let ty = &field.ty;
            struct_fields.push(quote! {
                #name: #ty
            });
        }
    }

    // Generate constructor field initializations
    let mut constructor_fields = Vec::new();

    // Initialize publishers
    if let Some(ref pub_section) = node_def.pub_section {
        for topic in &pub_section.fields {
            let name = &topic.name;
            let topic_expr = &topic.topic;
            constructor_fields.push(quote! {
                #name: horus_core::communication::Topic::new(#topic_expr)
                    .expect(&format!("Failed to create publisher '{}'", stringify!(#name)))
            });
        }
    }

    // Initialize subscribers
    if let Some(ref sub_section) = node_def.sub_section {
        for topic in &sub_section.fields {
            let name = &topic.name;
            let topic_expr = &topic.topic;
            constructor_fields.push(quote! {
                #name: horus_core::communication::Topic::new(#topic_expr)
                    .expect(&format!("Failed to create subscriber '{}'", stringify!(#name)))
            });
        }
    }

    // Initialize data fields
    if let Some(ref data_section) = node_def.data_section {
        for field in &data_section.fields {
            let name = &field.name;
            if let Some((_, ref default_expr)) = field.default {
                constructor_fields.push(quote! {
                    #name: #default_expr
                });
            } else {
                constructor_fields.push(quote! {
                    #name: Default::default()
                });
            }
        }
    }

    // Generate tick implementation
    let tick_body = &node_def.tick_section.body;
    let tick_impl = quote! {
        fn tick(&mut self) {
            #tick_body
        }
    };

    // Generate optional init implementation
    let init_impl = if let Some(ref init_section) = node_def.init_section {
        let init_body = &init_section.body;
        if init_section.ctx_arg.is_some() {
            quote! {
                fn init(&mut self, ctx: &mut horus_core::core::NodeInfo) -> horus_core::error::HorusResult<()> {
                    #init_body
                }
            }
        } else {
            quote! {
                fn init(&mut self, _ctx: &mut horus_core::core::NodeInfo) -> horus_core::error::HorusResult<()> {
                    #init_body
                }
            }
        }
    } else {
        quote! {}
    };

    // Generate optional shutdown implementation
    let shutdown_impl = if let Some(ref shutdown_section) = node_def.shutdown_section {
        let shutdown_body = &shutdown_section.body;
        if shutdown_section.ctx_arg.is_some() {
            quote! {
                fn shutdown(&mut self, ctx: &mut horus_core::core::NodeInfo) -> horus_core::error::HorusResult<()> {
                    #shutdown_body
                }
            }
        } else {
            quote! {
                fn shutdown(&mut self, _ctx: &mut horus_core::core::NodeInfo) -> horus_core::error::HorusResult<()> {
                    #shutdown_body
                }
            }
        }
    } else {
        quote! {}
    };

    // Generate additional impl methods
    let impl_methods = if let Some(ref impl_section) = node_def.impl_section {
        let impl_body = &impl_section.body;
        quote! {
            impl #struct_name #impl_body
        }
    } else {
        quote! {}
    };

    // Generate get_publishers() implementation
    let publishers_impl = if let Some(ref pub_section) = node_def.pub_section {
        let publishers: Vec<_> = pub_section
            .fields
            .iter()
            .map(|topic| {
                let topic_expr = &topic.topic;
                let ty = &topic.ty;
                quote! {
                    horus_core::core::node::TopicMetadata {
                        topic_name: #topic_expr.to_string(),
                        type_name: ::std::any::type_name::<#ty>().to_string(),
                    }
                }
            })
            .collect();
        quote! {
            fn get_publishers(&self) -> Vec<horus_core::core::node::TopicMetadata> {
                vec![#(#publishers),*]
            }
        }
    } else {
        quote! {}
    };

    // Generate get_subscribers() implementation
    let subscribers_impl = if let Some(ref sub_section) = node_def.sub_section {
        let subscribers: Vec<_> = sub_section
            .fields
            .iter()
            .map(|topic| {
                let topic_expr = &topic.topic;
                let ty = &topic.ty;
                quote! {
                    horus_core::core::node::TopicMetadata {
                        topic_name: #topic_expr.to_string(),
                        type_name: ::std::any::type_name::<#ty>().to_string(),
                    }
                }
            })
            .collect();
        quote! {
            fn get_subscribers(&self) -> Vec<horus_core::core::node::TopicMetadata> {
                vec![#(#subscribers),*]
            }
        }
    } else {
        quote! {}
    };

    // Generate optional rate_hz implementation
    let rate_impl = if let Some(ref rate_section) = node_def.rate_section {
        let rate_value = &rate_section.rate_hz;
        quote! {
            fn rate_hz(&self) -> Option<f64> {
                Some(#rate_value)
            }
        }
    } else {
        quote! {}
    };

    // Generate the complete output
    let expanded = quote! {
        pub struct #struct_name {
            #(#struct_fields,)*
        }

        impl #struct_name {
            pub fn new() -> Self {
                Self {
                    #(#constructor_fields,)*
                }
            }
        }

        impl horus_core::core::node::Node for #struct_name {
            fn name(&self) -> &'static str {
                #node_name_str
            }

            #tick_impl
            #init_impl
            #shutdown_impl
            #publishers_impl
            #subscribers_impl
            #rate_impl
        }

        impl Default for #struct_name {
            fn default() -> Self {
                Self::new()
            }
        }

        #impl_methods
    };

    TokenStream::from(expanded)
}

// Helper function to convert CamelCase to snake_case
fn to_snake_case(s: &str) -> String {
    let mut result = String::new();
    let mut prev_upper = false;

    for (i, ch) in s.chars().enumerate() {
        if ch.is_uppercase() {
            if i > 0 && !prev_upper {
                result.push('_');
            }
            result.push(ch.to_ascii_lowercase());
            prev_upper = true;
        } else {
            result.push(ch);
            prev_upper = false;
        }
    }

    result
}
