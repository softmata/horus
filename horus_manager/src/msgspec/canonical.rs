//! The single rendering authority.
//!
//! Every emitter and the layout hash render types through this module. That is
//! the whole point: if the Rust emitter printed a type one way and the hash
//! were computed from a separate rendering, the two would disagree the moment a
//! spelling differed — and that is not a hypothetical failure. `message!`
//! computes its hash with `stringify!`, which reproduces source spacing, so
//! `[u8;32]` and `[u8; 32]` produce different hashes for the same type today.
//!
//! The generated Rust does not use `stringify!`. It embeds the canonical string
//! as a literal and hashes that, so the text that was hashed and the text that
//! was emitted are the same bytes by construction.

use super::{MsgDef, Type};

/// How a type is spelled in generated Rust, and in the canonical form.
///
/// One space after the `;` in an array, always. The choice does not matter;
/// having exactly one does.
pub fn render_rust(t: &Type) -> String {
    match t {
        Type::Prim(p) => p.rust().to_string(),
        Type::Array(elem, n) => format!("[{}; {}]", render_rust(elem), n),
        Type::Ref(r) => r.rust_path.clone(),
    }
}

/// How a type is spelled in generated C++.
///
/// Arrays become `std::array<T, N>`, which is a POD with the same layout as
/// `T[N]` and, unlike a raw array, can be returned and assigned.
pub fn render_cpp(t: &Type) -> String {
    match t {
        Type::Prim(p) => p.cpp().to_string(),
        Type::Array(elem, n) => format!("std::array<{}, {}>", render_cpp(elem), n),
        Type::Ref(r) => r.cpp_path.clone(),
    }
}

/// The string a message's layout hash is computed over.
///
/// `Name|field:type|field:type`. Deliberately identical in shape to what
/// `commands::msg` computes for built-in types, so `horus msg hash` prints the
/// same number for a generated type as for a shipped one.
pub fn canonical_form(m: &MsgDef) -> String {
    let mut s = m.name.clone();
    for f in &m.fields {
        s.push('|');
        s.push_str(&f.name);
        s.push(':');
        s.push_str(&render_rust(&f.ty));
    }
    s
}

/// FNV-1a over the canonical form.
///
/// The same algorithm as `horus_core::communication::topic::const_fnv1a` and
/// `commands::msg::fnv1a`. One algorithm, three call sites, because the value
/// has to match what the runtime computes or the layout check it feeds is
/// worse than nothing.
pub fn fnv1a(bytes: &[u8]) -> u32 {
    let mut hash: u32 = 2166136261;
    for &byte in bytes {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(16777619);
    }
    hash
}

pub fn layout_hash(m: &MsgDef) -> u32 {
    fnv1a(canonical_form(m).as_bytes())
}
