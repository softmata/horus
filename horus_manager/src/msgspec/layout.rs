//! `repr(C)` size and alignment, computed rather than assumed.
//!
//! Used for diagnostics — the byte size reported to the user, and the padding
//! warning — and to lay out the Python `ctypes` mirror.
//!
//! Deliberately *not* the source of the C++ `static_assert`s. Those are emitted
//! against `offsetof` in the generated header and checked by the C++ compiler,
//! so a mistake in this function cannot produce a contract that is silently
//! wrong. It can only produce a wrong number in a message.

use std::collections::HashMap;

use super::{MsgDef, Type};

/// Sizes and alignments of the message types already known.
pub type Env = HashMap<String, (usize, usize)>;

/// No table of built-in layouts.
///
/// The first version of this module carried one, with `Twist` written down as
/// 48 bytes. `Twist` is `[f64; 3]`, `[f64; 3]`, `u64` — 56. A hardcoded layout
/// that is wrong produces a header whose `static_assert` passes against the
/// wrong number, which is precisely the failure `horus_cpp`'s layout contract
/// exists to catch: its own documentation records `JointCommand` at 928 bytes
/// in Rust against 88 in C++.
///
/// So v1 does not accept references to built-in types at all. A `.hmsg` may
/// use primitives, arrays, and other messages declared alongside it — every
/// one of which this module can size exactly. Referencing `Vector3` is
/// rejected with a diagnostic saying to write out the three fields, which is
/// correct today and can be relaxed once the sizes come from the compiler
/// rather than from a table.
pub fn builtin_layouts() -> Env {
    Env::new()
}

/// `(size, align)` of a type, or the name of a reference that could not be
/// resolved.
pub fn size_align(t: &Type, env: &Env) -> Result<(usize, usize), String> {
    Ok(match t {
        Type::Prim(p) => p.size_align(),
        Type::Array(elem, n) => {
            let (s, a) = size_align(elem, env)?;
            (s * n, a)
        }
        Type::Ref(r) => *env.get(&r.short).ok_or_else(|| r.short.clone())?,
    })
}

/// Where each field lands, and how big the message is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layout {
    /// One entry per field: `(offset, size, align)`.
    pub fields: Vec<(usize, usize, usize)>,
    pub size: usize,
    pub align: usize,
    /// Bytes that exist only to satisfy alignment. Worth telling the user
    /// about: reordering fields largest-first usually removes them, and on a
    /// message published at 1 kHz they are pure bandwidth.
    pub padding: usize,
}

/// The `repr(C)` algorithm: each field is placed at the next offset satisfying
/// its own alignment; the struct's alignment is the maximum of its fields'; its
/// size is rounded up to that.
pub fn compute(m: &MsgDef, env: &Env) -> Result<Layout, String> {
    let mut cursor = 0usize;
    let mut max_align = 1usize;
    let mut fields = Vec::with_capacity(m.fields.len());
    let mut used = 0usize;

    for f in &m.fields {
        let (size, align) = size_align(&f.ty, env)?;
        let offset = round_up(cursor, align);
        fields.push((offset, size, align));
        cursor = offset + size;
        used += size;
        max_align = max_align.max(align);
    }

    let size = round_up(cursor, max_align);
    Ok(Layout {
        fields,
        size,
        align: max_align,
        padding: size - used,
    })
}

fn round_up(n: usize, align: usize) -> usize {
    debug_assert!(align.is_power_of_two());
    (n + align - 1) & !(align - 1)
}
