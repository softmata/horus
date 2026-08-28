//! Decode a POD message payload from its declared field layout.
//!
//! `horus topic echo` used a hand-written `match` arm per message type. Nineteen
//! of the ninety-two types HORUS ships had one; everything else printed a hex
//! dump. Among the missing was `Twist` — the type in the project template
//! `horus new` generates — so a developer running the starter project and
//! echoing their own velocity command got 56 bytes of hex.
//!
//! The layout is not a secret: `horus msg info Twist` already prints
//! `linear: [f64; 3]`, parsed out of `horus_types/src` at runtime by
//! [`super::msg::discover_messages`]. Every message struct is `#[repr(C)]`, so
//! declared field types plus C layout rules give exact offsets.
//!
//! # Being wrong is worse than being unreadable
//!
//! A misplaced offset prints a plausible number that is not the value on the
//! wire, and an operator has no way to tell. So the computed struct size must
//! equal the payload length exactly; anything else falls back to the hex dump.
//! That check catches a bad layout computation, an unknown field type, and a
//! payload from a different version of the type.

use std::collections::HashMap;

use super::msg::FieldInfo;

/// A field type as declared in the message source.
#[derive(Debug, Clone, PartialEq)]
pub enum PodType {
    Prim(Prim),
    /// `[T; N]`
    Array(Box<PodType>, usize),
    /// Another message struct, resolved through the registry.
    Struct(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prim {
    U8,
    U16,
    U32,
    U64,
    I8,
    I16,
    I32,
    I64,
    F32,
    F64,
    Bool,
}

impl Prim {
    fn width(self) -> usize {
        match self {
            Prim::U8 | Prim::I8 | Prim::Bool => 1,
            Prim::U16 | Prim::I16 => 2,
            Prim::U32 | Prim::I32 | Prim::F32 => 4,
            Prim::U64 | Prim::I64 | Prim::F64 => 8,
        }
    }

    fn parse(s: &str) -> Option<Prim> {
        Some(match s {
            "u8" => Prim::U8,
            "u16" => Prim::U16,
            "u32" => Prim::U32,
            "u64" => Prim::U64,
            "i8" => Prim::I8,
            "i16" => Prim::I16,
            "i32" => Prim::I32,
            "i64" => Prim::I64,
            "f32" => Prim::F32,
            "f64" => Prim::F64,
            "bool" => Prim::Bool,
            _ => return None,
        })
    }

    fn read(self, b: &[u8]) -> Option<String> {
        let w = self.width();
        if b.len() < w {
            return None;
        }
        Some(match self {
            Prim::U8 => b[0].to_string(),
            Prim::I8 => (b[0] as i8).to_string(),
            Prim::Bool => (b[0] != 0).to_string(),
            Prim::U16 => u16::from_le_bytes(b[..2].try_into().ok()?).to_string(),
            Prim::I16 => i16::from_le_bytes(b[..2].try_into().ok()?).to_string(),
            Prim::U32 => u32::from_le_bytes(b[..4].try_into().ok()?).to_string(),
            Prim::I32 => i32::from_le_bytes(b[..4].try_into().ok()?).to_string(),
            Prim::F32 => fmt_float(f32::from_le_bytes(b[..4].try_into().ok()?) as f64),
            Prim::U64 => u64::from_le_bytes(b[..8].try_into().ok()?).to_string(),
            Prim::I64 => i64::from_le_bytes(b[..8].try_into().ok()?).to_string(),
            Prim::F64 => fmt_float(f64::from_le_bytes(b[..8].try_into().ok()?)),
        })
    }
}

/// Trim trailing zeros so `1.0` reads as `1` and `0.125` keeps its digits —
/// a robot's telemetry is easier to scan without three trailing zeros on
/// every field.
fn fmt_float(v: f64) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    if v == v.trunc() && v.abs() < 1e15 {
        return format!("{}", v as i64);
    }
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0');
    s.trim_end_matches('.').to_string()
}

/// Parse a declared type: `f64`, `[u8; 32]`, `[f64; 3]`, `Quaternion`.
pub fn parse_type(decl: &str) -> Option<PodType> {
    let s = decl.trim();
    if let Some(inner) = s.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
        let (elem, count) = inner.rsplit_once(';')?;
        let count: usize = count.trim().parse().ok()?;
        return Some(PodType::Array(Box::new(parse_type(elem)?), count));
    }
    if let Some(p) = Prim::parse(s) {
        return Some(PodType::Prim(p));
    }
    // A struct name, resolved later. Reject anything that is not a plain
    // identifier — references, generics and `Option<T>` are not POD and must
    // not be guessed at.
    if !s.is_empty()
        && s.chars().next().is_some_and(|c| c.is_ascii_uppercase())
        && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
    {
        return Some(PodType::Struct(s.to_string()));
    }
    None
}

/// Size and alignment, by C layout rules.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layout {
    pub size: usize,
    pub align: usize,
}

/// Guards against a cycle in the type graph — a struct that (transitively)
/// contains itself has no finite layout and would otherwise recurse forever.
const MAX_DEPTH: usize = 8;

pub type Registry = HashMap<String, Vec<FieldInfo>>;

pub fn layout_of(ty: &PodType, reg: &Registry, depth: usize) -> Option<Layout> {
    if depth > MAX_DEPTH {
        return None;
    }
    Some(match ty {
        PodType::Prim(p) => Layout {
            size: p.width(),
            align: p.width(),
        },
        PodType::Array(elem, n) => {
            let inner = layout_of(elem, reg, depth + 1)?;
            Layout {
                size: inner.size.checked_mul(*n)?,
                align: inner.align,
            }
        }
        PodType::Struct(name) => struct_layout(name, reg, depth + 1)?,
    })
}

fn struct_layout(name: &str, reg: &Registry, depth: usize) -> Option<Layout> {
    let fields = reg.get(name)?;
    let mut offset = 0usize;
    let mut align = 1usize;
    for f in fields {
        let ty = parse_type(&f.field_type)?;
        let l = layout_of(&ty, reg, depth)?;
        if l.align == 0 {
            return None;
        }
        offset = round_up(offset, l.align)?;
        offset = offset.checked_add(l.size)?;
        align = align.max(l.align);
    }
    Some(Layout {
        size: round_up(offset, align)?,
        align,
    })
}

fn round_up(v: usize, align: usize) -> Option<usize> {
    if align == 0 {
        return None;
    }
    let rem = v % align;
    if rem == 0 {
        Some(v)
    } else {
        v.checked_add(align - rem)
    }
}

/// Render one value of `ty` from `data`.
fn render(ty: &PodType, data: &[u8], reg: &Registry, depth: usize) -> Option<String> {
    if depth > MAX_DEPTH {
        return None;
    }
    match ty {
        PodType::Prim(p) => p.read(data),
        PodType::Array(elem, n) => {
            // A `[u8; N]` is nearly always a fixed-width name or label, which is
            // what an operator wants to read — not 32 integers.
            if matches!(**elem, PodType::Prim(Prim::U8)) {
                if let Some(text) = as_fixed_string(&data[..(*n).min(data.len())]) {
                    return Some(text);
                }
            }
            let inner = layout_of(elem, reg, depth + 1)?;
            let shown = (*n).min(8);
            let mut parts = Vec::with_capacity(shown);
            for i in 0..shown {
                let off = i.checked_mul(inner.size)?;
                parts.push(render(elem, data.get(off..)?, reg, depth + 1)?);
            }
            if *n > shown {
                parts.push(format!("… +{}", *n - shown));
            }
            Some(format!("[{}]", parts.join(", ")))
        }
        PodType::Struct(name) => {
            let body = render_struct(name, data, reg, depth + 1)?;
            Some(format!("{{ {body} }}"))
        }
    }
}

/// A NUL-padded ASCII string, or `None` if the bytes are not one.
fn as_fixed_string(bytes: &[u8]) -> Option<String> {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    let text = &bytes[..end];
    if text.is_empty() {
        return Some(String::from("\"\""));
    }
    if !text.iter().all(|&b| (0x20..0x7f).contains(&b)) {
        return None;
    }
    // Everything after the first NUL must be padding, or this is binary data
    // that happens to start with printable bytes.
    if !bytes[end..].iter().all(|&b| b == 0) {
        return None;
    }
    Some(format!("{:?}", String::from_utf8_lossy(text)))
}

fn render_struct(name: &str, data: &[u8], reg: &Registry, depth: usize) -> Option<String> {
    let fields = reg.get(name)?;
    let mut offset = 0usize;
    let mut out = Vec::with_capacity(fields.len());
    for f in fields {
        let ty = parse_type(&f.field_type)?;
        let l = layout_of(&ty, reg, depth)?;
        offset = round_up(offset, l.align)?;
        let value = render(&ty, data.get(offset..)?, reg, depth)?;
        out.push(format!("{}: {}", f.name, value));
        offset = offset.checked_add(l.size)?;
    }
    Some(out.join(", "))
}

/// Decode `data` as `type_name`, or `None` if it cannot be done exactly.
///
/// Returns `None` unless the computed size matches the payload exactly — see
/// the module docs on why a plausible wrong answer is worse than a hex dump.
pub fn decode(type_name: &str, data: &[u8], reg: &Registry) -> Option<String> {
    let layout = struct_layout(type_name, reg, 0)?;
    if layout.size != data.len() {
        return None;
    }
    render_struct(type_name, data, reg, 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field(name: &str, ty: &str) -> FieldInfo {
        FieldInfo {
            name: name.to_string(),
            field_type: ty.to_string(),
            doc: String::new(),
        }
    }

    fn registry() -> Registry {
        let mut r = Registry::new();
        // The real definition, from horus_types/src/math.rs.
        r.insert(
            "Twist".into(),
            vec![
                field("linear", "[f64; 3]"),
                field("angular", "[f64; 3]"),
                field("timestamp_ns", "u64"),
            ],
        );
        // Mixed widths, to exercise C padding.
        r.insert(
            "Mixed".into(),
            vec![field("flag", "u8"), field("value", "u64")],
        );
        r.insert(
            "Named".into(),
            vec![field("name", "[u8; 8]"), field("id", "u32")],
        );
        r.insert(
            "Nested".into(),
            vec![field("inner", "Mixed"), field("tag", "u16")],
        );
        r
    }

    #[test]
    fn twist_is_decoded_field_by_field() {
        // The type in the `horus new` template. It had no hand-written arm, so
        // `horus topic echo` on the starter project printed 56 bytes of hex.
        let mut data = Vec::new();
        for v in [1.5f64, 0.0, -2.25, 0.0, 0.0, 0.75] {
            data.extend_from_slice(&v.to_le_bytes());
        }
        data.extend_from_slice(&42u64.to_le_bytes());
        assert_eq!(data.len(), 56, "Twist is 56 bytes");

        let out = decode("Twist", &data, &registry()).expect("Twist must decode");
        assert_eq!(
            out,
            "linear: [1.5, 0, -2.25], angular: [0, 0, 0.75], timestamp_ns: 42"
        );
    }

    #[test]
    fn c_padding_is_honoured() {
        // `{ u8, u64 }` is 16 bytes with the u64 at offset 8, not 9. Getting
        // this wrong prints a plausible number that is not the value on the
        // wire — the failure mode the size check exists to prevent.
        let l = struct_layout("Mixed", &registry(), 0).unwrap();
        assert_eq!(l, Layout { size: 16, align: 8 });

        let mut data = vec![0u8; 16];
        data[0] = 1;
        data[8..16].copy_from_slice(&7u64.to_le_bytes());
        assert_eq!(
            decode("Mixed", &data, &registry()).unwrap(),
            "flag: 1, value: 7"
        );
    }

    #[test]
    fn a_size_mismatch_declines_rather_than_guessing() {
        // A payload from a different version of the type, or a layout this code
        // computed wrongly. Either way the caller falls back to hex.
        let reg = registry();
        assert!(decode("Twist", &[0u8; 55], &reg).is_none());
        assert!(decode("Twist", &[0u8; 57], &reg).is_none());
        assert!(decode("NotAMessage", &[0u8; 8], &reg).is_none());
    }

    #[test]
    fn fixed_byte_arrays_read_as_text() {
        // `[u8; N]` in these messages is a name or label. Thirty-two integers
        // is not what an operator is looking for.
        let mut data = vec![0u8; 12];
        data[..5].copy_from_slice(b"wheel");
        data[8..12].copy_from_slice(&9u32.to_le_bytes());
        assert_eq!(
            decode("Named", &data, &registry()).unwrap(),
            "name: \"wheel\", id: 9"
        );
    }

    #[test]
    fn binary_byte_arrays_stay_numeric() {
        // Not every `[u8; N]` is text; one that is not must not be mangled into
        // a string.
        let mut data = vec![0u8; 12];
        data[..8].copy_from_slice(&[0xff, 0x01, 0x80, 0x00, 0x7f, 0x03, 0x00, 0x00]);
        let out = decode("Named", &data, &registry()).unwrap();
        assert!(
            out.starts_with("name: ["),
            "expected numeric array, got {out}"
        );
    }

    #[test]
    fn nested_structs_resolve() {
        let l = struct_layout("Nested", &registry(), 0).unwrap();
        // Mixed is 16/align 8; u16 at 16; padded to 24.
        assert_eq!(l, Layout { size: 24, align: 8 });
        let mut data = vec![0u8; 24];
        data[0] = 3;
        data[8..16].copy_from_slice(&11u64.to_le_bytes());
        data[16..18].copy_from_slice(&5u16.to_le_bytes());
        assert_eq!(
            decode("Nested", &data, &registry()).unwrap(),
            "inner: { flag: 3, value: 11 }, tag: 5"
        );
    }

    #[test]
    fn a_recursive_type_is_declined_not_hung() {
        let mut r = Registry::new();
        r.insert("Loop".into(), vec![field("me", "Loop")]);
        assert!(struct_layout("Loop", &r, 0).is_none());
    }

    #[test]
    fn non_pod_declarations_are_rejected() {
        assert!(parse_type("Option<u8>").is_none());
        assert!(parse_type("&str").is_none());
        assert!(parse_type("Vec<f32>").is_none());
        assert_eq!(parse_type("f64"), Some(PodType::Prim(Prim::F64)));
        assert_eq!(
            parse_type("[f64; 3]"),
            Some(PodType::Array(Box::new(PodType::Prim(Prim::F64)), 3))
        );
    }
}

/// The layout computation, checked against the real message definitions.
///
/// These tests read `horus_types/src` through the same discovery the CLI uses,
/// so they fail if the layout rules here drift from the structs they describe.
#[cfg(test)]
mod real_definitions {
    use super::*;

    /// Sizes documented in the hand-written decoder arms in `topic.rs`, each
    /// measured against the real struct when that arm was written. They are an
    /// independent check on the C layout implemented here: agreement means two
    /// people arrived at the same number by different routes.
    const KNOWN_SIZES: &[(&str, usize)] = &[
        ("CmdVel", 16),
        ("Pose2D", 32),
        ("Imu", 304),
        // 1480, not the 1476 the arm's comment claimed: a `#[repr(C)]` struct
        // ending in a u64 cannot be 1476 long, and `size_of` on the real
        // definition reports 1480 with `timestamp_ns` at 1472. This table found
        // that discrepancy, which is what it is for.
        ("LaserScan", 1480),
        ("Odometry", 736),
        ("JointState", 912),
        ("NavSatFix", 128),
        ("BatteryState", 104),
        ("Temperature", 56),
        // Not documented in an arm, but the payload `horus topic echo` reports
        // for the project template's own type.
        ("Twist", 56),
    ];

    fn real_registry() -> Option<Registry> {
        let msgs = crate::commands::msg::discover_messages().ok()?;
        if msgs.is_empty() {
            return None;
        }
        Some(msgs.into_iter().map(|m| (m.name, m.fields)).collect())
    }

    #[test]
    fn computed_layouts_match_the_sizes_measured_by_hand() {
        let Some(reg) = real_registry() else {
            eprintln!("skipped: horus_types sources not found");
            return;
        };
        let mut wrong = Vec::new();
        for (name, expected) in KNOWN_SIZES {
            match struct_layout(name, &reg, 0) {
                Some(l) if l.size == *expected => {}
                Some(l) => wrong.push(format!("{name}: computed {} != {expected}", l.size)),
                None => wrong.push(format!("{name}: no layout computed")),
            }
        }
        assert!(
            wrong.is_empty(),
            "the C layout rules here disagree with the sizes the hand-written \
             decoder arms were built against:\n  {}",
            wrong.join("\n  ")
        );
    }

    #[test]
    fn most_shipped_messages_get_a_layout() {
        // Not a coverage target — a canary. If a change here starts declining
        // types it used to handle, `horus topic echo` quietly returns to
        // printing hex, which is exactly the regression this module exists to
        // fix and is invisible without a count.
        let Some(reg) = real_registry() else {
            eprintln!("skipped: horus_types sources not found");
            return;
        };
        let total = reg.len();
        let decoded = reg
            .keys()
            .filter(|n| struct_layout(n, &reg, 0).is_some())
            .count();
        assert!(
            decoded * 10 >= total * 8,
            "only {decoded} of {total} message types have a computable layout; \
             the rest fall back to a hex dump"
        );
    }
}
