//! One message definition, four artifacts.
//!
//! A message type usable from all three languages has to be written into six
//! places today, each with its own syntax: a `#[repr(C)]` struct plus
//! `impl_pod_message!` in `horus_types`, a `#[pyclass]` wrapper plus a
//! `pod_topic_types!` row in `horus_py`, an `impl_topic_ffi!` and an
//! `impl_pod_topic_c_api!` plus a hand-written header in `horus_cpp`, and a row
//! in `layout_contract_types!`.
//!
//! They do not stay in sync. Counting the registries gives 91 / 75 / 75 / 68 /
//! 62 / 61 / 60, and `layout_contract.rs` exists precisely because they once
//! diverged catastrophically — its own module documentation records
//! `JointCommand` at 928 bytes in Rust against 88 in C++, an 840-byte overrun
//! on every receive.
//!
//! This module is the single source those artifacts are generated from.
//!
//! # Why a separate parser
//!
//! `commands::msg` already scrapes type definitions out of Rust source, and is
//! deliberately tolerant: it silently drops fields it cannot read, which is
//! right for `horus msg list` and wrong here. A generator that quietly omits a
//! field emits a type whose layout disagrees with its own definition. This
//! parser rejects anything it does not fully understand, with a file:line:col.
//!
//! # Why one rendering authority
//!
//! [`canonical`] renders types to text, and every emitter and the layout hash
//! go through it. The alternative — letting the Rust emitter print a type and
//! computing the hash from a separate rendering — produces two different hashes
//! for one type the moment a spelling differs. That is not hypothetical:
//! `stringify!` preserves source spacing, so `[u8;32]` and `[u8; 32]` hash
//! differently today.

pub mod canonical;
pub mod emit_cpp;
pub mod emit_python;
pub mod emit_rust;
pub mod layout;
pub mod parse;

use std::path::PathBuf;

/// Everything declared under a project's `msgs/` directory.
#[derive(Debug, Clone, PartialEq)]
pub struct Package {
    /// The project name, used to name the generated crate and C++ namespace.
    pub name: String,
    pub messages: Vec<MsgDef>,
}

/// One message type.
#[derive(Debug, Clone, PartialEq)]
pub struct MsgDef {
    pub name: String,
    /// `#[topic = "..."]`, emitted as a `TOPIC` constant when present.
    pub topic: Option<String>,
    /// Doc comment, carried through to every artifact.
    pub doc: Vec<String>,
    pub fields: Vec<Field>,
    /// Where it was declared, for diagnostics and for the generated header.
    pub src: PathBuf,
    pub line: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub name: String,
    pub ty: Type,
    pub doc: Vec<String>,
    pub line: usize,
    pub col: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Type {
    Prim(Prim),
    /// `[T; N]`.
    Array(Box<Type>, usize),
    /// Another message, in this package or built in.
    Ref(Resolved),
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
    pub fn from_name(s: &str) -> Option<Self> {
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

    pub fn rust(self) -> &'static str {
        match self {
            Prim::U8 => "u8",
            Prim::U16 => "u16",
            Prim::U32 => "u32",
            Prim::U64 => "u64",
            Prim::I8 => "i8",
            Prim::I16 => "i16",
            Prim::I32 => "i32",
            Prim::I64 => "i64",
            Prim::F32 => "f32",
            Prim::F64 => "f64",
            Prim::Bool => "bool",
        }
    }

    /// The C++ spelling. Fixed-width by name so the two sides cannot disagree
    /// about what `long` means on some target.
    pub fn cpp(self) -> &'static str {
        match self {
            Prim::U8 => "uint8_t",
            Prim::U16 => "uint16_t",
            Prim::U32 => "uint32_t",
            Prim::U64 => "uint64_t",
            Prim::I8 => "int8_t",
            Prim::I16 => "int16_t",
            Prim::I32 => "int32_t",
            Prim::I64 => "int64_t",
            Prim::F32 => "float",
            Prim::F64 => "double",
            // Rust guarantees `bool` is one byte with values 0 and 1; C++
            // `bool` has implementation-defined size, so it is not safe across
            // the ABI. `uint8_t` is, and the accessor below keeps it ergonomic.
            Prim::Bool => "uint8_t",
        }
    }

    /// `size, align` on every target HORUS supports.
    pub fn size_align(self) -> (usize, usize) {
        match self {
            Prim::U8 | Prim::I8 | Prim::Bool => (1, 1),
            Prim::U16 | Prim::I16 => (2, 2),
            Prim::U32 | Prim::I32 | Prim::F32 => (4, 4),
            Prim::U64 | Prim::I64 | Prim::F64 => (8, 8),
        }
    }
}

/// A resolved reference to another message type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Resolved {
    pub short: String,
    /// How to name it in generated Rust, e.g. `horus_types::Vector3`.
    pub rust_path: String,
    /// How to name it in generated C++, e.g. `horus::msg::Vector3`.
    pub cpp_path: String,
    /// False when it is another message in the same `.hmsg` set.
    pub external: bool,
}

/// A parse or resolution failure, with somewhere to look.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Diag {
    pub file: PathBuf,
    pub line: usize,
    pub col: usize,
    pub message: String,
    /// What to do about it. Absent when there is nothing useful to say.
    pub help: Option<String>,
}

impl std::fmt::Display for Diag {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}:{}:{}: {}",
            self.file.display(),
            self.line,
            self.col,
            self.message
        )?;
        if let Some(ref help) = self.help {
            write!(f, "\n  help: {help}")?;
        }
        Ok(())
    }
}

impl Diag {
    pub fn new(file: &std::path::Path, line: usize, col: usize, message: impl Into<String>) -> Self {
        Self {
            file: file.to_path_buf(),
            line,
            col,
            message: message.into(),
            help: None,
        }
    }

    pub fn with_help(mut self, help: impl Into<String>) -> Self {
        self.help = Some(help.into());
        self
    }
}
