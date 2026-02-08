#![allow(clippy::should_implement_trait)]
//! Tensor data types with DLPack mapping
//!
//! Provides enumeration of supported data types and conversion to/from DLPack codes.

use std::fmt;

/// Data type for tensor elements
///
/// Matches common ML framework dtypes for seamless interop.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum TensorDtype {
    /// 32-bit floating point (default for most ML)
    #[default]
    F32 = 0,
    /// 64-bit floating point
    F64 = 1,
    /// 16-bit floating point (half precision)
    F16 = 2,
    /// Brain floating point (bfloat16)
    BF16 = 3,
    /// Signed 8-bit integer
    I8 = 4,
    /// Signed 16-bit integer
    I16 = 5,
    /// Signed 32-bit integer
    I32 = 6,
    /// Signed 64-bit integer
    I64 = 7,
    /// Unsigned 8-bit integer (common for images)
    U8 = 8,
    /// Unsigned 16-bit integer
    U16 = 9,
    /// Unsigned 32-bit integer
    U32 = 10,
    /// Unsigned 64-bit integer
    U64 = 11,
    /// Boolean (stored as u8)
    Bool = 12,
}

/// DLPack type codes (standard specification constants)
#[allow(dead_code)]
pub mod dlpack_codes {
    /// Integer type
    pub const DLPACK_INT: u8 = 0;
    /// Unsigned integer type
    pub const DLPACK_UINT: u8 = 1;
    /// Float type
    pub const DLPACK_FLOAT: u8 = 2;
    /// Opaque handle type
    pub const DLPACK_OPAQUE_HANDLE: u8 = 3;
    /// BFloat type
    pub const DLPACK_BFLOAT: u8 = 4;
    /// Complex type
    pub const DLPACK_COMPLEX: u8 = 5;
    /// Boolean type
    pub const DLPACK_BOOL: u8 = 6;
}

impl TensorDtype {
    /// Get the size in bytes of a single element
    #[inline]
    pub const fn size_bytes(&self) -> usize {
        match self {
            TensorDtype::F32 => 4,
            TensorDtype::F64 => 8,
            TensorDtype::F16 => 2,
            TensorDtype::BF16 => 2,
            TensorDtype::I8 => 1,
            TensorDtype::I16 => 2,
            TensorDtype::I32 => 4,
            TensorDtype::I64 => 8,
            TensorDtype::U8 => 1,
            TensorDtype::U16 => 2,
            TensorDtype::U32 => 4,
            TensorDtype::U64 => 8,
            TensorDtype::Bool => 1,
        }
    }

    /// Get numpy dtype string (for __array_interface__)
    pub const fn numpy_typestr(&self) -> &'static str {
        match self {
            TensorDtype::F32 => "<f4",
            TensorDtype::F64 => "<f8",
            TensorDtype::F16 => "<f2",
            TensorDtype::BF16 => "<V2", // bfloat16 not directly supported
            TensorDtype::I8 => "|i1",
            TensorDtype::I16 => "<i2",
            TensorDtype::I32 => "<i4",
            TensorDtype::I64 => "<i8",
            TensorDtype::U8 => "|u1",
            TensorDtype::U16 => "<u2",
            TensorDtype::U32 => "<u4",
            TensorDtype::U64 => "<u8",
            TensorDtype::Bool => "|b1",
        }
    }

    /// Convert to DLPack DLDataType (code, bits, lanes)
    ///
    /// Returns (type_code, bits, lanes)
    pub const fn to_dlpack(&self) -> (u8, u8, u16) {
        use dlpack_codes::*;
        match self {
            TensorDtype::F32 => (DLPACK_FLOAT, 32, 1),
            TensorDtype::F64 => (DLPACK_FLOAT, 64, 1),
            TensorDtype::F16 => (DLPACK_FLOAT, 16, 1),
            TensorDtype::BF16 => (DLPACK_BFLOAT, 16, 1),
            TensorDtype::I8 => (DLPACK_INT, 8, 1),
            TensorDtype::I16 => (DLPACK_INT, 16, 1),
            TensorDtype::I32 => (DLPACK_INT, 32, 1),
            TensorDtype::I64 => (DLPACK_INT, 64, 1),
            TensorDtype::U8 => (DLPACK_UINT, 8, 1),
            TensorDtype::U16 => (DLPACK_UINT, 16, 1),
            TensorDtype::U32 => (DLPACK_UINT, 32, 1),
            TensorDtype::U64 => (DLPACK_UINT, 64, 1),
            TensorDtype::Bool => (DLPACK_BOOL, 8, 1),
        }
    }

    /// Create from DLPack DLDataType
    ///
    /// Takes (type_code, bits, lanes) and returns the corresponding dtype.
    pub const fn from_dlpack(code: u8, bits: u8, lanes: u16) -> Option<Self> {
        use dlpack_codes::*;

        if lanes != 1 {
            return None; // We don't support vectorized types
        }

        match (code, bits) {
            (DLPACK_FLOAT, 16) => Some(TensorDtype::F16),
            (DLPACK_FLOAT, 32) => Some(TensorDtype::F32),
            (DLPACK_FLOAT, 64) => Some(TensorDtype::F64),
            (DLPACK_BFLOAT, 16) => Some(TensorDtype::BF16),
            (DLPACK_INT, 8) => Some(TensorDtype::I8),
            (DLPACK_INT, 16) => Some(TensorDtype::I16),
            (DLPACK_INT, 32) => Some(TensorDtype::I32),
            (DLPACK_INT, 64) => Some(TensorDtype::I64),
            (DLPACK_UINT, 8) => Some(TensorDtype::U8),
            (DLPACK_UINT, 16) => Some(TensorDtype::U16),
            (DLPACK_UINT, 32) => Some(TensorDtype::U32),
            (DLPACK_UINT, 64) => Some(TensorDtype::U64),
            (DLPACK_BOOL, 8) => Some(TensorDtype::Bool),
            _ => None,
        }
    }

    /// Parse from string (e.g., "float32", "f32", "uint8", "u8")
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "float32" | "f32" | "float" => Some(TensorDtype::F32),
            "float64" | "f64" | "double" => Some(TensorDtype::F64),
            "float16" | "f16" | "half" => Some(TensorDtype::F16),
            "bfloat16" | "bf16" => Some(TensorDtype::BF16),
            "int8" | "i8" => Some(TensorDtype::I8),
            "int16" | "i16" => Some(TensorDtype::I16),
            "int32" | "i32" | "int" => Some(TensorDtype::I32),
            "int64" | "i64" | "long" => Some(TensorDtype::I64),
            "uint8" | "u8" | "byte" => Some(TensorDtype::U8),
            "uint16" | "u16" => Some(TensorDtype::U16),
            "uint32" | "u32" => Some(TensorDtype::U32),
            "uint64" | "u64" => Some(TensorDtype::U64),
            "bool" | "boolean" => Some(TensorDtype::Bool),
            _ => None,
        }
    }

    /// Check if this is a floating point type
    #[inline]
    pub const fn is_float(&self) -> bool {
        matches!(
            self,
            TensorDtype::F16 | TensorDtype::F32 | TensorDtype::F64 | TensorDtype::BF16
        )
    }

    /// Check if this is a signed integer type
    #[inline]
    pub const fn is_signed_int(&self) -> bool {
        matches!(
            self,
            TensorDtype::I8 | TensorDtype::I16 | TensorDtype::I32 | TensorDtype::I64
        )
    }

    /// Check if this is an unsigned integer type
    #[inline]
    pub const fn is_unsigned_int(&self) -> bool {
        matches!(
            self,
            TensorDtype::U8 | TensorDtype::U16 | TensorDtype::U32 | TensorDtype::U64
        )
    }
}

impl fmt::Display for TensorDtype {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            TensorDtype::F32 => "float32",
            TensorDtype::F64 => "float64",
            TensorDtype::F16 => "float16",
            TensorDtype::BF16 => "bfloat16",
            TensorDtype::I8 => "int8",
            TensorDtype::I16 => "int16",
            TensorDtype::I32 => "int32",
            TensorDtype::I64 => "int64",
            TensorDtype::U8 => "uint8",
            TensorDtype::U16 => "uint16",
            TensorDtype::U32 => "uint32",
            TensorDtype::U64 => "uint64",
            TensorDtype::Bool => "bool",
        };
        write!(f, "{}", name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dtype_sizes() {
        assert_eq!(TensorDtype::F32.size_bytes(), 4);
        assert_eq!(TensorDtype::F64.size_bytes(), 8);
        assert_eq!(TensorDtype::F16.size_bytes(), 2);
        assert_eq!(TensorDtype::BF16.size_bytes(), 2);
        assert_eq!(TensorDtype::U8.size_bytes(), 1);
        assert_eq!(TensorDtype::I64.size_bytes(), 8);
    }

    #[test]
    fn test_dlpack_roundtrip() {
        for dtype in [
            TensorDtype::F32,
            TensorDtype::F64,
            TensorDtype::F16,
            TensorDtype::BF16,
            TensorDtype::I8,
            TensorDtype::I16,
            TensorDtype::I32,
            TensorDtype::I64,
            TensorDtype::U8,
            TensorDtype::U16,
            TensorDtype::U32,
            TensorDtype::U64,
            TensorDtype::Bool,
        ] {
            let (code, bits, lanes) = dtype.to_dlpack();
            let recovered = TensorDtype::from_dlpack(code, bits, lanes);
            assert_eq!(recovered, Some(dtype), "roundtrip failed for {:?}", dtype);
        }
    }

    #[test]
    fn test_dtype_parse() {
        assert_eq!(TensorDtype::from_str("float32"), Some(TensorDtype::F32));
        assert_eq!(TensorDtype::from_str("f32"), Some(TensorDtype::F32));
        assert_eq!(TensorDtype::from_str("uint8"), Some(TensorDtype::U8));
        assert_eq!(TensorDtype::from_str("bf16"), Some(TensorDtype::BF16));
        assert_eq!(TensorDtype::from_str("invalid"), None);
    }
}
