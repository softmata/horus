#![allow(dead_code)]
//! Edge-case tests for the doc extraction system.
//!
//! These tests exercise `extract_rust_file()` — the pure function that parses
//! Rust source code and returns structured `ModuleDoc` with all public symbols.
//! All tests create temporary `.rs` files and extract documentation from them.
//!
//! Covers:
//! - Documented pub fn extraction
//! - Documented pub struct with fields
//! - Impl block method association
//! - Empty file (no crash)
//! - Private-only file (no pub items extracted)

use horus_manager::commands::doc_extract::{SymbolDoc, Visibility};
use horus_manager::commands::doc_extract_rust::extract_rust_file;
use std::io::Write;
use tempfile::TempDir;

// ============================================================================
// Test 5: Extract Rust doc from pub fn
// ============================================================================

/// Create a temp .rs file with a documented pub fn. Extract docs.
/// Assert the function name and doc comment are captured.
#[test]
fn test_extract_rust_doc_from_pub_fn() {
    let dir = TempDir::new().unwrap();
    let file = dir.path().join("example.rs");
    let mut f = std::fs::File::create(&file).unwrap();
    writeln!(
        f,
        "/// Computes the sum of two integers.\npub fn add(a: i32, b: i32) -> i32 {{\n    a + b\n}}"
    )
    .unwrap();

    let result = extract_rust_file(&file, false).unwrap();

    assert_eq!(
        result.module.symbols.len(),
        1,
        "Should extract exactly one symbol from a single pub fn"
    );

    let sym = &result.module.symbols[0];
    assert_eq!(sym.name(), "add", "Function name should be 'add'");

    if let SymbolDoc::Function(func) = sym {
        assert_eq!(
            func.doc.as_deref(),
            Some("Computes the sum of two integers."),
            "Doc comment should be captured verbatim"
        );
        assert_eq!(func.visibility, Visibility::Public);
        assert_eq!(func.params.len(), 2, "Should capture both parameters");
        assert_eq!(func.params[0].name, "a");
        assert_eq!(func.params[1].name, "b");
        assert_eq!(
            func.returns.as_deref(),
            Some("i32"),
            "Return type should be captured"
        );
    } else {
        panic!("Expected SymbolDoc::Function, got {:?}", sym);
    }
}

// ============================================================================
// Test 6: Extract Rust doc from pub struct
// ============================================================================

/// Create a temp .rs file with a documented pub struct. Extract.
/// Assert struct name, doc comment, and fields are captured.
#[test]
fn test_extract_rust_doc_from_pub_struct() {
    let dir = TempDir::new().unwrap();
    let file = dir.path().join("config.rs");
    let source = r#"
/// Configuration for the robot controller.
#[derive(Debug, Clone)]
pub struct RobotConfig {
    /// Maximum velocity in m/s.
    pub max_velocity: f64,
    /// Whether to enable safety mode.
    pub safety_enabled: bool,
}
"#;
    std::fs::write(&file, source).unwrap();

    let result = extract_rust_file(&file, false).unwrap();

    assert_eq!(
        result.module.symbols.len(),
        1,
        "Should extract exactly one symbol from a single pub struct"
    );

    let sym = &result.module.symbols[0];
    assert_eq!(sym.name(), "RobotConfig");

    if let SymbolDoc::Struct(s) = sym {
        assert_eq!(
            s.doc.as_deref(),
            Some("Configuration for the robot controller."),
            "Struct doc comment should be captured"
        );
        assert_eq!(s.fields.len(), 2, "Both pub fields should be captured");
        assert_eq!(s.fields[0].name, "max_velocity");
        assert_eq!(
            s.fields[0].doc.as_deref(),
            Some("Maximum velocity in m/s."),
            "Field doc comments should be captured"
        );
        assert_eq!(s.fields[1].name, "safety_enabled");
        assert!(
            s.derives.contains(&"Debug".to_string()),
            "Derives should be captured"
        );
        assert!(
            s.derives.contains(&"Clone".to_string()),
            "Derives should be captured"
        );
    } else {
        panic!("Expected SymbolDoc::Struct, got {:?}", sym);
    }
}

// ============================================================================
// Test 7: Extract Rust doc from impl block
// ============================================================================

/// Create a temp .rs file with a pub struct and an impl block containing
/// a pub method. Extract. Assert the method is associated with the struct.
#[test]
fn test_extract_rust_doc_from_impl_block() {
    let dir = TempDir::new().unwrap();
    let file = dir.path().join("motor.rs");
    let source = r#"
/// A DC motor driver.
pub struct Motor {
    pub speed: f64,
}

impl Motor {
    /// Create a new motor at rest.
    pub fn new() -> Self {
        Motor { speed: 0.0 }
    }

    /// Set the motor speed in RPM.
    pub fn set_speed(&mut self, rpm: f64) {
        self.speed = rpm;
    }

    /// Internal helper — not public.
    fn clamp_speed(&self) -> f64 {
        self.speed.clamp(-1000.0, 1000.0)
    }
}
"#;
    std::fs::write(&file, source).unwrap();

    let result = extract_rust_file(&file, false).unwrap();

    // Should have the struct as the only top-level symbol
    assert_eq!(
        result.module.symbols.len(),
        1,
        "Should extract one top-level symbol (the struct)"
    );

    let sym = &result.module.symbols[0];
    assert_eq!(sym.name(), "Motor");

    if let SymbolDoc::Struct(s) = sym {
        // Methods from impl block should be associated with the struct
        assert!(
            s.methods.len() >= 2,
            "At least 2 pub methods (new, set_speed) should be associated, got {}",
            s.methods.len()
        );

        let method_names: Vec<&str> = s.methods.iter().map(|m| m.name.as_str()).collect();
        assert!(
            method_names.contains(&"new"),
            "Method 'new' should be captured, got {:?}",
            method_names
        );
        assert!(
            method_names.contains(&"set_speed"),
            "Method 'set_speed' should be captured, got {:?}",
            method_names
        );

        // Private method should NOT be captured (include_private=false)
        assert!(
            !method_names.contains(&"clamp_speed"),
            "Private method 'clamp_speed' should NOT be captured"
        );

        // Check that doc comment is captured on impl methods
        let new_method = s.methods.iter().find(|m| m.name == "new").unwrap();
        assert_eq!(
            new_method.doc.as_deref(),
            Some("Create a new motor at rest."),
            "Impl method doc should be captured"
        );
    } else {
        panic!("Expected SymbolDoc::Struct, got {:?}", sym);
    }
}

// ============================================================================
// Test 8: Extract from empty file
// ============================================================================

/// Extract from an empty file. Assert empty result (no crash).
#[test]
fn test_extract_rust_doc_empty_file() {
    let dir = TempDir::new().unwrap();
    let file = dir.path().join("empty.rs");
    std::fs::write(&file, "").unwrap();

    let result = extract_rust_file(&file, false).unwrap();

    assert!(
        result.module.symbols.is_empty(),
        "Empty file should produce no symbols, got {}",
        result.module.symbols.len()
    );
    assert!(
        result.module.imports.is_empty(),
        "Empty file should produce no imports"
    );
    assert_eq!(
        result.module.language, "rust",
        "Language should still be 'rust'"
    );
}

// ============================================================================
// Test 9: File with only private functions — no pub items extracted
// ============================================================================

/// File with only private functions. Assert no items extracted
/// (only pub items are documented when include_private=false).
#[test]
fn test_extract_rust_doc_no_pub_items() {
    let dir = TempDir::new().unwrap();
    let file = dir.path().join("private.rs");
    let source = r#"
/// A private helper function.
fn helper() -> u32 {
    42
}

/// Another private function.
fn internal_calc(x: f64) -> f64 {
    x * 2.0
}

struct InternalState {
    counter: u64,
}

impl InternalState {
    fn new() -> Self {
        InternalState { counter: 0 }
    }

    fn increment(&mut self) {
        self.counter += 1;
    }
}
"#;
    std::fs::write(&file, source).unwrap();

    let result = extract_rust_file(&file, false).unwrap();

    assert!(
        result.module.symbols.is_empty(),
        "File with only private items should produce no symbols when include_private=false, got {}",
        result.module.symbols.len()
    );
}
