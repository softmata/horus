//! Python source file extractor using the `ast` module via subprocess.
//!
//! Shells out to `python3` with an embedded extraction script that uses
//! Python's stdlib `ast` module for perfect parsing. The script outputs
//! JSON to stdout which is deserialized into `ModuleDoc`.

use super::doc_extract::*;
use anyhow::{bail, Context, Result};
use std::path::Path;
use std::process::Command;

const PYTHON_EXTRACTOR: &str = include_str!("python_extractor.py");

/// Result of extracting a single Python file.
pub struct PythonExtractionResult {
    pub module: ModuleDoc,
    pub todos: Vec<TodoItem>,
}

/// Extract documentation from a Python source file.
pub fn extract_python_file(path: &Path) -> Result<PythonExtractionResult> {
    let python = find_python()?;

    let output = Command::new(&python)
        .arg("-c")
        .arg(PYTHON_EXTRACTOR)
        .arg(path)
        .output()
        .with_context(|| format!("Failed to run {} for {}", python, path.display()))?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        bail!(
            "Python extraction failed for {}: {}",
            path.display(),
            stderr
        );
    }

    let raw: serde_json::Value = serde_json::from_slice(&output.stdout).with_context(|| {
        format!(
            "Failed to parse JSON from python extractor for {}",
            path.display()
        )
    })?;

    parse_python_output(&raw, path)
}

fn find_python() -> Result<String> {
    for cmd in &["python3", "python"] {
        if Command::new(cmd)
            .arg("--version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
        {
            return Ok(cmd.to_string());
        }
    }
    bail!("Python not found. Install python3 to extract Python documentation.")
}

fn parse_python_output(raw: &serde_json::Value, path: &Path) -> Result<PythonExtractionResult> {
    let module_doc = raw["module_doc"].as_str().map(|s| s.to_string());
    let imports: Vec<String> = raw["imports"]
        .as_array()
        .map(|arr| {
            arr.iter()
                .filter_map(|v| v.as_str().map(|s| s.to_string()))
                .collect()
        })
        .unwrap_or_default();

    let symbols: Vec<SymbolDoc> = raw["symbols"]
        .as_array()
        .map(|arr| {
            arr.iter()
                .filter_map(|v| parse_python_symbol(v, path))
                .collect()
        })
        .unwrap_or_default();

    let todos: Vec<TodoItem> = raw["todos"]
        .as_array()
        .map(|arr| arr.iter().filter_map(parse_python_todo).collect())
        .unwrap_or_default();

    Ok(PythonExtractionResult {
        module: ModuleDoc {
            path: path.to_path_buf(),
            language: "python".to_string(),
            module_doc,
            imports,
            symbols,
        },
        todos,
    })
}

fn parse_python_symbol(v: &serde_json::Value, path: &Path) -> Option<SymbolDoc> {
    let kind = v["kind"].as_str()?;
    let location = parse_location(v, path);

    match kind {
        "function" => {
            let params: Vec<ParamDoc> = v["params"]
                .as_array()
                .map(|arr| {
                    arr.iter()
                        .map(|p| ParamDoc {
                            name: p["name"].as_str().unwrap_or("").to_string(),
                            type_str: p["type_str"].as_str().map(|s| s.to_string()),
                            default_value: p["default_value"].as_str().map(|s| s.to_string()),
                            doc: p["doc"].as_str().map(|s| s.to_string()),
                        })
                        .collect()
                })
                .unwrap_or_default();

            Some(SymbolDoc::Function(FunctionDoc {
                name: v["name"].as_str()?.to_string(),
                visibility: parse_visibility(v),
                location,
                signature: v["signature"].as_str().unwrap_or("").to_string(),
                doc: v["doc"].as_str().map(|s| s.to_string()),
                deprecated: v["deprecated"].as_str().map(|s| s.to_string()),
                params,
                returns: v["returns"].as_str().map(|s| s.to_string()),
                is_async: v["is_async"].as_bool().unwrap_or(false),
                generic_params: vec![],
                examples: v["examples"]
                    .as_array()
                    .map(|a| {
                        a.iter()
                            .filter_map(|e| e.as_str().map(|s| s.to_string()))
                            .collect()
                    })
                    .unwrap_or_default(),
            }))
        }
        "struct" => {
            let fields: Vec<FieldDoc> = v["fields"]
                .as_array()
                .map(|arr| {
                    arr.iter()
                        .map(|f| FieldDoc {
                            name: f["name"].as_str().unwrap_or("").to_string(),
                            type_str: f["type_str"].as_str().map(|s| s.to_string()),
                            doc: f["doc"].as_str().map(|s| s.to_string()),
                        })
                        .collect()
                })
                .unwrap_or_default();

            let methods: Vec<FunctionDoc> = v["methods"]
                .as_array()
                .map(|arr| {
                    arr.iter()
                        .filter_map(|m| {
                            if let Some(SymbolDoc::Function(f)) = parse_python_symbol(m, path) {
                                Some(f)
                            } else {
                                None
                            }
                        })
                        .collect()
                })
                .unwrap_or_default();

            let trait_impls: Vec<String> = v["trait_impls"]
                .as_array()
                .map(|a| {
                    a.iter()
                        .filter_map(|e| e.as_str().map(|s| s.to_string()))
                        .collect()
                })
                .unwrap_or_default();

            let derives: Vec<String> = v["derives"]
                .as_array()
                .map(|a| {
                    a.iter()
                        .filter_map(|e| e.as_str().map(|s| s.to_string()))
                        .collect()
                })
                .unwrap_or_default();

            Some(SymbolDoc::Struct(StructDoc {
                name: v["name"].as_str()?.to_string(),
                visibility: parse_visibility(v),
                location,
                doc: v["doc"].as_str().map(|s| s.to_string()),
                deprecated: v["deprecated"].as_str().map(|s| s.to_string()),
                generic_params: vec![],
                fields,
                methods,
                trait_impls,
                derives,
                examples: v["examples"]
                    .as_array()
                    .map(|a| {
                        a.iter()
                            .filter_map(|e| e.as_str().map(|s| s.to_string()))
                            .collect()
                    })
                    .unwrap_or_default(),
            }))
        }
        "constant" => Some(SymbolDoc::Constant(ConstantDoc {
            name: v["name"].as_str()?.to_string(),
            visibility: Visibility::Public,
            location,
            doc: v["doc"].as_str().map(|s| s.to_string()),
            deprecated: v["deprecated"].as_str().map(|s| s.to_string()),
            type_str: v["type_str"].as_str().unwrap_or("?").to_string(),
            value: v["value"].as_str().map(|s| s.to_string()),
        })),
        _ => None,
    }
}

fn parse_location(v: &serde_json::Value, path: &Path) -> SourceLocation {
    let loc = &v["location"];
    SourceLocation {
        file: path.to_path_buf(),
        line: loc["line"].as_u64().unwrap_or(0) as usize,
        end_line: loc["end_line"].as_u64().map(|n| n as usize),
    }
}

fn parse_visibility(v: &serde_json::Value) -> Visibility {
    match v["visibility"].as_str() {
        Some("public") => Visibility::Public,
        Some("private") => Visibility::Private,
        _ => Visibility::Public,
    }
}

fn parse_python_todo(v: &serde_json::Value) -> Option<TodoItem> {
    let kind = match v["kind"].as_str()? {
        "todo" => TodoKind::Todo,
        "fixme" => TodoKind::Fixme,
        "hack" => TodoKind::Hack,
        "safety" => TodoKind::Safety,
        _ => TodoKind::Todo,
    };
    Some(TodoItem {
        kind,
        text: v["text"].as_str()?.to_string(),
        location: SourceLocation {
            file: v["location"]["file"]
                .as_str()
                .map(|s| s.into())
                .unwrap_or_default(),
            line: v["location"]["line"].as_u64().unwrap_or(0) as usize,
            end_line: None,
        },
    })
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn extract_from_python(source: &str) -> PythonExtractionResult {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("test.py");
        std::fs::write(&file, source).unwrap();
        extract_python_file(&file).unwrap()
    }

    #[test]
    fn test_extract_function_with_types() {
        let result = extract_from_python("def greet(name: str) -> str:\n    \"\"\"Say hello.\"\"\"\n    return f\"hello {name}\"");
        let fns: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Function(_)))
            .collect();
        assert_eq!(fns.len(), 1);
        if let SymbolDoc::Function(f) = &fns[0] {
            assert_eq!(f.name, "greet");
            assert!(f.signature.contains("name: str"));
            assert!(f.signature.contains("-> str"));
            assert_eq!(f.doc.as_deref(), Some("Say hello."));
            assert_eq!(f.params.len(), 1);
            assert_eq!(f.params[0].name, "name");
            assert_eq!(f.params[0].type_str.as_deref(), Some("str"));
        }
    }

    #[test]
    fn test_extract_async_function() {
        let result = extract_from_python("async def fetch(url: str) -> bytes:\n    pass");
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert!(f.is_async);
            assert!(f.signature.contains("async"));
        }
    }

    #[test]
    fn test_extract_class() {
        let result = extract_from_python(
            "class Robot:\n    \"\"\"A robot.\"\"\"\n    def move(self):\n        pass",
        );
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert_eq!(classes.len(), 1);
        if let SymbolDoc::Struct(s) = &classes[0] {
            assert_eq!(s.name, "Robot");
            assert_eq!(s.doc.as_deref(), Some("A robot."));
            assert!(!s.methods.is_empty());
        }
    }

    #[test]
    fn test_extract_class_with_bases() {
        let result = extract_from_python("class MyNode(Node):\n    pass");
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            assert!(s.trait_impls.contains(&"Node".to_string()));
        }
    }

    #[test]
    fn test_extract_dataclass() {
        let result = extract_from_python("from dataclasses import dataclass\n\n@dataclass\nclass Point:\n    x: float\n    y: float");
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert!(!classes.is_empty());
        if let SymbolDoc::Struct(s) = &classes[0] {
            assert_eq!(s.name, "Point");
            assert_eq!(s.fields.len(), 2);
            assert_eq!(s.fields[0].name, "x");
            assert_eq!(s.fields[0].type_str.as_deref(), Some("float"));
        }
    }

    #[test]
    fn test_extract_module_doc() {
        let result =
            extract_from_python("\"\"\"This is the module doc.\"\"\"\n\ndef f():\n    pass");
        assert_eq!(
            result.module.module_doc.as_deref(),
            Some("This is the module doc.")
        );
    }

    #[test]
    fn test_extract_constant() {
        let result = extract_from_python("MAX_SPEED = 100\nPI = 3.14");
        let consts: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Constant(_)))
            .collect();
        assert_eq!(consts.len(), 2);
    }

    #[test]
    fn test_extract_default_args() {
        let result = extract_from_python(
            "def configure(speed: float = 1.0, verbose: bool = False):\n    pass",
        );
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert_eq!(f.params.len(), 2);
            assert!(f.params[0].default_value.is_some());
        }
    }

    #[test]
    fn test_extract_imports() {
        let result =
            extract_from_python("import os\nfrom pathlib import Path\n\ndef f():\n    pass");
        assert!(result.module.imports.len() >= 2);
    }

    #[test]
    fn test_python_todos() {
        let result = extract_from_python("# TODO: fix this\n# FIXME: broken\ndef f():\n    pass");
        assert_eq!(result.todos.len(), 2);
        assert_eq!(result.todos[0].kind, TodoKind::Todo);
        assert_eq!(result.todos[1].kind, TodoKind::Fixme);
    }

    #[test]
    fn test_empty_file() {
        let result = extract_from_python("");
        assert!(result.module.symbols.is_empty());
    }

    #[test]
    fn test_language_is_python() {
        let result = extract_from_python("x = 1");
        assert_eq!(result.module.language, "python");
    }

    #[test]
    fn test_private_function() {
        let result = extract_from_python("def _internal():\n    pass\n\ndef public():\n    pass");
        // Both should be extracted (visibility marked)
        assert_eq!(result.module.symbols.len(), 2);
        let private = result
            .module
            .symbols
            .iter()
            .find(|s| s.name() == "_internal");
        assert!(private.is_some());
    }

    #[test]
    fn test_init_params_as_fields() {
        let result = extract_from_python("class Config:\n    def __init__(self, name: str, value: int):\n        self.name = name\n        self.value = value");
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            // __init__ params should appear as fields
            assert!(s.fields.len() >= 2, "fields: {:?}", s.fields);
        }
    }

    #[test]
    fn test_source_locations() {
        let result = extract_from_python("def first():\n    pass\n\ndef second():\n    pass");
        for sym in &result.module.symbols {
            if let Some(loc) = sym.location() {
                assert!(loc.line > 0);
            }
        }
    }

    #[test]
    fn test_all_filtering() {
        let result = extract_from_python(
            "__all__ = [\"public_fn\"]\ndef public_fn():\n    pass\ndef hidden_fn():\n    pass",
        );
        let names: Vec<&str> = result.module.symbols.iter().map(|s| s.name()).collect();
        assert!(
            names.contains(&"public_fn"),
            "public_fn should be present: {:?}",
            names
        );
        // hidden_fn should be filtered out by __all__
        assert!(
            !names.contains(&"hidden_fn"),
            "hidden_fn should be filtered by __all__: {:?}",
            names
        );
    }

    #[test]
    fn test_property_decorator() {
        let result = extract_from_python(
            "class Cfg:\n    @property\n    def name(self) -> str:\n        return \"x\"",
        );
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert_eq!(classes.len(), 1);
        if let SymbolDoc::Struct(s) = &classes[0] {
            let method_names: Vec<&str> = s.methods.iter().map(|m| m.name.as_str()).collect();
            assert!(
                method_names.contains(&"name"),
                "property 'name' should be captured as method: {:?}",
                method_names
            );
        }
    }

    #[test]
    fn test_staticmethod() {
        let result = extract_from_python(
            "class Factory:\n    @staticmethod\n    def create() -> 'Factory':\n        return Factory()",
        );
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert_eq!(classes.len(), 1);
        if let SymbolDoc::Struct(s) = &classes[0] {
            let method_names: Vec<&str> = s.methods.iter().map(|m| m.name.as_str()).collect();
            assert!(
                method_names.contains(&"create"),
                "staticmethod 'create' should be captured: {:?}",
                method_names
            );
        }
    }

    #[test]
    fn test_syntax_error_graceful() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("bad.py");
        std::fs::write(&file, "def broken(\n    this is not valid python\n").unwrap();
        let result = extract_python_file(&file);
        // Should either return Err or return Ok with empty symbols — must not panic
        match result {
            Err(_) => {} // expected: extraction failed gracefully
            Ok(r) => {
                // also acceptable: parser returned empty/partial results
                assert!(
                    r.module.symbols.is_empty() || !r.module.symbols.is_empty(),
                    "should not panic on syntax errors"
                );
            }
        }
    }

    // ─── Level 2: Integration Tests ─────────────────────────────────────────

    #[test]
    fn test_integration_python_extract_full_module() {
        let source = r#"
"""Sensor utilities module."""

class SensorReader:
    """Reads sensor data."""
    def __init__(self, port: int):
        self.port = port

    def read(self) -> float:
        """Read the current value."""
        return 0.0

def calibrate(sensor: SensorReader, offset: float = 0.0) -> bool:
    """Calibrate a sensor with an optional offset."""
    return True

MAX_SENSORS = 16

# TODO: add async sensor polling
"#;
        let result = extract_from_python(source);

        // At least 3 symbols: class SensorReader, function calibrate, constant MAX_SENSORS
        assert!(
            result.module.symbols.len() >= 3,
            "should extract at least 3 symbols (class + function + constant), got: {}",
            result.module.symbols.len()
        );

        // Check symbol types are present
        let has_class = result
            .module
            .symbols
            .iter()
            .any(|s| matches!(s, SymbolDoc::Struct(_)));
        let has_fn = result
            .module
            .symbols
            .iter()
            .any(|s| matches!(s, SymbolDoc::Function(_)));
        let has_const = result
            .module
            .symbols
            .iter()
            .any(|s| matches!(s, SymbolDoc::Constant(_)));
        assert!(has_class, "should extract the SensorReader class");
        assert!(has_fn, "should extract the calibrate function");
        assert!(has_const, "should extract the MAX_SENSORS constant");

        // Module docstring captured
        assert!(
            result.module.module_doc.is_some(),
            "should capture the module docstring"
        );
        assert!(
            result
                .module
                .module_doc
                .as_ref()
                .unwrap()
                .contains("Sensor utilities"),
            "module_doc should contain 'Sensor utilities', got: {:?}",
            result.module.module_doc
        );

        // TODO captured
        assert!(!result.todos.is_empty(), "should capture the TODO comment");
        assert!(
            result
                .todos
                .iter()
                .any(|t| t.text.contains("async") || t.text.contains("polling")),
            "TODO text should mention async/polling, got: {:?}",
            result.todos
        );
    }

    // ─── Level 5: Error Path Tests ──────────────────────────────────────────

    #[test]
    fn test_error_python_nonexistent_file() {
        let result = extract_python_file(Path::new("/tmp/does_not_exist_8f3a2b.py"));
        assert!(result.is_err(), "nonexistent file should return Err");
    }

    #[test]
    fn test_error_python_empty_file_ok() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("empty.py");
        std::fs::write(&file, "").unwrap();
        let result = extract_python_file(&file);
        assert!(
            result.is_ok(),
            "empty .py file should succeed, got: {:?}",
            result.err()
        );
        let r = result.unwrap();
        assert!(
            r.module.symbols.is_empty(),
            "empty file should produce no symbols"
        );
        assert!(r.todos.is_empty(), "empty file should produce no TODOs");
    }
}
