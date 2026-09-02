"""
HORUS Message Generator - Rust Code Generation

Generates Rust PyO3 message classes from Python definitions.
"""

import os
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any
import re
from dataclasses import dataclass

try:
    import yaml
    _HAS_YAML = True
except ImportError:
    yaml = None
    _HAS_YAML = False

# Type mapping from user-friendly names to Rust types
TYPE_MAP = {
    # Floats
    "f32": ("f32", "0.0"),
    "f64": ("f64", "0.0"),
    "float": ("f64", "0.0"),
    "float32": ("f32", "0.0"),
    "float64": ("f64", "0.0"),

    # Integers
    "i8": ("i8", "0"),
    "i16": ("i16", "0"),
    "i32": ("i32", "0"),
    "i64": ("i64", "0"),
    "u8": ("u8", "0"),
    "u16": ("u16", "0"),
    "u32": ("u32", "0"),
    "u64": ("u64", "0"),
    "int": ("i64", "0"),
    "uint": ("u64", "0"),

    # Boolean
    "bool": ("bool", "false"),
    "boolean": ("bool", "false"),

    # String (note: strings break zero-copy, use sparingly)
    "string": ("String", "String::new()"),
    "str": ("String", "String::new()"),

    # Arrays (common patterns)
    "vec_f32": ("Vec<f32>", "Vec::new()"),
    "vec_f64": ("Vec<f64>", "Vec::new()"),
    "vec_i32": ("Vec<i32>", "Vec::new()"),
    "vec_u8": ("Vec<u8>", "Vec::new()"),
}


@dataclass
class MessageField:
    """A field in a message definition."""
    name: str
    rust_type: str
    default: str
    doc: Optional[str] = None


@dataclass
class MessageDef:
    """A complete message definition."""
    name: str
    topic: str
    fields: List[MessageField]
    doc: Optional[str] = None


def parse_field_type(type_str: str) -> Tuple[str, str]:
    """Parse a type string and return (rust_type, default_value)."""
    type_lower = type_str.lower().strip()

    if type_lower in TYPE_MAP:
        return TYPE_MAP[type_lower]

    # Handle array types like [f32; 3]. The default has to be built from this
    # array's own element type and length: a hardcoded "[0.0; 3]" gave `[i32; 5]`
    # a float default of the wrong length.
    array = re.match(r"^\[\s*([A-Za-z0-9_]+)\s*;\s*(\d+)\s*\]$", type_str.strip())
    if array:
        elem, count = array.group(1), array.group(2)
        elem_default = TYPE_MAP.get(elem.lower(), (elem, "Default::default()"))[1]
        return (type_str, f"[{elem_default}; {count}]")

    # Handle Vec<T> directly
    if type_str.startswith("Vec<"):
        return (type_str, "Vec::new()")

    raise ValueError(f"Unknown type: {type_str}. Valid types: {list(TYPE_MAP.keys())}")


def generate_message(
    name: str,
    topic: str,
    fields: List[Tuple[str, str]],
    doc: Optional[str] = None,
    output_dir: Optional[Path] = None,
) -> str:
    """
    Generate Rust code for a custom message type.

    Args:
        name: Message class name (e.g., "RobotStatus")
        topic: Topic name (e.g., "robot.status")
        fields: List of (field_name, field_type) tuples
        doc: Optional docstring
        output_dir: Where to write the generated file

    Returns:
        Generated Rust code as a string
    """
    # Parse fields
    parsed_fields = []
    for field_name, field_type in fields:
        rust_type, default = parse_field_type(field_type)
        parsed_fields.append(MessageField(
            name=field_name,
            rust_type=rust_type,
            default=default,
        ))

    msg_def = MessageDef(
        name=name,
        topic=topic,
        fields=parsed_fields,
        doc=doc,
    )

    rust_code = _generate_rust_code(msg_def)

    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_file = output_dir / f"{name.lower()}.rs"
        output_file.write_text(rust_code)
        print(f"Generated: {output_file}")

    return rust_code


def _generate_rust_code(msg: MessageDef) -> str:
    """Generate the actual Rust code for a message."""

    # Build struct fields
    struct_fields = []
    for field in msg.fields:
        struct_fields.append(f"""    #[pyo3(get, set)]
    pub {field.name}: {field.rust_type},""")

    # Which fields would LIKE a default: Vec fields become Option<_>, and a
    # field literally named `timestamp` gets its type's zero value.
    wants_default = [
        f.rust_type.startswith("Vec<") or f.name == "timestamp" for f in msg.fields
    ]

    # Rust (and therefore pyo3's #[pyo3(signature = ...)]) rejects a defaulted
    # parameter followed by a required one, so only the TRAILING run of
    # would-be-defaulted fields may actually carry a default. Declaring
    # `[("samples", "vec_f32"), ("count", "i32")]` used to emit
    # `signature = (samples=None, count)`, which does not compile.
    has_default = [False] * len(msg.fields)
    for i in reversed(range(len(msg.fields))):
        trailing_ok = i == len(msg.fields) - 1 or has_default[i + 1]
        has_default[i] = wants_default[i] and trailing_ok

    constructor_params = []
    constructor_defaults = []
    for field, defaulted in zip(msg.fields, has_default):
        if defaulted and field.rust_type.startswith("Vec<"):
            constructor_params.append(f"{field.name}: Option<{field.rust_type}>")
            constructor_defaults.append(f"{field.name}=None")
        elif defaulted:
            # `timestamp=0` was emitted regardless of type; `0` is an integer
            # literal and does not coerce to f64, so a f64 timestamp did not
            # compile. field.default carries the right literal for the type.
            constructor_params.append(f"{field.name}: {field.rust_type}")
            constructor_defaults.append(f"{field.name}={field.default}")
        else:
            constructor_params.append(f"{field.name}: {field.rust_type}")
            constructor_defaults.append(f"{field.name}")

    # Build field assignments in constructor
    field_assignments = []
    for field, defaulted in zip(msg.fields, has_default):
        if defaulted and field.rust_type.startswith("Vec<"):
            field_assignments.append(f"            {field.name}: {field.name}.unwrap_or_default(),")
        else:
            field_assignments.append(f"            {field.name},")

    # Build repr fields
    repr_fields = []
    for field in msg.fields:
        if field.rust_type in ("f32", "f64"):
            repr_fields.append(f"{field.name}={{:.3}}")
        elif field.rust_type.startswith("Vec<"):
            repr_fields.append(f"{field.name}=[{{}} items]")
        elif field.rust_type.startswith("["):
            # Fixed-size arrays do not implement Display, only Debug. `{}` here
            # made every array-typed message fail to compile with E0277.
            repr_fields.append(f"{field.name}={{:?}}")
        else:
            repr_fields.append(f"{field.name}={{}}")

    repr_args = []
    for field in msg.fields:
        if field.rust_type.startswith("Vec<"):
            repr_args.append(f"self.{field.name}.len()")
        else:
            repr_args.append(f"self.{field.name}")

    # Generate docstring
    doc_comment = f'/// {msg.doc}' if msg.doc else f'/// {msg.name} message type (auto-generated)'

    # Full template
    code = f'''//! Auto-generated message type: {msg.name}
//! Topic: {msg.topic}
//! DO NOT EDIT - Generated by horus.msggen

use pyo3::prelude::*;

{doc_comment}
///
/// Topic: {msg.topic}
#[pyclass(name = "{msg.name}")]
#[derive(Clone, Debug)]
pub struct Py{msg.name} {{
{chr(10).join(struct_fields)}
}}

#[pymethods]
impl Py{msg.name} {{
    #[new]
    #[pyo3(signature = ({", ".join(constructor_defaults)}))]
    fn new({", ".join(constructor_params)}) -> Self {{
        Self {{
{chr(10).join(field_assignments)}
        }}
    }}

    #[classattr]
    fn __topic_name__() -> &'static str {{
        "{msg.topic}"
    }}

    fn __repr__(&self) -> String {{
        format!(
            "{msg.name}({", ".join(repr_fields)})",
            {", ".join(repr_args)}
        )
    }}
}}
'''
    return code


def generate_messages_from_yaml(yaml_path: str, output_dir: Optional[Path] = None) -> List[str]:
    if not _HAS_YAML:
        raise ImportError("pyyaml is required for YAML message generation: pip install pyyaml")

    """
    Generate messages from a YAML definition file.

    YAML format:
    ```yaml
    messages:
      - name: RobotStatus
        topic: robot/status
        doc: Robot status information
        fields:
          - name: battery_level
            type: f32
          - name: error_code
            type: i32
          - name: is_active
            type: bool
    ```
    """
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    results = []
    for msg_data in data.get('messages', []):
        fields = [(f['name'], f['type']) for f in msg_data['fields']]
        code = generate_message(
            name=msg_data['name'],
            topic=msg_data['topic'],
            fields=fields,
            doc=msg_data.get('doc'),
            output_dir=output_dir,
        )
        results.append(code)

    return results


# Store generated messages for the build step
_GENERATED_MESSAGES: List[MessageDef] = []


def register_message(name: str, topic: str, fields: List[Tuple[str, str]], doc: Optional[str] = None):
    """Register a message for batch generation."""
    parsed_fields = []
    for field_name, field_type in fields:
        rust_type, default = parse_field_type(field_type)
        parsed_fields.append(MessageField(
            name=field_name,
            rust_type=rust_type,
            default=default,
        ))

    _GENERATED_MESSAGES.append(MessageDef(
        name=name,
        topic=topic,
        fields=parsed_fields,
        doc=doc,
    ))


def get_registered_messages() -> List[MessageDef]:
    """Get all registered messages."""
    return _GENERATED_MESSAGES.copy()
