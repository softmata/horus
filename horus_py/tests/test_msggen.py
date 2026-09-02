"""
Tests for `horus.msggen`, the custom-message code generator.

This module had no test of any kind: nothing in the repo imported it, and the
only guard that existed (horus_manager/tests/py_typed_contract.rs) checks that
`lib.rs` declares `mod custom_messages;` -- i.e. that generated files are
*compiled*, not that they are *correct*. Three separate ways of producing Rust
that does not compile therefore went unnoticed. Each is pinned below, and each
assertion was verified against a real `cargo check -p horus_py` before landing.
"""
import pytest

from horus.msggen.generator import (
    TYPE_MAP,
    MessageDef,
    MessageField,
    generate_message,
    parse_field_type,
)


class TestParseFieldType:
    @pytest.mark.parametrize("alias,expected", [
        ("f32", "f32"), ("float", "f64"), ("float32", "f32"),
        ("int", "i64"), ("uint", "u64"), ("bool", "bool"),
        ("string", "String"), ("str", "String"), ("vec_u8", "Vec<u8>"),
    ])
    def test_aliases_map_to_rust_types(self, alias, expected):
        assert parse_field_type(alias)[0] == expected

    def test_type_lookup_is_case_insensitive(self):
        assert parse_field_type("F32") == parse_field_type("f32")

    def test_unknown_type_is_rejected(self):
        with pytest.raises(ValueError, match="Unknown type"):
            parse_field_type("complex128")

    @pytest.mark.parametrize("type_str,default", [
        ("[f32; 3]", "[0.0; 3]"),
        ("[f64; 16]", "[0.0; 16]"),
        ("[i32; 5]", "[0; 5]"),
        ("[u8; 2]", "[0; 2]"),
    ])
    def test_array_default_matches_element_type_and_length(self, type_str, default):
        """
        The default used to be the constant "[0.0; 3]" for every array, so
        `[i32; 5]` got a float default of the wrong length.
        """
        rust_type, got = parse_field_type(type_str)
        assert rust_type == type_str
        assert got == default

    @pytest.mark.parametrize("type_str,rust_type,default", [
        ("[float; 3]", "[f64; 3]", "[0.0; 3]"),
        ("[int; 4]", "[i64; 4]", "[0; 4]"),
        ("[boolean; 2]", "[bool; 2]", "[false; 2]"),
    ])
    def test_array_element_is_resolved_to_a_rust_type(self, type_str, rust_type, default):
        """HORUS spellings are not Rust type names.

        The element token used to be echoed back verbatim, so `[float; 3]`
        emitted `[float; 3]` — `float` is a legal HORUS scalar and not a Rust
        type, and the generated file failed to compile with E0412. The scalar
        path has always mapped these; the array path did not.
        """
        assert parse_field_type(type_str) == (rust_type, default)

    def test_non_copy_array_default_is_built_per_slot(self):
        """`[expr; N]` requires Copy, and String is not.

        `[String; 2]` used to default to `[String::new(); 2]`, which is E0277
        rather than a value. Each slot has to be constructed on its own.
        """
        rust_type, default = parse_field_type("[string; 2]")
        assert rust_type == "[String; 2]"
        assert default == "std::array::from_fn(|_| String::new())"

    def test_unknown_array_element_is_rejected(self):
        """An unknown element raises here, as it does for a bare scalar.

        It used to fall back to `Default::default()` and emit Rust that does
        not build, which moved the error from this generator to rustc — with
        no mention of the message definition that caused it.
        """
        with pytest.raises(ValueError, match="Unknown array element type"):
            parse_field_type("[weird; 2]")

    def test_vec_passthrough(self):
        assert parse_field_type("Vec<f32>") == ("Vec<f32>", "Vec::new()")


class TestGeneratedRustCompiles:
    """
    Each test here corresponds to a confirmed `cargo check` failure of the
    previous generator output. The error each one prevents is named.
    """

    def test_array_fields_use_debug_not_display(self):
        """E0277: `[f32; 4]` doesn't implement `std::fmt::Display`."""
        code = generate_message("PoseArr", "pose.arr", [("quat", "[f32; 4]")])
        assert "quat={:?}" in code
        assert "quat={}" not in code

    def test_defaulted_field_may_not_precede_a_required_one(self):
        """
        pyo3: "cannot have required positional parameter after an optional
        parameter". A Vec field is emitted as Option<_>=None, so declaring it
        before a plain field produced `signature = (samples=None, count)`.
        """
        code = generate_message(
            "Mixed", "m.x", [("samples", "vec_f32"), ("count", "i32")]
        )
        assert "signature = (samples, count)" in code
        # ...and if it carries no default it must not be Option-typed either,
        # or the struct assignment and the parameter disagree.
        assert "samples: Vec<f32>" in code
        assert "Option<Vec<f32>>" not in code
        assert "unwrap_or_default" not in code

    def test_trailing_vec_still_gets_its_default(self):
        """The ordering fix must not cost the common case its default."""
        code = generate_message(
            "Trailing", "t.x", [("count", "i32"), ("samples", "vec_u8")]
        )
        assert "signature = (count, samples=None)" in code
        assert "samples: Option<Vec<u8>>" in code
        assert "samples: samples.unwrap_or_default()" in code

    @pytest.mark.parametrize("declared,default", [
        ("f64", "0.0"), ("f32", "0.0"), ("u64", "0"), ("i32", "0"),
    ])
    def test_timestamp_default_matches_its_declared_type(self, declared, default):
        """
        E0308: expected `f64`, found integer. `timestamp=0` was emitted for
        every type, and an integer literal does not coerce to a float.
        """
        code = generate_message("Stamped", "s.x", [("timestamp", declared)])
        assert f"signature = (timestamp={default})" in code

    def test_topic_name_is_exposed_as_a_class_attribute(self):
        code = generate_message("RobotStatus", "robot.status", [("ok", "bool")])
        assert '"robot.status"' in code
        assert "__topic_name__" in code

    def test_pyclass_name_is_the_declared_name(self):
        code = generate_message("RobotStatus", "robot.status", [("ok", "bool")])
        assert '#[pyclass(name = "RobotStatus")]' in code
        assert "pub struct PyRobotStatus" in code

    def test_every_field_is_declared_and_assigned(self):
        fields = [("a", "f32"), ("b", "i64"), ("c", "string"), ("d", "bool")]
        code = generate_message("Many", "m.y", fields)
        for name, _ in fields:
            assert f"pub {name}:" in code
        body = code.split("fn new(")[1]
        for name, _ in fields:
            assert name in body

    def test_unknown_field_type_fails_at_generation_not_at_cargo_build(self):
        with pytest.raises(ValueError):
            generate_message("Bad", "b.x", [("weird", "quaternion")])


class TestMessageDataclasses:
    def test_message_def_round_trips(self):
        f = MessageField(name="x", rust_type="f32", default="0.0")
        m = MessageDef(name="M", topic="t", fields=[f], doc="d")
        assert m.fields[0].rust_type == "f32"
        assert m.doc == "d"

    def test_type_map_defaults_are_self_consistent(self):
        """Every default must be a plausible literal for its own rust type."""
        for alias, (rust_type, default) in TYPE_MAP.items():
            if rust_type in ("f32", "f64"):
                assert "." in default
            elif rust_type == "bool":
                assert default == "false"
            elif rust_type == "String":
                assert "String::new()" == default
            elif rust_type.startswith("Vec<"):
                assert default == "Vec::new()"
            else:
                assert default.isdigit()
