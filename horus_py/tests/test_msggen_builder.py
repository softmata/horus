"""
Tests for the pure parts of `horus.msggen.builder`.

builder.py sat at 13% coverage with nothing importing it. Two of its functions
carry real risk and neither needs the filesystem:

  * compute_messages_hash decides whether generated Rust is rebuilt. If it
    fails to change when a definition changes, the user keeps a stale
    extension and their edit silently does nothing.
  * generate_custom_messages_mod writes the mod.rs that wires generated
    classes into the extension. horus_manager/tests/py_typed_contract.rs
    exists because that wiring was once absent entirely and every generated
    class was dead on arrival with no error anywhere.

build_messages() itself shells out to cargo and writes into horus_py/src, so
it is left to the integration path rather than exercised here.
"""
import pytest

from horus.msggen.builder import (
    compute_messages_hash,
    generate_custom_messages_mod,
)
from horus.msggen.generator import MessageDef, MessageField


def msg(name, topic="t.x", fields=(("a", "f32"),)):
    return MessageDef(
        name=name,
        topic=topic,
        fields=[MessageField(name=n, rust_type=t, default="0.0") for n, t in fields],
    )


class TestMessagesHash:
    def test_is_stable_for_identical_input(self):
        assert compute_messages_hash([msg("A")]) == compute_messages_hash([msg("A")])

    def test_does_not_depend_on_declaration_order(self):
        """The function sorts by name; two orderings are the same set."""
        a, b = msg("A"), msg("B")
        assert compute_messages_hash([a, b]) == compute_messages_hash([b, a])

    def test_changes_when_a_field_type_changes(self):
        """The rebuild trigger. If this collides, users ship a stale extension."""
        before = compute_messages_hash([msg("A", fields=(("a", "f32"),))])
        after = compute_messages_hash([msg("A", fields=(("a", "f64"),))])
        assert before != after

    def test_changes_when_a_field_is_added(self):
        before = compute_messages_hash([msg("A", fields=(("a", "f32"),))])
        after = compute_messages_hash([msg("A", fields=(("a", "f32"), ("b", "f32")))])
        assert before != after

    def test_changes_when_a_field_is_renamed(self):
        before = compute_messages_hash([msg("A", fields=(("a", "f32"),))])
        after = compute_messages_hash([msg("A", fields=(("z", "f32"),))])
        assert before != after

    def test_changes_when_the_topic_changes(self):
        assert compute_messages_hash([msg("A", topic="one")]) != \
               compute_messages_hash([msg("A", topic="two")])

    def test_changes_when_a_message_is_added(self):
        assert compute_messages_hash([msg("A")]) != \
               compute_messages_hash([msg("A"), msg("B")])

    def test_empty_set_has_a_hash(self):
        assert isinstance(compute_messages_hash([]), str)
        assert compute_messages_hash([]) != compute_messages_hash([msg("A")])


class TestGeneratedModRs:
    def test_empty_module_registers_nothing_but_still_defines_the_hook(self):
        """
        lib.rs calls register_custom_messages unconditionally, so the empty
        form must still define it or the crate does not build.
        """
        code = generate_custom_messages_mod([])
        assert "pub fn register_custom_messages" in code
        assert "Ok(())" in code
        assert "add_class" not in code

    def test_each_message_is_declared_exported_and_registered(self):
        code = generate_custom_messages_mod([msg("RobotStatus"), msg("MotorCmd")])
        for name in ("RobotStatus", "MotorCmd"):
            assert f"mod {name.lower()};" in code
            assert f"pub use {name.lower()}::Py{name};" in code
            assert f"m.add_class::<Py{name}>()?;" in code

    def test_module_name_is_the_lowercased_type_name(self):
        """Must match generator.generate_message's `{name.lower()}.rs` output."""
        code = generate_custom_messages_mod([msg("RobotStatus")])
        assert "mod robotstatus;" in code
        assert "mod RobotStatus;" not in code

    def test_populated_form_binds_the_module_parameter(self):
        """
        The empty form names the parameter `_m`; the populated form must name
        it `m` because it calls m.add_class. Emitting `_m` there is E0425.
        """
        code = generate_custom_messages_mod([msg("RobotStatus")])
        assert "register_custom_messages(m: &Bound" in code

    @pytest.mark.parametrize("count", [1, 2, 5])
    def test_registration_count_matches_message_count(self, count):
        msgs = [msg(f"M{i}") for i in range(count)]
        code = generate_custom_messages_mod(msgs)
        assert code.count("add_class::<") == count
