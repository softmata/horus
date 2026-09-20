"""The TransformFrame docstrings must name the exception the code raises.

`wait_for_transform` and `tf_at_strict` documented `ValueError`. They route
through `to_py_err`, which maps `HorusError::Timeout` to `HorusTimeoutError`
and `HorusError::Transform` to `HorusTransformError` — both built on
`PyException`, so neither is a subclass of `ValueError`. The recovery path the
docstring told you to write (`except ValueError`) never caught the timeout, and
it escaped the node's tick() on exactly the case the timeout parameter exists
to handle.
"""

import inspect

import pytest

import horus


def _tf():
    return horus.TransformFrame()


def test_wait_for_transform_raises_the_documented_exception():
    tf = _tf()
    with pytest.raises(horus.HorusTimeoutError):
        tf.wait_for_transform("nowhere_src", "nowhere_dst", 0.01)


def test_tf_at_strict_raises_the_documented_exception():
    """Registered frames, a timestamp far outside the buffered range."""
    tf = _tf()
    tf.register_frame("world", None)
    tf.register_frame("sensor", "world")
    at = 4_000_000_000_000
    tf.update_transform("sensor", horus.Transform.from_translation([1.0, 0.0, 0.0]), at)

    # An hour before anything was buffered: strict mode must refuse rather than
    # extrapolate, which is the whole difference from tf_at().
    with pytest.raises(horus.HorusTransformError):
        tf.tf_at_strict("sensor", "world", at - 3_600_000_000_000)


def test_neither_is_a_value_error():
    """Pin the premise: this is why the old docstring was wrong, not a nitpick."""
    assert not issubclass(horus.HorusTimeoutError, ValueError)
    assert not issubclass(horus.HorusTransformError, ValueError)


@pytest.mark.parametrize(
    "method,expected",
    [
        ("wait_for_transform", "HorusTimeoutError"),
        ("tf_at_strict", "HorusTransformError"),
    ],
)
def test_the_docstring_names_that_exception(method, expected):
    doc = inspect.getdoc(getattr(horus.TransformFrame, method)) or ""
    assert expected in doc, f"{method} docstring does not name {expected}:\n{doc}"
    assert "ValueError:" not in doc, (
        f"{method} docstring still promises ValueError, which it does not raise:\n{doc}"
    )
