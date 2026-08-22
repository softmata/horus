//! The generated Python module.
//!
//! Two halves, and the second one is the point.
//!
//! The first is the layout mirror: a `ctypes.Structure` with an explicit
//! `_pack_`-free `_fields_` list, which gives the same `repr(C)` layout the Rust
//! and C++ sides use, plus the layout hash as a class constant. The struct is
//! checked against the size this generator computed for Rust, at import time. A
//! mismatch raises immediately and names the type, rather than producing a
//! shorter or longer read against shared memory.
//!
//! The second is topic access. A layout mirror on its own is a type a Python
//! node can *describe* and not publish: the C++ header gets
//! `<Type>Publisher`/`<Type>Subscriber` classes from [`super::emit_ffi`], and
//! Python got nothing, so a `.hmsg` was usable from two of the three languages
//! HORUS claims. The entry points those C++ classes call are `#[no_mangle]
//! extern "C"` functions in the generated crate, and `ctypes` can call the same
//! ones — so the same publisher/subscriber pair is emitted here, over the same
//! ABI, with the same layout check behind it.
//!
//! # Why the FFI crate is also a cdylib
//!
//! `ctypes.CDLL` loads a shared object. The generated crate was `staticlib` +
//! `rlib`, and neither is loadable at runtime, so [`super::emit_ffi::cargo_toml`]
//! asks for `cdylib` as well. The C++ link still takes the `.a`; Python takes
//! the `.so`/`.dylib`/`.dll` built from the same source in the same `cargo
//! build`.
//!
//! # Why loading is lazy, and why it can build
//!
//! Importing this module must stay free — the cross-language size assertion
//! above runs on import, and a `cargo build` there would make reading a message
//! type cost 40 seconds. So the library is opened on the first
//! `<Type>Publisher(...)` or `<Type>Subscriber(...)`, and if it has not been
//! built yet the module builds it, exactly the way `run_cpp.rs`'s
//! `ensure_generated_msgs_lib` does for the C++ path and into the same
//! `CARGO_TARGET_DIR`. Set `HORUS_MSGS_FFI_BUILD=0` to refuse, or
//! `HORUS_MSGS_FFI=/path/to/lib.so` to point at one directly.

use super::{canonical, emit_ffi, layout, Package, Prim, Type};

fn ctypes_name(p: Prim) -> &'static str {
    match p {
        Prim::U8 => "ctypes.c_uint8",
        Prim::U16 => "ctypes.c_uint16",
        Prim::U32 => "ctypes.c_uint32",
        Prim::U64 => "ctypes.c_uint64",
        Prim::I8 => "ctypes.c_int8",
        Prim::I16 => "ctypes.c_int16",
        Prim::I32 => "ctypes.c_int32",
        Prim::I64 => "ctypes.c_int64",
        Prim::F32 => "ctypes.c_float",
        Prim::F64 => "ctypes.c_double",
        // Rust's bool is one byte; c_bool's size is not guaranteed to be.
        Prim::Bool => "ctypes.c_uint8",
    }
}

fn render(t: &Type) -> String {
    match t {
        Type::Prim(p) => ctypes_name(*p).to_string(),
        Type::Array(elem, n) => format!("{} * {}", render(elem), n),
        Type::Ref(r) => r.short.clone(),
    }
}

/// The loader. One `@STEM@` substitution, no `{}` anywhere, so it can be a
/// plain string replace rather than a `format!` with every Python brace
/// doubled.
///
/// `os`, `sys` and `subprocess` are imported inside the functions that use
/// them: at module scope they would be attributes of the generated module, and
/// a message type named `Os` is fine but a module-level `os` shadowed by
/// anything at all is a confusing failure. Local imports hit `sys.modules` and
/// cost nothing.
const LOADER: &str = r#"
# ─── Topic access ────────────────────────────────────────────────────────────
#
# The classes above are a layout mirror — correct to read, with no way to reach
# a topic. Publishing needs the entry points, and they live in the crate
# `horus msg gen` wrote next to this file: the same `#[no_mangle] extern "C"`
# functions the generated C++ header calls. It is built as a cdylib as well as a
# staticlib so `ctypes` can open it; a `.a` is not loadable at runtime.


class MessageLibraryError(RuntimeError):
    """The generated message entry points could not be loaded."""


class TopicOpenError(RuntimeError):
    """A topic could not be opened for a generated message type.

    The most interesting cause is a layout mismatch: another build of this
    message reordered, renamed or retyped a field, so the bytes on the topic
    are not the bytes this build expects. The C ABI prints the detail, with
    both hashes, to stderr before failing.
    """


#: The generated FFI crate, as an absolute path. This module lives at
#: `.horus/generated/python/msgs.py`, and the crate at `.horus/generated/msgs_ffi`.
_FFI_CRATE = None

#: Library name without the platform prefix or suffix.
_FFI_STEM = "@STEM@"

#: The loaded `ctypes.CDLL`, or None until first use.
_LIB = None


def _ffi_crate():
    import os

    global _FFI_CRATE
    if _FFI_CRATE is None:
        here = os.path.dirname(os.path.abspath(__file__))
        _FFI_CRATE = os.path.join(os.path.dirname(here), "msgs_ffi")
    return _FFI_CRATE


def _ffi_filename():
    """What cargo names the cdylib on this platform."""
    import sys

    if sys.platform == "darwin":
        return "lib" + _FFI_STEM + ".dylib"
    if sys.platform == "win32":
        return _FFI_STEM + ".dll"
    return "lib" + _FFI_STEM + ".so"


def _ffi_candidates():
    """Every built copy of the library, most recently built first.

    Newest rather than a fixed debug/release preference: whichever profile was
    built last is the one the caller just asked for.
    """
    import os

    override = os.environ.get("HORUS_MSGS_FFI")
    if override:
        return [override]

    name = _ffi_filename()
    # Crate-local, and deliberately not $CARGO_TARGET_DIR: `run_cpp.rs` forces
    # this directory when it builds the same crate for the C++ link, so
    # honouring an ambient CARGO_TARGET_DIR here would send the two languages
    # looking in different places for one artifact.
    target = os.path.join(_ffi_crate(), "target")
    roots = [target]
    if os.path.isdir(target):
        # Cross-compiled builds land under target/<triple>/<profile>/.
        roots += [
            os.path.join(target, entry)
            for entry in sorted(os.listdir(target))
            if os.path.isdir(os.path.join(target, entry))
        ]

    found = []
    for root in roots:
        for profile in ("debug", "release"):
            path = os.path.join(root, profile, name)
            if os.path.isfile(path):
                found.append(path)
    found.sort(key=os.path.getmtime, reverse=True)
    return found


def _build_ffi():
    """Build the generated crate, the way the C++ path already does.

    `run_cpp.rs` runs exactly this before a C++ link, into the same
    CARGO_TARGET_DIR, so the two languages share one build rather than each
    keeping a copy. Returns True if cargo succeeded.
    """
    import os
    import subprocess
    import sys

    if os.environ.get("HORUS_MSGS_FFI_BUILD", "").strip().lower() in (
        "0",
        "no",
        "off",
        "never",
    ):
        return False

    crate = _ffi_crate()
    if not os.path.isfile(os.path.join(crate, "Cargo.toml")):
        return False

    sys.stderr.write(
        "horus: building generated message entry points (first use, once)...\n"
    )
    sys.stderr.flush()
    env = dict(os.environ)
    # Forced, not defaulted: `_ffi_candidates` looks only here, and `run_cpp.rs`
    # builds the same crate into the same directory. One build, two languages.
    env["CARGO_TARGET_DIR"] = os.path.join(crate, "target")
    # An inherited RUSTFLAGS or jobserver from whatever invoked this process is
    # not ours to pass on to a different crate's build.
    for stray in ("CARGO_MAKEFLAGS", "CARGO_MANIFEST_DIR", "CARGO_PRIMARY_PACKAGE"):
        env.pop(stray, None)
    try:
        done = subprocess.run(["cargo", "build"], cwd=crate, env=env)
    except OSError:
        return False
    return done.returncode == 0


def _open_first(paths):
    import ctypes as _ct

    for path in paths:
        try:
            return _ct.CDLL(path)
        except OSError:
            continue
    return None


def _ffi():
    """The loaded entry points, building them once if needed."""
    import os

    global _LIB
    if _LIB is not None:
        return _LIB

    lib = _open_first(_ffi_candidates())
    if lib is None and _build_ffi():
        lib = _open_first(_ffi_candidates())
    if lib is None:
        crate = _ffi_crate()
        raise MessageLibraryError(
            "the generated message entry points are not built, so this type "
            "cannot be published or subscribed from Python.\n"
            "Expected " + _ffi_filename() + " under " + os.path.join(crate, "target") + "\n"
            "Build it with:\n"
            "    cargo build --manifest-path " + os.path.join(crate, "Cargo.toml") + "\n"
            "or point HORUS_MSGS_FFI at an existing one. Reading and "
            "constructing the message classes needs none of this."
        )

    _bind(lib)
    _LIB = lib
    return _LIB


def _bind(lib):
    """Declare every entry point's signature.

    ctypes defaults to `int` for a return value, which truncates a 64-bit
    handle to 32 bits and hands back a pointer into nothing. Every symbol is
    declared before it is called.
    """
    import ctypes as _ct

    for cls, sym in _ENTRY_POINTS:
        for kind in ("publisher", "subscriber"):
            new = getattr(lib, sym + "_" + kind + "_new")
            new.argtypes = [_ct.c_char_p]
            new.restype = _ct.c_void_p
            free = getattr(lib, sym + "_" + kind + "_free")
            free.argtypes = [_ct.c_void_p]
            free.restype = None
        send = getattr(lib, sym + "_publisher_send")
        send.argtypes = [_ct.c_void_p, _ct.POINTER(cls)]
        send.restype = None
        recv = getattr(lib, sym + "_subscriber_recv")
        recv.argtypes = [_ct.c_void_p, _ct.POINTER(cls)]
        recv.restype = _ct.c_bool


class _Handle(object):
    """Shared open/close behaviour for the two endpoint classes.

    The C ABI hands back an owned pointer. It is freed exactly once: `close` is
    idempotent, `__exit__` calls it, and `__del__` calls it for code that does
    neither. The free function is held on the instance because module globals
    are torn down before instances are during interpreter shutdown.
    """

    _SYMBOL = None
    _MESSAGE = None
    _KIND = None  # "publisher" or "subscriber"
    _VERB = None  # "send" or "recv"

    def _open(self, topic):
        lib = _ffi()
        self._handle = None
        self._free = getattr(lib, self._SYMBOL + "_" + self._KIND + "_free")
        # The hot call, resolved once: `send` and `recv` run per message, and a
        # getattr on a CDLL per message is a needless dictionary lookup.
        self._call = getattr(lib, self._SYMBOL + "_" + self._KIND + "_" + self._VERB)
        handle = getattr(lib, self._SYMBOL + "_" + self._KIND + "_new")(
            topic.encode("utf-8")
        )
        if not handle:
            raise TopicOpenError(
                "could not open topic %r as a %s for %s. The reason was printed "
                "to stderr by the message library; a layout mismatch there means "
                "another build of this message has different fields — regenerate "
                "with `horus msg gen` and rebuild every language that uses it."
                % (topic, self._KIND, self._MESSAGE.__name__)
            )
        self._handle = handle
        self._topic = topic

    @property
    def topic(self):
        """The topic name this endpoint was opened on."""
        return self._topic

    def close(self):
        """Release the handle. Idempotent, so a double close is safe."""
        handle = getattr(self, "_handle", None)
        if handle is not None:
            self._handle = None
            self._free(handle)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def __repr__(self):
        return "<%s topic=%r>" % (type(self).__name__, getattr(self, "_topic", None))
"#;

pub fn module(pkg: &Package, env: &layout::Env) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "\"\"\"Message types for `{}`.\n\
         \n\
         Generated by `horus msg gen` from `msgs/*.hmsg`. DO NOT EDIT — this file\n\
         is rewritten whenever the definitions change. Edit the `.hmsg` files\n\
         instead.\n\
         \n\
         Each message is a `ctypes.Structure` laid out exactly as the Rust and C++\n\
         definitions are, and comes with a publisher and a subscriber that reach\n\
         the same topics those languages do.\n\
         \n\
         \x20   from msgs import {ex}, {ex}Publisher, {ex}Subscriber\n\
         \n\
         \x20   with {ex}Publisher({topic_arg}) as tx:\n\
         \x20       tx.send({ex}())\n\
         \n\
         \x20   with {ex}Subscriber({topic_arg}) as rx:\n\
         \x20       msg = rx.recv()   # None when nothing is waiting\n\
         \"\"\"\n\n\
         import ctypes\n\n",
        pkg.name,
        ex = pkg
            .messages
            .first()
            .map(|m| m.name.clone())
            .unwrap_or_else(|| "Message".to_string()),
        topic_arg = pkg
            .messages
            .first()
            .and_then(|m| m.topic.as_ref())
            .map(|_| String::new())
            .unwrap_or_else(|| "\"some.topic\"".to_string()),
    ));

    for m in &pkg.messages {
        out.push_str(&format!("class {}(ctypes.Structure):\n", m.name));
        if m.doc.is_empty() {
            out.push_str(&format!(
                "    \"\"\"Generated from {}:{}.\"\"\"\n\n",
                m.src
                    .file_name()
                    .map(|s| s.to_string_lossy().to_string())
                    .unwrap_or_default(),
                m.line
            ));
        } else {
            out.push_str("    \"\"\"");
            out.push_str(&m.doc.join("\n    "));
            out.push_str(&format!(
                "\n\n    Generated from {}:{}.\n    \"\"\"\n\n",
                m.src
                    .file_name()
                    .map(|s| s.to_string_lossy().to_string())
                    .unwrap_or_default(),
                m.line
            ));
        }

        out.push_str("    _fields_ = [\n");
        for f in &m.fields {
            out.push_str(&format!("        (\"{}\", {}),\n", f.name, render(&f.ty)));
        }
        out.push_str("    ]\n\n");

        out.push_str(&format!(
            "    #: FNV-1a of the canonical layout string — the number\n\
             \x20   #: ``horus msg hash {}`` prints.\n\
             \x20   LAYOUT_HASH = {}\n",
            m.name,
            canonical::layout_hash(m)
        ));
        out.push_str(&format!(
            "    LAYOUT_CANONICAL = {:?}\n",
            canonical::canonical_form(m)
        ));
        if let Some(ref topic) = m.topic {
            out.push_str(&format!("    TOPIC = {topic:?}\n"));
        }
        out.push('\n');

        if let Ok(l) = layout::compute(m, env) {
            out.push_str(&format!(
                "# The size Rust and C++ agree on. Checked at import rather than\n\
                 # trusted: a mismatch here means a read against shared memory would\n\
                 # be the wrong length, and failing now names the type.\n\
                 assert ctypes.sizeof({}) == {}, (\n\
                 \x20   f\"{} is {{ctypes.sizeof({})}} bytes in Python but {} in Rust — \"\n\
                 \x20   \"regenerate with `horus msg gen`\"\n\
                 )\n\n",
                m.name, l.size, m.name, m.name, l.size
            ));
        }
    }

    out.push_str(&LOADER.replace(
        "@STEM@",
        &format!("{}_msgs_ffi", pkg.name.replace('-', "_")),
    ));

    // The symbol table `_bind` walks. Emitted after the classes, because it
    // names them, and before the endpoint classes, because they do not.
    out.push_str("\n\n#: (message class, C symbol prefix) for every generated type.\n");
    out.push_str("_ENTRY_POINTS = [\n");
    for m in &pkg.messages {
        out.push_str(&format!(
            "    ({}, {:?}),\n",
            m.name,
            emit_ffi::symbol(pkg, m)
        ));
    }
    out.push_str("]\n\n");

    for m in &pkg.messages {
        let sym = emit_ffi::symbol(pkg, m);
        // A `#[topic = "..."]` becomes the default argument, so the common case
        // is `TelemetryPublisher()` — the same affordance the C++ header gives
        // through `TELEMETRY_TOPIC`.
        let sig = match m.topic {
            Some(_) => format!("self, topic={}.TOPIC", m.name),
            None => "self, topic".to_string(),
        };

        out.push_str(&format!(
            "\nclass {name}Publisher(_Handle):\n\
             \x20   \"\"\"Publish `{name}` on a HORUS topic.\n\
             \n\
             \x20   Opens the topic through the generated C ABI, which uses the\n\
             \x20   layout-checked constructor: if another build of this message\n\
             \x20   reordered a field, the open fails here rather than delivering\n\
             \x20   plausible-looking wrong values.\n\
             \x20   \"\"\"\n\
             \n\
             \x20   _SYMBOL = {sym:?}\n\
             \x20   _MESSAGE = {name}\n\
             \x20   _KIND = \"publisher\"\n\
             \x20   _VERB = \"send\"\n\
             \n\
             \x20   def __init__({sig}):\n\
             \x20       self._open(topic)\n\
             \n\
             \x20   def send(self, msg):\n\
             \x20       \"\"\"Publish one `{name}`.\n\
             \n\
             \x20       The type check is not ceremony: the C ABI takes a raw\n\
             \x20       pointer and reads sizeof({name}) bytes through it, so a\n\
             \x20       differently-shaped ctypes value would be read past its own\n\
             \x20       end or truncated.\n\
             \x20       \"\"\"\n\
             \x20       if not isinstance(msg, {name}):\n\
             \x20           raise TypeError(\n\
             \x20               \"expected {name}, got %s\" % type(msg).__name__\n\
             \x20           )\n\
             \x20       if self._handle is None:\n\
             \x20           raise ValueError(\"publisher is closed\")\n\
             \x20       self._call(self._handle, ctypes.byref(msg))\n\
             \n",
            name = m.name,
            sym = sym,
            sig = sig,
        ));

        out.push_str(&format!(
            "\nclass {name}Subscriber(_Handle):\n\
             \x20   \"\"\"Receive `{name}` from a HORUS topic.\n\
             \n\
             \x20   Single-threaded per handle: the transport's consumer contract\n\
             \x20   is one reader per subscription, the same rule the C++\n\
             \x20   `{name}Subscriber` carries.\n\
             \x20   \"\"\"\n\
             \n\
             \x20   _SYMBOL = {sym:?}\n\
             \x20   _MESSAGE = {name}\n\
             \x20   _KIND = \"subscriber\"\n\
             \x20   _VERB = \"recv\"\n\
             \n\
             \x20   def __init__({sig}):\n\
             \x20       self._open(topic)\n\
             \n\
             \x20   def recv(self):\n\
             \x20       \"\"\"Take one `{name}` if one is waiting, else None.\n\
             \n\
             \x20       Non-blocking, like `try_recv` on the Rust side.\n\
             \x20       \"\"\"\n\
             \x20       if self._handle is None:\n\
             \x20           raise ValueError(\"subscriber is closed\")\n\
             \x20       out = {name}()\n\
             \x20       if self._call(self._handle, ctypes.byref(out)):\n\
             \x20           return out\n\
             \x20       return None\n\
             \n",
            name = m.name,
            sym = sym,
            sig = sig,
        ));
    }

    out.push('\n');
    out.push_str("__all__ = [\n");
    for m in &pkg.messages {
        out.push_str(&format!("    {:?},\n", m.name));
        out.push_str(&format!("    {:?},\n", format!("{}Publisher", m.name)));
        out.push_str(&format!("    {:?},\n", format!("{}Subscriber", m.name)));
    }
    out.push_str("    \"MessageLibraryError\",\n");
    out.push_str("    \"TopicOpenError\",\n");
    out.push_str("]\n");
    out
}
