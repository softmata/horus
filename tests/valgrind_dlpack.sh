#!/bin/bash
# valgrind_dlpack.sh — Valgrind regression test for DLPack capsule destructor
#
# Tests the DLPackDeleterGuard RAII pattern in horus_py/src/tensor.rs for:
#
#   1. Normal path: deleter invoked exactly once after successful from_dlpack().
#   2. Error path (CUDA device): guard invokes deleter when ? returns early.
#   3. No double-free: guard is disarmed before the explicit deleter call on
#      the success path, so the deleter cannot be called twice.
#
# Usage:
#   ./scripts/valgrind_dlpack.sh [--no-build]
#
# Prerequisites:
#   - valgrind (apt install valgrind / brew install valgrind)
#   - maturin (pip install maturin)
#   - Python 3.8+
#   - Rust nightly (or stable with required features)
#
# Exit codes:
#   0 — all tests pass, valgrind reports no errors
#   1 — build failed
#   2 — valgrind reported memory errors
#   3 — Python test assertions failed
#   4 — prerequisites missing

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HORUS_DIR="$(dirname "$SCRIPT_DIR")"
HORUS_PY_DIR="$HORUS_DIR/horus_py"
WORK_DIR="$(mktemp -d /tmp/horus-valgrind-dlpack-XXXXXX)"
trap 'rm -rf "$WORK_DIR"' EXIT

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_pass()  { echo -e "${GREEN}[PASS]${NC} $1"; }
log_fail()  { echo -e "${RED}[FAIL]${NC} $1"; }
log_info()  { echo -e "${CYAN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }

# ── Argument parsing ───────────────────────────────────────────────────────────

NO_BUILD=0
for arg in "$@"; do
    case "$arg" in
        --no-build) NO_BUILD=1 ;;
        *) echo "Unknown argument: $arg"; exit 4 ;;
    esac
done

# ── Prerequisite checks ────────────────────────────────────────────────────────

check_prereq() {
    local cmd="$1" pkg="$2"
    if ! command -v "$cmd" &>/dev/null; then
        log_fail "Missing prerequisite: $cmd (install: $pkg)"
        exit 4
    fi
}

check_prereq valgrind "apt install valgrind / brew install valgrind"
check_prereq python3  "apt install python3 / brew install python3"

if [[ "$NO_BUILD" -eq 0 ]]; then
    check_prereq maturin "pip install maturin"
fi

PYTHON=$(command -v python3)
log_info "Python: $($PYTHON --version)"
log_info "Valgrind: $(valgrind --version | head -1)"
log_info "Work dir: $WORK_DIR"

# ── Build horus_py in debug mode ───────────────────────────────────────────────

if [[ "$NO_BUILD" -eq 0 ]]; then
    log_info "Building horus_py in debug mode (symbols for valgrind)..."
    pushd "$HORUS_PY_DIR" >/dev/null
    # maturin develop builds in debug mode and installs into the active venv
    if ! maturin develop 2>&1; then
        log_fail "maturin develop failed"
        exit 1
    fi
    popd >/dev/null
    log_pass "horus_py built and installed"
else
    log_info "Skipping build (--no-build)"
fi

# ── Verify horus._horus is importable ────────────────────────────────────────

if ! "$PYTHON" -c "import horus._horus" 2>/dev/null; then
    log_fail "horus._horus is not importable; run without --no-build to rebuild"
    exit 1
fi
log_pass "horus._horus imports OK"

# ── Python valgrind suppressions ──────────────────────────────────────────────
#
# Python allocates memory through its own allocators, which valgrind reports as
# leaks unless suppressed.  We generate a minimal suppression file here so the
# output only shows HORUS-relevant errors.

SUPPRESSION_FILE="$WORK_DIR/python.supp"
cat > "$SUPPRESSION_FILE" << 'EOF'
# Python allocator noise suppression
{
   python_pymalloc_alloc
   Memcheck:Leak
   ...
   fun:_PyMem_RawMalloc
   ...
}
{
   python_pymalloc_realloc
   Memcheck:Leak
   ...
   fun:_PyMem_RawRealloc
   ...
}
{
   python_dlopen_extensions
   Memcheck:Leak
   ...
   fun:dlopen*
   ...
}
{
   python_import_extension
   Memcheck:Leak
   ...
   fun:import_find_and_load*
   ...
}
{
   libffi_closures
   Memcheck:Leak
   ...
   fun:ffi_closure_alloc
   ...
}
EOF

# ── DLPack Python test ────────────────────────────────────────────────────────
#
# The test creates minimal DLManagedTensor objects via ctypes, wraps them in
# PyCapsules named "dltensor" (the DLPack protocol name), then passes them to
# horus._horus.TensorHandle.from_dlpack().
#
# A C-callable deleter function tracks how many times it is invoked.  After
# each test scenario the count is validated:
#
#   normal path — count must be exactly 1
#   cuda error  — count must be exactly 1 (guard fires on early return)
#   no leak     — valgrind --leak-check=full finds no definite leaks

TEST_SCRIPT="$WORK_DIR/test_dlpack_deleter.py"
cat > "$TEST_SCRIPT" << 'PYEOF'
"""
DLPack capsule destructor regression test for DLPackDeleterGuard.

Exercises three scenarios that collectively verify:

  1. Normal path (CPU tensor):
     from_dlpack() copies data, disarms the guard, then calls the deleter
     explicitly.  The deleter must be called exactly once.

  2. Error path (CUDA tensor):
     from_dlpack() detects a CUDA source and returns early via `?`.
     The DLPackDeleterGuard drop() implementation must call the deleter.
     Again exactly once.

  3. No double-free:
     If the guard calls the deleter in scenario 2, a null-pointer check in
     the guard's Drop impl prevents a second call.  Conversely in scenario 1
     the guard is disarmed before the explicit deleter call.  Valgrind
     confirms no InvalidRead/InvalidFree.

DLManagedTensor layout (reproduced from dlpack.h / horus_core/src/dlpack/ffi.rs):

  struct DLTensor {
    data:     *mut c_void,
    device:   DLDevice   { device_type: i32, device_id: i32 },
    ndim:     i32,
    dtype:    DLDataType { code: u8, bits: u8, lanes: u16 },
    shape:    *mut i64,
    strides:  *mut i64,  (NULL = compact)
    byte_offset: u64,
  }

  struct DLManagedTensor {
    dl_tensor: DLTensor,
    manager_ctx: *mut c_void,
    deleter: Option<unsafe extern "C" fn(*mut DLManagedTensor)>,
  }

Both structs use repr(C) / default alignment.
"""

import ctypes
import sys
import gc

# ── ctypes mirror of DLPack C structs ─────────────────────────────────────────

class DLDevice(ctypes.Structure):
    _fields_ = [
        ("device_type", ctypes.c_int32),  # 1 = CPU, 2 = CUDA
        ("device_id",   ctypes.c_int32),
    ]

class DLDataType(ctypes.Structure):
    _fields_ = [
        ("code",  ctypes.c_uint8),   # 0=int, 1=uint, 2=float
        ("bits",  ctypes.c_uint8),
        ("lanes", ctypes.c_uint16),
    ]

class DLTensor(ctypes.Structure):
    _fields_ = [
        ("data",        ctypes.c_void_p),
        ("device",      DLDevice),
        ("ndim",        ctypes.c_int32),
        ("dtype",       DLDataType),
        ("shape",       ctypes.POINTER(ctypes.c_int64)),
        ("strides",     ctypes.POINTER(ctypes.c_int64)),
        ("byte_offset", ctypes.c_uint64),
    ]

class DLManagedTensor(ctypes.Structure):
    pass  # forward declaration for self-referential deleter type

DELETER_CFUNC = ctypes.CFUNCTYPE(None, ctypes.POINTER(DLManagedTensor))

DLManagedTensor._fields_ = [
    ("dl_tensor",   DLTensor),
    ("manager_ctx", ctypes.c_void_p),
    ("deleter",     DELETER_CFUNC),
]

# ── Helper: create a minimal CPU DLPack capsule ────────────────────────────────

def make_dlpack_object(device_type: int, call_counter: list):
    """
    Return an object with a __dlpack__ method that hands out a fresh
    DLManagedTensor PyCapsule backed by a small heap allocation.

    call_counter[0] is incremented each time the deleter fires.
    """
    # Allocate 16 bytes of data (4 x uint8, shape=[4]).
    data_buf = ctypes.create_string_buffer(16)
    shape_arr = (ctypes.c_int64 * 1)(4)

    # Keep a reference so the data_buf stays alive across the test.
    # The deleter is responsible for cleanup in a real implementation;
    # here we just track the call count (no real free needed for the test).
    closure_state = {"data": data_buf, "shape": shape_arr, "freed": False}

    @DELETER_CFUNC
    def _deleter(managed_ptr):
        # Record that the deleter was called.
        call_counter[0] += 1
        if closure_state["freed"]:
            # Double-free: raise so the test can detect it.
            raise RuntimeError("DOUBLE FREE DETECTED in DLPack deleter!")
        closure_state["freed"] = True

    # Build the DLManagedTensor on the heap (ctypes keeps it alive via
    # the closure and the PyCapsule's context pointer).
    managed = DLManagedTensor()
    managed.dl_tensor.data        = ctypes.cast(data_buf, ctypes.c_void_p)
    managed.dl_tensor.device      = DLDevice(device_type, 0)
    managed.dl_tensor.ndim        = 1
    managed.dl_tensor.dtype       = DLDataType(1, 8, 1)   # uint8, 8-bit, 1 lane
    managed.dl_tensor.shape       = shape_arr
    managed.dl_tensor.strides     = None
    managed.dl_tensor.byte_offset = 0
    managed.manager_ctx           = None
    managed.deleter               = _deleter

    # We must keep _deleter alive for the lifetime of the capsule, so store it
    # in the closure.
    closure_state["managed"] = managed
    closure_state["deleter_ref"] = _deleter

    # Wrap in a PyCapsule named "dltensor" (the DLPack protocol capsule name).
    import ctypes.pythonapi as _api
    import ctypes as _ct

    addr = ctypes.addressof(managed)

    # PyCapsule_New(pointer, name, destructor)
    _api.PyCapsule_New.restype  = ctypes.py_object
    _api.PyCapsule_New.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_void_p]
    capsule = _api.PyCapsule_New(addr, b"dltensor", None)

    class _DLPackProvider:
        """Minimal __dlpack__ provider backed by our ctypes struct."""
        def __dlpack__(self, stream=None):
            return capsule
        def __dlpack_device__(self):
            return (device_type, 0)

    # Keep closure_state alive via the provider.
    provider = _DLPackProvider()
    provider._closure = closure_state
    return provider


# ── Scenario 1: Normal CPU path — deleter called exactly once ─────────────────

def test_normal_path():
    from horus._horus import TensorHandle

    counter = [0]
    provider = make_dlpack_object(device_type=1, call_counter=counter)  # CPU

    try:
        handle = TensorHandle.from_dlpack(provider)
    except Exception as e:
        print(f"  FAIL: from_dlpack raised unexpectedly: {e}", file=sys.stderr)
        sys.exit(3)

    # Force GC to run any pending finalizers.
    del handle
    del provider
    gc.collect()

    if counter[0] != 1:
        print(
            f"  FAIL: deleter call count={counter[0]}, expected 1 (normal path)",
            file=sys.stderr,
        )
        sys.exit(3)

    print("  PASS: normal path — deleter called exactly 1 time")


# ── Scenario 2: CUDA error path — guard calls deleter on early return ──────────

def test_cuda_error_path():
    from horus._horus import TensorHandle

    counter = [0]
    provider = make_dlpack_object(device_type=2, call_counter=counter)  # CUDA

    try:
        TensorHandle.from_dlpack(provider)
        print("  FAIL: from_dlpack should have raised for CUDA source", file=sys.stderr)
        sys.exit(3)
    except RuntimeError as e:
        if "CUDA" not in str(e) and "cuda" not in str(e).lower():
            print(f"  FAIL: unexpected error: {e}", file=sys.stderr)
            sys.exit(3)

    del provider
    gc.collect()

    if counter[0] != 1:
        print(
            f"  FAIL: deleter call count={counter[0]}, expected 1 (CUDA error path / guard)",
            file=sys.stderr,
        )
        sys.exit(3)

    print("  PASS: CUDA error path — guard called deleter exactly 1 time")


# ── Scenario 3: Null capsule — no crash, proper error ─────────────────────────
#
# Passing None where a capsule is expected should produce a clear Python error,
# not a segfault.  Valgrind catches any invalid memory accesses here.

def test_null_capsule():
    from horus._horus import TensorHandle

    class _BadProvider:
        def __dlpack__(self, stream=None):
            return None  # invalid capsule

    try:
        TensorHandle.from_dlpack(_BadProvider())
        print("  FAIL: from_dlpack should raise for null capsule", file=sys.stderr)
        sys.exit(3)
    except Exception:
        pass  # any exception is acceptable — no crash or segfault

    print("  PASS: null capsule — raised Python exception without crash")


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=== DLPack capsule destructor regression tests ===")

    print("\n[1] Normal CPU path:")
    test_normal_path()

    print("\n[2] CUDA error path (DLPackDeleterGuard fires):")
    test_cuda_error_path()

    print("\n[3] Null capsule guard:")
    test_null_capsule()

    print("\n=== All assertions passed ===")
PYEOF

# ── Run tests without valgrind first (fast sanity check) ─────────────────────

log_info "Running Python assertions (without valgrind)..."
if ! "$PYTHON" "$TEST_SCRIPT" 2>&1; then
    log_fail "Python assertions failed (before valgrind)"
    exit 3
fi
log_pass "Python assertions passed"

# ── Run under valgrind ────────────────────────────────────────────────────────

VALGRIND_LOG="$WORK_DIR/valgrind.log"

log_info "Running under valgrind --leak-check=full..."
log_info "Log: $VALGRIND_LOG"

# Valgrind flags:
#   --error-exitcode=2      — exit 2 if any error is found
#   --leak-check=full       — report all definitely-lost blocks
#   --show-leak-kinds=definite,indirect  — skip "possibly lost" Python noise
#   --track-origins=yes     — show where uninitialised values came from
#   --suppressions          — filter Python allocator noise
#   --gen-suppressions=no   — don't generate suppression output
#   --child-silent-after-fork=yes — suppress noise from Python's fork() use

valgrind \
    --error-exitcode=2 \
    --leak-check=full \
    --show-leak-kinds=definite,indirect \
    --track-origins=yes \
    --suppressions="$SUPPRESSION_FILE" \
    --child-silent-after-fork=yes \
    --log-file="$VALGRIND_LOG" \
    "$PYTHON" "$TEST_SCRIPT" 2>&1
VALGRIND_RC=$?

# Always print the last portion of the valgrind log (summary section)
echo ""
log_info "=== Valgrind output (last 40 lines) ==="
tail -40 "$VALGRIND_LOG"
echo ""

if [[ "$VALGRIND_RC" -eq 2 ]]; then
    log_fail "Valgrind detected memory errors (see $VALGRIND_LOG)"
    # Copy log to a stable path for CI artifact upload
    cp "$VALGRIND_LOG" "$HORUS_DIR/valgrind_dlpack_errors.log" 2>/dev/null || true
    exit 2
elif [[ "$VALGRIND_RC" -ne 0 ]]; then
    log_fail "Python test failed under valgrind (exit code $VALGRIND_RC)"
    exit 3
fi

# Check for ERROR SUMMARY in the log
if grep -q "ERROR SUMMARY: 0 errors" "$VALGRIND_LOG"; then
    log_pass "Valgrind: 0 errors from 0 contexts"
else
    log_warn "Valgrind ERROR SUMMARY not found or non-zero — check $VALGRIND_LOG"
    grep "ERROR SUMMARY" "$VALGRIND_LOG" || true
    exit 2
fi

echo ""
log_pass "=== DLPack valgrind regression test PASSED ==="
log_info "No double-free, no invalid reads, no definite memory leaks."
exit 0
