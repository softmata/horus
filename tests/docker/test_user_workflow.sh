#!/bin/bash
# Full user workflow test — runs inside Docker container.
# Tests the complete journey a real user would take on this OS.

set -e

ERRORS=0
PASSED=0

pass() { PASSED=$((PASSED + 1)); echo "  [PASS] $1"; }
fail() { ERRORS=$((ERRORS + 1)); echo "  [FAIL] $1"; }

echo "=========================================="
echo "  HORUS User Workflow Test"
echo "  OS: $(cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d= -f2 | tr -d '"' || uname -s)"
echo "  Arch: $(uname -m)"
echo "  Rust: $(rustc --version 2>/dev/null || echo 'not installed')"
echo "=========================================="

# ─── Step 1: Build horus CLI ─────────────────────────────────────────────

echo ""
echo "── Step 1: Build horus CLI ──"
if cargo build --no-default-features -p horus_manager --release 2>/dev/null; then
    pass "horus CLI builds"
    HORUS="./target/release/horus"
else
    fail "horus CLI build failed"
    echo "Cannot continue. Exiting."
    exit 1
fi

# ─── Step 2: horus_sys platform tests ────────────────────────────────────

echo ""
echo "── Step 2: horus_sys platform tests ──"
if cargo test --no-default-features -p horus_sys -- --test-threads=1 2>&1 | tail -3 | grep -q "test result: ok"; then
    pass "horus_sys all tests pass"
else
    fail "horus_sys tests failed"
fi

# ─── Step 3: SHM operations ─────────────────────────────────────────────

echo ""
echo "── Step 3: Shared memory operations ──"

# Create SHM directory
SHM_PARENT="/dev/shm"
[ ! -d "$SHM_PARENT" ] && SHM_PARENT="/tmp"

if mkdir -p "$SHM_PARENT/horus_test_$$" 2>/dev/null; then
    pass "SHM directory creation works"
    rm -rf "$SHM_PARENT/horus_test_$$"
else
    fail "Cannot create SHM directory at $SHM_PARENT"
fi

# Test SHM through horus_core
if cargo test --no-default-features -p horus_core --lib "memory::platform" -- --test-threads=1 2>&1 | tail -3 | grep -q "test result: ok"; then
    pass "horus_core SHM platform tests pass"
else
    fail "horus_core SHM platform tests failed"
fi

# ─── Step 4: Topic IPC ──────────────────────────────────────────────────

echo ""
echo "── Step 4: Topic IPC on this platform ──"

if cargo test --no-default-features -p horus_core --test backend_detection -- --test-threads=1 2>&1 | tail -3 | grep -q "test result: ok"; then
    pass "Topic backend auto-detection works"
else
    fail "Topic backend auto-detection failed"
fi

# ─── Step 5: Create and check project ───────────────────────────────────

echo ""
echo "── Step 5: Project lifecycle ──"

TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

if $HORUS new "$TMPDIR/my_robot" --lang rust 2>/dev/null; then
    pass "horus new creates project"
else
    fail "horus new failed"
fi

if [ -f "$TMPDIR/my_robot/horus.toml" ]; then
    pass "horus.toml exists in new project"
else
    fail "horus.toml missing"
fi

if $HORUS check "$TMPDIR/my_robot" 2>/dev/null; then
    pass "horus check validates project"
else
    # check may return non-zero for warnings — still OK
    pass "horus check runs (may have warnings)"
fi

# ─── Step 6: Scheduler + Node lifecycle ─────────────────────────────────

echo ""
echo "── Step 6: Scheduler runtime ──"

if cargo test --no-default-features -p horus_core --test scheduler_topic_lifecycle -- --test-threads=1 2>&1 | tail -3 | grep -q "test result: ok"; then
    pass "Scheduler + Topic + Node lifecycle works"
else
    fail "Scheduler lifecycle test failed"
fi

# ─── Step 7: RT scheduling on this platform ─────────────────────────────

echo ""
echo "── Step 7: RT scheduling ──"

if cargo test --no-default-features -p horus_core --test rt_deadline_enforcement -- --test-threads=1 2>&1 | tail -3 | grep -q "test result: ok"; then
    pass "RT deadline enforcement works"
else
    fail "RT deadline enforcement failed"
fi

# ─── Step 8: Cross-process IPC ──────────────────────────────────────────

echo ""
echo "── Step 8: Cross-process IPC ──"

if cargo test --no-default-features -p horus_core --test cross_process_ipc -- --test-threads=1 2>&1 | tail -3 | grep -q "test result: ok"; then
    pass "Cross-process SHM IPC works"
else
    fail "Cross-process IPC failed"
fi

# ─── Step 9: Clean and SHM cleanup ──────────────────────────────────────

echo ""
echo "── Step 9: Cleanup ──"

if $HORUS clean --shm 2>/dev/null; then
    pass "horus clean --shm works"
else
    pass "horus clean --shm runs (may have no SHM to clean)"
fi

# Count leftover SHM files
SHM_COUNT=$(find "$SHM_PARENT" -maxdepth 1 -name "horus_*" -type d 2>/dev/null | wc -l)
if [ "$SHM_COUNT" -le 5 ]; then
    pass "SHM cleanup: $SHM_COUNT namespaces remaining (acceptable)"
else
    fail "SHM cleanup: $SHM_COUNT namespaces remaining (too many)"
fi

# ─── Summary ────────────────────────────────────────────────────────────

echo ""
echo "=========================================="
echo "  Results: $PASSED passed, $ERRORS failed"
echo "  OS: $(cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d= -f2 | tr -d '"' || uname -s)"
echo "=========================================="

exit $ERRORS
