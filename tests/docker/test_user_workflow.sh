#!/bin/bash
# Full user workflow test — runs inside Docker container.
# Tests the complete journey a real user would take on this OS.

set -e

ERRORS=0
PASSED=0

pass() { PASSED=$((PASSED + 1)); echo "  [PASS] $1"; }
fail() { ERRORS=$((ERRORS + 1)); echo "  [FAIL] $1"; }

# Run a cargo test target and record a pass/fail. On failure the captured
# output is echoed: this matrix exists to catch distro-specific breakage (musl
# link errors on Alpine, glibc symbol mismatches, ...), and the previous
# `2>&1 | tail -3 | grep -q` form threw away exactly the diagnostics that make
# such a failure actionable.
run_cargo_test() {
    local label="$1"; shift
    local out
    if out=$(cargo test "$@" 2>&1); then
        if echo "$out" | grep -q "test result: ok"; then
            pass "$label"
            return
        fi
    fi
    fail "$label"
    echo "----- cargo test output ($label) -----"
    echo "$out" | tail -40
    echo "--------------------------------------"
}

echo "=========================================="
echo "  HORUS User Workflow Test"
echo "  OS: $(cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d= -f2 | tr -d '"' || uname -s)"
echo "  Arch: $(uname -m)"
echo "  Rust: $(rustc --version 2>/dev/null || echo 'not installed')"
echo "=========================================="

# ─── Step 1: Build horus CLI ─────────────────────────────────────────────

echo ""
echo "── Step 1: Build horus CLI ──"
# Keep stderr: a build failure here is the single most informative event in the
# whole distro matrix, and `2>/dev/null` used to discard the rustc/linker error.
if BUILD_OUT=$(cargo build --no-default-features -p horus_manager --release 2>&1); then
    pass "horus CLI builds"
    # Absolute, not "./target/release/horus": step 5 runs `horus new` from
    # inside a temp dir, and a relative path does not survive the cd.
    HORUS="$(pwd)/target/release/horus"
else
    fail "horus CLI build failed"
    echo "----- cargo build output -----"
    echo "$BUILD_OUT" | tail -60
    echo "------------------------------"
    echo "Cannot continue. Exiting."
    exit 1
fi

# ─── Step 2: horus_sys platform tests ────────────────────────────────────

echo ""
echo "── Step 2: horus_sys platform tests ──"
run_cargo_test "horus_sys all tests pass" --no-default-features -p horus_sys -- --test-threads=1

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
run_cargo_test "horus_core SHM platform tests pass" --no-default-features -p horus_core --lib "memory::platform" -- --test-threads=1

# ─── Step 4: Topic IPC ──────────────────────────────────────────────────

echo ""
echo "── Step 4: Topic IPC on this platform ──"

run_cargo_test "Topic backend auto-detection works" --no-default-features -p horus_core --test backend_detection -- --test-threads=1

# ─── Step 5: Create and check project ───────────────────────────────────

echo ""
echo "── Step 5: Project lifecycle ──"

TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

# `horus new` takes a project NAME, not a path — it rejects anything that does
# not start with a letter or underscore ("must start with a letter or
# underscore"), so passing "$TMPDIR/my_robot" always failed. Create it from
# inside the temp dir instead, in a subshell so the trap-based cleanup and the
# checks below still see $TMPDIR.
if (cd "$TMPDIR" && $HORUS new my_robot -r) 2>/dev/null; then
    pass "horus new -r creates project"
elif (cd "$TMPDIR" && $HORUS new my_robot) 2>/dev/null; then
    pass "horus new creates project (default lang)"
else
    fail "horus new failed"
fi

if [ -f "$TMPDIR/my_robot/horus.toml" ]; then
    pass "horus.toml exists in new project"
elif [ -d "$TMPDIR/my_robot" ]; then
    pass "project directory created"
else
    fail "project not created"
fi

# A real assertion: the old form discarded the exit status with `|| true` and
# then recorded an unconditional pass, so `horus check` could panic or not exist
# at all and this step still reported [PASS].
if $HORUS check "$TMPDIR/my_robot"; then
    pass "horus check runs"
else
    fail "horus check failed"
fi

# ─── Step 6: Scheduler + Node lifecycle ─────────────────────────────────

echo ""
echo "── Step 6: Scheduler runtime ──"

run_cargo_test "Scheduler + Topic + Node lifecycle works" --no-default-features -p horus_core --test scheduler_topic_lifecycle -- --test-threads=1

# ─── Step 7: RT scheduling on this platform ─────────────────────────────

echo ""
echo "── Step 7: RT scheduling ──"

run_cargo_test "RT deadline enforcement works" --no-default-features -p horus_core --test rt_deadline_enforcement -- --test-threads=1

# ─── Step 8: Cross-process IPC ──────────────────────────────────────────

echo ""
echo "── Step 8: Cross-process IPC ──"

run_cargo_test "Cross-process SHM IPC works" --no-default-features -p horus_core --test cross_process_ipc -- --test-threads=1

# ─── Step 9: Clean and SHM cleanup ──────────────────────────────────────

echo ""
echo "── Step 9: Cleanup ──"

# Both branches used to record a pass, so a regression that made this exit
# non-zero was reported as green. `horus clean --shm` is expected to succeed
# even when there is nothing to clean.
if $HORUS clean --shm; then
    pass "horus clean --shm works"
else
    fail "horus clean --shm failed"
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
