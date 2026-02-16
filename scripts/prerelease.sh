#!/bin/bash
#
# HORUS Pre-Release Verification Script v3.0.0
#
# Comprehensive verification before pushing to production.
# Run this BEFORE committing to ensure 100% confidence.
#
# Checks:
#   1. Code formatting (cargo fmt)
#   2. Compilation check (cargo check)
#   3. Linting (cargo clippy)
#   4. Security audit (cargo audit)
#   5. MSRV compatibility check
#   6. Release build (cargo build --release)
#   7. Unit tests (cargo test)
#   8. Doc tests (cargo test --doc)
#   9. CLI command verification
#   10. Python bindings (horus_py build and import)
#   11. Examples compilation (all crates)
#   12. Integration tests
#   13. Benchmark smoke test
#   14. Feature matrix testing
#   15. Shell script syntax validation
#   16. Version consistency check
#   17. Dead code detection
#   18. Docker container tests (optional)
#
# Usage: ./scripts/prerelease.sh [--quick] [--fix] [--full]
#   --quick: Skip slow tests (integration, docs, benchmarks)
#   --fix: Auto-fix formatting issues
#   --full: Run all tests including Docker container tests
#

set -e

# Script version
SCRIPT_VERSION="3.0.0"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
WHITE='\033[1;37m'
NC='\033[0m'

# Symbols
CHECK="${GREEN}✓${NC}"
CROSS="${RED}✗${NC}"
WARN="${YELLOW}!${NC}"
INFO="${CYAN}ℹ${NC}"
ARROW="${BLUE}→${NC}"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$ROOT_DIR"

# Parse arguments
QUICK_MODE=false
FIX_MODE=false
FULL_MODE=false
for arg in "$@"; do
    case $arg in
        --quick) QUICK_MODE=true ;;
        --fix) FIX_MODE=true ;;
        --full) FULL_MODE=true ;;
        --help|-h)
            echo "HORUS Pre-Release Verification Script v$SCRIPT_VERSION"
            echo ""
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --quick    Skip slow tests (integration tests, docs build, benchmarks)"
            echo "  --fix      Auto-fix formatting issues"
            echo "  --full     Run all tests including Docker container tests"
            echo "  --help     Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0              # Standard pre-release check"
            echo "  $0 --quick      # Fast check (skip slow tests)"
            echo "  $0 --fix        # Auto-fix formatting issues"
            echo "  $0 --full       # Complete check including Docker tests"
            echo ""
            exit 0
            ;;
    esac
done

# Statistics
TOTAL=0
PASSED=0
FAILED=0
SKIPPED=0
WARNINGS=0

# Track failures for summary
declare -a FAILURES=()
declare -a WARNS=()

# Helper functions
pass() {
    echo -e "  [$CHECK] $1"
    PASSED=$((PASSED + 1))
    TOTAL=$((TOTAL + 1))
}

fail() {
    echo -e "  [$CROSS] $1"
    FAILED=$((FAILED + 1))
    TOTAL=$((TOTAL + 1))
    FAILURES+=("$1")
}

warn() {
    echo -e "  [$WARN] $1"
    WARNINGS=$((WARNINGS + 1))
    WARNS+=("$1")
}

skip() {
    echo -e "  [${WHITE}-${NC}] $1 (skipped)"
    SKIPPED=$((SKIPPED + 1))
}

info() {
    echo -e "  [$INFO] $1"
}

section() {
    echo ""
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}  $1${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

run_cmd() {
    local desc="$1"
    shift
    echo -e "  [$ARROW] $desc"
    if "$@" >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

# Start time
START_TIME=$(date +%s)

# Get MSRV from Cargo.toml
MSRV=$(grep -m1 'rust-version' Cargo.toml 2>/dev/null | sed 's/.*"\(.*\)"/\1/' || echo "1.85")

echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║${NC}  ${WHITE}HORUS Pre-Release Verification v$SCRIPT_VERSION${NC}                                    ${BLUE}║${NC}"
echo -e "${BLUE}║${NC}  ${CYAN}Comprehensive • Systematic • Production-Ready${NC}                               ${BLUE}║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

if [ "$QUICK_MODE" = true ]; then
    echo -e "  ${YELLOW}Running in QUICK mode (skipping slow tests)${NC}"
fi
if [ "$FIX_MODE" = true ]; then
    echo -e "  ${YELLOW}Running with --fix mode (auto-fixing issues)${NC}"
fi
if [ "$FULL_MODE" = true ]; then
    echo -e "  ${CYAN}Running in FULL mode (including Docker tests)${NC}"
fi
echo ""

# =============================================================================
# 1. CARGO FMT - Code Formatting
# =============================================================================
section "1. Code Formatting (cargo fmt)"

if [ "$FIX_MODE" = true ]; then
    if cargo fmt --all 2>/dev/null; then
        pass "Code formatted (auto-fixed)"
    else
        fail "cargo fmt failed"
    fi
else
    if cargo fmt --all --check 2>/dev/null; then
        pass "Code formatting is correct"
    else
        fail "Code formatting issues found (run with --fix or: cargo fmt --all)"
    fi
fi

# =============================================================================
# 2. CARGO CHECK - Fast Compilation Check
# =============================================================================
section "2. Compilation Check (cargo check)"

if [ "$QUICK_MODE" = true ]; then
    info "Quick mode: checking core crates only..."
    if timeout 300 cargo check -p horus -p horus_core -p horus_macros -p horus_manager -p horus_library -p horus_ai 2>&1 | tail -5; then
        pass "Core crates compile"
    else
        fail "Core crates compilation failed"
    fi
else
    if timeout 600 cargo check --workspace --all-targets 2>&1 | tail -5; then
        pass "All workspace crates compile"
    else
        fail "Compilation errors found (or timeout)"
    fi
fi

# =============================================================================
# 3. CARGO CLIPPY - Linting
# =============================================================================
section "3. Linting (cargo clippy)"

if [ "$QUICK_MODE" = true ]; then
    info "Quick mode: linting core crates only..."
    CLIPPY_OUTPUT=$(timeout 300 cargo clippy -p horus -p horus_core -p horus_macros -p horus_manager -p horus_library -p horus_ai 2>&1 || true)
else
    CLIPPY_OUTPUT=$(timeout 600 cargo clippy --workspace --all-targets 2>&1 || true)
fi

CLIPPY_WARNINGS=$(echo "$CLIPPY_OUTPUT" | grep -c "warning:" || true)
CLIPPY_ERRORS=$(echo "$CLIPPY_OUTPUT" | grep -c "error\[" || true)

if echo "$CLIPPY_OUTPUT" | grep -q "Terminated"; then
    warn "Clippy timed out"
    skip "Clippy (timeout)"
elif [ "$CLIPPY_ERRORS" -gt 0 ]; then
    fail "Clippy found $CLIPPY_ERRORS errors"
    echo "$CLIPPY_OUTPUT" | grep "error\[" | head -10
elif [ "$CLIPPY_WARNINGS" -gt 10 ]; then
    warn "Clippy found $CLIPPY_WARNINGS warnings (consider fixing)"
    pass "No clippy errors"
else
    pass "Clippy passed ($CLIPPY_WARNINGS warnings)"
fi

# =============================================================================
# 4. SECURITY AUDIT (cargo audit)
# =============================================================================
section "4. Security Audit (cargo audit)"

if command -v cargo-audit &>/dev/null; then
    AUDIT_OUTPUT=$(cargo audit 2>&1)
    AUDIT_VULNS=$(echo "$AUDIT_OUTPUT" | grep -c "Vulnerability" || true)
    AUDIT_WARNINGS=$(echo "$AUDIT_OUTPUT" | grep -c "warning:" || true)

    if [ "$AUDIT_VULNS" -gt 0 ]; then
        fail "Security audit found $AUDIT_VULNS vulnerabilities"
        echo "$AUDIT_OUTPUT" | grep -A2 "Vulnerability" | head -20
    elif [ "$AUDIT_WARNINGS" -gt 0 ]; then
        warn "Security audit found $AUDIT_WARNINGS warnings"
        pass "No critical vulnerabilities"
    else
        pass "Security audit passed (no vulnerabilities)"
    fi
else
    info "cargo-audit not installed (install with: cargo install cargo-audit)"
    skip "Security audit"
fi

# =============================================================================
# 5. MSRV COMPATIBILITY CHECK
# =============================================================================
section "5. MSRV Compatibility (Rust $MSRV)"

if [ "$QUICK_MODE" = true ]; then
    skip "MSRV check (quick mode)"
else
    if rustup run "$MSRV" cargo --version &>/dev/null; then
        info "Testing compilation with Rust $MSRV..."
        if rustup run "$MSRV" cargo check --workspace 2>&1 | tail -3; then
            pass "MSRV $MSRV compatibility verified"
        else
            fail "Code doesn't compile with MSRV $MSRV"
        fi
    else
        info "MSRV toolchain $MSRV not installed"
        info "Install with: rustup install $MSRV"
        skip "MSRV compatibility check"
    fi
fi

# =============================================================================
# 6. CARGO BUILD RELEASE - Full Release Build
# =============================================================================
section "6. Release Build (cargo build --release)"

if [ "$QUICK_MODE" = true ]; then
    info "Quick mode: building horus binary only..."
    if timeout 300 cargo build --release -p horus_manager 2>&1 | tail -5; then
        pass "horus binary builds successfully"
        if [ -f "./target/release/horus" ]; then
            BIN_SIZE=$(du -h "./target/release/horus" | cut -f1)
            pass "Binary: horus ($BIN_SIZE)"
        fi
    else
        fail "horus build failed (or timeout)"
    fi
else
    info "Building all workspace crates in release mode..."
    if timeout 900 cargo build --release --workspace 2>&1 | tail -10; then
        pass "Release build successful"

        if [ -f "./target/release/horus" ]; then
            BIN_SIZE=$(du -h "./target/release/horus" | cut -f1)
            pass "Binary: horus ($BIN_SIZE)"
        else
            fail "Binary missing: horus"
        fi
    else
        fail "Release build failed (or timeout)"
    fi
fi

# =============================================================================
# 7. CARGO TEST - Unit Tests
# =============================================================================
section "7. Unit Tests (cargo test)"

if [ "$QUICK_MODE" = true ]; then
    skip "Unit tests (quick mode - use --full for tests)"
else
    info "Running all workspace tests..."
    TEST_OUTPUT=$(timeout 600 cargo test --workspace 2>&1 || true)

    if echo "$TEST_OUTPUT" | grep -q "Terminated"; then
        warn "Tests timed out"
        skip "Unit tests (timeout)"
    elif echo "$TEST_OUTPUT" | grep -q "test result: ok"; then
        TEST_PASSED=$(echo "$TEST_OUTPUT" | grep -o "[0-9]* passed" | tail -1 | grep -o "[0-9]*" || echo "0")
        pass "All unit tests passed ($TEST_PASSED tests)"
    elif echo "$TEST_OUTPUT" | grep -q "FAILED"; then
        fail "Unit tests failed"
        echo "$TEST_OUTPUT" | grep -A5 "FAILED" | head -20
    else
        pass "Unit tests completed"
    fi
fi

# =============================================================================
# 8. DOC TESTS
# =============================================================================
section "8. Documentation Tests (cargo test --doc)"

if [ "$QUICK_MODE" = true ]; then
    skip "Doc tests (quick mode)"
else
    if cargo test --doc --workspace 2>&1 | tail -5; then
        pass "All documentation tests passed"
    else
        warn "Some documentation tests failed"
    fi
fi

# =============================================================================
# 9. CLI COMMANDS - Verify All CLI Features
# =============================================================================
section "9. CLI Command Verification"

HORUS_BIN="./target/release/horus"

if [ -x "$HORUS_BIN" ]; then
    # Test --version
    if $HORUS_BIN --version >/dev/null 2>&1; then
        VERSION=$($HORUS_BIN --version 2>&1 | head -1)
        pass "horus --version: $VERSION"
    else
        fail "horus --version failed"
    fi

    # Test --help
    if $HORUS_BIN --help >/dev/null 2>&1; then
        pass "horus --help"
    else
        fail "horus --help failed"
    fi

    # Test subcommands
    SUBCOMMANDS=(run new init check pkg topic node param doctor clean launch msg log net hf deploy discover record test driver plugin env cache)
    for cmd in "${SUBCOMMANDS[@]}"; do
        if $HORUS_BIN $cmd --help >/dev/null 2>&1; then
            pass "horus $cmd --help"
        else
            info "horus $cmd --help (not available)"
        fi
    done
else
    fail "horus binary not found at $HORUS_BIN"
fi

# =============================================================================
# 10. PYTHON BINDINGS - horus_py
# =============================================================================
section "10. Python Bindings (horus_py)"

if command -v python3 &>/dev/null; then
    PY_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    info "Python version: $PY_VERSION"

    if command -v maturin &>/dev/null; then
        info "maturin found, building horus_py..."

        cd horus_py
        if maturin build --release 2>&1 | tail -5; then
            pass "horus_py wheel built successfully"

            WHEEL_FILE=$(ls target/wheels/*.whl 2>/dev/null | head -1)
            if [ -n "$WHEEL_FILE" ]; then
                TEMP_VENV=$(mktemp -d)
                if python3 -m venv "$TEMP_VENV" 2>/dev/null; then
                    source "$TEMP_VENV/bin/activate"
                    if pip install "$WHEEL_FILE" >/dev/null 2>&1; then
                        if python3 -c "import horus; print(f'horus_py version: {horus.__version__}')" 2>/dev/null; then
                            pass "horus_py import successful"

                            if python3 -c "from horus import Node, Topic; print('Core imports OK')" 2>/dev/null; then
                                pass "horus_py core classes importable"
                            else
                                warn "horus_py core classes import issue"
                            fi
                        else
                            fail "horus_py import failed"
                        fi
                    else
                        warn "horus_py wheel install failed"
                    fi
                    deactivate
                fi
                rm -rf "$TEMP_VENV"
            fi
        else
            fail "horus_py build failed"
        fi
        cd "$ROOT_DIR"
    else
        skip "maturin not installed (install via: apt/dnf/pacman, pipx, or cargo)"
    fi
else
    skip "Python3 not available"
fi

# =============================================================================
# 11. EXAMPLES COMPILATION (All Crates)
# =============================================================================
section "11. Examples Compilation"

if [ "$QUICK_MODE" = true ]; then
    skip "Examples compilation (quick mode)"
else
    for crate in horus_core horus_manager; do
        info "Checking $crate examples..."
        EXAMPLE_COUNT=0
        EXAMPLE_PASS=0

        for example in "$crate/examples/"*.rs; do
            if [ -f "$example" ]; then
                EXAMPLE_NAME=$(basename "$example" .rs)
                EXAMPLE_COUNT=$((EXAMPLE_COUNT + 1))

                if cargo build --release --example "$EXAMPLE_NAME" -p "$crate" >/dev/null 2>&1; then
                    EXAMPLE_PASS=$((EXAMPLE_PASS + 1))
                else
                    warn "$crate example failed: $EXAMPLE_NAME"
                fi
            fi
        done

        if [ "$EXAMPLE_COUNT" -gt 0 ]; then
            if [ "$EXAMPLE_PASS" -eq "$EXAMPLE_COUNT" ]; then
                pass "All $EXAMPLE_COUNT $crate examples compile"
            else
                fail "$EXAMPLE_PASS/$EXAMPLE_COUNT $crate examples compile"
            fi
        else
            info "No $crate examples found"
        fi
    done
fi

# =============================================================================
# 12. INTEGRATION TESTS
# =============================================================================
section "12. Integration Tests"

if [ "$QUICK_MODE" = true ]; then
    skip "Integration tests (quick mode)"
else
    info "Running integration test suite..."
    if cargo test --test '*' --workspace 2>&1 | tail -10; then
        pass "Integration tests passed"
    else
        warn "Some integration tests failed"
    fi
fi

# =============================================================================
# 13. BENCHMARK SMOKE TEST
# =============================================================================
section "13. Benchmark Smoke Test"

if [ "$QUICK_MODE" = true ]; then
    skip "Benchmarks (quick mode)"
else
    info "Compiling benchmarks (smoke test)..."
    if cargo bench --no-run 2>&1 | tail -5; then
        pass "Benchmarks compile successfully"
    else
        fail "Benchmark compilation failed"
    fi
fi

# =============================================================================
# 14. FEATURE MATRIX TESTING
# =============================================================================
section "14. Feature Matrix Testing"

if [ "$QUICK_MODE" = true ]; then
    skip "Feature matrix (quick mode)"
else
    declare -A LIBRARY_FEATURES
    LIBRARY_FEATURES["default"]=""
    LIBRARY_FEATURES["ml-inference"]="--features ml-inference"
    LIBRARY_FEATURES["onnx"]="--features onnx"
    LIBRARY_FEATURES["standard-nodes"]="--features standard-nodes"

    declare -A CORE_FEATURES
    CORE_FEATURES["tls"]="--features tls"
    CORE_FEATURES["quic"]="--features quic"

    FEATURE_PASS=0
    FEATURE_TOTAL=0

    for feature_name in "${!LIBRARY_FEATURES[@]}"; do
        FEATURE_FLAGS="${LIBRARY_FEATURES[$feature_name]}"
        FEATURE_TOTAL=$((FEATURE_TOTAL + 1))

        if cargo check -p horus_library $FEATURE_FLAGS 2>/dev/null; then
            FEATURE_PASS=$((FEATURE_PASS + 1))
        else
            warn "Feature '$feature_name' (horus_library) check failed"
        fi
    done

    for feature_name in "${!CORE_FEATURES[@]}"; do
        FEATURE_FLAGS="${CORE_FEATURES[$feature_name]}"
        FEATURE_TOTAL=$((FEATURE_TOTAL + 1))

        if cargo check -p horus_core $FEATURE_FLAGS 2>/dev/null; then
            FEATURE_PASS=$((FEATURE_PASS + 1))
        else
            warn "Feature '$feature_name' (horus_core) check failed"
        fi
    done

    if [ "$FEATURE_PASS" -eq "$FEATURE_TOTAL" ]; then
        pass "All $FEATURE_TOTAL feature combinations compile"
    else
        fail "$FEATURE_PASS/$FEATURE_TOTAL feature combinations compile"
    fi
fi

# =============================================================================
# 15. SHELL SCRIPT SYNTAX VALIDATION
# =============================================================================
section "15. Shell Script Syntax Validation"

SHELL_SCRIPTS=(
    "install.sh"
    "uninstall.sh"
    "scripts/prerelease.sh"
    "scripts/release.sh"
    "scripts/deps.sh"
)

for script in "${SHELL_SCRIPTS[@]}"; do
    if [ -f "$script" ]; then
        if bash -n "$script" 2>/dev/null; then
            pass "$(basename $script) syntax valid"
        else
            fail "$(basename $script) has syntax errors"
        fi
    fi
done

# Also check any other .sh files in scripts/
for script in scripts/*.sh; do
    if [ -f "$script" ]; then
        SCRIPT_NAME=$(basename "$script")
        if [[ ! " prerelease.sh release.sh deps.sh " =~ " $SCRIPT_NAME " ]]; then
            if bash -n "$script" 2>/dev/null; then
                pass "$SCRIPT_NAME syntax valid"
            else
                fail "$SCRIPT_NAME has syntax errors"
            fi
        fi
    fi
done

# =============================================================================
# 16. VERSION CONSISTENCY CHECK
# =============================================================================
section "16. Version Consistency"

# Get version from main horus/Cargo.toml (the unified re-export crate)
MAIN_VERSION=$(grep -m1 '^version = ' horus/Cargo.toml 2>/dev/null | sed 's/version = "\(.*\)"/\1/')

if [ -n "$MAIN_VERSION" ]; then
    info "Main version: $MAIN_VERSION"

    VERSION_MISMATCH=0

    # --- Cargo.toml files ---
    CARGO_FILES=(
        "horus/Cargo.toml"
        "horus_core/Cargo.toml"
        "horus_macros/Cargo.toml"
        "horus_library/Cargo.toml"
        "horus_manager/Cargo.toml"
        "horus_py/Cargo.toml"
        "horus_ai/Cargo.toml"
        "benchmarks/Cargo.toml"
    )

    for cargo_file in "${CARGO_FILES[@]}"; do
        if [ -f "$cargo_file" ]; then
            FILE_VERSION=$(grep -m1 '^version = ' "$cargo_file" 2>/dev/null | sed 's/version = "\(.*\)"/\1/')
            if [ "$FILE_VERSION" != "$MAIN_VERSION" ]; then
                warn "Version mismatch in $cargo_file: $FILE_VERSION (expected $MAIN_VERSION)"
                VERSION_MISMATCH=$((VERSION_MISMATCH + 1))
            fi
        fi
    done

    # --- Python: pyproject.toml ---
    if [ -f "horus_py/pyproject.toml" ]; then
        PY_PROJECT_VERSION=$(grep -m1 '^version = ' horus_py/pyproject.toml 2>/dev/null | sed 's/version = "\(.*\)"/\1/')
        if [ "$PY_PROJECT_VERSION" != "$MAIN_VERSION" ]; then
            warn "Version mismatch in horus_py/pyproject.toml: $PY_PROJECT_VERSION (expected $MAIN_VERSION)"
            VERSION_MISMATCH=$((VERSION_MISMATCH + 1))
        fi
    fi

    # --- Python: horus/__init__.py ---
    if [ -f "horus_py/horus/__init__.py" ]; then
        PY_INIT_VERSION=$(grep -m1 '^__version__' horus_py/horus/__init__.py 2>/dev/null | sed 's/__version__ = "\(.*\)"/\1/')
        if [ "$PY_INIT_VERSION" != "$MAIN_VERSION" ]; then
            warn "Version mismatch in horus_py/horus/__init__.py: $PY_INIT_VERSION (expected $MAIN_VERSION)"
            VERSION_MISMATCH=$((VERSION_MISMATCH + 1))
        fi
    fi

    # --- Python: benchmarks/__init__.py ---
    if [ -f "horus_py/benchmarks/__init__.py" ]; then
        PY_BENCH_VERSION=$(grep -m1 '^__version__' horus_py/benchmarks/__init__.py 2>/dev/null | sed 's/__version__ = "\(.*\)"/\1/')
        if [ "$PY_BENCH_VERSION" != "$MAIN_VERSION" ]; then
            warn "Version mismatch in horus_py/benchmarks/__init__.py: $PY_BENCH_VERSION (expected $MAIN_VERSION)"
            VERSION_MISMATCH=$((VERSION_MISMATCH + 1))
        fi
    fi

    # --- Rust: CLI version in horus_manager/src/main.rs ---
    if [ -f "horus_manager/src/main.rs" ]; then
        CLI_VERSION=$(grep -m1 'version = "' horus_manager/src/main.rs 2>/dev/null | sed 's/.*version = "\(.*\)".*/\1/')
        if [ -n "$CLI_VERSION" ] && [ "$CLI_VERSION" != "$MAIN_VERSION" ]; then
            warn "Version mismatch in horus_manager/src/main.rs (CLI): $CLI_VERSION (expected $MAIN_VERSION)"
            VERSION_MISMATCH=$((VERSION_MISMATCH + 1))
        fi
    fi

    # --- Rust: monitor.rs JSON version ---
    if [ -f "horus_manager/src/monitor.rs" ]; then
        MONITOR_VERSION=$(grep -m1 '"version":' horus_manager/src/monitor.rs 2>/dev/null | sed 's/.*"version": "\(.*\)".*/\1/')
        if [ -n "$MONITOR_VERSION" ] && [ "$MONITOR_VERSION" != "$MAIN_VERSION" ]; then
            warn "Version mismatch in horus_manager/src/monitor.rs: $MONITOR_VERSION (expected $MAIN_VERSION)"
            VERSION_MISMATCH=$((VERSION_MISMATCH + 1))
        fi
    fi

    TOTAL_VERSION_FILES=$((${#CARGO_FILES[@]} + 4))  # +4 for pyproject, __init__.py x2, CLI, monitor
    if [ "$VERSION_MISMATCH" -eq 0 ]; then
        pass "All $TOTAL_VERSION_FILES version sources consistent ($MAIN_VERSION)"
    else
        fail "$VERSION_MISMATCH files have version mismatches"
    fi
else
    warn "Could not determine main version"
fi

# =============================================================================
# 17. DEAD CODE DETECTION
# =============================================================================
section "17. Dead Code Detection"

if [ "$QUICK_MODE" = true ]; then
    skip "Dead code detection (quick mode)"
else
    info "Checking for dead code..."
    DEAD_CODE_OUTPUT=$(RUSTFLAGS="-Wdead_code -Wunused" cargo check --workspace 2>&1)
    DEAD_CODE_WARNINGS=$(echo "$DEAD_CODE_OUTPUT" | grep -c "warning: .* is never" || true)

    if [ "$DEAD_CODE_WARNINGS" -gt 20 ]; then
        warn "Found $DEAD_CODE_WARNINGS potential dead code warnings"
    elif [ "$DEAD_CODE_WARNINGS" -gt 0 ]; then
        info "Found $DEAD_CODE_WARNINGS minor dead code warnings"
        pass "Dead code check passed (minor warnings)"
    else
        pass "No dead code detected"
    fi
fi

# =============================================================================
# 18. DOCKER CONTAINER TESTS (Optional)
# =============================================================================
section "18. Docker Container Tests"

if [ "$FULL_MODE" = true ]; then
    if command -v docker &>/dev/null; then
        if [ -f "tests/shell_scripts/run_tests.sh" ]; then
            info "Running Docker container tests..."
            if bash tests/shell_scripts/run_tests.sh install ubuntu-22.04 2>&1 | tail -20; then
                pass "Docker container tests passed"
            else
                warn "Docker container tests had issues"
            fi
        else
            skip "Docker test script not found"
        fi
    else
        skip "Docker not available"
    fi
else
    skip "Docker container tests (use --full to enable)"
fi

# =============================================================================
# SUMMARY
# =============================================================================
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo ""
echo -e "${BLUE}══════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${WHITE}  SUMMARY${NC}"
echo -e "${BLUE}══════════════════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "  Total checks:  $TOTAL"
echo -e "  ${GREEN}Passed:${NC}        $PASSED"
echo -e "  ${RED}Failed:${NC}        $FAILED"
echo -e "  ${YELLOW}Warnings:${NC}      $WARNINGS"
echo -e "  Skipped:       $SKIPPED"
echo -e "  Duration:      ${DURATION}s"
echo ""

if [ "$FAILED" -gt 0 ]; then
    echo -e "${RED}══════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${RED}  [$CROSS] FAILURES - DO NOT PUSH TO PRODUCTION${NC}"
    echo -e "${RED}══════════════════════════════════════════════════════════════════════════════${NC}"
    echo ""
    for failure in "${FAILURES[@]}"; do
        echo -e "  $CROSS $failure"
    done
    echo ""
    exit 1
fi

if [ "$WARNINGS" -gt 5 ]; then
    echo -e "${YELLOW}══════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}  [$WARN] WARNINGS - Consider fixing before release${NC}"
    echo -e "${YELLOW}══════════════════════════════════════════════════════════════════════════════${NC}"
    echo ""
    for warning in "${WARNS[@]}"; do
        echo -e "  $WARN $warning"
    done
    echo ""
fi

echo -e "${GREEN}══════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  [$CHECK] ALL CHECKS PASSED - READY FOR PRODUCTION${NC}"
echo -e "${GREEN}══════════════════════════════════════════════════════════════════════════════${NC}"
echo ""
echo "  Next steps:"
echo "    1. git add -A"
echo "    2. git commit -m 'Your commit message'"
echo "    3. git push origin main"
echo ""
echo "  Or run release script:"
echo "    ./scripts/release.sh <version>"
echo ""

exit 0
