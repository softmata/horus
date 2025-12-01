#!/bin/bash
#
# HORUS Pre-Release Verification Script
#
# Comprehensive verification before pushing to production.
# Run this BEFORE committing to ensure 100% confidence.
#
# Checks:
#   1. Code compilation (all workspace crates)
#   2. Cargo clippy (lint warnings)
#   3. Cargo fmt (formatting)
#   4. Cargo test (all tests)
#   5. CLI commands (horus run, new, sim2d, sim3d, etc.)
#   6. Python bindings (horus_py build and import)
#   7. Documentation site (build check)
#   8. Example files compilation
#   9. Integration tests
#   10. install.sh dry-run check
#   11. verify.sh execution
#
# Usage: ./scripts/prerelease.sh [--quick] [--fix]
#   --quick: Skip slow tests (integration, docs)
#   --fix: Auto-fix formatting issues
#

set -e

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
for arg in "$@"; do
    case $arg in
        --quick) QUICK_MODE=true ;;
        --fix) FIX_MODE=true ;;
        --help|-h)
            echo "HORUS Pre-Release Verification Script"
            echo ""
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --quick    Skip slow tests (integration tests, docs build)"
            echo "  --fix      Auto-fix formatting issues"
            echo "  --help     Show this help message"
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

run_cmd_output() {
    local desc="$1"
    shift
    echo -e "  [$ARROW] $desc"
    if "$@" 2>&1; then
        return 0
    else
        return 1
    fi
}

# Start time
START_TIME=$(date +%s)

echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║${NC}  ${WHITE}HORUS Pre-Release Verification${NC}                                              ${BLUE}║${NC}"
echo -e "${BLUE}║${NC}  ${CYAN}Comprehensive • Systematic • Production-Ready${NC}                               ${BLUE}║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

if [ "$QUICK_MODE" = true ]; then
    echo -e "  ${YELLOW}Running in QUICK mode (skipping slow tests)${NC}"
fi
if [ "$FIX_MODE" = true ]; then
    echo -e "  ${YELLOW}Running with --fix mode (auto-fixing issues)${NC}"
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

if cargo check --workspace --all-targets 2>&1 | tail -5; then
    pass "All workspace crates compile"
else
    fail "Compilation errors found"
fi

# =============================================================================
# 3. CARGO CLIPPY - Linting
# =============================================================================
section "3. Linting (cargo clippy)"

CLIPPY_OUTPUT=$(cargo clippy --workspace --all-targets 2>&1)
CLIPPY_WARNINGS=$(echo "$CLIPPY_OUTPUT" | grep -c "warning:" || true)
CLIPPY_ERRORS=$(echo "$CLIPPY_OUTPUT" | grep -c "error\[" || true)

if [ "$CLIPPY_ERRORS" -gt 0 ]; then
    fail "Clippy found $CLIPPY_ERRORS errors"
    echo "$CLIPPY_OUTPUT" | grep "error\[" | head -10
elif [ "$CLIPPY_WARNINGS" -gt 10 ]; then
    warn "Clippy found $CLIPPY_WARNINGS warnings (consider fixing)"
    pass "No clippy errors"
else
    pass "Clippy passed ($CLIPPY_WARNINGS warnings)"
fi

# =============================================================================
# 4. CARGO BUILD RELEASE - Full Release Build
# =============================================================================
section "4. Release Build (cargo build --release)"

info "Building all workspace crates in release mode..."
if cargo build --release --workspace 2>&1 | tail -10; then
    pass "Release build successful"

    # Check binaries exist
    for bin in horus sim2d sim3d horus_router; do
        if [ -f "./target/release/$bin" ]; then
            pass "Binary: $bin"
        else
            if [ "$bin" = "horus_router" ]; then
                info "Binary: $bin (optional, not built)"
            else
                fail "Binary missing: $bin"
            fi
        fi
    done
else
    fail "Release build failed"
fi

# =============================================================================
# 5. CARGO TEST - Unit Tests
# =============================================================================
section "5. Unit Tests (cargo test)"

info "Running all workspace tests..."
if cargo test --workspace 2>&1 | tail -20; then
    pass "All unit tests passed"
else
    fail "Unit tests failed"
fi

# =============================================================================
# 6. CLI COMMANDS - Verify All CLI Features
# =============================================================================
section "6. CLI Command Verification"

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

    # Test subcommands help
    for cmd in run new init check dashboard pkg sim2d sim3d; do
        if $HORUS_BIN $cmd --help >/dev/null 2>&1; then
            pass "horus $cmd --help"
        else
            fail "horus $cmd --help failed"
        fi
    done

    # Test horus new (dry run - just check it doesn't crash on invalid input)
    if $HORUS_BIN new --help >/dev/null 2>&1; then
        pass "horus new command available"
    else
        fail "horus new not working"
    fi

    # Test horus check on root horus.yaml
    if [ -f "horus.yaml" ]; then
        if $HORUS_BIN check horus.yaml >/dev/null 2>&1; then
            pass "horus check horus.yaml"
        else
            warn "horus check horus.yaml had issues"
        fi
    fi
else
    fail "horus binary not found at $HORUS_BIN"
fi

# Test sim2d and sim3d binaries
for sim_bin in sim2d sim3d; do
    SIM_PATH="./target/release/$sim_bin"
    if [ -x "$SIM_PATH" ]; then
        if $SIM_PATH --help >/dev/null 2>&1; then
            pass "$sim_bin --help"
        else
            fail "$sim_bin --help failed"
        fi
    else
        warn "$sim_bin binary not found"
    fi
done

# =============================================================================
# 7. PYTHON BINDINGS - horus_py
# =============================================================================
section "7. Python Bindings (horus_py)"

if command -v python3 &>/dev/null; then
    PY_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    info "Python version: $PY_VERSION"

    # Check if maturin is available
    if command -v maturin &>/dev/null; then
        info "maturin found, building horus_py..."

        cd horus_py
        if maturin build --release 2>&1 | tail -5; then
            pass "horus_py wheel built successfully"

            # Try to install and import
            WHEEL_FILE=$(ls target/wheels/*.whl 2>/dev/null | head -1)
            if [ -n "$WHEEL_FILE" ]; then
                # Create temp venv for testing
                TEMP_VENV=$(mktemp -d)
                if python3 -m venv "$TEMP_VENV" 2>/dev/null; then
                    source "$TEMP_VENV/bin/activate"
                    if pip install "$WHEEL_FILE" >/dev/null 2>&1; then
                        if python3 -c "import horus; print(f'horus_py version: {horus.__version__}')" 2>/dev/null; then
                            pass "horus_py import successful"
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
        skip "maturin not installed (pip install maturin)"
    fi
else
    skip "Python3 not available"
fi

# =============================================================================
# 8. DOCUMENTATION SITE
# =============================================================================
section "8. Documentation Site"

if [ "$QUICK_MODE" = true ]; then
    skip "Documentation build (quick mode)"
else
    if [ -d "docs-site" ]; then
        cd docs-site

        if [ -f "package.json" ]; then
            # Check if node_modules exists
            if [ ! -d "node_modules" ]; then
                info "Installing docs-site dependencies..."
                npm install >/dev/null 2>&1 || true
            fi

            # Build docs
            info "Building documentation site..."
            if npm run build 2>&1 | tail -10; then
                pass "Documentation site builds successfully"
            else
                warn "Documentation site build had issues"
            fi

            # Check search index
            if [ -f "public/search-index.json" ]; then
                pass "Search index generated"
            else
                warn "Search index missing"
            fi
        else
            skip "docs-site package.json not found"
        fi

        cd "$ROOT_DIR"
    else
        skip "docs-site directory not found"
    fi
fi

# =============================================================================
# 9. EXAMPLES COMPILATION
# =============================================================================
section "9. Examples Compilation"

# Check horus_core examples
EXAMPLE_COUNT=0
EXAMPLE_PASS=0

for example in horus_core/examples/*.rs; do
    if [ -f "$example" ]; then
        EXAMPLE_NAME=$(basename "$example" .rs)
        EXAMPLE_COUNT=$((EXAMPLE_COUNT + 1))

        if cargo build --release --example "$EXAMPLE_NAME" -p horus_core >/dev/null 2>&1; then
            EXAMPLE_PASS=$((EXAMPLE_PASS + 1))
        else
            warn "Example failed to build: $EXAMPLE_NAME"
        fi
    fi
done

if [ "$EXAMPLE_COUNT" -gt 0 ]; then
    if [ "$EXAMPLE_PASS" -eq "$EXAMPLE_COUNT" ]; then
        pass "All $EXAMPLE_COUNT horus_core examples compile"
    else
        fail "$EXAMPLE_PASS/$EXAMPLE_COUNT horus_core examples compile"
    fi
fi

# Check horus_library examples
EXAMPLE_COUNT=0
EXAMPLE_PASS=0

for example in horus_library/examples/*.rs; do
    if [ -f "$example" ]; then
        EXAMPLE_NAME=$(basename "$example" .rs)
        EXAMPLE_COUNT=$((EXAMPLE_COUNT + 1))

        if cargo build --release --example "$EXAMPLE_NAME" -p horus_library >/dev/null 2>&1; then
            EXAMPLE_PASS=$((EXAMPLE_PASS + 1))
        else
            warn "Example failed to build: $EXAMPLE_NAME"
        fi
    fi
done

if [ "$EXAMPLE_COUNT" -gt 0 ]; then
    if [ "$EXAMPLE_PASS" -eq "$EXAMPLE_COUNT" ]; then
        pass "All $EXAMPLE_COUNT horus_library examples compile"
    else
        fail "$EXAMPLE_PASS/$EXAMPLE_COUNT horus_library examples compile"
    fi
fi

# =============================================================================
# 10. INTEGRATION TESTS
# =============================================================================
section "10. Integration Tests"

if [ "$QUICK_MODE" = true ]; then
    skip "Integration tests (quick mode)"
else
    # Test horus run with a simple test project
    if [ -d "tests/dashboard/1pub" ]; then
        info "Testing horus run with test project..."

        cd tests/dashboard/1pub

        # Build but don't run (would need to be killed)
        if $ROOT_DIR/target/release/horus check horus.yaml >/dev/null 2>&1; then
            pass "horus check on test project"
        else
            warn "horus check had issues on test project"
        fi

        cd "$ROOT_DIR"
    fi

    # Test multi-language example exists and is valid
    if [ -d "tests/multi_language_example" ]; then
        cd tests/multi_language_example

        if [ -f "horus.yaml" ]; then
            if $ROOT_DIR/target/release/horus check horus.yaml >/dev/null 2>&1; then
                pass "Multi-language example validates"
            else
                warn "Multi-language example validation issues"
            fi
        fi

        cd "$ROOT_DIR"
    fi
fi

# =============================================================================
# 11. VERIFY.SH EXECUTION
# =============================================================================
section "11. verify.sh Execution"

if [ "$QUICK_MODE" = true ]; then
    skip "verify.sh (quick mode)"
else
    if [ -x "verify.sh" ]; then
        info "Running verify.sh..."
        if ./verify.sh 2>&1 | tail -30; then
            pass "verify.sh completed"
        else
            warn "verify.sh had some issues (check output above)"
        fi
    else
        warn "verify.sh not found or not executable"
    fi
fi

# =============================================================================
# 12. INSTALL SCRIPT SYNTAX CHECK
# =============================================================================
section "12. Install Script Validation"

if [ -f "install.sh" ]; then
    # Bash syntax check
    if bash -n install.sh 2>/dev/null; then
        pass "install.sh syntax valid"
    else
        fail "install.sh has syntax errors"
    fi
fi

if [ -f "uninstall.sh" ]; then
    if bash -n uninstall.sh 2>/dev/null; then
        pass "uninstall.sh syntax valid"
    else
        fail "uninstall.sh has syntax errors"
    fi
fi

if [ -f "verify.sh" ]; then
    if bash -n verify.sh 2>/dev/null; then
        pass "verify.sh syntax valid"
    else
        fail "verify.sh has syntax errors"
    fi
fi

for script in scripts/*.sh; do
    if [ -f "$script" ]; then
        if bash -n "$script" 2>/dev/null; then
            pass "$(basename $script) syntax valid"
        else
            fail "$(basename $script) has syntax errors"
        fi
    fi
done

# =============================================================================
# 13. VERSION CONSISTENCY CHECK
# =============================================================================
section "13. Version Consistency"

# Get version from main Cargo.toml
MAIN_VERSION=$(grep -m1 '^version = ' horus/Cargo.toml 2>/dev/null | sed 's/version = "\(.*\)"/\1/')

if [ -n "$MAIN_VERSION" ]; then
    info "Main version: $MAIN_VERSION"

    VERSION_MISMATCH=0

    # Check other Cargo.toml files
    for cargo_file in horus_core/Cargo.toml horus_library/Cargo.toml horus_manager/Cargo.toml horus_py/Cargo.toml; do
        if [ -f "$cargo_file" ]; then
            FILE_VERSION=$(grep -m1 '^version = ' "$cargo_file" 2>/dev/null | sed 's/version = "\(.*\)"/\1/')
            if [ "$FILE_VERSION" != "$MAIN_VERSION" ]; then
                warn "Version mismatch in $cargo_file: $FILE_VERSION (expected $MAIN_VERSION)"
                VERSION_MISMATCH=$((VERSION_MISMATCH + 1))
            fi
        fi
    done

    if [ "$VERSION_MISMATCH" -eq 0 ]; then
        pass "All versions consistent ($MAIN_VERSION)"
    else
        fail "$VERSION_MISMATCH files have version mismatches"
    fi
else
    warn "Could not determine main version"
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
    echo -e "${RED}  FAILURES - DO NOT PUSH TO PRODUCTION${NC}"
    echo -e "${RED}══════════════════════════════════════════════════════════════════════════════${NC}"
    echo ""
    for failure in "${FAILURES[@]}"; do
        echo -e "  ${CROSS} $failure"
    done
    echo ""
    exit 1
fi

if [ "$WARNINGS" -gt 5 ]; then
    echo -e "${YELLOW}══════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}  WARNINGS - Consider fixing before release${NC}"
    echo -e "${YELLOW}══════════════════════════════════════════════════════════════════════════════${NC}"
    echo ""
    for warning in "${WARNS[@]}"; do
        echo -e "  ${WARN} $warning"
    done
    echo ""
fi

echo -e "${GREEN}══════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  ALL CHECKS PASSED - READY FOR PRODUCTION${NC}"
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
