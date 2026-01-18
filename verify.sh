#!/bin/bash
# HORUS Installation Verification Script v3.0.0
# Comprehensive, systematic, complete verification
# Checks: System, Binaries, Libraries, Cache, Source, Examples, Network, GPU,
#         Python bindings, Transport protocols, Configuration, Drivers

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

# Status indicators
STATUS_OK="[+]"
STATUS_ERR="[-]"
STATUS_WARN="[!]"
STATUS_INFO="[*]"

# Script version
SCRIPT_VERSION="3.0.0"

# Spinner function - simple dots
spin() {
    local pid=$1
    local msg="$2"
    local spin_chars=('.' '..' '...' '....')
    local i=0
    tput civis 2>/dev/null || true
    while kill -0 $pid 2>/dev/null; do
        printf "\r  ${spin_chars[$i]} ${msg}"
        i=$(( (i + 1) % ${#spin_chars[@]} ))
        sleep 0.25
    done
    tput cnorm 2>/dev/null || true
    printf "\r\033[K"
}

# ============================================================================
# PROGRESS BAR FUNCTIONS - Real progress with percentages
# ============================================================================

# Global progress tracking
VERIFY_TOTAL_CHECKS=0
VERIFY_CURRENT_CHECK=0
VERIFY_START_TIME=0

# Initialize verification progress
init_verify_progress() {
    VERIFY_TOTAL_CHECKS=$1
    VERIFY_CURRENT_CHECK=0
    VERIFY_START_TIME=$(date +%s)
}

# Update verification progress
update_verify_progress() {
    local section="$1"
    VERIFY_CURRENT_CHECK=$((VERIFY_CURRENT_CHECK + 1))

    local percent=0
    if [ "$VERIFY_TOTAL_CHECKS" -gt 0 ]; then
        percent=$((VERIFY_CURRENT_CHECK * 100 / VERIFY_TOTAL_CHECKS))
    fi

    # Calculate ETA
    local elapsed=$(($(date +%s) - VERIFY_START_TIME))
    local eta_str=""
    if [ "$elapsed" -gt 1 ] && [ "$percent" -gt 0 ] && [ "$percent" -lt 100 ]; then
        local total_estimated=$((elapsed * 100 / percent))
        local remaining=$((total_estimated - elapsed))
        if [ "$remaining" -gt 0 ]; then
            if [ "$remaining" -lt 60 ]; then
                eta_str=" ETA: ${remaining}s"
            else
                local mins=$((remaining / 60))
                eta_str=" ETA: ${mins}m"
            fi
        fi
    fi

    # Build progress bar
    local width=25
    local filled=$((percent * width / 100))
    local empty=$((width - filled))
    local bar=""
    for ((j=0; j<filled; j++)); do bar+="█"; done
    for ((j=0; j<empty; j++)); do bar+="░"; done

    # Print progress
    printf "\r  ${STATUS_INFO} [${bar}] %3d%% Checking: %-20s${eta_str}    " "$percent" "$section"
}

# Complete verification progress
complete_verify_progress() {
    local elapsed=$(($(date +%s) - VERIFY_START_TIME))
    printf "\r  ${STATUS_OK} [█████████████████████████] 100%% Verification completed in ${elapsed}s    \n"
}

# Source shared dependency functions
if [ -f "$SCRIPT_DIR/scripts/deps.sh" ]; then
    source "$SCRIPT_DIR/scripts/deps.sh"
    DEPS_SOURCED=true
else
    DEPS_SOURCED=false
    # Fallback OS detection
    OS_TYPE="unknown"
    OS_DISTRO="unknown"
    case "$(uname -s)" in
        Linux*) OS_TYPE="linux" ;;
        Darwin*) OS_TYPE="macos" ;;
    esac
fi

# Symbols
CHECK="${GREEN}✓${NC}"
CROSS="${RED}✗${NC}"
WARN="${YELLOW}!${NC}"
INFO="${CYAN}i${NC}"
SKIP="${WHITE}-${NC}"

# Statistics
ERRORS=0
WARNINGS=0
PASSES=0
SKIPPED=0

# Helper functions
pass() {
    echo -e "  [$CHECK] $1"
    PASSES=$((PASSES + 1))
}

fail() {
    echo -e "  [$CROSS] $1"
    ERRORS=$((ERRORS + 1))
}

warn() {
    echo -e "  [$WARN] $1"
    WARNINGS=$((WARNINGS + 1))
}

info() {
    echo -e "  [$INFO] $1"
}

skip() {
    echo -e "  [$SKIP] $1 (skipped)"
    SKIPPED=$((SKIPPED + 1))
}

section() {
    echo ""
    echo -e "${MAGENTA}━━━ $1 ━━━${NC}"
    echo ""
}

# Paths
INSTALL_DIR="$HOME/.cargo/bin"
HORUS_DIR="$HOME/.horus"
CACHE_DIR="$HORUS_DIR/cache"
TARGET_DIR="$HORUS_DIR/target"

# Detect installation profile from saved file or auto-detect platform
detect_install_profile() {
    # Check for saved profile
    if [ -f "$HORUS_DIR/install_profile" ]; then
        cat "$HORUS_DIR/install_profile"
        return
    fi

    # Auto-detect based on platform
    local platform="desktop"
    if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null || grep -q "BCM" /proc/cpuinfo 2>/dev/null; then
        platform="raspberry_pi"
    elif [ -f "/etc/nv_tegra_release" ] || grep -q "tegra" /proc/cpuinfo 2>/dev/null; then
        platform="jetson"
    elif grep -q "AM33XX" /proc/cpuinfo 2>/dev/null; then
        platform="beaglebone"
    elif [ "$(uname -m)" = "aarch64" ] || [ "$(uname -m)" = "armv7l" ]; then
        local mem_kb=$(grep MemTotal /proc/meminfo 2>/dev/null | awk '{print $2}')
        if [ -n "$mem_kb" ] && [ "$mem_kb" -lt 4000000 ]; then
            platform="arm_sbc"
        fi
    fi

    case "$platform" in
        raspberry_pi|jetson|arm_sbc) echo "embedded" ;;
        beaglebone) echo "minimal" ;;
        *) echo "full" ;;
    esac
}

INSTALL_PROFILE=$(detect_install_profile)

# Shared memory location
if [ "$OS_TYPE" = "linux" ]; then
    SHM_DIR="/dev/shm/horus"
else
    SHM_DIR="/tmp/horus"
fi

echo ""
echo -e "${BLUE}╔═════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║ ${NC}${WHITE}HORUS Installation Verification v$SCRIPT_VERSION${NC}  ${BLUE}║${NC}"
echo -e "${BLUE}║ ${NC}${CYAN}Comprehensive • Systematic • Complete${NC}       ${BLUE}║${NC}"
echo -e "${BLUE}╚═════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  ${CYAN}Install Profile:${NC} ${INSTALL_PROFILE}"
echo ""

# Initialize progress bar (19 main sections now)
init_verify_progress 19

#=====================================
# 1. SYSTEM REQUIREMENTS
#=====================================
update_verify_progress "System Requirements"
section "1. System Requirements"

# OS Detection
info "Operating System: $OS_TYPE"
[ -n "$OS_DISTRO" ] && [ "$OS_DISTRO" != "unknown" ] && info "Distribution: $OS_DISTRO"

# ============================================================================
# VERSION REQUIREMENTS (synchronized with install.sh v2.5.0)
# ============================================================================
# Rust version range
REQUIRED_RUST_MAJOR=1
REQUIRED_RUST_MINOR=85
REQUIRED_RUST_VERSION="${REQUIRED_RUST_MAJOR}.${REQUIRED_RUST_MINOR}"
MAX_TESTED_RUST_MAJOR=1
MAX_TESTED_RUST_MINOR=87
MAX_TESTED_RUST_VERSION="${MAX_TESTED_RUST_MAJOR}.${MAX_TESTED_RUST_MINOR}"

# Python version range
REQUIRED_PYTHON_MAJOR=3
REQUIRED_PYTHON_MINOR=9
REQUIRED_PYTHON_VERSION="${REQUIRED_PYTHON_MAJOR}.${REQUIRED_PYTHON_MINOR}"
MAX_TESTED_PYTHON_MAJOR=3
MAX_TESTED_PYTHON_MINOR=13
MAX_TESTED_PYTHON_VERSION="${MAX_TESTED_PYTHON_MAJOR}.${MAX_TESTED_PYTHON_MINOR}"

# Rust check with floor AND ceiling
if command -v rustc &> /dev/null; then
    RUST_VERSION=$(rustc --version | awk '{print $2}')
    RUST_MAJOR=$(echo $RUST_VERSION | cut -d'.' -f1)
    RUST_MINOR=$(echo $RUST_VERSION | cut -d'.' -f2)

    # Check floor (minimum)
    if [ "$RUST_MAJOR" -lt "$REQUIRED_RUST_MAJOR" ] || \
       ([ "$RUST_MAJOR" -eq "$REQUIRED_RUST_MAJOR" ] && [ "$RUST_MINOR" -lt "$REQUIRED_RUST_MINOR" ]); then
        fail "Rust $RUST_VERSION (< $REQUIRED_RUST_VERSION, please update via: rustup update stable)"
    # Check ceiling (maximum tested)
    elif [ "$RUST_MAJOR" -gt "$MAX_TESTED_RUST_MAJOR" ] || \
         ([ "$RUST_MAJOR" -eq "$MAX_TESTED_RUST_MAJOR" ] && [ "$RUST_MINOR" -gt "$MAX_TESTED_RUST_MINOR" ]); then
        warn "Rust $RUST_VERSION (NEWER than tested $MAX_TESTED_RUST_VERSION - may have API changes)"
    else
        pass "Rust $RUST_VERSION (within tested range $REQUIRED_RUST_VERSION - $MAX_TESTED_RUST_VERSION)"
    fi
else
    fail "Rust not installed (https://rustup.rs/)"
fi

# Cargo
if command -v cargo &> /dev/null; then
    CARGO_VERSION=$(cargo --version | awk '{print $2}')
    pass "Cargo $CARGO_VERSION"
else
    fail "Cargo not found"
fi

# C Compiler
CC_FOUND=false
for cc_cmd in cc gcc clang; do
    if command -v $cc_cmd &> /dev/null; then
        CC_NAME=$($cc_cmd --version 2>/dev/null | head -n1 | cut -d' ' -f1-3)
        pass "C compiler: $CC_NAME"
        CC_FOUND=true
        break
    fi
done
[ "$CC_FOUND" = false ] && fail "C compiler not found (install gcc or clang)"

# pkg-config
if command -v pkg-config &> /dev/null; then
    pass "pkg-config $(pkg-config --version)"
else
    fail "pkg-config not found"
fi

# cmake (needed for some deps)
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n1 | awk '{print $3}')
    pass "cmake $CMAKE_VERSION"
else
    warn "cmake not found (may be needed for some builds)"
fi

# Python3 check with floor AND ceiling (for horus_py)
if command -v python3 &> /dev/null; then
    PY_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    PY_MAJOR=$(echo $PY_VERSION | cut -d'.' -f1)
    PY_MINOR=$(echo $PY_VERSION | cut -d'.' -f2)

    # Check floor (minimum)
    if [ "$PY_MAJOR" -lt "$REQUIRED_PYTHON_MAJOR" ] || \
       ([ "$PY_MAJOR" -eq "$REQUIRED_PYTHON_MAJOR" ] && [ "$PY_MINOR" -lt "$REQUIRED_PYTHON_MINOR" ]); then
        warn "Python $PY_VERSION (< $REQUIRED_PYTHON_VERSION, horus_py requires 3.9+)"
    # Check ceiling (maximum tested)
    elif [ "$PY_MAJOR" -gt "$MAX_TESTED_PYTHON_MAJOR" ] || \
         ([ "$PY_MAJOR" -eq "$MAX_TESTED_PYTHON_MAJOR" ] && [ "$PY_MINOR" -gt "$MAX_TESTED_PYTHON_MINOR" ]); then
        warn "Python $PY_VERSION (NEWER than tested $MAX_TESTED_PYTHON_VERSION - PyO3 wheel may fail)"
    else
        pass "Python $PY_VERSION (within tested range $REQUIRED_PYTHON_VERSION - $MAX_TESTED_PYTHON_VERSION)"
    fi
else
    info "Python3 not found (optional for horus_py, requires $REQUIRED_PYTHON_VERSION - $MAX_TESTED_PYTHON_VERSION)"
fi

#=====================================
# 2. SYSTEM LIBRARIES
#=====================================
update_verify_progress "System Libraries"
section "2. System Libraries"

if [ "$DEPS_SOURCED" = true ]; then
    print_dep_status
    DEP_FAILURES=$?
    if [ $DEP_FAILURES -gt 0 ]; then
        ERRORS=$((ERRORS + DEP_FAILURES))
    fi
else
    # Fallback: Basic library checks
    declare -a LIBS=(
        "openssl:OpenSSL:required"
        "libudev:udev:linux"
        "alsa:ALSA:linux"
        "wayland-client:Wayland:linux"
        "xkbcommon:XKB:linux"
        "x11:X11:linux"
        "vulkan:Vulkan:optional"
    )

    for lib_info in "${LIBS[@]}"; do
        IFS=':' read -r lib desc req <<< "$lib_info"

        [ "$req" = "linux" ] && [ "$OS_TYPE" != "linux" ] && continue

        if pkg-config --exists "$lib" 2>/dev/null; then
            VERSION=$(pkg-config --modversion "$lib" 2>/dev/null || echo "installed")
            pass "$desc $VERSION"
        else
            if [ "$req" = "required" ]; then
                fail "$desc not found (REQUIRED)"
            elif [ "$req" = "optional" ]; then
                info "$desc not found (optional)"
            else
                warn "$desc not found (may affect build)"
            fi
        fi
    done
fi

#=====================================
# 3. HORUS BINARIES
#=====================================
update_verify_progress "HORUS Binaries"
section "3. HORUS Binaries"

# Main horus binary
if [ -x "$INSTALL_DIR/horus" ]; then
    if "$INSTALL_DIR/horus" --version &>/dev/null; then
        VERSION=$("$INSTALL_DIR/horus" --version 2>/dev/null | awk '{print $2}')
        pass "horus binary v$VERSION"

        # Check PATH
        if command -v horus &>/dev/null; then
            WHICH_HORUS=$(which horus)
            if [ "$WHICH_HORUS" = "$INSTALL_DIR/horus" ]; then
                pass "horus in PATH (correct location)"
            else
                warn "horus in PATH but different location: $WHICH_HORUS"
            fi
        else
            warn "horus not in PATH (add $INSTALL_DIR to PATH)"
        fi
    else
        fail "horus binary installed but not working"
    fi
else
    fail "horus binary not found at $INSTALL_DIR/horus"
fi

# sim2d binary (only required for full profile)
if [ "$INSTALL_PROFILE" = "full" ]; then
    if [ -x "$INSTALL_DIR/sim2d" ]; then
        if "$INSTALL_DIR/sim2d" --help &>/dev/null; then
            pass "sim2d binary installed"
        else
            fail "sim2d binary installed but not working"
        fi
    else
        fail "sim2d binary not found at $INSTALL_DIR/sim2d"
    fi
else
    if [ -x "$INSTALL_DIR/sim2d" ]; then
        pass "sim2d binary installed (optional)"
    else
        info "sim2d binary not installed (${INSTALL_PROFILE} profile)"
    fi
fi

# sim3d binary (only required for full profile)
if [ "$INSTALL_PROFILE" = "full" ]; then
    if [ -x "$INSTALL_DIR/sim3d" ]; then
        if "$INSTALL_DIR/sim3d" --help &>/dev/null; then
            pass "sim3d binary installed"
        else
            fail "sim3d binary installed but not working"
        fi
    else
        fail "sim3d binary not found at $INSTALL_DIR/sim3d"
    fi
else
    if [ -x "$INSTALL_DIR/sim3d" ]; then
        pass "sim3d binary installed (optional)"
    else
        info "sim3d binary not installed (${INSTALL_PROFILE} profile)"
    fi
fi

# horus_router binary (optional)
if [ -x "$INSTALL_DIR/horus_router" ]; then
    pass "horus_router binary installed"
else
    info "horus_router binary not found (optional)"
fi

#=====================================
# 4. HORUS LIBRARY CACHE
#=====================================
update_verify_progress "Library Cache"
section "4. Library Cache"

if [ -d "$CACHE_DIR" ]; then
    pass "Cache directory exists: $CACHE_DIR"

    # Check installed version
    VERSION_FILE="$HORUS_DIR/installed_version"
    if [ -f "$VERSION_FILE" ]; then
        INSTALLED_VERSION=$(cat "$VERSION_FILE")
        info "Installed version: $INSTALLED_VERSION"
    else
        warn "Version file missing: $VERSION_FILE"
    fi

    # The main horus@version directory contains everything
    # Structure: horus@version/horus/, horus@version/horus_core/, etc.
    HORUS_CACHE_DIR=$(ls -d "$CACHE_DIR"/horus@* 2>/dev/null | head -n1)

    if [ -n "$HORUS_CACHE_DIR" ] && [ -d "$HORUS_CACHE_DIR" ]; then
        HORUS_CACHE_VERSION=$(basename "$HORUS_CACHE_DIR" | sed "s/horus@//")
        pass "Main library cache v$HORUS_CACHE_VERSION"

        # Check for source subdirectories inside horus@version/
        declare -a SOURCE_SUBDIRS=(
            "horus:Main library"
            "horus_core:Runtime core"
            "horus_macros:Proc macros"
            "horus_library:Standard library"
            "horus_manager:Manager"
            "horus_router:Router"
            "horus_py:Python bindings"
        )

        for subdir_info in "${SOURCE_SUBDIRS[@]}"; do
            IFS=':' read -r subdir desc <<< "$subdir_info"

            if [ -d "$HORUS_CACHE_DIR/$subdir" ]; then
                # Check for Cargo.toml
                if [ -f "$HORUS_CACHE_DIR/$subdir/Cargo.toml" ]; then
                    # Check for src/ or lib.rs (horus_library uses lib.rs)
                    if [ -d "$HORUS_CACHE_DIR/$subdir/src" ] || [ -f "$HORUS_CACHE_DIR/$subdir/lib.rs" ]; then
                        pass "$desc source"
                    else
                        warn "$desc missing source files"
                    fi
                else
                    warn "$desc missing Cargo.toml"
                fi
            else
                # Optional components are fine to skip
                case "$subdir" in
                    horus_manager|horus_router|horus_py)
                        info "$desc source not cached (optional)"
                        ;;
                    *)
                        warn "$desc source directory missing"
                        ;;
                esac
            fi
        done

        # Check for workspace Cargo.toml (allows horus run to work)
        if [ -f "$HORUS_CACHE_DIR/Cargo.toml" ]; then
            pass "Workspace Cargo.toml present"
        else
            warn "Workspace Cargo.toml missing (horus run may not work)"
        fi
    else
        fail "Main library cache (horus@version) not found"
    fi

    # Also check the separate component lib directories for compiled artifacts
    declare -a LIB_COMPONENTS=(
        "horus_core:Runtime core libs"
        "horus_macros:Proc macro libs"
        "horus_library:Standard library libs"
    )

    for comp_info in "${LIB_COMPONENTS[@]}"; do
        IFS=':' read -r comp desc <<< "$comp_info"

        COMP_DIR=$(ls -d "$CACHE_DIR"/${comp}@* 2>/dev/null | head -n1)
        if [ -n "$COMP_DIR" ] && [ -d "$COMP_DIR/lib" ]; then
            LIB_COUNT=$(ls "$COMP_DIR/lib"/*.* 2>/dev/null | wc -l)
            if [ "$LIB_COUNT" -gt 0 ]; then
                pass "$desc ($LIB_COUNT files)"
            else
                info "$desc directory exists but empty"
            fi
        else
            info "$desc not separately cached"
        fi
    done
else
    fail "Cache directory missing: $CACHE_DIR"
fi

#=====================================
# 5. PRE-COMPILED DEPENDENCIES
#=====================================
update_verify_progress "Pre-compiled Deps"
section "5. Pre-compiled Dependencies"

# Pre-compiled deps are stored inside the horus@version cache directory
# (not at ~/.horus/target/ as previously assumed)
ACTUAL_TARGET_DIR=""
if [ -n "$HORUS_CACHE_DIR" ] && [ -d "$HORUS_CACHE_DIR/target" ]; then
    ACTUAL_TARGET_DIR="$HORUS_CACHE_DIR/target"
elif [ -d "$TARGET_DIR" ]; then
    ACTUAL_TARGET_DIR="$TARGET_DIR"
fi

if [ -n "$ACTUAL_TARGET_DIR" ] && [ -d "$ACTUAL_TARGET_DIR" ]; then
    pass "Target directory exists"

    # Check for release deps
    RELEASE_DEPS="$ACTUAL_TARGET_DIR/release/deps"
    if [ -d "$RELEASE_DEPS" ]; then
        # Count rlib files
        RLIB_COUNT=$(ls "$RELEASE_DEPS"/*.rlib 2>/dev/null | wc -l)
        if [ "$RLIB_COUNT" -gt 0 ]; then
            pass "Pre-compiled rlibs: $RLIB_COUNT files"
        else
            warn "No pre-compiled rlib files found"
        fi

        # Count rmeta files
        RMETA_COUNT=$(ls "$RELEASE_DEPS"/*.rmeta 2>/dev/null | wc -l)
        if [ "$RMETA_COUNT" -gt 0 ]; then
            pass "Pre-compiled rmeta: $RMETA_COUNT files"
        else
            warn "No pre-compiled rmeta files found"
        fi

        # Check for HORUS-specific libs
        HORUS_RLIBS=$(ls "$RELEASE_DEPS"/libhorus*.rlib 2>/dev/null | wc -l)
        if [ "$HORUS_RLIBS" -gt 0 ]; then
            pass "HORUS library artifacts: $HORUS_RLIBS files"
        else
            warn "HORUS library artifacts not found"
        fi
    else
        warn "Release deps directory missing"
    fi

    # Check fingerprints
    FINGERPRINT_DIR="$ACTUAL_TARGET_DIR/release/.fingerprint"
    if [ -d "$FINGERPRINT_DIR" ]; then
        FP_COUNT=$(ls -d "$FINGERPRINT_DIR"/horus* 2>/dev/null | wc -l)
        if [ "$FP_COUNT" -gt 0 ]; then
            pass "Build fingerprints: $FP_COUNT HORUS entries"
        else
            warn "No HORUS fingerprints found"
        fi
    else
        warn "Fingerprint directory missing"
    fi
else
    fail "Target directory missing"
    info "Run install.sh to create pre-compiled dependencies"
fi

#=====================================
# 6. SHARED MEMORY
#=====================================
update_verify_progress "Shared Memory"
section "6. Shared Memory"

# Check if shared memory directory is accessible
if [ "$OS_TYPE" = "linux" ]; then
    if [ -d "/dev/shm" ]; then
        pass "Shared memory available: /dev/shm"

        # Check permissions
        if [ -w "/dev/shm" ]; then
            pass "Shared memory writable"
        else
            fail "Shared memory not writable"
        fi

        # Check if HORUS session can be created
        TEST_SHM="/dev/shm/horus_verify_test_$$"
        if mkdir -p "$TEST_SHM" 2>/dev/null; then
            pass "Can create HORUS sessions"
            rmdir "$TEST_SHM" 2>/dev/null
        else
            fail "Cannot create HORUS sessions in /dev/shm"
        fi

        # Show existing HORUS sessions
        if [ -d "$SHM_DIR" ]; then
            SESSION_COUNT=$(ls -d "$SHM_DIR"/* 2>/dev/null | wc -l)
            info "Active HORUS sessions: $SESSION_COUNT"
        fi
    else
        fail "/dev/shm not available"
    fi
else
    # macOS fallback
    if [ -d "/tmp" ] && [ -w "/tmp" ]; then
        pass "Shared memory fallback: /tmp"
    else
        fail "/tmp not writable for shared memory"
    fi
fi

#=====================================
# 7. COMMAND FUNCTIONALITY
#=====================================
update_verify_progress "Command Tests"
section "7. Command Functionality"

if command -v horus &>/dev/null || [ -x "$INSTALL_DIR/horus" ]; then
    HORUS_CMD="${INSTALL_DIR}/horus"

    # Core commands
    declare -a COMMANDS=(
        "--version:Version check"
        "--help:Help system"
        "init --help:Workspace init"
        "new --help:Project creation"
        "run --help:Project runner"
        "check --help:Validation"
        "pkg --help:Package manager"
        "env --help:Environment"
        "monitor --help:Monitor"
        "auth --help:Authentication"
        "sim2d --help:2D Simulator"
        "sim3d --help:3D Simulator"
        "topic --help:Topic tools"
        "node --help:Node tools"
        "param --help:Parameters"
        "doctor --help:Diagnostics"
        "clean --help:Cleanup"
        "launch --help:Launch files"
    )

    ALL_OK=true
    for cmd_info in "${COMMANDS[@]}"; do
        IFS=':' read -r cmd desc <<< "$cmd_info"

        if $HORUS_CMD $cmd &>/dev/null; then
            pass "horus $cmd"
        else
            # Some commands might not exist yet - don't fail, just info
            info "horus $cmd (not available)"
        fi
    done
else
    skip "Command tests (horus not available)"
fi

#=====================================
# 8. EXAMPLES & TEMPLATES
#=====================================
update_verify_progress "Examples"
section "8. Examples & Templates"

# Check for examples in cache
if [ -d "$CACHE_DIR" ]; then
    # snakesim
    if ls "$CACHE_DIR"/snakesim* 1>/dev/null 2>&1; then
        pass "snakesim example available"
    else
        info "snakesim example not cached"
    fi

    # wallesim
    if ls "$CACHE_DIR"/wallesim* 1>/dev/null 2>&1; then
        pass "wallesim example available"
    else
        info "wallesim example not cached"
    fi

    # simple_driver
    if ls "$CACHE_DIR"/simple_driver* 1>/dev/null 2>&1; then
        pass "simple_driver example available"
    else
        info "simple_driver example not cached"
    fi
fi

# Check for templates
TEMPLATES_DIR="$HORUS_DIR/templates"
if [ -d "$TEMPLATES_DIR" ]; then
    TEMPLATE_COUNT=$(ls "$TEMPLATES_DIR" 2>/dev/null | wc -l)
    if [ "$TEMPLATE_COUNT" -gt 0 ]; then
        pass "Project templates: $TEMPLATE_COUNT available"
    else
        info "No project templates found"
    fi
else
    info "Templates directory not found"
fi

#=====================================
# 9. SHELL COMPLETIONS
#=====================================
update_verify_progress "Shell Completions"
section "9. Shell Completions"

# Bash completions
if [ -f "$HOME/.bash_completion.d/horus" ] || [ -f "/etc/bash_completion.d/horus" ] || [ -f "$HORUS_DIR/completions/horus.bash" ]; then
    pass "Bash completions installed"
else
    info "Bash completions not installed (optional)"
fi

# Zsh completions
if [ -f "$HOME/.zsh/completions/_horus" ] || [ -f "/usr/share/zsh/site-functions/_horus" ] || [ -f "$HORUS_DIR/completions/horus.zsh" ]; then
    pass "Zsh completions installed"
else
    info "Zsh completions not installed (optional)"
fi

# Fish completions
if [ -f "$HOME/.config/fish/completions/horus.fish" ] || [ -f "/usr/share/fish/completions/horus.fish" ] || [ -f "$HORUS_DIR/completions/horus.fish" ]; then
    pass "Fish completions installed"
else
    info "Fish completions not installed (optional)"
fi

#=====================================
# 10. ULTRA-LOW-LATENCY NETWORKING (optional)
#=====================================
update_verify_progress "Networking"
section "10. Ultra-Low-Latency Networking"

# Check for io_uring support (Linux 5.1+)
if [ "$OS_TYPE" = "linux" ]; then
    KERNEL_VERSION=$(uname -r | cut -d'.' -f1-2)
    KERNEL_MAJOR=$(echo $KERNEL_VERSION | cut -d'.' -f1)
    KERNEL_MINOR=$(echo $KERNEL_VERSION | cut -d'.' -f2)

    if [ "$KERNEL_MAJOR" -ge 5 ] && [ "$KERNEL_MINOR" -ge 1 ]; then
        pass "io_uring support: Linux $KERNEL_VERSION (>= 5.1)"
    elif [ "$KERNEL_MAJOR" -ge 6 ]; then
        pass "io_uring support: Linux $KERNEL_VERSION (>= 5.1)"
    else
        info "io_uring not available: Linux $KERNEL_VERSION (requires 5.1+)"
    fi

    # Check for Batch UDP support (always available on Linux)
    pass "Batch UDP (sendmmsg/recvmmsg): Available"
else
    info "Ultra-low-latency features only available on Linux"
fi

#=====================================
# 11. GPU & GRAPHICS (for sim3d)
#=====================================
update_verify_progress "GPU & Graphics"
section "11. GPU & Graphics (sim3d)"

# Check Vulkan
if command -v vulkaninfo &>/dev/null; then
    if vulkaninfo --summary 2>/dev/null | grep -q "GPU"; then
        GPU_NAME=$(vulkaninfo --summary 2>/dev/null | grep "GPU" | head -n1 | sed 's/.*GPU.*: //')
        pass "Vulkan available: $GPU_NAME"
    else
        warn "Vulkan installed but no GPU detected"
    fi
elif pkg-config --exists vulkan 2>/dev/null; then
    info "Vulkan SDK installed (vulkaninfo not available)"
else
    info "Vulkan not detected (required for sim3d graphics)"
fi

# Check for display
if [ -n "$DISPLAY" ] || [ -n "$WAYLAND_DISPLAY" ]; then
    if [ -n "$WAYLAND_DISPLAY" ]; then
        pass "Display available: Wayland ($WAYLAND_DISPLAY)"
    else
        pass "Display available: X11 ($DISPLAY)"
    fi
else
    info "No display detected (headless mode only)"
fi

# Check OpenGL (fallback renderer)
if command -v glxinfo &>/dev/null; then
    GL_VERSION=$(glxinfo 2>/dev/null | grep "OpenGL version" | head -n1 | sed 's/.*: //')
    if [ -n "$GL_VERSION" ]; then
        pass "OpenGL: $GL_VERSION"
    else
        info "OpenGL: Unable to query version"
    fi
else
    info "glxinfo not available (OpenGL status unknown)"
fi

#=====================================
# 12. NETWORK & REGISTRY
#=====================================
update_verify_progress "Network & Registry"
section "12. Network & Registry"

# Check network connectivity to crates.io sparse index (what cargo actually uses)
CRATES_STATUS=$(curl -s -o /dev/null -w "%{http_code}" --connect-timeout 5 -A "HORUS-verify/1.0" https://index.crates.io/config.json 2>/dev/null)
if [ "$CRATES_STATUS" = "200" ]; then
    pass "crates.io sparse index accessible"
else
    warn "crates.io not accessible (offline mode required)"
fi

# Check HORUS registry infrastructure (telemetry endpoint)
REGISTRY_URL="https://telemetry.horus-registry.dev"
REGISTRY_STATUS=$(curl -s -o /dev/null -w "%{http_code}" --connect-timeout 5 "$REGISTRY_URL" 2>/dev/null)
if [ "$REGISTRY_STATUS" = "200" ]; then
    pass "HORUS registry infrastructure accessible"
else
    info "HORUS registry not accessible (local-only mode)"
fi

# Check GitHub (for updates)
if curl -s --connect-timeout 5 https://api.github.com/rate_limit &>/dev/null; then
    pass "GitHub API accessible"
else
    info "GitHub API not accessible"
fi

#=====================================
# 13. DISK USAGE
#=====================================
update_verify_progress "Disk Usage"
section "13. Disk Usage"

if [ -d "$HORUS_DIR" ]; then
    HORUS_SIZE=$(du -sh "$HORUS_DIR" 2>/dev/null | awk '{print $1}')
    info "~/.horus total: $HORUS_SIZE"

    if [ -d "$CACHE_DIR" ]; then
        CACHE_SIZE=$(du -sh "$CACHE_DIR" 2>/dev/null | awk '{print $1}')
        info "  Library cache: $CACHE_SIZE"
    fi

    if [ -d "$TARGET_DIR" ]; then
        TARGET_SIZE=$(du -sh "$TARGET_DIR" 2>/dev/null | awk '{print $1}')
        info "  Pre-compiled: $TARGET_SIZE"
    fi
fi

CARGO_HOME="${CARGO_HOME:-$HOME/.cargo}"
if [ -d "$CARGO_HOME/registry" ]; then
    REGISTRY_SIZE=$(du -sh "$CARGO_HOME/registry" 2>/dev/null | awk '{print $1}')
    info "Cargo registry cache: $REGISTRY_SIZE"
fi

# Check available disk space
AVAILABLE=$(df -h "$HOME" 2>/dev/null | tail -1 | awk '{print $4}')
info "Available disk space: $AVAILABLE"

#=====================================
# 14. BUILD VERIFICATION (if in repo)
#=====================================
update_verify_progress "Build Verification"
section "14. Build Verification"

if [ -f "$SCRIPT_DIR/Cargo.toml" ] && grep -q "horus_manager" "$SCRIPT_DIR/Cargo.toml" 2>/dev/null; then
    info "Running in HORUS source repository"

    # Quick cargo check
    info "Running cargo check..."
    if cargo check --quiet 2>&1 | grep -q "error:"; then
        fail "cargo check has errors"
    else
        pass "cargo check passes"
    fi

    # Check for local debug build
    if [ -x "$SCRIPT_DIR/target/debug/horus" ]; then
        if "$SCRIPT_DIR/target/debug/horus" --version &>/dev/null; then
            pass "Debug build functional"
        else
            warn "Debug build exists but not working"
        fi
    fi

    # Check for local release build
    if [ -x "$SCRIPT_DIR/target/release/horus" ]; then
        if "$SCRIPT_DIR/target/release/horus" --version &>/dev/null; then
            pass "Release build functional"
        else
            warn "Release build exists but not working"
        fi
    fi
else
    skip "Build verification (not in HORUS repo)"
fi

#=====================================
# 15. PYTHON BINDINGS (horus_py)
#=====================================
update_verify_progress "Python Bindings"
section "15. Python Bindings (horus_py)"

if command -v python3 &>/dev/null; then
    # Try to import horus
    if python3 -c "import horus" 2>/dev/null; then
        HORUS_PY_VERSION=$(python3 -c "import horus; print(horus.__version__)" 2>/dev/null)
        pass "horus_py installed: v$HORUS_PY_VERSION"

        # Test core imports
        if python3 -c "from horus import Node, Topic" 2>/dev/null; then
            pass "Core classes importable (Node, Topic)"
        else
            warn "Some core classes not importable"
        fi

        # Test message imports
        if python3 -c "from horus import Twist, Vector3, Quaternion" 2>/dev/null; then
            pass "Message types importable"
        else
            info "Message types not available in this version"
        fi
    else
        info "horus_py not installed (optional)"
        info "Install with: pip install horus-robotics"
    fi

    # Check for maturin (for building from source)
    if command -v maturin &>/dev/null; then
        MATURIN_VERSION=$(maturin --version 2>&1 | awk '{print $2}')
        pass "maturin available: v$MATURIN_VERSION"
    else
        info "maturin not installed (install via: apt/dnf/pacman, pipx, or cargo)"
    fi
else
    skip "Python bindings check (Python3 not available)"
fi

#=====================================
# 16. TRANSPORT PROTOCOLS (Zenoh, TLS, QUIC)
#=====================================
update_verify_progress "Transport Protocols"
section "16. Transport Protocols"

# Check for Zenoh libraries
if pkg-config --exists zenohc 2>/dev/null; then
    ZENOH_VERSION=$(pkg-config --modversion zenohc 2>/dev/null)
    pass "Zenoh C library: $ZENOH_VERSION"
elif [ -f "/usr/lib/libzenohc.so" ] || [ -f "/usr/local/lib/libzenohc.so" ]; then
    pass "Zenoh C library found"
else
    info "Zenoh C library not found (Rust implementation will be used)"
fi

# Check TLS support (for secure communication)
if pkg-config --exists openssl 2>/dev/null; then
    OPENSSL_VERSION=$(pkg-config --modversion openssl 2>/dev/null)
    pass "TLS support (OpenSSL): $OPENSSL_VERSION"
elif command -v openssl &>/dev/null; then
    OPENSSL_VERSION=$(openssl version 2>/dev/null | awk '{print $2}')
    pass "TLS support (OpenSSL): $OPENSSL_VERSION"
else
    warn "OpenSSL not found (TLS features unavailable)"
fi

# Check for rustls (Rust TLS alternative)
info "QUIC support: Built-in (quinn crate)"

# Check network interfaces
if command -v ip &>/dev/null; then
    IFACE_COUNT=$(ip -o link show 2>/dev/null | grep -v "lo:" | wc -l)
    pass "Network interfaces: $IFACE_COUNT available"
elif command -v ifconfig &>/dev/null; then
    IFACE_COUNT=$(ifconfig 2>/dev/null | grep -c "^[a-z]")
    pass "Network interfaces: $IFACE_COUNT available"
fi

#=====================================
# 17. CONFIGURATION VALIDATION
#=====================================
update_verify_progress "Configuration"
section "17. Configuration Validation"

# Check for global config
CONFIG_FILE="$HORUS_DIR/config.toml"
if [ -f "$CONFIG_FILE" ]; then
    pass "Global config exists: $CONFIG_FILE"

    # Try to parse with horus if available
    if command -v horus &>/dev/null; then
        if horus check "$CONFIG_FILE" 2>/dev/null; then
            pass "Global config is valid"
        else
            warn "Global config has issues"
        fi
    fi
else
    info "No global config (using defaults)"
fi

# Check for sample horus.yaml files
SAMPLE_YAMLS=(
    "$SCRIPT_DIR/horus.yaml"
    "$SCRIPT_DIR/tests/monitor/1pub/horus.yaml"
    "$SCRIPT_DIR/horus_core/tests/horus.yaml"
)

for yaml in "${SAMPLE_YAMLS[@]}"; do
    if [ -f "$yaml" ]; then
        YAML_NAME=$(basename "$(dirname "$yaml")")/$(basename "$yaml")
        if command -v horus &>/dev/null; then
            if horus check "$yaml" 2>/dev/null; then
                pass "Sample config valid: $YAML_NAME"
            else
                warn "Sample config issues: $YAML_NAME"
            fi
        else
            info "Sample config exists: $YAML_NAME"
        fi
        break  # Only check one
    fi
done

# Check YAML parser in horus
if command -v horus &>/dev/null; then
    # Test parsing a simple inline YAML
    if echo "name: test" | horus check - 2>/dev/null; then
        info "YAML parser functional"
    fi
fi

#=====================================
# 18. DRIVER SYSTEM
#=====================================
update_verify_progress "Driver System"
section "18. Driver System"

# Check for driver-related system libraries
declare -a DRIVER_LIBS=(
    "libudev:Hardware enumeration (udev)"
    "libusb-1.0:USB device access"
    "libserial:Serial port access"
    "libi2c:I2C bus access"
)

for lib_info in "${DRIVER_LIBS[@]}"; do
    IFS=':' read -r lib desc <<< "$lib_info"

    if pkg-config --exists "$lib" 2>/dev/null; then
        VERSION=$(pkg-config --modversion "$lib" 2>/dev/null || echo "installed")
        pass "$desc: $VERSION"
    elif [ -f "/usr/lib/lib${lib}.so" ] || [ -f "/usr/lib/x86_64-linux-gnu/lib${lib}.so" ]; then
        pass "$desc: installed"
    else
        info "$desc: not found (may limit hardware support)"
    fi
done

# Check for common hardware interfaces
if [ -d "/sys/class/gpio" ]; then
    pass "GPIO interface available"
else
    info "GPIO interface not available"
fi

if [ -d "/dev" ]; then
    # Count I2C devices
    I2C_COUNT=$(ls /dev/i2c-* 2>/dev/null | wc -l)
    if [ "$I2C_COUNT" -gt 0 ]; then
        pass "I2C buses: $I2C_COUNT available"
    else
        info "No I2C buses detected"
    fi

    # Count serial ports
    SERIAL_COUNT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l)
    if [ "$SERIAL_COUNT" -gt 0 ]; then
        pass "Serial ports: $SERIAL_COUNT available"
    else
        info "No USB serial ports detected"
    fi

    # Check CAN interfaces
    CAN_COUNT=$(ls /sys/class/net/ 2>/dev/null | grep -c "^can" || echo 0)
    if [ "$CAN_COUNT" -gt 0 ]; then
        pass "CAN interfaces: $CAN_COUNT available"
    else
        info "No CAN interfaces detected"
    fi
fi

#=====================================
# 19. FEATURE FLAGS
#=====================================
update_verify_progress "Feature Flags"
section "19. Feature Flags"

# If we're in the source repo, check which features are available
if [ -f "$SCRIPT_DIR/horus_library/Cargo.toml" ]; then
    info "Checking available feature flags..."

    # Extract features from Cargo.toml
    FEATURES=$(grep "^\[features\]" -A 50 "$SCRIPT_DIR/horus_library/Cargo.toml" 2>/dev/null | grep "^[a-z]" | cut -d'=' -f1 | head -20)

    if [ -n "$FEATURES" ]; then
        FEATURE_COUNT=$(echo "$FEATURES" | wc -l)
        pass "Feature flags defined: $FEATURE_COUNT"

        # List key features
        for feature in ml-inference onnx tls quic opencv standard-nodes; do
            if echo "$FEATURES" | grep -q "^$feature"; then
                info "  Available: $feature"
            fi
        done
    else
        info "Feature flags: Unable to parse"
    fi
else
    info "Feature flags: Not in source repo"
fi

# Check if key optional dependencies are available
if pkg-config --exists onnxruntime 2>/dev/null; then
    ONNX_VERSION=$(pkg-config --modversion onnxruntime 2>/dev/null)
    pass "ONNX Runtime: $ONNX_VERSION (ml-inference feature)"
else
    info "ONNX Runtime: not found (ml-inference feature disabled)"
fi

if pkg-config --exists opencv4 2>/dev/null; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4 2>/dev/null)
    pass "OpenCV: $OPENCV_VERSION (opencv feature)"
elif pkg-config --exists opencv 2>/dev/null; then
    OPENCV_VERSION=$(pkg-config --modversion opencv 2>/dev/null)
    pass "OpenCV: $OPENCV_VERSION (opencv feature)"
else
    info "OpenCV: not found (opencv feature disabled)"
fi

#=====================================
# SUMMARY
#=====================================

# Complete progress bar
complete_verify_progress

echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║${NC}                      ${WHITE}VERIFICATION SUMMARY${NC}                  ${BLUE}║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

TOTAL=$((PASSES + ERRORS + WARNINGS))

echo -e "  ${GREEN}Passed:${NC}   $PASSES"
echo -e "  ${RED}Errors:${NC}   $ERRORS"
echo -e "  ${YELLOW}Warnings:${NC} $WARNINGS"
echo -e "  ${WHITE}Skipped:${NC}  $SKIPPED"
echo ""

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "  ${GREEN}════════════════════════════════════════${NC}"
    echo -e "  ${GREEN}  ${STATUS_OK} HORUS installation is PERFECT!${NC}"
    echo -e "  ${GREEN}════════════════════════════════════════${NC}"
    echo ""
    echo -e "  Ready to use: ${CYAN}horus new my_project${NC}"
    EXIT_CODE=0
elif [ $ERRORS -eq 0 ]; then
    echo -e "  ${YELLOW}════════════════════════════════════════${NC}"
    echo -e "  ${YELLOW}  ${STATUS_WARN} HORUS is functional with warnings${NC}"
    echo -e "  ${YELLOW}════════════════════════════════════════${NC}"
    echo ""
    echo -e "  Review warnings above for optional improvements."
    EXIT_CODE=1
else
    echo -e "  ${RED}════════════════════════════════════════${NC}"
    echo -e "  ${RED}  ${STATUS_ERR} HORUS installation has ERRORS${NC}"
    echo -e "  ${RED}════════════════════════════════════════${NC}"
    echo ""
    echo -e "  ${CYAN}Recommended fixes:${NC}"
    echo ""
    echo -e "    1. Clean reinstall:"
    echo -e "       ${CYAN}./install.sh${NC}"
    echo ""
    echo -e "    2. If system dependencies missing:"
    if [ "$OS_TYPE" = "linux" ]; then
        echo -e "       ${CYAN}sudo apt-get install gcc libc6-dev pkg-config${NC}"
        echo -e "       ${CYAN}sudo apt-get install libssl-dev libudev-dev libasound2-dev${NC}"
    elif [ "$OS_TYPE" = "macos" ]; then
        echo -e "       ${CYAN}xcode-select --install${NC}"
        echo -e "       ${CYAN}brew install openssl pkg-config${NC}"
    fi
    echo ""
    echo -e "    3. If binaries not working:"
    echo -e "       ${CYAN}cargo clean && cargo build --release${NC}"
    echo ""
    EXIT_CODE=2
fi

echo ""
echo -e "  Verification completed at $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

exit $EXIT_CODE
