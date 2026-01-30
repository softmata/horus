#!/bin/bash
# HORUS Installation Script
# Universal installer that works across all major operating systems
# Uses shared deps.sh for consistent dependency management

set -e  # Exit on error
set -o pipefail  # Fail on pipe errors

# Script version
SCRIPT_VERSION="2.6.0"

# Check bash version - we need bash 4+ for associative arrays
# Use explicit check that works even if BASH_VERSINFO is unset
BASH_MAJOR_VERSION="${BASH_VERSINFO[0]:-0}"
if [ -z "$BASH_MAJOR_VERSION" ] || [ "$BASH_MAJOR_VERSION" -lt 4 ] 2>/dev/null; then
    # On macOS, try to auto-detect and use Homebrew's bash
    if [[ "$(uname)" == "Darwin" ]]; then
        BREW_BASH=""
        # Check Apple Silicon path first, then Intel path
        if [[ -x "/opt/homebrew/bin/bash" ]]; then
            BREW_BASH="/opt/homebrew/bin/bash"
        elif [[ -x "/usr/local/bin/bash" ]]; then
            BREW_BASH="/usr/local/bin/bash"
        fi

        if [[ -n "$BREW_BASH" ]]; then
            # Verify it's bash 4+
            BREW_BASH_VERSION=$("$BREW_BASH" -c 'echo ${BASH_VERSINFO[0]}' 2>/dev/null || echo "0")
            if [[ "$BREW_BASH_VERSION" -ge 4 ]]; then
                echo "[*] macOS default bash is version ${BASH_VERSION} (too old)"
                echo "[*] Auto-detected Homebrew bash ${BREW_BASH_VERSION}.x at: $BREW_BASH"
                echo "[*] Re-executing script with Homebrew bash..."
                echo ""
                # Re-execute this script with the newer bash
                # Pass all original arguments and prevent infinite recursion with marker
                if [[ -z "$HORUS_REEXEC" ]]; then
                    export HORUS_REEXEC=1
                    exec "$BREW_BASH" "$0" "$@"
                fi
            fi
        fi

        # If we get here, no suitable bash was found
        echo "Error: This script requires bash version 4.0 or higher."
        echo "Your current bash version: ${BASH_VERSION}"
        echo ""
        echo "macOS ships with bash 3.2 by default. To fix this:"
        echo ""
        echo "  1. Install bash 4+ via Homebrew:"
        echo "     brew install bash"
        echo ""
        echo "  2. Then run this script again - it will auto-detect Homebrew's bash"
        echo "     ./install.sh"
        echo ""
        echo "  Or manually run with:"
        echo "     /opt/homebrew/bin/bash install.sh  (Apple Silicon)"
        echo "     /usr/local/bin/bash install.sh     (Intel Mac)"
        exit 1
    else
        echo "Error: This script requires bash version 4.0 or higher."
        echo "Your current bash version: ${BASH_VERSION}"
        echo ""
        echo "Please install bash 4.0 or higher and try again."
        exit 1
    fi
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Status indicators
STATUS_OK="[+]"
STATUS_ERR="[-]"
STATUS_WARN="[!]"
STATUS_INFO="[*]"

# Spinner function - simple dots
spin() {
    local pid=$1
    local msg="$2"
    local spin_chars=('.' '..' '...' '....')
    local i=0

    # Hide cursor
    tput civis 2>/dev/null || true

    while kill -0 $pid 2>/dev/null; do
        printf "\r  ${spin_chars[$i]} ${msg}"
        i=$(( (i + 1) % ${#spin_chars[@]} ))
        sleep 0.25
    done

    # Show cursor and clear line
    tput cnorm 2>/dev/null || true
    printf "\r\033[K"
}

# Build spinner - simple dots
spin_build() {
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
# PROGRESS BAR FUNCTIONS - Real progress with percentages and ETA
# ============================================================================

# Format seconds into human-readable duration
format_duration() {
    local seconds=$1
    if [ "$seconds" -lt 60 ]; then
        echo "${seconds}s"
    elif [ "$seconds" -lt 3600 ]; then
        local mins=$((seconds / 60))
        local secs=$((seconds % 60))
        echo "${mins}m ${secs}s"
    else
        local hours=$((seconds / 3600))
        local mins=$(((seconds % 3600) / 60))
        echo "${hours}h ${mins}m"
    fi
}

# Format bytes into human-readable size
format_bytes() {
    local bytes=$1
    if [ "$bytes" -lt 1024 ]; then
        echo "${bytes}B"
    elif [ "$bytes" -lt 1048576 ]; then
        echo "$((bytes / 1024))KB"
    elif [ "$bytes" -lt 1073741824 ]; then
        echo "$((bytes / 1048576))MB"
    else
        echo "$((bytes / 1073741824))GB"
    fi
}

# Draw a progress bar with percentage and ETA
# Usage: draw_progress_bar current total width message [start_time]
draw_progress_bar() {
    local current=$1
    local total=$2
    local width=${3:-30}
    local msg="$4"
    local start_time=${5:-$PROGRESS_START_TIME}

    # Calculate percentage
    local percent=0
    if [ "$total" -gt 0 ]; then
        percent=$((current * 100 / total))
    fi

    # Calculate filled width
    local filled=$((current * width / total))
    [ "$filled" -gt "$width" ] && filled=$width
    local empty=$((width - filled))

    # Build the bar
    local bar=""
    for ((j=0; j<filled; j++)); do bar+="█"; done
    for ((j=0; j<empty; j++)); do bar+="░"; done

    # Calculate ETA
    local eta_str=""
    if [ -n "$start_time" ] && [ "$current" -gt 0 ]; then
        local elapsed=$(($(date +%s) - start_time))
        if [ "$elapsed" -gt 0 ] && [ "$percent" -gt 0 ] && [ "$percent" -lt 100 ]; then
            local total_estimated=$((elapsed * 100 / percent))
            local remaining=$((total_estimated - elapsed))
            if [ "$remaining" -gt 0 ]; then
                eta_str=" ETA: $(format_duration $remaining)"
            fi
        fi
    fi

    # Print the progress bar
    printf "\r  ${STATUS_INFO} [${bar}] %3d%% ${msg}${eta_str}    " "$percent"
}

# Complete the progress bar (success)
complete_progress_bar() {
    local msg="$1"
    local width=${2:-30}
    local bar=""
    for ((j=0; j<width; j++)); do bar+="█"; done
    printf "\r  ${STATUS_OK} [${bar}] 100%% ${msg}    \n"
}

# Fail the progress bar (error)
fail_progress_bar() {
    local msg="$1"
    local percent=${2:-0}
    local width=${3:-30}
    local filled=$((percent * width / 100))
    local empty=$((width - filled))
    local bar=""
    for ((j=0; j<filled; j++)); do bar+="█"; done
    for ((j=0; j<empty; j++)); do bar+="░"; done
    printf "\r  ${STATUS_ERR} [${bar}] %3d%% ${msg}    \n" "$percent"
}

# Step-based progress tracker
# Usage: init_step_progress "Step1:weight" "Step2:weight" ...
STEP_NAMES=()
STEP_WEIGHTS=()
STEP_TOTAL_WEIGHT=0
STEP_CURRENT=0
STEP_COMPLETED_WEIGHT=0
STEP_START_TIME=0

init_step_progress() {
    STEP_NAMES=()
    STEP_WEIGHTS=()
    STEP_TOTAL_WEIGHT=0
    STEP_CURRENT=0
    STEP_COMPLETED_WEIGHT=0
    STEP_START_TIME=$(date +%s)

    for step_info in "$@"; do
        local name="${step_info%%:*}"
        local weight="${step_info##*:}"
        STEP_NAMES+=("$name")
        STEP_WEIGHTS+=("$weight")
        STEP_TOTAL_WEIGHT=$((STEP_TOTAL_WEIGHT + weight))
    done
}

# Start the next step
# Usage: next_step
next_step() {
    if [ "$STEP_CURRENT" -gt 0 ] && [ "$STEP_CURRENT" -le "${#STEP_WEIGHTS[@]}" ]; then
        local prev_idx=$((STEP_CURRENT - 1))
        STEP_COMPLETED_WEIGHT=$((STEP_COMPLETED_WEIGHT + STEP_WEIGHTS[prev_idx]))
    fi
    STEP_CURRENT=$((STEP_CURRENT + 1))
}

# Update step progress (0-100 within current step)
# Usage: update_step_progress percent message
update_step_progress() {
    local step_percent=$1
    local msg="$2"

    if [ "$STEP_CURRENT" -gt 0 ] && [ "$STEP_CURRENT" -le "${#STEP_WEIGHTS[@]}" ]; then
        local idx=$((STEP_CURRENT - 1))
        local step_weight=${STEP_WEIGHTS[$idx]}
        local step_contribution=$((step_weight * step_percent / 100))
        local total_progress=$((STEP_COMPLETED_WEIGHT + step_contribution))

        # Calculate overall percentage
        local overall_percent=0
        if [ "$STEP_TOTAL_WEIGHT" -gt 0 ]; then
            overall_percent=$((total_progress * 100 / STEP_TOTAL_WEIGHT))
        fi

        draw_progress_bar "$overall_percent" 100 30 "$msg" "$STEP_START_TIME"
    fi
}

# Complete current step
# Usage: complete_step message
complete_step() {
    local msg="$1"
    if [ "$STEP_CURRENT" -gt 0 ] && [ "$STEP_CURRENT" -le "${#STEP_WEIGHTS[@]}" ]; then
        update_step_progress 100 "$msg"
    fi
}

# Finish all steps successfully
finish_step_progress() {
    local msg="$1"
    complete_progress_bar "$msg" 30
}

# Source shared dependency functions (if available)
DEPS_SHARED=false
if [ -f "$SCRIPT_DIR/scripts/deps.sh" ]; then
    source "$SCRIPT_DIR/scripts/deps.sh"
    DEPS_SHARED=true
fi

# Log file for debugging
LOG_FILE="/tmp/horus_install_$(date +%Y%m%d_%H%M%S).log"
exec 2> >(tee -a "$LOG_FILE" >&2)

echo ""
echo -e "${CYAN}HORUS Installation Script v${SCRIPT_VERSION}${NC}"
echo ""

# Detect operating system
detect_os() {
    local os_type=""
    local os_distro=""

    if [[ "$OSTYPE" == "darwin"* ]]; then
        os_type="macos"
        os_distro="macos"
    elif [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "cygwin"* ]] || [[ "$OSTYPE" == "mingw"* ]]; then
        # Native Windows with Git Bash, MSYS2, or Cygwin
        os_type="windows"
        os_distro="windows"
    elif [[ "$OSTYPE" == "freebsd"* ]]; then
        os_type="bsd"
        os_distro="freebsd"
    elif [[ "$OSTYPE" == "openbsd"* ]]; then
        os_type="bsd"
        os_distro="openbsd"
    elif [[ "$OSTYPE" == "netbsd"* ]]; then
        os_type="bsd"
        os_distro="netbsd"
    elif [[ "$OSTYPE" == "dragonfly"* ]]; then
        os_type="bsd"
        os_distro="dragonfly"
    elif [[ "$OSTYPE" == "solaris"* ]] || [[ "$OSTYPE" == "sunos"* ]]; then
        os_type="solaris"
        os_distro="solaris"
    elif [[ "$OSTYPE" == "linux"* ]]; then
        os_type="linux"

        # Check for WSL
        if grep -qE "(Microsoft|WSL)" /proc/version 2>/dev/null; then
            os_type="wsl"
        fi

        # Detect Linux distribution
        if [ -f /etc/os-release ]; then
            . /etc/os-release
            os_distro="${ID,,}"

            # Group similar distros
            case "$os_distro" in
                ubuntu|debian|raspbian|pop|mint|elementary)
                    os_distro="debian-based"
                    ;;
                fedora|rhel|centos|rocky|almalinux)
                    os_distro="fedora-based"
                    ;;
                arch|manjaro|endeavouros)
                    os_distro="arch-based"
                    ;;
                opensuse*)
                    os_distro="opensuse"
                    ;;
                alpine)
                    os_distro="alpine"
                    ;;
                void)
                    os_distro="void"
                    ;;
                nixos)
                    os_distro="nixos"
                    ;;
                *)
                    os_distro="unknown"
                    ;;
            esac
        fi
    else
        os_type="unknown"
        os_distro="unknown"
    fi

    echo "$os_type:$os_distro"
}

# Use shared OS detection if available, otherwise use local function
if [ "$DEPS_SHARED" = true ] && [ -n "$OS_TYPE" ]; then
    # OS already detected by deps.sh
    echo -e "${CYAN}[i]${NC} Detected OS: $OS_TYPE ($OS_DISTRO)"
else
    OS_INFO=$(detect_os)
    IFS=':' read -r OS_TYPE OS_DISTRO <<< "$OS_INFO"
    echo -e "${CYAN}[i]${NC} Detected OS: $OS_TYPE ($OS_DISTRO)"
fi

# ============================================================================
# WINDOWS NATIVE CHECK - RECOMMEND WSL
# ============================================================================
# HORUS requires Unix-like environment. On Windows, WSL is required.
if [ "$OS_TYPE" = "windows" ]; then
    echo ""
    echo -e "${RED}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║                    WINDOWS NATIVE ENVIRONMENT DETECTED                       ║${NC}"
    echo -e "${RED}╠══════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${RED}║  HORUS requires a Unix-like environment and cannot run natively on Windows. ║${NC}"
    echo -e "${RED}║                                                                              ║${NC}"
    echo -e "${RED}║  Please use Windows Subsystem for Linux (WSL2) instead:                     ║${NC}"
    echo -e "${RED}║                                                                              ║${NC}"
    echo -e "${RED}║  1. Install WSL2:                                                            ║${NC}"
    echo -e "${RED}║     wsl --install                                                            ║${NC}"
    echo -e "${RED}║                                                                              ║${NC}"
    echo -e "${RED}║  2. Restart your computer                                                    ║${NC}"
    echo -e "${RED}║                                                                              ║${NC}"
    echo -e "${RED}║  3. Open WSL (Ubuntu) and run the install script there:                     ║${NC}"
    echo -e "${RED}║     curl -fsSL https://softmata.ai/install.sh | bash                        ║${NC}"
    echo -e "${RED}║                                                                              ║${NC}"
    echo -e "${RED}║  WSL provides full Linux compatibility with excellent performance.          ║${NC}"
    echo -e "${RED}║  HORUS will work exactly as it does on native Linux.                        ║${NC}"
    echo -e "${RED}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    exit 1
fi

# ============================================================================
# BSD SYSTEMS - EXPERIMENTAL SUPPORT
# ============================================================================
if [ "$OS_TYPE" = "bsd" ]; then
    echo ""
    echo -e "${YELLOW}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║                      BSD SYSTEM DETECTED: $OS_DISTRO                         ${NC}"
    echo -e "${YELLOW}╠══════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}║  HORUS has EXPERIMENTAL support for BSD systems.                            ║${NC}"
    echo -e "${YELLOW}║                                                                              ║${NC}"
    echo -e "${YELLOW}║  Known limitations:                                                          ║${NC}"
    echo -e "${YELLOW}║  - Some Linux-specific features may not work (e.g., /proc filesystem)       ║${NC}"
    echo -e "${YELLOW}║  - Package names differ from Linux (install manually if needed)             ║${NC}"
    echo -e "${YELLOW}║  - Shared memory uses different paths                                       ║${NC}"
    echo -e "${YELLOW}║  - Audio subsystem differs (OSS vs ALSA)                                    ║${NC}"
    echo -e "${YELLOW}║                                                                              ║${NC}"
    if [ "$OS_DISTRO" = "freebsd" ]; then
        echo -e "${YELLOW}║  FreeBSD prerequisites:                                                     ║${NC}"
        echo -e "${YELLOW}║    pkg install rust cmake pkgconf openssl llvm git                         ║${NC}"
    elif [ "$OS_DISTRO" = "openbsd" ]; then
        echo -e "${YELLOW}║  OpenBSD prerequisites:                                                     ║${NC}"
        echo -e "${YELLOW}║    pkg_add rust cmake pkgconf openssl llvm git                             ║${NC}"
    elif [ "$OS_DISTRO" = "netbsd" ]; then
        echo -e "${YELLOW}║  NetBSD prerequisites:                                                      ║${NC}"
        echo -e "${YELLOW}║    pkgin install rust cmake pkg-config openssl llvm git                    ║${NC}"
    fi
    echo -e "${YELLOW}║                                                                              ║${NC}"
    echo -e "${YELLOW}║  For best experience, Linux (native or VM) is recommended.                  ║${NC}"
    echo -e "${YELLOW}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}[?]${NC} Continue with experimental BSD support? [y/N]: "
    read -r BSD_CONTINUE
    if [[ ! "$BSD_CONTINUE" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Installation cancelled. Consider using Linux for full compatibility.${NC}"
        exit 0
    fi
    echo -e "${GREEN}[✓]${NC} Continuing with experimental BSD support..."
fi

# ============================================================================
# SOLARIS/ILLUMOS - NOT SUPPORTED
# ============================================================================
if [ "$OS_TYPE" = "solaris" ]; then
    echo ""
    echo -e "${RED}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║                    SOLARIS/ILLUMOS DETECTED - NOT SUPPORTED                  ║${NC}"
    echo -e "${RED}╠══════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${RED}║  HORUS does not currently support Solaris or illumos-based systems.         ║${NC}"
    echo -e "${RED}║                                                                              ║${NC}"
    echo -e "${RED}║  HORUS relies on Linux-specific features:                                   ║${NC}"
    echo -e "${RED}║  - POSIX shared memory semantics                                            ║${NC}"
    echo -e "${RED}║  - Linux proc filesystem                                                    ║${NC}"
    echo -e "${RED}║  - GNU toolchain compatibility                                              ║${NC}"
    echo -e "${RED}║                                                                              ║${NC}"
    echo -e "${RED}║  Alternatives:                                                              ║${NC}"
    echo -e "${RED}║  - Use a Linux VM (VirtualBox, bhyve, zones with Linux brand)              ║${NC}"
    echo -e "${RED}║  - Use a Linux container                                                    ║${NC}"
    echo -e "${RED}║  - Install on a Linux system                                                ║${NC}"
    echo -e "${RED}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    exit 1
fi

# ============================================================================
# UNKNOWN OS - WARN AND OFFER TO CONTINUE
# ============================================================================
if [ "$OS_TYPE" = "unknown" ]; then
    echo ""
    echo -e "${YELLOW}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║                    UNKNOWN OPERATING SYSTEM DETECTED                         ║${NC}"
    echo -e "${YELLOW}╠══════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}║  Could not detect your operating system.                                    ║${NC}"
    echo -e "${YELLOW}║  OSTYPE: $OSTYPE                                                            ${NC}"
    echo -e "${YELLOW}║                                                                              ║${NC}"
    echo -e "${YELLOW}║  HORUS is tested on:                                                        ║${NC}"
    echo -e "${YELLOW}║  - Linux (Ubuntu, Debian, Fedora, Arch, etc.)                              ║${NC}"
    echo -e "${YELLOW}║  - macOS (Intel and Apple Silicon)                                         ║${NC}"
    echo -e "${YELLOW}║  - Windows via WSL2                                                         ║${NC}"
    echo -e "${YELLOW}║                                                                              ║${NC}"
    echo -e "${YELLOW}║  You may continue, but expect potential compatibility issues.               ║${NC}"
    echo -e "${YELLOW}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}[?]${NC} Continue anyway? [y/N]: "
    read -r UNKNOWN_CONTINUE
    if [[ ! "$UNKNOWN_CONTINUE" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Installation cancelled.${NC}"
        exit 0
    fi
    echo -e "${GREEN}[✓]${NC} Continuing with unknown OS support..."
fi

# ============================================================================
# INSTALLATION PROFILE
# ============================================================================
# Single profile: builds all core packages.
# sim2d/sim3d are standalone packages - install via: horus pkg install sim2d
INSTALL_PROFILE="full"
export INSTALL_PROFILE

# ============================================================================
# SMART VERSION SOLVER - Automatic dependency resolution
# ============================================================================
# Features:
#   - Auto-installs exact required Rust version
#   - Parses build errors to detect version mismatches
#   - Checks system library minimum versions
#   - Auto-updates Cargo.lock when stale
#   - Resolves Python package version conflicts

# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║           COMPREHENSIVE VERSION REQUIREMENTS TABLE                         ║
# ╠═══════════════════════════════════════════════════════════════════════════╣
# ║ All version constraints are synchronized with:                             ║
# ║   - Cargo.toml (rust-version, dependency versions)                        ║
# ║   - pyproject.toml (requires-python, dependencies)                        ║
# ║   - CI/CD tested configurations                                           ║
# ║                                                                           ║
# ║ UPDATE THESE AFTER TESTING NEW RELEASES!                                  ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │ RUST VERSION REQUIREMENTS                                                   │
# │ - MSRV defined in Cargo.toml: rust-version = "1.92"                        │
# │ - Max tested: Update after CI passes on new Rust releases                  │
# └─────────────────────────────────────────────────────────────────────────────┘
REQUIRED_RUST_MAJOR=1
REQUIRED_RUST_MINOR=92
REQUIRED_RUST_VERSION="${REQUIRED_RUST_MAJOR}.${REQUIRED_RUST_MINOR}"
MAX_TESTED_RUST_MAJOR=1
MAX_TESTED_RUST_MINOR=92
MAX_TESTED_RUST_VERSION="${MAX_TESTED_RUST_MAJOR}.${MAX_TESTED_RUST_MINOR}"

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │ PYTHON VERSION REQUIREMENTS                                                 │
# │ - Min: Python 3.9 (PyO3 compatibility, type hints)                         │
# │ - Max tested: Python 3.13 (verify horus_py wheel builds)                   │
# └─────────────────────────────────────────────────────────────────────────────┘
REQUIRED_PYTHON_MAJOR=3
REQUIRED_PYTHON_MINOR=9
REQUIRED_PYTHON_VERSION="${REQUIRED_PYTHON_MAJOR}.${REQUIRED_PYTHON_MINOR}"
MAX_TESTED_PYTHON_MAJOR=3
MAX_TESTED_PYTHON_MINOR=13
MAX_TESTED_PYTHON_VERSION="${MAX_TESTED_PYTHON_MAJOR}.${MAX_TESTED_PYTHON_MINOR}"

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │ SYSTEM LIBRARY VERSION REQUIREMENTS                                         │
# │ - Minimum versions for FFI/bindgen compatibility                           │
# │ - Maximum versions: generally no ceiling (backwards compatible)            │
# └─────────────────────────────────────────────────────────────────────────────┘
declare -A REQUIRED_LIB_VERSIONS=(
    ["openssl"]="1.1.0"       # TLS/crypto - min for modern cipher suites
    ["libclang"]="11.0"       # bindgen - min for Rust 2021 edition parsing
    ["libudev"]="1.0"         # Device enumeration - systemd 220+
    ["alsa"]="1.1.0"          # Audio - ALSA lib 1.1.0+ for modern audio APIs
    ["x11"]="1.6.0"           # X11 - XCB integration
    ["wayland"]="1.18.0"      # Wayland - xdg-shell stable
)

# Maximum known-compatible versions (warn if newer - API might have changed)
declare -A MAX_TESTED_LIB_VERSIONS=(
    ["openssl"]="3.3.0"       # OpenSSL 3.x has API changes from 1.x
    ["libclang"]="18.0"       # LLVM 18 tested
)

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │ CARGO DEPENDENCY VERSION REQUIREMENTS                                       │
# │ - Key crates with known compatibility constraints                          │
# │ - Used for build error diagnosis and auto-resolution                       │
# └─────────────────────────────────────────────────────────────────────────────┘
declare -A CARGO_MIN_VERSIONS=(
    # Note: syn intentionally omitted - syn 1.x and 2.x coexist in modern Rust
    # ecosystems (proc-macro crates often depend on both transitively)
    ["proc-macro2"]="1.0"     # Token streams
    ["quote"]="1.0"           # Token generation
    ["serde"]="1.0"           # Serialization
    ["tokio"]="1.0"           # Async runtime
    ["pyo3"]="0.21"           # Python bindings
)

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │ PYTHON PACKAGE VERSION REQUIREMENTS                                         │
# │ - For horus_py and its dependencies                                        │
# └─────────────────────────────────────────────────────────────────────────────┘
declare -A PYTHON_MIN_VERSIONS=(
    ["numpy"]="1.21.0"        # Array operations
    ["setuptools"]="65.0.0"   # Build backend
)

# Compare semantic versions: returns 0 if v1 > v2, 1 otherwise (strict greater than)
version_gt() {
    local v1="$1"
    local v2="$2"

    # Validate inputs - handle empty or invalid version strings gracefully
    if [ -z "$v1" ] || [ -z "$v2" ]; then
        echo -e "${YELLOW}${STATUS_WARN} version_gt: empty version string (v1='$v1', v2='$v2')${NC}" >&2
        return 1
    fi

    # Strip any non-version characters (e.g., 'v' prefix, trailing text)
    v1=$(echo "$v1" | grep -oE '^[0-9]+(\.[0-9]+)*' | head -1)
    v2=$(echo "$v2" | grep -oE '^[0-9]+(\.[0-9]+)*' | head -1)

    if [ -z "$v1" ] || [ -z "$v2" ]; then
        echo -e "${YELLOW}${STATUS_WARN} version_gt: invalid version format${NC}" >&2
        return 1
    fi

    # Extract major.minor.patch (default to 0 if missing)
    # Note: cut returns the whole string if delimiter not found, so we must check
    local v1_major v1_minor v1_patch v2_major v2_minor v2_patch

    # Parse v1 - handle cases where dots may be missing
    v1_major=$(echo "$v1" | cut -d'.' -f1)
    if [[ "$v1" == *"."* ]]; then
        v1_minor=$(echo "$v1" | cut -d'.' -f2)
        if [[ "$v1" == *"."*"."* ]]; then
            v1_patch=$(echo "$v1" | cut -d'.' -f3)
        else
            v1_patch=0
        fi
    else
        v1_minor=0
        v1_patch=0
    fi

    # Parse v2 - handle cases where dots may be missing
    v2_major=$(echo "$v2" | cut -d'.' -f1)
    if [[ "$v2" == *"."* ]]; then
        v2_minor=$(echo "$v2" | cut -d'.' -f2)
        if [[ "$v2" == *"."*"."* ]]; then
            v2_patch=$(echo "$v2" | cut -d'.' -f3)
        else
            v2_patch=0
        fi
    else
        v2_minor=0
        v2_patch=0
    fi

    # Default empty values to 0
    v1_major=${v1_major:-0}
    v1_minor=${v1_minor:-0}
    v1_patch=${v1_patch:-0}
    v2_major=${v2_major:-0}
    v2_minor=${v2_minor:-0}
    v2_patch=${v2_patch:-0}

    # Use arithmetic expansion with default to 0 for safety
    if [ "${v1_major:-0}" -gt "${v2_major:-0}" ] 2>/dev/null; then return 0; fi
    if [ "${v1_major:-0}" -lt "${v2_major:-0}" ] 2>/dev/null; then return 1; fi
    if [ "${v1_minor:-0}" -gt "${v2_minor:-0}" ] 2>/dev/null; then return 0; fi
    if [ "${v1_minor:-0}" -lt "${v2_minor:-0}" ] 2>/dev/null; then return 1; fi
    if [ "${v1_patch:-0}" -gt "${v2_patch:-0}" ] 2>/dev/null; then return 0; fi
    return 1
}

# Compare semantic versions: returns 0 if v1 >= v2, 1 otherwise
version_gte() {
    local v1="$1"
    local v2="$2"

    # Validate inputs - handle empty or invalid version strings gracefully
    if [ -z "$v1" ] || [ -z "$v2" ]; then
        echo -e "${YELLOW}${STATUS_WARN} version_gte: empty version string (v1='$v1', v2='$v2')${NC}" >&2
        return 1
    fi

    # Strip any non-version characters (e.g., 'v' prefix, trailing text)
    v1=$(echo "$v1" | grep -oE '^[0-9]+(\.[0-9]+)*' | head -1)
    v2=$(echo "$v2" | grep -oE '^[0-9]+(\.[0-9]+)*' | head -1)

    if [ -z "$v1" ] || [ -z "$v2" ]; then
        echo -e "${YELLOW}${STATUS_WARN} version_gte: invalid version format${NC}" >&2
        return 1
    fi

    # Extract major.minor.patch (default to 0 if missing)
    # Note: cut returns the whole string if delimiter not found, so we must check
    local v1_major v1_minor v1_patch v2_major v2_minor v2_patch

    # Parse v1 - handle cases where dots may be missing
    v1_major=$(echo "$v1" | cut -d'.' -f1)
    if [[ "$v1" == *"."* ]]; then
        v1_minor=$(echo "$v1" | cut -d'.' -f2)
        if [[ "$v1" == *"."*"."* ]]; then
            v1_patch=$(echo "$v1" | cut -d'.' -f3)
        else
            v1_patch=0
        fi
    else
        v1_minor=0
        v1_patch=0
    fi

    # Parse v2 - handle cases where dots may be missing
    v2_major=$(echo "$v2" | cut -d'.' -f1)
    if [[ "$v2" == *"."* ]]; then
        v2_minor=$(echo "$v2" | cut -d'.' -f2)
        if [[ "$v2" == *"."*"."* ]]; then
            v2_patch=$(echo "$v2" | cut -d'.' -f3)
        else
            v2_patch=0
        fi
    else
        v2_minor=0
        v2_patch=0
    fi

    # Default empty values to 0
    v1_major=${v1_major:-0}
    v1_minor=${v1_minor:-0}
    v1_patch=${v1_patch:-0}
    v2_major=${v2_major:-0}
    v2_minor=${v2_minor:-0}
    v2_patch=${v2_patch:-0}

    # Use arithmetic expansion with default to 0 for safety
    if [ "${v1_major:-0}" -gt "${v2_major:-0}" ] 2>/dev/null; then return 0; fi
    if [ "${v1_major:-0}" -lt "${v2_major:-0}" ] 2>/dev/null; then return 1; fi
    if [ "${v1_minor:-0}" -gt "${v2_minor:-0}" ] 2>/dev/null; then return 0; fi
    if [ "${v1_minor:-0}" -lt "${v2_minor:-0}" ] 2>/dev/null; then return 1; fi
    if [ "${v1_patch:-0}" -ge "${v2_patch:-0}" ] 2>/dev/null; then return 0; fi
    return 1
}

# Auto-install or upgrade Rust to required version
# Also checks for versions that are TOO NEW (untested, may have breaking API changes)
install_rust() {
    local need_install=false
    local need_upgrade=false
    local version_too_new=false
    local current_version=""

    if ! command -v cargo &> /dev/null; then
        need_install=true
    else
        current_version=$(rustc --version | awk '{print $2}')
        local rust_major=$(echo $current_version | cut -d'.' -f1)
        local rust_minor=$(echo $current_version | cut -d'.' -f2)

        # Check if version is too OLD
        if [ "$rust_major" -lt "$REQUIRED_RUST_MAJOR" ] || \
           ([ "$rust_major" -eq "$REQUIRED_RUST_MAJOR" ] && [ "$rust_minor" -lt "$REQUIRED_RUST_MINOR" ]); then
            need_upgrade=true
        fi

        # Check if version is too NEW (untested)
        if [ "$rust_major" -gt "$MAX_TESTED_RUST_MAJOR" ] || \
           ([ "$rust_major" -eq "$MAX_TESTED_RUST_MAJOR" ] && [ "$rust_minor" -gt "$MAX_TESTED_RUST_MINOR" ]); then
            version_too_new=true
        fi
    fi

    if [ "$need_install" = true ]; then
        echo -e "${YELLOW}${STATUS_WARN} Rust is not installed${NC}"
        read -p "$(echo -e ${CYAN}?${NC}) Install Rust $REQUIRED_RUST_VERSION automatically? [Y/n]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            echo -e "${CYAN}${STATUS_INFO} Installing Rust $REQUIRED_RUST_VERSION...${NC}"
            curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain "$REQUIRED_RUST_VERSION"
            source "$HOME/.cargo/env"

            if ! command -v cargo &> /dev/null; then
                echo -e "${RED}${STATUS_ERR} Failed to install Rust${NC}"
                echo "Please install manually from https://rustup.rs/"
                exit 1
            fi
            echo -e "${GREEN}${STATUS_OK} Rust $REQUIRED_RUST_VERSION installed successfully${NC}"
        else
            echo -e "${RED}${STATUS_ERR} Rust is required to build HORUS${NC}"
            exit 1
        fi
    elif [ "$need_upgrade" = true ]; then
        echo -e "${YELLOW}${STATUS_WARN} Rust version $current_version is too old (requires >= $REQUIRED_RUST_VERSION)${NC}"
        read -p "$(echo -e ${CYAN}?${NC}) Upgrade Rust to $REQUIRED_RUST_VERSION automatically? [Y/n]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            echo -e "${CYAN}${STATUS_INFO} Installing Rust $REQUIRED_RUST_VERSION...${NC}"

            # Install specific version and set as default
            if rustup install "$REQUIRED_RUST_VERSION" 2>&1; then
                rustup default "$REQUIRED_RUST_VERSION"
                source "$HOME/.cargo/env" 2>/dev/null || true

                # Verify upgrade
                local new_version=$(rustc --version | awk '{print $2}')
                local new_major=$(echo $new_version | cut -d'.' -f1)
                local new_minor=$(echo $new_version | cut -d'.' -f2)

                if [ "$new_major" -ge "$REQUIRED_RUST_MAJOR" ] && [ "$new_minor" -ge "$REQUIRED_RUST_MINOR" ]; then
                    echo -e "${GREEN}${STATUS_OK} Rust upgraded: $current_version → $new_version${NC}"
                else
                    echo -e "${RED}${STATUS_ERR} Upgrade failed. Please run manually:${NC}"
                    echo -e "   ${CYAN}rustup install $REQUIRED_RUST_VERSION && rustup default $REQUIRED_RUST_VERSION${NC}"
                    exit 1
                fi
            else
                echo -e "${RED}${STATUS_ERR} Failed to install Rust $REQUIRED_RUST_VERSION${NC}"
                echo -e "   Try: ${CYAN}rustup update stable${NC}"
                exit 1
            fi
        else
            echo -e "${RED}${STATUS_ERR} Rust >= $REQUIRED_RUST_VERSION is required to build HORUS${NC}"
            exit 1
        fi
    elif [ "$version_too_new" = true ]; then
        # Version is newer than tested - warn user about potential API incompatibility
        echo ""
        echo -e "${YELLOW}${STATUS_WARN} WARNING: Rust $current_version is NEWER than tested version range${NC}"
        echo -e "${YELLOW}   HORUS is tested with Rust $REQUIRED_RUST_VERSION - $MAX_TESTED_RUST_VERSION${NC}"
        echo ""
        echo -e "${YELLOW}   Newer Rust versions may have breaking API changes that cause build failures.${NC}"
        echo -e "${YELLOW}   If the build fails, consider switching to a tested version.${NC}"
        echo ""
        echo -e "   Options:"
        echo -e "     1. ${GREEN}Continue${NC} with Rust $current_version (may work, but untested)"
        echo -e "     2. ${CYAN}Switch${NC} to tested version $MAX_TESTED_RUST_VERSION"
        echo -e "     3. ${RED}Abort${NC} installation"
        echo ""
        read -p "$(echo -e ${CYAN}?${NC}) Choose [1/2/3] (default: 1): " -n 1 -r
        echo

        case "$REPLY" in
            2)
                echo -e "${CYAN}${STATUS_INFO} Switching to Rust $MAX_TESTED_RUST_VERSION...${NC}"
                if rustup install "$MAX_TESTED_RUST_VERSION" 2>&1; then
                    rustup default "$MAX_TESTED_RUST_VERSION"
                    source "$HOME/.cargo/env" 2>/dev/null || true
                    local switched_version=$(rustc --version | awk '{print $2}')
                    echo -e "${GREEN}${STATUS_OK} Switched to Rust $switched_version${NC}"
                else
                    echo -e "${RED}${STATUS_ERR} Failed to install Rust $MAX_TESTED_RUST_VERSION${NC}"
                    echo -e "   Continuing with $current_version..."
                fi
                ;;
            3)
                echo -e "${RED}${STATUS_ERR} Installation aborted by user${NC}"
                exit 1
                ;;
            *)
                echo -e "${YELLOW}${STATUS_WARN} Continuing with untested Rust $current_version${NC}"
                echo -e "   If build fails, run: ${CYAN}rustup install $MAX_TESTED_RUST_VERSION && rustup default $MAX_TESTED_RUST_VERSION${NC}"
                ;;
        esac
    fi
}

# Install Rust (with smart version solving)
install_rust

# Display final Rust version
RUST_VERSION=$(rustc --version | awk '{print $2}')
echo -e "${GREEN}${STATUS_OK} Rust version $RUST_VERSION meets requirement (>= $REQUIRED_RUST_VERSION)${NC}"

# Auto-install system dependencies
install_system_deps() {
    local missing_deps=""

    # Check C compiler
    if ! command -v cc &> /dev/null && ! command -v gcc &> /dev/null && ! command -v clang &> /dev/null; then
        missing_deps="compiler"
    fi

    # Check pkg-config
    if ! command -v pkg-config &> /dev/null; then
        missing_deps="$missing_deps pkg-config"
    fi

    # Check for OpenSSL
    if ! pkg-config --exists openssl 2>/dev/null && [ "$OS_TYPE" != "macos" ]; then
        missing_deps="$missing_deps openssl"
    fi

    # Check for libudev (Linux only)
    if [ "$OS_TYPE" = "linux" ] || [ "$OS_TYPE" = "wsl" ]; then
        if ! pkg-config --exists libudev 2>/dev/null; then
            missing_deps="$missing_deps libudev"
        fi
    fi

    # Check for ALSA (Linux only)
    if [ "$OS_TYPE" = "linux" ] || [ "$OS_TYPE" = "wsl" ]; then
        if ! pkg-config --exists alsa 2>/dev/null; then
            missing_deps="$missing_deps alsa"
        fi
    fi

    # Check for libclang (required for OpenCV)
    if ! ldconfig -p 2>/dev/null | grep -q libclang && [ "$OS_TYPE" != "macos" ]; then
        missing_deps="$missing_deps libclang"
    fi

    if [ -n "$missing_deps" ]; then
        echo -e "${YELLOW} Missing dependencies:${missing_deps}${NC}"
        echo ""
        read -p "$(echo -e ${CYAN}?${NC}) Install missing dependencies automatically? [Y/n]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            echo -e "${CYAN} Installing dependencies for $OS_DISTRO...${NC}"

            case "$OS_DISTRO" in
                debian-based)
                    sudo apt-get update
                    # Note: gcc is sufficient, g++/build-essential not needed for Rust
                    # Note: pipx is for Python CLI tools (Ubuntu 24+ recommended)
                    sudo apt-get install -y gcc libc6-dev pkg-config libssl-dev libudev-dev libasound2-dev \
                        libclang-dev libopencv-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev \
                        libxinerama-dev libwayland-dev wayland-protocols libxkbcommon-dev pipx
                    # Ensure pipx path is available
                    pipx ensurepath 2>/dev/null || true
                    ;;
                fedora-based)
                    # Note: gcc is sufficient, Development Tools group includes C++ which is not needed
                    sudo dnf install -y gcc glibc-devel pkg-config openssl-devel systemd-devel alsa-lib-devel \
                        clang-devel opencv-devel libX11-devel libXrandr-devel libXi-devel \
                        libXcursor-devel libXinerama-devel wayland-devel wayland-protocols-devel \
                        libxkbcommon-devel pipx
                    pipx ensurepath 2>/dev/null || true
                    ;;
                arch-based)
                    # Note: gcc is sufficient, base-devel includes C++ which is not needed
                    sudo pacman -Sy --noconfirm gcc pkg-config openssl systemd alsa-lib \
                        clang opencv libx11 libxrandr libxi libxcursor libxinerama \
                        wayland wayland-protocols libxkbcommon python-pipx
                    pipx ensurepath 2>/dev/null || true
                    ;;
                opensuse)
                    # Note: gcc is sufficient, devel_basis includes C++ which is not needed
                    sudo zypper install -y gcc glibc-devel pkg-config libopenssl-devel libudev-devel alsa-devel \
                        clang-devel opencv-devel libX11-devel libXrandr-devel libXi-devel \
                        libXcursor-devel libXinerama-devel wayland-devel wayland-protocols-devel \
                        libxkbcommon-devel python3-pipx
                    pipx ensurepath 2>/dev/null || true
                    ;;
                alpine)
                    # Note: gcc and musl-dev are sufficient for Rust
                    sudo apk add --no-cache gcc musl-dev pkgconfig openssl-dev eudev-dev alsa-lib-dev \
                        clang-dev opencv-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev \
                        libxinerama-dev wayland-dev wayland-protocols libxkbcommon-dev py3-pipx
                    pipx ensurepath 2>/dev/null || true
                    ;;
                macos)
                    # Check for Xcode Command Line Tools
                    if ! xcode-select -p &> /dev/null; then
                        echo -e "${CYAN} Installing Xcode Command Line Tools...${NC}"
                        xcode-select --install
                        echo "Please wait for Xcode tools to install, then re-run this script"
                        exit 1
                    fi
                    # Install via Homebrew
                    if ! command -v brew &> /dev/null; then
                        echo -e "${YELLOW} Homebrew not found${NC}"
                        echo "Please install from https://brew.sh then re-run this script"
                        exit 1
                    fi
                    # Install dependencies for macOS:
                    # - pkg-config: library detection
                    # - openssl: TLS/crypto (macOS uses LibreSSL by default)
                    # - llvm: provides libclang for bindgen
                    # - opencv: computer vision (optional)
                    # - cmake: build system for some native deps
                    # - pipx: Python CLI tools
                    brew install pkg-config openssl llvm opencv cmake pipx

                    # Set environment variables for OpenSSL and LLVM
                    # These are needed because Homebrew doesn't link these by default
                    echo -e "${CYAN}  Note: You may need to set these environment variables:${NC}"
                    echo -e "  export OPENSSL_DIR=\"\$(brew --prefix openssl)\""
                    echo -e "  export LIBCLANG_PATH=\"\$(brew --prefix llvm)/lib\""

                    pipx ensurepath 2>/dev/null || true
                    ;;
                *)
                    echo -e "${YELLOW} Cannot auto-install for $OS_DISTRO${NC}"
                    echo ""
                    echo "Please install manually:"
                    echo "  - C compiler (gcc or clang) - C++ is NOT required"
                    echo "  - pkg-config"
                    echo "  - OpenSSL development headers"
                    echo "  - libudev development headers (Linux)"
                    echo "  - ALSA development headers (Linux)"
                    echo "  - libclang development headers"
                    echo "  - OpenCV development headers (optional)"
                    echo ""
                    exit 1
                    ;;
            esac

            echo -e "${GREEN} Dependencies installed${NC}"
        else
            echo -e "${YELLOW} Continuing without installing dependencies${NC}"
            echo "Note: Build may fail if required dependencies are missing"
        fi
    else
        echo -e "${GREEN} All required dependencies found${NC}"
    fi
}

# Check and install system dependencies
# Use shared deps.sh function if available for consistency
if [ "$DEPS_SHARED" = true ]; then
    echo -e "${CYAN}[*]${NC} Checking system dependencies (using shared deps.sh)..."
    MISSING=$(check_all_deps)
    if [ -n "$MISSING" ]; then
        echo -e "${YELLOW}[!]${NC} Missing: $(get_missing_deps_readable)"
        install_system_deps
    else
        echo -e "${GREEN}[+]${NC} All system dependencies found"
    fi
else
    install_system_deps
fi

echo -e "${CYAN}${NC} Detected C compiler: $(cc --version 2>/dev/null | head -n1 || gcc --version 2>/dev/null | head -n1 || clang --version 2>/dev/null | head -n1)"
echo -e "${CYAN}${NC} Detected pkg-config: $(pkg-config --version)"

# ============================================================================
# SYSTEM LIBRARY VERSION CHECKING - Comprehensive min/max validation
# ============================================================================

# Get library version from pkg-config
get_lib_version() {
    local lib="$1"
    pkg-config --modversion "$lib" 2>/dev/null || echo "0.0.0"
}

# Check library exists AND meets version requirements (min and optionally max)
# Returns: "missing", "old:VERSION", "new:VERSION", or "VERSION" (OK)
check_lib_version_full() {
    local lib="$1"
    local min_version="${REQUIRED_LIB_VERSIONS[$lib]:-0.0.0}"
    local max_version="${MAX_TESTED_LIB_VERSIONS[$lib]:-}"

    if ! pkg-config --exists "$lib" 2>/dev/null; then
        echo "missing"
        return 1
    fi

    local current=$(get_lib_version "$lib")

    # Check minimum version
    if ! version_gte "$current" "$min_version"; then
        echo "old:$current:$min_version"
        return 1
    fi

    # Check maximum version (if defined)
    if [ -n "$max_version" ] && version_gt "$current" "$max_version"; then
        echo "new:$current:$max_version"
        return 2  # Warning, not error
    fi

    echo "$current"
    return 0
}

# Display library check result with appropriate formatting
display_lib_result() {
    local lib="$1"
    local display_name="$2"
    local result="$3"

    case "$result" in
        missing)
            echo -e "${YELLOW}${STATUS_WARN}  $display_name: not found${NC}"
            MISSING_LIBS="${MISSING_LIBS} $lib"
            ;;
        old:*)
            local parts=(${result//:/ })
            local ver="${parts[1]}"
            local min="${parts[2]}"
            echo -e "${YELLOW}${STATUS_WARN}  $display_name $ver is too old (requires >= $min)${NC}"
            OLD_LIBS="${OLD_LIBS} $lib"
            ;;
        new:*)
            local parts=(${result//:/ })
            local ver="${parts[1]}"
            local max="${parts[2]}"
            echo -e "${YELLOW}${STATUS_WARN}  $display_name $ver is NEWER than tested ($max)${NC}"
            NEW_LIBS="${NEW_LIBS} $lib"
            ;;
        *)
            echo -e "${GREEN}${STATUS_OK}  $display_name $result${NC}"
            ;;
    esac
}

# Check for required system libraries WITH COMPREHENSIVE VERSION CHECKING
echo ""
echo -e "${CYAN}${STATUS_INFO} Checking system dependencies (with version requirements)...${NC}"
echo -e "${CYAN}   Min versions from REQUIRED_LIB_VERSIONS, max from MAX_TESTED_LIB_VERSIONS${NC}"
echo ""

MISSING_LIBS=""
OLD_LIBS=""
NEW_LIBS=""

# Core libraries with version requirements
display_lib_result "openssl" "OpenSSL" "$(check_lib_version_full openssl || true)"
display_lib_result "libudev" "libudev" "$(check_lib_version_full libudev || true)"

# ALSA check (Linux only - macOS uses CoreAudio)
if [ "$OS_TYPE" = "linux" ] || [ "$OS_TYPE" = "wsl" ]; then
    if ! pkg-config --exists alsa 2>/dev/null; then
        echo -e "${YELLOW}${STATUS_WARN}  ALSA: not found${NC}"
        MISSING_LIBS="${MISSING_LIBS} alsa"
    else
        ALSA_VER=$(get_lib_version "alsa")
        ALSA_MIN="${REQUIRED_LIB_VERSIONS[alsa]:-1.1.0}"
        if version_gte "$ALSA_VER" "$ALSA_MIN"; then
            echo -e "${GREEN}${STATUS_OK}  ALSA $ALSA_VER${NC}"
        else
            echo -e "${YELLOW}${STATUS_WARN}  ALSA $ALSA_VER is too old (requires >= $ALSA_MIN)${NC}"
            OLD_LIBS="${OLD_LIBS} alsa"
        fi
    fi
elif [ "$OS_TYPE" = "macos" ]; then
    echo -e "${GREEN}${STATUS_OK}  CoreAudio (macOS native audio)${NC}"
fi

# Check libclang version (critical for bindgen)
LIBCLANG_VERSION=""
if command -v llvm-config &>/dev/null; then
    LIBCLANG_VERSION=$(llvm-config --version 2>/dev/null | cut -d'.' -f1-2)
elif [ -f /usr/lib/llvm-*/bin/llvm-config ]; then
    # Linux: check system LLVM (use sed instead of grep -P for BSD compatibility)
    LIBCLANG_VERSION=$(ls -d /usr/lib/llvm-* 2>/dev/null | tail -1 | sed 's/.*llvm-//' | sed 's/[^0-9].*//')
elif [ "$OS_TYPE" = "macos" ]; then
    # macOS: check Homebrew LLVM (both Intel and Apple Silicon paths)
    if [ -f "/opt/homebrew/opt/llvm/bin/llvm-config" ]; then
        LIBCLANG_VERSION=$(/opt/homebrew/opt/llvm/bin/llvm-config --version 2>/dev/null | cut -d'.' -f1-2)
    elif [ -f "/usr/local/opt/llvm/bin/llvm-config" ]; then
        LIBCLANG_VERSION=$(/usr/local/opt/llvm/bin/llvm-config --version 2>/dev/null | cut -d'.' -f1-2)
    fi
fi

LIBCLANG_MIN="${REQUIRED_LIB_VERSIONS[libclang]:-11.0}"
LIBCLANG_MAX="${MAX_TESTED_LIB_VERSIONS[libclang]:-}"

if [ -n "$LIBCLANG_VERSION" ]; then
    if ! version_gte "$LIBCLANG_VERSION" "$LIBCLANG_MIN"; then
        echo -e "${YELLOW}${STATUS_WARN}  libclang $LIBCLANG_VERSION is too old (requires >= $LIBCLANG_MIN)${NC}"
        OLD_LIBS="${OLD_LIBS} libclang"
    elif [ -n "$LIBCLANG_MAX" ] && version_gt "$LIBCLANG_VERSION" "$LIBCLANG_MAX"; then
        echo -e "${YELLOW}${STATUS_WARN}  libclang $LIBCLANG_VERSION is NEWER than tested ($LIBCLANG_MAX)${NC}"
        NEW_LIBS="${NEW_LIBS} libclang"
    else
        echo -e "${GREEN}${STATUS_OK}  libclang $LIBCLANG_VERSION${NC}"
    fi
else
    # Try to detect from ldconfig (Linux only) or check for library files
    if ldconfig -p 2>/dev/null | grep -q libclang; then
        echo -e "${GREEN}${STATUS_OK}  libclang found (version unknown)${NC}"
    elif [ "$OS_TYPE" = "macos" ] && ([ -f "/opt/homebrew/lib/libclang.dylib" ] || [ -f "/usr/local/lib/libclang.dylib" ]); then
        echo -e "${GREEN}${STATUS_OK}  libclang found (version unknown, Homebrew)${NC}"
    else
        echo -e "${YELLOW}${STATUS_WARN}  libclang not found (needed for some FFI bindings)${NC}"
        MISSING_LIBS="${MISSING_LIBS} libclang"
    fi
fi

# GUI/Graphics libraries (required for monitor TUI)
if [ "$(uname -s)" = "Linux" ]; then
    if ! pkg-config --exists x11 2>/dev/null; then
        echo -e "${YELLOW}${NC}  X11 development library not found"
        MISSING_LIBS="${MISSING_LIBS} x11"
    fi

    if ! pkg-config --exists xrandr 2>/dev/null; then
        echo -e "${YELLOW}${NC}  Xrandr development library not found"
        MISSING_LIBS="${MISSING_LIBS} xrandr"
    fi

    if ! pkg-config --exists xi 2>/dev/null; then
        echo -e "${YELLOW}${NC}  Xi (X11 Input) development library not found"
        MISSING_LIBS="${MISSING_LIBS} xi"
    fi

    if ! pkg-config --exists xcursor 2>/dev/null; then
        echo -e "${YELLOW}${NC}  Xcursor development library not found"
        MISSING_LIBS="${MISSING_LIBS} xcursor"
    fi

    if ! pkg-config --exists wayland-client 2>/dev/null; then
        echo -e "${YELLOW}${NC}  Wayland development library not found"
        MISSING_LIBS="${MISSING_LIBS} wayland"
    fi

    if ! pkg-config --exists xkbcommon 2>/dev/null; then
        echo -e "${YELLOW}${NC}  xkbcommon development library not found"
        MISSING_LIBS="${MISSING_LIBS} xkbcommon"
    fi
fi

# Optional but recommended libraries
OPTIONAL_MISSING=""

if ! pkg-config --exists fontconfig 2>/dev/null; then
    echo -e "${YELLOW}${NC}  fontconfig not found (optional - improves text rendering)"
    OPTIONAL_MISSING="${OPTIONAL_MISSING} fontconfig"
fi

if [ ! -z "$MISSING_LIBS" ]; then
    echo ""
    echo -e "${RED} Missing REQUIRED system libraries!${NC}"
    echo ""
    echo "Please install the following packages:"
    echo ""
    echo -e "${CYAN}Ubuntu/Debian/Raspberry Pi OS:${NC}"
    echo "  sudo apt update"
    echo "  sudo apt install -y gcc libc6-dev pkg-config \\"
    echo "    libssl-dev libudev-dev libasound2-dev \\"
    echo "    libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev \\"
    echo "    libwayland-dev wayland-protocols libxkbcommon-dev \\"
    echo "    libvulkan-dev libfontconfig-dev libfreetype-dev"
    echo ""
    echo -e "${CYAN}Fedora/RHEL/CentOS:${NC}"
    echo "  sudo dnf install -y gcc glibc-devel pkg-config openssl-devel systemd-devel alsa-lib-devel \\"
    echo "    libX11-devel libXrandr-devel libXi-devel libXcursor-devel libXinerama-devel \\"
    echo "    wayland-devel wayland-protocols-devel libxkbcommon-devel \\"
    echo "    vulkan-devel fontconfig-devel freetype-devel"
    echo ""
    echo -e "${CYAN}Arch Linux:${NC}"
    echo "  sudo pacman -S gcc pkg-config openssl systemd alsa-lib \\"
    echo "    libx11 libxrandr libxi libxcursor libxinerama \\"
    echo "    wayland wayland-protocols libxkbcommon \\"
    echo "    vulkan-icd-loader fontconfig freetype2"
    echo ""
    echo -e "${CYAN}macOS:${NC}"
    echo "  xcode-select --install"
    echo "  brew install pkg-config openssl llvm cmake"
    echo ""
    echo "  # Set environment variables (add to ~/.zshrc or ~/.bash_profile):"
    echo "  export OPENSSL_DIR=\"\$(brew --prefix openssl)\""
    echo "  export LIBCLANG_PATH=\"\$(brew --prefix llvm)/lib\""
    echo ""

    # Platform-specific notes
    if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null || grep -q "BCM" /proc/cpuinfo 2>/dev/null; then
        echo -e "${CYAN}Raspberry Pi detected - Additional packages:${NC}"
        echo "  sudo apt install -y libraspberrypi-dev i2c-tools python3-smbus"
        echo ""
        echo -e "${CYAN}Enable hardware interfaces (I2C, SPI, Serial):${NC}"
        echo "  sudo raspi-config"
        echo "  # Navigate to: Interface Options → I2C → Enable"
        echo "  # Navigate to: Interface Options → SPI → Enable"
        echo "  # Navigate to: Interface Options → Serial Port → Enable"
        echo ""
    fi

    if [ -f "/etc/nv_tegra_release" ] || grep -q "tegra" /proc/cpuinfo 2>/dev/null; then
        echo -e "${CYAN}NVIDIA Jetson detected - Additional packages:${NC}"
        echo "  sudo apt install -y nvidia-jetpack"
        echo "  # For GPU acceleration, ensure CUDA toolkit is installed"
        echo ""
    fi

    exit 1
fi

echo -e "${GREEN}${NC} All required system dependencies found"

if [ ! -z "$OPTIONAL_MISSING" ]; then
    echo -e "${YELLOW}${NC} Some optional dependencies missing (font support may be limited)"
fi

# ============================================================================
# PYTHON VERSION CHECK - with min/max range validation
# ============================================================================
# Check if Python is installed and within tested version range

check_python_version() {
    if ! command -v python3 &> /dev/null; then
        echo -e "${YELLOW}${STATUS_WARN} Python3 not found - horus_py will be skipped${NC}"
        echo -e "   Requires Python $REQUIRED_PYTHON_VERSION - $MAX_TESTED_PYTHON_VERSION"
        PYTHON_AVAILABLE=false
        return
    fi

    PYTHON_VERSION=$(python3 --version | awk '{print $2}')
    echo -e "${CYAN}${STATUS_INFO} Detected Python: $PYTHON_VERSION${NC}"

    PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
    PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

    local version_too_old=false
    local version_too_new=false

    # Check if version is too OLD
    if [ "$PYTHON_MAJOR" -lt "$REQUIRED_PYTHON_MAJOR" ] || \
       ([ "$PYTHON_MAJOR" -eq "$REQUIRED_PYTHON_MAJOR" ] && [ "$PYTHON_MINOR" -lt "$REQUIRED_PYTHON_MINOR" ]); then
        version_too_old=true
    fi

    # Check if version is too NEW (untested)
    if [ "$PYTHON_MAJOR" -gt "$MAX_TESTED_PYTHON_MAJOR" ] || \
       ([ "$PYTHON_MAJOR" -eq "$MAX_TESTED_PYTHON_MAJOR" ] && [ "$PYTHON_MINOR" -gt "$MAX_TESTED_PYTHON_MINOR" ]); then
        version_too_new=true
    fi

    if [ "$version_too_old" = true ]; then
        echo -e "${YELLOW}${STATUS_WARN} Python $PYTHON_VERSION is too old (requires >= $REQUIRED_PYTHON_VERSION)${NC}"
        echo -e "   horus_py will be skipped"
        echo ""
        echo -e "   To use Python bindings, install Python $REQUIRED_PYTHON_VERSION+:"
        echo -e "     ${CYAN}Ubuntu/Debian:${NC} sudo apt install python3.11 python3.11-dev"
        echo -e "     ${CYAN}pyenv:${NC} pyenv install 3.11 && pyenv global 3.11"
        echo -e "     ${CYAN}conda:${NC} conda create -n horus python=3.11 && conda activate horus"
        PYTHON_AVAILABLE=false
    elif [ "$version_too_new" = true ]; then
        echo ""
        echo -e "${YELLOW}${STATUS_WARN} WARNING: Python $PYTHON_VERSION is NEWER than tested range${NC}"
        echo -e "${YELLOW}   HORUS horus_py is tested with Python $REQUIRED_PYTHON_VERSION - $MAX_TESTED_PYTHON_VERSION${NC}"
        echo ""
        echo -e "${YELLOW}   Newer Python versions may have incompatible C API changes.${NC}"
        echo -e "${YELLOW}   Pre-built wheels may not be available for this version.${NC}"
        echo ""
        echo -e "   Options:"
        echo -e "     1. ${GREEN}Continue${NC} with Python $PYTHON_VERSION (may work, but untested)"
        echo -e "     2. ${RED}Skip${NC} horus_py installation"
        echo ""
        read -p "$(echo -e ${CYAN}?${NC}) Choose [1/2] (default: 1): " -n 1 -r
        echo

        case "$REPLY" in
            2)
                echo -e "${YELLOW}${STATUS_WARN} Skipping horus_py due to untested Python version${NC}"
                PYTHON_AVAILABLE=false
                ;;
            *)
                echo -e "${YELLOW}${STATUS_WARN} Continuing with untested Python $PYTHON_VERSION${NC}"
                PYTHON_AVAILABLE=true
                ;;
        esac
    else
        # Version is within tested range
        echo -e "${GREEN}${STATUS_OK} Python version within tested range ($REQUIRED_PYTHON_VERSION - $MAX_TESTED_PYTHON_VERSION)${NC}"
        PYTHON_AVAILABLE=true
    fi
}

check_python_version

# Check for pip (needed for Python bindings)
if [ "$PYTHON_AVAILABLE" = true ]; then
    if command -v pip3 &> /dev/null || command -v pip &> /dev/null; then
        PIP_VERSION=$(pip3 --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1 || echo "0.0")
        echo -e "${CYAN}${STATUS_INFO} Detected pip: $PIP_VERSION${NC}"
    else
        echo -e "${YELLOW}${STATUS_WARN}  pip not found - horus_py will be skipped"
        echo "  Install pip: sudo apt install python3-pip (Debian/Ubuntu)"
        PYTHON_AVAILABLE=false
    fi
fi

# ============================================================================
# PYTHON PACKAGE VERSION CHECKING - Comprehensive validation
# ============================================================================

# Get installed Python package version
get_pip_package_version() {
    local pkg="$1"
    pip3 show "$pkg" 2>/dev/null | grep -i "^version:" | awk '{print $2}' || echo ""
}

# Check Python package version meets requirements
check_python_package() {
    local pkg="$1"
    local min_version="${PYTHON_MIN_VERSIONS[$pkg]:-}"

    if [ -z "$min_version" ]; then
        return 0  # No version requirement
    fi

    local installed=$(get_pip_package_version "$pkg")

    if [ -z "$installed" ]; then
        echo "missing"
        return 1
    fi

    if version_gte "$installed" "$min_version"; then
        echo "$installed"
        return 0
    else
        echo "old:$installed:$min_version"
        return 1
    fi
}

# Auto-upgrade Python package if needed
upgrade_python_package() {
    local pkg="$1"
    local min_version="$2"

    echo -e "${CYAN}   Upgrading $pkg to >= $min_version...${NC}"

    # Try different pip install methods (some systems restrict --user, some require --break-system-packages)
    if pip3 install "$pkg>=$min_version" --user --upgrade --quiet 2>/dev/null; then
        echo -e "${GREEN}   ✓ Upgraded $pkg successfully${NC}"
        return 0
    elif pip3 install "$pkg>=$min_version" --upgrade --quiet 2>/dev/null; then
        echo -e "${GREEN}   ✓ Upgraded $pkg successfully${NC}"
        return 0
    elif pip3 install "$pkg>=$min_version" --user --break-system-packages --upgrade --quiet 2>/dev/null; then
        echo -e "${GREEN}   ✓ Upgraded $pkg successfully (break-system-packages)${NC}"
        return 0
    else
        echo -e "${YELLOW}   ⚠ Could not auto-upgrade $pkg${NC}"
        return 1
    fi
}

# Check required Python packages (if Python is available)
if [ "$PYTHON_AVAILABLE" = true ]; then
    echo ""
    echo -e "${CYAN}${STATUS_INFO} Checking Python package requirements...${NC}"

    PY_PKG_ISSUES=""

    for pkg in "${!PYTHON_MIN_VERSIONS[@]}"; do
        result=$(check_python_package "$pkg" || true)
        min_ver="${PYTHON_MIN_VERSIONS[$pkg]}"

        case "$result" in
            missing)
                echo -e "${YELLOW}${STATUS_WARN}  $pkg: not installed (will install >= $min_ver)${NC}"
                PY_PKG_ISSUES="${PY_PKG_ISSUES} $pkg"
                ;;
            old:*)
                parts=(${result//:/ })
                installed="${parts[1]}"
                echo -e "${YELLOW}${STATUS_WARN}  $pkg $installed is too old (requires >= $min_ver)${NC}"
                read -p "$(echo -e ${CYAN}?${NC}) Auto-upgrade $pkg? [Y/n]: " -n 1 -r
                echo
                if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
                    upgrade_python_package "$pkg" "$min_ver"
                else
                    PY_PKG_ISSUES="${PY_PKG_ISSUES} $pkg"
                fi
                ;;
            *)
                echo -e "${GREEN}${STATUS_OK}  $pkg $result${NC}"
                ;;
        esac
    done

    if [ -n "$PY_PKG_ISSUES" ]; then
        echo ""
        echo -e "${YELLOW}${STATUS_WARN} Some Python packages need attention: $PY_PKG_ISSUES${NC}"
        echo -e "${YELLOW}   horus_py build may fail - packages will be installed during build${NC}"
    fi
fi

echo ""

# Determine installation paths
INSTALL_DIR="$HOME/.cargo/bin"
CACHE_DIR="$HOME/.horus/cache"

echo -e "${CYAN}${NC} Installation paths:"
echo "  CLI binary: $INSTALL_DIR/horus"
echo "  Libraries:  $CACHE_DIR/"
echo ""

# Ask for confirmation
read -p "$(echo -e ${YELLOW}?${NC}) Proceed with installation? [Y/n]: " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]] && [[ ! -z $REPLY ]]; then
    echo -e "${RED}${NC} Installation cancelled"
    exit 0
fi

# =============================================================================
# STEP 0: COMPLETE CLEAN - Remove ALL stale artifacts
# =============================================================================
echo ""
echo -e "${CYAN}${NC} Performing complete clean installation..."
echo -e "${CYAN}   ${NC} This ensures a fresh build with no stale artifacts"
echo ""

# Clean target directory (compiled artifacts)
if [ -d "target" ]; then
    echo -e "${CYAN}  ${NC} Removing target/ directory..."
    # Use || true to prevent script exit if some files are locked
    # This can happen if cargo/rustc is running or an IDE has files open
    if rm -rf target/ 2>/dev/null; then
        echo -e "${GREEN}  ${NC} Removed target/"
    else
        # Try harder: kill any cargo/rustc processes that might be holding locks
        echo -e "${YELLOW}  ${NC} Some files in use, attempting cleanup..."
        pkill -9 -f "cargo|rustc|rust-analyzer" 2>/dev/null || true
        sleep 1
        rm -rf target/ 2>/dev/null || true
        if [ -d "target" ]; then
            echo -e "${YELLOW}  ${NC} Could not fully remove target/ (some files in use)"
            echo -e "${CYAN}     ${NC} Build will continue with existing artifacts"
        else
            echo -e "${GREEN}  ${NC} Removed target/"
        fi
    fi
fi

# Clean HORUS cache directory
if [ -d "$HOME/.horus/cache" ]; then
    echo -e "${CYAN}  ${NC} Removing ~/.horus/cache/..."
    rm -rf "$HOME/.horus/cache" 2>/dev/null || true
    if [ ! -d "$HOME/.horus/cache" ]; then
        echo -e "${GREEN}  ${NC} Removed ~/.horus/cache/"
    else
        echo -e "${YELLOW}  ${NC} Could not fully remove ~/.horus/cache/"
    fi
fi

# Clean installed binaries
BINARIES_TO_CLEAN=("horus" "sim2d" "sim3d" "horus_router")
for binary in "${BINARIES_TO_CLEAN[@]}"; do
    if [ -f "$HOME/.cargo/bin/$binary" ]; then
        echo -e "${CYAN}  ${NC} Removing ~/.cargo/bin/$binary..."
        rm -f "$HOME/.cargo/bin/$binary" 2>/dev/null || true
        if [ ! -f "$HOME/.cargo/bin/$binary" ]; then
            echo -e "${GREEN}  ${NC} Removed $binary"
        else
            echo -e "${YELLOW}  ${NC} Could not remove $binary (in use?)"
        fi
    fi
done

# Clean stale shared memory sessions
if [ -d "/dev/shm/horus" ]; then
    echo -e "${CYAN}  ${NC} Removing stale shared memory sessions..."
    rm -rf /dev/shm/horus/ 2>/dev/null || true
    if [ ! -d "/dev/shm/horus" ]; then
        echo -e "${GREEN}  ${NC} Removed /dev/shm/horus/"
    else
        echo -e "${YELLOW}  ${NC} Could not fully remove /dev/shm/horus/"
    fi
elif [ -d "/tmp/horus" ]; then
    # macOS
    echo -e "${CYAN}  ${NC} Removing stale shared memory sessions..."
    rm -rf /tmp/horus/ 2>/dev/null || true
    if [ ! -d "/tmp/horus" ]; then
        echo -e "${GREEN}  ${NC} Removed /tmp/horus/"
    else
        echo -e "${YELLOW}  ${NC} Could not fully remove /tmp/horus/"
    fi
fi

# Clean Cargo incremental build cache (can cause issues)
if [ -d "$HOME/.cargo/registry/cache" ]; then
    echo -e "${CYAN}  ${NC} Cleaning Cargo registry cache..."
    # Only remove horus-related cached crates, not all crates
    find "$HOME/.cargo/registry/cache" -name "horus*" -exec rm -rf {} + 2>/dev/null || true
    echo -e "${GREEN}  ${NC} Cleaned horus-related Cargo cache"
fi

echo ""
echo -e "${GREEN}${NC} Clean complete - starting fresh build"
echo ""

# ============================================================================
# GITHUB ISSUE CREATION - Automatic bug report on install failure
# ============================================================================

# Create a GitHub issue for install failure (requires gh CLI)
create_github_issue() {
    local log_file="$1"

    # Check if gh CLI is available
    if ! command -v gh &>/dev/null; then
        echo -e "${YELLOW}${STATUS_WARN} GitHub CLI (gh) not installed. Cannot create issue automatically.${NC}"
        echo "  Install with: brew install gh (macOS) or apt install gh (Linux)"
        echo "  Then run: gh auth login"
        return 1
    fi

    # Check if user is authenticated
    if ! gh auth status &>/dev/null; then
        echo -e "${YELLOW}${STATUS_WARN} Not logged into GitHub CLI. Cannot create issue.${NC}"
        echo "  Run: gh auth login"
        return 1
    fi

    # Gather system info
    local os_info=$(uname -a 2>/dev/null || echo "Unknown")
    local rust_version=$(rustc --version 2>/dev/null || echo "Unknown")
    local cargo_version=$(cargo --version 2>/dev/null || echo "Unknown")
    local script_ver="$SCRIPT_VERSION"
    local profile="${INSTALL_PROFILE:-full}"

    # Extract last 100 lines of errors from log (sanitize paths)
    local error_log=""
    if [ -f "$log_file" ]; then
        error_log=$(tail -100 "$log_file" 2>/dev/null | sed "s|$HOME|~|g" | head -80)
    fi

    # Create issue body
    local issue_body="## Installation Failure Report

**Install Script Version:** $script_ver
**Installation Profile:** $profile

### System Information
- **OS:** $os_info
- **Rust:** $rust_version
- **Cargo:** $cargo_version

### Error Log (last 80 lines)
\`\`\`
$error_log
\`\`\`

### Steps to Reproduce
1. Clone the repository
2. Run \`./install.sh\`
3. Build fails after $max_retries retry attempts

---
*This issue was automatically created by the HORUS install script.*"

    local issue_title="[Install Failure] Build failed on $(uname -s) - $(date +%Y-%m-%d)"

    echo -e "${CYAN}${STATUS_INFO} Creating GitHub issue...${NC}"

    # Create the issue
    if gh issue create \
        --repo "softmata/horus" \
        --title "$issue_title" \
        --body "$issue_body" \
        --label "bug,install" 2>/dev/null; then
        echo -e "${GREEN}${STATUS_OK} GitHub issue created successfully!${NC}"
        return 0
    else
        echo -e "${RED}${STATUS_ERR} Failed to create GitHub issue.${NC}"
        echo "  You can manually report at: https://github.com/softmata/horus/issues"
        return 1
    fi
}

# Prompt user to create GitHub issue on failure
offer_github_issue() {
    local log_file="$1"

    echo ""
    echo -e "${CYAN}Would you like to automatically create a GitHub issue to report this failure?${NC}"
    echo -e "${YELLOW}This will include system info and error logs (no personal data).${NC}"
    echo ""
    read -p "Create GitHub issue? [y/N]: " -n 1 -r
    echo ""

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        create_github_issue "$log_file"
    else
        echo "  You can manually report at: https://github.com/softmata/horus/issues"
    fi
}

# ============================================================================
# SMART BUILD ERROR PARSER - Detects and auto-resolves common failures
# ============================================================================

# Parse cargo build errors and suggest/apply fixes
parse_build_error() {
    local log_file="$1"
    local error_type="unknown"
    local fix_applied=false

    # Check for common error patterns
    if grep -q "error\[E0308\].*expected.*found" "$log_file" 2>/dev/null; then
        error_type="type_mismatch"
    elif grep -qE "requires rustc [0-9]+\.[0-9]+|rustc [0-9]+\.[0-9]+(\.[0-9]+)? is not supported" "$log_file" 2>/dev/null; then
        error_type="rust_version"
        # Extract the problematic crate and required version
        local crate_info=$(grep -oE "[a-z_-]+@[0-9]+\.[0-9]+\.[0-9]+ requires rustc [0-9]+\.[0-9]+" "$log_file" 2>/dev/null | head -1)
        if [ -n "$crate_info" ]; then
            # Parse crate name, current version, and required rustc version
            local crate_name=$(echo "$crate_info" | sed 's/@.*//')
            local crate_ver=$(echo "$crate_info" | grep -oE '@[0-9]+\.[0-9]+\.[0-9]+' | tr -d '@')
            local required_rustc=$(echo "$crate_info" | grep -oE 'rustc [0-9]+\.[0-9]+' | sed 's/rustc //')

            echo -e "${YELLOW}${STATUS_WARN} Crate $crate_name@$crate_ver requires Rust $required_rustc but you have $(rustc --version | cut -d' ' -f2)${NC}"

            # Option 1: Downgrade the crate to a compatible version
            echo -e "${CYAN}${STATUS_INFO} Attempting to downgrade $crate_name to a compatible version...${NC}"

            # Try progressively older versions
            local major_ver=$(echo "$crate_ver" | cut -d. -f1)
            local minor_ver=$(echo "$crate_ver" | cut -d. -f2)
            local patch_ver=$(echo "$crate_ver" | cut -d. -f3)

            # Try previous patch versions first
            for try_patch in $(seq $((patch_ver - 1)) -1 0); do
                local try_ver="${major_ver}.${minor_ver}.${try_patch}"
                if cargo update "${crate_name}@${crate_ver}" --precise "$try_ver" 2>/dev/null; then
                    echo -e "${GREEN}${STATUS_OK} Downgraded $crate_name to $try_ver${NC}"
                    fix_applied=true
                    break
                fi
            done

            # If patch downgrade failed, try previous minor versions
            if [ "$fix_applied" != true ] && [ "$minor_ver" -gt 0 ]; then
                for try_minor in $(seq $((minor_ver - 1)) -1 0); do
                    local try_ver="${major_ver}.${try_minor}.0"
                    if cargo update "${crate_name}@${crate_ver}" --precise "$try_ver" 2>/dev/null; then
                        echo -e "${GREEN}${STATUS_OK} Downgraded $crate_name to $try_ver${NC}"
                        fix_applied=true
                        break
                    fi
                done
            fi

            if [ "$fix_applied" != true ]; then
                echo -e "${YELLOW}${STATUS_WARN} Could not find compatible version of $crate_name${NC}"
                echo -e "${CYAN}${STATUS_INFO} You may need to upgrade Rust: rustup update${NC}"
            fi
        else
            # Fallback to original behavior - suggest upgrading Rust
            local required_ver=$(grep -oE 'requires rustc [0-9]+\.[0-9]+' "$log_file" | grep -oE '[0-9]+\.[0-9]+' | head -1)
            if [ -n "$required_ver" ]; then
                echo -e "${YELLOW}${STATUS_WARN} Build requires Rust $required_ver${NC}"
                if version_gte "$required_ver" "$REQUIRED_RUST_VERSION"; then
                    echo -e "${CYAN}${STATUS_INFO} Auto-upgrading Rust to $required_ver...${NC}"
                    if rustup install "$required_ver" && rustup default "$required_ver"; then
                        fix_applied=true
                        echo -e "${GREEN}${STATUS_OK} Rust upgraded to $required_ver${NC}"
                    fi
                fi
            fi
        fi
    elif grep -q "failed to select a version for" "$log_file" 2>/dev/null; then
        error_type="version_conflict"
        echo -e "${YELLOW}${STATUS_WARN} Dependency version conflict detected${NC}"
        echo -e "${CYAN}${STATUS_INFO} Running cargo update to resolve...${NC}"
        if cargo update 2>/dev/null; then
            fix_applied=true
            echo -e "${GREEN}${STATUS_OK} Cargo.lock updated${NC}"
        fi
    elif grep -q "Blocking waiting for file lock" "$log_file" 2>/dev/null; then
        error_type="lock_file"
        echo -e "${YELLOW}${STATUS_WARN} Cargo lock file held by another process${NC}"
        echo -e "${CYAN}${STATUS_INFO} Waiting for lock to release...${NC}"
        sleep 5
        fix_applied=true
    elif grep -q "no space left on device" "$log_file" 2>/dev/null; then
        error_type="disk_space"
        echo -e "${RED}${STATUS_ERR} Disk space exhausted!${NC}"
        echo "  Free up space and try again"
    elif grep -q "could not compile" "$log_file" 2>/dev/null && grep -q "aborting due to" "$log_file" 2>/dev/null; then
        error_type="compile_error"
        # Check if it's a known fixable error
        if grep -q "perhaps a crate was updated" "$log_file" 2>/dev/null; then
            echo -e "${YELLOW}${STATUS_WARN} Crate version mismatch - lockfile may be stale${NC}"
            echo -e "${CYAN}${STATUS_INFO} Regenerating Cargo.lock...${NC}"
            rm -f Cargo.lock
            if cargo generate-lockfile 2>/dev/null; then
                fix_applied=true
                echo -e "${GREEN}${STATUS_OK} Cargo.lock regenerated${NC}"
            fi
        fi
    elif grep -q "linker .* not found" "$log_file" 2>/dev/null; then
        error_type="linker_missing"
        echo -e "${RED}${STATUS_ERR} Linker not found - install build tools${NC}"
        echo "  Ubuntu/Debian: sudo apt install build-essential"
        echo "  Fedora: sudo dnf install gcc"
    elif grep -qE "openssl|ssl" "$log_file" 2>/dev/null && grep -q "error" "$log_file" 2>/dev/null; then
        error_type="openssl"
        echo -e "${YELLOW}${STATUS_WARN} OpenSSL-related build error${NC}"
        echo -e "${CYAN}${STATUS_INFO} Try: export OPENSSL_DIR=/usr/local${NC}"
    elif grep -q "GLIBC" "$log_file" 2>/dev/null; then
        error_type="glibc"
        echo -e "${RED}${STATUS_ERR} GLIBC version incompatibility${NC}"
        echo "  Your system's glibc may be too old for some pre-built dependencies"
    fi

    if [ "$fix_applied" = true ]; then
        echo "$error_type:fixed"
        return 0
    else
        echo "$error_type:not_fixed"
        return 1
    fi
}

# Check and refresh Cargo.lock if stale
check_lockfile_freshness() {
    local lockfile="Cargo.lock"
    local cargo_toml="Cargo.toml"

    if [ ! -f "$lockfile" ]; then
        echo -e "${CYAN}${STATUS_INFO} No Cargo.lock found - generating...${NC}"
        cargo generate-lockfile 2>/dev/null
        return 0
    fi

    # Check if any Cargo.toml is newer than Cargo.lock
    local lockfile_mtime=$(stat -c %Y "$lockfile" 2>/dev/null || stat -f %m "$lockfile" 2>/dev/null)
    local needs_update=false

    # Find all Cargo.toml files and check mtimes
    while IFS= read -r toml; do
        local toml_mtime=$(stat -c %Y "$toml" 2>/dev/null || stat -f %m "$toml" 2>/dev/null)
        if [ "$toml_mtime" -gt "$lockfile_mtime" ]; then
            needs_update=true
            break
        fi
    done < <(find . -name "Cargo.toml" -not -path "./target/*" 2>/dev/null)

    if [ "$needs_update" = true ]; then
        echo -e "${YELLOW}${STATUS_WARN} Cargo.lock is older than Cargo.toml files${NC}"
        echo -e "${CYAN}${STATUS_INFO} Updating Cargo.lock to match current dependencies...${NC}"
        if cargo update 2>/dev/null; then
            echo -e "${GREEN}${STATUS_OK} Cargo.lock updated${NC}"
        fi
    else
        echo -e "${GREEN}${STATUS_OK} Cargo.lock is up to date${NC}"
    fi
}

# ============================================================================
# CARGO DEPENDENCY VERSION VALIDATION
# ============================================================================
# Validates that critical crates in Cargo.lock meet minimum version requirements
# This catches version issues BEFORE the expensive build step

# Get crate version from Cargo.lock
get_cargo_lock_version() {
    local crate_name="$1"
    local lockfile="Cargo.lock"

    if [ ! -f "$lockfile" ]; then
        echo ""
        return 1
    fi

    # Parse Cargo.lock TOML format to find crate version
    # Format: [[package]] name = "crate_name" version = "X.Y.Z"
    grep -A 2 "name = \"$crate_name\"" "$lockfile" 2>/dev/null | \
        grep "^version = " | head -1 | \
        sed 's/version = "\([^"]*\)"/\1/'
}

# Validate Cargo dependencies against CARGO_MIN_VERSIONS
validate_cargo_dependencies() {
    echo ""
    echo -e "${CYAN}${STATUS_INFO} Validating Cargo dependency versions...${NC}"

    local issues=""
    local lockfile="Cargo.lock"

    if [ ! -f "$lockfile" ]; then
        echo -e "${YELLOW}${STATUS_WARN} No Cargo.lock yet - will validate after generation${NC}"
        return 0
    fi

    for crate in "${!CARGO_MIN_VERSIONS[@]}"; do
        local min_ver="${CARGO_MIN_VERSIONS[$crate]}"
        local current_ver=$(get_cargo_lock_version "$crate")

        if [ -z "$current_ver" ]; then
            # Crate not in lockfile (might be optional or not used)
            continue
        fi

        if version_gte "$current_ver" "$min_ver"; then
            echo -e "${GREEN}${STATUS_OK}  $crate $current_ver (>= $min_ver)${NC}"
        else
            echo -e "${YELLOW}${STATUS_WARN}  $crate $current_ver is below minimum $min_ver${NC}"
            issues="${issues} $crate"
        fi
    done

    if [ -n "$issues" ]; then
        echo ""
        echo -e "${YELLOW}${STATUS_WARN} Some crate versions are below recommended minimums:$issues${NC}"
        echo -e "${CYAN}${STATUS_INFO} Running 'cargo update' to fetch newer versions...${NC}"

        if cargo update 2>/dev/null; then
            echo -e "${GREEN}${STATUS_OK} Dependencies updated${NC}"

            # Re-check if issues are resolved
            local still_issues=""
            for crate in $issues; do
                local min_ver="${CARGO_MIN_VERSIONS[$crate]}"
                local new_ver=$(get_cargo_lock_version "$crate")
                if [ -n "$new_ver" ] && ! version_gte "$new_ver" "$min_ver"; then
                    still_issues="${still_issues} $crate"
                fi
            done

            if [ -n "$still_issues" ]; then
                echo -e "${YELLOW}${STATUS_WARN} Some crates still at older versions:$still_issues${NC}"
                echo -e "${YELLOW}   This may be due to other crate requirements. Build may still succeed.${NC}"
            fi
        else
            echo -e "${YELLOW}${STATUS_WARN} cargo update failed - build may fail${NC}"
        fi
    else
        echo -e "${GREEN}${STATUS_OK} All critical dependencies meet version requirements${NC}"
    fi
}

# Build with automatic retry and error recovery
# Shows BOTH overall progress bar AND per-submodule progress bars
build_with_recovery() {
    local max_retries=3
    local retry=0

    # Pre-build checks: ensure dependencies are correct
    check_lockfile_freshness
    validate_cargo_dependencies

    # Define packages to build (order matters: dependencies first)
    BUILD_PACKAGES=(
        "horus_macros"
        "horus_core"
        "horus"
        "horus_ai"
        "horus_perception"
        "horus_ros2_bridge"
        "horus_manager"
        "horus_library"
        "horus_py"
    )
    echo -e "${GREEN}   Building all packages${NC}"
    echo -e "${CYAN}   sim2d/sim3d available via registry: horus pkg install sim2d${NC}"

    # Estimated crate counts for each package (for progress calculation)
    # Based on actual dependency counts from cargo tree (fresh build)
    declare -A PACKAGE_CRATES=(
        ["horus_macros"]=10
        ["horus_core"]=350
        ["horus"]=50
        ["horus_ai"]=20
        ["horus_perception"]=15
        ["horus_ros2_bridge"]=80
        ["horus_manager"]=150
        ["horus_library"]=50
        ["horus_py"]=20
    )

    # Calculate total crates for overall progress
    local TOTAL_ALL_CRATES=0
    for pkg in "${BUILD_PACKAGES[@]}"; do
        TOTAL_ALL_CRATES=$((TOTAL_ALL_CRATES + PACKAGE_CRATES[$pkg]))
    done

    # Package status tracking (global for function access)
    declare -A PKG_STATUS  # "pending", "building", "done", "failed"
    declare -A PKG_TIME    # Build time in seconds
    declare -A PKG_PERCENT # Current build percentage
    declare -A PKG_RETRIES # Per-package retry count
    local PKG_MAX_RETRIES=2  # Max retries per individual package

    # Note: horus_py is installed from PyPI, not built from source
    # Note: horus_router is part of horus_library (not a separate binary)

    local total_packages=${#BUILD_PACKAGES[@]}

    while [ $retry -lt $max_retries ]; do
        echo ""
        echo -e "${CYAN}   Building HORUS packages (attempt $((retry + 1))/$max_retries)...${NC}"
        echo -e "${CYAN}   Skipping: benchmarks, tanksim, horus_router${NC}"
        echo ""

        # Clean build on retry
        if [ $retry -gt 0 ]; then
            echo -e "${CYAN}${STATUS_INFO} Cleaning previous build artifacts...${NC}"
            cargo clean
        fi

        local OVERALL_START=$(date +%s)
        local TEMP_OUTPUT="/tmp/horus_build_output_$$.txt"
        local all_succeeded=true
        local current_package_num=0
        local OVERALL_CRATE_COUNT=0

        # Initialize status tracking
        declare -A PKG_CRATE  # Current crate being compiled
        declare -A PKG_ETA    # ETA string
        for pkg in "${BUILD_PACKAGES[@]}"; do
            PKG_STATUS[$pkg]="pending"
            PKG_TIME[$pkg]=0
            PKG_PERCENT[$pkg]=0
            PKG_CRATE[$pkg]=""
            PKG_ETA[$pkg]=""
            # Only initialize retry count on first global attempt
            if [ $retry -eq 0 ]; then
                PKG_RETRIES[$pkg]=0
            fi
        done

        # Hide cursor for clean progress display
        tput civis 2>/dev/null || true

        # Print overall progress header (fixed position - we'll update this in place)
        echo ""
        echo -e "  ${MAGENTA}Overall Progress:${NC}"
        # Line where overall bar lives - we'll track this
        local OVERALL_BAR_LINE=0
        OVERALL_BAR_LINE=$(tput lines 2>/dev/null || echo 24)  # Current line
        echo -e "  [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░]   0%"
        echo ""

        # Track how many lines down we are from the overall bar
        local LINES_BELOW_OVERALL=1  # Start 1 line below (the blank line)

        # Build each package sequentially
        for pkg in "${BUILD_PACKAGES[@]}"; do
            current_package_num=$((current_package_num + 1))
            PKG_STATUS[$pkg]="building"
            PKG_PERCENT[$pkg]=0

            local PKG_START=$(date +%s)
            local CRATE_COUNT=0
            local TOTAL_CRATES=${PACKAGE_CRATES[$pkg]:-50}

            # Print package header on new line
            echo -e "  ${STATUS_INFO} [${current_package_num}/${total_packages}] Building ${CYAN}${pkg}${NC}..."
            LINES_BELOW_OVERALL=$((LINES_BELOW_OVERALL + 1))

            # Run cargo build for this package and track progress
            # Use process substitution to avoid subshell variable scope issues
            local build_status=0
            local last_line=""
            while IFS= read -r line; do
                last_line="$line"
                echo "$line" >> "$LOG_FILE"

                # Parse Compiling lines for progress
                if [[ "$line" =~ ^[[:space:]]*Compiling[[:space:]]+([a-zA-Z0-9_-]+) ]]; then
                    CRATE_COUNT=$((CRATE_COUNT + 1))
                    OVERALL_CRATE_COUNT=$((OVERALL_CRATE_COUNT + 1))
                    local CURRENT_CRATE="${BASH_REMATCH[1]}"

                    # Calculate package progress
                    local percent=$((CRATE_COUNT * 100 / TOTAL_CRATES))
                    [ "$percent" -gt 99 ] && percent=99  # Cap at 99% until done
                    PKG_PERCENT[$pkg]=$percent

                    # Calculate overall progress
                    local overall_percent=$((OVERALL_CRATE_COUNT * 100 / TOTAL_ALL_CRATES))
                    [ "$overall_percent" -gt 99 ] && overall_percent=99

                    # Calculate ETAs
                    local elapsed=$(($(date +%s) - PKG_START))
                    local pkg_eta=""
                    if [ "$elapsed" -gt 1 ] && [ "$percent" -gt 5 ]; then
                        local total_estimated=$((elapsed * 100 / percent))
                        local remaining=$((total_estimated - elapsed))
                        [ "$remaining" -gt 0 ] && pkg_eta="ETA: $(format_duration $remaining)"
                    fi

                    local overall_elapsed=$(($(date +%s) - OVERALL_START))
                    local overall_eta=""
                    if [ "$overall_elapsed" -gt 2 ] && [ "$overall_percent" -gt 3 ]; then
                        local total_est=$((overall_elapsed * 100 / overall_percent))
                        local remain=$((total_est - overall_elapsed))
                        [ "$remain" -gt 0 ] && overall_eta="ETA: $(format_duration $remain)"
                    fi

                    # Build overall progress bar (40 chars wide) for top bar
                    local overall_width=40
                    local overall_filled=$((overall_percent * overall_width / 100))
                    local overall_empty=$((overall_width - overall_filled))
                    local overall_bar=""
                    for ((k=0; k<overall_filled; k++)); do overall_bar+="█"; done
                    for ((k=0; k<overall_empty; k++)); do overall_bar+="░"; done

                    # Build package progress bar (20 chars wide)
                    local pkg_width=20
                    local pkg_filled=$((percent * pkg_width / 100))
                    local pkg_empty=$((pkg_width - pkg_filled))
                    local pkg_bar=""
                    for ((k=0; k<pkg_filled; k++)); do pkg_bar+="█"; done
                    for ((k=0; k<pkg_empty; k++)); do pkg_bar+="░"; done

                    # Truncate crate name to fit
                    local crate_display="$CURRENT_CRATE"
                    [ ${#crate_display} -gt 16 ] && crate_display="${crate_display:0:13}..."

                    # Update overall progress bar at top
                    # Move up LINES_BELOW_OVERALL+1 lines (to get to the bar line)
                    local up_count=$((LINES_BELOW_OVERALL + 1))
                    printf "\033[%dA" "$up_count"  # Move up to overall bar
                    printf "\r  [${MAGENTA}${overall_bar}${NC}]%3d%% ${overall_eta}\033[K" "$overall_percent"
                    printf "\033[%dB" "$up_count"  # Move back down

                    # Update current package progress on current line
                    printf "\r\033[K    [${CYAN}${pkg_bar}${NC}]%3d%% ${crate_display}" "$percent"
                fi

                # Check for errors - catch multiple error formats
                if [[ "$line" =~ ^error\[E ]] || [[ "$line" =~ ^error:.*requires\ rustc ]] || [[ "$line" =~ "is not supported by the following package" ]]; then
                    echo "$line" >> "$TEMP_OUTPUT"
                    build_status=1
                fi
            done < <(if [ "$pkg" = "horus_manager" ]; then cargo build --release -p "$pkg" --bin horus; else cargo build --release -p "$pkg"; fi 2>&1; echo "BUILD_EXIT_CODE:$?")

            # Extract exit code from the marker line
            if [[ "$last_line" =~ BUILD_EXIT_CODE:([0-9]+) ]]; then
                build_status="${BASH_REMATCH[1]}"
            fi

            # For horus_manager, verify the binary was actually created (catches silent MSRV failures)
            if [ "$pkg" = "horus_manager" ] && [ "$build_status" -eq 0 ]; then
                if [ ! -f "target/release/horus" ]; then
                    echo -e "${RED}Binary not created despite build success - checking for MSRV issues...${NC}" >> "$TEMP_OUTPUT"
                    build_status=1
                    # Try to find and log the actual error from cargo output
                    if grep -q "requires rustc" "$LOG_FILE" 2>/dev/null; then
                        echo "MSRV error detected in build log" >> "$TEMP_OUTPUT"
                    fi
                fi
            fi

            # Update package status
            local PKG_END=$(date +%s)
            PKG_TIME[$pkg]=$((PKG_END - PKG_START))

            # Clear the progress line and print result
            printf "\r\033[K"  # Clear line

            if [ "$build_status" -eq 0 ]; then
                PKG_STATUS[$pkg]="done"
                PKG_PERCENT[$pkg]=100
                echo -e "    ${GREEN}[███████████████████]${NC} 100% ${GREEN}${pkg}${NC} done ($(format_duration ${PKG_TIME[$pkg]}))"
                LINES_BELOW_OVERALL=$((LINES_BELOW_OVERALL + 1))
            else
                # Increment per-package retry counter
                PKG_RETRIES[$pkg]=$((${PKG_RETRIES[$pkg]:-0} + 1))
                local pkg_retry_count=${PKG_RETRIES[$pkg]}

                if [ "$pkg_retry_count" -ge "$PKG_MAX_RETRIES" ]; then
                    # Package has exceeded its retry limit
                    PKG_STATUS[$pkg]="failed"
                    all_succeeded=false
                    echo -e "    ${RED}[███████████████████]${NC} ${RED}FAILED${NC} ${pkg} (max retries: ${pkg_retry_count}/${PKG_MAX_RETRIES})"
                    LINES_BELOW_OVERALL=$((LINES_BELOW_OVERALL + 1))
                    break  # Stop - this package cannot be recovered
                else
                    # Package can still retry
                    PKG_STATUS[$pkg]="retry"
                    all_succeeded=false
                    echo -e "    ${YELLOW}[███████████████████]${NC} ${YELLOW}RETRY${NC} ${pkg} (attempt ${pkg_retry_count}/${PKG_MAX_RETRIES})"
                    LINES_BELOW_OVERALL=$((LINES_BELOW_OVERALL + 1))
                    break  # Stop and retry from this package
                fi
            fi
        done

        # Show cursor again
        tput cnorm 2>/dev/null || true

        if [ "$all_succeeded" = true ]; then
            local OVERALL_END=$(date +%s)
            local total_elapsed=$((OVERALL_END - OVERALL_START))

            # Update overall bar to 100%
            local up_count=$((LINES_BELOW_OVERALL + 1))
            printf "\033[%dA" "$up_count"
            printf "\r  [${GREEN}████████████████████████████████████████${NC}] 100%% Done!\033[K"
            printf "\033[%dB" "$up_count"

            echo ""
            echo -e "  ${STATUS_OK} All packages built successfully in $(format_duration $total_elapsed)"
            rm -f "$TEMP_OUTPUT" 2>/dev/null
            return 0
        else
            # Check if any package has exceeded its per-package retry limit
            local pkg_max_exceeded=false
            local failed_pkg=""
            for check_pkg in "${BUILD_PACKAGES[@]}"; do
                if [ "${PKG_STATUS[$check_pkg]}" = "failed" ]; then
                    pkg_max_exceeded=true
                    failed_pkg="$check_pkg"
                    break
                fi
            done

            if [ "$pkg_max_exceeded" = true ]; then
                # A package has exceeded its max retries - give up immediately
                echo ""
                echo -e "  ${STATUS_ERR} Package ${RED}${failed_pkg}${NC} failed after ${PKG_MAX_RETRIES} attempts"
                break
            fi

            ((retry++))
            echo ""
            echo -e "  ${STATUS_WARN} Build paused for recovery"

            if [ $retry -lt $max_retries ]; then
                echo -e "${YELLOW}${STATUS_INFO} Analyzing build failure and attempting auto-fix...${NC}"

                # Use smart error parser to detect and fix issues
                local error_result=$(parse_build_error "$LOG_FILE")
                local error_type="${error_result%%:*}"
                local fix_status="${error_result##*:}"

                if [ "$fix_status" = "fixed" ]; then
                    echo -e "${GREEN}${STATUS_OK} Auto-fix applied for: $error_type${NC}"
                else
                    # Fall back to generic fixes
                    echo -e "${CYAN}${STATUS_INFO} Applying generic recovery steps...${NC}"

                    # Common fixes for build failures
                    echo -e "${CYAN}   Updating cargo index...${NC}"
                    cargo update 2>/dev/null || true

                    # Fix potential permission issues
                    if grep -q "permission denied" "$LOG_FILE" 2>/dev/null; then
                        echo -e "${CYAN}   Fixing permissions...${NC}"
                        chmod -R u+rwx target/ 2>/dev/null || true
                        chmod -R u+rwx ~/.cargo/ 2>/dev/null || true
                    fi

                    # Clear cargo cache if download failed
                    if grep -q "failed to download\|failed to fetch" "$LOG_FILE" 2>/dev/null; then
                        echo -e "${CYAN}   Clearing cargo cache...${NC}"
                        rm -rf ~/.cargo/registry/cache 2>/dev/null || true
                        rm -rf ~/.cargo/registry/index 2>/dev/null || true
                    fi

                    # Check if linker/compilation error - try cleaning affected package
                    if grep -q "could not compile\|linker command failed" "$LOG_FILE" 2>/dev/null; then
                        FAILED_PKG_NAME=$(grep -oE "could not compile ['\`][^'\`]+" "$LOG_FILE" 2>/dev/null | sed "s/could not compile ['\`]//" | head -1)
                        if [ -n "$FAILED_PKG_NAME" ]; then
                            echo -e "${CYAN}   Cleaning $FAILED_PKG_NAME artifacts...${NC}"
                            cargo clean -p "$FAILED_PKG_NAME" 2>/dev/null || true
                        fi
                    fi
                fi

                sleep 2
            fi
        fi
    done

    rm -f "$TEMP_OUTPUT" 2>/dev/null
    echo -e "  ${STATUS_ERR} Build failed after $max_retries attempts"
    echo -e "${YELLOW} Check the log file for details: $LOG_FILE${NC}"
    echo ""
    echo "Troubleshooting steps:"
    echo "  1. Try: cargo clean && rm -rf ~/.horus/cache && ./install.sh"
    echo "  2. Check if you have enough disk space: df -h"
    echo "  3. Try updating Rust: rustup update stable"
    echo "  4. Report issue: https://github.com/softmata/horus/issues"

    # Offer to create GitHub issue automatically
    offer_github_issue "$LOG_FILE"

    return 1
}

# Step 1: Build all packages
if ! build_with_recovery; then
    exit 1
fi
echo ""

# Step 2: Install CLI binary
echo -e "${CYAN}${STATUS_INFO} Installing CLI binary...${NC}"

if [ ! -d "$INSTALL_DIR" ]; then
    mkdir -p "$INSTALL_DIR"
fi

cp target/release/horus "$INSTALL_DIR/horus"
chmod +x "$INSTALL_DIR/horus"

echo -e "${GREEN}${NC} CLI installed to $INSTALL_DIR/horus"

echo ""

# Step 3: Create cache directory structure
echo -e "${CYAN}${STATUS_INFO} Setting up library cache...${NC}"

mkdir -p "$CACHE_DIR"

# Get version from Cargo.toml
HORUS_VERSION=$(grep -m1 '^version' horus/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_CORE_VERSION=$(grep -m1 '^version' horus_core/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_MACROS_VERSION=$(grep -m1 '^version' horus_macros/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_LIBRARY_VERSION=$(grep -m1 '^version' horus_library/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_AI_VERSION=$(grep -m1 '^version' horus_ai/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_PERCEPTION_VERSION=$(grep -m1 '^version' horus_perception/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_ROS2_BRIDGE_VERSION=$(grep -m1 '^version' horus_ros2_bridge/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_PY_VERSION=$(grep -m1 '^version' horus_py/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')

echo -e "${CYAN}  ${NC} Detected versions:"
echo "    horus: $HORUS_VERSION"
echo "    horus_core: $HORUS_CORE_VERSION"
echo "    horus_macros: $HORUS_MACROS_VERSION"
echo "    horus_library: $HORUS_LIBRARY_VERSION"
echo "    horus_ai: $HORUS_AI_VERSION"
echo "    horus_perception: $HORUS_PERCEPTION_VERSION"
echo "    horus_ros2_bridge: $HORUS_ROS2_BRIDGE_VERSION"
echo "    horus_py: $HORUS_PY_VERSION"
echo ""

# Check for version changes
VERSION_FILE="$HOME/.horus/installed_version"
if [ -f "$VERSION_FILE" ]; then
    OLD_VERSION=$(cat "$VERSION_FILE")
    if [ "$OLD_VERSION" != "$HORUS_VERSION" ]; then
        echo -e "${YELLOW}${NC}  Version changed: ${OLD_VERSION}  ${HORUS_VERSION}"
        echo -e "${CYAN}${NC} Cleaning old library versions..."

        # Remove old versioned directories
        rm -rf "$CACHE_DIR/horus@$OLD_VERSION" 2>/dev/null || true
        rm -rf "$CACHE_DIR/horus_core@$OLD_VERSION" 2>/dev/null || true
        rm -rf "$CACHE_DIR/horus_macros@$OLD_VERSION" 2>/dev/null || true
        rm -rf "$CACHE_DIR/horus_library@$OLD_VERSION" 2>/dev/null || true
        rm -rf "$CACHE_DIR/horus_ai@$OLD_VERSION" 2>/dev/null || true
        rm -rf "$CACHE_DIR/horus_perception@$OLD_VERSION" 2>/dev/null || true
        rm -rf "$CACHE_DIR/horus_ros2_bridge@$OLD_VERSION" 2>/dev/null || true
        rm -rf "$CACHE_DIR/horus_py@$OLD_VERSION" 2>/dev/null || true

        echo -e "${GREEN}${NC} Old versions removed"
        echo ""
    fi
fi

# Step 4: Install horus_core library
echo -e "${CYAN}${NC} Installing horus_core@$HORUS_CORE_VERSION..."
HORUS_CORE_DIR="$CACHE_DIR/horus_core@$HORUS_CORE_VERSION"
mkdir -p "$HORUS_CORE_DIR/lib"

# Copy compiled libraries
cp -r target/release/libhorus_core.* "$HORUS_CORE_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_core*.rlib "$HORUS_CORE_DIR/lib/" 2>/dev/null || true

# Create metadata
cat > "$HORUS_CORE_DIR/metadata.json" << EOF
{
  "name": "horus_core",
  "version": "$HORUS_CORE_VERSION",
  "description": "HORUS Core - Runtime and scheduler",
  "install_type": "source"
}
EOF

echo -e "${GREEN}${NC} Installed horus_core"

# Step 5: Install horus library
echo -e "${CYAN}${NC} Installing horus@$HORUS_VERSION..."
HORUS_DIR="$CACHE_DIR/horus@$HORUS_VERSION"
mkdir -p "$HORUS_DIR/lib"

# Copy compiled libraries
cp -r target/release/libhorus.* "$HORUS_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus*.rlib "$HORUS_DIR/lib/" 2>/dev/null || true

# Also copy target/release for Cargo path dependencies
mkdir -p "$HORUS_DIR/target/release"
cp -r target/release/libhorus*.rlib "$HORUS_DIR/target/release/" 2>/dev/null || true
cp -r target/release/deps/libhorus_core*.rlib "$HORUS_DIR/target/release/" 2>/dev/null || true

# CRITICAL: Copy ALL transitive dependencies for Cargo compilation
# This ensures user projects don't need to recompile HORUS dependencies
mkdir -p "$HORUS_DIR/target/release/deps"
echo -e "${CYAN}  ${NC} Bundling pre-compiled dependencies for instant user builds..."

# Copy all compiled artifacts
cp target/release/deps/*.rlib "$HORUS_DIR/target/release/deps/" 2>/dev/null || true
cp target/release/deps/*.rmeta "$HORUS_DIR/target/release/deps/" 2>/dev/null || true
cp target/release/deps/*.d "$HORUS_DIR/target/release/deps/" 2>/dev/null || true

# Copy fingerprints so Cargo knows these are already built
if [ -d "target/release/.fingerprint" ]; then
    mkdir -p "$HORUS_DIR/target/release/.fingerprint"
    cp -r target/release/.fingerprint/horus* "$HORUS_DIR/target/release/.fingerprint/" 2>/dev/null || true
fi

RLIB_COUNT=$(ls target/release/deps/*.rlib 2>/dev/null | wc -l)
echo -e "${GREEN}${NC} Bundled $RLIB_COUNT pre-compiled dependency libraries"
echo -e "${CYAN}     ${NC} Users won't need to recompile these!"

# Copy source Cargo.toml and src for `horus run` Cargo compilation
echo -e "${CYAN}  ${NC} Copying source files for horus run compatibility..."

# Copy workspace Cargo.toml to make cache a valid workspace
cp Cargo.toml "$HORUS_DIR/Cargo.toml" 2>/dev/null || true

# Copy horus crate
mkdir -p "$HORUS_DIR/horus"
cp horus/Cargo.toml "$HORUS_DIR/horus/" 2>/dev/null || true
cp -r horus/src "$HORUS_DIR/horus/" 2>/dev/null || true

# Copy horus_core crate
mkdir -p "$HORUS_DIR/horus_core"
cp horus_core/Cargo.toml "$HORUS_DIR/horus_core/" 2>/dev/null || true
cp -r horus_core/src "$HORUS_DIR/horus_core/" 2>/dev/null || true

# Copy horus_macros crate
mkdir -p "$HORUS_DIR/horus_macros"
cp horus_macros/Cargo.toml "$HORUS_DIR/horus_macros/" 2>/dev/null || true
cp -r horus_macros/src "$HORUS_DIR/horus_macros/" 2>/dev/null || true

# Copy horus_library crate (has lib.rs and subdirectories, not src/)
mkdir -p "$HORUS_DIR/horus_library"
cp horus_library/Cargo.toml "$HORUS_DIR/horus_library/" 2>/dev/null || true
cp horus_library/lib.rs "$HORUS_DIR/horus_library/" 2>/dev/null || true
cp -r horus_library/nodes "$HORUS_DIR/horus_library/" 2>/dev/null || true
cp -r horus_library/messages "$HORUS_DIR/horus_library/" 2>/dev/null || true
cp -r horus_library/traits "$HORUS_DIR/horus_library/" 2>/dev/null || true
cp -r horus_library/algorithms "$HORUS_DIR/horus_library/" 2>/dev/null || true

# Copy horus_ai crate
mkdir -p "$HORUS_DIR/horus_ai"
cp horus_ai/Cargo.toml "$HORUS_DIR/horus_ai/" 2>/dev/null || true
cp -r horus_ai/src "$HORUS_DIR/horus_ai/" 2>/dev/null || true

# Copy horus_perception crate
mkdir -p "$HORUS_DIR/horus_perception"
cp horus_perception/Cargo.toml "$HORUS_DIR/horus_perception/" 2>/dev/null || true
cp -r horus_perception/src "$HORUS_DIR/horus_perception/" 2>/dev/null || true

# Copy horus_ros2_bridge crate
mkdir -p "$HORUS_DIR/horus_ros2_bridge"
cp horus_ros2_bridge/Cargo.toml "$HORUS_DIR/horus_ros2_bridge/" 2>/dev/null || true
cp -r horus_ros2_bridge/src "$HORUS_DIR/horus_ros2_bridge/" 2>/dev/null || true

# Copy horus_manager crate (CLI binary source)
mkdir -p "$HORUS_DIR/horus_manager"
cp horus_manager/Cargo.toml "$HORUS_DIR/horus_manager/" 2>/dev/null || true
cp -r horus_manager/src "$HORUS_DIR/horus_manager/" 2>/dev/null || true

# Copy horus_py crate (Python bindings source - for reference)
mkdir -p "$HORUS_DIR/horus_py"
cp horus_py/Cargo.toml "$HORUS_DIR/horus_py/" 2>/dev/null || true
cp -r horus_py/src "$HORUS_DIR/horus_py/" 2>/dev/null || true

# Create metadata
cat > "$HORUS_DIR/metadata.json" << EOF
{
  "name": "horus",
  "version": "$HORUS_VERSION",
  "description": "HORUS Framework - Main library",
  "install_type": "source"
}
EOF

echo -e "${GREEN}${NC} Installed horus"

# Step 6: Install horus_macros
echo -e "${CYAN}${NC} Installing horus_macros@$HORUS_MACROS_VERSION..."
HORUS_MACROS_DIR="$CACHE_DIR/horus_macros@$HORUS_MACROS_VERSION"
mkdir -p "$HORUS_MACROS_DIR/lib"

# Copy procedural macro library (platform-specific extensions)
cp -r target/release/libhorus_macros.* "$HORUS_MACROS_DIR/lib/" 2>/dev/null || true
# Linux uses .so, macOS uses .dylib
cp -r target/release/deps/libhorus_macros*.so "$HORUS_MACROS_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_macros*.dylib "$HORUS_MACROS_DIR/lib/" 2>/dev/null || true

# Also copy to target/release for Cargo
mkdir -p "$HORUS_MACROS_DIR/target/release"
# Linux
cp -r target/release/libhorus_macros.so "$HORUS_MACROS_DIR/target/release/" 2>/dev/null || true
cp -r target/release/deps/libhorus_macros*.so "$HORUS_MACROS_DIR/target/release/" 2>/dev/null || true
# macOS
cp -r target/release/libhorus_macros.dylib "$HORUS_MACROS_DIR/target/release/" 2>/dev/null || true
cp -r target/release/deps/libhorus_macros*.dylib "$HORUS_MACROS_DIR/target/release/" 2>/dev/null || true

# Create metadata
cat > "$HORUS_MACROS_DIR/metadata.json" << EOF
{
  "name": "horus_macros",
  "version": "$HORUS_MACROS_VERSION",
  "description": "HORUS Macros - Procedural macros for simplified node creation",
  "install_type": "source"
}
EOF

echo -e "${GREEN}${NC} Installed horus_macros"

# Step 7: Install horus_library
echo -e "${CYAN}${NC} Installing horus_library@$HORUS_LIBRARY_VERSION..."
HORUS_LIBRARY_DIR="$CACHE_DIR/horus_library@$HORUS_LIBRARY_VERSION"
mkdir -p "$HORUS_LIBRARY_DIR/lib"

# Copy compiled libraries (platform-specific extensions)
cp -r target/release/libhorus_library.* "$HORUS_LIBRARY_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_library*.rlib "$HORUS_LIBRARY_DIR/lib/" 2>/dev/null || true
# Linux .so and macOS .dylib
cp -r target/release/deps/libhorus_library*.so "$HORUS_LIBRARY_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_library*.dylib "$HORUS_LIBRARY_DIR/lib/" 2>/dev/null || true

# Also copy to target/release
mkdir -p "$HORUS_LIBRARY_DIR/target/release"
cp -r target/release/libhorus_library*.rlib "$HORUS_LIBRARY_DIR/target/release/" 2>/dev/null || true
cp -r target/release/libhorus_library*.so "$HORUS_LIBRARY_DIR/target/release/" 2>/dev/null || true
cp -r target/release/libhorus_library*.dylib "$HORUS_LIBRARY_DIR/target/release/" 2>/dev/null || true

# Create metadata
cat > "$HORUS_LIBRARY_DIR/metadata.json" << EOF
{
  "name": "horus_library",
  "version": "$HORUS_LIBRARY_VERSION",
  "description": "HORUS Standard Library - Common messages and nodes",
  "install_type": "source"
}
EOF

echo -e "${GREEN}${NC} Installed horus_library"

# Step 7b: Install horus_ai
echo -e "${CYAN}${NC} Installing horus_ai@$HORUS_AI_VERSION..."
HORUS_AI_DIR="$CACHE_DIR/horus_ai@$HORUS_AI_VERSION"
mkdir -p "$HORUS_AI_DIR/lib"

cp -r target/release/libhorus_ai.* "$HORUS_AI_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_ai*.rlib "$HORUS_AI_DIR/lib/" 2>/dev/null || true

cat > "$HORUS_AI_DIR/metadata.json" << EOF
{
  "name": "horus_ai",
  "version": "$HORUS_AI_VERSION",
  "description": "HORUS AI - Tensor system with DLPack interop",
  "install_type": "source"
}
EOF

echo -e "${GREEN}${NC} Installed horus_ai"

# Step 7c: Install horus_perception
echo -e "${CYAN}${NC} Installing horus_perception@$HORUS_PERCEPTION_VERSION..."
HORUS_PERCEPTION_DIR="$CACHE_DIR/horus_perception@$HORUS_PERCEPTION_VERSION"
mkdir -p "$HORUS_PERCEPTION_DIR/lib"

cp -r target/release/libhorus_perception.* "$HORUS_PERCEPTION_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_perception*.rlib "$HORUS_PERCEPTION_DIR/lib/" 2>/dev/null || true

cat > "$HORUS_PERCEPTION_DIR/metadata.json" << EOF
{
  "name": "horus_perception",
  "version": "$HORUS_PERCEPTION_VERSION",
  "description": "HORUS Perception - AI-friendly detection, point cloud, and landmark types",
  "install_type": "source"
}
EOF

echo -e "${GREEN}${NC} Installed horus_perception"

# Step 7d: Install horus_ros2_bridge
echo -e "${CYAN}${NC} Installing horus_ros2_bridge@$HORUS_ROS2_BRIDGE_VERSION..."
HORUS_ROS2_BRIDGE_DIR="$CACHE_DIR/horus_ros2_bridge@$HORUS_ROS2_BRIDGE_VERSION"
mkdir -p "$HORUS_ROS2_BRIDGE_DIR/lib"

cp -r target/release/libhorus_ros2_bridge.* "$HORUS_ROS2_BRIDGE_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_ros2_bridge*.rlib "$HORUS_ROS2_BRIDGE_DIR/lib/" 2>/dev/null || true

cat > "$HORUS_ROS2_BRIDGE_DIR/metadata.json" << EOF
{
  "name": "horus_ros2_bridge",
  "version": "$HORUS_ROS2_BRIDGE_VERSION",
  "description": "HORUS ROS2 Bridge - ROS2 protocol bridge via Zenoh transport",
  "install_type": "source"
}
EOF

echo -e "${GREEN}${NC} Installed horus_ros2_bridge"

# ============================================================================
# PYTHON VERSION SOLVER - Smart dependency resolution for horus_py
# ============================================================================

# Parse pip errors and auto-resolve
parse_pip_error() {
    local pip_output="$1"
    local error_type="unknown"
    local fix_applied=false

    if echo "$pip_output" | grep -qi "no matching distribution"; then
        error_type="no_wheel"
        # Check if it's platform-specific
        if echo "$pip_output" | grep -qi "manylinux\|macosx\|win"; then
            echo -e "${YELLOW}${STATUS_WARN} No pre-built wheel for your platform${NC}"
        fi
    elif echo "$pip_output" | grep -qi "requires python"; then
        error_type="python_version"
        # Use sed instead of grep -P for BSD (macOS) compatibility
        local required=$(echo "$pip_output" | sed -n 's/.*[Rr]equires [Pp]ython[^0-9]*\([0-9.]*\).*/\1/p' | head -1)
        if [ -n "$required" ]; then
            echo -e "${YELLOW}${STATUS_WARN} Package requires Python $required (you have $PYTHON_VERSION)${NC}"
        fi
    elif echo "$pip_output" | grep -qi "glibc"; then
        error_type="glibc"
        echo -e "${YELLOW}${STATUS_WARN} GLIBC version incompatibility - building from source${NC}"
    elif echo "$pip_output" | grep -qi "conflict"; then
        error_type="dependency_conflict"
        echo -e "${YELLOW}${STATUS_WARN} Dependency conflict detected${NC}"
        echo -e "${CYAN}${STATUS_INFO} Trying with --force-reinstall...${NC}"
        if pip3 install horus-robotics --user --force-reinstall 2>/dev/null; then
            fix_applied=true
        fi
    elif echo "$pip_output" | grep -qi "externally-managed-environment"; then
        error_type="pep668"
        echo -e "${YELLOW}${STATUS_WARN} PEP 668 protected environment${NC}"
        echo -e "${CYAN}${STATUS_INFO} Trying with --break-system-packages...${NC}"
        if pip3 install horus-robotics --user --break-system-packages 2>/dev/null; then
            fix_applied=true
            echo -e "${GREEN}${STATUS_OK} Installed with --break-system-packages${NC}"
        fi
    fi

    if [ "$fix_applied" = true ]; then
        echo "$error_type:fixed"
        return 0
    else
        echo "$error_type:not_fixed"
        return 1
    fi
}

# Step 8: Install horus_py (Python bindings) - Optional
if [ "$PYTHON_AVAILABLE" = true ]; then
    echo -e "${CYAN}${STATUS_INFO} Installing horus_py@$HORUS_PY_VERSION (Python bindings)...${NC}"

    HORUS_PY_INSTALLED=false

    # NOTE: PyPI install is disabled - always build from source with maturin
    # This ensures users get the latest local version matching their Rust build
    #
    # # Try to install from PyPI (pre-built wheel)
    # echo -e "${CYAN}   Installing from PyPI...${NC}"
    #
    # PIP_OUTPUT=$(pip3 install horus-robotics --user 2>&1)
    # PIP_EXIT_CODE=$?
    #
    # if [ $PIP_EXIT_CODE -eq 0 ]; then
    #     HORUS_PY_INSTALLED=true
    # else
    #     # Parse the error and try auto-fix
    #     echo -e "${YELLOW}   PyPI install failed - analyzing error...${NC}"
    #
    #     PIP_FIX_RESULT=$(parse_pip_error "$PIP_OUTPUT")
    #     PIP_ERROR_TYPE="${PIP_FIX_RESULT%%:*}"
    #     PIP_FIX_STATUS="${PIP_FIX_RESULT##*:}"
    #
    #     if [ "$PIP_FIX_STATUS" = "fixed" ]; then
    #         HORUS_PY_INSTALLED=true
    #     else
    #         # Show relevant error lines (filter out noise)
    #         echo "$PIP_OUTPUT" | grep -E "(ERROR|error:|Could not|No matching|requires|glibc)" | head -3
    #         echo ""
    #         echo -e "${YELLOW}[-]${NC} Could not install horus_py from PyPI"
    # END OF COMMENTED PyPI SECTION

    # Build from source with maturin (primary installation method)
    if [ -d "horus_py" ]; then
        echo -e "${CYAN}${STATUS_INFO} Building horus_py from source with maturin...${NC}"

        # Check if maturin is available, install if not
        MATURIN_AVAILABLE=false
        if command -v maturin &> /dev/null; then
            MATURIN_AVAILABLE=true
            echo -e "${GREEN}${STATUS_OK} maturin found${NC}"
        else
            echo -e "${CYAN}   Installing maturin...${NC}"

            # Check if we can use sudo without password (for non-interactive install)
            CAN_SUDO=false
            if command -v sudo &>/dev/null && sudo -n true 2>/dev/null; then
                CAN_SUDO=true
            fi

            # Try multiple installation methods (OS-aware, ordered by speed)
            # 1. Try non-sudo methods first (pipx, pip, brew on macOS)
            if command -v brew &> /dev/null && brew install maturin 2>/dev/null; then
                # Homebrew doesn't need sudo
                MATURIN_AVAILABLE=true
                echo -e "${GREEN}${STATUS_OK} maturin installed via Homebrew${NC}"
            elif command -v pipx &> /dev/null && pipx install maturin 2>/dev/null; then
                # pipx doesn't need sudo
                MATURIN_AVAILABLE=true
                export PATH="$HOME/.local/bin:$PATH"
                echo -e "${GREEN}${STATUS_OK} maturin installed via pipx${NC}"
            elif pip3 install maturin --user --break-system-packages 2>/dev/null; then
                # pip --user doesn't need sudo
                MATURIN_AVAILABLE=true
                echo -e "${GREEN}${STATUS_OK} maturin installed via pip${NC}"
            elif pip3 install maturin --user 2>/dev/null; then
                # pip --user doesn't need sudo (older systems)
                MATURIN_AVAILABLE=true
                echo -e "${GREEN}${STATUS_OK} maturin installed via pip${NC}"
            # 2. Try OS package manager if sudo available without password
            elif [ "$CAN_SUDO" = true ]; then
                if command -v apt-get &> /dev/null && sudo apt-get install -y python3-maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via apt${NC}"
                elif command -v dnf &> /dev/null && sudo dnf install -y python3-maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via dnf${NC}"
                elif command -v pacman &> /dev/null && sudo pacman -S --noconfirm python-maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via pacman${NC}"
                elif command -v zypper &> /dev/null && sudo zypper install -y python3-maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via zypper${NC}"
                elif command -v apk &> /dev/null && sudo apk add py3-maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via apk${NC}"
                # BSD systems
                elif command -v pkg &> /dev/null && sudo pkg install -y py39-maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via pkg (FreeBSD)${NC}"
                # Void Linux
                elif command -v xbps-install &> /dev/null && sudo xbps-install -y python3-maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via xbps${NC}"
                fi
            fi

            # NixOS - try nix-env (doesn't need sudo in user profile mode)
            if [ "$MATURIN_AVAILABLE" != true ] && command -v nix-env &> /dev/null; then
                if nix-env -iA nixpkgs.maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via nix-env${NC}"
                fi
            fi

            # 3. cargo install as last resort (slow but universal, no sudo needed)
            if [ "$MATURIN_AVAILABLE" != true ]; then
                echo -e "${CYAN}   Trying cargo install (this may take a few minutes)...${NC}"
                if cargo install maturin 2>/dev/null; then
                    MATURIN_AVAILABLE=true
                    echo -e "${GREEN}${STATUS_OK} maturin installed via cargo${NC}"
                fi
            fi

            # 4. If still not available, show manual options
            if [ "$MATURIN_AVAILABLE" != true ]; then
                echo -e "${YELLOW}${STATUS_WARN} Could not install maturin automatically${NC}"
                echo -e "   ${CYAN}Manual fix options (choose one):${NC}"
                echo -e "   ${CYAN}  Debian/Ubuntu: sudo apt install python3-maturin${NC}"
                echo -e "   ${CYAN}  Fedora/RHEL:   sudo dnf install python3-maturin${NC}"
                echo -e "   ${CYAN}  Arch:          sudo pacman -S python-maturin${NC}"
                echo -e "   ${CYAN}  openSUSE:      sudo zypper install python3-maturin${NC}"
                echo -e "   ${CYAN}  Alpine:        sudo apk add py3-maturin${NC}"
                echo -e "   ${CYAN}  FreeBSD:       sudo pkg install py39-maturin${NC}"
                echo -e "   ${CYAN}  Void:          sudo xbps-install python3-maturin${NC}"
                echo -e "   ${CYAN}  NixOS:         nix-env -iA nixpkgs.maturin${NC}"
                echo -e "   ${CYAN}  macOS:         brew install maturin${NC}"
                echo -e "   ${CYAN}  Any OS:        pipx install maturin${NC}"
                echo -e "   ${CYAN}  Any OS:        cargo install maturin${NC}"
            fi
        fi

        # Build with maturin if available
        if [ "$MATURIN_AVAILABLE" = true ]; then
            echo -e "${CYAN}   Building horus_py from source...${NC}"
            CURRENT_DIR=$(pwd)
            cd horus_py

            # Build the wheel
            if maturin build --release 2>/dev/null; then
                # Try to install the wheel (handle PEP 668 systems)
                WHEEL_FILE=$(ls target/wheels/*.whl 2>/dev/null | head -1)
                if [ -n "$WHEEL_FILE" ]; then
                    if pip3 install "$WHEEL_FILE" --user --force-reinstall 2>/dev/null; then
                        HORUS_PY_INSTALLED=true
                        echo -e "${GREEN}${STATUS_OK} horus_py built and installed from source${NC}"
                    elif pip3 install "$WHEEL_FILE" --user --force-reinstall --break-system-packages 2>/dev/null; then
                        HORUS_PY_INSTALLED=true
                        echo -e "${GREEN}${STATUS_OK} horus_py built and installed from source${NC}"
                    elif command -v pipx &> /dev/null && pipx install "$WHEEL_FILE" --force 2>/dev/null; then
                        HORUS_PY_INSTALLED=true
                        echo -e "${GREEN}${STATUS_OK} horus_py built and installed via pipx${NC}"
                    fi
                fi
            fi

            # Fallback: Try maturin develop if build+install failed
            if [ "$HORUS_PY_INSTALLED" != true ]; then
                # maturin develop respects virtual environments and handles PEP 668
                if maturin develop --release 2>/dev/null; then
                    HORUS_PY_INSTALLED=true
                    echo -e "${GREEN}${STATUS_OK} horus_py installed in development mode${NC}"
                else
                    echo -e "${YELLOW}[-]${NC} maturin build failed"
                    echo -e "   ${CYAN}Try: cd horus_py && maturin develop --release${NC}"
                fi
            fi

            cd "$CURRENT_DIR"
        fi
    fi

    # Verify installation and show result
    if [ "$HORUS_PY_INSTALLED" = true ]; then
        if python3 -c "import horus; print(f'horus {horus.__version__}')" 2>/dev/null; then
            INSTALLED_VERSION=$(python3 -c "import horus; print(horus.__version__)" 2>/dev/null)
            echo -e "${GREEN}${STATUS_OK} horus_py built and installed (v$INSTALLED_VERSION)${NC}"
        else
            echo -e "${YELLOW}[-]${NC} Python bindings installed but import failed"
            echo -e "  ${CYAN}Try: python3 -c 'import horus'${NC} to see the error"
        fi
    fi
else
    echo -e "${YELLOW}[-]${NC} Skipping horus_py (Python not available)"
fi
echo ""

# Step 10: Copy examples
echo -e "${CYAN}${NC} Installing examples..."
EXAMPLES_DIR="$HORUS_DIR/examples"
mkdir -p "$EXAMPLES_DIR"

# Copy snakesim example (complete directory structure)
if [ -d "horus_library/apps/snakesim" ]; then
    echo -e "${CYAN}  ${NC} Installing Snake Game example..."

    # Copy entire snakesim directory to preserve structure
    cp -r horus_library/apps/snakesim "$EXAMPLES_DIR/" 2>/dev/null || true

    # Clean up unnecessary files from the copied example
    rm -rf "$EXAMPLES_DIR/snakesim/.horus" 2>/dev/null || true
    rm -rf "$EXAMPLES_DIR/snakesim/.gitignore" 2>/dev/null || true
    rm -rf "$EXAMPLES_DIR/snakesim/snakesim_gui/.horus" 2>/dev/null || true
    rm -rf "$EXAMPLES_DIR/snakesim/snakesim_gui/.claude" 2>/dev/null || true

    # Verify the copy
    if [ -f "$EXAMPLES_DIR/snakesim/main.rs" ] && [ -f "$EXAMPLES_DIR/snakesim/snakesim_gui/main.rs" ]; then
        echo -e "${GREEN}${NC} Installed Snake Game example"
        echo -e "${CYAN}     ${NC} Backend: main.rs + horus.yaml"
        echo -e "${CYAN}     ${NC} GUI: snakesim_gui/main.rs + horus.yaml"
    else
        echo -e "${YELLOW}${NC}  Warning: Snake Game example may be incomplete"
    fi
else
    echo -e "${YELLOW}${NC}  Snake Game example not found in source"
fi

# Copy wallesim example (WALL-E 3D simulation)
if [ -d "horus_library/apps/wallesim" ]; then
    echo -e "${CYAN}  ${NC} Installing WALL-E 3D Simulation example..."

    # Copy entire wallesim directory to preserve structure
    cp -r horus_library/apps/wallesim "$EXAMPLES_DIR/" 2>/dev/null || true

    # Clean up unnecessary files from the copied example
    rm -rf "$EXAMPLES_DIR/wallesim/.horus" 2>/dev/null || true
    rm -rf "$EXAMPLES_DIR/wallesim/.gitignore" 2>/dev/null || true
    rm -rf "$EXAMPLES_DIR/wallesim/.claude" 2>/dev/null || true

    # Verify the copy
    if [ -f "$EXAMPLES_DIR/wallesim/world.yaml" ] && [ -f "$EXAMPLES_DIR/wallesim/models/walle/walle.urdf" ]; then
        echo -e "${GREEN}${NC} Installed WALL-E 3D Simulation example"
        echo -e "${CYAN}     ${NC} World: world.yaml (Axiom cargo bay)"
        echo -e "${CYAN}     ${NC} Robot: models/walle/walle.urdf"
    else
        echo -e "${YELLOW}${NC}  Warning: WALL-E simulation example may be incomplete"
    fi
else
    echo -e "${YELLOW}${NC}  WALL-E simulation example not found in source"
fi

echo ""

# Save installed version for future updates
echo "$HORUS_VERSION" > "$VERSION_FILE"

# Save install profile for uninstall.sh
echo "full" > "$HOME/.horus/install_profile"

# Migrate old config files from localhost to production
AUTH_CONFIG="$HOME/.horus/auth.json"
if [ -f "$AUTH_CONFIG" ]; then
    if grep -q "localhost" "$AUTH_CONFIG" 2>/dev/null; then
        echo -e "${CYAN}${NC} Migrating registry configuration..."
        # Update localhost URLs to production
        sed -i.bak 's|http://localhost:3001|https://horus-marketplace-api.onrender.com|g' "$AUTH_CONFIG"
        sed -i.bak 's|http://localhost:8080|https://horus-marketplace-api.onrender.com|g' "$AUTH_CONFIG"
        echo -e "${GREEN}${NC} Registry URL updated to production"
        echo ""
    fi
fi

# Step 10: Verify installation
echo -e "${CYAN} Verifying installation...${NC}"

if [ -x "$INSTALL_DIR/horus" ]; then
    echo -e "${GREEN}${NC} CLI binary (horus): OK"
else
    echo -e "${RED}${NC} CLI binary (horus): Missing"
fi

if [ -d "$HORUS_DIR" ]; then
    echo -e "${GREEN}${NC} horus library: OK"
else
    echo -e "${RED}${NC} horus library: Missing"
fi

if [ -d "$HORUS_CORE_DIR" ]; then
    echo -e "${GREEN}${NC} horus_core library: OK"
else
    echo -e "${RED}${NC} horus_core library: Missing"
fi

if [ -d "$HORUS_MACROS_DIR" ]; then
    echo -e "${GREEN}${NC} horus_macros library: OK"
else
    echo -e "${RED}${NC} horus_macros library: Missing"
fi

if [ -d "$HORUS_LIBRARY_DIR" ]; then
    echo -e "${GREEN}${NC} horus_library: OK"
else
    echo -e "${RED}${NC} horus_library: Missing"
fi

if [ -d "$HORUS_AI_DIR" ]; then
    echo -e "${GREEN}${NC} horus_ai: OK"
else
    echo -e "${RED}${NC} horus_ai: Missing"
fi

if [ -d "$HORUS_PERCEPTION_DIR" ]; then
    echo -e "${GREEN}${NC} horus_perception: OK"
else
    echo -e "${RED}${NC} horus_perception: Missing"
fi

if [ -d "$HORUS_ROS2_BRIDGE_DIR" ]; then
    echo -e "${GREEN}${NC} horus_ros2_bridge: OK"
else
    echo -e "${RED}${NC} horus_ros2_bridge: Missing"
fi

if [ "$PYTHON_AVAILABLE" = true ]; then
    if [ -d "$HORUS_PY_DIR" ]; then
        echo -e "${GREEN}${NC} horus_py: OK"
    else
        echo -e "${RED}${NC} horus_py: Missing"
    fi
else
    echo -e "${YELLOW}[-]${NC} horus_py: Skipped (Python not available)"
fi

echo ""

# Check if CLI is in PATH
if command -v horus &> /dev/null; then
    echo -e "${GREEN}${NC} 'horus' command is available in PATH"
    echo -e "${CYAN}${NC} Version: $(horus --version 2>/dev/null || echo 'unknown')"
else
    echo -e "${YELLOW}${NC}  'horus' command not found in PATH"
    echo -e "  Add ${CYAN}$INSTALL_DIR${NC} to your PATH:"
    echo -e "  ${CYAN}export PATH=\"\$HOME/.cargo/bin:\$PATH\"${NC}"
    echo ""
    echo -e "  Add this to your shell profile (~/.bashrc, ~/.zshrc, etc.)"
fi

# Step 11: Setup shell completions
echo ""
echo -e "${CYAN}${NC} Setting up shell completions..."

# Detect user's shell
SHELL_NAME=$(basename "$SHELL")
COMPLETION_INSTALLED=false

case "$SHELL_NAME" in
    bash)
        # Try to set up bash completions
        if [ -f ~/.bashrc ]; then
            # Check if completion is already in bashrc
            if ! grep -q "horus completion bash" ~/.bashrc 2>/dev/null; then
                echo "" >> ~/.bashrc
                echo "# HORUS shell completions" >> ~/.bashrc
                echo 'eval "$(horus completion bash)"' >> ~/.bashrc
                echo -e "${GREEN}${NC} Added bash completions to ~/.bashrc"
                COMPLETION_INSTALLED=true
            else
                echo -e "${GREEN}${NC} Bash completions already configured"
                COMPLETION_INSTALLED=true
            fi
        fi
        ;;
    zsh)
        # Try to set up zsh completions
        if [ -f ~/.zshrc ]; then
            if ! grep -q "horus completion zsh" ~/.zshrc 2>/dev/null; then
                echo "" >> ~/.zshrc
                echo "# HORUS shell completions" >> ~/.zshrc
                echo 'eval "$(horus completion zsh)"' >> ~/.zshrc
                echo -e "${GREEN}${NC} Added zsh completions to ~/.zshrc"
                COMPLETION_INSTALLED=true
            else
                echo -e "${GREEN}${NC} Zsh completions already configured"
                COMPLETION_INSTALLED=true
            fi
        fi
        ;;
    fish)
        # Try to set up fish completions
        FISH_COMP_DIR="$HOME/.config/fish/completions"
        if command -v fish &> /dev/null; then
            mkdir -p "$FISH_COMP_DIR"
            if [ -x "$INSTALL_DIR/horus" ]; then
                "$INSTALL_DIR/horus" completion fish > "$FISH_COMP_DIR/horus.fish" 2>/dev/null
                echo -e "${GREEN}${NC} Generated fish completions to $FISH_COMP_DIR/horus.fish"
                COMPLETION_INSTALLED=true
            fi
        fi
        ;;
    *)
        echo -e "${YELLOW}${NC}  Unknown shell: $SHELL_NAME"
        echo -e "  You can manually set up completions later:"
        echo -e "    ${CYAN}horus completion --help${NC}"
        ;;
esac

if [ "$COMPLETION_INSTALLED" = true ]; then
    echo -e "${CYAN}  [i]${NC} Shell completions will be active in new terminal sessions"
    echo -e "  To use in this session: ${CYAN}source ~/.${SHELL_NAME}rc${NC} (bash/zsh)"
fi

# Step 12: Real-Time Setup (Optional, Linux only)
if [ "$OS_TYPE" = "linux" ] || [ "$OS_TYPE" = "wsl" ]; then
    echo ""
    echo -e "${CYAN}${STATUS_INFO}${NC} Real-Time Scheduling (Optional)"
    echo ""
    echo "  HORUS supports real-time scheduling for deterministic robot control."
    echo "  RT scheduling requires system configuration (needs sudo)."
    echo ""
    echo "  Benefits of RT scheduling:"
    echo "    - Deterministic timing (<500ns IPC latency)"
    echo "    - SCHED_FIFO/SCHED_RR priorities up to 99"
    echo "    - Memory locking to prevent page faults"
    echo ""

    RT_SCRIPT="$SCRIPT_DIR/scripts/setup-realtime.sh"
    if [ -f "$RT_SCRIPT" ]; then
        read -p "$(echo -e ${CYAN}?${NC}) Configure real-time scheduling now? [y/N]: " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo ""
            echo -e "${CYAN}${STATUS_INFO}${NC} Running real-time setup (requires sudo)..."
            echo ""
            if sudo bash "$RT_SCRIPT"; then
                echo ""
                echo -e "${GREEN}${STATUS_OK}${NC} Real-time configuration applied"
                echo -e "${YELLOW}${STATUS_WARN}${NC} You must LOG OUT and LOG BACK IN for changes to take effect"
            else
                echo ""
                echo -e "${YELLOW}${STATUS_WARN}${NC} Real-time setup had issues - check output above"
            fi
        else
            echo ""
            echo -e "${CYAN}${STATUS_INFO}${NC} Skipped. You can run it later:"
            echo -e "     ${CYAN}sudo ./scripts/setup-realtime.sh${NC}"
        fi
    else
        echo -e "${YELLOW}${STATUS_WARN}${NC} RT setup script not found: $RT_SCRIPT"
        echo "  Download from: https://github.com/softmata/horus/blob/main/scripts/setup-realtime.sh"
    fi
fi

# ============================================================================
# INSTALLATION VERSION SUMMARY
# ============================================================================
# Display all version information for debugging and support

echo ""
echo -e "╔═══════════════════════════════════════════════════════════════════════════╗"
echo -e "║                    INSTALLATION VERSION SUMMARY                           ║"
echo -e "╚═══════════════════════════════════════════════════════════════════════════╝"
echo ""

# Rust version
FINAL_RUST_VERSION=$(rustc --version 2>/dev/null | awk '{print $2}' || echo "unknown")
if version_gte "$FINAL_RUST_VERSION" "$REQUIRED_RUST_VERSION" && ! version_gt "$FINAL_RUST_VERSION" "$MAX_TESTED_RUST_VERSION"; then
    echo -e "${GREEN}${STATUS_OK} Rust: $FINAL_RUST_VERSION (within tested range $REQUIRED_RUST_VERSION - $MAX_TESTED_RUST_VERSION)${NC}"
elif version_gt "$FINAL_RUST_VERSION" "$MAX_TESTED_RUST_VERSION"; then
    echo -e "${YELLOW}${STATUS_WARN} Rust: $FINAL_RUST_VERSION (NEWER than tested $MAX_TESTED_RUST_VERSION)${NC}"
else
    echo -e "${YELLOW}${STATUS_WARN} Rust: $FINAL_RUST_VERSION (older than recommended $REQUIRED_RUST_VERSION)${NC}"
fi

# Python version (if available)
if [ "$PYTHON_AVAILABLE" = true ]; then
    FINAL_PYTHON_VERSION=$(python3 --version 2>/dev/null | awk '{print $2}' || echo "unknown")
    PYTHON_MAJOR=$(echo $FINAL_PYTHON_VERSION | cut -d. -f1)
    PYTHON_MINOR=$(echo $FINAL_PYTHON_VERSION | cut -d. -f2)
    if [ "$PYTHON_MAJOR" -eq "$MAX_TESTED_PYTHON_MAJOR" ] && [ "$PYTHON_MINOR" -le "$MAX_TESTED_PYTHON_MINOR" ]; then
        echo -e "${GREEN}${STATUS_OK} Python: $FINAL_PYTHON_VERSION (within tested range $REQUIRED_PYTHON_VERSION - $MAX_TESTED_PYTHON_VERSION)${NC}"
    elif [ "$PYTHON_MAJOR" -gt "$MAX_TESTED_PYTHON_MAJOR" ] || ([ "$PYTHON_MAJOR" -eq "$MAX_TESTED_PYTHON_MAJOR" ] && [ "$PYTHON_MINOR" -gt "$MAX_TESTED_PYTHON_MINOR" ]); then
        echo -e "${YELLOW}${STATUS_WARN} Python: $FINAL_PYTHON_VERSION (NEWER than tested $MAX_TESTED_PYTHON_VERSION)${NC}"
    else
        echo -e "${GREEN}${STATUS_OK} Python: $FINAL_PYTHON_VERSION${NC}"
    fi
fi

# HORUS version
echo -e "${GREEN}${STATUS_OK} HORUS: ${HORUS_VERSION:-0.1.x}${NC}"

# Key Cargo dependencies
if [ -f "Cargo.lock" ]; then
    for crate in serde tokio pyo3; do
        CRATE_VER=$(get_cargo_lock_version "$crate")
        if [ -n "$CRATE_VER" ]; then
            MIN_VER="${CARGO_MIN_VERSIONS[$crate]:-}"
            if [ -n "$MIN_VER" ] && version_gte "$CRATE_VER" "$MIN_VER"; then
                echo -e "${GREEN}${STATUS_OK} $crate: $CRATE_VER${NC}"
            else
                echo -e "${CYAN}${STATUS_INFO} $crate: $CRATE_VER${NC}"
            fi
        fi
    done
fi

# System info
echo ""
echo -e "${CYAN}System:${NC}"
echo -e "  OS: $(uname -s) $(uname -r)"
echo -e "  Arch: $(uname -m)"
echo -e "  Script: install.sh v$SCRIPT_VERSION"
echo -e "  Profile: full"

echo ""
echo -e "${GREEN}${STATUS_OK} HORUS installation complete!${NC}"
echo ""
echo -e "${CYAN}Next steps:${NC}"
echo "  1. Create a new project:"
echo -e "     ${CYAN}horus new my_robot${NC}"
echo ""
echo "  2. Or try the Snake Game example:"
echo -e "     ${CYAN}cp -r ~/.horus/cache/horus@${HORUS_VERSION}/examples/snakesim ~/my_snakesim${NC}"
echo -e "     ${CYAN}cd ~/my_snakesim${NC}"
echo ""
echo -e "     Terminal 1 (Backend): ${CYAN}horus run${NC}"
echo -e "     Terminal 2 (GUI): ${CYAN}cd snakesim_gui && horus run${NC}"
echo ""
echo -e "     Use ${CYAN}Arrow Keys${NC} or ${CYAN}WASD${NC} to control the snake!"
echo ""
echo "  3. Run your project:"
echo -e "     ${CYAN}cd my_robot${NC}"
echo -e "     ${CYAN}horus run${NC}"
echo ""

if [ "$PYTHON_AVAILABLE" = true ]; then
    echo -e "${CYAN}Python bindings:${NC}"
    echo "  Try the Python API:"
    echo -e "     ${CYAN}python3 -c 'import horus; print(horus.__doc__)'${NC}"
    echo ""
fi

# RT setup tip (Linux only)
if [ "$OS_TYPE" = "linux" ] || [ "$OS_TYPE" = "wsl" ]; then
    echo -e "${CYAN}Real-time scheduling:${NC}"
    echo "  After re-login, verify RT setup with:"
    echo -e "     ${CYAN}ulimit -r${NC}    # Should show 99"
    echo -e "     ${CYAN}ulimit -l${NC}    # Should show 'unlimited'"
    echo ""
    echo "  Or run the verification script:"
    echo -e "     ${CYAN}/tmp/verify-horus-rt.sh${NC}"
    echo ""
fi

echo -e "For help: ${CYAN}horus --help${NC}"
echo ""

# =============================================================================
# ANONYMOUS TELEMETRY (Privacy-First)
# =============================================================================
# Send anonymous installation count. No personal data, no tracking IDs.
# Only: event type, OS, timestamp. See: https://github.com/softmata/horus-telemetry-api
# This helps us understand how HORUS is being used and prioritize development.
# To disable: set HORUS_NO_TELEMETRY=1 before running install.sh

if [ -z "${HORUS_NO_TELEMETRY:-}" ]; then
    # Send anonymous install count in background (non-blocking)
    (
        TELEMETRY_URL="https://telemetry.horus-registry.dev/count"
        TIMESTAMP=$(date +%s)
        OS_NAME=$(uname -s)

        # Fire and forget - don't wait for response, don't fail if it doesn't work
        curl -s -X POST "$TELEMETRY_URL" \
            -H "Content-Type: application/json" \
            -d "{\"event\":\"install\",\"os\":\"$OS_NAME\",\"timestamp\":$TIMESTAMP}" \
            --connect-timeout 3 \
            --max-time 5 \
            >/dev/null 2>&1 || true
    ) &
fi

