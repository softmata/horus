#!/bin/bash
# HORUS Installation Script
# Universal installer that works across all major operating systems
# Uses shared deps.sh for consistent dependency management

set -e  # Exit on error
set -o pipefail  # Fail on pipe errors

# Script version
SCRIPT_VERSION="2.2.0"

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
# PLATFORM PROFILE SELECTION
# ============================================================================
# Profiles determine which packages to build:
#   - full: All packages including sim2d, sim3d (Desktop/Workstation)
#   - embedded: Core packages only, no heavy GUI (Raspberry Pi, Jetson)
#   - minimal: Only essential runtime, no library (Resource-constrained)

# Detect if running on embedded/SBC hardware
detect_embedded_platform() {
    local platform="desktop"

    # Check for Raspberry Pi
    if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null || grep -q "BCM" /proc/cpuinfo 2>/dev/null; then
        platform="raspberry_pi"
    # Check for NVIDIA Jetson
    elif [ -f "/etc/nv_tegra_release" ] || grep -q "tegra" /proc/cpuinfo 2>/dev/null; then
        platform="jetson"
    # Check for BeagleBone
    elif grep -q "AM33XX" /proc/cpuinfo 2>/dev/null; then
        platform="beaglebone"
    # Check for Orange Pi / other ARM SBCs
    elif [ "$(uname -m)" = "aarch64" ] || [ "$(uname -m)" = "armv7l" ]; then
        # Check available RAM (less than 4GB suggests embedded)
        local mem_kb=$(grep MemTotal /proc/meminfo 2>/dev/null | awk '{print $2}')
        if [ -n "$mem_kb" ] && [ "$mem_kb" -lt 4000000 ]; then
            platform="arm_sbc"
        fi
    fi

    echo "$platform"
}

# Platform selection prompt
select_platform_profile() {
    local detected_platform=$(detect_embedded_platform)
    local default_profile="full"
    local suggested=""

    # Suggest embedded profile for detected SBC platforms
    case "$detected_platform" in
        raspberry_pi)
            default_profile="embedded"
            suggested=" (Raspberry Pi detected)"
            ;;
        jetson)
            default_profile="embedded"
            suggested=" (NVIDIA Jetson detected)"
            ;;
        beaglebone)
            default_profile="minimal"
            suggested=" (BeagleBone detected)"
            ;;
        arm_sbc)
            default_profile="embedded"
            suggested=" (ARM SBC detected)"
            ;;
    esac

    echo ""
    echo -e "${CYAN}Select installation profile:${NC}${suggested}"
    echo ""
    echo -e "  ${GREEN}1)${NC} Full        - All packages including simulators (sim2d, sim3d)"
    echo -e "                   Best for: Desktop, Workstation, Development"
    echo -e "                   Requires: ~8GB RAM, GPU recommended"
    echo ""
    echo -e "  ${YELLOW}2)${NC} Embedded    - Core packages only, no heavy GUI dependencies"
    echo -e "                   Best for: Raspberry Pi, Jetson, ARM SBCs"
    echo -e "                   Requires: ~2GB RAM, no GPU needed"
    echo ""
    echo -e "  ${MAGENTA}3)${NC} Minimal     - Only essential runtime (horus, horus_core, horus_macros)"
    echo -e "                   Best for: Resource-constrained devices, CI/CD"
    echo -e "                   Requires: ~1GB RAM"
    echo ""

    local profile_num=""
    case "$default_profile" in
        full) profile_num="1" ;;
        embedded) profile_num="2" ;;
        minimal) profile_num="3" ;;
    esac

    read -p "$(echo -e ${CYAN}?${NC}) Select profile [1/2/3] (default: ${profile_num}): " -n 1 -r PROFILE_CHOICE
    echo

    # Default to detected profile if user just presses enter
    if [ -z "$PROFILE_CHOICE" ]; then
        PROFILE_CHOICE="$profile_num"
    fi

    case "$PROFILE_CHOICE" in
        1)
            INSTALL_PROFILE="full"
            echo -e "${GREEN}${STATUS_OK}${NC} Selected: Full installation"
            ;;
        2)
            INSTALL_PROFILE="embedded"
            echo -e "${YELLOW}${STATUS_OK}${NC} Selected: Embedded installation (no sim2d/sim3d)"
            ;;
        3)
            INSTALL_PROFILE="minimal"
            echo -e "${MAGENTA}${STATUS_OK}${NC} Selected: Minimal installation"
            ;;
        *)
            # Invalid input, use default
            INSTALL_PROFILE="$default_profile"
            echo -e "${CYAN}${STATUS_INFO}${NC} Using default: ${INSTALL_PROFILE}"
            ;;
    esac

    # Export the profile for later use
    export INSTALL_PROFILE
}

# Select platform profile
select_platform_profile

# Auto-install Rust if not present
install_rust() {
    if ! command -v cargo &> /dev/null; then
        echo -e "${YELLOW} Rust is not installed${NC}"
        read -p "$(echo -e ${CYAN}?${NC}) Install Rust automatically? [Y/n]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            echo -e "${CYAN} Installing Rust...${NC}"
            curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
            source "$HOME/.cargo/env"

            if ! command -v cargo &> /dev/null; then
                echo -e "${RED} Failed to install Rust${NC}"
                echo "Please install manually from https://rustup.rs/"
                exit 1
            fi
            echo -e "${GREEN} Rust installed successfully${NC}"
        else
            echo -e "${RED} Rust is required to build HORUS${NC}"
            exit 1
        fi
    fi
}

# Install Rust
install_rust

# Required Rust version (minimum stable version for HORUS)
REQUIRED_RUST_MAJOR=1
REQUIRED_RUST_MINOR=85
REQUIRED_RUST_VERSION="${REQUIRED_RUST_MAJOR}.${REQUIRED_RUST_MINOR}"

# Check Rust version
RUST_VERSION=$(rustc --version | awk '{print $2}')
RUST_MAJOR=$(echo $RUST_VERSION | cut -d'.' -f1)
RUST_MINOR=$(echo $RUST_VERSION | cut -d'.' -f2)

echo -e "${CYAN}${NC} Detected Rust version: $RUST_VERSION"

# Version check
if [ "$RUST_MAJOR" -lt "$REQUIRED_RUST_MAJOR" ] || \
   ([ "$RUST_MAJOR" -eq "$REQUIRED_RUST_MAJOR" ] && [ "$RUST_MINOR" -lt "$REQUIRED_RUST_MINOR" ]); then
    echo -e "${RED}${STATUS_ERR} Rust version $RUST_VERSION is too old${NC}"
    echo -e "${YELLOW}   HORUS requires Rust >= $REQUIRED_RUST_VERSION${NC}"
    echo ""
    echo -e "   Update Rust with: ${CYAN}rustup update stable${NC}"
    echo -e "   Or install specific version: ${CYAN}rustup install $REQUIRED_RUST_VERSION && rustup default $REQUIRED_RUST_VERSION${NC}"
    echo ""
    exit 1
else
    echo -e "${GREEN}${STATUS_OK} Rust version $RUST_VERSION meets requirement (>= $REQUIRED_RUST_VERSION)${NC}"
fi

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
                    sudo apt-get install -y gcc libc6-dev pkg-config libssl-dev libudev-dev libasound2-dev \
                        libclang-dev libopencv-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev \
                        libxinerama-dev libwayland-dev wayland-protocols libxkbcommon-dev
                    ;;
                fedora-based)
                    # Note: gcc is sufficient, Development Tools group includes C++ which is not needed
                    sudo dnf install -y gcc glibc-devel pkg-config openssl-devel systemd-devel alsa-lib-devel \
                        clang-devel opencv-devel libX11-devel libXrandr-devel libXi-devel \
                        libXcursor-devel libXinerama-devel wayland-devel wayland-protocols-devel \
                        libxkbcommon-devel
                    ;;
                arch-based)
                    # Note: gcc is sufficient, base-devel includes C++ which is not needed
                    sudo pacman -Sy --noconfirm gcc pkg-config openssl systemd alsa-lib \
                        clang opencv libx11 libxrandr libxi libxcursor libxinerama \
                        wayland wayland-protocols libxkbcommon
                    ;;
                opensuse)
                    # Note: gcc is sufficient, devel_basis includes C++ which is not needed
                    sudo zypper install -y gcc glibc-devel pkg-config libopenssl-devel libudev-devel alsa-devel \
                        clang-devel opencv-devel libX11-devel libXrandr-devel libXi-devel \
                        libXcursor-devel libXinerama-devel wayland-devel wayland-protocols-devel \
                        libxkbcommon-devel
                    ;;
                alpine)
                    # Note: gcc and musl-dev are sufficient for Rust
                    sudo apk add --no-cache gcc musl-dev pkgconfig openssl-dev eudev-dev alsa-lib-dev \
                        clang-dev opencv-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev \
                        libxinerama-dev wayland-dev wayland-protocols libxkbcommon-dev
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
                    brew install pkg-config opencv
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

# Check for required system libraries
echo ""
echo -e "${CYAN}${NC} Checking system dependencies..."

MISSING_LIBS=""

# Core libraries
if ! pkg-config --exists openssl 2>/dev/null; then
    echo -e "${YELLOW}${NC}  OpenSSL development library not found"
    MISSING_LIBS="${MISSING_LIBS} openssl"
fi

if ! pkg-config --exists libudev 2>/dev/null; then
    echo -e "${YELLOW}${NC}  udev development library not found"
    MISSING_LIBS="${MISSING_LIBS} udev"
fi

if ! pkg-config --exists alsa 2>/dev/null; then
    echo -e "${YELLOW}${NC}  ALSA development library not found"
    MISSING_LIBS="${MISSING_LIBS} alsa"
fi

# GUI/Graphics libraries (required for sim2d and dashboard)
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

if ! pkg-config --exists libv4l2 2>/dev/null; then
    echo -e "${YELLOW}${NC}  libv4l2 not found (optional - needed for camera support)"
    OPTIONAL_MISSING="${OPTIONAL_MISSING} libv4l2"
fi

if ! pkg-config --exists fontconfig 2>/dev/null; then
    echo -e "${YELLOW}${NC}  fontconfig not found (optional - improves text rendering)"
    OPTIONAL_MISSING="${OPTIONAL_MISSING} fontconfig"
fi

# Hardware driver libraries (optional - for real hardware access)
HARDWARE_MISSING=""

# Check for RealSense camera support
if ! pkg-config --exists realsense2 2>/dev/null; then
    echo -e "${YELLOW}${NC}  librealsense2 not found (optional - for RealSense depth cameras)"
    HARDWARE_MISSING="${HARDWARE_MISSING} realsense"
fi

# Check for CAN utilities (useful for debugging SocketCAN)
if ! command -v cansend &> /dev/null; then
    echo -e "${YELLOW}${NC}  can-utils not found (optional - for CAN bus debugging)"
    HARDWARE_MISSING="${HARDWARE_MISSING} can-utils"
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
    echo "    libvulkan-dev libfontconfig-dev libfreetype-dev \\"
    echo "    libv4l-dev"
    echo ""
    echo -e "${CYAN}Fedora/RHEL/CentOS:${NC}"
    echo "  sudo dnf install -y gcc glibc-devel pkg-config openssl-devel systemd-devel alsa-lib-devel \\"
    echo "    libX11-devel libXrandr-devel libXi-devel libXcursor-devel libXinerama-devel \\"
    echo "    wayland-devel wayland-protocols-devel libxkbcommon-devel \\"
    echo "    vulkan-devel fontconfig-devel freetype-devel \\"
    echo "    libv4l-devel"
    echo ""
    echo -e "${CYAN}Arch Linux:${NC}"
    echo "  sudo pacman -S gcc pkg-config openssl systemd alsa-lib \\"
    echo "    libx11 libxrandr libxi libxcursor libxinerama \\"
    echo "    wayland wayland-protocols libxkbcommon \\"
    echo "    vulkan-icd-loader fontconfig freetype2 \\"
    echo "    v4l-utils"
    echo ""
    echo -e "${CYAN}macOS:${NC}"
    echo "  xcode-select --install"
    echo "  brew install pkg-config"
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
    echo -e "${YELLOW}${NC} Some optional dependencies missing (camera/font support may be limited)"
fi

if [ ! -z "$HARDWARE_MISSING" ]; then
    echo -e "${CYAN}${NC}  Optional hardware driver packages available:"
    echo ""
    if [[ "$HARDWARE_MISSING" == *"realsense"* ]]; then
        echo -e "  ${CYAN}RealSense Depth Cameras:${NC}"
        echo "    Ubuntu/Debian: sudo apt install -y librealsense2-dev librealsense2-utils"
        echo "    See: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md"
        echo ""
    fi
    if [[ "$HARDWARE_MISSING" == *"can-utils"* ]]; then
        echo -e "  ${CYAN}CAN Bus Debugging Tools:${NC}"
        echo "    Ubuntu/Debian: sudo apt install -y can-utils"
        echo "    Usage: candump can0, cansend can0 123#DEADBEEF"
        echo ""
    fi
    echo -e "  ${CYAN}Note:${NC} Hardware features are optional. You can install these later if needed."
    echo ""
fi

# Check if Python is installed (for horus_py)
# Required Python version (must match pyproject.toml and verify.sh)
REQUIRED_PYTHON_MAJOR=3
REQUIRED_PYTHON_MINOR=9
REQUIRED_PYTHON_VERSION="${REQUIRED_PYTHON_MAJOR}.${REQUIRED_PYTHON_MINOR}"

if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version | awk '{print $2}')
    echo -e "${CYAN}${NC} Detected Python: $PYTHON_VERSION"

    # Check if Python version meets requirement
    PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
    PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

    if [ "$PYTHON_MAJOR" -gt "$REQUIRED_PYTHON_MAJOR" ] || \
       ([ "$PYTHON_MAJOR" -eq "$REQUIRED_PYTHON_MAJOR" ] && [ "$PYTHON_MINOR" -ge "$REQUIRED_PYTHON_MINOR" ]); then
        PYTHON_AVAILABLE=true
        echo -e "${GREEN}${STATUS_OK} Python version meets requirement (>= $REQUIRED_PYTHON_VERSION)${NC}"
    else
        echo -e "${YELLOW}${NC}  Python $REQUIRED_PYTHON_VERSION+ required for horus_py (found $PYTHON_VERSION)"
        echo -e "  horus_py will be skipped"
        PYTHON_AVAILABLE=false
    fi
else
    echo -e "${YELLOW}${NC}  Python3 not found - horus_py will be skipped (requires >= $REQUIRED_PYTHON_VERSION)"
    PYTHON_AVAILABLE=false
fi

# Check for pip (needed for Python bindings)
if [ "$PYTHON_AVAILABLE" = true ]; then
    if command -v pip3 &> /dev/null || command -v pip &> /dev/null; then
        echo -e "${CYAN}${NC} Detected pip: $(pip3 --version 2>/dev/null || pip --version)"
    else
        echo -e "${YELLOW}${NC}  pip not found - horus_py will be skipped"
        echo "  Install pip: sudo apt install python3-pip (Debian/Ubuntu)"
        PYTHON_AVAILABLE=false
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

# Build with automatic retry and error recovery
# Shows BOTH overall progress bar AND per-submodule progress bars
build_with_recovery() {
    local max_retries=3
    local retry=0

    # Define packages to build based on installation profile
    # Order matters: dependencies first, then dependents
    case "${INSTALL_PROFILE:-full}" in
        minimal)
            # Only essential runtime packages
            BUILD_PACKAGES=(
                "horus_macros"
                "horus_core"
                "horus"
                "horus_manager"
            )
            echo -e "${MAGENTA}   Profile: Minimal - Building core runtime only${NC}"
            ;;
        embedded)
            # Core packages + library + Python bindings, but no heavy simulators
            BUILD_PACKAGES=(
                "horus_macros"
                "horus_core"
                "horus"
                "horus_manager"
                "horus_library"
                "horus_py"
            )
            echo -e "${YELLOW}   Profile: Embedded - Skipping sim2d/sim3d (saves ~1GB RAM during build)${NC}"
            ;;
        full|*)
            # All packages including simulators and Python bindings
            BUILD_PACKAGES=(
                "horus_macros"
                "horus_core"
                "horus"
                "horus_manager"
                "horus_library"
                "horus_py"
                "sim2d"
                "sim3d"
            )
            echo -e "${GREEN}   Profile: Full - Building all packages including simulators${NC}"
            ;;
    esac

    # Estimated crate counts for each package (for progress calculation)
    # Based on actual dependency counts from cargo tree (fresh build)
    declare -A PACKAGE_CRATES=(
        ["horus_macros"]=10
        ["horus_core"]=350
        ["horus"]=50
        ["horus_manager"]=150
        ["horus_library"]=50
        ["horus_py"]=20
        ["sim2d"]=300
        ["sim3d"]=400
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
        echo -e "${CYAN}   Skipping: benchmarks, horus_py (installed from PyPI), tanksim, horus_router${NC}"
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

                # Check for errors
                if [[ "$line" =~ ^error\[E ]]; then
                    echo "$line" >> "$TEMP_OUTPUT"
                    build_status=1
                fi
            done < <(cargo build --release -p "$pkg" $([ "$pkg" = "horus_manager" ] && echo "--bin horus") 2>&1; echo "BUILD_EXIT_CODE:$?")

            # Extract exit code from the marker line
            if [[ "$last_line" =~ BUILD_EXIT_CODE:([0-9]+) ]]; then
                build_status="${BASH_REMATCH[1]}"
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
                echo -e "${YELLOW} Attempting recovery for failed package...${NC}"

                # Common fixes for build failures
                echo -e "${CYAN} Updating cargo index...${NC}"
                cargo update

                # Fix potential permission issues
                if grep -q "permission denied" "$LOG_FILE"; then
                    echo -e "${CYAN} Fixing permissions...${NC}"
                    chmod -R u+rwx target/ 2>/dev/null || true
                    chmod -R u+rwx ~/.cargo/ 2>/dev/null || true
                fi

                # Clear cargo cache if download failed
                if grep -q "failed to download\|failed to fetch" "$LOG_FILE"; then
                    echo -e "${CYAN} Clearing cargo cache...${NC}"
                    rm -rf ~/.cargo/registry/cache
                    rm -rf ~/.cargo/registry/index
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

# Install sim2d binary (only for full profile)
if [ "${INSTALL_PROFILE:-full}" = "full" ]; then
    if [ -f "target/release/sim2d" ]; then
        cp target/release/sim2d "$INSTALL_DIR/sim2d"
        chmod +x "$INSTALL_DIR/sim2d"
        echo -e "${GREEN}${NC} sim2d binary installed to $INSTALL_DIR/sim2d"
    fi
else
    echo -e "${YELLOW}[-]${NC} sim2d: Skipped (${INSTALL_PROFILE} profile)"
fi

# Install sim3d binary (only for full profile)
if [ "${INSTALL_PROFILE:-full}" = "full" ]; then
    if [ -f "target/release/sim3d" ]; then
        cp target/release/sim3d "$INSTALL_DIR/sim3d"
        chmod +x "$INSTALL_DIR/sim3d"
        echo -e "${GREEN}${NC} sim3d binary installed to $INSTALL_DIR/sim3d"
    fi
else
    echo -e "${YELLOW}[-]${NC} sim3d: Skipped (${INSTALL_PROFILE} profile)"
fi

echo ""

# Step 3: Create cache directory structure
echo -e "${CYAN}${STATUS_INFO} Setting up library cache...${NC}"

mkdir -p "$CACHE_DIR"

# Get version from Cargo.toml
HORUS_VERSION=$(grep -m1 '^version' horus/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_CORE_VERSION=$(grep -m1 '^version' horus_core/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_MACROS_VERSION=$(grep -m1 '^version' horus_macros/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_LIBRARY_VERSION=$(grep -m1 '^version' horus_library/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
HORUS_PY_VERSION=$(grep -m1 '^version' horus_py/Cargo.toml | sed 's/.*"\(.*\)".*/\1/')

echo -e "${CYAN}  ${NC} Detected versions:"
echo "    horus: $HORUS_VERSION"
echo "    horus_core: $HORUS_CORE_VERSION"
echo "    horus_macros: $HORUS_MACROS_VERSION"
echo "    horus_library: $HORUS_LIBRARY_VERSION"
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

# Copy horus_manager crate (CLI binary source)
mkdir -p "$HORUS_DIR/horus_manager"
cp horus_manager/Cargo.toml "$HORUS_DIR/horus_manager/" 2>/dev/null || true
cp -r horus_manager/src "$HORUS_DIR/horus_manager/" 2>/dev/null || true

# Copy horus_router crate
mkdir -p "$HORUS_DIR/horus_router"
cp horus_router/Cargo.toml "$HORUS_DIR/horus_router/" 2>/dev/null || true
cp -r horus_router/src "$HORUS_DIR/horus_router/" 2>/dev/null || true

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

# Copy procedural macro library
cp -r target/release/libhorus_macros.* "$HORUS_MACROS_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_macros*.so "$HORUS_MACROS_DIR/lib/" 2>/dev/null || true

# Also copy to target/release for Cargo
mkdir -p "$HORUS_MACROS_DIR/target/release"
cp -r target/release/libhorus_macros.so "$HORUS_MACROS_DIR/target/release/" 2>/dev/null || true
cp -r target/release/deps/libhorus_macros*.so "$HORUS_MACROS_DIR/target/release/" 2>/dev/null || true

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

# Copy compiled libraries
cp -r target/release/libhorus_library.* "$HORUS_LIBRARY_DIR/lib/" 2>/dev/null || true
cp -r target/release/deps/libhorus_library*.rlib "$HORUS_LIBRARY_DIR/lib/" 2>/dev/null || true

# Also copy to target/release
mkdir -p "$HORUS_LIBRARY_DIR/target/release"
cp -r target/release/libhorus_library*.rlib "$HORUS_LIBRARY_DIR/target/release/" 2>/dev/null || true

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

# Step 8: Install horus_py (Python bindings) - Optional
if [ "$PYTHON_AVAILABLE" = true ]; then
    echo -e "${CYAN}${NC} Installing horus_py@$HORUS_PY_VERSION (Python bindings - optional)..."

    # Try to install from PyPI (pre-built wheel)
    echo -e "${CYAN}  ${NC} Attempting to install from PyPI..."

    # Try pip install with --user flag first
    if pip3 install horus --user --quiet 2>/dev/null; then
        echo -e "${GREEN}${NC} Installed horus Python package from PyPI"

        # Verify installation
        if python3 -c "import horus; print(f'horus {horus.__version__}')" 2>/dev/null; then
            INSTALLED_VERSION=$(python3 -c "import horus; print(horus.__version__)" 2>/dev/null)
            echo -e "${GREEN}${NC} Python bindings working (version: $INSTALLED_VERSION)"
        else
            echo -e "${YELLOW}[-]${NC} Python bindings installed but import failed"
        fi
    else
        # If pip install fails, it's optional - don't build from source
        echo -e "${YELLOW}[-]${NC} Python bindings: Not available on PyPI yet (optional)"
        echo ""
        echo -e "  ${CYAN}Python bindings will be available after the first release.${NC}"
        echo -e "  For now, you can:"
        echo -e "    1. Wait for the next release (recommended)"
        echo -e "    2. Build from source (developers only):"
        echo -e "       ${CYAN}cd horus_py && pip install maturin && maturin develop --release${NC}"
        echo ""
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

# Save install profile for verify.sh and uninstall.sh
echo "$INSTALL_PROFILE" > "$HOME/.horus/install_profile"

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

if [ "${INSTALL_PROFILE:-full}" = "full" ]; then
    if [ -x "$INSTALL_DIR/sim2d" ]; then
        echo -e "${GREEN}${NC} sim2d binary: OK"
    else
        echo -e "${YELLOW}[-]${NC} sim2d binary: Not installed"
    fi

    if [ -x "$INSTALL_DIR/sim3d" ]; then
        echo -e "${GREEN}${NC} sim3d binary: OK"
    else
        echo -e "${YELLOW}[-]${NC} sim3d binary: Not installed"
    fi
else
    echo -e "${CYAN}[-]${NC} sim2d/sim3d: Skipped (${INSTALL_PROFILE} profile)"
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

