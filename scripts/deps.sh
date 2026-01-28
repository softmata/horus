#!/bin/bash
# HORUS Shared Functions Library
# Sourced by install.sh, uninstall.sh
#
# Usage: source "$(dirname "$0")/scripts/deps.sh" 2>/dev/null || source "$(dirname "$0")/deps.sh"
#
# Provides:
#   - Color definitions
#   - Status indicators
#   - OS detection (Linux, macOS, BSD, Windows, Solaris)
#   - Spinner and progress bar functions
#   - Shared memory path functions
#   - Dependency checking and installation

# ============================================================================
# COLORS (only set if not already defined)
# ============================================================================
: "${RED:=\033[0;31m}"
: "${GREEN:=\033[0;32m}"
: "${YELLOW:=\033[1;33m}"
: "${CYAN:=\033[0;36m}"
: "${BLUE:=\033[0;34m}"
: "${MAGENTA:=\033[0;35m}"
: "${WHITE:=\033[1;37m}"
: "${NC:=\033[0m}"

# ============================================================================
# STATUS INDICATORS (only set if not already defined)
# ============================================================================
: "${STATUS_OK:=[+]}"
: "${STATUS_ERR:=[-]}"
: "${STATUS_WARN:=[!]}"
: "${STATUS_INFO:=[*]}"

# Unicode symbols for fancy output
: "${CHECK:=${GREEN}✓${NC}}"
: "${CROSS:=${RED}✗${NC}}"
: "${WARN:=${YELLOW}!${NC}}"
: "${INFO:=${CYAN}i${NC}}"
: "${SKIP:=${WHITE}-${NC}}"

# ============================================================================
# SPINNER FUNCTION
# ============================================================================
# Usage: long_command & spin $! "Processing..."
spin() {
    local pid=$1
    local msg="$2"
    local spin_chars=('.' '..' '...' '....')
    local i=0
    tput civis 2>/dev/null || true  # Hide cursor
    while kill -0 $pid 2>/dev/null; do
        printf "\r  ${spin_chars[$i]} ${msg}"
        i=$(( (i + 1) % ${#spin_chars[@]} ))
        sleep 0.25
    done
    tput cnorm 2>/dev/null || true  # Show cursor
    printf "\r\033[K"  # Clear line
}

# ============================================================================
# OS DETECTION - Comprehensive cross-platform detection
# ============================================================================
# Detects: Linux, macOS, Windows, FreeBSD, OpenBSD, NetBSD, DragonFly, Solaris
# Returns: "os_type:os_distro" (e.g., "linux:debian-based", "bsd:freebsd")
detect_os() {
    local os_type="unknown"
    local os_distro="unknown"

    case "$(uname -s)" in
        Linux*)
            os_type="linux"
            if [ -f /etc/os-release ]; then
                . /etc/os-release
                case "$ID" in
                    ubuntu|debian|pop|mint|elementary|zorin|kali|raspbian)
                        os_distro="debian-based"
                        ;;
                    fedora|rhel|centos|rocky|alma|nobara)
                        os_distro="fedora-based"
                        ;;
                    arch|manjaro|endeavouros|garuda)
                        os_distro="arch-based"
                        ;;
                    opensuse*|suse*)
                        os_distro="opensuse"
                        ;;
                    alpine)
                        os_distro="alpine"
                        ;;
                    nixos)
                        os_distro="nixos"
                        ;;
                    void)
                        os_distro="void"
                        ;;
                    *)
                        # Try to detect based on package manager
                        if command -v apt-get &>/dev/null; then
                            os_distro="debian-based"
                        elif command -v dnf &>/dev/null; then
                            os_distro="fedora-based"
                        elif command -v pacman &>/dev/null; then
                            os_distro="arch-based"
                        elif command -v zypper &>/dev/null; then
                            os_distro="opensuse"
                        elif command -v apk &>/dev/null; then
                            os_distro="alpine"
                        elif command -v xbps-install &>/dev/null; then
                            os_distro="void"
                        fi
                        ;;
                esac
            fi
            # Check for WSL
            if grep -qi microsoft /proc/version 2>/dev/null; then
                os_type="wsl"
            fi
            ;;
        Darwin*)
            os_type="macos"
            os_distro="macos"
            ;;
        FreeBSD*)
            os_type="bsd"
            os_distro="freebsd"
            ;;
        OpenBSD*)
            os_type="bsd"
            os_distro="openbsd"
            ;;
        NetBSD*)
            os_type="bsd"
            os_distro="netbsd"
            ;;
        DragonFly*)
            os_type="bsd"
            os_distro="dragonfly"
            ;;
        SunOS*|Solaris*)
            os_type="solaris"
            os_distro="solaris"
            ;;
        MINGW*|MSYS*|CYGWIN*)
            os_type="windows"
            os_distro="windows"
            ;;
    esac

    echo "$os_type:$os_distro"
}

# ============================================================================
# SHARED MEMORY PATH FUNCTIONS
# ============================================================================
# Returns the appropriate shared memory base directory for the current OS
get_shm_base_dir() {
    case "$(uname -s)" in
        Linux*)
            echo "/dev/shm/horus"
            ;;
        Darwin*|FreeBSD*|OpenBSD*|NetBSD*|DragonFly*)
            echo "/tmp/horus"
            ;;
        MINGW*|MSYS*|CYGWIN*)
            echo "${TEMP:-/tmp}/horus"
            ;;
        *)
            echo "/tmp/horus"
            ;;
    esac
}

# Returns the appropriate shared memory logs path for the current OS
get_shm_logs_path() {
    case "$(uname -s)" in
        Linux*)
            echo "/dev/shm/horus_logs"
            ;;
        Darwin*|FreeBSD*|OpenBSD*|NetBSD*|DragonFly*)
            echo "/tmp/horus_logs"
            ;;
        MINGW*|MSYS*|CYGWIN*)
            echo "${TEMP:-/tmp}/horus_logs"
            ;;
        *)
            echo "/tmp/horus_logs"
            ;;
    esac
}

# ============================================================================
# LIBRARY FILE EXTENSION
# ============================================================================
# Returns the appropriate library file extension for the current OS
get_lib_extension() {
    case "$(uname -s)" in
        Darwin*)
            echo "dylib"
            ;;
        MINGW*|MSYS*|CYGWIN*)
            echo "dll"
            ;;
        *)
            echo "so"
            ;;
    esac
}

# ============================================================================
# PLATFORM SUPPORT CHECK
# ============================================================================
# Checks if the current platform is supported and returns appropriate guidance
# Returns: 0 for full support, 1 for experimental, 2 for unsupported
check_platform_support() {
    local os_type="$1"
    local os_distro="$2"

    case "$os_type" in
        linux|wsl|macos)
            return 0  # Full support
            ;;
        bsd)
            return 1  # Experimental
            ;;
        windows|solaris|unknown)
            return 2  # Not supported
            ;;
        *)
            return 2
            ;;
    esac
}

# Initialize OS detection if not already done
if [ -z "$OS_TYPE" ] || [ -z "$OS_DISTRO" ]; then
    _OS_INFO=$(detect_os)
    OS_TYPE="${_OS_INFO%%:*}"
    OS_DISTRO="${_OS_INFO##*:}"
fi

#=====================================
# Dependency Definitions
#=====================================

# All required system dependencies for HORUS
# Format: "pkg-config-name:human-readable-name:required(yes/no)"
HORUS_SYSTEM_DEPS=(
    # Build essentials (C compiler only - Rust doesn't need C++)
    "compiler:C Compiler:yes"
    "pkg-config:pkg-config:yes"

    # Core libraries
    "openssl:OpenSSL:yes"
    "libudev:udev (device management):linux-only"
    "alsa:ALSA (audio):linux-only"
    "libclang:libclang (for bindings):yes"

    # Graphics/Wayland (required for Bevy/egui)
    "wayland-client:Wayland client:linux-only"
    "wayland-cursor:Wayland cursor:linux-only"
    "wayland-protocols:Wayland protocols:linux-only"
    "xkbcommon:XKB common:linux-only"

    # X11 (fallback/alternative to Wayland)
    "x11:X11:linux-only"
    "xrandr:Xrandr:linux-only"
    "xi:Xi (input):linux-only"
    "xcursor:Xcursor:linux-only"
    "xinerama:Xinerama:linux-only"
)

#=====================================
# Dependency Checking Functions
#=====================================

# Check if a specific dependency is installed
# Returns 0 if installed, 1 if not
check_dep() {
    local dep="$1"

    case "$dep" in
        compiler)
            command -v cc &>/dev/null || command -v gcc &>/dev/null || command -v clang &>/dev/null
            return $?
            ;;
        pkg-config)
            command -v pkg-config &>/dev/null
            return $?
            ;;
        libclang)
            # Check for libclang library
            if [ "$OS_TYPE" = "macos" ]; then
                [ -d "/Library/Developer/CommandLineTools" ] || xcode-select -p &>/dev/null
            else
                ldconfig -p 2>/dev/null | grep -q libclang || [ -f /usr/lib/llvm-*/lib/libclang.so ]
            fi
            return $?
            ;;
        wayland-client|wayland-cursor)
            pkg-config --exists "$dep" 2>/dev/null
            return $?
            ;;
        wayland-protocols)
            pkg-config --exists wayland-protocols 2>/dev/null || [ -d "/usr/share/wayland-protocols" ]
            return $?
            ;;
        xkbcommon)
            pkg-config --exists xkbcommon 2>/dev/null
            return $?
            ;;
        x11)
            pkg-config --exists x11 2>/dev/null
            return $?
            ;;
        xrandr)
            pkg-config --exists xrandr 2>/dev/null
            return $?
            ;;
        xi)
            pkg-config --exists xi 2>/dev/null
            return $?
            ;;
        xcursor)
            pkg-config --exists xcursor 2>/dev/null
            return $?
            ;;
        xinerama)
            pkg-config --exists xinerama 2>/dev/null
            return $?
            ;;
        libudev)
            pkg-config --exists libudev 2>/dev/null
            return $?
            ;;
        alsa)
            pkg-config --exists alsa 2>/dev/null
            return $?
            ;;
        openssl)
            pkg-config --exists openssl 2>/dev/null || pkg-config --exists libssl 2>/dev/null
            return $?
            ;;
        *)
            pkg-config --exists "$dep" 2>/dev/null
            return $?
            ;;
    esac
}

# Check all dependencies and return list of missing ones
# Output: space-separated list of missing dependency names
check_all_deps() {
    local missing=""
    local dep_info dep_name dep_desc dep_required

    for dep_info in "${HORUS_SYSTEM_DEPS[@]}"; do
        IFS=':' read -r dep_name dep_desc dep_required <<< "$dep_info"

        # Skip linux-only deps on non-Linux
        if [ "$dep_required" = "linux-only" ] && [ "$OS_TYPE" != "linux" ] && [ "$OS_TYPE" != "wsl" ]; then
            continue
        fi

        if ! check_dep "$dep_name"; then
            missing="$missing $dep_name"
        fi
    done

    echo "$missing" | xargs  # Trim whitespace
}

# Get human-readable list of missing dependencies
get_missing_deps_readable() {
    local missing=""
    local dep_info dep_name dep_desc dep_required

    for dep_info in "${HORUS_SYSTEM_DEPS[@]}"; do
        IFS=':' read -r dep_name dep_desc dep_required <<< "$dep_info"

        # Skip linux-only deps on non-Linux
        if [ "$dep_required" = "linux-only" ] && [ "$OS_TYPE" != "linux" ] && [ "$OS_TYPE" != "wsl" ]; then
            continue
        fi

        if ! check_dep "$dep_name"; then
            if [ -n "$missing" ]; then
                missing="$missing, $dep_desc"
            else
                missing="$dep_desc"
            fi
        fi
    done

    echo "$missing"
}

#=====================================
# Dependency Installation Functions
#=====================================

# Get the install command for the current OS
get_install_command() {
    case "$OS_DISTRO" in
        debian-based)
            echo "sudo apt-get install -y"
            ;;
        fedora-based)
            echo "sudo dnf install -y"
            ;;
        arch-based)
            echo "sudo pacman -S --noconfirm"
            ;;
        opensuse)
            echo "sudo zypper install -y"
            ;;
        alpine)
            echo "sudo apk add --no-cache"
            ;;
        macos)
            echo "brew install"
            ;;
        *)
            echo ""
            ;;
    esac
}

# Get the package names for the current OS
get_packages_for_os() {
    case "$OS_DISTRO" in
        debian-based)
            # Note: gcc is sufficient, g++ not needed for Rust projects
            echo "gcc libc6-dev pkg-config libssl-dev libudev-dev libasound2-dev libclang-dev libwayland-dev wayland-protocols libxkbcommon-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev mesa-vulkan-drivers libvulkan1 libvulkan-dev"
            ;;
        fedora-based)
            # Note: gcc is sufficient, gcc-c++ not needed for Rust projects
            echo "gcc glibc-devel pkg-config openssl-devel systemd-devel alsa-lib-devel clang-devel wayland-devel wayland-protocols-devel libxkbcommon-devel libX11-devel libXrandr-devel libXi-devel libXcursor-devel libXinerama-devel vulkan-loader-devel mesa-vulkan-drivers"
            ;;
        arch-based)
            # Note: gcc is sufficient, base-devel includes both but that's OK
            echo "gcc pkg-config openssl systemd alsa-lib clang wayland wayland-protocols libxkbcommon libx11 libxrandr libxi libxcursor libxinerama vulkan-icd-loader vulkan-headers"
            ;;
        opensuse)
            # Note: gcc is sufficient, gcc-c++ not needed for Rust projects
            echo "gcc glibc-devel pkg-config libopenssl-devel libudev-devel alsa-devel clang-devel wayland-devel wayland-protocols-devel libxkbcommon-devel libX11-devel libXrandr-devel libXi-devel libXcursor-devel libXinerama-devel libvulkan1 vulkan-devel"
            ;;
        alpine)
            # Note: gcc and musl-dev are sufficient for Rust
            echo "gcc musl-dev pkgconfig openssl-dev eudev-dev alsa-lib-dev clang-dev wayland-dev wayland-protocols libxkbcommon-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev vulkan-loader-dev mesa-vulkan-swrast"
            ;;
        macos)
            # macOS handles most of this through Xcode Command Line Tools
            echo ""
            ;;
        *)
            echo ""
            ;;
    esac
}

# Install all system dependencies
# Returns 0 on success, 1 on failure
install_system_deps() {
    local auto_yes="${1:-false}"  # Pass "true" to skip confirmation

    # Check what's missing
    local missing=$(check_all_deps)

    if [ -z "$missing" ]; then
        echo -e "${GREEN}[+]${NC} All system dependencies are installed"
        return 0
    fi

    local missing_readable=$(get_missing_deps_readable)
    echo -e "${YELLOW}[!]${NC} Missing dependencies: $missing_readable"
    echo ""

    # Get install command and packages
    local install_cmd=$(get_install_command)
    local packages=$(get_packages_for_os)

    if [ -z "$install_cmd" ] || [ -z "$packages" ]; then
        echo -e "${RED}[x]${NC} Cannot auto-install for $OS_DISTRO"
        echo ""
        echo "Please install the following manually:"
        echo "  - C compiler (gcc or clang) - C++ is NOT required"
        echo "  - pkg-config"
        echo "  - OpenSSL development files"
        echo "  - Wayland development files"
        echo "  - X11 development files"
        echo "  - ALSA development files (Linux)"
        echo "  - libclang"
        return 1
    fi

    # Confirm installation
    if [ "$auto_yes" != "true" ]; then
        echo "Will run: $install_cmd $packages"
        echo ""
        read -p "$(echo -e ${CYAN}?${NC}) Install missing dependencies? [Y/n]: " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Nn]$ ]]; then
            echo -e "${YELLOW}[!]${NC} Skipping dependency installation"
            return 1
        fi
    fi

    echo -e "${CYAN}[*]${NC} Installing dependencies for $OS_DISTRO..."
    echo ""

    # Special handling for different distros
    case "$OS_DISTRO" in
        debian-based)
            sudo apt-get update
            $install_cmd $packages
            ;;
        fedora-based)
            # Also install development tools group
            sudo dnf groupinstall -y "Development Tools" 2>/dev/null || true
            $install_cmd $packages
            ;;
        arch-based)
            # Sync package database first
            sudo pacman -Sy
            $install_cmd $packages
            ;;
        macos)
            # Check for Xcode Command Line Tools
            if ! xcode-select -p &>/dev/null; then
                echo -e "${CYAN}[*]${NC} Installing Xcode Command Line Tools..."
                xcode-select --install
                echo ""
                echo -e "${YELLOW}[!]${NC} Please wait for Xcode tools to install, then re-run this script"
                return 1
            fi
            # Check for Homebrew
            if ! command -v brew &>/dev/null; then
                echo -e "${YELLOW}[!]${NC} Homebrew not found. Install from: https://brew.sh"
                return 1
            fi
            ;;
        *)
            $install_cmd $packages
            ;;
    esac

    local result=$?

    if [ $result -eq 0 ]; then
        echo ""
        echo -e "${GREEN}[+]${NC} Dependencies installed successfully"
        return 0
    else
        echo ""
        echo -e "${RED}[x]${NC} Some dependencies may have failed to install"
        return 1
    fi
}

#=====================================
# Rust Installation
#=====================================

# Check and install Rust if needed
ensure_rust() {
    local auto_yes="${1:-false}"

    if command -v cargo &>/dev/null; then
        local rust_version=$(rustc --version | awk '{print $2}')
        echo -e "${GREEN}[+]${NC} Rust is installed: $rust_version"
        return 0
    fi

    echo -e "${YELLOW}[!]${NC} Rust is not installed"

    if [ "$auto_yes" != "true" ]; then
        read -p "$(echo -e ${CYAN}?${NC}) Install Rust automatically? [Y/n]: " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Nn]$ ]]; then
            echo -e "${RED}[x]${NC} Rust is required. Install from: https://rustup.rs"
            return 1
        fi
    fi

    echo -e "${CYAN}[*]${NC} Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

    # Source cargo environment
    if [ -f "$HOME/.cargo/env" ]; then
        source "$HOME/.cargo/env"
    fi

    if command -v cargo &>/dev/null; then
        echo -e "${GREEN}[+]${NC} Rust installed successfully"
        return 0
    else
        echo -e "${RED}[x]${NC} Failed to install Rust"
        return 1
    fi
}

#=====================================
# Verification Functions
#=====================================

# Print a detailed dependency status report
print_dep_status() {
    local dep_info dep_name dep_desc dep_required
    local check_count=0
    local pass_count=0
    local fail_count=0
    local skip_count=0

    echo -e "${MAGENTA}System Dependencies:${NC}"
    echo ""

    for dep_info in "${HORUS_SYSTEM_DEPS[@]}"; do
        IFS=':' read -r dep_name dep_desc dep_required <<< "$dep_info"

        # Skip linux-only deps on non-Linux
        if [ "$dep_required" = "linux-only" ] && [ "$OS_TYPE" != "linux" ] && [ "$OS_TYPE" != "wsl" ]; then
            skip_count=$((skip_count + 1))
            continue
        fi

        check_count=$((check_count + 1))

        if check_dep "$dep_name"; then
            # Try to get version
            local version=""
            case "$dep_name" in
                compiler)
                    version=$(cc --version 2>/dev/null | head -n1 | cut -d' ' -f1-3 || echo "installed")
                    ;;
                pkg-config)
                    version=$(pkg-config --version 2>/dev/null || echo "installed")
                    ;;
                *)
                    version=$(pkg-config --modversion "$dep_name" 2>/dev/null || echo "installed")
                    ;;
            esac
            echo -e "  ${GREEN}[+]${NC} $dep_desc: $version"
            pass_count=$((pass_count + 1))
        else
            echo -e "  ${RED}[x]${NC} $dep_desc: Not found"
            fail_count=$((fail_count + 1))
        fi
    done

    echo ""
    echo -e "  Checked: $check_count | ${GREEN}Passed: $pass_count${NC} | ${RED}Failed: $fail_count${NC} | Skipped: $skip_count"

    return $fail_count
}

#=====================================
# Utility Functions
#=====================================

# Check if running with sudo/root (some operations need it)
check_sudo_available() {
    if [ "$EUID" -eq 0 ]; then
        return 0
    fi

    if command -v sudo &>/dev/null; then
        # Test if sudo works (might need password)
        if sudo -n true 2>/dev/null; then
            return 0
        else
            # Sudo exists but might need password - that's OK
            return 0
        fi
    fi

    return 1
}

# Export functions and variables for use by other scripts
export -f detect_os check_dep check_all_deps get_missing_deps_readable
export -f get_install_command get_packages_for_os install_system_deps
export -f ensure_rust print_dep_status check_sudo_available
export OS_TYPE OS_DISTRO
