#!/bin/bash
# HORUS Uninstallation Script v2.7.0
# Complete removal of HORUS CLI, libraries, binaries, cache, and artifacts
# Cross-platform: Linux, macOS, Windows (Git Bash/MSYS2)
# Matches install.sh v2.6.0

set -e  # Exit on error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source shared functions from deps.sh (provides colors, status indicators, OS detection, spinner, shm paths)
if [ -f "$SCRIPT_DIR/scripts/deps.sh" ]; then
    source "$SCRIPT_DIR/scripts/deps.sh"
    DEPS_SOURCED=true
else
    DEPS_SOURCED=false
    # Minimal fallback if deps.sh not found
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    CYAN='\033[0;36m'
    BLUE='\033[0;34m'
    MAGENTA='\033[0;35m'
    WHITE='\033[1;37m'
    NC='\033[0m'
    STATUS_OK="[+]"
    STATUS_ERR="[-]"
    STATUS_WARN="[!]"
    STATUS_INFO="[*]"
    # Fallback spinner
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
    # Fallback shared memory paths (namespaced layout)
    get_shm_parent_dir() {
        case "$(uname -s)" in
            Linux*) echo "/dev/shm" ;;
            Darwin*|FreeBSD*|OpenBSD*|NetBSD*) echo "/tmp" ;;
            MINGW*|MSYS*|CYGWIN*) echo "${TEMP:-/tmp}" ;;
            *) echo "/tmp" ;;
        esac
    }
    get_shm_base_dir() { get_shm_parent_dir; }
    get_shm_glob() { echo "$(get_shm_parent_dir)/horus_*"; }
    get_shm_logs_path() { echo "$(get_shm_parent_dir)/horus_*/logs"; }
fi

# ============================================================================
# PROGRESS BAR FUNCTIONS - Uninstall-specific
# ============================================================================

# Global uninstall progress tracking
UNINSTALL_TOTAL_STEPS=5
UNINSTALL_CURRENT_STEP=0
UNINSTALL_START_TIME=0

# Initialize uninstall progress
init_uninstall_progress() {
    UNINSTALL_TOTAL_STEPS=$1
    UNINSTALL_CURRENT_STEP=0
    UNINSTALL_START_TIME=$(date +%s)
}

# Update uninstall progress bar
update_uninstall_progress() {
    local step_name="$1"
    UNINSTALL_CURRENT_STEP=$((UNINSTALL_CURRENT_STEP + 1))

    local percent=0
    if [ "$UNINSTALL_TOTAL_STEPS" -gt 0 ]; then
        percent=$((UNINSTALL_CURRENT_STEP * 100 / UNINSTALL_TOTAL_STEPS))
    fi

    # Calculate ETA
    local elapsed=$(($(date +%s) - UNINSTALL_START_TIME))
    local eta_str=""
    if [ "$elapsed" -gt 0 ] && [ "$percent" -gt 0 ] && [ "$percent" -lt 100 ]; then
        local total_estimated=$((elapsed * 100 / percent))
        local remaining=$((total_estimated - elapsed))
        if [ "$remaining" -gt 0 ]; then
            eta_str=" ETA: ${remaining}s"
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
    printf "\r  ${STATUS_INFO} [${bar}] %3d%% %-25s${eta_str}    " "$percent" "$step_name"
}

# Complete uninstall progress
complete_uninstall_progress() {
    local elapsed=$(($(date +%s) - UNINSTALL_START_TIME))
    printf "\r  ${STATUS_OK} [█████████████████████████] 100%% Uninstall completed in ${elapsed}s    \n"
}

# Detect OS for platform-specific paths
detect_platform() {
    case "$(uname -s)" in
        Darwin*) echo "macos" ;;
        MINGW*|MSYS*|CYGWIN*) echo "windows" ;;
        Linux*)
            if grep -qE "(Microsoft|WSL)" /proc/version 2>/dev/null; then
                echo "wsl"
            else
                echo "linux"
            fi
            ;;
        FreeBSD*|OpenBSD*|NetBSD*) echo "bsd" ;;
        *) echo "linux" ;;
    esac
}
PLATFORM="$(detect_platform)"

# Determine installation paths (platform-aware)
INSTALL_DIR="$HOME/.cargo/bin"
HORUS_DIR="$HOME/.horus"
CACHE_DIR="$HORUS_DIR/cache"
TARGET_DIR="$HORUS_DIR/target"
SHM_PARENT="$(get_shm_parent_dir)"
SHM_GLOB="$(get_shm_glob)"

# Platform-specific config directories
case "$PLATFORM" in
    macos)
        HORUS_APP_SUPPORT="$HOME/Library/Application Support/horus"
        HORUS_CACHES="$HOME/Library/Caches/horus"
        BINARY_NAME="horus"
        ;;
    windows)
        HORUS_APPDATA="${APPDATA:-$HOME/AppData/Roaming}/horus"
        HORUS_LOCALAPPDATA="${LOCALAPPDATA:-$HOME/AppData/Local}/horus"
        BINARY_NAME="horus.exe"
        ;;
    *)
        BINARY_NAME="horus"
        ;;
esac

# ============================================================================
# PROFILE DETECTION
# ============================================================================

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

# Binaries installed by HORUS (depends on profile)
case "$INSTALL_PROFILE" in
    minimal)
        BINARIES=("horus")
        ;;
    embedded)
        BINARIES=("horus")
        ;;
    full|*)
        BINARIES=("horus")
        ;;
esac

# Shell completion paths
BASH_COMPLETION_PATHS=(
    "$HOME/.bash_completion.d/horus"
    "/etc/bash_completion.d/horus"
    "$HORUS_DIR/completions/horus.bash"
)
ZSH_COMPLETION_PATHS=(
    "$HOME/.zsh/completions/_horus"
    "/usr/share/zsh/site-functions/_horus"
    "$HORUS_DIR/completions/_horus"
    "$HORUS_DIR/completions/horus.zsh"
)
FISH_COMPLETION_PATHS=(
    "$HOME/.config/fish/completions/horus.fish"
    "/usr/share/fish/completions/horus.fish"
    "$HORUS_DIR/completions/horus.fish"
)

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${WHITE}   HORUS Uninstallation Script v2.7.0${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "  ${CYAN}Install Profile:${NC} ${INSTALL_PROFILE}"
echo ""

# Calculate sizes
calculate_sizes() {
    local total_size=0

    # Binaries
    for bin in "${BINARIES[@]}"; do
        if [ -f "$INSTALL_DIR/$bin" ]; then
            size=$(du -k "$INSTALL_DIR/$bin" 2>/dev/null | cut -f1)
            total_size=$((total_size + size))
        fi
    done

    # HORUS directory
    if [ -d "$HORUS_DIR" ]; then
        size=$(du -sk "$HORUS_DIR" 2>/dev/null | cut -f1)
        total_size=$((total_size + size))
    fi

    # Shared memory (all horus_* namespace dirs)
    for ns_dir in $SHM_GLOB; do
        if [ -d "$ns_dir" ]; then
            size=$(du -sk "$ns_dir" 2>/dev/null | cut -f1)
            total_size=$((total_size + ${size:-0}))
        fi
    done

    echo $total_size
}

# Show what will be removed
echo -e "${MAGENTA}Components to remove:${NC}"
echo ""

# Binaries
echo -e "  ${CYAN}Binaries:${NC}"
BINARY_COUNT=0
for bin in "${BINARIES[@]}"; do
    if [ -f "$INSTALL_DIR/$bin" ]; then
        size=$(du -h "$INSTALL_DIR/$bin" 2>/dev/null | cut -f1)
        echo -e "    [x] $INSTALL_DIR/$bin ($size)"
        BINARY_COUNT=$((BINARY_COUNT + 1))
    fi
done
[ $BINARY_COUNT -eq 0 ] && echo -e "    ${YELLOW}(no binaries found)${NC}"

# HORUS directory
echo ""
echo -e "  ${CYAN}HORUS Data:${NC}"
if [ -d "$HORUS_DIR" ]; then
    size=$(du -sh "$HORUS_DIR" 2>/dev/null | cut -f1)
    echo -e "    [x] $HORUS_DIR/ ($size)"

    # Show subdirectories
    if [ -d "$CACHE_DIR" ]; then
        cache_size=$(du -sh "$CACHE_DIR" 2>/dev/null | cut -f1)
        echo -e "        - cache/ ($cache_size) - library cache & pre-compiled deps"
    fi
    if [ -f "$HORUS_DIR/config.toml" ]; then
        echo -e "        - config.toml - user settings"
    fi
    if [ -f "$HORUS_DIR/credentials" ] || [ -f "$HORUS_DIR/auth.json" ]; then
        echo -e "        - credentials - authentication data"
    fi
    if [ -f "$HORUS_DIR/install_profile" ]; then
        echo -e "        - install_profile - installation type"
    fi
else
    echo -e "    ${YELLOW}(~/.horus not found)${NC}"
fi

# Shared memory
echo ""
echo -e "  ${CYAN}Shared Memory:${NC}"
SHM_NS_COUNT=0
for ns_dir in $SHM_GLOB; do
    if [ -d "$ns_dir" ]; then
        ns_size=$(du -sh "$ns_dir" 2>/dev/null | cut -f1)
        ns_name=$(basename "$ns_dir")
        echo -e "    [x] $ns_dir/ ($ns_size)"
        SHM_NS_COUNT=$((SHM_NS_COUNT + 1))
    fi
done
[ $SHM_NS_COUNT -eq 0 ] && echo -e "    ${YELLOW}(no shared memory data)${NC}"

# Shell completions
echo ""
echo -e "  ${CYAN}Shell Completions:${NC}"
COMP_COUNT=0
for path in "${BASH_COMPLETION_PATHS[@]}" "${ZSH_COMPLETION_PATHS[@]}" "${FISH_COMPLETION_PATHS[@]}"; do
    if [ -f "$path" ]; then
        echo -e "    [x] $path"
        COMP_COUNT=$((COMP_COUNT + 1))
    fi
done
[ $COMP_COUNT -eq 0 ] && echo -e "    ${YELLOW}(no completions found)${NC}"

# Estimate total
TOTAL_KB=$(calculate_sizes)
if [ $TOTAL_KB -gt 1024 ]; then
    TOTAL_MB=$((TOTAL_KB / 1024))
    echo ""
    echo -e "  ${WHITE}Total space to reclaim: ~${TOTAL_MB}MB${NC}"
else
    echo ""
    echo -e "  ${WHITE}Total space to reclaim: ~${TOTAL_KB}KB${NC}"
fi

echo ""
echo -e "${BLUE}--------------------------------------------${NC}"
echo ""

# Ask for confirmation
read -p "$(echo -e ${YELLOW}?${NC}) Are you sure you want to uninstall HORUS? [y/N]: " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo -e "${GREEN}Uninstallation cancelled.${NC}"
    exit 0
fi

echo ""
echo -e "${CYAN}${STATUS_INFO} Uninstalling HORUS...${NC}"
echo ""

REMOVED=0
SKIPPED=0

# Initialize progress tracking (6 main steps)
init_uninstall_progress 6

#=====================================
# 1. Remove binaries
#=====================================
update_uninstall_progress "Removing binaries"
echo ""

for bin in "${BINARIES[@]}"; do
    # Try both with and without .exe extension (Windows compat)
    for ext in "" ".exe"; do
        if [ -f "$INSTALL_DIR/${bin}${ext}" ]; then
            rm -f "$INSTALL_DIR/${bin}${ext}"
            echo -e "  ${GREEN}[+]${NC} Removed ${bin}${ext}"
            REMOVED=$((REMOVED + 1))
        fi
    done
done

#=====================================
# 2. Remove shell completions
#=====================================
update_uninstall_progress "Removing completions"
echo ""

for path in "${BASH_COMPLETION_PATHS[@]}" "${ZSH_COMPLETION_PATHS[@]}" "${FISH_COMPLETION_PATHS[@]}"; do
    if [ -f "$path" ]; then
        rm -f "$path" 2>/dev/null || true
        echo -e "  ${GREEN}[+]${NC} Removed $(basename $path)"
        REMOVED=$((REMOVED + 1))
    fi
done

# Remove completions directory if empty
if [ -d "$HORUS_DIR/completions" ]; then
    rmdir "$HORUS_DIR/completions" 2>/dev/null || true
fi

#=====================================
# 3. Remove shared memory
#=====================================
update_uninstall_progress "Cleaning shared memory"
echo ""

for ns_dir in $SHM_GLOB; do
    if [ -d "$ns_dir" ]; then
        ns_name=$(basename "$ns_dir")
        rm -rf "$ns_dir" 2>/dev/null || true
        echo -e "  ${GREEN}[+]${NC} Removed $ns_name"
        REMOVED=$((REMOVED + 1))
    fi
done

#=====================================
# 4. Remove HORUS directory
#=====================================
update_uninstall_progress "Removing HORUS data"
echo ""

if [ -d "$HORUS_DIR" ]; then
    # Check for user data
    HAS_CONFIG=false
    HAS_CREDENTIALS=false

    [ -f "$HORUS_DIR/config.toml" ] && HAS_CONFIG=true
    [ -f "$HORUS_DIR/credentials" ] || [ -f "$HORUS_DIR/auth.json" ] && HAS_CREDENTIALS=true

    if [ "$HAS_CONFIG" = true ] || [ "$HAS_CREDENTIALS" = true ]; then
        echo ""
        echo -e "  ${YELLOW}[!]${NC} Found user data in ~/.horus:"
        [ "$HAS_CONFIG" = true ] && echo "      - config.toml (settings)"
        [ "$HAS_CREDENTIALS" = true ] && echo "      - credentials (authentication)"
        echo ""
        read -p "$(echo -e "  ${YELLOW}?${NC}") Remove user configuration and credentials? [y/N]: " -n 1 -r
        echo

        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$HORUS_DIR"
            echo -e "  ${GREEN}[+]${NC} Removed entire ~/.horus directory"
            REMOVED=$((REMOVED + 1))
        else
            # Remove everything except config files
            # Note: pre-compiled deps are inside cache/horus@version/target/, so removing cache removes them too
            [ -d "$CACHE_DIR" ] && rm -rf "$CACHE_DIR" && echo -e "  ${GREEN}[+]${NC} Removed cache/ (includes pre-compiled deps)"
            [ -f "$HORUS_DIR/installed_version" ] && rm -f "$HORUS_DIR/installed_version"
            [ -f "$HORUS_DIR/install_profile" ] && rm -f "$HORUS_DIR/install_profile"
            echo -e "  ${CYAN}[i]${NC} Kept user configuration files"
            REMOVED=$((REMOVED + 1))
            SKIPPED=$((SKIPPED + 1))
        fi
    else
        rm -rf "$HORUS_DIR"
        echo -e "  ${GREEN}[+]${NC} Removed ~/.horus directory"
        REMOVED=$((REMOVED + 1))
    fi
else
    echo -e "  ${YELLOW}[-]${NC} ~/.horus not found (already removed?)"
fi

#=====================================
# 5. Platform-specific cleanup
#=====================================
update_uninstall_progress "Platform cleanup"
echo ""

# macOS: clean Application Support and Caches
if [ "$PLATFORM" = "macos" ]; then
    if [ -d "$HORUS_APP_SUPPORT" ]; then
        rm -rf "$HORUS_APP_SUPPORT"
        echo -e "  ${GREEN}[+]${NC} Removed macOS Application Support data"
        REMOVED=$((REMOVED + 1))
    fi
    if [ -d "$HORUS_CACHES" ]; then
        rm -rf "$HORUS_CACHES"
        echo -e "  ${GREEN}[+]${NC} Removed macOS Caches"
        REMOVED=$((REMOVED + 1))
    fi
fi

# Windows: clean AppData directories
if [ "$PLATFORM" = "windows" ]; then
    if [ -d "$HORUS_APPDATA" ]; then
        rm -rf "$HORUS_APPDATA"
        echo -e "  ${GREEN}[+]${NC} Removed Windows AppData/Roaming/horus"
        REMOVED=$((REMOVED + 1))
    fi
    if [ -d "$HORUS_LOCALAPPDATA" ]; then
        rm -rf "$HORUS_LOCALAPPDATA"
        echo -e "  ${GREEN}[+]${NC} Removed Windows AppData/Local/horus"
        REMOVED=$((REMOVED + 1))
    fi
fi

# Python: uninstall horus-robotics wheel
if command -v pip3 &> /dev/null || command -v pip &> /dev/null; then
    PIP_CMD="pip3"
    command -v pip3 &> /dev/null || PIP_CMD="pip"
    if $PIP_CMD show horus-robotics &> /dev/null 2>&1; then
        echo ""
        read -p "$(echo -e "  ${YELLOW}?${NC}") Uninstall Python package horus-robotics? [Y/n]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            $PIP_CMD uninstall -y horus-robotics 2>/dev/null || true
            echo -e "  ${GREEN}[+]${NC} Uninstalled Python horus-robotics"
            REMOVED=$((REMOVED + 1))
        fi
    fi
fi

# Linux: remove RT scheduling configuration
if [ "$PLATFORM" = "linux" ] || [ "$PLATFORM" = "wsl" ]; then
    RT_CLEANED=false
    if [ -f "/etc/security/limits.d/99-horus-realtime.conf" ]; then
        echo ""
        read -p "$(echo -e "  ${YELLOW}?${NC}") Remove RT scheduling config (requires sudo)? [Y/n]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            sudo rm -f /etc/security/limits.d/99-horus-realtime.conf 2>/dev/null && RT_CLEANED=true
            sudo rm -f /etc/sysctl.d/99-horus-realtime.conf 2>/dev/null
            if [ -f "/etc/systemd/system/horus-performance-governor.service" ]; then
                sudo systemctl disable horus-performance-governor.service 2>/dev/null || true
                sudo rm -f /etc/systemd/system/horus-performance-governor.service 2>/dev/null
            fi
            [ "$RT_CLEANED" = true ] && echo -e "  ${GREEN}[+]${NC} Removed RT scheduling configuration"
            REMOVED=$((REMOVED + 1))
        fi
    fi
fi

# Shell profiles: remove horus completion eval lines
for profile in "$HOME/.bashrc" "$HOME/.zshrc" "$HOME/.profile" "$HOME/.bash_profile"; do
    if [ -f "$profile" ] && grep -q "horus completion" "$profile" 2>/dev/null; then
        # Create backup before modifying
        cp "$profile" "${profile}.horus-backup" 2>/dev/null
        # Remove lines containing horus completion
        sed -i.bak '/horus completion/d' "$profile" 2>/dev/null || \
            sed -i '' '/horus completion/d' "$profile" 2>/dev/null  # macOS sed
        rm -f "${profile}.bak" 2>/dev/null
        echo -e "  ${GREEN}[+]${NC} Cleaned horus completion from $(basename $profile)"
        REMOVED=$((REMOVED + 1))
    fi
done

#=====================================
# 6. Optional: Clean Cargo cache
#=====================================
update_uninstall_progress "Optional cleanup"
echo ""

CARGO_HOME="${CARGO_HOME:-$HOME/.cargo}"
CARGO_REGISTRY="$CARGO_HOME/registry"

if [ -d "$CARGO_REGISTRY" ]; then
    registry_size=$(du -sh "$CARGO_REGISTRY" 2>/dev/null | cut -f1)
    echo ""
    echo -e "  ${CYAN}[i]${NC} Cargo registry cache: $registry_size"
    echo -e "      This contains downloaded crates for all Rust projects."
    echo ""
    read -p "$(echo -e "  ${YELLOW}?${NC}") Clean Cargo registry cache? (affects all Rust projects) [y/N]: " -n 1 -r
    echo

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$CARGO_REGISTRY"
        echo -e "  ${GREEN}[+]${NC} Cleaned Cargo registry cache"
        REMOVED=$((REMOVED + 1))
    else
        echo -e "  ${CYAN}[i]${NC} Kept Cargo registry cache"
        SKIPPED=$((SKIPPED + 1))
    fi
fi

#=====================================
# Summary
#=====================================

# Show final progress bar completion
complete_uninstall_progress

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${WHITE}   Uninstallation Complete${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "  ${GREEN}Removed:${NC} $REMOVED component(s)"
echo -e "  ${YELLOW}Skipped:${NC} $SKIPPED component(s)"
echo ""

# Check if any horus processes are still running
if pgrep -x "horus" > /dev/null 2>&1; then
    echo -e "${YELLOW}[!] Warning:${NC} Some HORUS processes may still be running."
    echo -e "    Run: ${CYAN}pkill -f 'horus'${NC} to terminate them."
    echo ""
fi

echo -e "${GREEN}${STATUS_OK} HORUS has been uninstalled. Goodbye!${NC}"
echo ""
echo -e "${CYAN}Notes:${NC}"
echo -e "  - Project-local .horus/ directories were NOT removed"
echo -e "  - System packages (libssl-dev, etc.) were NOT removed (may be used by other projects)"
echo -e "  - To reinstall: ${CYAN}./install.sh${NC}"
echo ""
