#!/bin/bash
# HORUS Uninstallation Script v2.6.0
# Complete removal of HORUS CLI, libraries, binaries, cache, and artifacts
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
    # Fallback shared memory paths
    get_shm_base_dir() {
        case "$(uname -s)" in
            Linux*) echo "/dev/shm/horus" ;;
            Darwin*|FreeBSD*|OpenBSD*|NetBSD*) echo "/tmp/horus" ;;
            MINGW*|MSYS*|CYGWIN*) echo "${TEMP:-/tmp}/horus" ;;
            *) echo "/tmp/horus" ;;
        esac
    }
    get_shm_logs_path() {
        case "$(uname -s)" in
            Linux*) echo "/dev/shm/horus_logs" ;;
            Darwin*|FreeBSD*|OpenBSD*|NetBSD*) echo "/tmp/horus_logs" ;;
            MINGW*|MSYS*|CYGWIN*) echo "${TEMP:-/tmp}/horus_logs" ;;
            *) echo "/tmp/horus_logs" ;;
        esac
    }
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

# Determine installation paths
INSTALL_DIR="$HOME/.cargo/bin"
HORUS_DIR="$HOME/.horus"
CACHE_DIR="$HORUS_DIR/cache"
TARGET_DIR="$HORUS_DIR/target"
SHM_DIR="$(get_shm_base_dir)"
SHM_LOGS="$(get_shm_logs_path)"

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
echo -e "${WHITE}   HORUS Uninstallation Script v2.6.0${NC}"
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

    # Shared memory
    if [ -d "$SHM_DIR" ]; then
        size=$(du -sk "$SHM_DIR" 2>/dev/null | cut -f1)
        total_size=$((total_size + size))
    fi

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
if [ -d "$SHM_DIR" ]; then
    shm_size=$(du -sh "$SHM_DIR" 2>/dev/null | cut -f1)
    session_count=$(ls -d "$SHM_DIR"/* 2>/dev/null | wc -l)
    echo -e "    [x] $SHM_DIR/ ($shm_size) - $session_count session(s)"
else
    echo -e "    ${YELLOW}(no shared memory data)${NC}"
fi
if [ -f "$SHM_LOGS" ]; then
    logs_size=$(du -h "$SHM_LOGS" 2>/dev/null | cut -f1)
    echo -e "    [x] $SHM_LOGS ($logs_size)"
fi

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

# Initialize progress tracking (5 main steps)
init_uninstall_progress 5

#=====================================
# 1. Remove binaries
#=====================================
update_uninstall_progress "Removing binaries"
echo ""

for bin in "${BINARIES[@]}"; do
    if [ -f "$INSTALL_DIR/$bin" ]; then
        rm -f "$INSTALL_DIR/$bin"
        echo -e "  ${GREEN}[+]${NC} Removed $bin"
        REMOVED=$((REMOVED + 1))
    fi
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

if [ -d "$SHM_DIR" ]; then
    # Count active sessions
    if [ -d "$SHM_DIR" ]; then
        active=$(ls -d "$SHM_DIR"/* 2>/dev/null | wc -l)
        if [ "$active" -gt 0 ]; then
            echo -e "  ${YELLOW}[!]${NC} Warning: $active active session(s) will be terminated"
        fi
    fi

    rm -rf "$SHM_DIR" 2>/dev/null || true
    echo -e "  ${GREEN}[+]${NC} Removed $SHM_DIR/"
    REMOVED=$((REMOVED + 1))
fi

if [ -f "$SHM_LOGS" ]; then
    rm -f "$SHM_LOGS" 2>/dev/null || true
    echo -e "  ${GREEN}[+]${NC} Removed log buffer"
    REMOVED=$((REMOVED + 1))
fi

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
# 5. Optional: Clean Cargo cache
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
echo -e "  - To reinstall: ${CYAN}./install.sh${NC}"
echo ""
