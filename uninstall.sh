#!/bin/bash
# HORUS Uninstallation Script v2.7.0
# Complete removal of HORUS CLI, libraries, binaries, cache, and artifacts
# Cross-platform: Linux, macOS, Windows (Git Bash/MSYS2)
# Matches install.sh v2.6.0
#
# Local:
#   ./uninstall.sh              interactive
#   ./uninstall.sh --dry-run    list what would be removed, remove nothing
#   ./uninstall.sh --yes        unattended
#
# One-line uninstall:
#   curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | bash -s -- --yes
#
# An install that set HORUS_PREFIX has to be uninstalled with the same value —
# install.sh:228-241 puts bin/, cache/ and both state files under that prefix
# instead of ~/.horus, and records the prefix nowhere outside it:
#   HORUS_PREFIX=/opt/horus ./uninstall.sh --yes
#
# The piped form has worked since the empty-BASH_SOURCE guard below was added
# and was documented nowhere, so nobody could use it. It needs --yes: piped
# into bash, stdin is the script text rather than a keyboard, so a prompt would
# read the script's own next line as the answer.
#
# --yes is itself the answer to "are you sure"; every later prompt then takes
# the default printed in its [brackets]. So an unattended run removes HORUS but
# KEEPS ~/.horus/config.toml, ~/.horus/credentials and the Cargo registry
# cache: an unattended run must not delete more than a supervised one would by
# default, and that registry belongs to every Rust project on the machine.

set -e  # Exit on error

# --- Arguments ---
# Parsed before everything else, including the deps.sh source below, so that
# --help and --dry-run keep working on a machine where the rest of this script
# would not.
DRY_RUN=false
ASSUME_YES=false

usage() {
    # Plain text, no colour variables: those come from deps.sh, which has not
    # been sourced yet at the point --help has to answer.
    cat <<'USAGE'
HORUS Uninstallation Script

Usage: uninstall.sh [options]
   or: curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | bash -s -- --yes

Options:
  -n, --dry-run   List everything that would be removed, remove nothing, exit 0.
  -y, --yes       Do not prompt. Removes HORUS; keeps ~/.horus/config.toml,
                  ~/.horus/credentials and the Cargo registry cache.
  -h, --help      Show this message.

Environment:
  HORUS_PREFIX    The install root that was passed to install.sh. Required to
                  uninstall such an install: nothing outside that prefix
                  records where it went.
USAGE
}

while [ $# -gt 0 ]; do
    case "$1" in
        -n|--dry-run) DRY_RUN=true ;;
        -y|--yes|--assume-yes) ASSUME_YES=true ;;
        -h|--help) usage; exit 0 ;;
        *)
            # Refuse rather than guess. A mistyped flag on a script whose job is
            # `rm -rf` must not fall through to the interactive path as if it
            # had never been passed.
            echo "uninstall.sh: unknown option '$1'" >&2
            echo "" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

# Get script directory
# When this script is piped to bash (curl ... | bash) BASH_SOURCE is empty, so
# `dirname ""` yields "." and SCRIPT_DIR collapses to the CALLER'S CWD — after
# which the `source "$SCRIPT_DIR/scripts/deps.sh"` below would execute a
# scripts/deps.sh from whatever directory the user happened to be in. Only trust
# BASH_SOURCE when it is actually set.
if [ -n "${BASH_SOURCE[0]:-}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    SCRIPT_DIR=""
fi

# Source shared functions from deps.sh (provides colors, status indicators, OS detection, spinner, shm paths)
if [ -n "$SCRIPT_DIR" ] && [ -f "$SCRIPT_DIR/scripts/deps.sh" ]; then
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
# PROMPTS
# ============================================================================

# Ask a yes/no question. $1 is the question, $2 the default ("y" or "n") — both
# the answer shown capitalised in the hint and the answer --yes takes.
#
# Three things the open-coded `read -p` calls this replaces got wrong:
#   * There was no way to answer without a keyboard, so this script could never
#     run in CI. That is how uninstall.sh:264 shipped a bash-4-only `mapfile`
#     that made every macOS run exit 127 before printing a single line: the
#     only automated check that could reach it was `bash -n`, which parses
#     mapfile happily.
#   * Piped from curl, bash reads this script from stdin, so a bare `read`
#     swallows the script's next line instead of the user's answer. Ask
#     /dev/tty whenever stdin is not a terminal.
#   * With neither a terminal nor --yes there is no safe guess. Refuse: quietly
#     taking a default here deletes files nobody approved.
confirm() {
    local question="$1"
    local default="$2"
    local hint="[y/N]"
    local reply=""
    [ "$default" = "y" ] && hint="[Y/n]"

    if [ "$ASSUME_YES" = true ]; then
        printf '  %b?%b %s %s --yes\n' "$YELLOW" "$NC" "$question" "$hint"
        [ "$default" = "y" ]
        return
    fi

    if [ -t 0 ]; then
        read -p "$(printf '  %b?%b %s %s: ' "$YELLOW" "$NC" "$question" "$hint")" -n 1 -r reply
        echo
    # Probe by opening, not with `[ -r /dev/tty ]`. Under setsid, cron, a
    # systemd unit or a container the device node is there and readable while
    # the open fails with ENXIO, so the test passes, the `read` fails, $reply
    # stays empty and the default is taken silently — which for the [Y/n]
    # prompts means answering YES on behalf of a user who is not there. `:`
    # opens the redirect and reads nothing; 2>/dev/null comes first because
    # redirections are applied left to right and the failing open is what has
    # to be silenced.
    elif : 2>/dev/null < /dev/tty; then
        read -p "$(printf '  %b?%b %s %s: ' "$YELLOW" "$NC" "$question" "$hint")" -n 1 -r reply < /dev/tty
        echo
    else
        echo ""
        echo -e "  ${RED}[-]${NC} No terminal available to answer: $question"
        echo -e "      Re-run with --yes, or with --dry-run to see what would be removed."
        exit 1
    fi

    [ -z "$reply" ] && reply="$default"
    [[ $reply =~ ^[Yy]$ ]]
}

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
# install.sh's find_install_dir() prefers ~/.cargo/bin but falls back to
# ~/.local/bin when Rust isn't present — which is exactly the pre-built-binary
# user. Check both, or uninstall silently leaves the binary behind.
INSTALL_DIRS=("$HOME/.cargo/bin" "$HOME/.local/bin")
HORUS_DIR="$HOME/.horus"
CACHE_DIR="$HORUS_DIR/cache"
TARGET_DIR="$HORUS_DIR/target"
MANIFEST_FILE="$HORUS_DIR/install_manifest.toml"
SHM_PARENT="$(get_shm_parent_dir)"
SHM_GLOB="$(get_shm_glob)"

# Always returns 0 — a bare `[ ... ] && return 0` loop would leave the function
# exiting non-zero on the "not found" path, and `set -e` would kill the script
# at the call site.
add_install_dir() {
    local known
    for known in "${INSTALL_DIRS[@]}"; do
        if [ "$known" = "$1" ]; then
            return 0
        fi
    done
    INSTALL_DIRS=("$1" "${INSTALL_DIRS[@]}")
}

# HORUS_PREFIX relocates the install root, and install.sh also consults it when
# running as root, so neither default above need hold the binary. Same failure
# as the ~/.local/bin case above: an install location this list does not know
# about is a binary that survives the uninstall.
[ -n "${HORUS_PREFIX:-}" ] && add_install_dir "${HORUS_PREFIX}/bin"

# install.sh and `horus self update` record the binary they actually installed
# in install_manifest.toml. Prefer that over guessing. Only its *directory* is
# used, and only horus/horus.exe inside that directory is ever removed — the
# recorded path is never deleted verbatim, because this file is writable by
# anyone who can write $HOME while this script is sometimes run under sudo.
# Absence is normal and stays non-fatal: installs predating the manifest, and
# the "keep my config" branch of a previous uninstall, leave none.
# Under HORUS_PREFIX install.sh writes the manifest there rather than in
# ~/.horus, so read whichever one exists. Only the READ location moves:
# MANIFEST_FILE stays the ~/.horus copy that the step-4 cleanup removes, and the
# prefix copy is removed with the rest of the prefix further down.
_manifest_read="$MANIFEST_FILE"
[ -n "${HORUS_PREFIX:-}" ] && [ -f "${HORUS_PREFIX}/install_manifest.toml" ] && _manifest_read="${HORUS_PREFIX}/install_manifest.toml"

if [ -f "$_manifest_read" ]; then
    _manifest_bin=$(sed -n 's/^[[:space:]]*binary[[:space:]]*=[[:space:]]*"\(.*\)".*$/\1/p' "$_manifest_read" | head -n 1)
    [ -n "$_manifest_bin" ] && add_install_dir "$(dirname "$_manifest_bin")"
    unset _manifest_bin
fi
unset _manifest_read

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

# >>> uninstall.sh: artifact paths >>>
# Shell completion paths.
#
# The first entry of each list is where install.sh's install_completions()
# actually writes today; the rest are older/system locations kept so an upgrade
# from an earlier layout still cleans up. These two lists drifted apart once
# already: install.sh moved to ~/.zfunc and XDG bash-completion, and this file
# was never updated, so two of the three files the installer writes survived a
# full uninstall.
BASH_COMPLETION_PATHS=(
    "${XDG_DATA_HOME:-$HOME/.local/share}/bash-completion/completions/horus"
    "$HOME/.local/share/bash-completion/completions/horus"
    "$HOME/.bash_completion.d/horus"
    "/etc/bash_completion.d/horus"
    "/usr/share/bash-completion/completions/horus"
    "$HORUS_DIR/completions/horus.bash"
)
ZSH_COMPLETION_PATHS=(
    "$HOME/.zfunc/_horus"
    "$HOME/.zsh/completions/_horus"
    "/usr/share/zsh/site-functions/_horus"
    "$HORUS_DIR/completions/_horus"
    "$HORUS_DIR/completions/horus.zsh"
)
FISH_COMPLETION_PATHS=(
    "${XDG_CONFIG_HOME:-$HOME/.config}/fish/completions/horus.fish"
    "$HOME/.config/fish/completions/horus.fish"
    "/usr/share/fish/completions/horus.fish"
    "$HORUS_DIR/completions/horus.fish"
)

# Man page paths — install.sh's install_man_page() writes the first as a normal
# user and the third when run as root. None of them was ever removed here.
MAN_PAGE_PATHS=(
    "${XDG_DATA_HOME:-$HOME/.local/share}/man/man1/horus.1"
    "$HOME/.local/share/man/man1/horus.1"
    "/usr/local/share/man/man1/horus.1"
    "/usr/share/man/man1/horus.1"
)

# With XDG_DATA_HOME/XDG_CONFIG_HOME unset the XDG entry and the plain $HOME one
# above expand to the same string; collapse them so nothing is listed or counted
# twice.
for _list in BASH_COMPLETION_PATHS ZSH_COMPLETION_PATHS FISH_COMPLETION_PATHS MAN_PAGE_PATHS; do
    eval "_tmp=(\"\${${_list}[@]}\")"
    # Read the lines in a loop rather than with `mapfile -t`. mapfile is bash 4+
    # and macOS still ships /bin/bash 3.2.57, so on every Mac this line was
    # `mapfile: command not found` — and because this block is unconditional
    # top-level code under `set -e`, running before the banner and long before
    # the confirmation prompt, the whole uninstaller exited 127 having removed
    # nothing and having printed nothing. Process substitution keeps the loop in
    # this shell, so _uniq survives it.
    _uniq=()
    while IFS= read -r _line; do
        _uniq+=("$_line")
    done < <(printf '%s\n' "${_tmp[@]}" | awk '!seen[$0]++')
    eval "${_list}=(\"\${_uniq[@]}\")"
done
unset _list _tmp _uniq _line
# <<< uninstall.sh: artifact paths <<<

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
    for install_dir in "${INSTALL_DIRS[@]}"; do
        for bin in "${BINARIES[@]}"; do
            if [ -f "$install_dir/$bin" ]; then
                size=$(du -k "$install_dir/$bin" 2>/dev/null | cut -f1)
                total_size=$((total_size + size))
            fi
        done
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
for install_dir in "${INSTALL_DIRS[@]}"; do
    for bin in "${BINARIES[@]}"; do
        if [ -f "$install_dir/$bin" ]; then
            size=$(du -h "$install_dir/$bin" 2>/dev/null | cut -f1)
            echo -e "    [x] $install_dir/$bin ($size)"
            BINARY_COUNT=$((BINARY_COUNT + 1))
        fi
    done
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
    if [ -f "$HORUS_DIR/installed_version" ]; then
        echo -e "        - installed_version - version gate state"
    fi
    if [ -f "$MANIFEST_FILE" ]; then
        echo -e "        - install_manifest.toml - install record (version, tag, commit)"
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
        # printf %s, not `echo -e "$ns_dir"` — the same rule the removal loop in
        # step 3 already follows, and this listing needs it more. /dev/shm is
        # world-writable, so any local user can create a horus_* directory whose
        # NAME contains escape sequences; `echo -e` interprets them, and this is
        # the inventory the operator reads immediately before answering the
        # "Are you sure you want to uninstall HORUS?" prompt below. Interpreted
        # escapes let that user clear lines, move the cursor and repaint the
        # list, so what the operator confirms is not what is about to be
        # deleted — from a run that may be under sudo.
        printf '    [x] %s/ (%s)\n' "$ns_dir" "$ns_size"
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

# Man page
echo ""
echo -e "  ${CYAN}Man Page:${NC}"
MAN_COUNT=0
for path in "${MAN_PAGE_PATHS[@]}"; do
    if [ -f "$path" ]; then
        echo -e "    [x] $path"
        MAN_COUNT=$((MAN_COUNT + 1))
    fi
done
[ $MAN_COUNT -eq 0 ] && echo -e "    ${YELLOW}(no man page found)${NC}"

# Steps 5 and 6 below remove things this listing never mentioned, so neither
# --dry-run nor the operator staring at the confirmation prompt could see them
# coming. Same drift that let the completion paths rot: what the script removes
# and what it says it removes have to be written next to each other.
echo ""
echo -e "  ${CYAN}Shell Integration & Platform Data:${NC}"
EXTRA_COUNT=0
note_extra() {
    echo -e "    [x] $1"
    EXTRA_COUNT=$((EXTRA_COUNT + 1))
}

case "$PLATFORM" in
    macos)
        if [ -d "$HORUS_APP_SUPPORT" ]; then note_extra "$HORUS_APP_SUPPORT/"; fi
        if [ -d "$HORUS_CACHES" ]; then note_extra "$HORUS_CACHES/"; fi
        ;;
    windows)
        if [ -d "$HORUS_APPDATA" ]; then note_extra "$HORUS_APPDATA/"; fi
        if [ -d "$HORUS_LOCALAPPDATA" ]; then note_extra "$HORUS_LOCALAPPDATA/"; fi
        ;;
    *)
        # auth.json (registry credentials) and workspaces.json live here.
        if [ -d "${XDG_CONFIG_HOME:-$HOME/.config}/horus" ]; then
            note_extra "${XDG_CONFIG_HOME:-$HOME/.config}/horus/ (credentials + workspace registry)"
        fi
        ;;
esac

for _extra in "$HORUS_DIR/env.sh" "$HORUS_DIR/env.fish" "$HOME/.config/fish/conf.d/horus.fish"; do
    if [ -f "$_extra" ]; then note_extra "$_extra"; fi
done

for _extra in "$HOME/.bashrc" "$HOME/.zshrc" "$HOME/.profile" "$HOME/.bash_profile"; do
    if [ -f "$_extra" ] && grep -qE '# >>> horus completions >>>|horus completion|\.horus/env\.sh' "$_extra" 2>/dev/null; then
        note_extra "$_extra (horus lines only; a .horus-backup copy is kept)"
    fi
done

if [ -n "${HORUS_PREFIX:-}" ] && [ "$HORUS_PREFIX" != "$HORUS_DIR" ] && [ -d "$HORUS_PREFIX" ]; then
    note_extra "$HORUS_PREFIX/ - the HORUS_PREFIX install root: bin/horus, cache/, target/, completions/, env.sh, env.fish and the state files. Anything else in it is left alone."
fi

if [ "$PLATFORM" = "linux" ] || [ "$PLATFORM" = "wsl" ]; then
    if [ -f "/etc/security/limits.d/99-horus-realtime.conf" ]; then
        note_extra "/etc/security/limits.d/99-horus-realtime.conf and the rest of the RT config (asks first, needs sudo)"
    fi
fi

if command -v pip3 &> /dev/null || command -v pip &> /dev/null; then
    _extra_pip="pip3"
    command -v pip3 &> /dev/null || _extra_pip="pip"
    if $_extra_pip show horus-robotics &> /dev/null 2>&1; then
        note_extra "Python package horus-robotics (via $_extra_pip, asks first)"
    fi
    unset _extra_pip
fi
unset _extra

[ $EXTRA_COUNT -eq 0 ] && echo -e "    ${YELLOW}(nothing found)${NC}"

# Offered, not removed by default — and --yes keeps it, so it is listed as a
# note rather than as an [x].
if [ -d "${CARGO_HOME:-$HOME/.cargo}/registry" ]; then
    echo -e "    ${CYAN}[i]${NC} Cargo registry cache is offered at the end; kept unless you answer yes"
fi

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

# --dry-run stops here. Everything above this line only reads, so this is
# exactly the non-destructive half of the script — and it is the path CI runs.
# It is the only check that would have caught uninstall.sh:264 exiting 127 on
# macOS before the banner ever printed, because the prompt below is what kept
# this script out of CI in the first place.
if [ "$DRY_RUN" = true ]; then
    echo -e "${CYAN}${STATUS_INFO} Dry run: nothing was removed.${NC}"
    echo -e "  Re-run without --dry-run (add --yes to skip the prompts) to uninstall."
    echo ""
    exit 0
fi

# Ask for confirmation. --yes IS the answer to this one, which is why it is not
# routed through confirm(): confirm() answers with the bracketed default, and
# the default here is "no".
if [ "$ASSUME_YES" != true ]; then
    if ! confirm "Are you sure you want to uninstall HORUS?" n; then
        echo ""
        echo -e "${GREEN}Uninstallation cancelled.${NC}"
        exit 0
    fi
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

for install_dir in "${INSTALL_DIRS[@]}"; do
    for bin in "${BINARIES[@]}"; do
        # Try both with and without .exe extension (Windows compat)
        for ext in "" ".exe"; do
            if [ -f "$install_dir/${bin}${ext}" ]; then
                rm -f "$install_dir/${bin}${ext}"
                echo -e "  ${GREEN}[+]${NC} Removed ${bin}${ext} from ${install_dir}"
                REMOVED=$((REMOVED + 1))
            fi
        done
    done
done

#=====================================
# 2. Remove shell completions
#=====================================
update_uninstall_progress "Removing completions"
echo ""

# >>> uninstall.sh: completion + man removal >>>
# Kept in a function with these markers so the regression test can run this
# exact code against a throw-away $HOME without also running the shared-memory,
# sudo and pip steps further down. install.sh and this list have already drifted
# apart once; the test now installs with install.sh and uninstalls with this.
remove_completions_and_man_page() {
    local path

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
    # ~/.zfunc is created by install.sh purely to hold _horus. Only remove it if
    # nothing else moved in.
    if [ -d "$HOME/.zfunc" ]; then
        rmdir "$HOME/.zfunc" 2>/dev/null || true
    fi

    # The man page install.sh writes with install_man_page(). This step used to
    # remove completions only, so `man horus` still worked after an uninstall.
    for path in "${MAN_PAGE_PATHS[@]}"; do
        if [ -f "$path" ]; then
            if rm -f "$path" 2>/dev/null; then
                echo -e "  ${GREEN}[+]${NC} Removed man page $path"
                REMOVED=$((REMOVED + 1))
            else
                echo -e "  ${YELLOW}[!]${NC} Could not remove $path (try sudo)"
                SKIPPED=$((SKIPPED + 1))
            fi
        fi
    done
}
# <<< uninstall.sh: completion + man removal <<<
remove_completions_and_man_page

#=====================================
# 3. Remove shared memory
#=====================================
update_uninstall_progress "Cleaning shared memory"
echo ""

for ns_dir in $SHM_GLOB; do
    if [ -d "$ns_dir" ]; then
        ns_name=$(basename "$ns_dir")
        rm -rf "$ns_dir" 2>/dev/null || true
        # printf %s, not `echo -e "$ns_name"`. /dev/shm is world-writable, so any
        # local user can create a horus_* directory whose NAME contains escape
        # sequences; `echo -e` would interpret them and let that user drive the
        # operator's terminal (cursor moves, colour, cleared lines) from a
        # privileged uninstall run.
        printf '  %b[+]%b Removed %s\n' "$GREEN" "$NC" "$ns_name"
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
        if confirm "Remove user configuration and credentials?" n; then
            rm -rf "$HORUS_DIR"
            echo -e "  ${GREEN}[+]${NC} Removed entire ~/.horus directory"
            REMOVED=$((REMOVED + 1))
        else
            # Remove everything except config files
            # Note: pre-compiled deps are inside cache/horus@version/target/, so removing cache removes them too
            [ -d "$CACHE_DIR" ] && rm -rf "$CACHE_DIR" && echo -e "  ${GREEN}[+]${NC} Removed cache/ (includes pre-compiled deps)"
            # Both state files, always together. install.sh and `horus self
            # update` write installed_version and install_manifest.toml on every
            # successful install; leaving either behind points the version gate
            # in version.rs at a HORUS that is no longer on the machine, and the
            # manifest is the richer of the two (it carries topic_version, the
            # field that actually decides whether the CLI can read its libs).
            [ -f "$HORUS_DIR/installed_version" ] && rm -f "$HORUS_DIR/installed_version"
            [ -f "$MANIFEST_FILE" ] && rm -f "$MANIFEST_FILE"
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

# HORUS_PREFIX install roots.
# install.sh:228-241 puts everything under $HORUS_PREFIX when it is set — bin/,
# cache/ and both state files — instead of under ~/.horus, so the block above
# cleans nothing for those installs. Enumerate what the installer writes rather
# than `rm -rf "$HORUS_PREFIX"`: the prefix is a directory the operator named
# (/opt/horus, but nothing stops /usr/local), it can predate HORUS, and a
# mistyped value must not take the rest of it along. config.toml and credentials
# are left for the same reason they are in ~/.horus. The rmdirs fail harmlessly
# when anything else still lives there.
if [ -n "${HORUS_PREFIX:-}" ] && [ "$HORUS_PREFIX" != "$HORUS_DIR" ] && [ -d "$HORUS_PREFIX" ]; then
    for _artifact in \
        "$HORUS_PREFIX/cache" \
        "$HORUS_PREFIX/target" \
        "$HORUS_PREFIX/completions" \
        "$HORUS_PREFIX/installed_version" \
        "$HORUS_PREFIX/install_manifest.toml" \
        "$HORUS_PREFIX/install_profile" \
        "$HORUS_PREFIX/env.sh" \
        "$HORUS_PREFIX/env.fish"
    do
        if [ -e "$_artifact" ]; then
            rm -rf "$_artifact"
            echo -e "  ${GREEN}[+]${NC} Removed $_artifact"
            REMOVED=$((REMOVED + 1))
        fi
    done
    unset _artifact
    rmdir "$HORUS_PREFIX/bin" 2>/dev/null || true
    rmdir "$HORUS_PREFIX" 2>/dev/null || true
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

# Linux/BSD: clean the XDG config dir.
# horus_sys::platform::config_dir() resolves to $XDG_CONFIG_HOME/horus (default
# ~/.config/horus) on non-macOS/Windows. It holds auth.json (registry
# credentials) and workspaces.json — leaving it behind leaks credentials.
if [ "$PLATFORM" != "macos" ] && [ "$PLATFORM" != "windows" ]; then
    HORUS_XDG_CONFIG="${XDG_CONFIG_HOME:-$HOME/.config}/horus"
    if [ -d "$HORUS_XDG_CONFIG" ]; then
        rm -rf "$HORUS_XDG_CONFIG"
        echo -e "  ${GREEN}[+]${NC} Removed ${HORUS_XDG_CONFIG} (credentials + workspace registry)"
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
        if confirm "Uninstall Python package horus-robotics?" y; then
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
        if confirm "Remove RT scheduling config (requires sudo)?" y; then
            # `sudo -n` under --yes: an unattended run has no terminal for a
            # password prompt, so plain sudo would either block on one or sit
            # through its timeout before failing.
            SUDO="sudo"
            [ "$ASSUME_YES" = true ] && SUDO="sudo -n"
            # `if sudo ...; then` rather than `sudo ... && RT_CLEANED=true`: the
            # latter ends an && list with a failing command, which under `set -e`
            # aborts the entire uninstall here — after the binary and ~/.horus
            # are already gone — whenever sudo is declined or unavailable.
            if $SUDO rm -f /etc/security/limits.d/99-horus-realtime.conf 2>/dev/null; then
                RT_CLEANED=true
            fi
            $SUDO rm -f /etc/sysctl.d/99-horus-realtime.conf 2>/dev/null || true
            if [ -f "/etc/systemd/system/horus-performance-governor.service" ]; then
                $SUDO systemctl disable horus-performance-governor.service 2>/dev/null || true
                $SUDO rm -f /etc/systemd/system/horus-performance-governor.service 2>/dev/null || true
            fi
            if [ "$RT_CLEANED" = true ]; then
                echo -e "  ${GREEN}[+]${NC} Removed RT scheduling configuration"
                REMOVED=$((REMOVED + 1))
            else
                # Was counted as Removed unconditionally before, so a declined
                # sudo reported a cleanup that never happened.
                echo -e "  ${YELLOW}[!]${NC} Could not remove RT scheduling config (sudo declined or unavailable)"
                SKIPPED=$((SKIPPED + 1))
            fi
        fi
    fi
fi

# >>> uninstall.sh: shell profile cleanup >>>
# In a function, with markers, for the same reason as the completion
# removal above: the regression test runs this exact text.
clean_shell_profiles() {
    local profile
    # Shell profiles: remove horus completion eval lines and shell integration
    for profile in "$HOME/.bashrc" "$HOME/.zshrc" "$HOME/.profile" "$HOME/.bash_profile"; do
        # The block install.sh's add_zsh_fpath_block() writes. It is delimited by
        # markers precisely so this can delete exactly what was added; matching on
        # the string "horus completion" (below) never caught it, because the line it
        # writes is `fpath=(~/.zfunc $fpath)` and contains no "horus" at all.
        if [ -f "$profile" ] && grep -qF "# >>> horus completions >>>" "$profile" 2>/dev/null; then
            cp "$profile" "${profile}.horus-backup" 2>/dev/null
            sed -i.bak '/^# >>> horus completions >>>$/,/^# <<< horus completions <<<$/d' "$profile" 2>/dev/null || \
                sed -i '' '/^# >>> horus completions >>>$/,/^# <<< horus completions <<<$/d' "$profile" 2>/dev/null
            rm -f "${profile}.bak" 2>/dev/null
            echo -e "  ${GREEN}[+]${NC} Cleaned horus completion fpath block from $(basename $profile)"
            REMOVED=$((REMOVED + 1))
        fi
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
        # Remove shell integration (cargo/pip/cmake proxy)
        if [ -f "$profile" ] && grep -q ".horus/env.sh" "$profile" 2>/dev/null; then
            cp "$profile" "${profile}.horus-backup" 2>/dev/null
            sed -i.bak '/\.horus\/env\.sh/d' "$profile" 2>/dev/null || \
                sed -i '' '/\.horus\/env\.sh/d' "$profile" 2>/dev/null
            sed -i.bak '/# Horus shell integration/d' "$profile" 2>/dev/null || \
                sed -i '' '/# Horus shell integration/d' "$profile" 2>/dev/null
            rm -f "${profile}.bak" 2>/dev/null
            echo -e "  ${GREEN}[+]${NC} Cleaned horus shell integration from $(basename $profile)"
            REMOVED=$((REMOVED + 1))
        fi
    done

    # Remove fish shell integration
    if [ -f "$HOME/.config/fish/conf.d/horus.fish" ]; then
        rm -f "$HOME/.config/fish/conf.d/horus.fish"
        echo -e "  ${GREEN}[+]${NC} Removed fish shell integration"
        REMOVED=$((REMOVED + 1))
    fi
}
# <<< uninstall.sh: shell profile cleanup <<<
clean_shell_profiles

# Remove env.sh/env.fish files
for envfile in "$HORUS_DIR/env.sh" "$HORUS_DIR/env.fish"; do
    if [ -f "$envfile" ]; then
        rm -f "$envfile"
        echo -e "  ${GREEN}[+]${NC} Removed $(basename $envfile)"
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
    if confirm "Clean Cargo registry cache? (affects all Rust projects)" n; then
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
echo -e "  - To reinstall: ${CYAN}./install.sh${NC}, or"
echo -e "    ${CYAN}curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash${NC}"
echo ""
