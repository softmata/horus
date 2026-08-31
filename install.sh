#!/bin/bash
# HORUS Installer
#
# One-line install:
#   curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
#
# Every URL this script prints is that one. The short form
# https://horusrobotics.dev/install used to be advertised here and in nine of
# the recovery hints below, and it has never been served: horus-site has no
# route for it, so it 404s, and `curl -f` on a 404 exits 22 having printed
# nothing at all — a user following the installer's own advice after a failure
# got silence. Publish that route on horus-site before putting it back.
#
# Flow:
#   1. Detect OS and architecture, and whose install this is
#   2. Resolve ONE ref: a release tag, or the tag/branch/tree the caller pinned
#   3. Download the pre-built binary for that tag and verify it against that
#      tag's SHA256SUMS (fast)
#   4. Clone the source at the SAME ref, cache it, and build from it if there is
#      no usable binary
#   5. Install to ~/.cargo/bin/horus or ~/.local/bin/horus
#   6. Record what was installed, verify it runs, and configure PATH
#
# Steps 2-4 are one decision on purpose. The binary used to come from
# releases/latest/download/... (a tag) while the source was cloned from main
# HEAD — 93 commits apart, both trees calling themselves 0.4.0, with
# TOPIC_VERSION 3 at the tag and 4 on main. The installed CLI then could not
# read the shared memory its own libraries wrote: `horus topic list` reported 0
# messages on a live topic and `horus launch` failed with "Incompatible topic
# version: 4 (expected 3)". Nothing below may mix two refs.
#
# Environment:
#   HORUS_VERSION=v0.4.0          Install exactly this tag, binary AND source.
#                                 Accepted with or without the leading "v".
#   HORUS_BUILD_FROM_SOURCE=1     Skip the pre-built binary; compile the source
#                                 at the resolved tag.
#   HORUS_INSTALL_BRANCH=<branch> Developer escape hatch: build from source at
#                                 that branch. Implies HORUS_BUILD_FROM_SOURCE=1
#                                 — a branch tree and a tagged binary are the
#                                 skew described above.
#   HORUS_LOCAL_SOURCE=/path      Offline/air-gapped: build an existing local
#                                 tree. No clone, no source download.
#   HORUS_PREFIX=/opt/horus       Install root override, and how a root install
#                                 says where it means to go.
#   HORUS_NO_SHELL_INTEGRATION=1  Do not touch shell rc files.
#
# These are read from the environment of *this* script, so with a pipe they go
# on the right-hand side:
#   curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.4.0 bash

set -e

# --- Config ---
REPO="softmata/horus"
# The branch every raw.githubusercontent URL in this repository points at, and
# the default for the HORUS_INSTALL_BRANCH escape hatch. There is no long-lived
# "release" branch on origin — tags (v*.*.*) are cut from main.
#
# It is NOT what a default install clones any more: that is the resolved release
# tag, because cloning main while downloading a tagged binary is exactly the
# version skew described above. install_url_contract.rs parses this line to
# check the repository's URLs, so it stays in this shape.
BRANCH="${HORUS_INSTALL_BRANCH:-main}"

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# --- Helpers ---
info()  { echo -e "  ${CYAN}→${NC} $1"; }
ok()    { echo -e "  ${GREEN}✓${NC} $1"; }
warn()  { echo -e "  ${YELLOW}!${NC} $1"; }
fail()  { echo -e "  ${RED}✗${NC} $1"; }

# --- Platform detection ---
detect_os() {
    case "$(uname -s)" in
        Linux*)  echo "linux" ;;
        Darwin*) echo "macos" ;;
        MINGW*|MSYS*|CYGWIN*) echo "windows" ;;
        *) echo "unknown" ;;
    esac
}

detect_arch() {
    case "$(uname -m)" in
        x86_64|amd64)  echo "amd64" ;;
        aarch64|arm64) echo "arm64" ;;
        armv7l)        echo "armv7" ;;
        *) echo "unknown" ;;
    esac
}

detect_distro() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        echo "${ID:-unknown}"
    elif command -v sw_vers &>/dev/null; then
        echo "macos"
    else
        echo "unknown"
    fi
}

# Every path this script writes to hangs off one home directory, which is why
# this takes it as an argument instead of reading $HOME: under `sudo bash` $HOME
# is /root and the account that will actually run HORUS is $SUDO_USER's.
find_install_dir() {
    local home="$1"
    if [ -d "${home}/.cargo/bin" ]; then
        echo "${home}/.cargo/bin"
    elif [ -d "${home}/.local/bin" ]; then
        echo "${home}/.local/bin"
    else
        mkdir -p "${home}/.local/bin"
        echo "${home}/.local/bin"
    fi
}

# Home directory of an arbitrary account, without `eval echo ~$user`.
# getent is not on macOS and dscl is not on Linux, so try both and let the
# caller refuse when neither answers — guessing /home/$user is how an installer
# ends up writing to a directory that is not the user's.
resolve_user_home() {
    local user="$1"
    if command -v getent >/dev/null 2>&1; then
        getent passwd "$user" 2>/dev/null | cut -d: -f6
    elif command -v dscl >/dev/null 2>&1; then
        dscl . -read "/Users/${user}" NFSHomeDirectory 2>/dev/null | awk '{print $2}'
    fi
}

# Files created while running as root land root-owned inside the user's home,
# and the next `horus run` — which writes into the source cache to compile a
# project — then fails with EACCES on a tree the installer said it had set up.
# Hand back everything this script created.
reown_for_target_user() {
    [ -n "${TARGET_USER:-}" ] || return 0
    if [ -n "${SUDO_UID:-}" ] && [ -n "${SUDO_GID:-}" ]; then
        chown -R "${SUDO_UID}:${SUDO_GID}" "$@" 2>/dev/null || true
    else
        chown -R "$TARGET_USER" "$@" 2>/dev/null || true
    fi
    return 0
}

# --- Dependency installation ---
install_build_deps() {
    local distro
    distro=$(detect_distro)
    info "Installing build dependencies for ${distro}..."

    case "$distro" in
        ubuntu|debian|pop|linuxmint|elementary)
            sudo apt-get update -qq
            sudo apt-get install -y -qq \
                build-essential pkg-config libssl-dev libudev-dev \
                libasound2-dev libclang-dev >/dev/null 2>&1
            ;;
        fedora)
            sudo dnf install -y -q \
                gcc gcc-c++ pkg-config openssl-devel systemd-devel \
                alsa-lib-devel clang-devel >/dev/null 2>&1
            ;;
        arch|manjaro|endeavouros)
            sudo pacman -Sy --noconfirm --needed \
                base-devel pkg-config openssl alsa-lib clang >/dev/null 2>&1
            ;;
        opensuse*|sles)
            sudo zypper install -y \
                gcc gcc-c++ pkg-config libopenssl-devel libudev-devel \
                alsa-devel clang-devel >/dev/null 2>&1
            ;;
        macos)
            if command -v brew &>/dev/null; then
                brew install pkg-config openssl >/dev/null 2>&1 || true
            fi
            ;;
        *)
            warn "Unknown distro '${distro}' — you may need to install build deps manually"
            echo "    Needed: gcc, pkg-config, openssl-dev, libudev-dev, alsa-dev, clang"
            ;;
    esac
}

install_rust() {
    info "Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable 2>&1 | tail -1
    export PATH="$HOME/.cargo/bin:$PATH"
    # shellcheck disable=SC1091
    [ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
    # `return 0` because that test is the function's last command: when rustup
    # failed and never wrote ~/.cargo/env, the function returns 1, and it is
    # called bare at :655 under `set -e` — so the script died right there and
    # the user saw "Installing Rust..." and a bare exit 1, never the
    # `fail "Failed to install Rust. Install manually: https://rustup.rs"` two
    # lines below the call that exists to explain it.
    return 0
}

# --- Main ---
OS=$(detect_os)
ARCH=$(detect_arch)
INSTALL_START=$(date +%s)

# --- Whose install is this? ---
#
# `curl ... | sudo bash` is how ansible, cloud-init and every "provision the
# fleet" runbook drives a shell installer, and $HOME is /root there. Nothing in
# this script used to check: the binary, the source cache and the state file all
# landed under /root, "Installation complete!" was printed, and the account that
# actually runs HORUS had none of it. scripts/setup-realtime.sh already resolves
# the invoking user this way, including refusing when SUDO_USER is empty (which
# is the normal case under Docker, systemd and Ansible's `become`).
TARGET_USER=""
if [ "$(id -u)" = "0" ] && [ -z "${HORUS_PREFIX:-}" ]; then
    TARGET_USER="${SUDO_USER:-}"
    if [ -z "$TARGET_USER" ] || [ "$TARGET_USER" = "root" ]; then
        fail "Running as root with no SUDO_USER — there is no way to tell whose install this is."
        echo "    Everything would land in ${HOME:-/root}: binary, source cache and state."
        echo "    Run it as the account that will run HORUS:"
        echo "      curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash"
        echo "    or name the install root explicitly:"
        echo "      curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_PREFIX=/opt/horus bash"
        exit 1
    fi
    TARGET_HOME=$(resolve_user_home "$TARGET_USER")
    if [ -z "$TARGET_HOME" ] || [ ! -d "$TARGET_HOME" ]; then
        fail "Could not resolve a home directory for '${TARGET_USER}'."
        echo "    Set HORUS_PREFIX to say where HORUS should be installed instead."
        exit 1
    fi
else
    TARGET_HOME="$HOME"
fi

# HORUS_PREFIX relocates everything the installer writes: $HORUS_PREFIX/bin for
# the binary, and $HORUS_PREFIX itself in place of ~/.horus for the state files
# and the source cache. Without it the root is ~/.horus, which is what
# version.rs and uninstall.sh already read.
if [ -n "${HORUS_PREFIX:-}" ]; then
    HORUS_STATE_DIR="$HORUS_PREFIX"
    INSTALL_DIR="${HORUS_PREFIX}/bin"
    if ! mkdir -p "$INSTALL_DIR"; then
        fail "Cannot create ${INSTALL_DIR}"
        exit 1
    fi
else
    HORUS_STATE_DIR="${TARGET_HOME}/.horus"
    INSTALL_DIR=$(find_install_dir "$TARGET_HOME")
fi
# Where the source tree is kept after building. `horus run` compiles user
# projects against horus as *path* dependencies (see cargo_gen.rs ->
# find_horus_source_dir), so the source must outlive the install or no Rust
# project can ever be built.
HORUS_CACHE="${HORUS_STATE_DIR}/cache"
if [ "$OS" = "windows" ]; then
    BINARY_NAME="horus.exe"
else
    BINARY_NAME="horus"
fi

echo ""
echo -e "${CYAN}  ╦ ╦╔═╗╦═╗╦ ╦╔═╗${NC}"
echo -e "${CYAN}  ╠═╣║ ║╠╦╝║ ║╚═╗${NC}"
echo -e "${CYAN}  ╩ ╩╚═╝╩╚═╚═╝╚═╝${NC}"
echo ""
echo "  Deterministic Real-Time Robotics Framework"
echo ""
echo -e "  OS:      ${GREEN}${OS}${NC}"
echo -e "  Arch:    ${GREEN}${ARCH}${NC}"
echo -e "  Install: ${GREEN}${INSTALL_DIR}${NC}"
echo ""

if [ -n "$TARGET_USER" ]; then
    info "Running as root on behalf of ${TARGET_USER} — installing into ${TARGET_HOME}"
fi

if [ "$OS" = "unknown" ] || [ "$ARCH" = "unknown" ]; then
    fail "Unsupported platform: $(uname -s) $(uname -m)"
    # armv7 belongs in this list: detect_arch() returns it and release.yml
    # publishes horus-linux-armv7.tar.gz, so a 32-bit Pi is a first-class
    # platform rather than the source-build fallback it used to be.
    echo "    Supported: Linux (amd64, arm64, armv7), macOS (amd64, arm64), Windows (via install.ps1, Git Bash or WSL)"
    exit 1
fi

if ! command -v curl &>/dev/null; then
    fail "curl is required"
    exit 1
fi

# --- Resolve the one ref that drives this whole install ---
#
# Resolution goes through the plain redirect rather than
# api.github.com/repos/.../releases/latest: the API allows 60 unauthenticated
# requests per hour *per IP*, which one lab behind one NAT (or a CI matrix)
# exhausts in minutes, and its 403 body is JSON — not something that looks like
# a failure to a shell reading a version out of it. The redirect from
# /releases/latest to /releases/tag/<T> is not rate limited.
resolve_latest_tag() {
    local effective
    effective=$(curl -fsSLI -o /dev/null -w '%{url_effective}' \
        "https://github.com/${REPO}/releases/latest" 2>/dev/null) || return 1
    case "$effective" in
        */tag/*) printf '%s\n' "${effective##*/tag/}" ;;
        # A repository with no published release redirects to /releases instead,
        # so there is no tag to strip and nothing to install.
        *) return 1 ;;
    esac
}

# The ref becomes a URL path segment and a `git clone --branch` argument. One
# beginning with "-" is read as an option by both curl and git, and a "/" or ".."
# would walk out of the release URL. Release tags are v*.*.* (release.yml:6).
valid_ref() {
    case "$1" in
        ''|-*|*..*|*[!A-Za-z0-9._-]*) return 1 ;;
        *) return 0 ;;
    esac
}

SOURCE_REF=""                                    # the ref cloned; empty for a local tree
RELEASE_TAG=""                                   # the tag the binary comes from
BUILD_FROM_SOURCE="${HORUS_BUILD_FROM_SOURCE:-0}"
INSTALL_METHOD="release-binary"

if [ -n "${HORUS_LOCAL_SOURCE:-}" ]; then
    # The offline/air-gapped path. Every other route through this script needs
    # github.com, so a machine without it could not install at all. A local tree
    # has no tag, hence no matching release binary: pairing one with it would
    # recreate the exact skew this section exists to prevent.
    BUILD_FROM_SOURCE=1
    INSTALL_METHOD="local-source"
elif [ -n "${HORUS_INSTALL_BRANCH:-}" ]; then
    # The developer escape hatch used to change only the clone while the binary
    # still came from releases/latest — so the one documented way to "pin" an
    # install was the surest way to manufacture the skew. A branch has no
    # release binary; it forces a source build.
    if ! valid_ref "$BRANCH"; then
        fail "HORUS_INSTALL_BRANCH='${BRANCH}' is not a usable git ref"
        exit 1
    fi
    SOURCE_REF="$BRANCH"
    BUILD_FROM_SOURCE=1
    INSTALL_METHOD="source-build"
    warn "HORUS_INSTALL_BRANCH=${BRANCH} — building from source at that branch"
    echo "    No release binary is downloaded: a branch tree and a tagged binary"
    echo "    are different code, and installing both is what broke topic IPC."
else
    if [ -n "${HORUS_VERSION:-}" ]; then
        # Accept 0.4.0 and v0.4.0 alike; the tags themselves are v-prefixed.
        RELEASE_TAG="v${HORUS_VERSION#v}"
        info "Installing pinned release ${RELEASE_TAG}"
    else
        info "Resolving the latest release..."
        RELEASE_TAG=$(resolve_latest_tag) || RELEASE_TAG=""
        if [ -z "$RELEASE_TAG" ]; then
            # Deliberately no fallback to main: cloning main while the binary
            # came from a tag is the bug. Every alternative below names one ref.
            fail "Could not resolve the latest release of ${REPO}."
            echo "    github.com may be unreachable, or there may be no published release."
            echo "    Pin a release:      curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.4.0 bash"
            echo "    Build from a branch: curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_INSTALL_BRANCH=main bash"
            echo "    Use a local tree:    curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_LOCAL_SOURCE=/path/to/horus bash"
            exit 1
        fi
    fi
    if ! valid_ref "$RELEASE_TAG"; then
        fail "Refusing to use '${RELEASE_TAG}' as a release tag"
        exit 1
    fi
    SOURCE_REF="$RELEASE_TAG"
    [ "$BUILD_FROM_SOURCE" = "1" ] && INSTALL_METHOD="source-build"
    ok "Release ${RELEASE_TAG}"
fi

# --- Pre-built binary from GitHub Releases ---
ASSET_NAME="horus-${OS}-${ARCH}"
if [ "$OS" = "windows" ]; then
    ASSET_EXT="zip"
else
    ASSET_EXT="tar.gz"
fi
# Pinned to the resolved tag rather than /releases/latest/download/: "latest"
# names whatever is newest at the moment of each request, so it can move between
# the asset fetch and the SHA256SUMS fetch, and it is a different tree from the
# source unless the tag is written out explicitly. Both halves, one tag.
RELEASE_URL="https://github.com/${REPO}/releases/download/${RELEASE_TAG}/${ASSET_NAME}.${ASSET_EXT}"
CHECKSUM_URL="https://github.com/${REPO}/releases/download/${RELEASE_TAG}/SHA256SUMS"

# The source tree is required regardless of how we obtain the binary: `horus
# run`/`horus build` generate .horus/Cargo.toml with horus as *path*
# dependencies. A binary-only install produces a CLI that cannot build a single
# Rust project. So: always fetch source into the cache, and treat a pre-built
# binary purely as a way to skip the compile step.

CLONE_DIR=""
if [ -n "${HORUS_LOCAL_SOURCE:-}" ]; then
    # Used where it lies. Copying it would duplicate a checkout that carries a
    # multi-GB target/, and an installer must never point the rm -rf below at a
    # path the user handed it.
    SRC_TREE=$(cd "$HORUS_LOCAL_SOURCE" 2>/dev/null && pwd) || SRC_TREE=""
    if [ -z "$SRC_TREE" ]; then
        fail "HORUS_LOCAL_SOURCE='${HORUS_LOCAL_SOURCE}' is not a directory"
        exit 1
    fi
    # Same two markers the clone is checked against: horus/Cargo.toml is what
    # find_horus_source_dir() looks for and horus_core/Cargo.toml is what
    # SRC_VERSION parses, so a wrong path fails here rather than at the build.
    if [ ! -f "${SRC_TREE}/horus/Cargo.toml" ] || [ ! -f "${SRC_TREE}/horus_core/Cargo.toml" ]; then
        fail "HORUS_LOCAL_SOURCE='${SRC_TREE}' is not a HORUS source tree (no horus/Cargo.toml)"
        exit 1
    fi
    info "Using local source at ${SRC_TREE} (no network)"
else
    if ! command -v git &>/dev/null; then
        fail "git is required"
        exit 1
    fi

    info "Fetching HORUS source (${SOURCE_REF})..."
    CLONE_DIR=$(mktemp -d)
    CLONE_LOG=$(mktemp)
    # `git clone ... 2>&1 | tail -1` used to be the guard here, but this script
    # sets `set -e` without `set -o pipefail`, so a pipeline reports *tail*'s
    # status: the failure branch was dead code and a failed clone sailed on to
    # the rm -rf below. Redirect to a log instead, test git's own status, and
    # show the real error.
    #
    # --branch takes a tag as readily as a branch name, which is the whole point:
    # SOURCE_REF is the same tag the binary above was downloaded from.
    if ! git clone --depth 1 --branch "$SOURCE_REF" "https://github.com/${REPO}.git" "$CLONE_DIR" >"$CLONE_LOG" 2>&1; then
        fail "Failed to clone https://github.com/${REPO}.git at ref '${SOURCE_REF}'"
        tail -20 "$CLONE_LOG"
        rm -rf "$CLONE_DIR" "$CLONE_LOG"
        exit 1
    fi
    rm -f "$CLONE_LOG"

    # A clone can also exit 0 with a tree that is unusable to `horus run`.
    # horus/Cargo.toml is the exact marker find_horus_source_dir() looks for
    # (run_rust.rs), and horus_core/Cargo.toml is what SRC_VERSION parses below.
    # Both are checked *before* the destructive rm -rf further down, so a bad
    # fetch can never delete a working cached source tree.
    if [ ! -f "${CLONE_DIR}/horus/Cargo.toml" ] || [ ! -f "${CLONE_DIR}/horus_core/Cargo.toml" ]; then
        fail "Fetched tree for ref '${SOURCE_REF}' is incomplete (no horus/Cargo.toml)"
        rm -rf "$CLONE_DIR"
        exit 1
    fi
    SRC_TREE="$CLONE_DIR"
fi

# Version the cache dir by the crate version so multiple installs coexist and
# find_horus_source_dir() can prefer the tree matching the running CLI. An
# unparseable version used to fall back to "unknown" and cache the tree anyway;
# that only hid the failure until the first `horus run`.
SRC_VERSION=$(grep -m1 '^version' "${SRC_TREE}/horus_core/Cargo.toml" | sed 's/.*"\(.*\)".*/\1/')
# The parsed string becomes a path component below, and `rm -rf "$HORUS_SRC_DIR"`
# then deletes whatever it names. It is read out of the tree that was just
# cloned, so a `/` or `..` in it would point that rm -rf outside the cache — at
# whatever the caller (sometimes root) can delete. A crate version is
# alphanumerics with `. + - _` and nothing else; reject anything that is not.
case "$SRC_VERSION" in
    ''|*[!A-Za-z0-9.+_-]*|.*)
        fail "Refusing to use version '${SRC_VERSION}' from horus_core/Cargo.toml as a directory name"
        [ -n "$CLONE_DIR" ] && rm -rf "$CLONE_DIR"
        exit 1
        ;;
esac
HORUS_SRC_DIR="${HORUS_CACHE}/horus@${SRC_VERSION}"

# horus@<version> is the name run_rust.rs:1062 builds from CARGO_PKG_VERSION and
# registry/helpers.rs hardcodes. Renaming it breaks both.
mkdir -p "$HORUS_CACHE"
if [ -n "$CLONE_DIR" ]; then
    rm -rf "$HORUS_SRC_DIR"
    mv "$CLONE_DIR" "$HORUS_SRC_DIR"
    CLONE_DIR=""
elif [ "$SRC_TREE" != "$HORUS_SRC_DIR" ]; then
    # Link, do not copy, and never rm -rf the user's own tree. The link is what
    # lets find_horus_source_dir() resolve an air-gapped install without the
    # user exporting HORUS_SOURCE by hand; `rm -rf` on a symlink removes the
    # link, so `horus clean -a` still cannot reach the tree behind it.
    rm -rf "$HORUS_SRC_DIR"
    if ln -s "$SRC_TREE" "$HORUS_SRC_DIR" 2>/dev/null; then
        info "Linked ${HORUS_SRC_DIR} -> ${SRC_TREE}"
    else
        warn "Could not link ${HORUS_SRC_DIR} -> ${SRC_TREE}"
        echo "    Export HORUS_SOURCE=${SRC_TREE} so 'horus run' can find the source."
        HORUS_SRC_DIR="$SRC_TREE"
    fi
fi

# Record what was actually cached. A tag alone does not identify a tree once
# tags move, and the commit is the only thing that survives being compared
# against a bug report.
SRC_COMMIT=""
if command -v git >/dev/null 2>&1; then
    SRC_COMMIT=$(git -C "$HORUS_SRC_DIR" rev-parse HEAD 2>/dev/null || echo "")
fi
ok "Source cached at ${HORUS_SRC_DIR}"
info "  ref ${SOURCE_REF:-<local tree>}, commit ${SRC_COMMIT:-unknown}"

# NOT the exported POSIX TMPDIR: that is the variable mktemp, cc/ld, rustc,
# cargo, git and rustup's own installer all consult, and assigning to it here
# handed every child process a scratch directory that the cleanup below then
# deleted. Use a private name, and clean up from a single EXIT trap so the temp
# dir also goes away on the error paths and on a `set -e` abort mid-download.
HORUS_TMP=$(mktemp -d)
trap 'rm -rf "${HORUS_TMP:-}" "${CLONE_DIR:-}"' EXIT

# HORUS_BUILD_FROM_SOURCE=1 skips the pre-built binary entirely and compiles the
# cached source. This is the documented escape hatch when the checksum
# verification below cannot run (no sha256sum/shasum) or when a user does not
# want to trust a release artifact — so it has to actually exist. It is also
# what HORUS_INSTALL_BRANCH and HORUS_LOCAL_SOURCE set above: for those there is
# no release binary that matches the tree, and installing one anyway is the bug.
if [ "$BUILD_FROM_SOURCE" = "1" ]; then
    if [ "${HORUS_BUILD_FROM_SOURCE:-0}" = "1" ]; then
        info "HORUS_BUILD_FROM_SOURCE=1 — skipping the pre-built binary"
    fi
    HTTP_CODE="000"
else
    info "Checking for pre-built binary (${RELEASE_TAG})..."
    HTTP_CODE=$(curl -fsSL -o "${HORUS_TMP}/${ASSET_NAME}.${ASSET_EXT}" -w "%{http_code}" "$RELEASE_URL" 2>/dev/null || echo "000")
fi

# Refuse early on a toolchain that cannot build HORUS.
#
# The floor was previously discovered by the user only after install.sh had run
# and cargo had started compiling — several minutes in, as a cargo error about
# a package they had never heard of. It is read from the workspace manifest
# rather than hardcoded here, so there is one number and it lives in one place.
check_rust_version() {
    local required found
    required=$(grep -m1 '^rust-version' "$HORUS_SRC_DIR/Cargo.toml" 2>/dev/null \
        | sed 's/.*"\(.*\)".*/\1/')
    [ -n "$required" ] || return 0

    found=$(rustc --version 2>/dev/null | awk '{print $2}' | cut -d- -f1)
    [ -n "$found" ] || return 0

    # Compare as version numbers, not as strings: 1.100 is newer than 1.9.
    if [ "$(printf '%s\n%s\n' "$required" "$found" | sort -V | head -n1)" != "$required" ]; then
        fail "Rust $required or newer is required; found $found."
        echo "  Run: rustup update stable"
        exit 1
    fi
    ok "Rust $found (>= $required required)"
}

if [ "$HTTP_CODE" = "200" ] && [ -s "${HORUS_TMP}/${ASSET_NAME}.${ASSET_EXT}" ]; then
    # --- Fast path: pre-built binary, skip the compile ---

    # Verify the download against the release's published SHA256SUMS before
    # making it executable. The release workflow has always published this file
    # (release.yml:340: `sha256sum -- *.tar.gz *.zip > SHA256SUMS`) and
    # SECURITY.md claims "Package Verification: ... checksum verification" — but
    # the installer never fetched it, so a tampered or truncated asset was
    # executed unchecked. TLS alone does not cover a compromised or substituted
    # asset. The `--` there is load-bearing for the lookup below: shellcheck's
    # SC2035 fix is `./*.tar.gz`, which would prefix every name in SHA256SUMS
    # with `./` and make this `grep` miss on every platform.
    info "Verifying checksum..."
    if curl -fsSL -o "${HORUS_TMP}/SHA256SUMS" "$CHECKSUM_URL" 2>/dev/null && [ -s "${HORUS_TMP}/SHA256SUMS" ]; then
        EXPECTED=$(grep " ${ASSET_NAME}.${ASSET_EXT}\$" "${HORUS_TMP}/SHA256SUMS" 2>/dev/null | awk '{print $1}' | head -1)
        if [ -z "$EXPECTED" ]; then
            fail "SHA256SUMS has no entry for ${ASSET_NAME}.${ASSET_EXT}. Refusing to install an unverified binary."
            exit 1
        fi
        if command -v sha256sum >/dev/null 2>&1; then
            ACTUAL=$(sha256sum "${HORUS_TMP}/${ASSET_NAME}.${ASSET_EXT}" | awk '{print $1}')
        elif command -v shasum >/dev/null 2>&1; then
            ACTUAL=$(shasum -a 256 "${HORUS_TMP}/${ASSET_NAME}.${ASSET_EXT}" | awk '{print $1}')
        else
            fail "Neither sha256sum nor shasum is available, so the download cannot be verified. Install one, or build from source with HORUS_BUILD_FROM_SOURCE=1."
            exit 1
        fi
        if [ "$EXPECTED" != "$ACTUAL" ]; then
            fail "Checksum MISMATCH for ${ASSET_NAME}.${ASSET_EXT}"
            fail "  expected: $EXPECTED"
            fail "  actual:   $ACTUAL"
            fail "Refusing to install. This asset does not match the published release."
            exit 1
        fi
        ok "Checksum verified"
    else
        fail "Could not fetch SHA256SUMS from $CHECKSUM_URL — refusing to install an unverified binary."
        fail "Build from source instead: HORUS_BUILD_FROM_SOURCE=1 $0"
        exit 1
    fi

    info "Extracting binary..."
    if [ "$OS" = "windows" ]; then
        # `unzip` is not in Git for Windows' bundled MSYS2 set, and the `tar` that
        # *is* there is GNU tar, which cannot read a zip at all. So a Git Bash
        # install died at "unzip: command not found" (exit 127) one line after
        # printing "Checksum verified" — the extraction step had no guard while
        # every other external tool in this script has one. Windows ships its own
        # bsdtar as tar.exe since 1803, which does read a zip; try that next.
        if command -v unzip >/dev/null 2>&1; then
            unzip -q "${HORUS_TMP}/${ASSET_NAME}.zip" -d "$HORUS_TMP"
        elif [ -x "/c/Windows/System32/tar.exe" ]; then
            /c/Windows/System32/tar.exe -xf "${HORUS_TMP}/${ASSET_NAME}.zip" -C "$HORUS_TMP"
        elif [ -x "/mnt/c/Windows/System32/tar.exe" ]; then
            /mnt/c/Windows/System32/tar.exe -xf "${HORUS_TMP}/${ASSET_NAME}.zip" -C "$HORUS_TMP"
        else
            # Keep the verified download: HORUS_TMP is removed by the EXIT trap,
            # and telling someone to open a path that no longer exists by the
            # time they read it is its own small lie.
            cp "${HORUS_TMP}/${ASSET_NAME}.zip" "${HORUS_CACHE}/" 2>/dev/null || true
            fail "Nothing here can unpack a .zip: no unzip, and no Windows tar.exe."
            echo "    The verified download is at ${HORUS_CACHE}/${ASSET_NAME}.zip"
            echo "    Extract ${BINARY_NAME} from it into ${INSTALL_DIR}, or install"
            echo "    from source instead:"
            echo "      curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_BUILD_FROM_SOURCE=1 bash"
            exit 1
        fi
    else
        if ! command -v tar >/dev/null 2>&1; then
            fail "tar is required to unpack ${ASSET_NAME}.tar.gz"
            exit 1
        fi
        tar xzf "${HORUS_TMP}/${ASSET_NAME}.tar.gz" -C "$HORUS_TMP"
    fi
    # An archive that unpacked without the binary in it is not an install. This
    # used to `mv` unconditionally, so a layout change in the release workflow
    # would have surfaced as a bare "mv: No such file or directory".
    if [ ! -f "${HORUS_TMP}/${BINARY_NAME}" ]; then
        fail "${ASSET_NAME}.${ASSET_EXT} does not contain ${BINARY_NAME}"
        exit 1
    fi
    chmod +x "${HORUS_TMP}/${BINARY_NAME}" 2>/dev/null || true
    mv "${HORUS_TMP}/${BINARY_NAME}" "${INSTALL_DIR}/${BINARY_NAME}"
    ok "Downloaded pre-built binary (${RELEASE_TAG})"

else
    # --- Slow path: compile the cached source ---
    if [ "$BUILD_FROM_SOURCE" = "1" ]; then
        info "Building from source (~3-5 min)"
    else
        # Not an error: the release matrix does not cover every platform.
        warn "No pre-built binary for ${OS}-${ARCH} at ${RELEASE_TAG} — building from source (~3-5 min)"
        INSTALL_METHOD="source-build"
    fi
    echo ""

    # Dependencies
    if ! command -v cargo &>/dev/null; then
        install_rust
        if ! command -v cargo &>/dev/null; then
            fail "Failed to install Rust. Install manually: https://rustup.rs"
            exit 1
        fi
    fi
    check_rust_version
    if [ "$OS" = "linux" ] || [ "$OS" = "macos" ]; then
        install_build_deps
    fi
    ok "Dependencies ready"

    # Build — cargo shows its own progress
    echo ""
    info "Building from source (this takes a few minutes)..."
    echo ""
    BUILD_START=$(date +%s)
    cd "$HORUS_SRC_DIR"
    # Force stable toolchain — nightly may have compiler bugs
    # First try with LTO (smaller binary). If LLVM crashes (SIGILL — known
    # bug on some CPUs), retry without LTO.
    if ! cargo +stable build --release -p horus_manager 2>&1; then
        echo ""
        warn "Release build failed (possible LLVM/LTO bug), retrying without LTO..."
        echo ""
        export CARGO_PROFILE_RELEASE_LTO=off
        if ! cargo +stable build --release -p horus_manager 2>&1; then
            echo ""
            fail "Build failed"
            echo "    Report issues: https://github.com/${REPO}/issues"
            exit 1
        fi
    fi
    build_elapsed=$(($(date +%s) - BUILD_START))
    echo ""

    # Install binary. CARGO_TARGET_DIR may redirect the output tree, so ask
    # cargo where it actually put things rather than assuming ./target.
    BUILT_BIN="${CARGO_TARGET_DIR:-${HORUS_SRC_DIR}/target}/release/${BINARY_NAME}"
    if [ -f "$BUILT_BIN" ]; then
        cp "$BUILT_BIN" "${INSTALL_DIR}/${BINARY_NAME}"
        chmod +x "${INSTALL_DIR}/${BINARY_NAME}"
        ok "Built and installed in ${build_elapsed}s"
    else
        fail "Build succeeded but binary not found at ${BUILT_BIN}"
        echo "    Report issues: https://github.com/${REPO}/issues"
        exit 1
    fi
    cd /
fi

# --- Verify ---
#
# This used to be `VERSION=$(... --version 2>/dev/null || echo "installed")`,
# which turned every possible failure of the thing just installed into the word
# "installed" and then printed "Verified: horus installed" and "Installation
# complete!". The Linux release binaries carry a non-weak GLIBC_2.39
# version-need, so on Raspberry Pi OS, JetPack, Ubuntu 22.04, Debian 12 and
# RHEL 9 the loader refuses to start them — and the user was told it worked.
# Test the exit status, show what the binary actually said, and never continue.
if [ ! -f "${INSTALL_DIR}/${BINARY_NAME}" ]; then
    fail "Installation failed: nothing at ${INSTALL_DIR}/${BINARY_NAME}"
    exit 1
fi

VERSION_ERR="${HORUS_TMP}/version.err"
if ! VERSION_OUT=$("${INSTALL_DIR}/${BINARY_NAME}" --version 2>"$VERSION_ERR"); then
    fail "The installed binary does not run on this system."
    echo ""
    sed 's/^/      /' "$VERSION_ERR" 2>/dev/null | tail -5
    echo ""
    echo "    A \"GLIBC_2.xx not found\" here means the release binary was built"
    echo "    against a newer glibc than this machine has. Build from source"
    echo "    instead — it is the same tag, compiled locally:"
    echo "      curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_BUILD_FROM_SOURCE=1 bash"
    echo ""
    echo "    The binary that failed is at ${INSTALL_DIR}/${BINARY_NAME}."
    # Deliberately not falling back to the source path automatically: it runs
    # sudo apt-get and pipes rustup into sh, neither of which belongs in a
    # curl|bash with no TTY to ask.
    exit 1
fi

# `horus --version` prints "horus <x.y.z>" (clap, main.rs:14 — CARGO_PKG_VERSION
# of horus_manager). Keep only the version so the messages below do not read
# "horus horus 0.4.0". Drop the name and take the first remaining field rather
# than the last, so a future long_version with a commit suffix still parses.
VERSION=$(printf '%s\n' "$VERSION_OUT" | head -1 | sed 's/^horus //' | awk '{print $1}')

# The binary and the cached source must be one tree. If they are not, the CLI
# cannot read the shared memory its own libraries write and the symptom appears
# hours later as "Incompatible topic version". Compare here, where it is one
# string comparison, rather than there. horus_manager is what --version reports;
# horus_core is what named the cache directory. They are the same number in this
# workspace, and a build where they are not is a repo bug, not a skew — so
# prefer horus_manager's and fall back to SRC_VERSION.
EXPECTED_VERSION=$(grep -m1 '^version' "${HORUS_SRC_DIR}/horus_manager/Cargo.toml" 2>/dev/null \
    | sed 's/.*"\(.*\)".*/\1/')
[ -n "$EXPECTED_VERSION" ] || EXPECTED_VERSION="$SRC_VERSION"
if [ "$VERSION" != "$EXPECTED_VERSION" ]; then
    fail "Version skew: the binary reports ${VERSION}, the cached source is ${EXPECTED_VERSION}."
    echo "    Binary: ${INSTALL_DIR}/${BINARY_NAME}"
    echo "    Source: ${HORUS_SRC_DIR} (ref ${SOURCE_REF:-<local tree>})"
    echo "    These must be one tree. Build both halves from the same tag:"
    echo "      curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_BUILD_FROM_SOURCE=1 bash"
    exit 1
fi
ok "Verified: horus ${VERSION}"

# --- Record what was installed ---
#
# Two files, and nothing has written the first one since v0.2.0: version.rs:32-44
# reads ~/.horus/installed_version and uninstall.sh:819 deletes it, but no
# installer ever created it, so the version gate has been dead code and the
# remedy it printed ("re-run install.sh") could not have fixed anything.
#
# installed_version stays a bare version string with a trailing newline because
# that is what read_to_string().trim() there expects. install_manifest.toml is
# the richer record doctor.rs and `horus self update` read; readers must
# tolerate its absence (older installs) and the absence of individual keys — an
# air-gapped tree has no commit, and a tag old enough may not have the header
# TOPIC_VERSION is parsed from.
write_install_state() {
    local method="$1" topic_version manifest recorded_version

    if ! mkdir -p "$HORUS_STATE_DIR" 2>/dev/null; then
        warn "Could not create ${HORUS_STATE_DIR} — version state not recorded"
        return 0
    fi

    # Record what the binary reports, not SRC_VERSION. version.rs compares this
    # file against horus_manager's CARGO_PKG_VERSION, while SRC_VERSION is read
    # out of horus_core/Cargo.toml — two independently declared numbers (the
    # workspace inherits rust-version, not version). They agree today, and the
    # skew check above refuses the install when they do not, but writing the
    # other one would make the gate compare a number nothing else uses.
    recorded_version="${VERSION:-$SRC_VERSION}"

    # A read-only state dir must not abort an otherwise complete install: the
    # binary is already in place by this point, and `set -e` on a failed redirect
    # would exit non-zero after the work was done.
    if ! printf '%s\n' "$recorded_version" > "${HORUS_STATE_DIR}/installed_version"; then
        warn "Could not write ${HORUS_STATE_DIR}/installed_version"
        return 0
    fi

    # The number the two halves actually have to agree on: a CLI built at
    # TOPIC_VERSION 3 cannot read a topic written by libraries at 4.
    topic_version=$(grep -m1 'TOPIC_VERSION: u32' \
        "${HORUS_SRC_DIR}/horus_core/src/communication/topic/header.rs" 2>/dev/null \
        | sed 's/.*= *\([0-9][0-9]*\).*/\1/')

    manifest="${HORUS_STATE_DIR}/install_manifest.toml"
    {
        echo "# Written by install.sh. Describes the tree this install came from;"
        echo "# horus doctor and horus self update read it. Do not hand-edit."
        echo "version = \"${recorded_version}\""
        echo "tag = \"${RELEASE_TAG}\""
        echo "commit = \"${SRC_COMMIT}\""
        case "$topic_version" in
            ''|*[!0-9]*) echo "# topic_version could not be read from the source tree" ;;
            *) echo "topic_version = ${topic_version}" ;;
        esac
        echo "source_dir = \"${HORUS_SRC_DIR}\""
        echo "binary = \"${INSTALL_DIR}/${BINARY_NAME}\""
        echo "install_method = \"${method}\""
        echo "installed_at = \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\""
    } > "$manifest"

    ok "Recorded ${recorded_version} in ${HORUS_STATE_DIR}/installed_version"
    return 0
}
write_install_state "$INSTALL_METHOD"

# Everything above ran as root when the caller used sudo; hand the results to
# the account that will use them.
reown_for_target_user "$HORUS_STATE_DIR" "${INSTALL_DIR}/${BINARY_NAME}"

# --- Which rc file does this shell read? ---
#
# Detected once, up front, because three separate blocks below need it: the PATH
# block, the completion installer (zsh needs an fpath line to load the script at
# all) and the closing "restart your shell" hint.
#
# This used to live *inside* the else-branch of the PATH check below, which made
# it dead for the common case. Anyone re-installing, upgrading, or simply
# already carrying ~/.cargo/bin on PATH — i.e. everyone, since install.sh
# requires a Rust toolchain — takes the "PATH already configured" branch and
# never assigned SHELL_RC. install.sh runs `set -e` but not `set -u`, so the
# later `[ -n "$SHELL_RC" ]` guard silently did nothing and the zsh fpath line
# was never written, while the installer still printed "Shell completions
# installed (zsh)". A completion script in a directory zsh never scans, plus a
# success message saying otherwise.
SHELL_RC=""
case "${SHELL:-/bin/bash}" in
    */zsh)  SHELL_RC="$HOME/.zshrc" ;;
    */bash) SHELL_RC="$HOME/.bashrc" ;;
    */fish) SHELL_RC="${XDG_CONFIG_HOME:-$HOME/.config}/fish/config.fish" ;;
esac

# --- Configure PATH ---
#
# `echo "$PATH" | grep -q "$INSTALL_DIR"` was a substring test, so /opt/horus/bin
# counted as configured when PATH held /opt/horus/bin2, and an INSTALL_DIR with
# a `.` or `+` in it was read as a regex. Match whole entries.
#
# PATH_WAS_CONFIGURED is remembered for the closing hint: this script exports
# INSTALL_DIR into its own PATH below, so by the end `command -v horus` always
# succeeds and the "restart your terminal" warning could never fire — which is
# exactly the case where the user's *next* shell has no horus on PATH.
PATH_WAS_CONFIGURED=true
case ":${PATH}:" in
    *":${INSTALL_DIR}:"*) ;;
    *) PATH_WAS_CONFIGURED=false ;;
esac
if [ "$PATH_WAS_CONFIGURED" = true ]; then
    ok "PATH already configured"
else
    # Under sudo, $SHELL_RC is root's rc file, not ${TARGET_HOME}'s — see the
    # shell-integration block below. The closing hint tells the user what to add.
    if [ -n "$SHELL_RC" ] && [ -z "$TARGET_USER" ]; then
        mkdir -p "$(dirname "$SHELL_RC")" 2>/dev/null || true
        case "$SHELL_RC" in
            */config.fish)
                # fish has no `export` builtin: `export PATH=...` is a syntax
                # error there and config.fish stops loading at it. Earlier
                # installers wrote exactly that line, and the old
                # `grep -q "$INSTALL_DIR"` guard then *found* the broken line
                # and skipped writing a working one — so a config.fish poisoned
                # once stayed poisoned through every upgrade. Remove it first.
                if grep -q "^export PATH=.*${INSTALL_DIR}" "$SHELL_RC" 2>/dev/null; then
                    sed -i.horusbak "\\|^export PATH=.*${INSTALL_DIR}|d" "$SHELL_RC" 2>/dev/null || \
                        sed -i '' "\\|^export PATH=.*${INSTALL_DIR}|d" "$SHELL_RC" 2>/dev/null
                    rm -f "${SHELL_RC}.horusbak" 2>/dev/null
                    warn "Removed a POSIX 'export PATH' line fish cannot parse from ${SHELL_RC}"
                fi
                grep -qF "fish_add_path ${INSTALL_DIR}" "$SHELL_RC" 2>/dev/null || \
                    echo "fish_add_path ${INSTALL_DIR}" >> "$SHELL_RC"
                ;;
            *)
                grep -q "$INSTALL_DIR" "$SHELL_RC" 2>/dev/null || \
                    echo "export PATH=\"${INSTALL_DIR}:\$PATH\"" >> "$SHELL_RC"
                ;;
        esac
    fi
    export PATH="${INSTALL_DIR}:$PATH"
    ok "Added to PATH"
fi

# --- Shell integration ---
#
# `horus env --init` writes ~/.horus/env.sh and appends a source line to
# .bashrc, .zshrc and fish's conf.d. That file defines shell functions shadowing
# cargo, pip, pip3, cmake, conan and vcpkg — inside a horus project they
# delegate to `horus <tool>`, outside they pass straight through.
#
# That is a reasonable default and `horus env --uninstall` reverses it cleanly,
# but editing someone's shell rc files is not something to do silently. The
# output used to go to /dev/null, so the user was never told. Now they are told,
# and they can decline. The variable is read by *this* script, so with a pipe it
# belongs on the right-hand side — `HORUS_NO_SHELL_INTEGRATION=1 curl ... | bash`
# puts it in curl's environment, where nothing reads it:
#
#   curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_NO_SHELL_INTEGRATION=1 bash
if [ -n "$TARGET_USER" ]; then
    # `horus env --init` and the completion installer below both write to $HOME,
    # and under sudo that is /root — not ${TARGET_HOME}. Root-owned lines in the
    # wrong user's rc file is worse than no shell integration.
    info "Skipping shell integration: running as root on behalf of ${TARGET_USER}"
    echo "  Run 'horus env --init' as ${TARGET_USER} to enable the cargo/pip/cmake proxies."
elif [ -n "${HORUS_NO_SHELL_INTEGRATION:-}" ]; then
    info "Skipping shell integration (HORUS_NO_SHELL_INTEGRATION is set)"
    echo "  Run 'horus env --init' later to enable the cargo/pip/cmake proxies."
else
    if horus env --init; then
        echo "  Shadows cargo, pip, pip3, cmake, conan and vcpkg inside horus"
        echo "  projects only. Undo with: horus env --uninstall"
        echo "  Skip next time: curl ... | HORUS_NO_SHELL_INTEGRATION=1 bash"
    fi
fi

# --- Shell completions ---
#
# `horus completion <shell>` has worked since the CLI shipped, and the code
# comment on it claims "Hidden command used by install.sh for automatic
# completion setup" — but install.sh never mentioned it, so nobody has ever
# received a completion script. For a CLI with ~40 commands, dozens of
# subcommands and single-letter aliases (t n p a m s i l srv bb mon rec tf),
# that is the difference between discoverable and not.
#
# Writing the file is only half the job. bash and fish rescan their completion
# directories on every start, but zsh only reads `fpath`, and only at compinit
# time — so a `_horus` no fpath line mentions, or an fpath line appended to the
# *end* of .zshrc after the user's compinit has already run (oh-my-zsh, prezto
# and every other framework call compinit from the file .zshrc sources), is a
# file zsh never loads. That is why rustup's own instructions put the fpath line
# before compinit, and it is what these markers are for: uninstall.sh deletes
# the block between them verbatim.
HORUS_COMP_BEGIN="# >>> horus completions >>>"
HORUS_COMP_END="# <<< horus completions <<<"

add_zsh_fpath_block() {
    rc="$1"
    fpath_line="$2"

    [ -n "$rc" ] || return 0
    if [ -f "$rc" ] && grep -qF "$HORUS_COMP_BEGIN" "$rc" 2>/dev/null; then
        return 0
    fi
    mkdir -p "$(dirname "$rc")" 2>/dev/null || return 0
    [ -f "$rc" ] || : > "$rc" 2>/dev/null || return 0

    # First line that brings zsh's completion system up, if any.
    anchor=$(grep -nE 'compinit|oh-my-zsh\.sh|prezto/init\.zsh' "$rc" 2>/dev/null | head -n 1 | cut -d: -f1)
    tmp="${rc}.horus.$$"

    if [ -n "$anchor" ]; then
        if awk -v n="$anchor" -v b="$HORUS_COMP_BEGIN" -v f="$fpath_line" -v e="$HORUS_COMP_END" \
               'NR == n { print b; print f; print e } { print }' "$rc" > "$tmp" 2>/dev/null &&
           [ -s "$tmp" ]; then
            # cat rather than mv: keeps the rc file's inode, mode and any
            # symlink the user pointed it at.
            cat "$tmp" > "$rc"
        fi
        rm -f "$tmp" 2>/dev/null || true
    else
        # Nothing initialises completions, so an fpath line on its own would
        # still load nothing. Turn compinit on too.
        printf '\n%s\n%s\nautoload -Uz compinit && compinit\n%s\n' \
            "$HORUS_COMP_BEGIN" "$fpath_line" "$HORUS_COMP_END" >> "$rc"
    fi
    return 0
}

install_completions() {
    case "${SHELL:-/bin/bash}" in
        */zsh)
            COMP_DIR="${HOME}/.zfunc"
            COMP_FILE="${COMP_DIR}/_horus"
            COMP_SHELL="zsh"
            ;;
        */fish)
            COMP_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/fish/completions"
            COMP_FILE="${COMP_DIR}/horus.fish"
            COMP_SHELL="fish"
            ;;
        */bash)
            # Same XDG rule install_man_page() below already follows. This used
            # to hardcode ~/.local/share, so a user with XDG_DATA_HOME set got a
            # file bash-completion does not look at.
            COMP_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/bash-completion/completions"
            COMP_FILE="${COMP_DIR}/horus"
            COMP_SHELL="bash"
            ;;
        *)
            return 0
            ;;
    esac

    mkdir -p "$COMP_DIR" 2>/dev/null || return 0
    if horus completion "$COMP_SHELL" > "${COMP_FILE}.tmp" 2>/dev/null &&
       [ -s "${COMP_FILE}.tmp" ]; then
        mv "${COMP_FILE}.tmp" "$COMP_FILE"
        if [ "$COMP_SHELL" = "zsh" ]; then
            add_zsh_fpath_block "$SHELL_RC" "fpath=(${COMP_DIR} \$fpath)"
        fi
        ok "Shell completions installed (${COMP_SHELL}): ${COMP_FILE}"
    else
        # Never fail the install over completions.
        rm -f "${COMP_FILE}.tmp" 2>/dev/null
    fi
}
# Same reason as the shell integration above: COMP_DIR hangs off $HOME, which
# under sudo is root's, not ${TARGET_HOME}'s.
if [ -z "$TARGET_USER" ]; then
    install_completions
fi

# --- Man page ---
#
# HORUS shipped no man page at all: `man horus` found nothing, on a tool that
# already installed a completion script. `horus man` renders it from the same
# clap tree the binary is built from, so it cannot describe a command that does
# not exist.
install_man_page() {
    # Prefer the user-local location so this needs no root; fall back to the
    # system one only if we are already root.
    if [ "$(id -u)" = "0" ]; then
        MAN_DIR="/usr/local/share/man/man1"
    else
        MAN_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/man/man1"
    fi

    mkdir -p "$MAN_DIR" 2>/dev/null || return 0
    if horus man > "${MAN_DIR}/horus.1.tmp" 2>/dev/null &&
       [ -s "${MAN_DIR}/horus.1.tmp" ]; then
        mv "${MAN_DIR}/horus.1.tmp" "${MAN_DIR}/horus.1"
        ok "Man page installed (man horus)"
    else
        # Never fail the install over a man page.
        rm -f "${MAN_DIR}/horus.1.tmp" 2>/dev/null
    fi
}
install_man_page

# --- Done ---
elapsed_total=$(($(date +%s) - INSTALL_START))
echo ""
echo -e "  ${GREEN}${BOLD}Installation complete!${NC}  horus ${VERSION}  (${elapsed_total}s)"
echo ""
echo "  Get started:"
echo -e "    ${CYAN}horus new my_robot -r${NC}     Create a Rust project"
echo -e "    ${CYAN}horus new my_robot -p${NC}     Create a Python project"
echo -e "    ${CYAN}horus doctor${NC}              Check your environment"
echo ""
echo -e "  Docs: ${CYAN}https://docs.horusrobotics.dev${NC}"
echo ""

# `command -v horus` was the condition here, and this script had already run
# `export PATH="${INSTALL_DIR}:$PATH"` — so the lookup always succeeded and this
# hint never printed for the one person who needs it: the user whose *login*
# shell still has no ${INSTALL_DIR}. Ask what was true before the export.
if [ "$PATH_WAS_CONFIGURED" = false ]; then
    # ${SHELL_RC##*/} used to be printed as `source ~/<basename>`, which is the
    # wrong path for fish (~/.config/fish/config.fish) and, when SHELL_RC was
    # still scoped to the PATH branch, degenerated to a bare `source ~/`.
    if [ -n "$SHELL_RC" ]; then
        warn "Restart your terminal or run: ${CYAN}source ${SHELL_RC}${NC}"
    else
        warn "Restart your terminal to pick up ${INSTALL_DIR} on PATH"
    fi
    echo ""
fi

if [ -n "$TARGET_USER" ]; then
    warn "Installed for ${TARGET_USER}. Nothing was added to their PATH — as ${TARGET_USER}, run:"
    echo "      export PATH=\"${INSTALL_DIR}:\$PATH\""
    echo ""
fi

if [ -n "${HORUS_PREFIX:-}" ]; then
    # find_horus_source_dir() (run_rust.rs) searches HORUS_SOURCE, a handful of
    # fixed development paths, then the two cache roots — none of which is an
    # arbitrary prefix. Say so rather than let the first `horus run` discover it.
    warn "Installed under ${HORUS_PREFIX}, which 'horus run' does not search. Each user needs:"
    echo "      export PATH=\"${INSTALL_DIR}:\$PATH\""
    echo "      export HORUS_SOURCE=\"${HORUS_SRC_DIR}\""
    echo ""
    # Nothing outside the prefix records where the prefix was, so the uninstaller
    # cannot find it either — uninstall.sh reads HORUS_PREFIX from its own
    # environment for exactly this case (uninstall.sh:55-58).
    warn "To remove it later, pass the same prefix:"
    echo "      curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | HORUS_PREFIX=${HORUS_PREFIX} bash -s -- --yes"
    echo ""
    # version.rs reads ~/.horus/installed_version and ~/.horus/install_manifest.toml
    # unconditionally, so the copies written under the prefix are state nothing
    # reads back. The gate stays quiet rather than mis-firing, but it also cannot
    # warn about drift on this install.
    warn "The version gate is inactive for a prefix install: version.rs looks in ~/.horus, not ${HORUS_PREFIX}."
    echo ""
fi
