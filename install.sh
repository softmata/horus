#!/bin/bash
# HORUS Installer
#
# One-line install:
#   curl -fsSL https://horusrobotics.dev/install | bash
#   curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
#
# Flow:
#   1. Detect OS and architecture
#   2. Try downloading pre-built binary from GitHub Releases (fast)
#   3. Clone the source (default branch: main), cache it, and build from source
#   4. Install to ~/.cargo/bin/horus or ~/.local/bin/horus
#   5. Verify and configure PATH

set -e

# --- Config ---
REPO="softmata/horus"
# Branch to build from. There is no long-lived "release" branch on origin —
# tags (v*.*.*) are cut from main, so main is the source of truth here.
BRANCH="${HORUS_INSTALL_BRANCH:-main}"
# Where the source tree is kept after building. `horus run` compiles user
# projects against horus as *path* dependencies (see cargo_gen.rs ->
# find_horus_source_dir), so the source must outlive the install or no Rust
# project can ever be built.
HORUS_CACHE="$HOME/.horus/cache"

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

find_install_dir() {
    if [ -d "$HOME/.cargo/bin" ]; then
        echo "$HOME/.cargo/bin"
    elif [ -d "$HOME/.local/bin" ]; then
        echo "$HOME/.local/bin"
    else
        mkdir -p "$HOME/.local/bin"
        echo "$HOME/.local/bin"
    fi
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
}

# --- Main ---
OS=$(detect_os)
ARCH=$(detect_arch)
INSTALL_DIR=$(find_install_dir)
INSTALL_START=$(date +%s)
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

if [ "$OS" = "unknown" ] || [ "$ARCH" = "unknown" ]; then
    fail "Unsupported platform: $(uname -s) $(uname -m)"
    echo "    Supported: Linux/macOS (amd64, arm64), Windows (via Git Bash/WSL)"
    exit 1
fi

if ! command -v curl &>/dev/null; then
    fail "curl is required"
    exit 1
fi

# --- Try pre-built binary from GitHub Releases ---
ASSET_NAME="horus-${OS}-${ARCH}"
if [ "$OS" = "windows" ]; then
    ASSET_EXT="zip"
else
    ASSET_EXT="tar.gz"
fi
# GitHub release asset URL
RELEASE_URL="https://github.com/${REPO}/releases/latest/download/${ASSET_NAME}.${ASSET_EXT}"
CHECKSUM_URL="https://github.com/${REPO}/releases/latest/download/SHA256SUMS"

# The source tree is required regardless of how we obtain the binary: `horus
# run`/`horus build` generate .horus/Cargo.toml with horus as *path*
# dependencies. A binary-only install produces a CLI that cannot build a single
# Rust project. So: always fetch source into the cache, and treat a pre-built
# binary purely as a way to skip the compile step.

if ! command -v git &>/dev/null; then
    fail "git is required"
    exit 1
fi

info "Fetching HORUS source (${BRANCH})..."
CLONE_DIR=$(mktemp -d)
CLONE_LOG=$(mktemp)
# `git clone ... 2>&1 | tail -1` used to be the guard here, but this script sets
# `set -e` without `set -o pipefail`, so a pipeline reports *tail*'s status: the
# failure branch was dead code and a failed clone sailed on to the rm -rf below.
# Redirect to a log instead, test git's own status, and show the real error.
if ! git clone --depth 1 --branch "$BRANCH" "https://github.com/${REPO}.git" "$CLONE_DIR" >"$CLONE_LOG" 2>&1; then
    fail "Failed to clone https://github.com/${REPO}.git (branch: ${BRANCH})"
    tail -20 "$CLONE_LOG"
    rm -rf "$CLONE_DIR" "$CLONE_LOG"
    exit 1
fi
rm -f "$CLONE_LOG"

# A clone can also exit 0 with a tree that is unusable to `horus run`.
# horus/Cargo.toml is the exact marker find_horus_source_dir() looks for
# (run_rust.rs), and horus_core/Cargo.toml is what SRC_VERSION parses below.
# Both are checked *before* the destructive rm -rf further down, so a bad fetch
# can never delete a working cached source tree.
if [ ! -f "${CLONE_DIR}/horus/Cargo.toml" ] || [ ! -f "${CLONE_DIR}/horus_core/Cargo.toml" ]; then
    fail "Fetched tree for branch '${BRANCH}' is incomplete (no horus/Cargo.toml)"
    rm -rf "$CLONE_DIR"
    exit 1
fi

# Version the cache dir by the crate version so multiple installs coexist and
# find_horus_source_dir() can prefer the tree matching the running CLI. An
# unparseable version used to fall back to "unknown" and cache the tree anyway;
# that only hid the failure until the first `horus run`.
SRC_VERSION=$(grep -m1 '^version' "${CLONE_DIR}/horus_core/Cargo.toml" | sed 's/.*"\(.*\)".*/\1/')
# The parsed string becomes a path component below, and `rm -rf "$HORUS_SRC_DIR"`
# then deletes whatever it names. It is read out of the tree that was just
# cloned, so a `/` or `..` in it would point that rm -rf outside the cache — at
# whatever the caller (sometimes root) can delete. A crate version is
# alphanumerics with `. + - _` and nothing else; reject anything that is not.
case "$SRC_VERSION" in
    ''|*[!A-Za-z0-9.+_-]*|.*)
        fail "Refusing to use version '${SRC_VERSION}' from horus_core/Cargo.toml as a directory name"
        rm -rf "$CLONE_DIR"
        exit 1
        ;;
esac
HORUS_SRC_DIR="${HORUS_CACHE}/horus@${SRC_VERSION}"

mkdir -p "$HORUS_CACHE"
rm -rf "$HORUS_SRC_DIR"
mv "$CLONE_DIR" "$HORUS_SRC_DIR"
CLONE_DIR=""
ok "Source cached at ~/.horus/cache/horus@${SRC_VERSION}"

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
# want to trust a release artifact — so it has to actually exist.
if [ "${HORUS_BUILD_FROM_SOURCE:-0}" = "1" ]; then
    info "HORUS_BUILD_FROM_SOURCE=1 — skipping the pre-built binary"
    HTTP_CODE="000"
else
    info "Checking for pre-built binary..."
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
    # (release.yml: `sha256sum *.tar.gz *.zip > SHA256SUMS`) and SECURITY.md
    # claims "Package Verification: ... checksum verification" — but the
    # installer never fetched it, so a tampered or truncated asset was executed
    # unchecked. TLS alone does not cover a compromised or substituted asset.
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
        unzip -q "${HORUS_TMP}/${ASSET_NAME}.zip" -d "$HORUS_TMP"
    else
        tar xzf "${HORUS_TMP}/${ASSET_NAME}.tar.gz" -C "$HORUS_TMP"
    fi
    chmod +x "${HORUS_TMP}/${BINARY_NAME}" 2>/dev/null || true
    mv "${HORUS_TMP}/${BINARY_NAME}" "${INSTALL_DIR}/${BINARY_NAME}"
    ok "Downloaded pre-built binary"

else
    # --- Slow path: compile the cached source ---
    warn "No pre-built binary for ${OS}-${ARCH} — building from source (~3-5 min)"
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
if [ -f "${INSTALL_DIR}/${BINARY_NAME}" ]; then
    VERSION=$("${INSTALL_DIR}/${BINARY_NAME}" --version 2>/dev/null || echo "installed")
    ok "Verified: horus ${VERSION}"
else
    fail "Installation failed"
    exit 1
fi

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
if echo "$PATH" | grep -q "$INSTALL_DIR"; then
    ok "PATH already configured"
else
    if [ -n "$SHELL_RC" ]; then
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
# and they can decline:
#
#   HORUS_NO_SHELL_INTEGRATION=1 curl ... | bash
if [ -n "${HORUS_NO_SHELL_INTEGRATION:-}" ]; then
    info "Skipping shell integration (HORUS_NO_SHELL_INTEGRATION is set)"
    echo "  Run 'horus env --init' later to enable the cargo/pip/cmake proxies."
else
    if horus env --init; then
        echo "  Shadows cargo, pip, pip3, cmake, conan and vcpkg inside horus"
        echo "  projects only. Undo with: horus env --uninstall"
        echo "  Skip next time with: HORUS_NO_SHELL_INTEGRATION=1"
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
install_completions

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

if ! command -v horus &>/dev/null; then
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
