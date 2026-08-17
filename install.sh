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
if ! git clone --depth 1 --branch "$BRANCH" "https://github.com/${REPO}.git" "$CLONE_DIR" 2>&1 | tail -1; then
    fail "Failed to clone https://github.com/${REPO}.git (branch: ${BRANCH})"
    rm -rf "$CLONE_DIR"
    exit 1
fi

# Version the cache dir by the crate version so multiple installs coexist and
# find_horus_source_dir() can prefer the tree matching the running CLI.
SRC_VERSION=$(grep -m1 '^version' "${CLONE_DIR}/horus_core/Cargo.toml" 2>/dev/null | sed 's/.*"\(.*\)".*/\1/')
[ -n "$SRC_VERSION" ] || SRC_VERSION="unknown"
HORUS_SRC_DIR="${HORUS_CACHE}/horus@${SRC_VERSION}"

mkdir -p "$HORUS_CACHE"
rm -rf "$HORUS_SRC_DIR"
mv "$CLONE_DIR" "$HORUS_SRC_DIR"
ok "Source cached at ~/.horus/cache/horus@${SRC_VERSION}"

TMPDIR=$(mktemp -d)

# HORUS_BUILD_FROM_SOURCE=1 skips the pre-built binary entirely and compiles the
# cached source. This is the documented escape hatch when the checksum
# verification below cannot run (no sha256sum/shasum) or when a user does not
# want to trust a release artifact — so it has to actually exist.
if [ "${HORUS_BUILD_FROM_SOURCE:-0}" = "1" ]; then
    info "HORUS_BUILD_FROM_SOURCE=1 — skipping the pre-built binary"
    HTTP_CODE="000"
else
    info "Checking for pre-built binary..."
    HTTP_CODE=$(curl -fsSL -o "${TMPDIR}/${ASSET_NAME}.${ASSET_EXT}" -w "%{http_code}" "$RELEASE_URL" 2>/dev/null || echo "000")
fi

if [ "$HTTP_CODE" = "200" ] && [ -s "${TMPDIR}/${ASSET_NAME}.${ASSET_EXT}" ]; then
    # --- Fast path: pre-built binary, skip the compile ---

    # Verify the download against the release's published SHA256SUMS before
    # making it executable. The release workflow has always published this file
    # (release.yml: `sha256sum *.tar.gz *.zip > SHA256SUMS`) and SECURITY.md
    # claims "Package Verification: ... checksum verification" — but the
    # installer never fetched it, so a tampered or truncated asset was executed
    # unchecked. TLS alone does not cover a compromised or substituted asset.
    info "Verifying checksum..."
    if curl -fsSL -o "${TMPDIR}/SHA256SUMS" "$CHECKSUM_URL" 2>/dev/null && [ -s "${TMPDIR}/SHA256SUMS" ]; then
        EXPECTED=$(grep " ${ASSET_NAME}.${ASSET_EXT}\$" "${TMPDIR}/SHA256SUMS" 2>/dev/null | awk '{print $1}' | head -1)
        if [ -z "$EXPECTED" ]; then
            rm -rf "$TMPDIR"
            fail "SHA256SUMS has no entry for ${ASSET_NAME}.${ASSET_EXT}. Refusing to install an unverified binary."
            exit 1
        fi
        if command -v sha256sum >/dev/null 2>&1; then
            ACTUAL=$(sha256sum "${TMPDIR}/${ASSET_NAME}.${ASSET_EXT}" | awk '{print $1}')
        elif command -v shasum >/dev/null 2>&1; then
            ACTUAL=$(shasum -a 256 "${TMPDIR}/${ASSET_NAME}.${ASSET_EXT}" | awk '{print $1}')
        else
            rm -rf "$TMPDIR"
            fail "Neither sha256sum nor shasum is available, so the download cannot be verified. Install one, or build from source with HORUS_BUILD_FROM_SOURCE=1."
            exit 1
        fi
        if [ "$EXPECTED" != "$ACTUAL" ]; then
            rm -rf "$TMPDIR"
            fail "Checksum MISMATCH for ${ASSET_NAME}.${ASSET_EXT}"
            fail "  expected: $EXPECTED"
            fail "  actual:   $ACTUAL"
            fail "Refusing to install. This asset does not match the published release."
            exit 1
        fi
        ok "Checksum verified"
    else
        rm -rf "$TMPDIR"
        fail "Could not fetch SHA256SUMS from $CHECKSUM_URL — refusing to install an unverified binary."
        fail "Build from source instead: HORUS_BUILD_FROM_SOURCE=1 $0"
        exit 1
    fi

    info "Extracting binary..."
    if [ "$OS" = "windows" ]; then
        unzip -q "${TMPDIR}/${ASSET_NAME}.zip" -d "$TMPDIR"
    else
        tar xzf "${TMPDIR}/${ASSET_NAME}.tar.gz" -C "$TMPDIR"
    fi
    chmod +x "${TMPDIR}/${BINARY_NAME}" 2>/dev/null || true
    mv "${TMPDIR}/${BINARY_NAME}" "${INSTALL_DIR}/${BINARY_NAME}"
    rm -rf "$TMPDIR"
    ok "Downloaded pre-built binary"

else
    # --- Slow path: compile the cached source ---
    rm -rf "$TMPDIR"
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
    if ! cargo +stable build --release -p horus_manager --no-default-features 2>&1; then
        echo ""
        warn "Release build failed (possible LLVM/LTO bug), retrying without LTO..."
        echo ""
        export CARGO_PROFILE_RELEASE_LTO=off
        if ! cargo +stable build --release -p horus_manager --no-default-features 2>&1; then
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

# --- Configure PATH ---
if echo "$PATH" | grep -q "$INSTALL_DIR"; then
    ok "PATH already configured"
else
    SHELL_RC=""
    case "${SHELL:-/bin/bash}" in
        */zsh)  SHELL_RC="$HOME/.zshrc" ;;
        */bash) SHELL_RC="$HOME/.bashrc" ;;
        */fish) SHELL_RC="$HOME/.config/fish/config.fish" ;;
    esac

    if [ -n "$SHELL_RC" ]; then
        if ! grep -q "$INSTALL_DIR" "$SHELL_RC" 2>/dev/null; then
            echo "export PATH=\"${INSTALL_DIR}:\$PATH\"" >> "$SHELL_RC"
        fi
    fi
    export PATH="${INSTALL_DIR}:$PATH"
    ok "Added to PATH"
fi

# Shell integration
horus env --init 2>/dev/null || true

# --- Shell completions ---
#
# `horus completion <shell>` has worked since the CLI shipped, and the code
# comment on it claims "Hidden command used by install.sh for automatic
# completion setup" — but install.sh never mentioned it, so nobody has ever
# received a completion script. For a CLI with ~40 commands, dozens of
# subcommands and single-letter aliases (t n p a m s i l srv bb mon rec tf),
# that is the difference between discoverable and not.
install_completions() {
    case "${SHELL:-/bin/bash}" in
        */zsh)
            COMP_DIR="${HOME}/.zfunc"
            COMP_FILE="${COMP_DIR}/_horus"
            COMP_SHELL="zsh"
            # zsh needs the directory on fpath before compinit.
            COMP_HINT="fpath=(${COMP_DIR} \$fpath)"
            ;;
        */fish)
            COMP_DIR="${HOME}/.config/fish/completions"
            COMP_FILE="${COMP_DIR}/horus.fish"
            COMP_SHELL="fish"
            COMP_HINT=""
            ;;
        */bash)
            COMP_DIR="${HOME}/.local/share/bash-completion/completions"
            COMP_FILE="${COMP_DIR}/horus"
            COMP_SHELL="bash"
            COMP_HINT=""
            ;;
        *)
            return 0
            ;;
    esac

    mkdir -p "$COMP_DIR" 2>/dev/null || return 0
    if horus completion "$COMP_SHELL" > "${COMP_FILE}.tmp" 2>/dev/null &&
       [ -s "${COMP_FILE}.tmp" ]; then
        mv "${COMP_FILE}.tmp" "$COMP_FILE"
        ok "Shell completions installed (${COMP_SHELL})"
        if [ -n "$COMP_HINT" ] && [ -n "$SHELL_RC" ] &&
           ! grep -qF "$COMP_HINT" "$SHELL_RC" 2>/dev/null; then
            echo "$COMP_HINT" >> "$SHELL_RC"
        fi
    else
        # Never fail the install over completions.
        rm -f "${COMP_FILE}.tmp" 2>/dev/null
    fi
}
install_completions

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
    warn "Restart your terminal or run: ${CYAN}source ~/${SHELL_RC##*/}${NC}"
    echo ""
fi
