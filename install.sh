#!/bin/bash
# HORUS Installer
#
# One-line install:
#   curl -fsSL https://horusrobotics.dev/install | bash
#   curl -fsSL https://raw.githubusercontent.com/softmata/horus/release/install.sh | bash
#
# Flow:
#   1. Detect OS and architecture
#   2. Try downloading pre-built binary from GitHub Releases (fast)
#   3. If unavailable, clone release branch and build from source (slow)
#   4. Install to ~/.cargo/bin/horus or ~/.local/bin/horus
#   5. Verify and configure PATH

set -e

# --- Config ---
REPO="softmata/horus"
BRANCH="release"

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# --- Platform detection ---
detect_os() {
    local uname_s
    uname_s="$(uname -s)"
    case "$uname_s" in
        Linux*)
            if grep -qi microsoft /proc/version 2>/dev/null; then
                echo "linux"  # WSL is still linux
            else
                echo "linux"
            fi
            ;;
        Darwin*)  echo "macos" ;;
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
    echo -e "  ${CYAN}Installing build dependencies for ${distro}...${NC}"

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
            echo -e "  ${YELLOW}Unknown distro '${distro}' — you may need to install build deps manually${NC}"
            echo "  Needed: gcc, pkg-config, openssl-dev, libudev-dev, alsa-dev, clang"
            ;;
    esac
}

install_rust() {
    echo -e "  ${CYAN}Installing Rust...${NC}"
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable 2>&1 | tail -1
    export PATH="$HOME/.cargo/bin:$PATH"
    # shellcheck disable=SC1091
    [ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
}

# --- Main ---
OS=$(detect_os)
ARCH=$(detect_arch)
INSTALL_DIR=$(find_install_dir)
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
    echo -e "${RED}  Unsupported platform: $(uname -s) $(uname -m)${NC}"
    echo "  Supported: Linux/macOS (amd64, arm64), Windows (via Git Bash/WSL)"
    exit 1
fi

if ! command -v curl &>/dev/null; then
    echo -e "${RED}  curl is required${NC}"
    exit 1
fi

# --- Step 1: Try pre-built binary from GitHub Releases ---
ASSET_NAME="horus-${OS}-${ARCH}"
if [ "$OS" = "windows" ]; then
    ASSET_EXT="zip"
else
    ASSET_EXT="tar.gz"
fi
RELEASE_URL="https://github.com/${REPO}/releases/latest/download/${ASSET_NAME}.${ASSET_EXT}"

echo -e "${CYAN}[1/3]${NC} Downloading horus..."

TMPDIR=$(mktemp -d)
HTTP_CODE=$(curl -fsSL -o "${TMPDIR}/${ASSET_NAME}.${ASSET_EXT}" -w "%{http_code}" "$RELEASE_URL" 2>/dev/null || echo "000")

if [ "$HTTP_CODE" = "200" ] && [ -s "${TMPDIR}/${ASSET_NAME}.${ASSET_EXT}" ]; then
    # Extract and install pre-built binary
    if [ "$OS" = "windows" ]; then
        unzip -q "${TMPDIR}/${ASSET_NAME}.zip" -d "$TMPDIR"
    else
        tar xzf "${TMPDIR}/${ASSET_NAME}.tar.gz" -C "$TMPDIR"
    fi
    chmod +x "${TMPDIR}/${BINARY_NAME}" 2>/dev/null || true
    mv "${TMPDIR}/${BINARY_NAME}" "${INSTALL_DIR}/${BINARY_NAME}"
    rm -rf "$TMPDIR"
    echo -e "${GREEN}[1/3]${NC} Downloaded pre-built binary"

else
    rm -rf "$TMPDIR"
    echo -e "${YELLOW}[1/3]${NC} No pre-built binary for ${OS}-${ARCH}, building from source..."
    echo ""

    # Ensure Rust is available
    if ! command -v cargo &>/dev/null; then
        install_rust
        if ! command -v cargo &>/dev/null; then
            echo -e "${RED}  Failed to install Rust. Install manually: https://rustup.rs${NC}"
            exit 1
        fi
    fi

    # Install system build dependencies
    if [ "$OS" = "linux" ] || [ "$OS" = "macos" ]; then
        install_build_deps
    fi

    # Clone release branch and build
    CLONE_DIR=$(mktemp -d)
    echo -e "  ${CYAN}Cloning release branch...${NC}"
    git clone --depth 1 --branch "$BRANCH" "https://github.com/${REPO}.git" "$CLONE_DIR" 2>&1 | tail -1

    echo -e "  ${CYAN}Building from source (this takes a few minutes)...${NC}"
    cd "$CLONE_DIR"
    cargo build --release -p horus_manager --no-default-features 2>&1 | tail -5

    # Install the binary
    if [ -f "target/release/${BINARY_NAME}" ]; then
        cp "target/release/${BINARY_NAME}" "${INSTALL_DIR}/${BINARY_NAME}"
        chmod +x "${INSTALL_DIR}/${BINARY_NAME}"
        echo -e "${GREEN}[1/3]${NC} Built and installed from source"
    else
        echo -e "${RED}  Build failed — binary not found${NC}"
        echo "  Build log: ${CLONE_DIR}"
        echo "  Report issues: https://github.com/${REPO}/issues"
        exit 1
    fi

    # Cleanup
    cd /
    rm -rf "$CLONE_DIR"
fi

# --- Step 2: Verify ---
echo -e "${CYAN}[2/3]${NC} Verifying installation..."

if [ -f "${INSTALL_DIR}/${BINARY_NAME}" ]; then
    VERSION=$("${INSTALL_DIR}/${BINARY_NAME}" --version 2>/dev/null || echo "installed")
    echo -e "${GREEN}[2/3]${NC} horus ${VERSION}"
else
    echo -e "${RED}[2/3]${NC} Installation failed"
    exit 1
fi

# --- Step 3: PATH ---
echo -e "${CYAN}[3/3]${NC} Checking PATH..."

if echo "$PATH" | grep -q "$INSTALL_DIR"; then
    echo -e "${GREEN}[3/3]${NC} ${INSTALL_DIR} already in PATH"
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
            echo -e "${GREEN}[3/3]${NC} Added to PATH in $(basename "$SHELL_RC")"
        else
            echo -e "${GREEN}[3/3]${NC} Already configured in $(basename "$SHELL_RC")"
        fi
    fi
    export PATH="${INSTALL_DIR}:$PATH"
fi

# --- Shell integration (cargo/pip/cmake proxy) ---
horus env --init 2>/dev/null || true

# --- Done ---
echo ""
echo -e "${GREEN}  Installation complete!${NC}"
echo ""
echo "  Get started:"
echo -e "    ${CYAN}horus new my_robot -r${NC}     Create a Rust project"
echo -e "    ${CYAN}horus new my_robot -p${NC}     Create a Python project"
echo -e "    ${CYAN}horus doctor${NC}              Check your environment"
echo ""
echo -e "  Docs: ${CYAN}https://docs.horusrobotics.dev${NC}"
echo ""

if ! command -v horus &>/dev/null; then
    echo -e "  ${YELLOW}Restart your terminal or run:${NC}"
    echo -e "    ${CYAN}source ~/${SHELL_RC##*/}${NC}"
    echo ""
fi
