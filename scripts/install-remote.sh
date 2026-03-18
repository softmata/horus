#!/bin/bash
# HORUS One-Line Installer
#
# Usage:
#   curl -fsSL https://horus.dev/install | bash
#   curl -fsSL https://raw.githubusercontent.com/softmata/horus/main/scripts/install-remote.sh | bash
#
# What this does:
#   1. Detects your OS and architecture
#   2. Downloads the latest pre-built binary from GitHub Releases
#   3. Installs to ~/.cargo/bin/horus (or ~/.local/bin/horus if no cargo)
#   4. Verifies the installation
#
# For a full source build with all features, use:
#   git clone https://github.com/softmata/horus && cd horus && ./install.sh

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

REPO="softmata/horus"
BINARY_NAME="horus"

# Detect OS
detect_os() {
    case "$(uname -s)" in
        Linux*)   echo "linux" ;;
        Darwin*)  echo "macos" ;;
        MINGW*|MSYS*|CYGWIN*) echo "windows" ;;
        *) echo "unknown" ;;
    esac
}

# Detect architecture
detect_arch() {
    case "$(uname -m)" in
        x86_64|amd64)  echo "x86_64" ;;
        aarch64|arm64) echo "aarch64" ;;
        armv7l)        echo "armv7" ;;
        *) echo "unknown" ;;
    esac
}

# Find install directory
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

OS=$(detect_os)
ARCH=$(detect_arch)
INSTALL_DIR=$(find_install_dir)

echo -e "${CYAN}"
echo "  ╦ ╦╔═╗╦═╗╦ ╦╔═╗"
echo "  ╠═╣║ ║╠╦╝║ ║╚═╗"
echo "  ╩ ╩╚═╝╩╚═╚═╝╚═╝"
echo -e "${NC}"
echo "  Deterministic Real-Time Robotics Framework"
echo ""
echo -e "  OS:      ${GREEN}${OS}${NC}"
echo -e "  Arch:    ${GREEN}${ARCH}${NC}"
echo -e "  Install: ${GREEN}${INSTALL_DIR}${NC}"
echo ""

if [ "$OS" = "unknown" ] || [ "$ARCH" = "unknown" ]; then
    echo -e "${RED}Error: Unsupported platform: $(uname -s) $(uname -m)${NC}"
    echo "Supported: Linux/macOS (x86_64, aarch64), Windows (via Git Bash)"
    exit 1
fi

# Check for required tools
if ! command -v curl &>/dev/null && ! command -v wget &>/dev/null; then
    echo -e "${RED}Error: curl or wget required${NC}"
    exit 1
fi

# Construct download URL
# Try GitHub Releases first, fall back to cargo install
ASSET_NAME="horus-${OS}-${ARCH}"
[ "$OS" = "windows" ] && ASSET_NAME="${ASSET_NAME}.exe"

RELEASE_URL="https://github.com/${REPO}/releases/latest/download/${ASSET_NAME}"

echo -e "${CYAN}[1/3]${NC} Downloading horus..."

# Try pre-built binary from GitHub Releases
TMPFILE=$(mktemp)
HTTP_CODE=$(curl -fsSL -o "$TMPFILE" -w "%{http_code}" "$RELEASE_URL" 2>/dev/null || echo "000")

if [ "$HTTP_CODE" = "200" ] && [ -s "$TMPFILE" ]; then
    # Pre-built binary found
    chmod +x "$TMPFILE"
    mv "$TMPFILE" "${INSTALL_DIR}/${BINARY_NAME}"
    echo -e "${GREEN}[1/3]${NC} Downloaded pre-built binary"
else
    rm -f "$TMPFILE"
    echo -e "${YELLOW}[1/3]${NC} No pre-built binary available, building from source..."

    # Fall back to cargo install
    if ! command -v cargo &>/dev/null; then
        echo -e "${CYAN}[1/3]${NC} Installing Rust first..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
        export PATH="$HOME/.cargo/bin:$PATH"
    fi

    # Install system dependencies
    if [ "$OS" = "linux" ]; then
        if command -v apt-get &>/dev/null; then
            echo -e "${CYAN}[1/3]${NC} Installing build dependencies..."
            sudo apt-get update -qq && sudo apt-get install -y -qq \
                gcc libc6-dev pkg-config libssl-dev libudev-dev libasound2-dev libclang-dev >/dev/null 2>&1
        elif command -v dnf &>/dev/null; then
            sudo dnf install -y -q gcc pkg-config openssl-devel systemd-devel alsa-lib-devel clang-devel >/dev/null 2>&1
        elif command -v pacman &>/dev/null; then
            sudo pacman -Sy --noconfirm gcc pkg-config openssl alsa-lib clang >/dev/null 2>&1
        fi
    elif [ "$OS" = "macos" ]; then
        if command -v brew &>/dev/null; then
            brew install pkg-config openssl >/dev/null 2>&1 || true
        fi
    fi

    echo -e "${CYAN}[1/3]${NC} Building from source (this takes a few minutes)..."
    cargo install --git "https://github.com/${REPO}" horus_manager --no-default-features --locked 2>&1 | tail -3
fi

# Verify installation
echo -e "${CYAN}[2/3]${NC} Verifying installation..."

if [ -f "${INSTALL_DIR}/${BINARY_NAME}" ] || command -v horus &>/dev/null; then
    VERSION=$(horus --version 2>/dev/null || "${INSTALL_DIR}/${BINARY_NAME}" --version 2>/dev/null || echo "unknown")
    echo -e "${GREEN}[2/3]${NC} horus installed: ${VERSION}"
else
    echo -e "${RED}[2/3]${NC} Installation failed — binary not found"
    exit 1
fi

# Ensure install dir is in PATH
echo -e "${CYAN}[3/3]${NC} Checking PATH..."

if ! echo "$PATH" | grep -q "$INSTALL_DIR"; then
    SHELL_RC=""
    case "$SHELL" in
        */zsh)  SHELL_RC="$HOME/.zshrc" ;;
        */bash) SHELL_RC="$HOME/.bashrc" ;;
        */fish) SHELL_RC="$HOME/.config/fish/config.fish" ;;
    esac

    if [ -n "$SHELL_RC" ] && ! grep -q "$INSTALL_DIR" "$SHELL_RC" 2>/dev/null; then
        echo "export PATH=\"${INSTALL_DIR}:\$PATH\"" >> "$SHELL_RC"
        echo -e "${GREEN}[3/3]${NC} Added ${INSTALL_DIR} to PATH in $(basename $SHELL_RC)"
    fi
fi

echo ""
echo -e "${GREEN}  Installation complete!${NC}"
echo ""
echo "  Get started:"
echo -e "    ${CYAN}horus new my_robot -r${NC}     Create a new Rust project"
echo -e "    ${CYAN}horus new my_robot -p${NC}     Create a new Python project"
echo -e "    ${CYAN}horus doctor${NC}              Check your environment"
echo ""
echo -e "  Documentation: ${CYAN}https://docs.horusrobotics.dev${NC}"
echo ""

# Restart shell hint if PATH was modified
if ! command -v horus &>/dev/null; then
    echo -e "  ${YELLOW}Note:${NC} Restart your terminal or run:"
    echo -e "    ${CYAN}source ~/${SHELL_RC##*/}${NC}"
    echo ""
fi
