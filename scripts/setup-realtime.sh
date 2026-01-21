#!/bin/bash
#
# HORUS Real-Time Setup Script
#
# This script configures your Linux system for real-time scheduling.
# Run with: sudo ./setup-realtime.sh
#
# What it does:
# 1. Creates 'realtime' group
# 2. Adds current user to the group
# 3. Sets up RT priority limits
# 4. Sets up memory lock limits
# 5. Optionally installs PREEMPT_RT kernel
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=============================================="
echo "  HORUS Real-Time System Setup"
echo "=============================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root (sudo)${NC}"
    echo "Usage: sudo $0"
    exit 1
fi

# Get the actual user (not root)
ACTUAL_USER="${SUDO_USER:-$USER}"
if [ "$ACTUAL_USER" = "root" ]; then
    echo -e "${YELLOW}Warning: Running as root user. RT limits will apply to root.${NC}"
    read -p "Enter username to configure (or press Enter for root): " ACTUAL_USER
    ACTUAL_USER="${ACTUAL_USER:-root}"
fi

echo "Configuring real-time for user: $ACTUAL_USER"
echo ""

# Step 1: Create realtime group
echo -n "Creating 'realtime' group... "
if getent group realtime > /dev/null 2>&1; then
    echo -e "${YELLOW}already exists${NC}"
else
    groupadd realtime
    echo -e "${GREEN}done${NC}"
fi

# Step 2: Add user to realtime group
echo -n "Adding $ACTUAL_USER to 'realtime' group... "
if id -nG "$ACTUAL_USER" | grep -qw "realtime"; then
    echo -e "${YELLOW}already member${NC}"
else
    usermod -a -G realtime "$ACTUAL_USER"
    echo -e "${GREEN}done${NC}"
fi

# Step 3: Create limits.conf for realtime group
LIMITS_FILE="/etc/security/limits.d/99-horus-realtime.conf"
echo -n "Creating RT limits config ($LIMITS_FILE)... "

cat > "$LIMITS_FILE" << 'EOF'
# HORUS Real-Time Limits Configuration
# Allows members of 'realtime' group to use RT scheduling
#
# Format: <domain> <type> <item> <value>

# Real-time priority: allow up to 99 (max)
@realtime    -    rtprio    99

# Memory locking: unlimited (required for RT to prevent page faults)
@realtime    -    memlock   unlimited

# Nice priority: allow full range
@realtime    -    nice      -20

# Max locked memory (alternative syntax)
@realtime    soft memlock   unlimited
@realtime    hard memlock   unlimited
EOF

echo -e "${GREEN}done${NC}"

# Step 4: Check current kernel
echo ""
echo "=============================================="
echo "  Kernel Check"
echo "=============================================="

KERNEL_VERSION=$(uname -r)
echo "Current kernel: $KERNEL_VERSION"

if uname -v | grep -qi "PREEMPT_RT"; then
    echo -e "${GREEN}PREEMPT_RT kernel detected!${NC}"
    PREEMPT_RT=true
elif [ -f /sys/kernel/realtime ]; then
    if [ "$(cat /sys/kernel/realtime)" = "1" ]; then
        echo -e "${GREEN}PREEMPT_RT kernel detected!${NC}"
        PREEMPT_RT=true
    fi
else
    echo -e "${YELLOW}Standard kernel (no PREEMPT_RT)${NC}"
    PREEMPT_RT=false
fi

# Step 5: Offer to install PREEMPT_RT kernel
if [ "$PREEMPT_RT" = "false" ]; then
    echo ""
    echo "For hard real-time guarantees, you need a PREEMPT_RT kernel."
    echo ""

    # Detect distro
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        DISTRO="$ID"
    else
        DISTRO="unknown"
    fi

    echo "Detected distribution: $DISTRO"

    case "$DISTRO" in
        ubuntu|debian)
            echo ""
            echo "To install PREEMPT_RT kernel on Ubuntu/Debian:"
            echo ""
            echo "  # Check available RT kernels"
            echo "  apt-cache search linux-image | grep -i rt"
            echo ""
            echo "  # Install (example for Ubuntu)"
            echo "  sudo apt install linux-image-rt-amd64"
            echo ""
            echo "  # Or for specific version"
            echo "  sudo apt install linux-image-6.1.0-rt-amd64"
            echo ""
            read -p "Would you like to search for available RT kernels now? [y/N] " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                echo ""
                echo "Available RT kernels:"
                apt-cache search linux-image | grep -i rt || echo "No RT kernels found in repositories"
            fi
            ;;
        fedora|rhel|centos)
            echo ""
            echo "To install PREEMPT_RT kernel on Fedora/RHEL:"
            echo ""
            echo "  # Enable RT repository (RHEL)"
            echo "  sudo subscription-manager repos --enable=rhel-8-for-x86_64-rt-rpms"
            echo ""
            echo "  # Install"
            echo "  sudo dnf install kernel-rt kernel-rt-devel"
            ;;
        arch)
            echo ""
            echo "To install PREEMPT_RT kernel on Arch:"
            echo ""
            echo "  # From AUR"
            echo "  yay -S linux-rt linux-rt-headers"
            echo ""
            echo "  # Or use linux-zen (has some RT features)"
            echo "  sudo pacman -S linux-zen linux-zen-headers"
            ;;
        *)
            echo ""
            echo "For other distributions, search for 'linux-rt' or 'PREEMPT_RT' packages,"
            echo "or build a custom kernel with the PREEMPT_RT patch:"
            echo "  https://wiki.linuxfoundation.org/realtime/start"
            ;;
    esac
fi

# Step 6: CPU isolation recommendations
echo ""
echo "=============================================="
echo "  CPU Isolation (Optional)"
echo "=============================================="
echo ""
echo "For best RT performance, isolate CPUs from the scheduler."
echo "Add to kernel command line (GRUB_CMDLINE_LINUX in /etc/default/grub):"
echo ""

CPU_COUNT=$(nproc)
if [ "$CPU_COUNT" -ge 4 ]; then
    ISOLATED=$((CPU_COUNT - 2))-$((CPU_COUNT - 1))
    echo "  isolcpus=$ISOLATED nohz_full=$ISOLATED rcu_nocbs=$ISOLATED"
    echo ""
    echo "This isolates CPUs $ISOLATED for RT tasks (you have $CPU_COUNT CPUs)."
else
    echo "  (Your system has only $CPU_COUNT CPUs - isolation not recommended)"
fi

# Summary
echo ""
echo "=============================================="
echo "  Setup Complete!"
echo "=============================================="
echo ""
echo -e "${GREEN}Real-time configuration applied.${NC}"
echo ""
echo "IMPORTANT: You must LOG OUT and LOG BACK IN for group changes to take effect."
echo ""
echo "To verify after re-login:"
echo "  # Check group membership"
echo "  groups"
echo ""
echo "  # Check RT limits"
echo "  ulimit -r    # Should show 99"
echo "  ulimit -l    # Should show 'unlimited'"
echo ""
echo "  # Run HORUS and check capabilities"
echo "  cargo run --example scheduler_test"
echo ""

# Create a verification script
VERIFY_SCRIPT="/tmp/verify-horus-rt.sh"
cat > "$VERIFY_SCRIPT" << 'EOF'
#!/bin/bash
echo "HORUS RT Verification"
echo "====================="
echo ""
echo "Group membership:"
groups
echo ""
echo "RT priority limit (should be 99):"
ulimit -r
echo ""
echo "Memory lock limit (should be unlimited):"
ulimit -l
echo ""
echo "Kernel RT status:"
if uname -v | grep -qi "PREEMPT_RT"; then
    echo "PREEMPT_RT: YES"
elif [ -f /sys/kernel/realtime ] && [ "$(cat /sys/kernel/realtime)" = "1" ]; then
    echo "PREEMPT_RT: YES"
else
    echo "PREEMPT_RT: NO (standard kernel)"
fi
EOF
chmod +x "$VERIFY_SCRIPT"
echo "Verification script created: $VERIFY_SCRIPT"
echo "Run it after re-login to verify the setup."
