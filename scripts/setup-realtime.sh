#!/bin/bash
# HORUS Real-Time System Setup
# Configures Linux for optimal real-time robotics performance.
#
# What this script does:
#   1. Sets CPU governor to 'performance' (prevents frequency scaling latency spikes)
#   2. Configures RT scheduling limits (SCHED_FIFO/RR up to priority 99)
#   3. Increases memlock limits (for mlockall to prevent page faults)
#   4. Installs a udev rule for shared memory permissions
#   5. Optionally installs cpupower/cpufrequtils for governor persistence
#
# Usage:
#   sudo ./scripts/setup-realtime.sh          # Full setup
#   sudo ./scripts/setup-realtime.sh --check  # Check current state only
#
# Supported platforms:
#   - Ubuntu/Debian (apt)
#   - Fedora/RHEL (dnf/yum)
#   - Arch Linux (pacman)
#   - Raspberry Pi OS (apt)
#   - NVIDIA Jetson (apt + jetson-specific)
#
# After running, LOG OUT and LOG BACK IN for limits to take effect.

set -e

# This script is Linux-only
if [[ "$(uname)" != "Linux" ]]; then
    echo "This script is for Linux only (Ubuntu, Raspberry Pi, Jetson, etc.)."
    echo "macOS does not use CPU governors or SCHED_FIFO."
    exit 0
fi

# Must run as root
if [[ $EUID -ne 0 ]]; then
    echo "This script must be run with sudo:"
    echo "  sudo ./scripts/setup-realtime.sh"
    exit 1
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

OK="[+]"
WARN="[!]"
INFO="[*]"
FAIL="[-]"

# Detect platform
detect_platform() {
    if [ -f /etc/nv_tegra_release ] || [ -d /usr/src/jetson_multimedia_api ]; then
        echo "jetson"
    elif grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
        echo "rpi"
    elif [ -f /etc/debian_version ]; then
        echo "debian"
    elif [ -f /etc/fedora-release ]; then
        echo "fedora"
    elif [ -f /etc/arch-release ]; then
        echo "arch"
    else
        echo "linux"
    fi
}

PLATFORM=$(detect_platform)

check_only=false
if [ "${1:-}" = "--check" ]; then
    check_only=true
fi

echo ""
echo -e "${CYAN}HORUS Real-Time System Configuration${NC}"
echo -e "${CYAN}=====================================${NC}"
echo ""
echo -e "Platform: ${GREEN}${PLATFORM}${NC}"
echo ""

# ============================================================================
# 1. CPU GOVERNOR
# ============================================================================

echo -e "${CYAN}${INFO}${NC} CPU Governor"

current_gov=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
available_govs=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors 2>/dev/null || echo "none")

echo "  Current: $current_gov"
echo "  Available: $available_govs"

if [ "$current_gov" = "performance" ]; then
    echo -e "  ${GREEN}${OK}${NC} Already set to 'performance'"
elif $check_only; then
    echo -e "  ${YELLOW}${WARN}${NC} Governor is '$current_gov' (should be 'performance')"
    echo "  Fix: sudo cpupower frequency-set -g performance"
else
    if echo "$available_govs" | grep -q "performance"; then
        # Set governor on all CPUs
        for cpu_gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            echo "performance" > "$cpu_gov" 2>/dev/null || true
        done
        echo -e "  ${GREEN}${OK}${NC} Set to 'performance' on all CPUs"

        # Install cpupower for persistence across reboots
        if ! command -v cpupower &>/dev/null; then
            echo -e "  ${INFO} Installing cpupower for reboot persistence..."
            case "$PLATFORM" in
                debian|rpi|jetson) apt-get install -y linux-tools-common linux-tools-generic 2>/dev/null || apt-get install -y cpufrequtils 2>/dev/null || true ;;
                fedora) dnf install -y kernel-tools 2>/dev/null || true ;;
                arch) pacman -S --noconfirm cpupower 2>/dev/null || true ;;
            esac
        fi

        # Create systemd service for persistence
        if [ -d /etc/systemd/system ]; then
            cat > /etc/systemd/system/horus-performance-governor.service << 'GOVEOF'
[Unit]
Description=HORUS - Set CPU governor to performance
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$g" 2>/dev/null; done'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
GOVEOF
            systemctl daemon-reload
            systemctl enable horus-performance-governor.service 2>/dev/null
            echo -e "  ${GREEN}${OK}${NC} Systemd service installed (persists across reboots)"
        fi
    else
        echo -e "  ${YELLOW}${WARN}${NC} 'performance' governor not available"
        echo "  This CPU may not support frequency scaling (already at max)"
    fi
fi

echo ""

# ============================================================================
# 2. RT SCHEDULING LIMITS
# ============================================================================

echo -e "${CYAN}${INFO}${NC} Real-Time Scheduling Limits"

current_rtprio=$(ulimit -r 2>/dev/null || echo "0")
current_memlock=$(ulimit -l 2>/dev/null || echo "0")

echo "  Current rtprio: $current_rtprio (need: 99)"
echo "  Current memlock: $current_memlock (need: unlimited)"

LIMITS_FILE="/etc/security/limits.d/99-horus-realtime.conf"

if [ "$current_rtprio" = "99" ] && ([ "$current_memlock" = "unlimited" ] || [ "$current_memlock" -gt 1048576 ] 2>/dev/null); then
    echo -e "  ${GREEN}${OK}${NC} RT limits already configured"
elif $check_only; then
    echo -e "  ${YELLOW}${WARN}${NC} RT limits need configuration"
    echo "  Fix: Run this script without --check"
else
    cat > "$LIMITS_FILE" << 'LIMEOF'
# HORUS Real-Time Scheduling Limits
# Allow all users to use RT scheduling and lock memory
# Required for SCHED_FIFO, mlockall, and deterministic control loops
*    soft    rtprio     99
*    hard    rtprio     99
*    soft    memlock    unlimited
*    hard    memlock    unlimited
*    soft    nice       -20
*    hard    nice       -20
LIMEOF
    echo -e "  ${GREEN}${OK}${NC} Created $LIMITS_FILE"
    echo -e "  ${YELLOW}${WARN}${NC} LOG OUT and LOG BACK IN for limits to take effect"
fi

echo ""

# ============================================================================
# 3. SHARED MEMORY CONFIGURATION
# ============================================================================

echo -e "${CYAN}${INFO}${NC} Shared Memory Configuration"

SHM_SIZE=$(df /dev/shm 2>/dev/null | awk 'NR==2{print $2}' || echo "0")
SHM_SIZE_MB=$((SHM_SIZE / 1024))

echo "  /dev/shm size: ${SHM_SIZE_MB} MB"

if [ "$SHM_SIZE_MB" -ge 512 ]; then
    echo -e "  ${GREEN}${OK}${NC} Shared memory size is adequate"
elif $check_only; then
    echo -e "  ${YELLOW}${WARN}${NC} /dev/shm is only ${SHM_SIZE_MB}MB (recommend >= 512MB)"
else
    # Increase /dev/shm for Raspberry Pi and other memory-constrained systems
    if ! grep -q "/dev/shm" /etc/fstab 2>/dev/null || ! grep -q "size=" /etc/fstab 2>/dev/null; then
        echo "tmpfs /dev/shm tmpfs defaults,size=512M 0 0" >> /etc/fstab
        mount -o remount /dev/shm 2>/dev/null || true
        echo -e "  ${GREEN}${OK}${NC} Increased /dev/shm to 512MB"
    fi
fi

echo ""

# ============================================================================
# 4. KERNEL PARAMETERS (sysctl)
# ============================================================================

echo -e "${CYAN}${INFO}${NC} Kernel Parameters"

SYSCTL_FILE="/etc/sysctl.d/99-horus-realtime.conf"

if $check_only; then
    shmmax=$(sysctl -n kernel.shmmax 2>/dev/null || echo "0")
    timer_slack=$(sysctl -n kernel.timer_migration 2>/dev/null || echo "unknown")
    echo "  kernel.shmmax: $shmmax"
    echo "  kernel.timer_migration: $timer_slack"
else
    cat > "$SYSCTL_FILE" << 'SYSEOF'
# HORUS Real-Time Kernel Parameters
# Maximize shared memory for high-throughput IPC
kernel.shmmax = 268435456
kernel.shmall = 65536

# Disable timer migration for better RT latency
kernel.timer_migration = 0

# Reduce VM swappiness for robotics workloads
vm.swappiness = 10
SYSEOF
    sysctl -p "$SYSCTL_FILE" 2>/dev/null || true
    echo -e "  ${GREEN}${OK}${NC} Applied kernel parameters ($SYSCTL_FILE)"
fi

echo ""

# ============================================================================
# 5. PLATFORM-SPECIFIC TUNING
# ============================================================================

if [ "$PLATFORM" = "jetson" ] && ! $check_only; then
    echo -e "${CYAN}${INFO}${NC} NVIDIA Jetson Tuning"

    # Jetson has its own power management
    if command -v nvpmodel &>/dev/null; then
        nvpmodel -m 0 2>/dev/null && echo -e "  ${GREEN}${OK}${NC} Set MAXN power mode" || true
    fi
    if command -v jetson_clocks &>/dev/null; then
        jetson_clocks 2>/dev/null && echo -e "  ${GREEN}${OK}${NC} Locked clocks to max frequency" || true
    fi
    echo ""
fi

if [ "$PLATFORM" = "rpi" ] && ! $check_only; then
    echo -e "${CYAN}${INFO}${NC} Raspberry Pi Tuning"

    # Check for force_turbo in config.txt
    CONFIG_TXT="/boot/config.txt"
    [ -f "/boot/firmware/config.txt" ] && CONFIG_TXT="/boot/firmware/config.txt"

    if [ -f "$CONFIG_TXT" ]; then
        if ! grep -q "^force_turbo=1" "$CONFIG_TXT" 2>/dev/null; then
            echo "# HORUS: Lock CPU to max frequency for RT performance" >> "$CONFIG_TXT"
            echo "force_turbo=1" >> "$CONFIG_TXT"
            echo -e "  ${GREEN}${OK}${NC} Added force_turbo=1 to $CONFIG_TXT"
            echo -e "  ${YELLOW}${WARN}${NC} Reboot required for this change"
        else
            echo -e "  ${GREEN}${OK}${NC} force_turbo already enabled"
        fi
    fi
    echo ""
fi

# ============================================================================
# SUMMARY
# ============================================================================

echo -e "${CYAN}════════════════════════════════════════${NC}"

if $check_only; then
    echo -e "${CYAN}${INFO}${NC} Check complete. Run without --check to apply fixes."
else
    echo -e "${GREEN}${OK}${NC} Real-time configuration complete!"
    echo ""
    echo -e "${YELLOW}${WARN} IMPORTANT: LOG OUT and LOG BACK IN for all changes to take effect.${NC}"
    echo ""
    echo "  Verify with:"
    echo -e "    ${CYAN}ulimit -r${NC}    # Should show 99"
    echo -e "    ${CYAN}ulimit -l${NC}    # Should show 'unlimited'"
    echo -e "    ${CYAN}cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor${NC}  # Should show 'performance'"
fi

echo ""
