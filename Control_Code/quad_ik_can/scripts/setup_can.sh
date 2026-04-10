#!/usr/bin/env bash
# =============================================================================
#  setup_can.sh  —  Bring up the CANable as a SocketCAN interface
#
#  Run ONCE before starting main.py:
#      sudo bash setup_can.sh
#
#  The CANable enumerates as /dev/ttyACM0 (or ttyACM1 etc.).
#  slcand bridges it to a socketcan network interface (can0).
# =============================================================================

set -e

BITRATE=1000000          # must match config.py CAN_BITRATE
DEVICE=${1:-/dev/ttyACM0}   # pass your ttyACMx as first argument if different
IFACE=can0

echo "──────────────────────────────────────────"
echo "  Quad Leg IK — CANable Setup"
echo "  Device  : $DEVICE"
echo "  Bitrate : $BITRATE bps"
echo "  iface   : $IFACE"
echo "──────────────────────────────────────────"

# Check device exists
if [ ! -c "$DEVICE" ]; then
    echo "ERROR: $DEVICE not found. Check USB connection."
    echo "  Available ttyACM* devices:"
    ls /dev/ttyACM* 2>/dev/null || echo "    (none found)"
    exit 1
fi

# Install slcan utils if missing
if ! command -v slcand &>/dev/null; then
    echo "Installing can-utils..."
    apt-get install -y can-utils
fi

# Tear down any existing interface
if ip link show $IFACE &>/dev/null; then
    echo "Tearing down existing $IFACE..."
    ip link set $IFACE down 2>/dev/null || true
    slcan_attach -c /dev/$IFACE 2>/dev/null || true
fi

# Kill any stale slcand
pkill slcand 2>/dev/null || true
sleep 0.3

# Start slcand (CANable USB → SocketCAN)
echo "Starting slcand on $DEVICE..."
slcand -o -c -f -s8 $DEVICE $IFACE
sleep 0.5

# Bring up the interface
ip link set $IFACE up type can bitrate $BITRATE
ip link set $IFACE txqueuelen 1000

echo ""
echo "✓  $IFACE is up at $BITRATE bps"
echo ""
echo "Verify with:  ip -details link show $IFACE"
echo "Monitor bus:  candump $IFACE"
echo ""
echo "Now run:  python3 main.py"
