#!/bin/bash
# restart_can.sh
# Kills all slcand processes and restarts on the first available USB serial port.

BITRATE=1000000
CAN_IFACE=can0

echo "Killing existing slcand processes..."
sudo killall slcand 2>/dev/null
sleep 0.5

echo "Bringing down $CAN_IFACE if up..."
sudo ip link set $CAN_IFACE down 2>/dev/null
sleep 0.5

# Look through all ttyACM devices
PORTS=$(ls /dev/ttyACM* 2>/dev/null)

if [ -z "$PORTS" ]; then
    echo "ERROR: No ttyACM devices found. Is the CAN adapter plugged in?"
    exit 1
fi

SUCCESS=false

for PORT in $PORTS; do
    echo "Trying $PORT..."

    sudo slcand -o -s8 -t hw -S $BITRATE $PORT $CAN_IFACE 2>/dev/null
    sleep 0.5

    sudo ip link set $CAN_IFACE up
    sleep 0.5

    # Check if interface came up cleanly
    STATE=$(ip -details link show $CAN_IFACE 2>/dev/null | grep -o "state [A-Z]*" | awk '{print $2}')

    if [ "$STATE" == "UP" ]; then
        echo ""
        echo "Success! CAN bus restarted on $PORT"
        echo "  Interface : $CAN_IFACE"
        echo "  Bitrate   : $BITRATE"
        echo "  State     : $STATE"
        SUCCESS=true
        break
    else
        echo "  $PORT did not bring up $CAN_IFACE, trying next..."
        sudo killall slcand 2>/dev/null
        sudo ip link set $CAN_IFACE down 2>/dev/null
        sleep 0.3
    fi
done

if [ "$SUCCESS" = false ]; then
    echo ""
    echo "ERROR: Could not start CAN bus on any USB port."
    echo "Tried: $PORTS"
    exit 1
fi