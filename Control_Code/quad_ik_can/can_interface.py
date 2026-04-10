"""
can_interface.py
================
CAN driver for the UCB Recoil FOC firmware running on STM32G431 B-G431B-ESC1.

Protocol (from motor_controller.c  MotorController_handleCANMessage)
----------------------------------------------------------------------
11-bit standard CAN ID:
    bits [10:7] = FUNC_ID   (4 bits)
    bits  [6:0] = DEVICE_ID (7 bits)

    CAN_ID = (FUNC_ID << 7) | DEVICE_ID

PDO2 command (FUNC_RECEIVE_PDO_2 = 0x04)  — used for position control
    TX:  8 bytes  [float32 position_target (rad), float32 velocity_target (rad/s)]
    RX:  8 bytes  [float32 position_measured (rad), float32 velocity_measured (rad/s)]

PDO3 command (FUNC_RECEIVE_PDO_3 = 0x05)
    TX:  8 bytes  [float32 position_target (rad), float32 torque_feedforward (Nm)]
    RX:  8 bytes  [float32 position_measured (rad), float32 torque_measured (Nm)]

NMT command (FUNC_NMT = 0x00)  — set operating mode
    TX:  2 bytes  [uint8 mode, uint8 device_id]
    RX:  none

HEARTBEAT (FUNC_HEARTBEAT = 0x08)  — resets watchdog
    TX:  0 bytes
    RX:  none

All floats are IEEE-754 single precision, little-endian (native STM32).
"""

from __future__ import annotations

import struct
import threading
import time
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

log = logging.getLogger(__name__)

try:
    import can
    _CAN_AVAILABLE = True
except ImportError:
    _CAN_AVAILABLE = False
    log.warning("python-can not installed — SIMULATION MODE (no hardware needed)")


# ── CAN ID helpers ────────────────────────────────────────────────────────────

def make_can_id(func_id: int, device_id: int) -> int:
    """Pack Recoil 11-bit CAN ID: (func_id << 7) | device_id."""
    return ((func_id & 0xF) << 7) | (device_id & 0x7F)


def split_can_id(can_id: int) -> tuple[int, int]:
    """Return (func_id, device_id) from an 11-bit Recoil CAN ID."""
    return (can_id >> 7) & 0xF, can_id & 0x7F


# ── Received telemetry frame ──────────────────────────────────────────────────

@dataclass
class JointTelemetry:
    """Decoded telemetry returned by PDO2 or PDO3."""
    device_id:         int   = 0
    position_rad:      float = 0.0
    velocity_rad_s:    float = 0.0
    torque_nm:         float = 0.0   # only valid for PDO3 replies
    timestamp:         float = field(default_factory=time.monotonic)


@dataclass
class RawFrame:
    """Raw received CAN frame for the log panel."""
    can_id:    int
    func_id:   int
    device_id: int
    data:      bytes
    timestamp: float = field(default_factory=time.monotonic)

    def hex_data(self) -> str:
        return " ".join(f"{b:02X}" for b in self.data)


# ── Main interface class ──────────────────────────────────────────────────────

class RecoilCAN:
    """
    Thread-safe CAN interface for Recoil FOC firmware.

    Usage
    -----
        bus = RecoilCAN()
        bus.connect()

        # Set all three joints to MODE_POSITION
        bus.set_mode(DEVICE_ID_HIP,   MODE_POSITION)
        bus.set_mode(DEVICE_ID_KNEE,  MODE_POSITION)
        bus.set_mode(DEVICE_ID_ANKLE, MODE_POSITION)

        # Send PDO2 position + velocity targets, get back telemetry
        telem = bus.send_pdo2(DEVICE_ID_HIP, position_rad=1.2, velocity_rad_s=0.0)

        # Keep ESCs alive
        bus.send_heartbeat(DEVICE_ID_HIP)

        bus.disconnect()
    """

    def __init__(
        self,
        interface: str   = "socketcan",
        channel:   str   = "can0",
        bitrate:   int   = 1_000_000,
        timeout:   float = 0.05,
    ):
        self._interface = interface
        self._channel   = channel
        self._bitrate   = bitrate
        self._timeout   = timeout

        self._bus: Optional[object] = None
        self._connected = False
        self._sim_mode  = not _CAN_AVAILABLE

        # RX storage
        self._rx_frames: deque[RawFrame]  = deque(maxlen=512)
        self._telemetry: dict[int, JointTelemetry] = {}   # keyed by device_id
        self._rx_lock   = threading.Lock()

        self._rx_thread: Optional[threading.Thread] = None
        self._stop_rx   = threading.Event()

        # Stats
        self.tx_count    = 0
        self.rx_count    = 0
        self.error_count = 0

    # ── Connection ─────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        if self._sim_mode:
            log.info("SIMULATION MODE — no CAN hardware required")
            self._connected = True
            return True
        try:
            self._bus = can.interface.Bus(
                interface=self._interface,
                channel=self._channel,
                bitrate=self._bitrate,
            )
            self._connected = True
            self._start_rx_thread()
            log.info(f"CAN connected: {self._channel} @ {self._bitrate} bps")
            return True
        except Exception as exc:
            log.error(f"CAN connect failed: {exc}")
            return False

    def disconnect(self):
        self._stop_rx.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
            self._bus = None
        self._connected = False
        log.info("CAN disconnected")

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def simulation_mode(self) -> bool:
        return self._sim_mode

    # ── NMT — set mode ─────────────────────────────────────────────────────────

    def set_mode(self, device_id: int, mode: int) -> bool:
        """
        Send NMT frame to set operating mode on one ESC.
        data[0] = mode byte, data[1] = device_id
        """
        can_id = make_can_id(0x00, 0x00)   # NMT always uses func=0, device=0
        payload = struct.pack("<BB", mode, device_id)
        return self._send(can_id, payload)

    # ── PDO2 — position + velocity ────────────────────────────────────────────

    def send_pdo2(
        self,
        device_id:       int,
        position_rad:    float,
        velocity_rad_s:  float = 0.0,
    ) -> Optional[JointTelemetry]:
        """
        Send PDO2 position command and return latest telemetry for this joint.

        Frame TX:  CAN_ID = (0x04 << 7) | device_id
                   Data   = [float32 position_target, float32 velocity_target]

        Frame RX:  CAN_ID = (0x0A << 7) | device_id
                   Data   = [float32 position_measured, float32 velocity_measured]
        """
        can_id  = make_can_id(0x04, device_id)
        payload = struct.pack("<ff", position_rad, velocity_rad_s)
        self._send(can_id, payload)
        return self._telemetry.get(device_id)

    # ── PDO3 — position + torque feedforward ──────────────────────────────────

    def send_pdo3(
        self,
        device_id:       int,
        position_rad:    float,
        torque_nm:       float = 0.0,
    ) -> Optional[JointTelemetry]:
        """
        Send PDO3 position + torque feedforward command.

        Frame TX:  CAN_ID = (0x05 << 7) | device_id
                   Data   = [float32 position_target, float32 torque_feedforward]
        """
        can_id  = make_can_id(0x05, device_id)
        payload = struct.pack("<ff", position_rad, torque_nm)
        self._send(can_id, payload)
        return self._telemetry.get(device_id)

    # ── Heartbeat ─────────────────────────────────────────────────────────────

    def send_heartbeat(self, device_id: int) -> bool:
        """Send FUNC_HEARTBEAT to reset watchdog on one ESC (0-byte payload)."""
        can_id = make_can_id(0x08, device_id)
        return self._send(can_id, b"")

    def send_heartbeat_all(self, device_ids: list[int]) -> None:
        for did in device_ids:
            self.send_heartbeat(did)

    # ── SDO — read/write any parameter ────────────────────────────────────────

    def sdo_write(self, device_id: int, byte_offset: int, value_float: float) -> bool:
        """
        SDO download: write a float32 at byte_offset into the MotorController struct.
        ccs = 1 (download), packed as:
            data[0]   = (ccs=1) << 5  = 0x20
            data[1:2] = uint16 byte_offset (little-endian)
            data[4:7] = float32 value   (little-endian)
        """
        can_id  = make_can_id(0x02, device_id)
        payload = struct.pack("<BHxf", 0x20, byte_offset, value_float)
        return self._send(can_id, payload)

    def sdo_read(self, device_id: int, byte_offset: int) -> bool:
        """
        SDO upload request: read float32 at byte_offset.
        ccs = 2 (upload), reply comes back as FUNC_TRANSMIT_SDO frame.
        """
        can_id  = make_can_id(0x02, device_id)
        payload = struct.pack("<BHx", 0x40, byte_offset)
        return self._send(can_id, payload)

    # ── Transmit ──────────────────────────────────────────────────────────────

    def _send(self, can_id: int, payload: bytes) -> bool:
        if self._sim_mode:
            func_id, dev_id = split_can_id(can_id)
            log.debug(
                f"[SIM] TX id=0x{can_id:03X} "
                f"func=0x{func_id:02X} dev={dev_id} "
                f"data={payload.hex().upper()}"
            )
            self.tx_count += 1
            return True

        if not self._connected or self._bus is None:
            return False

        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=payload,
                is_extended_id=False,
            )
            self._bus.send(msg, timeout=self._timeout)
            self.tx_count += 1
            return True
        except Exception as exc:
            self.error_count += 1
            log.error(f"CAN send error: {exc}")
            return False

    # ── Receive thread ────────────────────────────────────────────────────────

    def _start_rx_thread(self):
        self._stop_rx.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="CAN-RX"
        )
        self._rx_thread.start()

    def _rx_loop(self):
        while not self._stop_rx.is_set():
            try:
                msg = self._bus.recv(timeout=0.1)
                if msg is None:
                    continue

                func_id, device_id = split_can_id(msg.arbitration_id)
                raw = RawFrame(
                    can_id=msg.arbitration_id,
                    func_id=func_id,
                    device_id=device_id,
                    data=bytes(msg.data),
                    timestamp=msg.timestamp,
                )

                with self._rx_lock:
                    self._rx_frames.append(raw)
                    self.rx_count += 1

                    # Decode PDO2 / PDO4 telemetry replies
                    if func_id in (0x0A, 0x0C) and len(msg.data) >= 8:
                        pos, vel = struct.unpack_from("<ff", msg.data, 0)
                        t = self._telemetry.setdefault(device_id, JointTelemetry(device_id=device_id))
                        t.position_rad   = pos
                        t.velocity_rad_s = vel
                        t.timestamp      = msg.timestamp

                    # Decode PDO3 telemetry replies
                    elif func_id == 0x0B and len(msg.data) >= 8:
                        pos, torque = struct.unpack_from("<ff", msg.data, 0)
                        t = self._telemetry.setdefault(device_id, JointTelemetry(device_id=device_id))
                        t.position_rad = pos
                        t.torque_nm    = torque
                        t.timestamp    = msg.timestamp

            except Exception as exc:
                if not self._stop_rx.is_set():
                    log.warning(f"CAN RX error: {exc}")
                    self.error_count += 1
                    time.sleep(0.01)

    # ── Accessors ─────────────────────────────────────────────────────────────

    def get_telemetry(self, device_id: int) -> Optional[JointTelemetry]:
        with self._rx_lock:
            return self._telemetry.get(device_id)

    def get_recent_frames(self, n: int = 20) -> list[RawFrame]:
        with self._rx_lock:
            return list(self._rx_frames)[-n:]

    def flush(self):
        with self._rx_lock:
            self._rx_frames.clear()
            self._telemetry.clear()

    def status_string(self) -> str:
        mode  = "SIM" if self._sim_mode else self._channel
        state = "CONNECTED" if self._connected else "DISCONNECTED"
        return f"{mode} | {state} | TX:{self.tx_count} RX:{self.rx_count} ERR:{self.error_count}"
