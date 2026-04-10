# =============================================================================
#  QUAD LEG IK — CONFIGURATION
#  UCB Recoil FOC Firmware (STM32G431 B-G431B-ESC1)
#
#  Edit this file to match your hardware. All other files read from here.
# =============================================================================

import math

# -----------------------------------------------------------------------------
#  CAN BUS  (CANable via SocketCAN)
# -----------------------------------------------------------------------------
CAN_INTERFACE   = "socketcan"
CAN_CHANNEL     = "can0"        # run: sudo bash scripts/setup_can.sh
CAN_BITRATE     = 1_000_000     # must match FDCAN1 config in firmware
CAN_TIMEOUT     = 0.05

# -----------------------------------------------------------------------------
#  RECOIL FIRMWARE — CAN PROTOCOL
#
#  11-bit standard CAN ID layout:
#     bits [10:7]  = FUNC_ID   (4 bits, function code)
#     bits  [6:0]  = DEVICE_ID (7 bits, node address)
#
#  CAN ID = (FUNC_ID << 7) | DEVICE_ID
# -----------------------------------------------------------------------------
FUNC_NMT             = 0x00   # Network Management — set operating mode
FUNC_SYNC_EMCY       = 0x01   # Sync / emergency
FUNC_RECEIVE_SDO     = 0x02   # SDO read/write any parameter by byte offset
FUNC_RECEIVE_PDO_1   = 0x03   # PDO1 RX — echo test frame
FUNC_RECEIVE_PDO_2   = 0x04   # PDO2 RX — [position_target, velocity_target]
FUNC_RECEIVE_PDO_3   = 0x05   # PDO3 RX — [position_target, torque_feedfwd]
FUNC_RECEIVE_PDO_4   = 0x06   # PDO4 RX — fast-frame slot (TX-only from ESC)
FUNC_FLASH           = 0x07   # Flash store (0x01) / load (0x02)
FUNC_HEARTBEAT       = 0x08   # Resets watchdog timer on ESC

FUNC_TRANSMIT_PDO_1  = 0x09   # PDO1 TX — echo reply
FUNC_TRANSMIT_PDO_2  = 0x0A   # PDO2 TX — [position_measured, velocity_measured]
FUNC_TRANSMIT_PDO_3  = 0x0B   # PDO3 TX — [position_measured, torque_measured]
FUNC_TRANSMIT_PDO_4  = 0x0C   # PDO4 TX — fast-frame [position, velocity]
FUNC_TRANSMIT_SDO    = 0x0D   # SDO reply

# Operating mode bytes (sent in NMT frame data[0]) — must match firmware enum
MODE_DISABLED            = 0x00
MODE_CALIBRATION         = 0x01
MODE_IDLE                = 0x02
MODE_DAMPING             = 0x03
MODE_CURRENT             = 0x04
MODE_TORQUE              = 0x05
MODE_VELOCITY            = 0x06
MODE_POSITION            = 0x07   # ← used for IK leg control
MODE_VQD_OVERRIDE        = 0x08
MODE_VALPHABETA_OVERRIDE = 0x09
MODE_VABC_OVERRIDE       = 0x0A
MODE_DEBUG               = 0x0B

# -----------------------------------------------------------------------------
#  DEVICE IDs — one per joint
#  Each ESC must be flashed with a unique DEVICE_CAN_ID in firmware.
#  Check/set via SDO or by recompiling with DEVICE_CAN_ID defined.
# -----------------------------------------------------------------------------
DEVICE_ID_HIP   = 0x01   # hip   ESC CAN node
DEVICE_ID_KNEE  = 0x02   # knee  ESC CAN node
DEVICE_ID_ANKLE = 0x03   # ankle ESC CAN node

# -----------------------------------------------------------------------------
#  PDO SELECTION
#
#  PDO2 (recommended): send [position_target (rad), velocity_target (rad/s)]
#                      recv [position_measured (rad), velocity_measured (rad/s)]
#
#  PDO3:               send [position_target (rad), torque_feedforward (Nm)]
#                      recv [position_measured (rad), torque_measured (Nm)]
#
#  All values are IEEE-754 float32, little-endian, packed back-to-back.
#  DLC = 8 bytes for all PDO frames.
# -----------------------------------------------------------------------------
USE_PDO = 2                  # 2 or 3
VELOCITY_FEEDFORWARD = 0.0   # rad/s — sent in PDO2
TORQUE_FEEDFORWARD   = 0.0   # Nm    — sent in PDO3

# -----------------------------------------------------------------------------
#  LEG GEOMETRY  (mm)
#
#  Motor layout:
#    DEVICE_ID_HIP   -- SHOULDER tilt: swings leg laterally in/out
#    DEVICE_ID_KNEE  -- THIGH pitch:   swings thigh forward/back
#    DEVICE_ID_ANKLE -- SHIN via push-rod that runs through the thigh
#
#  L1 = thigh link  (shoulder/hip pivot -> knee pivot)
#  L2 = shin  link  (knee pivot -> foot contact point)
#  L3 = foot  stub  -- set 0 if no rigid foot extension
#
#  SHIN_IS_ABSOLUTE:
#    True  -- push-rod holds shin at an ABSOLUTE world-frame angle regardless
#             of thigh angle (typical parallelogram / push-rod behaviour).
#             Motor 3 command = shin angle from world +X axis.
#    False -- motor 3 command = shin angle RELATIVE to the thigh.
#  Start with True. Flip if the foot moves the wrong direction.
#
#  SHOULDER_TO_HIP_MM -- lateral distance between shoulder pivot and hip pivot.
# -----------------------------------------------------------------------------
L1 = 100.0   # mm -- thigh
L2 =  90.0   # mm -- shin
L3 =   0.0   # mm -- foot stub (0 = no foot link)

SHIN_IS_ABSOLUTE    = True
SHOULDER_TO_HIP_MM  = 0.0

# -----------------------------------------------------------------------------
#  JOINT LIMITS (radians)
#  Clamped before sending. GUI shows red bar when outside range.
# -----------------------------------------------------------------------------
SHOULDER_LIMIT_RAD = (-math.pi / 2,  math.pi / 2)  # -90° → 90° (tilt in/out)
HIP_LIMIT_RAD   = (-math.pi,  -math.pi / 2)    # -180° → -90°
KNEE_LIMIT_RAD  = (-math.pi,  -math.pi / 2)  #   -180° → -90°
ANKLE_LIMIT_RAD = (-math.pi / 2,  0)    # ±0°

# -----------------------------------------------------------------------------
#  ZERO-POSITION OFFSETS (radians) — added to IK angles before CAN send
#  Set these to null out mechanical home-position differences between
#  joints without changing any IK math.
# -----------------------------------------------------------------------------
HIP_OFFSET_RAD   = 0.0
KNEE_OFFSET_RAD  = 0.0
ANKLE_OFFSET_RAD = 0.0

# -----------------------------------------------------------------------------
#  WATCHDOG / HEARTBEAT
#  Recoil firmware sets MODE_DAMPING if no CAN traffic within watchdog_timeout.
#  The app sends FUNC_HEARTBEAT frames to keep the ESCs alive while connected.
# -----------------------------------------------------------------------------
WATCHDOG_TIMEOUT_MS   = 1000
HEARTBEAT_INTERVAL_MS = 200   # must be < WATCHDOG_TIMEOUT_MS

# -----------------------------------------------------------------------------
#  SEND RATE
# -----------------------------------------------------------------------------
SEND_RATE_HZ = 50    # max PDO frames / second when dragging

# -----------------------------------------------------------------------------
#  GUI
# -----------------------------------------------------------------------------
WINDOW_TITLE  = "Quad Leg IK — Recoil CAN Tester"
DARK_MODE     = True
PIXELS_PER_MM = 1.8
