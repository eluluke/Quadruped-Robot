# Quad Leg IK — CAN Tester
**STM32 BESC · CANable · SocketCAN · Python 3**

Interactive inverse kinematics visualiser that sends computed joint angles
over CAN bus to three STM32 BESC motor controllers — one per joint.

---

## Quick Start

### 1. Install dependencies

```bash
pip3 install python-can
# tkinter is usually built-in; if not:
sudo apt-get install python3-tk
```

### 2. Bring up the CANable

Plug in your CANable USB adapter, then:

```bash
sudo bash scripts/setup_can.sh          # default: /dev/ttyACM0
# or if your device is on a different port:
sudo bash scripts/setup_can.sh /dev/ttyACM1
```

This creates the `can0` SocketCAN interface at 1 Mbit/s.

### 3. Run the app

```bash
python3 main.py
```

---

## Files

```
quad_ik_can/
├── config.py          ← ALL tunable parameters (edit this first)
├── ik_solver.py       ← Pure IK math (no hardware dependency)
├── can_interface.py   ← python-can wrapper for STM32 BESC
├── main.py            ← tkinter GUI application
└── scripts/
    └── setup_can.sh   ← CANable bringup script
```

---

## Configuration  (`config.py`)

### Link Lengths
```python
L1 = 100.0   # mm — upper arm (hip → knee)
L2 =  90.0   # mm — lower arm (knee → ankle)
L3 =  70.0   # mm — foot link (ankle → toe)
```
Edit these (or use the GUI panel) once you have your measurements.

### CAN IDs
```python
CAN_ID_HIP   = 0x01
CAN_ID_KNEE  = 0x02
CAN_ID_ANKLE = 0x03
```
Also editable live in the GUI without restarting.

### Joint Limits
```python
HIP_LIMIT_DEG   = (-90.0,  90.0)
KNEE_LIMIT_DEG  = (  0.0, 150.0)
ANKLE_LIMIT_DEG = (-90.0,  90.0)
```
Angles outside these ranges are clamped before transmission and shown
in red in the GUI.

### Angle Offsets
```python
HIP_OFFSET_DEG   = 0.0
KNEE_OFFSET_DEG  = 0.0
ANKLE_OFFSET_DEG = 0.0
```
Added to each IK angle before sending. Use to zero mechanical offsets.

---

## CAN Frame Format (STM32 BESC)

| Field           | Value                                    |
|-----------------|------------------------------------------|
| Arbitration ID  | Joint CAN ID (0x01 / 0x02 / 0x03)       |
| DLC             | 4 bytes                                  |
| Data            | IEEE-754 float32, little-endian, degrees |

Example — hip at 30.0°:
```
ID: 0x001   DLC: 4   Data: 00 00 F0 41
```

---

## Simulation Mode

If `python-can` is not installed **or** the CAN interface is not available,
the app runs in **SIMULATION MODE** — all IK math runs normally, angles
are displayed and logged, but no frames are sent. The CAN log shows
`[SIM]` and the status indicator turns orange.

---

## Parallelogram Linkage Note

The IK solver treats the leg as a standard 3-DOF serial chain. For a true
parallelogram linkage the actuated joint angle equals the effective link
angle (transmission ratio = 1.0). If your linkage has a different ratio set:

```python
KNEE_TRANSMISSION_RATIO = 1.0   # in config.py
```

The knee angle sent over CAN is multiplied by this value.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `can0: No such device` | Run `setup_can.sh` first |
| `Permission denied /dev/ttyACM0` | `sudo usermod -aG dialout $USER` then re-login |
| Motors don't move | Check CAN IDs match firmware; verify bitrate |
| Wrong angle direction | Flip sign with offset: `HIP_OFFSET_DEG = 0` + negate in `can_interface.py` |
| GUI doesn't open | `sudo apt-get install python3-tk` |
