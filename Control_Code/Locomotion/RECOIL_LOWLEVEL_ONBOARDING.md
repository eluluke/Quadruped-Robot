# Berkeley Humanoid Lite Recoil Low-Level Onboarding

This document introduces the motor-control functions available from the
Berkeley Humanoid Lite Recoil low-level package and explains how we use them in
our quadruped robot software.

The package is useful to us because it gives a clean software bridge from
Python or C++ to the STM32 B-G431B-ESC1 motor controllers over Linux SocketCAN.
In our project, this is the lowest software layer above the CAN bus: it sends
joint mode commands, writes gains and limits, reads motor feedback, and streams
position targets during walking.

Source reference: https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel

Reading note: this file intentionally uses short paragraphs and separated code
examples so new members can scan it quickly during robot testing.

## Big Picture

Our quadruped code currently uses the Python API:

```python
import berkeley_humanoid_lite_lowlevel.recoil as recoil

bus = recoil.Bus(channel="can2", bitrate=1000000)
bus.set_mode(6, recoil.Mode.POSITION)
bus.feed(6)
bus.write_position_kp(6, 0.24)
bus.write_position_kd(6, 0.010)
bus.write_torque_limit(6, 1.30)
bus.transmit_pdo_2(6, position_target, 0.0)
bus.set_mode(6, recoil.Mode.IDLE)
bus.stop()
```

The same idea exists in C++ through `SocketCan` and `MotorController`:

```cpp
SocketCan bus;
bus.open("can2");

MotorController motor(&bus, 6);
motor.set_mode(MODE_POSITION);
motor.feed();
motor.write_position_kp(0.24);
motor.write_position_kd(0.010);
motor.write_torque_limit(1.30);
motor.set_target_position(position_target);
motor.set_target_velocity(0.0);
motor.write_pdo_2();
motor.set_mode(MODE_IDLE);
bus.close();
```

For our robot, each leg owns one CAN channel and three motors:

| Leg | CAN channel | Joint IDs |
| --- | --- | --- |
| `front_right` | `can1` | hip `3`, thigh `2`, shank `1` |
| `front_left` | `can2` | hip `6`, thigh `5`, shank `4` |
| `rear_right` | `can3` | hip `9`, thigh `8`, shank `7` |
| `rear_left` | `can4` | hip `12`, thigh `11`, shank `10` |

The low-level package does not know our robot geometry. It only knows motor
IDs, CAN frames, controller modes, gains, limits, and target values. Our files
such as `leg_config.py`, `leg_controller.py`, `robot_controller.py`, and
`trajectory_config.py` convert robot-level foot positions into motor raw target
positions before calling Recoil.

## Communication Types

The package uses two main CAN communication styles.

`SDO` style messages read or write configuration parameters. These are slower
and are good for setup: gains, torque limits, encoder parameters, gear ratio,
and firmware configuration.

`PDO` style messages are fast runtime process data. For walking, we mainly use
PDO 2 to send `position_target` and `velocity_target`, and optionally receive
`position_measured` and `velocity_measured`.

In simple words: use SDO before motion to configure the controller, then use
PDO repeatedly inside the control loop.

## Python API

Import:

```python
import berkeley_humanoid_lite_lowlevel.recoil as recoil
```

### Bus Setup

| Function | Purpose | Quadruped use |
| --- | --- | --- |
| `recoil.Bus(channel, bitrate=1000000)` | Opens a SocketCAN interface through `python-can`. | One bus per leg, such as `can1`, `can2`, `can3`, `can4`. |
| `bus.stop()` | Shuts down the Python CAN bus object. | Called during clean shutdown after sending motors to safe mode. |
| `bus.receive(filter_device_id=None, filter_function=None, timeout=None)` | Waits for a CAN frame and optionally filters by motor ID and function ID. | Useful for debugging, but most robot motion uses helper methods instead. |
| `bus.transmit(frame)` | Sends a raw `CANFrame`. | Normally wrapped by higher-level functions like `set_mode()` and `transmit_pdo_2()`. |
| `recoil.CANFrame(device_id, func_id, size, data)` | Raw frame container used by `transmit()`. | Advanced debugging only. |

Example:

```python
bus = recoil.Bus(channel="can3", bitrate=1000000)
```

### Device Health And Modes

| Function | Purpose | Quadruped use |
| --- | --- | --- |
| `bus.ping(device_id, timeout=0.1)` | Sends a small ping frame and checks whether the controller answers. | Quick check that a motor ID is online. |
| `bus.feed(device_id)` | Sends a heartbeat/watchdog feed frame. | Called before and during active position control so the controller stays alive. |
| `bus.set_mode(device_id, recoil.Mode.IDLE)` | Puts the motor in unpowered idle. | Emergency shutdown and final stop. |
| `bus.set_mode(device_id, recoil.Mode.DAMPING)` | Passive damping mode. | Useful as a softer stop if we choose to use it. |
| `bus.set_mode(device_id, recoil.Mode.CALIBRATION)` | Firmware calibration mode. | Used only during motor electrical calibration, not normal walking. |
| `bus.set_mode(device_id, recoil.Mode.POSITION)` | Closed-loop position mode. | Main mode for homing, standing, and trotting. |
| `bus.set_mode(device_id, recoil.Mode.CURRENT / TORQUE / VELOCITY)` | Other closed-loop modes. | Not used in our current walking code. |

Available Python mode constants:

```python
recoil.Mode.DISABLED
recoil.Mode.IDLE
recoil.Mode.DAMPING
recoil.Mode.CALIBRATION
recoil.Mode.CURRENT
recoil.Mode.TORQUE
recoil.Mode.VELOCITY
recoil.Mode.POSITION
recoil.Mode.VABC_OVERRIDE
recoil.Mode.VALPHABETA_OVERRIDE
recoil.Mode.VQD_OVERRIDE
recoil.Mode.DEBUG
```

For our robot, the safest shutdown pattern is:

```python
for motor_id in motor_ids:
    bus.set_mode(motor_id, recoil.Mode.IDLE)
bus.stop()
```

### Flash Settings

| Function | Purpose | Caution |
| --- | --- | --- |
| `bus.load_settings_from_flash(device_id)` | Loads controller parameters stored in flash. | Use when restoring saved motor configuration. |
| `bus.store_settings_to_flash(device_id)` | Saves current controller parameters to flash. | Do not call repeatedly in a loop; flash has limited write life. |

Most walking code should not write flash. Runtime gains can be sent after boot
without permanently changing the controller.

### Generic Parameter Access

These are lower-level helpers. They operate using parameter IDs from
`recoil.Parameter`.

| Function | Purpose |
| --- | --- |
| `bus._read_parameter(device_id, param_id, timeout=None)` | Reads a raw parameter frame. |
| `bus._write_parameter(device_id, param_id, tx_data)` | Writes raw bytes to a parameter. |
| `bus._read_parameter_bytes(device_id, param_id, timeout=None)` | Reads 4 raw bytes. |
| `bus._read_parameter_f32(device_id, param_id, timeout=None)` | Reads a 32-bit float. |
| `bus._read_parameter_i32(device_id, param_id, timeout=None)` | Reads a signed 32-bit integer. |
| `bus._read_parameter_u32(device_id, param_id, timeout=None)` | Reads an unsigned 32-bit integer. |
| `bus._write_parameter_bytes(device_id, param_id, value)` | Writes raw bytes. |
| `bus._write_parameter_f32(device_id, param_id, value)` | Writes a 32-bit float. |
| `bus._write_parameter_i32(device_id, param_id, value)` | Writes a signed 32-bit integer. |
| `bus._write_parameter_u32(device_id, param_id, value)` | Writes an unsigned 32-bit integer. |

These are powerful but easy to misuse. Prefer the named helper functions below
unless you are writing a configuration or diagnostic script.

### Position Controller Parameters

These are the most important functions for our current quadruped locomotion.

| Function | Purpose | Motion-control meaning |
| --- | --- | --- |
| `read_fast_frame_frequency(device_id)` / `write_fast_frame_frequency(device_id, value)` | Gets or sets fast status-frame rate. | Affects feedback update behavior. |
| `read_gear_ratio(device_id)` / `write_gear_ratio(device_id, value)` | Gets or sets controller gear ratio. | Usually configured once per actuator. |
| `read_position_kp(device_id)` / `write_position_kp(device_id, value)` | Position stiffness. | Higher `kp` makes the motor hold target position harder. |
| `read_position_kd(device_id)` / `write_position_kd(device_id, value)` | Damping for position control. Internally maps to velocity `kp`. | Higher `kd` reduces oscillation, but too much can feel sticky or unstable. |
| `read_position_ki(device_id)` / `write_position_ki(device_id, value)` | Position integrator gain. | Usually keep low or zero for walking tests. |
| `read_velocity_kp(device_id)` / `write_velocity_kp(device_id, value)` | Velocity-loop proportional gain. | Same backend parameter used by `position_kd`. |
| `read_velocity_ki(device_id)` / `write_velocity_ki(device_id, value)` | Velocity-loop integrator gain. | Usually firmware/configuration tuning. |
| `read_torque_limit(device_id)` / `write_torque_limit(device_id, value)` | Maximum output torque/current behavior allowed by the controller. | Critical safety knob; too low means soft legs, too high can damage hardware. |
| `read_velocity_limit(device_id)` / `write_velocity_limit(device_id, value)` | Maximum allowed velocity. | Can limit aggressive commands. |
| `read_position_limit_lower(device_id)` / `write_position_limit_lower(device_id, value)` | Lower software position limit. | Protection against overtravel. |
| `read_position_limit_upper(device_id)` / `write_position_limit_upper(device_id, value)` | Upper software position limit. | Protection against overtravel. |
| `read_position_offset(device_id)` / `write_position_offset(device_id, value)` | Position offset inside the controller. | Use carefully; our project mostly does homing offsets in Python. |
| `read_torque_filter_alpha(device_id)` / `write_torque_filter_alpha(device_id, value)` | Torque feedback/target filtering. | Advanced tuning for smoother response. |

In our quadruped code, the normal setup for each joint is:

```python
bus.write_position_kp(motor_id, kp)
bus.write_position_kd(motor_id, kd)
bus.write_torque_limit(motor_id, torque_limit)
bus.feed(motor_id)
bus.set_mode(motor_id, recoil.Mode.POSITION)
```

### Runtime Targets And Measurements

| Function | Purpose | Motion-control meaning |
| --- | --- | --- |
| `read_torque_target(device_id)` / `write_torque_target(device_id, value)` | Reads/writes torque target parameter. | Useful for torque-mode experiments. |
| `read_torque_measured(device_id)` | Reads estimated/measured torque. | Useful for load detection or debugging. |
| `read_velocity_target(device_id)` / `write_velocity_target(device_id, value)` | Reads/writes velocity target parameter. | Useful for velocity-mode experiments. |
| `read_velocity_measured(device_id)` | Reads measured motor velocity. | Useful for logs and instability diagnosis. |
| `read_position_target(device_id)` / `write_position_target(device_id, value)` | Reads/writes position target through SDO. | Prefer PDO for high-rate walking. |
| `read_position_measured(device_id)` | Reads measured motor position. | Used during homing and tracking checks. |

For walking, prefer `transmit_pdo_2()` over `write_position_target()` because
PDO is the intended fast loop path.

### Current Controller, Motor, Encoder, And Filters

These functions are mostly for actuator configuration, diagnostics, or deeper
tuning. New team members should not change them during walking tests unless the
team is intentionally tuning motor-controller firmware parameters.

| Function | Purpose |
| --- | --- |
| `read_current_limit(device_id)` / `write_current_limit(device_id, value)` | Current controller current limit. |
| `read_current_kp(device_id)` / `write_current_kp(device_id, value)` | Current loop proportional gain. |
| `read_current_ki(device_id)` / `write_current_ki(device_id, value)` | Current loop integral gain. |
| `read_bus_voltage_filter_alpha(device_id)` / `write_bus_voltage_filter_alpha(device_id, value)` | Bus voltage filter tuning. |
| `read_motor_pole_pairs(device_id)` / `write_motor_pole_pairs(device_id, value)` | Motor pole-pair count. |
| `read_motor_torque_constant(device_id)` / `write_motor_torque_constant(device_id, value)` | Motor torque constant. |
| `read_motor_phase_order(device_id)` / `write_motor_phase_order(device_id, value)` | Motor phase order. |
| `read_motor_calibration_current(device_id)` / `write_motor_calibration_current(device_id, value)` | Calibration current. |
| `read_encoder_cpr(device_id)` / `write_encoder_cpr(device_id, value)` | Encoder counts per revolution. |
| `read_encoder_position_offset(device_id)` / `write_encoder_position_offset(device_id, value)` | Encoder offset. |
| `read_encoder_velocity_filter_alpha(device_id)` / `write_encoder_velocity_filter_alpha(device_id, value)` | Encoder velocity filter. |
| `read_encoder_flux_offset(device_id)` / `write_encoder_flux_offset(device_id, value)` | Encoder flux offset. |

### Bandwidth Helper Functions

| Function | Purpose |
| --- | --- |
| `set_current_bandwidth(device_id, bandwidth_hz, phase_resistance, phase_inductance)` | Computes and writes current-loop `kp` and `ki`. |
| `set_torque_bandwidth(device_id, bandwidth_hz, position_loop_rate=2000)` | Computes and writes torque-filter alpha. |
| `set_bus_voltage_bandwidth(device_id, bandwidth_hz, bus_voltage_update_rate=10000)` | Computes and writes bus-voltage filter alpha. |
| `set_encoder_velocity_bandwidth(device_id, bandwidth_hz, encoder_update_rate=10000)` | Computes and writes encoder velocity filter alpha. |

These are advanced tuning helpers. They are not part of normal quadruped
startup.

### PDO 2 Fast Motion Functions

These are the most important runtime motion functions.

| Function | Purpose | When to use |
| --- | --- | --- |
| `bus.transmit_pdo_2(device_id, position_target, velocity_target)` | Sends one fast position/velocity target frame. | Use inside high-rate motion loops. |
| `bus.receive_pdo_2(device_id)` | Reads measured position and velocity from a PDO response. | Use for high-rate feedback if the firmware is configured to respond. |
| `bus.write_read_pdo_2(device_id, position_target, velocity_target)` | Sends PDO 2 target, then reads PDO 2 feedback. | Good for single-motor test scripts. |

The user sometimes writes `wrie_read_pdo()`, but the actual Python function name
in this package is:

```python
bus.write_read_pdo_2(device_id, position_target, velocity_target)
```

Example single-joint motion loop:

```python
bus.write_position_kp(device_id, 0.25)
bus.write_position_kd(device_id, 0.01)
bus.write_torque_limit(device_id, 1.0)
bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

for target in targets:
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id,
        target,
        0.0,
    )
```

In our whole-robot code, we usually use `transmit_pdo_2()` instead of
`write_read_pdo_2()` because reading every motor immediately can slow down the
control loop. We do tracking checks separately.

## C++ API

The C++ side contains two important classes: `SocketCan` and
`MotorController`.

### SocketCan

| Function | Purpose |
| --- | --- |
| `SocketCan()` | Creates a CAN socket wrapper. |
| `~SocketCan()` | Closes the socket if it is still open. |
| `bool open(const std::string& interface)` | Opens and binds a Linux CAN interface such as `can0`. |
| `void close()` | Closes the CAN socket. |
| `bool isOpen() const` | Returns whether the socket is open. |
| `void write(can_frame* frame) const` | Sends a raw CAN frame. |
| `can_frame read()` | Waits for and returns a CAN frame. |

Example:

```cpp
SocketCan bus;
if (!bus.open("can2")) {
  return 1;
}
```

### MotorController

Create one `MotorController` object for each motor ID:

```cpp
SocketCan bus;
bus.open("can2");
MotorController hip(&bus, 6);
```

Core functions:

| Function | Python equivalent | Purpose |
| --- | --- | --- |
| `ping()` | `bus.ping(device_id)` | Checks communication. |
| `feed()` | `bus.feed(device_id)` | Feeds watchdog/heartbeat. |
| `set_mode(mode)` | `bus.set_mode(device_id, mode)` | Sets controller mode. |
| `load_settings_from_flash()` | `bus.load_settings_from_flash(device_id)` | Loads stored parameters. |
| `store_settings_to_flash()` | `bus.store_settings_to_flash(device_id)` | Saves current parameters. |

Position controller functions:

| C++ function | Python equivalent |
| --- | --- |
| `read_fast_frame_frequency()` / `write_fast_frame_frequency(frequency)` | `read_fast_frame_frequency()` / `write_fast_frame_frequency()` |
| `read_gear_ratio()` / `write_gear_ratio(ratio)` | `read_gear_ratio()` / `write_gear_ratio()` |
| `read_position_kp()` / `write_position_kp(kp)` | `read_position_kp()` / `write_position_kp()` |
| `read_position_kd()` / `write_position_kd(kd)` | `read_position_kd()` / `write_position_kd()` |
| `read_position_ki()` / `write_position_ki(ki)` | `read_position_ki()` / `write_position_ki()` |
| `read_velocity_kp()` / `write_velocity_kp(kp)` | `read_velocity_kp()` / `write_velocity_kp()` |
| `read_velocity_ki()` / `write_velocity_ki(ki)` | `read_velocity_ki()` / `write_velocity_ki()` |
| `read_torque_limit()` / `write_torque_limit(torque)` | `read_torque_limit()` / `write_torque_limit()` |
| `read_velocity_limit()` / `write_velocity_limit(limit)` | `read_velocity_limit()` / `write_velocity_limit()` |
| `read_position_limit_lower()` / `write_position_limit_lower(limit)` | `read_position_limit_lower()` / `write_position_limit_lower()` |
| `read_position_limit_upper()` / `write_position_limit_upper(limit)` | `read_position_limit_upper()` / `write_position_limit_upper()` |
| `read_position_offset()` / `write_position_offset(offset)` | `read_position_offset()` / `write_position_offset()` |
| `read_torque_filter_alpha()` / `write_torque_filter_alpha(alpha)` | `read_torque_filter_alpha()` / `write_torque_filter_alpha()` |

Runtime target and feedback functions:

| C++ function | Python equivalent | Purpose |
| --- | --- | --- |
| `read_torque_target()` / `write_torque_target(torque)` | `read_torque_target()` / `write_torque_target()` | Torque target parameter. |
| `read_torque_measured()` | `read_torque_measured()` | Measured torque. |
| `read_velocity_measured()` | `read_velocity_measured()` | Measured velocity. |
| `read_position_target()` / `write_position_target(position)` | `read_position_target()` / `write_position_target()` | Position target through parameter access. |
| `read_position_measured()` | `read_position_measured()` | Measured position. |
| `set_target_position(position)` | No direct Python state setter | Stores target before `write_pdo_2()`. |
| `set_target_velocity(velocity)` | No direct Python state setter | Stores velocity target before `write_pdo_2()`. |
| `write_pdo_2()` | `transmit_pdo_2()` | Sends stored position and velocity target. |
| `read_pdo_2()` | `receive_pdo_2()` | Reads PDO feedback into stored measured fields. |
| `get_measured_position()` | Return value from `receive_pdo_2()` | Returns last PDO measured position. |
| `get_measured_velocity()` | Return value from `receive_pdo_2()` | Returns last PDO measured velocity. |

Current, motor, encoder, and filter functions:

| C++ function | Python equivalent |
| --- | --- |
| `read_current_limit()` / `write_current_limit(current)` | `read_current_limit()` / `write_current_limit()` |
| `read_current_kp()` / `write_current_kp(kp)` | `read_current_kp()` / `write_current_kp()` |
| `read_current_ki()` / `write_current_ki(ki)` | `read_current_ki()` / `write_current_ki()` |
| `read_bus_voltage_filter_alpha()` / `write_bus_voltage_filter_alpha(alpha)` | `read_bus_voltage_filter_alpha()` / `write_bus_voltage_filter_alpha()` |
| `read_motor_torque_constant()` / `write_motor_torque_constant(value)` | `read_motor_torque_constant()` / `write_motor_torque_constant()` |
| `read_motor_phase_order()` | `read_motor_phase_order()` | Reads configured phase order. |
| `write_motor_phase_order(order)` | `write_motor_phase_order()` | Declared in the C++ header, but the checked source snapshot does not show its C++ implementation. Use the Python helper or raw parameter write if needed. |
| `read_encoder_velocity_filter_alpha()` / `write_encoder_velocity_filter_alpha(alpha)` | `read_encoder_velocity_filter_alpha()` / `write_encoder_velocity_filter_alpha()` |
| `set_current_bandwidth(bandwidth_hz, phase_resistance, phase_inductance)` | `set_current_bandwidth()` |
| `set_torque_bandwidth(bandwidth_hz, position_loop_rate)` | `set_torque_bandwidth()` |
| `set_bus_voltage_bandwidth(bandwidth_hz, bus_voltage_update_rate)` | `set_bus_voltage_bandwidth()` |
| `set_encoder_velocity_bandwidth(bandwidth_hz, encoder_update_rate)` | `set_encoder_velocity_bandwidth()` |

The Python API exposes a few extra motor/encoder helpers that the C++ header
does not list directly, such as motor pole pairs, encoder CPR, encoder position
offset, motor calibration current, and encoder flux offset. For firmware
configuration work, Python is currently the more complete and convenient API.

### C++ Mode Constants

Use these C++ constants from `motor_controller_conf.h`:

```cpp
MODE_DISABLED
MODE_IDLE
MODE_DAMPING
MODE_CALIBRATION
MODE_CURRENT
MODE_TORQUE
MODE_VELOCITY
MODE_POSITION
MODE_VABC_OVERRIDE
MODE_VALPHABETA_OVERRIDE
MODE_VQD_OVERRIDE
MODE_DEBUG
```

## How This Fits Our Quadruped Motion Control

Our control stack is layered like this:

```text
Xbox / command input
    -> gait scheduler and trajectory generation
    -> inverse kinematics
    -> leg_controller.py raw motor targets
    -> Recoil Bus / PDO / SDO functions
    -> SocketCAN can1-can4
    -> STM32 motor controllers
    -> motors and legs
```

During startup:

1. Bring up CAN interfaces in Linux.
2. Create one `recoil.Bus` per leg.
3. Read current motor positions as the homing reference.
4. Write `kp`, `kd`, and torque limits from `leg_config.py`.
5. Set motors to `POSITION`.
6. Feed every motor watchdog.
7. Move from homing pose to neutral standing pose.

During trotting:

1. Xbox input creates forward/turn commands.
2. The gait scheduler selects which legs are swing or stance.
3. The trajectory code creates target foot positions.
4. IK converts each foot target into hip/thigh/shank angles.
5. `leg_controller.py` converts output joint angles into raw motor targets.
6. `bus.transmit_pdo_2(motor_id, raw_target, 0.0)` streams targets.

During shutdown:

1. Stop accepting new motion commands.
2. Send multiple rounds of `recoil.Mode.IDLE` to every motor.
3. Set gains or active commands to zero if the program supports it.
4. Call `bus.stop()` for every CAN channel.

## Practical Rules For New Team Members

Use `read_position_measured()` when you need to know where a motor is now.

Use `write_position_kp()`, `write_position_kd()`, and `write_torque_limit()`
before enabling position mode.

Use `set_mode(..., recoil.Mode.POSITION)` only after you know the current pose
and have a reasonable target.

Use `transmit_pdo_2()` for normal walking loops.

Use `write_read_pdo_2()` for simple single-motor tests where immediate feedback
is more important than maximum loop speed.

Use `set_mode(..., recoil.Mode.IDLE)` and `bus.stop()` in every `finally` block
or Ctrl+C shutdown path.

Do not write flash during normal walking tests.

Do not change encoder, motor, phase-order, or current-loop parameters during
normal robot testing unless the team is deliberately doing actuator
configuration.

## Minimal Python Test Pattern: Setup And Hold

This first example is mainly a communication and safety test.

It checks that the CAN ID responds, reads the current motor position, writes
`kp`, `kd`, and torque limit, enters `POSITION` mode, then keeps commanding the
same measured position.

Because `target = start`, this code should hold the motor where it already is.
It does not intentionally move the motor.

```python
import time
import berkeley_humanoid_lite_lowlevel.recoil as recoil

channel = "can2"
motor_id = 6

bus = recoil.Bus(channel=channel, bitrate=1000000)

try:
    if not bus.ping(motor_id):
        raise RuntimeError(f"Motor {motor_id} did not respond on {channel}")

    start = bus.read_position_measured(motor_id)
    if start is None:
        raise RuntimeError("Could not read measured position")

    bus.write_position_kp(motor_id, 0.20)
    bus.write_position_kd(motor_id, 0.010)
    bus.write_torque_limit(motor_id, 0.60)
    bus.set_mode(motor_id, recoil.Mode.POSITION)
    bus.feed(motor_id)

    target = start
    for _ in range(200):
        bus.feed(motor_id)
        bus.transmit_pdo_2(motor_id, target, 0.0)
        time.sleep(0.01)

finally:
    bus.set_mode(motor_id, recoil.Mode.IDLE)
    bus.stop()
```

## Python: Move To An Absolute Target

Use this when you want the motor to move to a known raw motor position.

The target is absolute in the motor controller's measured-position units. In
our quadruped code, this is not directly foot position or joint angle; our
`leg_controller.py` converts robot joint angles into raw motor target values.

```python
import time
import berkeley_humanoid_lite_lowlevel.recoil as recoil

bus = recoil.Bus(channel="can2", bitrate=1000000)
motor_id = 6

absolute_target = 10.0

try:
    bus.write_position_kp(motor_id, 0.20)
    bus.write_position_kd(motor_id, 0.010)
    bus.write_torque_limit(motor_id, 0.60)
    bus.set_mode(motor_id, recoil.Mode.POSITION)

    for _ in range(300):
        bus.feed(motor_id)
        bus.transmit_pdo_2(motor_id, absolute_target, 0.0)
        time.sleep(0.01)

finally:
    bus.set_mode(motor_id, recoil.Mode.IDLE)
    bus.stop()
```

## Python: Move By A Relative Angle

Use this when you want to move from the current measured position by a small
offset.

For example, if `relative_move = 0.5`, the motor target becomes current position
plus `0.5` in raw motor position units.

```python
import time
import berkeley_humanoid_lite_lowlevel.recoil as recoil

bus = recoil.Bus(channel="can2", bitrate=1000000)
motor_id = 6

relative_move = 0.5

try:
    start = bus.read_position_measured(motor_id)
    if start is None:
        raise RuntimeError("Could not read motor position")

    target = start + relative_move

    bus.write_position_kp(motor_id, 0.20)
    bus.write_position_kd(motor_id, 0.010)
    bus.write_torque_limit(motor_id, 0.60)
    bus.set_mode(motor_id, recoil.Mode.POSITION)

    for _ in range(300):
        bus.feed(motor_id)
        bus.transmit_pdo_2(motor_id, target, 0.0)
        time.sleep(0.01)

finally:
    bus.set_mode(motor_id, recoil.Mode.IDLE)
    bus.stop()
```

## Python: Follow A Smooth Curve

Use this pattern when you want the motor to follow a changing target over time.

This example sends a sine wave around the starting position. This is the same
basic idea as leg trotting: every control step computes a new target and sends
it through `transmit_pdo_2()`.

```python
import math
import time
import berkeley_humanoid_lite_lowlevel.recoil as recoil

bus = recoil.Bus(channel="can2", bitrate=1000000)
motor_id = 6

amplitude = 0.4
frequency_hz = 0.5
duration_s = 6.0
dt = 0.01

try:
    center = bus.read_position_measured(motor_id)
    if center is None:
        raise RuntimeError("Could not read motor position")

    bus.write_position_kp(motor_id, 0.20)
    bus.write_position_kd(motor_id, 0.010)
    bus.write_torque_limit(motor_id, 0.60)
    bus.set_mode(motor_id, recoil.Mode.POSITION)

    start_time = time.monotonic()
    while time.monotonic() - start_time < duration_s:
        t = time.monotonic() - start_time
        target = center + amplitude * math.sin(2.0 * math.pi * frequency_hz * t)

        bus.feed(motor_id)
        bus.transmit_pdo_2(motor_id, target, 0.0)
        time.sleep(dt)

finally:
    bus.set_mode(motor_id, recoil.Mode.IDLE)
    bus.stop()
```

## Python: Absolute, Relative, And Curve Summary

Absolute target:

```python
target = 10.0
bus.transmit_pdo_2(motor_id, target, 0.0)
```

Relative target:

```python
start = bus.read_position_measured(motor_id)
target = start + 0.5
bus.transmit_pdo_2(motor_id, target, 0.0)
```

Curve target:

```python
target = center + amplitude * math.sin(2.0 * math.pi * frequency_hz * t)
bus.transmit_pdo_2(motor_id, target, 0.0)
```

The motor only moves when the target you send is different from where the motor
already is, and the controller is in `POSITION` mode with reasonable gains and
torque limit.

## Minimal C++ Test Pattern: Setup And Hold

```cpp
#include "socketcan.h"
#include "motor_controller.h"
#include "motor_controller_conf.h"

#include <chrono>
#include <thread>

int main() {
  SocketCan bus;
  if (!bus.open("can2")) {
    return 1;
  }

  MotorController motor(&bus, 6);

  motor.write_position_kp(0.20);
  motor.write_position_kd(0.010);
  motor.write_torque_limit(0.60);
  motor.set_mode(MODE_POSITION);
  motor.feed();

  float start = motor.read_position_measured();
  motor.set_target_position(start);
  motor.set_target_velocity(0.0);

  for (int i = 0; i < 200; ++i) {
    motor.feed();
    motor.write_pdo_2();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  motor.set_mode(MODE_IDLE);
  bus.close();
  return 0;
}
```

## C++: Move To Absolute, Relative, Or Curve Target

Absolute target:

```cpp
motor.set_target_position(10.0f);
motor.set_target_velocity(0.0f);
motor.write_pdo_2();
```

Relative target:

```cpp
float start = motor.read_position_measured();
motor.set_target_position(start + 0.5f);
motor.set_target_velocity(0.0f);
motor.write_pdo_2();
```

Curve target:

```cpp
float target = center + amplitude * sinf(2.0f * M_PI * frequency_hz * t);
motor.set_target_position(target);
motor.set_target_velocity(0.0f);
motor.write_pdo_2();
```

Just like Python, C++ motion requires `MODE_POSITION`, watchdog `feed()`, valid
gains, a torque limit, and repeated PDO target updates.

## Common Failure Symptoms

If `ping()` fails, check CAN interface name, bitrate, wiring, termination,
controller power, and whether another motor with the same ID is on the bus.

If CAN LEDs flash but motors have no torque, check that the program wrote gains
and torque limit, fed the watchdog, and set `Mode.POSITION`.

If one controller prevents other controllers on the same channel from
responding, disconnect nodes one at a time. A bad or miswired controller can
disturb the whole CAN channel.

If the robot moves in the wrong direction, do not fix that in Recoil. Fix the
joint sign and homing mapping in our `leg_config.py` layer.

If the robot is too soft or too aggressive, tune `kp`, `kd`, torque limits,
trajectory step height, step length, cycle time, and stance ratio in our config
files before changing firmware-level current-loop parameters.
