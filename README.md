# Quadruped Robot

Open-source, 3D-printable, modular quadruped robot designed as a hardware platform for robotics education, controls development, kinematics, reinforcement learning, robotic vision, and future semi-commercial research applications.

This project focuses on building a practical legged robot platform using custom cycloidal actuators, off-the-shelf electronics, CAN-based motor communication, and open-source low-level motor control software. The goal is to create a robot that is mechanically capable, reproducible, and flexible enough for future work in locomotion, autonomous navigation, and robotics research.

## Project Overview

The quadruped is a 12-degree-of-freedom robotic platform with three actuated joints per leg. The mechanical design uses custom 3D-printed cycloidal actuators with a 17:1 gear ratio. These actuators were designed to make the robot modular, serviceable, and easier to manufacture using accessible tools such as 3D printers.

The CAD design is complete, and the actuator assemblies have been built. The current electronics setup uses off-the-shelf hardware for development and testing, while the repository also includes ongoing custom FOC driver board design files. Software development is focused on low-level actuator communication, calibration, inverse kinematics, single-leg testing, and full quadruped locomotion.

Full walking is still under development.

## Current Status

| Area                          | Status                                                   |
| ----------------------------- | -------------------------------------------------------- |
| CAD design                    | Complete                                                 |
| Actuator design               | Complete                                                 |
| Actuator build                | Complete                                                 |
| Electronics                   | Off-the-shelf electronics currently used for development |
| Custom FOC board              | KiCad design files included / in development             |
| Low-level motor communication | Based on Berkeley Humanoid Lite low-level code           |
| Calibration scripts           | In progress                                              |
| Single-leg control            | In progress                                              |
| Full quadruped walking        | In development                                           |
| Vision/autonomy               | Future work                                              |

## Features

* 12-DOF quadruped layout
* Three actuators per leg
* Custom 17:1 cycloidal actuator design
* 3D-printable mechanical structure
* Modular leg and actuator design
* CAD files for the full robot body, legs, and accessories
* CAN-based actuator communication
* Low-level motor control using Berkeley Humanoid Lite/Recoil-style communication code
* Calibration scripts for actuator setup and configuration
* Forward and inverse kinematics tools
* Early locomotion and gait-control scripts
* Xbox controller testing scripts for manual actuator/robot control
* KiCad files for custom FOC driver board development
* BOM and purchasing documentation

## Repository Structure

```text
Quadruped-Robot/
├── 3D_Model/
│   ├── Accessory/
│   ├── Quadruped Body/
│   └── Quadruped Leg/
│
├── Control_Code/
│   ├── Calibration/
│   ├── Locomotion/
│   ├── lowlevel_actuator/
│   ├── quad_ik_can/
│   └── recoil/
│
├── Documentation/
│
├── KiCAD Files/
│   ├── 3D models/
│   ├── BHL_S1_Current/
│   ├── Footprints/
│   ├── Protoype1_Final_revision/
│   └── Symbols/
│
├── Motor_Firmware/
│
├── Purchasing/
│   ├── BOM/
│   └── RFF/
│
└── README.md
```

## Folder Descriptions

### `3D_Model/`

Contains the mechanical design files for the robot. This includes accessory parts, the quadruped body assembly, and leg assembly files.

Important areas include:

* `Accessory/` — printable accessory parts such as body covers and handles
* `Quadruped Body/` — full robot body assembly files
* `Quadruped Leg/` — leg assembly files

These files are used for manufacturing, assembly planning, and future mechanical revisions.

### `Control_Code/`

Contains the Python control software for actuator testing, calibration, kinematics, and early locomotion development.

#### `Control_Code/Calibration/`

Scripts used for setting up and validating individual actuators and motor controllers.

Examples include:

* `ping.py` — basic communication test
* `calibrate_electrical_offset.py` — motor/encoder electrical offset calibration
* `configure_parameter.py` — motor controller parameter configuration
* `read_configurations.py` — read back motor controller settings
* `move_actuator.py` — basic actuator motion test

#### `Control_Code/Locomotion/`

Main area for robot-level locomotion development.

Includes:

* Forward kinematics
* Inverse kinematics
* Leg configuration
* Gain configuration
* Gait scheduling
* Robot controller scripts
* Xbox controller configuration
* IMU reading utilities
* Single-leg and quadruped testing scripts

Notable files include:

* `quadruped_leg_fk.py`
* `quadruped_leg_ik.py`
* `leg_controller.py`
* `robot_controller.py`
* `gait_scheduler.py`
* `quadruped_main_slim.py`
* `quadruped_main_state.py`
* `single_leg_trot.py`
* `remote_single_leg_trot.py`
* `xbox_home_v5.py`
* `read_bno055_live.py`

#### `Control_Code/lowlevel_actuator/`

Contains lower-level actuator test scripts used to command one or more actuators directly.

Examples include:

* `actuator_move_position.py`
* `actuator_ramp_position.py`
* `actuator_ramp_rel_pos.py`
* `triple_actuator_homing.py`
* `triple_actuator_rel_pos.py`
* `xbox_position_control_test.py`
* `xbox_velocity_control_test.py`

These scripts are useful for validating actuator behavior before integrating the full robot.

#### `Control_Code/quad_ik_can/`

Contains an additional inverse-kinematics-over-CAN control workflow.

Includes:

* `can_interface.py`
* `config.py`
* `ik_solver.py`
* `main.py`
* `README.md`

This section appears to be focused on combining inverse kinematics commands with CAN communication.

#### `Control_Code/recoil/`

Contains the low-level Recoil-style communication code used to interface with the FOC motor drivers.

This project uses the Berkeley Humanoid Lite low-level code as the foundation for communicating with the FOC drivers.

Files include:

* `can.py`
* `core.py`
* `fixed16.py`
* `util.py`

## Mechanical Design

The robot is built around custom cycloidal actuators. Each actuator is designed to be compact, modular, and manufacturable with accessible tools. The use of a cycloidal gearbox provides a high reduction ratio while keeping the actuator package suitable for legged robotics.

The mechanical system was designed with the following goals:

* Keep the robot modular and easy to repair
* Make the actuator design reproducible
* Use 3D printing where practical
* Reduce complexity compared to fully custom machined systems
* Provide enough torque and structure for walking development
* Create a platform that can support future controls and autonomy work

## Electronics

The current robot uses off-the-shelf electronics for development and testing. This makes it easier to focus on actuator validation, CAN communication, calibration, and walking control before depending on fully custom hardware.

The repository also includes KiCad files for custom FOC driver board development. These files include PCB designs, schematics, footprints, symbols, 3D models, and previous design revisions.

### `KiCAD Files/`

This folder contains custom motor controller PCB design files and related KiCad resources.

Included resources:

* KiCad project files
* PCB layout files
* Schematic files
* Footprints
* Symbols
* 3D models
* Older board revisions
* Prototype revisions

This section is useful for future development of custom motor-control electronics.

### `Motor_Firmware/`

Contains motor firmware resources, including a modified Recoil motor controller firmware archive.

## Software

The control software is written primarily in Python and is organized around actuator bring-up, calibration, inverse kinematics, and locomotion development.

The current low-level communication approach is based on the Berkeley Humanoid Lite low-level code. This code is used to communicate with the FOC drivers and send actuator commands over CAN.

### Software Goals

* Establish reliable communication with each actuator
* Test individual motor and actuator behavior
* Validate actuator direction, encoder feedback, and control modes
* Calibrate electrical offsets and controller parameters
* Develop single-leg control
* Develop inverse kinematics for each leg
* Build toward coordinated full-body locomotion
* Eventually support higher-level walking, autonomy, and vision experiments

## Running and Testing

Example test scripts are available in the Berkeley Humanoid Lite low-level software and in this repository’s `Control_Code/` folder.

A typical development workflow is:

1. Set up the Python environment.
2. Connect the CAN interface and motor controller hardware.
3. Confirm communication with the actuator.
4. Run a basic ping or configuration read test.
5. Calibrate the actuator electrical offset.
6. Test basic actuator movement.
7. Test multiple actuators together.
8. Run inverse kinematics tests.
9. Test single-leg motion.
10. Move toward full quadruped locomotion.

Example scripts to start with:

```bash
python Control_Code/Calibration/ping.py
python Control_Code/Calibration/read_configurations.py
python Control_Code/Calibration/calibrate_electrical_offset.py
python Control_Code/Calibration/move_actuator.py
```

For low-level actuator tests:

```bash
python Control_Code/lowlevel_actuator/actuator_move_position.py
python Control_Code/lowlevel_actuator/actuator_ramp_position.py
python Control_Code/lowlevel_actuator/triple_actuator_homing.py
```

For locomotion and kinematics development:

```bash
python Control_Code/Locomotion/ik_terminal_test.py
python Control_Code/Locomotion/single_leg_trot.py
python Control_Code/Locomotion/quadruped_main_slim.py
python Control_Code/Locomotion/quadruped_main_state.py
```

Exact commands may need to be adjusted depending on the CAN interface, operating system, motor IDs, and hardware configuration.

## Development Notes

This project is still actively being developed. Many scripts are experimental and may need to be updated for a specific hardware setup.

Before running full robot tests, verify:

* CAN interface name and bitrate
* Motor controller IDs
* Encoder direction
* Motor phase order
* Joint limits
* Homing offsets
* Control gains
* Power supply current limit
* Emergency stop or disconnect method

## Safety Notes

This project involves motors, gear reductions, motor controllers, power electronics, and moving robotic limbs. Use caution when testing.

Recommended safety practices:

* Test actuators one at a time before full robot integration
* Keep the robot lifted or supported during early walking tests
* Use current limits during initial motor testing
* Keep hands, wires, and tools away from moving joints
* Have an emergency stop or quick power disconnect available
* Double-check wiring before applying power
* Verify motor direction before installing actuators into the full robot
* Confirm joint limits before running autonomous motion
* Avoid testing high-power motion near people or fragile objects
* Monitor motor and driver temperature during testing
* Do not leave powered actuators unattended

This robot is still under development, so all testing should be done carefully and incrementally.

## Documentation

The `Documentation/` folder contains project images and supporting documents. More build documentation, assembly instructions, wiring diagrams, CAD renders, and videos will be added as the project progresses.

## Purchasing and BOM

The `Purchasing/` folder contains bill of materials files, expense tracking, and request-for-funding documentation.

This includes:

* FOC drive board BOM
* Overall robot BOM
* Expense tracker
* Funding/request documentation

These files are useful for tracking cost, sourcing parts, and documenting project funding.

## Roadmap

### Completed

* Designed the full quadruped CAD model
* Designed custom cycloidal actuators
* Built the actuator assemblies
* Created 3D-printable body, leg, and accessory files
* Set up off-the-shelf electronics for development
* Added low-level actuator testing scripts
* Added calibration scripts
* Added forward and inverse kinematics code
* Added early locomotion and single-leg test scripts
* Added KiCad files for custom FOC driver board development

### In Progress

* Low-level actuator testing
* Motor communication and control validation
* Actuator calibration
* Leg-level control development
* Gait scheduling
* Single-leg trot testing
* Full quadruped walking development

### Future Work

* Full quadruped walking demo
* Improved gait generation
* Higher-level body control
* Autonomous navigation
* Robotic vision integration
* Custom electronics refinement
* Cleaner software setup instructions
* Assembly guide
* Wiring diagrams
* CAD renders and demo videos
* More complete documentation for calibration and testing

## Acknowledgments

This project uses the Berkeley Humanoid Lite low-level code as the foundation for communication with the FOC drivers.

The project is inspired by open-source robotics platforms and the goal of making advanced legged robotics more accessible for students, researchers, and builders.

## Project Status Disclaimer

This repository is still a work in progress. The CAD and actuator builds are complete, but full walking control is still under development. Some scripts are experimental and may require hardware-specific configuration before use.

## License

License information has not been finalized yet. Add the appropriate license before using this project for public distribution or commercial work.
