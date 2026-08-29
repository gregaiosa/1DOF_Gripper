---

[![Portfolio](https://img.shields.io/badge/View_Portfolio_Post-blue?style=for-the-badge)](https://gregaiosa.github.io/projects/1_dof_robot_gripper/)

This repository contains the ROS 2 workspace and configuration for a custom 1 Degree of Freedom (DOF) robotic gripper adapted for the SO-101 robotic arm. The system is designed specifically for high-fidelity haptic feedback and bilateral teleoperation, utilizing quasi-direct-drive brushless motors to maximize backdriveability and eliminate gear backlash.

## System Overview
* **Bilateral Teleoperation:** Implements a 1:1 Leader-Follower setup using a 1kHz SocketCAN control loop. The leader mirrors the position of the follower while directly reflecting the follower's measured torque back to the user's hand.
* **Haptic Feedback & Transparency:** Employs CubeMars brushless gimbal motors with active Coulomb friction compensation, allowing the user to feel remote interactions with minimal mechanical resistance from the hardware itself.
* **Sensorless Homing:** Features a current-based homing routine that detects physical hard-stops by monitoring current spikes, ensuring consistent calibration without the need for external limit switches.
* **Unified ROS 2 Architecture:** Integrates the gripper's high-speed CAN loop directly with the SO-101 arm's serial control into a single ROS 2 node.

## Hardware Requirements
* SO-101 Robotic Arm (Feetech bus)
* CubeMars GL40 II (Follower / Gripper Motor)
* CubeMars GL60 II (Leader / Teleoperation Handle)
* CAN Bus interface (e.g., USB to CAN adapter configured on `can0`)
* Compute Node (Tested on Linux/Ubuntu)

## Software Dependencies
* ROS 2
* `python-can` (for SocketCAN communication)
* `lerobot` (for SO-101 arm control)

## Installation
1. Clone this repository into your workspace:
   ```bash
   cd ~/your_ws/src
   git clone https://github.com/gregaiosa/1DOF_Gripper.git
   ```
2. Setup the CAN interface:
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```
3. Build the workspace:
   ```bash
   cd ~/your_ws
   colcon build --packages-select gripper
   source install/setup.bash
   ```

## Usage

### 1. Launching the System
The entire robotic system (SO-101 Arm, CAN Grippers, and Web UI Bridge) can be brought up using the provided launch file. This starts the hardware interface and delays the Phosphobot Web UI bridge to allow for CAN initialization.
```bash
ros2 launch gripper system_launch.py
```

### 2. Manual Control & Homing
Once the nodes are active, the follower gripper must be homed against its hard stop before the full bilateral mode engages. You can trigger this via a ROS 2 service call:
```bash
ros2 service call /so101_arm_node/gripper/home std_srvs/srv/Trigger
```

### `so101_arm_node.py` (Core Control & Teleoperation)
* **High-Frequency CAN Loop:** Runs a dedicated `1000Hz` `_can_control_loop` overriding ROS topics to communicate directly with both the GL60 II (Node 1) and GL40 II (Node 3) over MIT mode.
* **Bilateral Mapping:** Translates the leader's actual position directly to the follower's target. It captures the follower's real-time torque and applies it as a negated feedforward (`tff`) torque to the leader, scaled by a configurable `haptic_gain`.
* **Friction Compensation:** Dynamically applies feedforward torque to overcome static/Coulomb friction, tapering linearly within 0.1 radians of the target to prevent bang-bang limit cycles.
* **Dynamic Parameter Tuning:** Implements parameter callbacks allowing on-the-fly adjustment of control gains (`kp`, `kd`), `friction_comp`, and `haptic_gain` via the ROS 2 parameter server.
* **Comprehensive Telemetry:** Publishes high-resolution actual and commanded states (position, velocity, current, torque) for both leader and follower motors, enabling deep system profiling and debugging.
* **Arm Integration:** Concurrently manages the Feetech serial bus for the SO-101 arm at ~30Hz, publishing `/arm/joint_states` and subscribing to `/arm/joint_commands`.

### `phosphobot_bridge.py` (Web UI Integration)
* Translates web-based teleoperation commands from the Phosphobot application into compatible `JointState` messages for the arm and normalized `Float64` commands for the gripper.
* Provides a bridge for the `/gripper/home`, `open`, and `close` services.

### `gl_control_mit_mode.py` (Tuning & Diagnostics)
* A standalone script used for profiling motor performance, tuning PD gains (`kp`, `kd`), and plotting step-response characteristics via `matplotlib`. It bypasses ROS entirely for low-level hardware debugging.