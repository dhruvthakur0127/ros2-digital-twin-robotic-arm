# 🤖 ROS 2 Digital Twin — 6-DOF Robotic Arm

A complete **Digital Twin** system for a 6-DOF robotic arm using **ROS 2 Jazzy**, **Gazebo Harmonic**, **MoveIt 2**, and **Arduino + PCA9685** servo control. When you move the robotic arm in the simulation, the physical arm follows in real-time.

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![MoveIt](https://img.shields.io/badge/MoveIt-2-green)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-purple)
![Arduino](https://img.shields.io/badge/Arduino-Uno_R3-teal)
![License](https://img.shields.io/badge/License-MIT-yellow)

> **Course:** 3EL13 — Electronics System Design Laboratory
> **Guide:** Dr. Dipakkumar M Patel
> **Authors:** Dhruv Thakur (23EL020)
> **Institution:** Birla Vishvakarma Mahavidyalaya Engineering College, Vallabh Vidyanagar, Gujarat

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Demo](#demo)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [URDF Source and Credits](#urdf-source-and-credits)
- [Installation Guide](#installation-guide)
- [Simulation Setup](#simulation-setup)
- [Hardware Setup](#hardware-setup)
- [Digital Twin Bridge](#digital-twin-bridge)
- [Customization Guide](#customization-guide)
- [Troubleshooting](#troubleshooting)
- [File Structure](#file-structure)
- [How It Works Technical](#how-it-works-technical)
- [Future Scope](#future-scope)
- [References](#references)

---

## Overview

This project creates a **Digital Twin** — a virtual replica of a physical robotic arm that operates in real-time synchronization. When you plan a motion in MoveIt 2 (the simulation), the physical robotic arm follows the same motion simultaneously.

**What this project does:**
- Simulates a 6-DOF robotic arm in Gazebo Harmonic with physics
- Plans collision-free motions using MoveIt 2 with multiple planners (OMPL, Pilz, STOMP)
- Extracts joint angles from the /joint_states ROS 2 topic
- Converts URDF radians to servo degrees using proportional mapping
- Sends commands via USB serial to Arduino Uno R3
- Arduino drives 6 servo motors through PCA9685 16-channel PWM driver
- Physical arm moves in sync with the simulation

**Key Features:**
- Proportional mapping between URDF joint range and physical servo range
- Per-joint configurable direction, limits, and home positions
- Sequential servo homing on startup (prevents jerk/damage)
- Safe shutdown with return-to-home on Ctrl+C
- Manual servo adjustment during operation via ROS 2 topics
- Custom PCA9685 channel mapping (any joint to any channel)

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                YOUR LAPTOP (Ubuntu VM)               │
│                                                      │
│   User (RViz GUI)                                    │
│       │ drag arm, click Plan & Execute               │
│       ▼                                              │
│   MoveIt 2 (move_group node)                         │
│       │ inverse kinematics + motion planning         │
│       ▼                                              │
│   arm_controller (JointTrajectoryController)         │
│       │                                              │
│       ├──► Gazebo Harmonic (simulation)              │
│       │                                              │
│       ▼                                              │
│   joint_state_broadcaster                            │
│       │ publishes /joint_states topic                 │
│       ▼                                              │
│   six_servo_bridge.py (Python ROS 2 node)            │
│       │ radians → proportional servo degrees         │
└───────┼──────────────────────────────────────────────┘
        │ USB Cable
        ▼
┌─────────────────────────────────────────────────────┐
│   ARDUINO UNO R3 → I2C → PCA9685 → 6 SERVOS        │
│                                                      │
│   5V 10A SMPS Power Supply → Powers servos           │
└─────────────────────────────────────────────────────┘
```

---


## Hardware Requirements

| Component | Specification | Qty | 
|-----------|---------------|-----|
| 6-DOF Robotic Arm Kit | RKI-2533 Aluminium | 1 | 
| Arduino Uno R3 | ATmega328P, CH340 | 1 | 
| PCA9685 Servo Driver | 16-ch, I2C, 12-bit | 1 | 
| MG995 Servo Motors | 11 kg·cm, metal gear | 6 | 
| 5V 10A SMPS | AC-DC, 50W | 1 | 
| Jumper Wires | M-M, M-F assorted | 1 set | 
| USB-B Cable | Arduino to PC | 1 | 

## Software Requirements

| Software | Version |
|----------|---------|
| Ubuntu | 24.04 LTS (Noble Numbat) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| MoveIt | 2.12.4 |
| Arduino IDE | 1.8.19+ |
| pyserial | 3.5 |

---

## URDF Source and Credits

The URDF robot model used in this project is the **myCobot 280** by Elephant Robotics.

> **Repository:** [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2)
> **Branch:** jazzy
> **Author:** Addison Sears-Collins ([Automatic Addison](https://automaticaddison.com/))
> **License:** BSD-3-Clause

We use this open-source URDF as the simulation model. The physical arm (RKI-2533) has different dimensions, so proportional mapping translates between the URDF joint ranges and the physical servo ranges.

**Special thanks** to Addison Sears-Collins for the tutorial series.

---

## Installation Guide

### Step 1: Install Ubuntu 24.04 LTS

Download from https://ubuntu.com/download/desktop (dual boot or VirtualBox).

### Step 2: Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y
sudo apt install ros-dev-tools -y

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install Gazebo Harmonic

```bash
sudo apt install ros-jazzy-ros-gz -y
```

For VirtualBox users:
```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc
source ~/.bashrc
```

### Step 4: Install MoveIt 2 and Dependencies

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-gz-ros2-control-demos ros-jazzy-gripper-controllers ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro -y
```

### Step 5: Create Workspace and Clone URDF

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/automaticaddison/mycobot_ros2.git
```

### Step 6: Build the Workspace

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --packages-skip mycobot_mtc_demos mycobot_moveit_demos mycobot_mtc_pick_place_demo
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7: Install Python Serial Library

```bash
sudo apt install python3-pip -y
pip3 install pyserial --break-system-packages
```

### Step 8: Set Up Serial Shortcut

```bash
echo "alias killserial='sudo fuser -k /dev/ttyUSB0 2>/dev/null; sudo fuser -k /dev/ttyUSB1 2>/dev/null; sudo chmod 666 /dev/ttyUSB* 2>/dev/null'" >> ~/.bashrc
source ~/.bashrc
```

---

## Simulation Setup

### Launch Gazebo (Terminal 1)

```bash
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py use_rviz:=false use_camera:=false
```
Wait 60 seconds.

### Verify Controllers (Terminal 2)

```bash
ros2 control list_controllers
```
All 3 should show "active".

### Launch MoveIt 2 (Terminal 3)

```bash
ros2 launch mycobot_moveit_config move_group.launch.py
```

Drag the arm in RViz and click Plan & Execute.

---

## Hardware Setup

### Wiring

```
Arduino A4 (SDA)  →  PCA9685 SDA
Arduino A5 (SCL)  →  PCA9685 SCL
Arduino 5V        →  PCA9685 VCC (logic power)
Arduino GND       →  PCA9685 GND
5V 10A PSU (+)    →  PCA9685 V+ (screw terminal - servo power)
5V 10A PSU (-)    →  PCA9685 GND (screw terminal)

PCA9685 CH0  →  Servo 1 (Base)
PCA9685 CH1  →  Servo 2 (Shoulder)
PCA9685 CH2  →  Servo 3 (Elbow)
PCA9685 CH3  →  Servo 4 (Wrist Pitch)
PCA9685 CH4  →  Servo 5 (Gripper)
PCA9685 CH5  →  Servo 6 (Wrist Roll)
```

### Upload Arduino Firmware

1. Install "Adafruit PWM Servo Driver" library in Arduino IDE
2. Open `arduino/digital_twin_firmware.ino`
3. Select Board: Arduino Uno, Port: /dev/ttyUSB0
4. Upload (use unplug-replug trick if needed)

### Test Servos

Open Serial Monitor (115200 baud, Newline):
```
0 90    → Base moves to 90°
1 100   → Shoulder moves to 100°
h       → All servos return to home
```

---

## Digital Twin Bridge

### Full System Launch

```bash
# Terminal 1: Gazebo
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py use_rviz:=false use_camera:=false
# Wait 60 seconds

# Terminal 2: Bridge
killserial && sudo chmod 666 /dev/ttyUSB0
python3 ros2_bridge/six_servo_bridge.py

# Terminal 3: MoveIt
ros2 launch mycobot_moveit_config move_group.launch.py
```

### Manual Servo Control (while bridge is running)

```bash
# Move single servo
ros2 topic pub --once /servo_adjust std_msgs/msg/String "{data: '0 90'}"

# Home all servos
ros2 topic pub --once /servo_adjust std_msgs/msg/String "{data: 'h'}"
```

---

## Customization Guide

All customization is done in `ros2_bridge/six_servo_bridge.py` in the `SERVO_CONFIG` list.

### Changing Servo Limits

```python
'servo_min': 55, 'servo_max': 150,    # Change min/max safe angles
```
Test your limits first using Arduino Serial Monitor.

### Changing Home Positions

```python
'servo_home': 100,    # Servo angle when URDF is at its home position
```
Also update Arduino code: `int homeAngles[NUM_SERVOS] = {90, 100, 50, 120, 120, 100};`

### Reversing Servo Direction

```python
'direction': -1,    # Change 1 to -1 (home position stays the same)
```

### Changing PCA9685 Channel Mapping

```python
'pca_channel': 5,    # Which physical PCA9685 channel this joint uses
```

### Changing Serial Port

```python
self.SERIAL_PORT = '/dev/ttyUSB1'    # Change if your Arduino is on a different port
```
Check with: `ls /dev/ttyUSB* /dev/ttyACM*`

### Changing URDF Home Position

Edit `config/initial_positions.yaml` and update matching `urdf_home` in Python bridge.
Rebuild: `cd ~/ros2_ws && colcon build --packages-skip mycobot_mtc_demos mycobot_moveit_demos mycobot_mtc_pick_place_demo`

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` |
| Gazebo black screen (VM) | Add `LIBGL_ALWAYS_SOFTWARE=1` to ~/.bashrc |
| Controller timeout | Wait 60s after Gazebo launch before MoveIt |
| Arduino upload fails | Unplug USB, click Upload, quickly replug |
| Servo wrong direction | Change `direction: -1` in bridge config |
| Serial port not found | `sudo modprobe ch341` then check `ls /dev/ttyUSB*` |
| Permission denied | `sudo chmod 666 /dev/ttyUSB0` |
| `killserial` not found | Add alias to ~/.bashrc (see Step 8 above) |

---

## File Structure

```
ros2-digital-twin-robotic-arm/
├── README.md
├── LICENSE
├── arduino/
│   └── digital_twin_firmware.ino
├── ros2_bridge/
│   └── six_servo_bridge.py
├── config/
│   └── initial_positions.yaml
└── docs/
    ├── system_architecture.png
    ├── software_architecture.png
    ├── hardware_block_diagram.png
    └── serial_protocol.png
```

---

## How It Works Technical

### Proportional Mapping Formula

```
delta = (current_urdf_radians - urdf_home) * direction

if delta >= 0:
    fraction = delta / (urdf_max - urdf_home)
    servo = servo_home + fraction * (servo_max - servo_home)
else:
    fraction = |delta| / (urdf_home - urdf_min)
    servo = servo_home - fraction * (servo_home - servo_min)
```

### Serial Protocol

```
Command (PC → Arduino):   "90 100 50 120 120 100\n"
Response (Arduino → PC):  "OK 90 100 50 120 120 100"
Single servo:             "0 90\n"
Home all:                 "h\n"
```

---

## Future Scope

- [ ] Custom URDF matching physical arm dimensions
- [ ] Camera-based pick and place with OpenCV
- [ ] Gesture control using MediaPipe
- [ ] Raspberry Pi for standalone operation
- [ ] STM32 with micro-ROS

---

## References

1. [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
2. [Gazebo Harmonic](https://gazebosim.org/docs)
3. [MoveIt 2](https://moveit.picknik.ai/main/)
4. [Automatic Addison Tutorial](https://automaticaddison.com/)
5. [mycobot_ros2 URDF Source](https://github.com/automaticaddison/mycobot_ros2) (BSD-3-Clause)
6. [Adafruit PCA9685](https://learn.adafruit.com/16-channel-pwm-servo-driver)
7. [Modern Robotics](http://modernrobotics.org/)

---

## License

MIT License. URDF model from [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2) (BSD-3-Clause).
