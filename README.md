# PX4 Thrust Vector Control (TVC) Workspace

[![ROS 2](https://img.shields.io/badge/ROS-2-blue)](https://docs.ros.org/en/humble/index.html)
[![PX4](https://img.shields.io/badge/PX4-Compatible-green)](https://px4.io/)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

This ROS 2 workspace provides a complete thrust vector control (TVC) implementation for PX4-based vehicles. The workspace includes PX4-ROS 2 communication bridges, message definitions, and an LQR (Linear Quadratic Regulator) controller for precise thrust vector control.

## 🏗️ Workspace Structure

```
px4_ws/
├── README.md              # This file
├── run.sh                 # Automated build and launch script
└── src/
    ├── px4_msgs/          # PX4 message definitions for ROS 2
    ├── px4_ros_com/       # PX4-ROS 2 communication bridge
    └── tvc_controller/    # Main TVC controller package
        ├── config/        # Configuration files
        ├── launch/        # Launch files
        ├── models/        # Vehicle/system models
        └── tvc_controller/
            ├── lqr.py                    # LQR controller implementation
            └── lqr_controller_node.py    # Main controller ROS 2 node
```

## 📋 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS (recommended) or Ubuntu 24.04 LTS
- **ROS 2**: Humble Hawksbill (recommended) or Galactic Geochelone
- **Python**: 3.10+
- **PX4**: v1.15.4 (SITL or hardware)
- **GZ**: Harmonic (recommended) or Ionic

### Dependencies
- ROS 2 (with `colcon` build tools)
- PX4 Autopilot with uXRCE-DDS bridge
- Python packages:
  - `numpy`
  - `scipy`
  - `rclpy`

## 🚀 Quick Start

### 1. Clone and Setup
```bash
# Navigate to your workspace
cd ~/px4_ws

# Install ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python dependencies
pip3 install numpy scipy
```

### 2. Build the Workspace
```bash
# Make the run script executable
chmod +x run.sh

# Build and launch (automated)
./run.sh
```

Or manually:
```bash
# Build the workspace
colcon build

# Source the setup file
source install/setup.bash

# Launch the TVC controller
ros2 launch tvc_controller lqr.launch.py
```

### 3. Connect to PX4
Ensure PX4 is running with the uXRCE-DDS bridge:
```bash
# For SITL
MicroXRCEAgent udp4 -p 8888
```

### 4. Bind ROS2 clock to GZ
```bash
# Bind ROS2 and GZ clock
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

``` 

## 🎮 Usage

### TVC Controller Node
The main controller node (`lqr_controller_node.py`) provides:
- **LQR-based control**: Linear Quadratic Regulator for optimal control
- **PX4 integration**: Direct communication with PX4 autopilot
- **Thrust vectoring**: Precise control of thrust magnitude and direction
- **Attitude control**: 6-DOF attitude and position control

### Topics and Services
The controller subscribes to:
- `/fmu/out/vehicle_odometry` - Vehicle position and velocity
- `/fmu/out/vehicle_attitude` - Vehicle attitude
- `/fmu/out/vehicle_status` - Vehicle status

The controller publishes to:
- `/fmu/in/offboard_control_mode` - Offboard control mode
- `/fmu/in/vehicle_command` - Vehicle commands
- `/fmu/in/vehicle_thrust_setpoint` - Thrust commands
- `/fmu/in/vehicle_torque_setpoint` - Torque commands



## 📊 Coordinate Frames
This implementation uses standard aerospace coordinate conventions:
- **NED Frame**: North-East-Down for position and linear velocity
- **FRD Frame**: Forward-Right-Down for angular rates and body frame


## 📚 Additional Resources

- [PX4 Documentation](https://docs.px4.io/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [uXRCE-DDS Guide](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [PX4-ROS 2 Interface](https://docs.px4.io/main/en/ros/ros2_comm.html)

---

**Maintainer**: yash.27.agarwal@gmail.com  
**Last Updated**: July 2025
