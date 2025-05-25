# Smart Wheelchair STVL Navigation Package

A comprehensive ROS Noetic package for autonomous navigation of smart wheelchairs using the Spatio-Temporal Voxel Layer (STVL) for real-time 3D perception and obstacle avoidance in dynamic environments.

## Features

- **Real-time STVL Navigation**: Advanced 3D perception with temporal decay for dynamic obstacle handling
- **Hardware Integration**: Direct interfacing with wheelchair motors, encoders, LiDAR, and IMU sensors
- **Multi-sensor Fusion**: Support for LiDAR, RGB-D cameras, and inertial sensors
- **Safety Systems**: Emergency stop functionality and collision avoidance
- **Autonomous Navigation**: Complete navigation stack with path planning and localization
- **Hardware-focused Design**: Optimized for real wheelchair hardware, not simulation

## Prerequisites

### Software Requirements
- **ROS Noetic** (Desktop Full installation)
- **Navigation Stack**: `sudo apt install ros-noetic-navigation`
- **STVL Plugin**: `sudo apt install ros-noetic-spatio-temporal-voxel-layer`
- **Robot Localization**: `sudo apt install ros-noetic-robot-localization`
- **AMCL**: `sudo apt install ros-noetic-amcl`
- **TEB Local Planner**: `sudo apt install ros-noetic-teb-local-planner`

### Hardware Requirements
- Smart wheelchair with motor controllers (compatible with serial/CAN communication)
- 2D/3D LiDAR sensor
- IMU sensor (e.g., Phidgets Spatial)
- Wheel encoders
- Optional: RGB-D camera for enhanced perception
- Computing platform (laptop/embedded computer with sufficient processing power)

### Hardware Setup
- Motor controllers accessible via `/dev/ttyUSB0` (configurable)
- LiDAR connected via `/dev/ttyUSB1` (configurable)
- Encoders connected to GPIO pins (if using Raspberry Pi)
- CAN interface for advanced motor controllers (optional)

## Installation

1. **Create catkin workspace** (if not already existing):
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. **Clone the repository**:
```bash
git clone https://github.com/your-username/smart_wheelchair_stvl_nav.git
cd ~/catkin_ws
```

3. **Install dependencies**:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. **Build the workspace**:
```bash
catkin_make
source devel/setup.bash
```

5. **Configure hardware parameters** in `config/` files according to your specific hardware setup.

## Hardware Configuration

### Motor Controller Setup
Configure your motor controller parameters in `config/wheelchair_navigation.yaml`:
- Wheel separation distance
- Wheel radius
- Serial port and baud rate
- Motor controller type and communication protocol

### Sensor Configuration
Update sensor parameters in `config/stvl_hardware_params.yaml`:
- LiDAR topic and frame configurations
- Camera parameters (if using RGB-D)
- IMU settings and calibration
- Transform tree relationships

## Usage

### Basic Hardware Testing

1. **Test motor controllers**:
```bash
roslaunch smart_wheelchair_stvl_nav wheelchair_hardware.launch
```

2. **Verify sensor data**:
```bash
rostopic echo /scan
rostopic echo /odom
rostopic echo /imu/data
```

### Full Navigation System

1. **Launch complete navigation stack**:
```bash
roslaunch smart_wheelchair_stvl_nav wheelchair_stvl_navigation.launch
```

2. **Visualize in RViz**:
```bash
rviz -d $(rospack find smart_wheelchair_stvl_nav)/rviz/wheelchair_navigation.rviz
```

3. **Send navigation goals** using RViz 2D Nav Goal tool or programmatically:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

### Emergency Stop
Activate emergency stop at any time:
```bash
rostopic pub /emergency_stop std_msgs/Bool "data: true"
```

## Package Structure

```
smart_wheelchair_stvl_nav/
├── config/
│   ├── stvl_hardware_params.yaml      # STVL costmap configuration
│   ├── wheelchair_navigation.yaml     # Navigation parameters
│   ├── base_local_planner_params.yaml # Local planner settings
│   └── ekf_template.yaml             # Sensor fusion configuration
├── launch/
│   ├── wheelchair_hardware.launch     # Hardware drivers
│   ├── wheelchair_stvl_navigation.launch # Complete navigation
│   └── mapping.launch                 # SLAM functionality
├── src/
│   ├── wheelchair_motor_driver.cpp    # Motor control interface
│   └── emergency_stop_handler.cpp     # Safety systems
├── scripts/
    ├── lidar_hardware_node.py        # LiDAR hardware interface
    └── encoder_odometry_node.py      # Encoder-based odometry

```

## Spatio-Temporal Voxel Layer (STVL)
The STVL provides several advantages for wheelchair navigation[4]:
- **Temporal Decay**: Obstacles that disappear are gradually removed from the map
- **3D Perception**: Full 3D understanding of the environment
- **Multi-sensor Fusion**: Combines data from multiple sensor types
- **Dynamic Environment Handling**: Adapts to moving people and objects

## Troubleshooting

### Common Issues

**Motor controllers not responding**:
- Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate and communication protocol
- Test hardware connections

**LiDAR data not publishing**:
- Confirm LiDAR power and USB connection
- Check device permissions and port configuration
- Verify frame_id matches tf tree

**Navigation not working**:
- Ensure proper tf tree: `rosrun tf view_frames`
- Check costmap visualization in RViz
- Verify AMCL localization is functioning

**STVL performance issues**:
- Reduce voxel resolution for better performance
- Adjust observation sources and ranges
- Monitor CPU usage and optimize accordingly

## Contributing

Contributions are welcome! Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly with real hardware
4. Submit a pull request with detailed description

---

**Note**: This package is designed for real hardware deployment. Ensure proper hardware setup and safety measures before operation.

Citations:
[1] https://github.com/tinymovr/Tinymovr-ROS
[2] http://wiki.ros.org/navigation
[3] https://arxiv.org/html/2501.09680v1
[4] https://github.com/mich-pest/ros2_navigation_stvl/blob/master/README.md
[5] https://www.csus.edu/indiv/t/tatror/senior_design/SD%20F17-S18/Team_1_Automated_Smart_Wheelchair.pdf
[6] https://index.ros.org/p/hardware_interface/
[7] https://github.com/chvmp/champ/wiki/Hardware-Integration
[8] https://www.linkedin.com/advice/0/how-do-you-coordinate-ros-developers-users-hardware-integration
[9] https://index.ros.org/p/hardware_interface_testing/
[10] http://wiki.ros.org/ROS/Tutorials/Creating%20a%20Simple%20Hardware%20Driver