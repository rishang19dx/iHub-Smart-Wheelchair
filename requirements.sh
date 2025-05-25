#!/bin/bash

#==============================================================================
# Complete ROS Packages Installation Script
# 
# This script installs all required ROS packages for:
# - SLAM systems (RTAB-Map, Hector SLAM, GMapping)
# - Sensor drivers (RealSense, RPLidar)
# - Localization and navigation
# - Visualization and utilities
#
# Author: [Your Name]
# Date: [Date]
# Version: 1.0
# Supported ROS Distributions: Melodic, Noetic
#==============================================================================

# Colors for output formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ROS_DISTRO is set
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS_DISTRO environment variable is not set!"
    print_status "Please source your ROS setup: source /opt/ros/[distro]/setup.bash"
    exit 1
fi

print_status "Installing packages for ROS $ROS_DISTRO"

#==============================================================================
# SYSTEM UPDATE
#==============================================================================
print_status "Updating package lists..."
sudo apt update

#==============================================================================
# SENSOR HARDWARE PACKAGES
#==============================================================================
print_status "Installing sensor hardware packages..."

# RealSense Camera
print_status "Installing RealSense camera packages..."
sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
sudo apt install -y ros-$ROS_DISTRO-realsense2-description

# RPLidar
print_status "Installing RPLidar packages..."
sudo apt install -y ros-$ROS_DISTRO-rplidar-ros

#==============================================================================
# CORE ROS PACKAGES
#==============================================================================
print_status "Installing core ROS packages..."

# Transform libraries
sudo apt install -y ros-$ROS_DISTRO-tf2-ros
sudo apt install -y ros-$ROS_DISTRO-tf2-geometry-msgs

# Message packages
sudo apt install -y ros-$ROS_DISTRO-sensor-msgs
sudo apt install -y ros-$ROS_DISTRO-geometry-msgs
sudo apt install -y ros-$ROS_DISTRO-nav-msgs

# Image processing
sudo apt install -y ros-$ROS_DISTRO-image-transport
sudo apt install -y ros-$ROS_DISTRO-cv-bridge

# Laser geometry
sudo apt install -y ros-$ROS_DISTRO-laser-geometry

#==============================================================================
# SLAM PACKAGES
#==============================================================================
print_status "Installing SLAM packages..."

# RTAB-Map SLAM
print_status "Installing RTAB-Map packages..."
sudo apt install -y ros-$ROS_DISTRO-rtabmap-ros
sudo apt install -y ros-$ROS_DISTRO-rtabmap-launch

# Hector SLAM
print_status "Installing Hector SLAM packages..."
sudo apt install -y ros-$ROS_DISTRO-hector-mapping
sudo apt install -y ros-$ROS_DISTRO-hector-slam
sudo apt install -y ros-$ROS_DISTRO-hector-slam-launch
sudo apt install -y ros-$ROS_DISTRO-hector-nav-msgs
sudo apt install -y ros-$ROS_DISTRO-hector-trajectory-server
sudo apt install -y ros-$ROS_DISTRO-hector-geotiff
sudo apt install -y ros-$ROS_DISTRO-hector-compressed-map-transport

# GMapping SLAM
print_status "Installing GMapping packages..."
sudo apt install -y ros-$ROS_DISTRO-gmapping

#==============================================================================
# LOCALIZATION AND NAVIGATION
#==============================================================================
print_status "Installing localization and navigation packages..."

# Robot Localization (EKF/UKF)
sudo apt install -y ros-$ROS_DISTRO-robot-localization

# Navigation stack
sudo apt install -y ros-$ROS_DISTRO-move-base
sudo apt install -y ros-$ROS_DISTRO-amcl
sudo apt install -y ros-$ROS_DISTRO-map-server

# IMU filtering
sudo apt install -y ros-$ROS_DISTRO-imu-filter-madgwick

#==============================================================================
# VISUALIZATION AND UTILITIES
#==============================================================================
print_status "Installing visualization and utility packages..."

# RViz
sudo apt install -y ros-$ROS_DISTRO-rviz

# RQT tools
sudo apt install -y ros-$ROS_DISTRO-rqt
sudo apt install -y ros-$ROS_DISTRO-rqt-common-plugins
sudo apt install -y ros-$ROS_DISTRO-rqt-robot-plugins

# Diagnostic tools
sudo apt install -y ros-$ROS_DISTRO-diagnostic-msgs
sudo apt install -y ros-$ROS_DISTRO-diagnostic-updater

#==============================================================================
# INSTALLATION VERIFICATION
#==============================================================================
print_status "Verifying installation..."

# Check if key packages are installed
key_packages=(
    "ros-$ROS_DISTRO-realsense2-camera"
    "ros-$ROS_DISTRO-rplidar-ros"
    "ros-$ROS_DISTRO-rtabmap-ros"
    "ros-$ROS_DISTRO-hector-mapping"
    "ros-$ROS_DISTRO-gmapping"
    "ros-$ROS_DISTRO-robot-localization"
)

failed_packages=()
for package in "${key_packages[@]}"; do
    if ! dpkg -l | grep -q "^ii  $package "; then
        failed_packages+=("$package")
    fi
done

if [ ${#failed_packages[@]} -eq 0 ]; then
    print_success "All key packages installed successfully!"
else
    print_error "The following packages failed to install:"
    for package in "${failed_packages[@]}"; do
        echo "  - $package"
    done
fi

#==============================================================================
# INSTALLATION SUMMARY
#==============================================================================
echo ""
echo "=========================================="
echo "    ROS PACKAGES INSTALLATION COMPLETE"
echo "=========================================="
echo ""
print_success "Successfully installed packages for:"
echo "  ✓ Sensor Hardware (RealSense, RPLidar)"
echo "  ✓ SLAM Systems (RTAB-Map, Hector, GMapping)"
echo "  ✓ Localization (Robot Localization, AMCL)"
echo "  ✓ Navigation (Move Base, Map Server)"
echo "  ✓ Visualization (RViz, RQT tools)"
echo "  ✓ Utilities (TF2, Image Transport, etc.)"
echo ""
print_warning "Note: The following packages need to be built locally:"
echo "  - my_imu_package (your custom IMU package)"
echo "  - lam_pkg (your custom configuration package)"
echo "  - mapping_pkg (your custom mapping package)"
echo ""
echo "=========================================="
echo "             NEXT STEPS"
echo "=========================================="
echo ""
echo "1. Source ROS environment:"
echo "   source /opt/ros/$ROS_DISTRO/setup.bash"
echo ""
echo "2. Navigate to your workspace and build:"
echo "   cd ~/catkin_ws"
echo "   catkin_make"
echo ""
echo "3. Source your workspace:"
echo "   source ~/catkin_ws/devel/setup.bash"
echo ""
echo "4. Test your installation:"
echo "   roslaunch your_package your_launch_file.launch"
echo ""
echo "=========================================="
echo "           USEFUL COMMANDS"
echo "=========================================="
echo ""
echo "• Save map:           rosrun map_server map_saver -f my_map"
echo "• View TF tree:       rosrun rqt_tf_tree rqt_tf_tree"
echo "• Monitor topics:     rostopic list"
echo "• Check nodes:        rosnode list"
echo "• RQT tools:          rqt"
echo "• System diagnostics: rosrun rqt_runtime_monitor rqt_runtime_monitor"
echo ""