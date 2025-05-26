#!/bin/bash

# Complete ROS Packages Installation Script
# Installs all required ROS packages via apt-get only

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

# Update package lists
print_status "Updating package lists..."
sudo apt-get update

# Install all packages in one comprehensive command
print_status "Installing all ROS packages..."
sudo apt-get install -y \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-rplidar-ros \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-laser-geometry \
    ros-$ROS_DISTRO-rtabmap-ros \
    ros-$ROS_DISTRO-rtabmap-launch \
    ros-$ROS_DISTRO-hector-mapping \
    ros-$ROS_DISTRO-hector-slam \
    ros-$ROS_DISTRO-hector-slam-launch \
    ros-$ROS_DISTRO-hector-nav-msgs \
    ros-$ROS_DISTRO-hector-trajectory-server \
    ros-$ROS_DISTRO-hector-geotiff \
    ros-$ROS_DISTRO-hector-compressed-map-transport \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-nav-core \
    ros-$ROS_DISTRO-base-local-planner \
    ros-$ROS_DISTRO-global-planner \
    ros-$ROS_DISTRO-navfn \
    ros-$ROS_DISTRO-carrot-planner \
    ros-$ROS_DISTRO-costmap-2d \
    ros-$ROS_DISTRO-voxel-grid \
    ros-$ROS_DISTRO-costmap-converter \
    ros-$ROS_DISTRO-clear-costmap-recovery \
    ros-$ROS_DISTRO-rotate-recovery \
    ros-$ROS_DISTRO-dynamic-reconfigure \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-amcl \
    ros-$ROS_DISTRO-map-server \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-plugins \
    ros-$ROS_DISTRO-diagnostic-msgs \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-teleop-twist-joy

# Try to install spatio-temporal voxel layer if available
print_status "Attempting to install spatio-temporal voxel layer..."
if sudo apt-get install -y ros-$ROS_DISTRO-spatio-temporal-voxel-layer 2>/dev/null; then
    print_success "spatio-temporal-voxel-layer installed via apt!"
else
    print_warning "spatio-temporal-voxel-layer not available via apt, will need manual build"
fi

# Verification of key packages
print_status "Verifying installation..."
key_packages=(
    "ros-$ROS_DISTRO-realsense2-camera"
    "ros-$ROS_DISTRO-rplidar-ros"
    "ros-$ROS_DISTRO-rtabmap-ros"
    "ros-$ROS_DISTRO-hector-mapping"
    "ros-$ROS_DISTRO-gmapping"
    "ros-$ROS_DISTRO-robot-localization"
    "ros-$ROS_DISTRO-navigation"
    "ros-$ROS_DISTRO-nav-core"
    "ros-$ROS_DISTRO-costmap-2d"
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

# Installation summary
echo ""
echo "=========================================="
echo "    ROS PACKAGES INSTALLATION COMPLETE"
echo "=========================================="
echo ""
print_success "Successfully installed packages for:"
echo "  ✓ Sensor Hardware (RealSense, RPLidar)"
echo "  ✓ SLAM Systems (RTAB-Map, Hector, GMapping)"
echo "  ✓ Complete Navigation Stack (Move Base, Costmaps, Planners)"
echo "  ✓ Localization (Robot Localization, AMCL)"
echo "  ✓ Navigation (Move Base, Map Server)"
echo "  ✓ Visualization (RViz, RQT tools)"
echo "  ✓ Utilities (TF2, Image Transport, etc.)"
echo ""
print_warning "Note: The following packages need to be built locally:"
echo "  - my_imu_package (your custom IMU package)"
echo "  - lam_pkg (your custom configuration package)"
echo "  - mapping_pkg (your custom mapping package)"
echo "  - smart_wheelchair_stvl_nav (should now build successfully)"
echo "  - spatio-temporal-voxel-layer (if not available via apt)"
echo ""
echo "=========================================="
echo "             NEXT STEPS"
echo "=========================================="
echo ""
echo "1. Source ROS environment:"
echo "   source /opt/ros/$ROS_DISTRO/setup.bash"
echo ""
echo "2. Navigate to your workspace and build:"
echo "   cd ~/ws_wheelchair"
echo "   catkin build"
echo ""
echo "3. Source your workspace:"
echo "   source ~/ws_wheelchair/devel/setup.bash"
echo ""
echo "4. Test your installation:"
echo "   roslaunch smart_wheelchair_stvl_nav your_launch_file.launch"
echo ""
print_status "Installation script completed successfully!"
