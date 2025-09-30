#!/bin/bash
# Setup script for Drone Autopilot Simulation

set -e

echo "=========================================="
echo "Drone Autopilot Simulation Setup"
echo "=========================================="
echo ""

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo "Detected Ubuntu version: $UBUNTU_VERSION"

# Determine ROS distribution
if [ "$UBUNTU_VERSION" = "20.04" ]; then
    ROS_DISTRO="noetic"
elif [ "$UBUNTU_VERSION" = "22.04" ]; then
    ROS_DISTRO="humble"
else
    echo "Warning: Unsupported Ubuntu version. Trying with ROS Noetic..."
    ROS_DISTRO="noetic"
fi

echo "Using ROS distribution: $ROS_DISTRO"
echo ""

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install ROS if not already installed
if ! command -v rosversion &> /dev/null; then
    echo "ROS not found. Installing ROS $ROS_DISTRO..."
    
    # Setup sources.list
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # Setup keys
    sudo apt install -y curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
    # Update and install
    sudo apt update
    sudo apt install -y ros-${ROS_DISTRO}-desktop-full
    
    # Initialize rosdep
    sudo apt install -y python3-rosdep
    sudo rosdep init
    rosdep update
    
    # Setup environment
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
    source /opt/ros/${ROS_DISTRO}/setup.bash
    
    echo "ROS $ROS_DISTRO installed successfully!"
else
    echo "ROS already installed: $(rosversion -d)"
fi

# Install dependencies
echo ""
echo "Installing dependencies..."
sudo apt install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rviz \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    git \
    build-essential

# Install Python dependencies
echo ""
echo "Installing Python packages..."
pip3 install --upgrade pip
pip3 install numpy scipy matplotlib pyyaml

# Create workspace if it doesn't exist
WORKSPACE_DIR="$HOME/drone_sim_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo ""
    echo "Creating catkin workspace at $WORKSPACE_DIR..."
    mkdir -p $WORKSPACE_DIR/src
    cd $WORKSPACE_DIR
    source /opt/ros/${ROS_DISTRO}/setup.bash
    catkin_make
    echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
else
    echo ""
    echo "Workspace already exists at $WORKSPACE_DIR"
fi

# Clone repository (if running from script, not from inside repo)
if [ ! -d "$WORKSPACE_DIR/src/drone_autopilot_sim" ]; then
    echo ""
    echo "Would you like to clone the repository? (y/n)"
    read -r CLONE_REPO
    if [ "$CLONE_REPO" = "y" ]; then
        echo "Enter repository URL:"
        read -r REPO_URL
        cd $WORKSPACE_DIR/src
        git clone $REPO_URL
    fi
fi

# Build the workspace
echo ""
echo "Building workspace..."
cd $WORKSPACE_DIR
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make

# Setup Gazebo model path
echo ""
echo "Setting up Gazebo model path..."
GAZEBO_MODEL_PATH_LINE="export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$WORKSPACE_DIR/src/drone_autopilot_sim/models"
if ! grep -q "GAZEBO_MODEL_PATH.*drone_autopilot_sim" ~/.bashrc; then
    echo $GAZEBO_MODEL_PATH_LINE >> ~/.bashrc
fi

# Make scripts executable
echo ""
echo "Making scripts executable..."
if [ -d "$WORKSPACE_DIR/src/drone_autopilot_sim/scripts" ]; then
    chmod +x $WORKSPACE_DIR/src/drone_autopilot_sim/scripts/*.py
fi

# Source the workspace
source $WORKSPACE_DIR/devel/setup.bash

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To use the simulation:"
echo "1. Open a new terminal (to load environment variables)"
echo "2. Run: roslaunch drone_autopilot_sim simulation.launch"
echo "3. In another terminal, run: rosrun drone_autopilot_sim autopilot_controller.py"
echo ""
echo "Or source your workspace manually:"
echo "  source $WORKSPACE_DIR/devel/setup.bash"
echo ""
echo "For more information, see the README.md file"
echo "=========================================="