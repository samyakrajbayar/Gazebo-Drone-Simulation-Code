# Autonomous Quadrotor Drone Simulation in Gazebo

A comprehensive simulation environment for autonomous quadrotor drones with wind effects, obstacle structures, and autopilot capabilities using ROS and Gazebo.

## Features

- **Autonomous Flight**: Pre-programmed waypoint navigation with PID control
- **Wind Simulation**: Realistic wind forces affecting drone flight dynamics
- **3D Environment**: Multiple structures (buildings, towers) for obstacle avoidance testing
- **Real-time Telemetry**: Position, velocity, and attitude monitoring
- **Sensor Integration**: IMU, GPS, and altitude sensors
- **Customizable Parameters**: Easy configuration of flight paths, wind conditions, and drone properties

## Prerequisites

### Required Software
- Ubuntu 20.04 or 22.04
- ROS Noetic (20.04) or ROS2 Humble (22.04)
- Gazebo 11
- Python 3.8+

### Installation

```bash
# Install ROS Noetic (Ubuntu 20.04)
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-gazebo-ros-pkgs
sudo apt install ros-noetic-gazebo-ros-control

# Install additional dependencies
sudo apt install python3-pip
pip3 install numpy scipy matplotlib

# Create workspace
mkdir -p ~/drone_sim_ws/src
cd ~/drone_sim_ws/src
```

## Repository Structure

```
drone_autopilot_sim/
├── README.md
├── models/
│   ├── quadrotor/
│   │   ├── model.sdf
│   │   └── model.config
│   └── structures/
│       ├── building/
│       └── tower/
├── worlds/
│   └── drone_world.world
├── scripts/
│   ├── autopilot_controller.py
│   └── wind_plugin.py
├── launch/
│   └── simulation.launch
├── config/
│   └── waypoints.yaml
└── plugins/
    └── wind_force_plugin.cc
```

## Quick Start

### 1. Clone the Repository

```bash
cd ~/drone_sim_ws/src
git clone https://github.com/yourusername/drone_autopilot_sim.git
cd ~/drone_sim_ws
catkin_make  # or colcon build for ROS2
source devel/setup.bash
```

### 2. Launch the Simulation

```bash
# Basic simulation
roslaunch drone_autopilot_sim simulation.launch

# With custom wind settings
roslaunch drone_autopilot_sim simulation.launch wind_speed:=5.0 wind_direction:=45

# With GUI disabled (headless)
roslaunch drone_autopilot_sim simulation.launch gui:=false
```

### 3. Start Autopilot

```bash
# In a new terminal
source ~/drone_sim_ws/devel/setup.bash
rosrun drone_autopilot_sim autopilot_controller.py
```

## Configuration

### Waypoints Configuration

Edit `config/waypoints.yaml` to define your flight path:

```yaml
waypoints:
  - x: 0.0
    y: 0.0
    z: 5.0
  - x: 10.0
    y: 10.0
    z: 5.0
  - x: 20.0
    y: 0.0
    z: 8.0
  - x: 0.0
    y: 0.0
    z: 5.0

tolerance: 0.5  # meters
max_velocity: 3.0  # m/s
```

### Wind Parameters

Modify wind settings in `worlds/drone_world.world`:

```xml
<plugin name="wind_plugin" filename="libwind_force_plugin.so">
  <wind_speed>3.0</wind_speed>
  <wind_direction>90</wind_direction>
  <turbulence>0.2</turbulence>
</plugin>
```

### Drone Parameters

Adjust PID gains in `scripts/autopilot_controller.py`:

```python
# Position PID gains
PID_X = {'kp': 1.5, 'ki': 0.01, 'kd': 0.8}
PID_Y = {'kp': 1.5, 'ki': 0.01, 'kd': 0.8}
PID_Z = {'kp': 2.0, 'ki': 0.02, 'kd': 1.0}

# Attitude PID gains
PID_ROLL = {'kp': 4.0, 'ki': 0.0, 'kd': 2.0}
PID_PITCH = {'kp': 4.0, 'ki': 0.0, 'kd': 2.0}
PID_YAW = {'kp': 3.0, 'ki': 0.0, 'kd': 1.5}
```

## Usage Examples

### Example 1: Basic Square Flight Pattern

```bash
roslaunch drone_autopilot_sim simulation.launch
# Wait for Gazebo to load
rosrun drone_autopilot_sim autopilot_controller.py --pattern square --size 10
```

### Example 2: High Wind Conditions

```bash
roslaunch drone_autopilot_sim simulation.launch wind_speed:=8.0 wind_direction:=180
rosrun drone_autopilot_sim autopilot_controller.py --robust_mode
```

### Example 3: Custom Waypoint Mission

```bash
# Edit config/waypoints.yaml first
roslaunch drone_autopilot_sim simulation.launch
rosrun drone_autopilot_sim autopilot_controller.py --waypoints config/waypoints.yaml
```

## ROS Topics

### Subscribed Topics
- `/drone/imu` (sensor_msgs/Imu): IMU data
- `/drone/gps` (sensor_msgs/NavSatFix): GPS position
- `/drone/altitude` (std_msgs/Float64): Altitude sensor

### Published Topics
- `/drone/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/drone/motor_speed` (std_msgs/Float64MultiArray): Individual motor speeds
- `/drone/pose` (geometry_msgs/PoseStamped): Current drone pose
- `/drone/status` (std_msgs/String): Autopilot status

## Architecture

### Control System
The autopilot uses a cascaded PID control architecture:
1. **Outer Loop**: Position control (X, Y, Z)
2. **Inner Loop**: Attitude control (Roll, Pitch, Yaw)
3. **Motor Mixing**: Converts desired forces to motor speeds

### Wind Simulation
Wind forces are applied using a Gazebo plugin that simulates:
- Constant wind velocity
- Turbulence with Dryden wind model
- Altitude-dependent wind profiles

## Troubleshooting

### Drone Not Taking Off
- Check if autopilot controller is running
- Verify waypoints are at positive Z coordinates
- Increase PID_Z gains if drone is underpowered

### Unstable Flight
- Reduce PID gains gradually
- Check for excessive wind speeds
- Verify mass and inertia properties in model.sdf

### Gazebo Crashes
- Reduce simulation complexity (fewer structures)
- Check available RAM (Gazebo requires 4GB+)
- Update graphics drivers

### No Wind Effect Visible
- Verify wind plugin is loaded: `gz topic -l | grep wind`
- Check plugin compilation: `ls ~/drone_sim_ws/devel/lib/`
- Increase wind_speed parameter

## Performance Optimization

For better performance:
```bash
# Reduce physics update rate in world file
<max_step_size>0.01</max_step_size>
<real_time_update_rate>100</real_time_update_rate>

# Run headless
roslaunch drone_autopilot_sim simulation.launch gui:=false

# Reduce sensor frequencies
<update_rate>50</update_rate>  # Instead of 100+
```

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push to branch: `git push origin feature-name`
5. Submit a pull request

## License

MIT License - see LICENSE file for details

## References

- [Gazebo Documentation](http://gazebosim.org/tutorials)
- [ROS Wiki](http://wiki.ros.org)
- [PX4 Autopilot](https://docs.px4.io/)
- [RotorS Simulator](https://github.com/ethz-asl/rotors_simulator)


