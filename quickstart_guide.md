# Quick Start Guide - Drone Autopilot Simulation

## Installation (5 minutes)

### Option 1: Automated Setup
```bash
# Download and run setup script
chmod +x setup.sh
./setup.sh
```

### Option 2: Manual Setup
```bash
# 1. Install ROS Noetic (Ubuntu 20.04)
sudo apt update
sudo apt install ros-noetic-desktop-full

# 2. Install dependencies
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
                 python3-pip python3-numpy python3-scipy

# 3. Create workspace
mkdir -p ~/drone_sim_ws/src
cd ~/drone_sim_ws/src
git clone <your-repo-url>
cd ~/drone_sim_ws
catkin_make
source devel/setup.bash
```

## Running the Simulation (2 minutes)

### Terminal 1: Launch Gazebo
```bash
source ~/drone_sim_ws/devel/setup.bash
roslaunch drone_autopilot_sim simulation.launch
```
Wait for Gazebo to fully load (you'll see the drone and structures).

### Terminal 2: Start Autopilot
```bash
source ~/drone_sim_ws/devel/setup.bash
rosrun drone_autopilot_sim autopilot_controller.py
```

The drone will automatically:
1. Take off to 5m altitude
2. Navigate through predefined waypoints
3. Avoid obstacles (buildings and towers)
4. Return to starting position

## Customizing Your Flight

### Change Waypoints
Edit `config/waypoints.yaml`:
```yaml
waypoints:
  - x: 0.0
    y: 0.0
    z: 5.0
  - x: 10.0
    y: 10.0
    z: 7.0
```

Then restart the autopilot.

### Adjust Wind Conditions
```bash
roslaunch drone_autopilot_sim simulation.launch wind_speed:=8.0 wind_direction:=90
```
- `wind_speed`: Speed in m/s (0-15 recommended)
- `wind_direction`: Direction in degrees (0=North, 90=East, 180=South, 270=West)

### Use Predefined Patterns
```bash
# Square pattern
rosrun drone_autopilot_sim autopilot_controller.py --pattern square --size 10

# Circle pattern
rosrun drone_autopilot_sim autopilot_controller.py --pattern circle

# Figure-8 pattern
rosrun drone_autopilot_sim autopilot_controller.py --pattern figure8
```

## Monitoring Flight Data

### View Topics
```bash
# List all active topics
rostopic list

# Monitor drone position
rostopic echo /ground_truth/state

# Monitor target waypoint
rostopic echo /autopilot/target_pose

# Monitor wind
rostopic echo /wind
```

### Plot Data in Real-time
```bash
rqt_plot /ground_truth/state/pose/pose/position/z /autopilot/target_pose/pose/position/z
```

### Visualize in RViz
```bash
rviz
# Add displays for:
# - TF (shows drone orientation)
# - Pose (shows target waypoint)
# - Path (shows trajectory)
```

## Troubleshooting

### Drone Doesn't Take Off
```bash
# Check if autopilot is running
rosnode list | grep autopilot

# Check waypoints are loaded
rosparam get /waypoints

# Increase thrust (edit autopilot_controller.py)
self.pid_z = PIDController(3.0, 0.03, 1.5)  # Increase kp
```

### Unstable Flight
```bash
# Reduce wind
roslaunch drone_autopilot_sim simulation.launch wind_speed:=1.0

# Lower PID gains (edit autopilot_controller.py)
self.pid_roll = PIDController(3.0, 0.0, 1.5)  # Reduce kp
```

### Gazebo Runs Slowly
```bash
# Run headless (no GUI)
roslaunch drone_autopilot_sim simulation.launch gui:=false

# Reduce update rate (edit drone_world.world)
<real_time_update_rate>500</real_time_update_rate>
```

## Testing Wind Effects

### Light Breeze (2-3 m/s)
```bash
roslaunch drone_autopilot_sim simulation.launch wind_speed:=2.5
```
Drone should maintain stable flight with minor corrections.

### Moderate Wind (5-7 m/s)
```bash
roslaunch drone_autopilot_sim simulation.launch wind_speed:=6.0
```
Noticeable drift; autopilot actively compensates.

### Strong Wind (8-12 m/s)
```bash
roslaunch drone_autopilot_sim simulation.launch wind_speed:=10.0
```
Challenging conditions; test autopilot robustness.

## Advanced Features

### Record Flight Data
```bash
# Record all topics
rosbag record -a

# Record specific topics
rosbag record /ground_truth/state /imu /autopilot/target_pose
```

### Playback Recorded Data
```bash
rosbag play <your_bag_file.bag>
```

### Add Custom Obstacles
Edit `worlds/drone_world.world` and add:
```xml
<model name="my_obstacle">
  <static>true</static>
  <pose>X Y Z 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>W H D</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>W H D</size></box>
      </geometry>
    </visual>
  </link>
</model>
```

## Next Steps

1. **Tune PID Controllers**: Optimize for your specific scenario
2. **Add Sensors**: GPS, rangefinders, cameras
3. **Implement SLAM**: For autonomous mapping
4. **Add Path Planning**: A* or RRT algorithms
5. **Multiple Drones**: Swarm simulation

## Getting Help

- Check logs: `~/.ros/log/latest/`
- Gazebo issues: `gzserver --verbose`
- ROS issues: `roswtf`
- GitHub Issues: Report bugs and request features

## Useful Commands Cheat Sheet

```bash
# Stop all ROS nodes
rosnode kill -a

# Reset simulation
rosservice call /gazebo/reset_simulation

# Pause simulation
rosservice call /gazebo/pause_physics

# Resume simulation
rosservice call /gazebo/unpause_physics

# Check node info
rosnode info /quadrotor_autopilot

# View message definition
rosmsg show geometry_msgs/Twist
```

Happy Flying! 🚁