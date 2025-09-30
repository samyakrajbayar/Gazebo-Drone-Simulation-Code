#!/usr/bin/env python3
"""
Autonomous Quadrotor Autopilot Controller
Implements cascaded PID control for waypoint navigation
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Odometry
import tf.transformations as tft
import yaml
import argparse


class PIDController:
    """Simple PID controller implementation"""
    
    def __init__(self, kp, ki, kd, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def update(self, error, current_time):
        """Update PID controller with new error"""
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0
            
        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0.0:
            return 0.0
            
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        if self.output_limits:
            self.integral = np.clip(self.integral, 
                                   self.output_limits[0] / self.ki if self.ki != 0 else -1000,
                                   self.output_limits[1] / self.ki if self.ki != 0 else 1000)
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
            
        self.prev_error = error
        self.prev_time = current_time
        
        return output
        
    def reset(self):
        """Reset PID controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


class QuadrotorAutopilot:
    """Autonomous quadrotor controller with waypoint navigation"""
    
    def __init__(self):
        rospy.init_node('quadrotor_autopilot', anonymous=True)
        
        # Drone state
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        
        # Flight parameters
        self.mass = 1.5  # kg
        self.gravity = 9.81
        self.max_tilt = np.deg2rad(30)  # max roll/pitch angle
        self.max_velocity = 3.0  # m/s
        self.waypoint_tolerance = 0.5  # meters
        
        # Waypoints
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.mission_complete = False
        
        # PID Controllers - Position (outer loop)
        self.pid_x = PIDController(1.5, 0.01, 0.8, output_limits=(-5, 5))
        self.pid_y = PIDController(1.5, 0.01, 0.8, output_limits=(-5, 5))
        self.pid_z = PIDController(2.0, 0.02, 1.0, output_limits=(-10, 10))
        
        # PID Controllers - Attitude (inner loop)
        self.pid_roll = PIDController(4.0, 0.0, 2.0, output_limits=(-self.max_tilt, self.max_tilt))
        self.pid_pitch = PIDController(4.0, 0.0, 2.0, output_limits=(-self.max_tilt, self.max_tilt))
        self.pid_yaw = PIDController(3.0, 0.0, 1.5, output_limits=(-1.0, 1.0))
        
        # ROS Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.motor_speed_pub = rospy.Publisher('/motor_speed', Float64MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('/autopilot/status', String, queue_size=10)
        self.pose_pub = rospy.Publisher('/autopilot/target_pose', PoseStamped, queue_size=10)
        
        # ROS Subscribers
        rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        # Load waypoints
        self.load_waypoints()
        
        # Control rate
        self.rate = rospy.Rate(50)  # 50 Hz
        
        rospy.loginfo("Quadrotor Autopilot Initialized")
        self.publish_status("INITIALIZED")
        
    def load_waypoints(self):
        """Load waypoints from parameter server or use defaults"""
        try:
            waypoints_param = rospy.get_param('~waypoints', None)
            if waypoints_param:
                for wp in waypoints_param:
                    self.waypoints.append(np.array([wp['x'], wp['y'], wp['z']]))
            else:
                # Default square pattern
                self.waypoints = [
                    np.array([0.0, 0.0, 5.0]),
                    np.array([10.0, 0.0, 5.0]),
                    np.array([10.0, 10.0, 5.0]),
                    np.array([0.0, 10.0, 5.0]),
                    np.array([0.0, 0.0, 5.0])
                ]
            rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints")
        except Exception as e:
            rospy.logerr(f"Error loading waypoints: {e}")
            self.waypoints = [np.array([0.0, 0.0, 5.0])]
            
    def odom_callback(self, msg):
        """Update drone state from odometry"""
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        self.orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
    def imu_callback(self, msg):
        """Update angular velocity from IMU"""
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
    def get_euler_from_quaternion(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        euler = tft.euler_from_quaternion(self.orientation)
        return np.array(euler)
        
    def compute_control(self):
        """Compute control commands using cascaded PID"""
        if self.mission_complete or len(self.waypoints) == 0:
            return np.zeros(4)  # [thrust, roll, pitch, yaw_rate]
            
        # Get current target waypoint
        target = self.waypoints[self.current_waypoint_idx]
        
        # Check if waypoint reached
        distance = np.linalg.norm(self.position - target)
        if distance < self.waypoint_tolerance:
            rospy.loginfo(f"Waypoint {self.current_waypoint_idx} reached!")
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= len(self.waypoints):
                rospy.loginfo("Mission complete!")
                self.mission_complete = True
                self.publish_status("MISSION_COMPLETE")
                return np.zeros(4)
                
            self.publish_status(f"WAYPOINT_{self.current_waypoint_idx}")
            
        # Position errors
        error_pos = target - self.position
        
        # Get current orientation
        roll, pitch, yaw = self.get_euler_from_quaternion()
        
        # Outer loop: Position control -> desired accelerations
        current_time = rospy.Time.now()
        acc_x_des = self.pid_x.update(error_pos[0], current_time)
        acc_y_des = self.pid_y.update(error_pos[1], current_time)
        acc_z_des = self.pid_z.update(error_pos[2], current_time)
        
        # Convert desired accelerations to thrust and desired angles
        # Thrust (compensate for gravity + vertical acceleration)
        thrust = self.mass * (self.gravity + acc_z_des)
        
        # Desired roll and pitch from horizontal accelerations
        # Small angle approximation: acc_x ≈ g * theta, acc_y ≈ -g * phi
        pitch_des = np.arctan2(acc_x_des, self.gravity)
        roll_des = np.arctan2(-acc_y_des, self.gravity)
        
        # Limit desired angles
        pitch_des = np.clip(pitch_des, -self.max_tilt, self.max_tilt)
        roll_des = np.clip(roll_des, -self.max_tilt, self.max_tilt)
        
        # Desired yaw (point towards waypoint)
        yaw_des = np.arctan2(error_pos[1], error_pos[0])
        
        # Inner loop: Attitude control
        roll_cmd = self.pid_roll.update(roll_des - roll, current_time)
        pitch_cmd = self.pid_pitch.update(pitch_des - pitch, current_time)
        
        # Yaw error with wrapping
        yaw_error = yaw_des - yaw
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        yaw_rate_cmd = self.pid_yaw.update(yaw_error, current_time)
        
        return np.array([thrust, roll_cmd, pitch_cmd, yaw_rate_cmd])
        
    def control_to_motor_speeds(self, control):
        """Convert control commands to motor speeds"""
        thrust, roll_cmd, pitch_cmd, yaw_rate_cmd = control
        
        # Simplified motor mixing for quadrotor (X configuration)
        # Motors: 0=front-left, 1=front-right, 2=rear-left, 3=rear-right
        base_speed = np.sqrt(thrust / (4 * self.mass * self.gravity)) if thrust > 0 else 0
        
        motor_speeds = np.array([
            base_speed + pitch_cmd + roll_cmd + yaw_rate_cmd,  # Front-left
            base_speed + pitch_cmd - roll_cmd - yaw_rate_cmd,  # Front-right
            base_speed - pitch_cmd + roll_cmd - yaw_rate_cmd,  # Rear-left
            base_speed - pitch_cmd - roll_cmd + yaw_rate_cmd   # Rear-right
        ])
        
        # Normalize and limit motor speeds (0-1000 range for typical ESCs)
        motor_speeds = np.clip(motor_speeds * 1000, 0, 1000)
        
        return motor_speeds
        
    def publish_commands(self, control):
        """Publish control commands"""
        # Publish as Twist message
        cmd_vel = Twist()
        cmd_vel.linear.z = control[0] / self.mass  # acceleration
        cmd_vel.angular.x = control[1]  # roll rate
        cmd_vel.angular.y = control[2]  # pitch rate
        cmd_vel.angular.z = control[3]  # yaw rate
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Publish motor speeds
        motor_speeds = self.control_to_motor_speeds(control)
        motor_msg = Float64MultiArray()
        motor_msg.data = motor_speeds.tolist()
        self.motor_speed_pub.publish(motor_msg)
        
        # Publish target pose for visualization
        if not self.mission_complete and len(self.waypoints) > 0:
            target = self.waypoints[self.current_waypoint_idx]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = target[0]
            pose_msg.pose.position.y = target[1]
            pose_msg.pose.position.z = target[2]
            self.pose_pub.publish(pose_msg)
            
    def publish_status(self, status):
        """Publish autopilot status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting autopilot control loop")
        self.publish_status("RUNNING")
        
        while not rospy.is_shutdown():
            # Compute control
            control = self.compute_control()
            
            # Publish commands
            self.publish_commands(control)
            
            # Log progress
            if not self.mission_complete and len(self.waypoints) > 0:
                target = self.waypoints[self.current_waypoint_idx]
                distance = np.linalg.norm(self.position - target)
                rospy.loginfo_throttle(2.0, 
                    f"WP {self.current_waypoint_idx}: "
                    f"Pos=[{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}] "
                    f"Dist={distance:.2f}m")
            
            self.rate.sleep()


def main():
    parser = argparse.ArgumentParser(description='Quadrotor Autopilot Controller')
    parser.add_argument('--waypoints', type=str, help='Path to waypoints YAML file')
    parser.add_argument('--pattern', type=str, choices=['square', 'circle', 'figure8'],
                       help='Predefined flight pattern')
    args, _ = parser.parse_known_args()
    
    try:
        autopilot = QuadrotorAutopilot()
        autopilot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autopilot shutting down")


if __name__ == '__main__':
    main()
