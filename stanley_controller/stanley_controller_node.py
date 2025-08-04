#!/usr/bin/env python3

import math
import numpy as np
import csv
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan


class StanleyController(Node):
    """
    High-Speed Stanley Controller Node for F1Tenth autonomous racing.

    Optimized for high-speed racing with:
    - Look-ahead path tracking
    - Path-based cross track error calculation
    - Adaptive gain control
    - CSV path loading with velocity profiles
    - Front axle reference point
    """

    def __init__(self):
        super().__init__('stanley_controller')

        # Declare parameters optimized for F1Tenth high-speed racing
        # Higher gain for responsive control
        self.declare_parameter('k_p', 0.8)
        self.declare_parameter('k_adaptive_min', 0.5)  # Minimum adaptive gain
        self.declare_parameter('k_adaptive_max', 1.5)  # Maximum adaptive gain
        # Lookahead distance (m)
        self.declare_parameter('lookahead_distance', 1.5)
        # Speed-dependent lookahead
        self.declare_parameter('lookahead_speed_factor', 0.3)
        # Higher max speed for F1Tenth
        self.declare_parameter('max_speed', 8.0)
        self.declare_parameter('min_speed', 1.0)  # Higher min speed
        self.declare_parameter('wheelbase', 0.3302)  # F1Tenth wheelbase
        # Max steering angle (rad)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('path_csv_file', '')  # CSV file path
        # Use front axle reference
        self.declare_parameter('use_front_axle', True)
        self.declare_parameter('speed_smoothing_factor',
                               0.8)  # Speed command smoothing
        # Damping for oscillations
        self.declare_parameter('cross_track_damping', 0.1)
        # Control loop frequency (Hz)
        self.declare_parameter('control_frequency', 50.0)
        # Maximum allowed cross track error (m)
        self.declare_parameter('max_cross_track_error', 3.0)
        # Distance to brake in emergency (m)
        self.declare_parameter('emergency_brake_distance', 0.8)
        self.declare_parameter(
            'odom_topic', '/ego_racecar/odom')  # Odometry topic
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')  # Drive topic
        self.declare_parameter(
            'error_topic', '/cross_track_error')  # Error topic
        self.declare_parameter('enable_emergency_brake', False)

        # Get parameters
        self.k_p = self.get_parameter('k_p').get_parameter_value().double_value
        self.k_adaptive_min = self.get_parameter(
            'k_adaptive_min').get_parameter_value().double_value
        self.k_adaptive_max = self.get_parameter(
            'k_adaptive_max').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter(
            'lookahead_distance').get_parameter_value().double_value
        self.lookahead_speed_factor = self.get_parameter(
            'lookahead_speed_factor').get_parameter_value().double_value
        self.max_speed = self.get_parameter(
            'max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter(
            'min_speed').get_parameter_value().double_value
        self.wheelbase = self.get_parameter(
            'wheelbase').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter(
            'max_steering_angle').get_parameter_value().double_value
        self.path_csv_file = self.get_parameter(
            'path_csv_file').get_parameter_value().string_value
        self.use_front_axle = self.get_parameter(
            'use_front_axle').get_parameter_value().bool_value
        self.speed_smoothing_factor = self.get_parameter(
            'speed_smoothing_factor').get_parameter_value().double_value
        self.cross_track_damping = self.get_parameter(
            'cross_track_damping').get_parameter_value().double_value
        self.control_frequency = self.get_parameter(
            'control_frequency').get_parameter_value().double_value
        self.max_cross_track_error = self.get_parameter(
            'max_cross_track_error').get_parameter_value().double_value
        self.emergency_brake_distance = self.get_parameter(
            'emergency_brake_distance').get_parameter_value().double_value
        self.odom_topic = self.get_parameter(
            'odom_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value
        self.error_topic = self.get_parameter(
            'error_topic').get_parameter_value().string_value
        self.enable_emergency_brake = self.get_parameter(
            'enable_emergency_brake').get_parameter_value().bool_value

        # State variables
        self.current_pose = None
        self.current_velocity = None
        self.reference_path = []  # List of waypoints with x, y, v
        self.current_waypoint_index = 0
        self.previous_cross_track_error = 0.0
        self.previous_time = self.get_clock().now()
        self.previous_speed_command = 0.0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.cross_track_error_pub = self.create_publisher(
            Float64,
            self.error_topic,
            10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # Load CSV path if provided
        if self.path_csv_file:
            self.load_csv_path()

        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop)  # Use parameter frequency

        self.get_logger().info('High-Speed Stanley Controller Node initialized')
        if self.reference_path:
            self.get_logger().info(
                f'Loaded {len(self.reference_path)} waypoints from CSV')

    def load_csv_path(self):
        """Load reference path from CSV file with x, y, v columns."""
        try:
            if not os.path.exists(self.path_csv_file):
                self.get_logger().error(
                    f'CSV file not found: {self.path_csv_file}')
                return

            self.reference_path = []
            with open(self.path_csv_file, 'r') as file:
                csv_reader = csv.reader(file)
                # Skip header if present
                first_row = next(csv_reader, None)
                if first_row and not self._is_numeric_row(first_row):
                    pass  # Header row, already skipped
                else:
                    # First row is data, process it
                    if first_row:
                        x, y, v = map(float, first_row[:3])
                        self.reference_path.append({'x': x, 'y': y, 'v': v})

                # Process remaining rows
                for row in csv_reader:
                    if len(row) >= 3:
                        x, y, v = map(float, row[:3])
                        self.reference_path.append({'x': x, 'y': y, 'v': v})

            self.get_logger().info(
                f'Successfully loaded {len(self.reference_path)} waypoints from CSV')

        except Exception as e:
            self.get_logger().error(f'Error loading CSV file: {str(e)}')
            self.reference_path = []

    def _is_numeric_row(self, row):
        """Check if a row contains numeric data."""
        try:
            if len(row) >= 3:
                float(row[0])
                float(row[1])
                float(row[2])
                return True
        except ValueError:
            pass
        return False

    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def get_front_axle_pose(self):
        """Get the pose of the front axle for more accurate control."""
        if self.current_pose is None:
            return None

        if not self.use_front_axle:
            return self.current_pose

        current_yaw = self.get_yaw_from_quaternion(
            self.current_pose.orientation)

        # Front axle position
        front_x = self.current_pose.position.x + \
            self.wheelbase * math.cos(current_yaw)
        front_y = self.current_pose.position.y + \
            self.wheelbase * math.sin(current_yaw)

        # Create front axle pose
        front_pose = type(self.current_pose)()
        front_pose.position.x = front_x
        front_pose.position.y = front_y
        front_pose.position.z = self.current_pose.position.z
        front_pose.orientation = self.current_pose.orientation

        return front_pose

    def find_lookahead_waypoint(self):
        """Find waypoint at optimal lookahead distance for high-speed control."""
        if not self.reference_path or self.current_pose is None:
            return None, 0

        reference_pose = self.get_front_axle_pose() or self.current_pose
        current_x = reference_pose.position.x
        current_y = reference_pose.position.y

        # Calculate speed-dependent lookahead distance
        current_speed = 0.0
        if self.current_velocity is not None:
            current_speed = math.sqrt(
                self.current_velocity.linear.x**2 +
                self.current_velocity.linear.y**2
            )

        dynamic_lookahead = self.lookahead_distance + \
            self.lookahead_speed_factor * current_speed

        # Find the closest waypoint first
        min_distance = float('inf')
        closest_index = 0

        for i, waypoint in enumerate(self.reference_path):
            dx = waypoint['x'] - current_x
            dy = waypoint['y'] - current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < min_distance:
                min_distance = distance
                closest_index = i

        # Update current waypoint index
        self.current_waypoint_index = closest_index

        # Find lookahead waypoint
        for i in range(closest_index, len(self.reference_path)):
            waypoint = self.reference_path[i]
            dx = waypoint['x'] - current_x
            dy = waypoint['y'] - current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= dynamic_lookahead:
                return waypoint, i

        # Return last waypoint if none found at lookahead distance
        return self.reference_path[-1], len(self.reference_path) - 1

    def calculate_path_based_cross_track_error(self, target_waypoint, target_index):
        """Calculate cross track error as perpendicular distance to path segment."""
        if not self.reference_path or self.current_pose is None or target_index == 0:
            return 0.0

        reference_pose = self.get_front_axle_pose() or self.current_pose
        current_x = reference_pose.position.x
        current_y = reference_pose.position.y

        # Get current and previous waypoint to form path segment
        if target_index >= len(self.reference_path):
            target_index = len(self.reference_path) - 1

        prev_index = max(0, target_index - 1)

        p1 = self.reference_path[prev_index]
        p2 = self.reference_path[target_index]

        x1, y1 = p1['x'], p1['y']
        x2, y2 = p2['x'], p2['y']

        # Calculate perpendicular distance to line segment
        A = current_x - x1
        B = current_y - y1
        C = x2 - x1
        D = y2 - y1

        dot = A * C + B * D
        len_sq = C * C + D * D

        if len_sq == 0:  # Points are the same
            return math.sqrt(A * A + B * B)

        param = dot / len_sq

        if param < 0:
            xx, yy = x1, y1
        elif param > 1:
            xx, yy = x2, y2
        else:
            xx = x1 + param * C
            yy = y1 + param * D

        dx = current_x - xx
        dy = current_y - yy

        # Determine sign using cross product (left = positive, right = negative)
        cross_product = (x2 - x1) * (current_y - y1) - \
            (y2 - y1) * (current_x - x1)
        sign = 1 if cross_product > 0 else -1

        return sign * math.sqrt(dx * dx + dy * dy)

    def calculate_heading_error(self, target_waypoint, target_index):
        """Calculate heading error using path tangent direction."""
        if not self.reference_path or self.current_pose is None:
            return 0.0

        # Current heading
        current_yaw = self.get_yaw_from_quaternion(
            self.current_pose.orientation)

        # Calculate path direction using multiple points for smoothness
        if target_index < len(self.reference_path) - 1:
            next_waypoint = self.reference_path[target_index + 1]
            dx = next_waypoint['x'] - target_waypoint['x']
            dy = next_waypoint['y'] - target_waypoint['y']
        else:
            # Use previous point for direction
            if target_index > 0:
                prev_waypoint = self.reference_path[target_index - 1]
                dx = target_waypoint['x'] - prev_waypoint['x']
                dy = target_waypoint['y'] - prev_waypoint['y']
            else:
                return 0.0

        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - current_yaw)

        return heading_error

    def adaptive_stanley_control(self, cross_track_error, heading_error, speed, curvature=0.0):
        """
        Adaptive Stanley controller with speed and curvature dependent gains.

        Args:
            cross_track_error: Perpendicular distance from vehicle to path
            heading_error: Difference between vehicle heading and path direction
            speed: Current vehicle speed
            curvature: Path curvature (optional)

        Returns:
            steering_angle: Computed steering angle in radians
        """
        if speed < 0.1:
            speed = 0.1

        # Adaptive gain based on speed and curvature
        speed_factor = np.clip(2.0 / max(speed, 0.5), 0.5, 2.0)
        curvature_factor = 1.0 + 0.5 * abs(curvature)

        k_adaptive = np.clip(
            self.k_p * speed_factor * curvature_factor,
            self.k_adaptive_min,
            self.k_adaptive_max
        )

        # Add damping term for high-speed stability
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9

        if dt > 0 and dt < 0.1:  # Valid time step
            cross_track_error_rate = (
                cross_track_error - self.previous_cross_track_error) / dt
            damping_term = self.cross_track_damping * cross_track_error_rate
        else:
            damping_term = 0.0

        # Stanley controller with adaptive gain and damping
        steering_angle = (heading_error +
                          math.atan(k_adaptive * cross_track_error / speed) +
                          damping_term)

        # Update for next iteration
        self.previous_cross_track_error = cross_track_error
        self.previous_time = current_time

        # Clamp steering angle
        steering_angle = np.clip(
            steering_angle, -self.max_steering_angle, self.max_steering_angle)

        return steering_angle

    def calculate_target_speed(self, target_waypoint, steering_angle):
        """Calculate target speed based on waypoint velocity and steering angle."""
        if target_waypoint is None:
            return self.min_speed

        # Use velocity from CSV if available
        csv_speed = target_waypoint.get('v', self.max_speed)

        # Reduce speed based on steering angle for stability
        steering_factor = 1.0 - 0.3 * \
            abs(steering_angle) / self.max_steering_angle

        # Calculate final target speed
        target_speed = csv_speed * steering_factor
        target_speed = np.clip(target_speed, self.min_speed, self.max_speed)

        # Apply speed smoothing to avoid jerky acceleration
        smoothed_speed = (self.speed_smoothing_factor * self.previous_speed_command +
                          (1 - self.speed_smoothing_factor) * target_speed)

        self.previous_speed_command = smoothed_speed

        return smoothed_speed

    def convert_to_cmd_vel(self, steering_angle, target_speed):
        """Convert steering angle and speed to cmd_vel Twist message."""
        cmd_vel = Twist()

        # Linear velocity
        cmd_vel.linear.x = target_speed
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        # Angular velocity using bicycle model
        # ω = v * tan(δ) / L
        if abs(steering_angle) > 0.001:  # Avoid division by zero
            angular_velocity = target_speed * \
                math.tan(steering_angle) / self.wheelbase
        else:
            angular_velocity = 0.0

        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = angular_velocity

        return cmd_vel

    def control_loop(self):
        """Main high-speed control loop with advanced Stanley implementation."""
        if not self.reference_path or self.current_pose is None:
            return

        # Find optimal lookahead waypoint
        target_waypoint, target_index = self.find_lookahead_waypoint()
        if target_waypoint is None:
            return

        # Calculate path-based cross track error
        cross_track_error = self.calculate_path_based_cross_track_error(
            target_waypoint, target_index)

        # Safety check: limit cross track error
        if abs(cross_track_error) > self.max_cross_track_error:
            self.get_logger().warn(
                f'Cross track error {cross_track_error:.3f} exceeds maximum {self.max_cross_track_error:.3f}')
            # Clamp the error to prevent excessive steering
            cross_track_error = math.copysign(
                self.max_cross_track_error, cross_track_error)

        # Calculate heading error using path tangent
        heading_error = self.calculate_heading_error(
            target_waypoint, target_index)

        # Get current speed
        current_speed = 1.0  # Default
        if self.current_velocity is not None:
            current_speed = math.sqrt(
                self.current_velocity.linear.x**2 +
                self.current_velocity.linear.y**2
            )

        # Calculate path curvature for adaptive control (simplified)
        curvature = 0.0
        if target_index < len(self.reference_path) - 1 and target_index > 0:
            # Simple curvature estimation using three points
            p1 = self.reference_path[target_index - 1]
            p2 = self.reference_path[target_index]
            p3 = self.reference_path[target_index + 1]

            # Calculate curvature using circumcircle method
            dx1, dy1 = p2['x'] - p1['x'], p2['y'] - p1['y']
            dx2, dy2 = p3['x'] - p2['x'], p3['y'] - p2['y']

            if abs(dx1) > 1e-6 and abs(dx2) > 1e-6:
                curvature = abs((dx1 * dy2 - dy1 * dx2) /
                                (math.sqrt(dx1**2 + dy1**2) * math.sqrt(dx2**2 + dy2**2) + 1e-6))

        # Apply adaptive Stanley controller
        steering_angle = self.adaptive_stanley_control(
            cross_track_error, heading_error, current_speed, curvature)

        # Calculate target speed from CSV and steering
        target_speed = self.calculate_target_speed(
            target_waypoint, steering_angle)

        # Safety check: emergency braking if too close to path boundary
        if self.enable_emergency_brake and abs(cross_track_error) > self.emergency_brake_distance:
            target_speed = min(target_speed, self.min_speed)
            self.get_logger().warn(
                f'Emergency speed reduction due to cross track error: {cross_track_error:.3f}')

        # Convert to cmd_vel message
        cmd_vel_msg = self.convert_to_cmd_vel(steering_angle, target_speed)
        self.cmd_vel_pub.publish(cmd_vel_msg)

        # Publish cross track error for debugging
        error_msg = Float64()
        error_msg.data = cross_track_error
        self.cross_track_error_pub.publish(error_msg)

        # Log debug information (reduced frequency for high-speed)
        if target_index % 25 == 0:  # Log every 25 waypoints
            self.get_logger().info(
                f'WP: {target_index}/{len(self.reference_path)}, '
                f'CTE: {cross_track_error:.3f}, '
                f'HE: {heading_error:.3f}, '
                f'Steer: {steering_angle:.3f}, '
                f'Speed: {target_speed:.2f}, '
                f'CSV_v: {target_waypoint.get("v", 0.0):.2f}'
            )

    def get_yaw_from_quaternion(self, quaternion):
        """Convert quaternion to yaw angle."""
        q = quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    stanley_controller = StanleyController()

    try:
        rclpy.spin(stanley_controller)
    except KeyboardInterrupt:
        pass
    finally:
        stanley_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
