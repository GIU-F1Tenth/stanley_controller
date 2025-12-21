#!/usr/bin/env python3

import math
import numpy as np
import csv
import os
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class StanleyController(Node):

    def __init__(self):
        super().__init__('stanley_controller')

        self._declare_parameters()
        self._load_parameters()
        self._initialize_state()
        self._setup_publishers()
        self._setup_subscribers()
        self._load_path()
        self._start_control_loop()

        self.get_logger().info(f'Stanley Controller initialized with {len(self.path)} waypoints')

    def _declare_parameters(self):
        self.declare_parameter('k', 0.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('path_file', '')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('error_topic', '/cross_track_error')

    def _load_parameters(self):
        self.k = self.get_parameter('k').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.freq = self.get_parameter('control_frequency').value
        self.path_file = self.get_parameter('path_file').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.error_topic = self.get_parameter('error_topic').value

    def _initialize_state(self):
        self.pose = None
        self.velocity = None
        self.path = []
        self.last_idx = 0

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)
        self.error_pub = self.create_publisher(Float64, self.error_topic, 10)

    def _setup_subscribers(self):
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)

    def _load_path(self):
        if not self.path_file or not os.path.exists(self.path_file):
            self.get_logger().warn(f'Path file not found: {self.path_file}')
            return

        try:
            with open(self.path_file, 'r') as f:
                reader = csv.reader(f)
                first_row = next(reader, None)
                
                if first_row and self._is_numeric(first_row):
                    self.path.append(self._parse_waypoint(first_row))
                
                for row in reader:
                    if len(row) >= 3 and self._is_numeric(row):
                        self.path.append(self._parse_waypoint(row))
                        
            self.get_logger().info(f'Loaded {len(self.path)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load path: {e}')

    def _is_numeric(self, row):
        try:
            float(row[0])
            float(row[1])
            float(row[2])
            return True
        except:
            return False

    def _parse_waypoint(self, row):
        return {
            'x': float(row[0]),
            'y': float(row[1]),
            'v': float(row[2])
        }

    def _start_control_loop(self):
        self.create_timer(1.0 / self.freq, self._control_step)

    def _odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.velocity = msg.twist.twist

    def _control_step(self):
        if not self.path or self.pose is None:
            return

        front_x, front_y, yaw = self._get_front_axle_state()
        
        target_idx = self._find_target_waypoint(front_x, front_y)
        if target_idx is None:
            return

        cte = self._compute_cross_track_error(front_x, front_y, target_idx)
        heading_error = self._compute_heading_error(yaw, target_idx)
        
        speed = self._get_current_speed()
        steering = self._stanley_control(cte, heading_error, speed)
        
        target_speed = self._compute_target_speed(target_idx, steering)
        
        self._publish_control(steering, target_speed)
        self._publish_error(cte)

    def _get_front_axle_state(self):
        yaw = self._quaternion_to_yaw(self.pose.orientation)
        front_x = self.pose.position.x + self.wheelbase * math.cos(yaw)
        front_y = self.pose.position.y + self.wheelbase * math.sin(yaw)
        return front_x, front_y, yaw

    def _find_target_waypoint(self, x, y):
        min_dist = float('inf')
        closest_idx = self.last_idx
        
        search_start = max(0, self.last_idx - 10)
        search_end = min(len(self.path), self.last_idx + 50)
        
        for i in range(search_start, search_end):
            wp = self.path[i]
            dist = math.hypot(wp['x'] - x, wp['y'] - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.last_idx = closest_idx
        return closest_idx

    def _compute_cross_track_error(self, x, y, idx):
        if idx == 0:
            idx = 1
        if idx >= len(self.path):
            idx = len(self.path) - 1

        p1 = self.path[idx - 1]
        p2 = self.path[idx]

        x1, y1 = p1['x'], p1['y']
        x2, y2 = p2['x'], p2['y']

        dx = x2 - x1
        dy = y2 - y1
        
        cross = (x - x1) * dy - (y - y1) * dx
        
        return cross / (math.hypot(dx, dy) + 1e-6)

    def _compute_heading_error(self, yaw, idx):
        if idx >= len(self.path) - 1:
            idx = len(self.path) - 2
        
        p1 = self.path[idx]
        p2 = self.path[idx + 1]
        
        path_yaw = math.atan2(p2['y'] - p1['y'], p2['x'] - p1['x'])
        
        return self._normalize_angle(path_yaw - yaw)

    def _stanley_control(self, cte, heading_error, speed):
        speed = max(speed, 0.1)
        
        steering = heading_error + math.atan(self.k * cte / speed)
        
        return np.clip(steering, -self.max_steer, self.max_steer)

    def _get_current_speed(self):
        if self.velocity is None:
            return 0.0
        return math.hypot(self.velocity.linear.x, self.velocity.linear.y)

    def _compute_target_speed(self, idx, steering):
        waypoint_speed = self.path[idx]['v']
        
        steer_factor = 1.0 - 0.4 * abs(steering) / self.max_steer
        
        target = waypoint_speed * steer_factor
        
        return np.clip(target, self.min_speed, self.max_speed)

    def _publish_control(self, steering, speed):
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steering
        cmd.drive.speed = speed
        self.cmd_pub.publish(cmd)

    def _publish_error(self, error):
        msg = Float64()
        msg.data = error
        self.error_pub.publish(msg)

    def _quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    controller = StanleyController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
