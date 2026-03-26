#!/usr/bin/env python3
"""
Test node that publishes fake /robotPose messages for testing co-simulation.

This node publishes Odometry messages on /robotPose simulating a robot
moving in a circular or linear path for testing the co-simulation system.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import numpy as np


class FakeRobotPosePublisher(Node):
    """Publishes fake robot poses for testing."""
    
    def __init__(self):
        super().__init__('fake_robot_pose_publisher')
        
        # Parameters
        self.declare_parameter('motion_type', 'circle')  # 'circle', 'linear', 'static'
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('radius', 5.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        
        self.motion_type = self.get_parameter('motion_type').value
        self.rate = self.get_parameter('rate').value
        self.radius = self.get_parameter('radius').value
        self.speed = self.get_parameter('speed').value
        self.start_pos = np.array([
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value,
            self.get_parameter('start_z').value
        ])
        
        # Publisher
        self.pose_pub = self.create_publisher(Odometry, '/robotPose', 10)
        
        # Timer
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.publish_pose)
        
        # State
        self.time = 0.0
        self.dt = timer_period
        
        self.get_logger().info(f'Fake robot pose publisher started')
        self.get_logger().info(f'Motion type: {self.motion_type}')
        self.get_logger().info(f'Publishing at {self.rate} Hz')
    
    def publish_pose(self):
        """Publish fake odometry."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Compute position based on motion type
        if self.motion_type == 'circle':
            # Circular motion
            angle = (self.time * self.speed) / self.radius
            x = self.start_pos[0] + self.radius * math.cos(angle)
            y = self.start_pos[1] + self.radius * math.sin(angle)
            z = self.start_pos[2]
            yaw = angle + math.pi / 2  # Tangent to circle
            
        elif self.motion_type == 'linear':
            # Linear motion along X axis
            x = self.start_pos[0] + self.time * self.speed
            y = self.start_pos[1]
            z = self.start_pos[2]
            yaw = 0.0
            
        else:  # static
            x, y, z = self.start_pos
            yaw = 0.0
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        
        # Convert yaw to quaternion
        quat = self.yaw_to_quaternion(yaw)
        msg.pose.pose.orientation = quat
        
        # Publish
        self.pose_pub.publish(msg)
        
        self.time += self.dt
    
    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert yaw angle to quaternion."""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


def main():
    rclpy.init()
    node = FakeRobotPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
