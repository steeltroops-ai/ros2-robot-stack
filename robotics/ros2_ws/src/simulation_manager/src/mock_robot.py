#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import random

class MockRobot(Node):

    def __init__(self):
        super().__init__('mock_robot')
        
        # Declare parameters (namespace is handled by launch file, but we can have local ID)
        self.declare_parameter('robot_id', 'robot_1')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        
        # Simulation State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.battery = 100.0
        self.battery_drain_rate = 0.05  # % per second
        
        # Timer (10Hz simulation loop)
        self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Mock Robot "{self.robot_id}" Started. Simulating movement.')

    def timer_callback(self):
        # 1. Physics Update (Circular Motion)
        t = self.get_clock().now().nanoseconds / 1e9
        radius = 5.0
        speed = 0.5
        
        self.x = radius * math.cos(speed * t)
        self.y = radius * math.sin(speed * t)
        
        # Calculate theta (tangent to circle)
        self.theta = (speed * t) + (math.pi / 2)
        
        # 2. Battery Update
        self.battery -= (self.battery_drain_rate * 0.1)
        if self.battery < 0:
            self.battery = 100.0 # Charge!
            
        # 3. Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = f"{self.robot_id}_base_link"
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Simple Euler to Quaternion (Z-axis rotation only)
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        self.odom_pub.publish(odom_msg)
        
        # 4. Publish Battery
        bat_msg = BatteryState()
        bat_msg.percentage = self.battery
        bat_msg.voltage = 24.0
        bat_msg.current = 2.5
        bat_msg.charge = self.battery # Using charge field for simplicity
        
        self.battery_pub.publish(bat_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
