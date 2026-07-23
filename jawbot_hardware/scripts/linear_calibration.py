#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CalibrationDriveNode(Node):
    def __init__(self):
        super().__init__('calibration_drive_node')
        # Create a publisher to the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Give the publisher a fraction of a second to establish connection to the network
        time.sleep(0.5) 

    def run_calibration(self):
        msg = Twist()
        
        # 1. Drive Forward
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        self.get_logger().info('Driving forward at 0.1 m/s...')
        self.publisher_.publish(msg)
        
        # 2. Wait exactly 10.0 seconds (0.1 m/s * 10 s = 1.0 meter)
        time.sleep(10.0)
        
        # 3. Hard Stop
        msg.linear.x = 0.0
        self.get_logger().info('10 seconds reached. Sending stop command.')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationDriveNode()
    
    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the robot stops even if the script is interrupted
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()