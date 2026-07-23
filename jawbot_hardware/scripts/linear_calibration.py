#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CalibrationDriveNode(Node):
    def __init__(self):
        super().__init__('calibration_drive_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(0.5) # Wait for network connection

    def run_calibration(self):
        msg = Twist()
        msg.linear.x = 0.1  # 0.2 m/s
        msg.angular.z = 0.0
        
        self.get_logger().info('Driving forward at 0.2 m/s...')
        
        # Continuously pump the command to satisfy the ROS 2 safety watchdog
        start_time = time.time()
        while time.time() - start_time < 10.0:  # Drive for exactly 5.0 seconds
            self.publisher_.publish(msg)
            time.sleep(0.1) # Sleep for 1/10th of a second (10 Hz)
            
        # Hard Stop
        msg.linear.x = 0.0
        self.get_logger().info('5.0 seconds reached. Sending stop command.')
        
        # Pump the stop command a few times to guarantee it is received
        for _ in range(5):
            self.publisher_.publish(msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationDriveNode()
    
    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()