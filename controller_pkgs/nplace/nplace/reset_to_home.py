#!/usr/bin/env python3
"""
Reset Doosan E0509 robot to home position using Doosan API.
"""

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
import sys


class ResetToHome(Node):
    def __init__(self):
        super().__init__('reset_to_home')
        
        # Home position (matching Isaac Lab default)
        # [joint1, joint2, joint3, joint4, joint5, joint6] in degrees
        self.home_position_deg = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        
        # self.home_position_deg = [-0.000 * 57.2958, 0.296* 57.2958, 1.571* 57.2958, 0.203* 57.2958, 1.275* 57.2958, 0.296* 57.2958]
        # Create service client for MoveJoint
        self.movej_client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        
        self.get_logger().info('Waiting for MoveJoint service...')
        if not self.movej_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('MoveJoint service not available!')
            sys.exit(1)
        
        self.get_logger().info('MoveJoint service available!')
    
    def move_to_home(self, velocity=30.0, acceleration=10.0):
        """
        Move robot to home position using MoveJoint service.
        
        Args:
            velocity: Joint velocity in deg/s (default: 60 deg/s)
            acceleration: Joint acceleration in deg/s² (default: 100 deg/s²)
        """
        request = MoveJoint.Request()
        
        # Set target position
        request.pos = self.home_position_deg
        
        # Set velocity and acceleration (deg/s and deg/s²)
        request.vel = velocity
        request.acc = acceleration
        
        # Movement mode
        request.time = 0.0      # Not used for MoveJoint
        request.mode = 0        # Absolute position mode
        request.blend_type = 0  # No blending
        request.sync_type = 0   # Synchronous movement
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Moving to HOME position:')
        self.get_logger().info(f'  Target: {self.home_position_deg} deg')
        self.get_logger().info(f'  Velocity: {velocity} deg/s')
        self.get_logger().info(f'  Acceleration: {acceleration} deg/s²')
        self.get_logger().info('=' * 60)
        
        # Call service
        future = self.movej_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            self.get_logger().info('✅ Successfully moved to HOME position!')
            return True
        else:
            self.get_logger().error('❌ Failed to move to HOME position!')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = ResetToHome()
    
    try:
        # Move to home with moderate speed
        success = node.move_to_home(velocity=30.0, acceleration=10.0)
        
        if success:
            node.get_logger().info('Robot is now at HOME position.')
        else:
            node.get_logger().error('Failed to reset robot.')
            
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
