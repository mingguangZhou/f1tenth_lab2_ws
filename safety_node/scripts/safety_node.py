#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # create ROS subscribers and publishers.
        self.drive_publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, "ego_racecar/odom", self.odom_callback, 10)
        self.scan_subscriber_ = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.get_logger().info("Safety node has been started")
        

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # calculate TTC
        # get the array of scan angels
        scan_angles = np.arange(start=scan_msg.angle_min, stop=scan_msg.angle_max, step=scan_msg.angle_increment, dtype=np.float32)
        scan_angles = scan_angles[:len(scan_msg.ranges)]
        # get the array of range rates
        range_rates = self.speed * np.cos(scan_angles)
        # make the rate zero when non-positive
        range_rates = np.maximum(range_rates,0)
        # get the TTC array
        with np.errstate(divide='ignore', invalid='ignore'):
            TTC_array = np.where(np.isnan(scan_msg.ranges), float('inf'), np.where(range_rates!=0, scan_msg.ranges/range_rates, float('inf')))
        
        # publish command to brake
        drive_msg = AckermannDriveStamped ()
        # TTC Condition
        TTC = min(TTC_array)
        if (TTC<=1.5):
            drive_msg.drive.speed = 0.0
            self.drive_publisher_.publish(drive_msg)
            self.get_logger().info("AEB Activated with TCC: " + str(TTC))
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
