#!/usr/bin/env python3

from lab6_pkg.laser_costmap_utils import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserCostmap(Node):

    def __init__(self):
        super().__init__("laser_costmap")

        self.__laserscan_subscriber = self.create_subscription(msg_type=LaserScan,
                                                               topic="scan",
                                                               callback=self.__laserscan_callback,
                                                               qos_profile=10)


    def __laserscan_callback(self, laserscan_msg: LaserScan) -> None:
        self.get_logger().info(f"Received laser scan message!")

def main(args=None):
    rclpy.init(args=args)
    laser_costmap_node = LaserCostmap()
    rclpy.spin(laser_costmap_node)
    laser_costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()