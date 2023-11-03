#!/usr/bin/env python3
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path

from lab6_pkg.laser_costmap_utils import *

class LaserCostmap(Node):

    def __init__(self):
        super().__init__("laser_costmap")

        # Set up / declare node parameters.

        # Set up subscriber for incoming LaserScan messages.
        self.__laserscan_subscriber = self.create_subscription(msg_type=LaserScan,
                                                               topic="scan",
                                                               callback=self.__laserscan_callback,
                                                               qos_profile=10)
        # Create subscriber for vehicle's pose, or whatever pose will serve as
        # the created occupancy grid's origin.
        self.__pose_subscriber = self.create_subscription(msg_type=PoseStamped,
                                                          topic="pose",
                                                          callback=self.__pose_callback)
        # Create a pose variable and associated mutex that we'll use to access
        # it. NOTE: May not actually need the lock unless we're running these in
        # multiple threads, but adding it here for completeness.
        self.__posestamped: PoseStamped = None
        self.__posestamped_lock = Lock()

        # Create publisher for publishing locally planned path.
        self.__path_publisher = self.create_publisher(msg_type=Path,
                                                      topic="path",
                                                      qos_profile=10)
    
    def __pose_callback(self, posestamped_msg: PoseStamped) -> None:
        """Callback function that synchronously updates the local copy of our
        pose.

        Args:
            posestamped_msg (PoseStamped): Received PoseStamped message. 
        """
        with self.__posestamped_lock:
            self.__posestamped = posestamped_msg

    def __laserscan_callback(self, laserscan_msg: LaserScan) -> None:
        # For now, just create a brand new occupancy grid and pass it into the
        # utils function to see what happens. need to set origin of that grid.
        new_grid = OccupancyGrid()
        # TODO THESE ARE ALL VALUES THAT SHOULD BE PARAMETERIZED FOR THIS NODE.
        new_grid.info.height = 500
        new_grid.info.width = 500
        new_grid.info.resolution = 0.05
        with self.__posestamped_lock:
            new_grid.info.origin = self.__posestamped.pose
        # Fill out the new grid's header. Set its parent frame equal to the
        # laser's frame (the frame that the laser scans are w.r.t.).
        new_grid.header.stamp = self.get_clock().now().to_msg()
        new_grid.header.frame_id = laserscan_msg.header.frame_id

        

        self.get_logger().info(f"Published new local occupancy grid!")

def main(args=None):
    rclpy.init(args=args)
    laser_costmap_node = LaserCostmap()
    rclpy.spin(laser_costmap_node)
    laser_costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()