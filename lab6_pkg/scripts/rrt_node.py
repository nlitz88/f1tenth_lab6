#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

from lab6_pkg.rrt_utils import *

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class TreeNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__("rrt")

        # Set up / declare node parameters.
        self.declare_parameters(namespace="",
                                parameters=[
                                    ("path_frame", rclpy.Parameter.Type.STRING)
                                ])
        self.__path_frame = self.get_parameter("path_frame").value

        # Create goal point subscriber.
        self.__goal_point_subscriber = self.create_subscription(msg_type=Point,
                                                                topic="goal_point",
                                                                callback=self.__goal_point_callback,
                                                                qos_profile=10)
        # Variable to store latest copy of received goal point. NOTE that I
        # don't think you need a lock here, as we're not asynchronously updating
        # this value from multiple threads in this node (at least I don't
        # think).
        self.__goal_point: Point = None

        # Create subscriber for the vehicle's pose. This will serve as the
        # starting point for RRT. I.e., where the path will be planned from.
        self.__pose_subscriber = self.create_subscription(msg_type=PoseStamped,
                                                          topic="pose",
                                                          callback=self.__pose_callback,
                                                          qos_profile=10)
        self.__pose: PoseStamped = None

        self.__costmap_subscriber = self.create_subscription(msg_type=OccupancyGrid,
                                                                   topic="costmap",
                                                                   callback=self.__costmap_callback,
                                                                   qos_profile=10)
        self.__costmap: OccupancyGrid = None

    def __goal_point_callback(self, goal_point: Point) -> None:
        """Callback function for storing the most recently received goal point
        that RRT will plan a path to.

        Args:
            goal_point (Point): (x,y) position that RRT will plan a path to.
        """
        self.__goal_point = goal_point
        return
    
    def __pose_callback(self, pose: PoseStamped) -> None:
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens
         
        Do we need a lock to synchronize access to the occupancy grid?

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        self.__pose = pose
        return

    def __costmap_callback(self, costmap: OccupancyGrid) -> None:
        """Plans a path to the latest goal point using RRT in the received
        costmap. It will transform the published goal point into the frame of
        the received costmap, plan a path in the received costmap to the goal
        point using RRT, and then publish a resulting Path in the costmap's
        frame or the frame specified by the path_frame parameter. It'd probably
        be best to set that parameter to whatever frame your path tracker is
        expecting the Path to be in, especially if it can't transform those
        points itself.

        Args:
            costmap (OccupancyGrid): 2D costmap that RRT will be run in. 
        """
        # Store local copy of costmap.
        self.__costmap = costmap

        # While this IS the function where we "run rrt," I don't want this to be
        # the function where all of its parts are integrated and put together.
        # That should be in a separate RRT algorithm function. If it has to be,
        # fine, but that's not ideal.

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRT()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()