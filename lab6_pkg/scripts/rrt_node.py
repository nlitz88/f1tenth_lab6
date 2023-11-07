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

        self.__costmap_subscriber = self.create_subscription(msg_type=OccupancyGrid,
                                                                   topic="costmap",
                                                                   callback=self.__costmap_callback,
                                                                   qos_profile=10)
        self.__costmap: OccupancyGrid = None
        
        # Create subscriber for the vehicle's pose. This will serve as the
        # starting point for RRT. I.e., where the path will be planned from.
        self.__pose_subscriber = self.create_subscription(msg_type=PoseStamped,
                                                          topic="pose",
                                                          callback=self.__pose_callback,
                                                          qos_profile=10)
        self.__pose: PoseStamped = None

    def __goal_point_callback(self, goal_point: Point) -> None:
        """Callback function for storing the most recently received goal point
        that RRT will plan a path to.

        Args:
            goal_point (Point): (x,y) position that RRT will plan a path to.
        """
        pass

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
        # NOTE: This is the function where the planning algorithm will be
        # invoked. I.e., whenever we receive an updated costmap, that's when
        # we'll replan with RRT.
        pass
    
    def __pose_callback(self, pose: PoseStamped) -> None:
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens
         
        Do we need a lock to synchronize access to the occupancy grid?

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        pass


    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable
        point
        
        Maybe we can basically just say "get me a point that itself doesn't land
        on an occupied cell. I bet there's a more intelligent way of obtaining
        this 2D position, rather than just trial and error. 

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = None
        y = None
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = None
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        # NOTE: Can probably use Bresenham's line algorithm to help us with
        # this--probably the most straightforward.
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRT()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()