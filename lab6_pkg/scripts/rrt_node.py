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
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from lab6_pkg.rrt_utils import *
from lab6_pkg.laser_costmap_utils import twod_numpy_from_occupancy_grid, project_continuous_point_to_grid, occupancy_grid_from_twod_numpy

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

        # Create goal pose subscriber.
        self.__goal_pose_subscriber = self.create_subscription(msg_type=PoseStamped,
                                                                topic="goal_pose",
                                                                callback=self.__goal_pose_callback,
                                                                qos_profile=10)
        # Variable to store latest copy of received goal pose. NOTE that I
        # don't think you need a lock here, as we're not asynchronously updating
        # this value from multiple threads in this node (at least I don't
        # think).
        self.__goal_pose: PoseStamped = None

        # Create subscriber for the vehicle's pose. This will serve as the
        # starting point for RRT. I.e., where the path will be planned from.
        self.__pose_subscriber = self.create_subscription(msg_type=PoseWithCovarianceStamped,
                                                          topic="pose",
                                                          callback=self.__pose_callback,
                                                          qos_profile=10)
        self.__pose: PoseWithCovarianceStamped = None

        self.__costmap_subscriber = self.create_subscription(msg_type=OccupancyGrid,
                                                                   topic="costmap",
                                                                   callback=self.__costmap_callback,
                                                                   qos_profile=10)
        self.__costmap: OccupancyGrid = None

        # Set up a transform listener and buffer for this node.
        self.__transform_buffer = Buffer()
        self.__transform_listener = TransformListener(buffer=self.__transform_buffer, node=self)

        # NOTE This is just a TEMPORARY occupancy grid publisher FOR DEBUGGING.
        # Just need this to visualize what's going on in the map and make sure
        # things are happening in the right place.
        self.__temp_occ_publisher = self.create_publisher(msg_type=OccupancyGrid, 
                                                          topic="debug_costmap",
                                                          qos_profile=10)

    def __goal_pose_callback(self, goal_pose: PoseStamped) -> None:
        """Callback function for storing the most recently received goal pose
        that RRT will plan a path to.

        Args:
            goal_pose (PoseStamped): (x,y) position that RRT will plan a path to.
        """
        self.__goal_pose = goal_pose
        return
    
    def __pose_callback(self, pose: PoseWithCovarianceStamped) -> None:
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens
         
        Do we need a lock to synchronize access to the occupancy grid?

        Args: 
            pose_msg (PoseWithCovarianceStamped): incoming message from
            subscribed topic 
        """
        self.__pose = pose
        return

    def __costmap_callback(self, costmap: OccupancyGrid) -> None:
        """Plans a path to the latest goal pose using RRT in the received
        costmap. It will transform the published goal pose into the frame of
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
        # TODO
        # While this IS the function where we "run rrt," I don't want this to be
        # the function where all of its parts are integrated and put together.
        # That should be in a separate RRT algorithm function. If it has to be,
        # fine, but that's not ideal. For now, I'll build it in here, but I can
        # always "integrate" (or bring together) all the steps in a separate
        # function later.

        # 1. Attempt transform the most recently received goal pose to the
        #    frame of the received costmap. Bail if not possible or if there
        #    hasn't been a received goal pose yet.
        try:
            # Check if we've received a goal pose.
            if self.__goal_pose is None:
                raise Exception("No goal pose set! Don't have a goal to plan a path to!")
            # If so, get the transform from the goal poses frame to the frame
            # the occupancy grid is in.
            goal_to_grid_transform = self.__transform_buffer.lookup_transform(source_frame=self.__goal_pose.header.frame_id,
                                                                              target_frame=self.__costmap.header.frame_id,
                                                                              time=Time())
            transformed_goal_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose=self.__goal_pose,
                                                                                transform=goal_to_grid_transform)
        except Exception as exc:
            self.get_logger().warning(f"Failed to transform goal pose to costmap's frame.\nException: {str(exc)}")
            return

        # 2. Project the transformed goal pose's position onto the costmap. 
        #    TODO For now, there aren't any protections against trying to
        #    project a point onto the grid and the resulting cell position is
        #    out of bounds. In that case, for now, the RRT node is just going to
        #    ignore that goal position until the vehicle moves enough so that it
        #    IS in bounds, but it won't publish a new path until the goal pose
        #    is in bounds of the occupancy grid. In the future, could use
        #    something like the steer function or Bresenham line function to
        #    get the closest point that IS in free space.
        # goal_grid_coords = project_continous_point_to_grid(grid_resolution_m_c=costmap.info.resolution,
        #                                                    continous_point=)
        continuous_goal_position = (transformed_goal_pose.pose.position.x, transformed_goal_pose.pose.position.y)
        try:
            goal_position = project_continuous_point_to_grid(occupancy_grid=costmap,
                                                            continuous_point=continuous_goal_position)
        except Exception as exc:
            self.get_logger().warning(f"Failed to project goal pose to costmap occupancy grid.\nException: {str(exc)}")
            return
        
        # 3. Convert the occupancy grid's underlying data field to an easier to
        #    work with 2D numpy array.
        numpy_occupancy_grid = twod_numpy_from_occupancy_grid(occupancy_grid=costmap)

        # 4. Create an array of all the free space cell locations in the grid.

        # From here, we run the RRT loop, where we sample a point, find the
        # closest node in the tree (euclidean distance) to this node, compute a
        # new node location on a line from nearest to sampled, check to see if
        # there's a collision between nearest and new using that same line, then
        # add that new node to the tree if not. Repeat.

        # NOTE: As a quick test, let's take the transformed goal pose, get the
        # point inside, project it onto the occupancy 
        numpy_occupancy_grid[goal_position[1], goal_position[0]] = 100
        
        
        # 4. Randomly select a cell from the free space discovered in the
        #    occupancy grid.
        sampled_cell = sample(costmap=numpy_occupancy_grid)
        self.get_logger().info(f"Sampled cell: {sampled_cell}")
        # TODO: Test sampled cell by publishing an updated costmap with this
        # cell Just to visualize.
        numpy_occupancy_grid[sampled_cell[1], sampled_cell[0]] = 100

        costmap.data = occupancy_grid_from_twod_numpy(numpy_occupancy_grid)
        self.__temp_occ_publisher.publish(costmap)


def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRT()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()