#!/usr/bin/env python3
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path

from lab6_pkg.laser_costmap_utils import *

class LaserCostmap(Node):

    def __init__(self):
        super().__init__("laser_costmap")

        # Set up / declare node parameters.
        self.declare_parameters(namespace="",
                                parameters=[
                                    ("height_c", 75),
                                    ("width_c", 100),
                                    ("resolution_m_c", 0.05),
                                    ("splat_radius_c", 4)
                                ])
        self.__height_c = self.get_parameter("height_c").value
        self.__width_c = self.get_parameter("width_c").value
        self.__resolution_m_c = self.get_parameter("resolution_m_c").value
        self.__splat_radius_c = self.get_parameter("splat_radius_c").value

        # Set up subscriber for incoming LaserScan messages.
        self.__laserscan_subscriber = self.create_subscription(msg_type=LaserScan,
                                                               topic="scan",
                                                               callback=self.__laserscan_callback,
                                                               qos_profile=10)
        # # Create subscriber for vehicle's pose, or whatever pose will serve as
        # # the created occupancy grid's origin.
        # self.__pose_subscriber = self.create_subscription(msg_type=PoseWithCovarianceStamped,
        #                                                   topic="pose",
        #                                                   callback=self.__pose_callback,
        #                                                   qos_profile=10)

        # # Create a pose variable and associated mutex that we'll use to access
        # # it. NOTE: May not actually need the lock unless we're running these in
        # # multiple threads, but adding it here for completeness.
        # self.__pose: Pose = None
        # self.__pose_lock = Lock()

        # Create publisher to publish the latest generated local costmap.
        self.__laser_local_costmap_publisher = self.create_publisher(msg_type=OccupancyGrid,
                                                                     topic="laser_local_costmap",
                                                                     qos_profile=10)
    
    # def __pose_callback(self, pose_with_covariance_stamped_msg: PoseWithCovarianceStamped) -> None:
    #     """Callback function that synchronously updates the local copy of our
    #     pose.

    #     Args:
    #         posestamped_msg (PoseStamped): Received PoseStamped message. 
    #     """
    #     with self.__pose_lock:
    #         self.__pose = pose_with_covariance_stamped_msg.pose.pose
    #     self.get_logger().debug("Received new pose!")

    def __laserscan_callback(self, laserscan_msg: LaserScan) -> None:

        try:
            self.get_logger().info("Received new laser scan!")
            # For now, just create a brand new occupancy grid and pass it into the
            # utils function to see what happens. need to set origin of that grid.
            new_grid = OccupancyGrid()
            # Fill out the new grid's header. Set its parent frame equal to the
            # laser's frame (the frame that the laser scans are w.r.t.).
            new_grid.header.stamp = self.get_clock().now().to_msg()
            new_grid.header.frame_id = "ego_racecar/base_link"
            # Fill out the occupancy grid's metadata info.
            new_grid.info.height = self.__height_c
            height_m = new_grid.info.height*self.__resolution_m_c
            new_grid.info.width = self.__width_c
            width_m = new_grid.info.width*self.__resolution_m_c
            new_grid.info.resolution = self.__resolution_m_c
            # Specify the relative pose of the occupancy grid with respect to
            # the origin of the parent frame_id that we specify below.
            new_point = Point()
            new_point.x = 0.0
            new_point.y = -(height_m/2.0)
            new_point.z = 0.0
            new_pose = Pose()
            new_pose.position = new_point
            new_grid.info.origin = new_pose
            # Temporarily populate the occupancy grid data with zeros as if
            # there were values from the previous timestep.
            new_grid.data = np.zeros(shape=(new_grid.info.height, new_grid.info.width), dtype=np.int8).flatten().tolist()
            # Call helper function to actually project laserscan ranges on the
            # occupancy grid.
            updated_grid = laser_update_occupancy_grid_temp(scan_message=laserscan_msg,
                                                            current_occupancy_grid=new_grid,
                                                            splat_radius=self.__splat_radius_c)
            # Publish the updated grid.
            self.__laser_local_costmap_publisher.publish(updated_grid)
            self.get_logger().debug(f"Published new local occupancy grid!")
        except Exception as exc:
            self.get_logger().warning(f"Failed to generate costmap from laserscan message.\nException: {str(exc.with_traceback())}")

def main(args=None):
    rclpy.init(args=args)
    laser_costmap_node = LaserCostmap()
    rclpy.spin(laser_costmap_node)
    laser_costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()