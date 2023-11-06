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

        # Set up subscriber for incoming LaserScan messages.
        self.__laserscan_subscriber = self.create_subscription(msg_type=LaserScan,
                                                               topic="scan",
                                                               callback=self.__laserscan_callback,
                                                               qos_profile=10)
        # Create subscriber for vehicle's pose, or whatever pose will serve as
        # the created occupancy grid's origin.
        self.__pose_subscriber = self.create_subscription(msg_type=PoseWithCovarianceStamped,
                                                          topic="pose",
                                                          callback=self.__pose_callback,
                                                          qos_profile=10)
        # Create a pose variable and associated mutex that we'll use to access
        # it. NOTE: May not actually need the lock unless we're running these in
        # multiple threads, but adding it here for completeness.
        self.__pose: Pose = None
        self.__pose_lock = Lock()

        # Create publisher to publish the latest generated local costmap.
        self.__laser_local_costmap_publisher = self.create_publisher(msg_type=OccupancyGrid,
                                                                     topic="laser_local_costmap",
                                                                     qos_profile=10)
    
    def __pose_callback(self, pose_with_covariance_stamped_msg: PoseWithCovarianceStamped) -> None:
        """Callback function that synchronously updates the local copy of our
        pose.

        Args:
            posestamped_msg (PoseStamped): Received PoseStamped message. 
        """
        with self.__pose_lock:
            self.__pose = pose_with_covariance_stamped_msg.pose.pose
        self.get_logger().info("Received new pose!")

    def __laserscan_callback(self, laserscan_msg: LaserScan) -> None:

        try:
            self.get_logger().info("Received new laser scan!")
            # For now, just create a brand new occupancy grid and pass it into the
            # utils function to see what happens. need to set origin of that grid.
            new_grid = OccupancyGrid()
            # TODO THESE ARE ALL VALUES THAT SHOULD BE PARAMETERIZED FOR THIS NODE.
            new_grid.info.height = 200
            height_m = new_grid.info.height*0.05
            new_grid.info.width = 200
            width_m = new_grid.info.width*0.05
            new_grid.info.resolution = 0.05
            # Specify the relative pose of the occupancy grid with respect to
            # the origin of the parent frame_id that we specify below! In this
            # case, I'll just place it at the origin of the base link frame.
            # NOW, to center the underlying 2D grid at the origin of the parent
            # frame, we just need to set its origin to be half of its height and
            # width away from the parent frame origin. BUT CAREFUL--not its
            # pixel/cell height and width away, but its height and width away in
            # meters--as that is what the relative pose is specified in terms
            # of!!! So, will have to convert.
            new_point = Point()
            new_point.x = 0.0
            new_point.y = -(height_m/2.0)
            new_point.z = 0.0
            new_pose = Pose()
            new_pose.position = new_point
            new_grid.info.origin = new_pose
            # Fill out the new grid's header. Set its parent frame equal to the
            # laser's frame (the frame that the laser scans are w.r.t.).
            new_grid.header.stamp = self.get_clock().now().to_msg()
            # new_grid.header.frame_id = laserscan_msg.header.frame_id #
            # WAIT--do laser scan values NOT have a frame??? No, nevermind,
            # that's not the issue.
            # THIS IS THE PROBLEM with the weird position. We're treating THE
            # POINTS in the occupancy grid as if they're in the base link's
            # frame. Therefore, if we're publishing this grid with a parent
            # frame id equal to the car's frame, then its origin that we specify
            # must be WITH RESPECT TO THE FRAME IT'S in. Therefore, we don't
            # actually need to get the pose of the car in this case--we only
            # need to know the RELATIVE pose of the occupancy grid with respect
            # to the car's base_link. I.e., how we want to offset it. And
            # ideally, the occupancy grid should be offset by the transform
            # between the laser frame and the base link frame. Technically, we
            # could even just say that the occupancy grid is in the frame of the
            # laser! That may just be easier for our simple application--but
            # I'll use base link first.
            new_grid.header.frame_id = "ego_racecar/base_link"
            # Temporarily populate the occupancy grid data with zeros as if
            # there were values from the previous timestep.
            new_grid.data = np.zeros(shape=(new_grid.info.height, new_grid.info.width), dtype=np.int8).flatten().tolist()
            # Call helper function to actually project laserscan ranges on the
            # occupancy grid.
            updated_grid = laser_update_occupancy_grid_temp(scan_message=laserscan_msg,
                                                            current_occupancy_grid=new_grid,
                                                            logger=self.get_logger())
            # TODO: FIRST, just going to publish a fully occupied grid just to
            # get a feel for what's going on here. Also, if the pose issue is
            # giving me problems, may get rid of that lock, as I don't think
            # there are multiple threads running in this node's process yet.
            # data_list = []
            # column = [100]*(new_grid.info.width-3) + [50]*3
            # data_list += column*new_grid.info.height
            # new_grid.data = data_list
            # updated_grid = new_grid
            # TODO: Okay, can successfully visualize what's going on now, but
            # the costmap isn't center on the car. Granted, though, it doesn't
            # necessarily "have" to be. That's just how RVIZ is interpretting
            # it. What's important is how the points are projected and laid out
            # inside of it! I have a suspicion that this is being caused by
            # either its parent frame id being set incorrectly or its origin
            # being wrong?
            # Publish the updated grid.
            self.__laser_local_costmap_publisher.publish(updated_grid)
            self.get_logger().info(f"Published new local occupancy grid!")
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