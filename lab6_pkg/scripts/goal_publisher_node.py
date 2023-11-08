#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from lab6_pkg.goal_publisher_utils import get_next_target_point

class GoalPublisher(Node):
    """Node class dedicated to examining a "global," course path and deciding
    which node from that course path should be planned to by a local planner.
    """
    def __init__(self):
        super().__init__("goal_publisher")

        self.declare_parameters(namespace="",
                                parameters=[
                                    ("lookahead_distance_m", 2.0)
                                ])
        # NOTE: MAY NEED TO MAKE THE LOOKAHEAD DISTANCE DYNAMIC WITH THE CURRENT
        # LONGITUDINAL VELOCITY.
        self.__lookahead_distance_m = self.get_parameter("lookahead_distance_m").value

        # Create subscriber for global path.
        self.__path_subscriber = self.create_subscription(msg_type=Path,
                                                          topic="path",
                                                          callback=self.__path_callback,
                                                          qos_profile=10)
        self.__path = None

        # Create subscriber for vehicle pose.
        self.__vehicle_pose_subscriber = self.create_subscription(msg_type=PoseWithCovarianceStamped,
                                                                  topic="pose",
                                                                  callback=self.__pose_callback,
                                                                  qos_profile=10)
        
        # Create publisher for goal point.
        self.__goal_publisher = self.create_publisher(msg_type=PoseStamped,
                                                      topic="goal_pose",
                                                      qos_profile=10)
        
    def __path_callback(self, path_msg: Path) -> None:
        """Simple path callback to store most recently received Path.

        Args:
            path_msg (Path): Received path message.
        """
        self.__path = path_msg
    
    def __pose_callback(self, pose_msg: PoseWithCovarianceStamped) -> None:
        """Callback where, upon receiving the most recent vehicle pose, the next
        point in the most recently received global path is picked as the next
        goal point.

        Args:
            pose_msg (PoseWithCovarianceStamped): Most recent vehicle pose.
        """
        # 1. Optionally compute the current lookahead distance between some min
        #    and max using some gain specified as a parameter given the current
        #    longitudinal velocity of the vehicle (get this from the pose's
        #    twist component).
        lookahead_distance_m = self.__lookahead_distance_m

        # 2. The vehicle pose is the starting point. Just like pure pursuit,
        #    first find the closest point. Then from there, scan all the rest of
        #    the points from that one that are at least one lookahead distance
        #    away. Don't remember how I solved the issue of being at the end of
        #    the path, maybe it was fine? 

        # This functionality is implemented in the get_next_target_point
        # function.
        try:
            target_pose = get_next_target_point(current_pose=pose_msg, 
                                                path=self.__path,
                                                lookahead_distance_m=lookahead_distance_m)
        except Exception as exc:
            self.get_logger().error(f"Failed to get next target pose.\nException: {str(exc)}")
            return
        
        # 3. Publish the selected target pose as the next goal pose.
        self.__goal_publisher.publish(target_pose)

        # REMEMBER: This node's lookahead distance PROBABLY needs to be 2x as
        # long as pure-pursuits, as pure pursuit is going to be following RRT's
        # path. Therefore, RRT should be planning ahead of what the path tracker
        # is following, right?

        # Also note: This node publishes goal points in the same frame as the
        # path received. I.e., this exclusively operates in the frame that the
        # subscribed path is in.

def main(args=None):
    rclpy.init()
    goal_publisher_node = GoalPublisher()
    rclpy.spin(goal_publisher_node)
    goal_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()