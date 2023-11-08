#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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
        self.__vehicle_pose_subscriber = self.create_subscription(msg_type=PoseStamped,
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
    
    def __pose_callback(self, pose_msg: PoseStamped) -> None:
        """Callback where, upon receiving the most recent vehicle pose, the next
        point in the most recently received global path is picked as the next
        goal point.

        Args:
            pose_msg (PoseStamped): _description_
        """
        pass
    
