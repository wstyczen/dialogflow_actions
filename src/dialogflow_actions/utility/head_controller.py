#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, GoalStatus

# Msgs
from geometry_msgs.msg import Point, Vector3
from control_msgs.msg import PointHeadAction, PointHeadGoal

# Local scripts
from logger import Logger
from utils import wait_until_server_ready


class HeadController:
    """
    A wrapper class to help with pointing the robot's head at specified points.

    Attributes:
        _logger (Logger): An instance of the Logger class.
        _point_head_action_server (SimpleActionClient): An action client for the PointHeadAction action.
    """

    def __init__(self):
        """
        Initialize the HeadController.
        """
        self._logger = Logger("head controller")

        point_head_action_name = rospy.get_param("point_head_action")
        self._point_head_action_server = SimpleActionClient(
            point_head_action_name, PointHeadAction
        )
        wait_until_server_ready(
            self._point_head_action_server, point_head_action_name, self._logger
        )
        self._logger.log("Head controller ready.")

    def point_at(self, point):
        """
        Point the robot's head at a specified point.

        Args:
            point (Point): The point which the head should look at.

        Returns:
            bool: Whether pointing action was successful.
        """
        self._logger.log(
            "Pointing head at {x: %f, y: %f, z: %f}" % (point.x, point.y, point.z)
        )
        goal = PointHeadGoal()

        goal.target.header.frame_id = rospy.get_param("robot_base_tf")
        goal.target.point = point
        goal.pointing_axis = Vector3(1.0, 0.0, 0.0)
        goal.pointing_frame = rospy.get_param(rospy.get_param("point_head_tf"))
        goal.max_velocity = rospy.get_param("head_rotation_velocity")

        self._point_head_action_server.send_goal(goal)
        self._point_head_action_server.wait_for_result(rospy.Duration(1.0))

        return True

    def reset(self):
        """
        Resets the head to its default orientation.
        """
        self._logger.log("Resetting head.")
        self.point_at(Point(1.0, 0.0, 1.0))

    def wait_till_idle(self):
        """
        Waits until the current movement is completed.
        """
        while (
            not rospy.is_shutdown()
            and self._point_head_action_server.get_state()
            in [
                GoalStatus.PENDING,
                GoalStatus.ACTIVE,
            ]
        ):
            continue
