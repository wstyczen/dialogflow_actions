#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, GoalStatus

# Msgs
from geometry_msgs.msg import Point, Vector3
from control_msgs.msg import PointHeadAction, PointHeadGoal


class HeadController:
    def __init__(self):
        self._head_action_server = SimpleActionClient(
            rospy.get_param("point_head_action"), PointHeadAction
        )
        self._head_action_server.wait_for_server()

    def point_at(self, point):
        goal = PointHeadGoal()

        goal.target.header.frame_id = rospy.get_param("robot_base_tf")
        goal.target.point = point
        goal.pointing_axis = Vector3(1.0, 0.0, 0.0)
        goal.pointing_frame = rospy.get_param(rospy.get_param("point_head_tf"))
        goal.max_velocity = rospy.get_param("head_rotation_velocity")

        self._head_action_server.send_goal(goal)
        self._head_action_server.wait_for_result(rospy.Duration(1.0))

        return True

    def reset(self):
        self.point_at(Point(1.0, 0.0, 1.0))

    def wait_till_idle(self):
        while not rospy.is_shutdown() and self._head_action_server.get_state() in [
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
        ]:
            continue
