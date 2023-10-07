#!/usr/bin/env python

import rospy
import actionlib
import tf
import math

# Msgs
from dialogflow_emergency_action_servers.msg import (
    MoveToHumanAction,
    MoveToHumanActionResult,
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.srv import GetPlan
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from logger import ActionServerLogger, Action, LogLevel


class MoveToHumanActionServer:
    def __init__(self):
        self._initialized = False

        self._logger = ActionServerLogger(Action.MOVE_TO_HUMAN)
        self._action_name = rospy.get_param("action_name")

        # Init the action server
        self._logger.log("Initializing %s server." % self._action_name)
        self._action_server = actionlib.SimpleActionServer(
            self._action_name,
            MoveToHumanAction,
            self.execute_callback,
            auto_start=False,
        )
        self._action_server.start()

        # Subscribers
        self._odom_subscriber = rospy.Subscriber(
            "odom", Odometry, self.robot_odometry_callback
        )
        self._tf_listener = tf.TransformListener()

        # Current state variables
        self._current_odom = Odometry()

        # Action servers
        self._move_base_client = actionlib.SimpleActionClient(
            rospy.get_param("move_base_action_name"), MoveBaseAction
        )
        self._head_client = actionlib.SimpleActionClient(
            rospy.get_param("move_head_action_name"), PointHeadAction
        )

        # Init odom plan service
        self._plan_service = rospy.ServiceProxy(
            rospy.get_param("move_base_plan_service_name"), GetPlan
        )

        # Ensure action servers ready
        def wait_until_server_ready(server, server_name):
            self._logger.log("Waiting for %s server..." % server_name)
            server.wait_for_server()
            self._logger.log("%s server ready." % server_name)

        wait_until_server_ready(
            self._move_base_client, rospy.get_param("move_base_action_name")
        )
        wait_until_server_ready(
            self._head_client, rospy.get_param("move_head_action_name")
        )

        self._logger.log("%s server initialization complete." % self._action_name)
        self._initialized = True

    def reset_head(self):
        self._logger.log("Resetting head orientation.")
        goal = PointHeadGoal()
        goal.target.header.frame_id = rospy.get_param("base_link")
        goal.target.point.x = 1.0
        goal.target.point.y = 0.0
        goal.target.point.z = 1.0
        goal.pointing_axis.x = 1.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 0.0
        goal.pointing_frame = rospy.get_param(
            "/head_controller/point_head_action/tilt_link"
        )
        goal.max_velocity = rospy.get_param("head_turning_velocity")
        self._head_client.send_goal(goal)

    def robot_odometry_callback(self, message):
        self._current_odom = message

    def get_pose(self, point, origin):
        try:
            self._tf_listener.waitForTransform(
                point, origin, rospy.Time(0), rospy.Duration(5.0)
            )
            (origin, rotation) = self._tf_listener.lookupTransform(
                point, origin, rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            self._logger.log("Failed to lookup transform.", LogLevel.ERROR)
            return None

        pose = Pose()
        pose.position.x = origin[0]
        pose.position.y = origin[1]
        pose.position.z = origin[2]
        pose.orientation.x = rotation[0]
        pose.orientation.y = rotation[1]
        pose.orientation.z = rotation[2]
        pose.orientation.w = rotation[3]

        return pose

    def get_plan(self):
        self._logger.log("Requesting plan.")
        start_pose = self.get_pose("base_link", "map")
        if not start_pose:
            self._logger.log("Can't determine start pose, aborting.", LogLevel.ERROR)
            return
        goal_pose = self.get_pose(rospy.get_param("human_tf"), "map")
        if not start_pose:
            self._logger.log("Can't determine goal pose, aborting.", LogLevel.ERROR)
            return

        request = GetPlanRequest()
        request.start.header.frame_id = "map"
        request.start.pose = start_pose
        request.goal.header.frame_id = "map"
        request.goal.pose = goal_pose
        request.tolerance = 0.5

        try:
            response = self._plan_service.call(request)

            plan = response.plan
            plan.header.frame_id = "map"

            human_pose = goal_pose
            final_poses_list = []

            # Filter points in trajectory that are to close to human
            min_distance = rospy.get_param("min_distance_to_human")
            for pose in plan.poses:
                distance = math.sqrt(
                    (pose.pose.position.x - human_pose.position.x) ** 2
                    + (pose.pose.position.y - human_pose.position.y) ** 2
                )
                if distance >= min_distance:
                    final_poses_list.append(pose)

            plan.poses = final_poses_list
            self._logger.log("Returning plan.")
            return plan
        except rospy.ServiceException as e:
            self._logger.log(
                "Exception when calling service %s."
                % rospy.get_param("move_base_plan_service_name")
            )
            raise Exception(e)

    def move_head(self):
        self._logger.log("Moving head.")
        goal = PointHeadGoal()
        pose = self.get_pose(rospy.get_param("human_tf"), rospy.get_param("base_link"))
        if not pose:
            self._logger.log("Can't determine pose, aborting.", LogLevel.ERROR)
            return

        goal.target.header.frame_id = rospy.get_param("base_link")
        goal.target.point.x = pose.position.x
        goal.target.point.y = pose.position.y
        goal.target.point.z = pose.position.z
        goal.pointing_axis.x = 1.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 0.0
        goal.pointing_frame = rospy.get_param(
            "/head_controller/point_head_action/tilt_link"
        )
        goal.max_velocity = rospy.get_param("head_turning_velocity")
        self._head_client.send_goal(goal)

        while not rospy.is_shutdown():
            state = self._head_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                return

    def execute_callback(self, goal):
        self._logger.log("New goal requested.")

        if not self._initialized:
            self._logger.log("Server not ready, ignoring request.", LogLevel.WARNING)
            return

        self.reset_head()
        plan = self.get_plan()

        if len(plan.poses) > 0:
            goal_pose = plan.poses[-1]
            navigation_goal = MoveBaseGoal()
            navigation_goal.target_pose = goal_pose
            self._move_base_client.send_goal(navigation_goal)
            self._move_base_client.wait_for_result()
            self.move_head()
            self._logger.log("Moving robot...")
        else:
            self._logger.log("Received empty movement plan.")

        result = MoveToHumanActionResult()
        result.status = String()
        result.status.data = "finished"
        self._action_server.set_succeeded(result)

        self._logger.log("%s action completed." % self._action_name)


def run_server():
    rospy.init_node("move_to_human_action_server")
    MoveToHumanActionServer()
    rospy.spin()


if __name__ == "__main__":
    run_server()
