#!/usr/bin/env python

import rospy
import actionlib
import math

# Msgs
from dialogflow_actions.msg import (
    MoveToHumanAction,
    MoveToHumanFeedback,
    MoveToHumanResult,
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.srv import GetPlan
from std_msgs.msg import String

# Local scripts
from logger import ActionServerLogger, Action, LogLevel
from tf_provider import TFProvider
from head_controller import HeadController
from utils import wait_until_server_ready


class MoveToHumanActionServer:
    """
    Server for the 'MoveToHuman' action.

    This class handles requests to move the robot to stand a safe distance away
    from the person's position and face them.

    Attributes:
        _initialized (bool): Flag to indicate if the server has been initialized.
        _logger (ActionServerLogger): An instance of the ActionServerLogger.
        _action_name (str): The name of the action server.
        _action_server (actionlib.SimpleActionServer): The action server instance.
        _current_odom (Odometry): The current odometry of the robot.
        _odom_subscriber (rospy.Subscriber): Subscriber for robot's Odometry messages.
        _tf_provider (TFProvider): An instance of the TFProvider for providing transforms.
        _head_controller (HeadController): An instance of the HeadController for controlling the robot's head movement.
        _move_base_client (actionlib.SimpleActionClient): The MoveBase action client.
        _plan_service (rospy.ServiceProxy): The service for getting a navigation plan.
    """

    def __init__(self):
        """
        Initialize the MoveToHumanActionServer.
        """
        self._initialized = False

        self._logger = ActionServerLogger(Action.MOVE_TO_HUMAN)
        self._action_name = rospy.get_param("move_to_human_action_name")

        # Init the action server
        self._logger.log("Initializing %s server." % self._action_name)
        self._action_server = actionlib.SimpleActionServer(
            self._action_name,
            MoveToHumanAction,
            self.execute_callback,
            auto_start=False,
        )
        self._action_server.start()

        # Current state variables
        self._current_odom = Odometry()

        # Subscribers
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("odometry_topic"), Odometry, self.robot_odometry_callback
        )

        # Transform provider
        self._tf_provider = TFProvider()

        # Head controller
        self._head_controller = HeadController()

        # Action servers
        self._move_base_client = actionlib.SimpleActionClient(
            rospy.get_param("move_base_action_name"), MoveBaseAction
        )

        # Init odom plan service
        self._plan_service = rospy.ServiceProxy(
            rospy.get_param("move_base_plan_service_name"), GetPlan
        )

        wait_until_server_ready(
            self._move_base_client,
            rospy.get_param("move_base_action_name"),
            self._logger,
        )

        self._logger.log("%s server initialization complete." % self._action_name)
        self._initialized = True

    def robot_odometry_callback(self, message):
        """
        Callback for receiving robot's current odometry.

        Args:
            message (Odometry): odometry message.
        """
        self._current_odom = message

    def publish_feedback(self):
        """
        Publish feedback for the client.
        """
        feedback = MoveToHumanFeedback()
        feedback.robot_pose = self._current_odom.pose.pose
        self._action_server.publish_feedback(feedback)

    def publish_result(self, status):
        """
        Publish the result of the action for the client.

        Args:
            status (str): The finished action's status.
        """
        result = MoveToHumanResult()
        result.status = String(status)
        result.robot_pose = self._current_odom.pose.pose
        self._action_server.set_succeeded(result)
        self._logger.log("Action ended with status: '%s'" % status)

    def get_pose(self, point, origin):
        """
        Get a pose from a point relative to the requested origin.

        Args:
            point: The point to transform into a pose.
            origin: The origin frame for the transformation.

        Returns:
            Pose: The resulting pose.
        """
        transform = self._tf_provider.get_tf(point, origin)
        return TFProvider.get_as_pose(transform)

    def get_plan(self):
        """
        Request a navigation plan from the MoveBase plan service and make sure
        the safe distance from human is maintained.

        Returns:
            list[Pose]: The navigation plan.
        """
        self._logger.log("Requesting plan.")

        start_pose = self.get_pose(rospy.get_param("robot_base_tf"), "map")
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

            # Filter points in trajectory that are to close to the human.
            get_distance_from_human = lambda robot_pose: math.sqrt(
                (robot_pose.pose.position.x - human_pose.position.x) ** 2
                + (robot_pose.pose.position.y - human_pose.position.y) ** 2
            )
            plan.poses = filter(
                lambda pose: get_distance_from_human(pose)
                >= rospy.get_param("min_distance_to_human"),
                plan.poses,
            )

            self._logger.log("Returning plan.")
            return plan
        except rospy.ServiceException as e:
            self._logger.log(
                "Exception when calling service %s."
                % rospy.get_param("move_base_plan_service_name")
            )
            raise Exception(e)

    def move_head(self):
        """
        Move the robot's head to face the human.
        """
        self._logger.log("Moving head.")
        pose = self.get_pose(
            rospy.get_param("human_tf"), rospy.get_param("robot_base_tf")
        )
        if not pose:
            self._logger.log("Can't determine pose, aborting.", LogLevel.ERROR)
            return

        self._head_controller.point_at(pose.position)
        self._head_controller.wait_till_idle()

    def execute_callback(self, _):
        """
        Callback for handling action requests.

        Moves the robot to the vicinity of the human and orients it towards them.

        Args:
            _: Goal(unused).
        """
        self._logger.log("New request received.")

        if not self._initialized:
            self._logger.log("Server not ready, ignoring request.", LogLevel.WARNING)
            self.publish_result("failure")
            return

        self._head_controller.reset()

        plan = self.get_plan()
        if plan and len(plan.poses) > 0:
            self._logger.log("Moving robot...")
            goal_pose = plan.poses[-1]
            navigation_goal = MoveBaseGoal()
            navigation_goal.target_pose = goal_pose
            self._move_base_client.send_goal(navigation_goal)
            while not self._move_base_client.wait_for_result(rospy.Duration(0.5)):
                self.publish_feedback()
            self.move_head()
            self.publish_result("success")
        else:
            self._logger.log("Failed to plan movement.")
            self.publish_result("failure")

        self._logger.log("%s execution finished." % self._action_name)


def run_server():
    rospy.init_node("move_to_human_action_server")
    MoveToHumanActionServer()
    rospy.spin()


if __name__ == "__main__":
    run_server()
