#!/usr/bin/env python
import math
from enum import Enum

import actionlib
import rospy
from actionlib import SimpleActionClient, SimpleActionServer

# Msgs
from human_interactions.msg import (
    TurnToHumanAction,
    TurnToHumanFeedback,
    TurnToHumanResult,
)
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from twist_mux_msgs.msg import JoyPriorityAction, JoyPriorityGoal

# Local scripts
from human_interactions.utility.logger import ActionServerLogger, Action, LogLevel
from human_interactions.utility.tf_provider import TFProvider
from human_interactions.utility.head_controller import HeadController
from human_interactions.utility.utils import wait_until_server_ready

# Relevant robot links' tf frames.
class RobotLink(str, Enum):
    BASE = rospy.get_param("robot_base_tf")
    HEAD = rospy.get_param("robot_head_tf")
    TORSO = rospy.get_param("robot_torso_tf")


class TurnToHumanActionServer:
    """
    Server for the 'TurnToHuman' action.

    This class handles requests to orient the robot towards the human.

    If possible only the robot's head will move, pointing in the direction of
    the human. If the movement exceeds its range of motion, the robot's torso
    will be rotated first.

    Attributes:
        _initialized (bool): Flag to indicate if the server has been initialized.
        _logger (ActionServerLogger): An instance of the ActionServerLogger.
        _action_name (str): The name of the action server.
        _action_server (actionlib.SimpleActionServer): The action server instance.
        _current_odom (Odometry): The current odometry of the robot.
        _odom_subscriber (rospy.Subscriber): Subscriber for robot's Odometry messages.
        _joy_priority_subscriber (rospy.Subscriber): Subscriber for joy priority status.
        _tf_provider (TFProvider): An instance of the TFProvider for providing transforms.
        _move_base_client (actionlib.SimpleActionClient): The MoveBase action client.
        _head_controller (HeadController): An instance of the HeadController for controlling the robot's head movement.
        _human_pose_topic (str): The topic to use for getting the pose of human.
    """

    def __init__(self):
        """
        Initialize the TurnToHumanActionServer.
        """
        self._initialized = False

        # Create logger
        self._logger = ActionServerLogger(Action.TURN_TO_HUMAN)
        self._action_name = rospy.get_param("turn_to_human_action_name")
        self._logger.log("Initializing %s server." % self._action_name)

        # Init the action server
        self._action_server = SimpleActionServer(
            self._action_name,
            TurnToHumanAction,
            self.execute_callback,
            auto_start=False,
        )
        self._action_server.start()

        # Init complementary action clients
        self._joy_action_server = SimpleActionClient(
            rospy.get_param("joy_priority_action"), JoyPriorityAction
        )
        self._move_base_client = actionlib.SimpleActionClient(
            rospy.get_param("move_base_action_name"), MoveBaseAction
        )

        # Current state variables
        self._current_odom = Odometry()
        self._current_joy_priority = Bool()

        # Subscribers
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("odometry_topic"), Odometry, self.robot_odometry_callback
        )
        self._joy_priority_subscriber = rospy.Subscriber(
            rospy.get_param("joy_priority_topic"), Bool, self.joy_priority_callback
        )

        # Transform provider
        self._tf_provider = TFProvider()

        # Head controller
        self._head_controller = HeadController()

        # Ensure action servers ready
        use_joy_action = rospy.get_param("use_joy_action")
        self._logger.log("Use joy priority action: %s." % str(use_joy_action))
        if use_joy_action:
            wait_until_server_ready(
                self._joy_action_server,
                rospy.get_param("joy_priority_action"),
                self._logger,
            )

        wait_until_server_ready(
            self._move_base_client,
            rospy.get_param("move_base_action_name"),
            self._logger,
        )

        # Initalize the topic for human pose with default value.
        self._human_pose_topic = rospy.get_param("human_tf")

        self._initialized = True
        self._logger.log("%s server initialization complete." % self._action_name)

    def abort(self, msg):
        """
        Abort the action and log an error message.

        Args:
            msg (str): Reason for aborting.
        """
        self._action_server.set_aborted()
        self._logger.log(msg, LogLevel.ERROR)

    def robot_odometry_callback(self, message):
        """
        Callback for receiving robot's current odometry.

        Args:
            message (Odometry): Robot's odometry.
        """
        self._current_odom = message

    def joy_priority_callback(self, message):
        """
        Callback for receiving joy priority status.

        Args:
            message (Bool): The received joy priority status.
        """
        if rospy.get_param("use_joy_action"):
            self._current_joy_priority = message
            self._logger.log(
                "Current joy priority: %s." % str(self._current_joy_priority.data)
            )

    def get_angle_to_face_human(self):
        """
        Return the yaw (z axis rotation), which will make the robot's torso face
        the human when rotated.

        Returns:
            float: The required angle in radians.
        """
        human_position_in_robot_frame = self._tf_provider.get_tf(
            self._human_pose_topic, RobotLink.TORSO.value
        ).translation
        return math.atan2(
            human_position_in_robot_frame.y, human_position_in_robot_frame.x
        )

    def publish_feedback(self, link):
        """
        Publish feedback to the client.

        Args:
            link (str): Robot's link which is currently moved.
        """
        feedback = TurnToHumanFeedback()
        feedback.link = String(link)
        feedback.robot_pose = self._current_odom.pose.pose
        self._action_server.publish_feedback(feedback)

    def publish_result(self, status, used_link="none"):
        """
        Publish the end result of the action.

        Args:
            status (str): The status of action completion ('success', 'failure', etc).
            used_link (str, optional): The link used during the action ('none' by default).
        """
        result = TurnToHumanResult()
        result.status = String(status)
        result.robot_pose = self._current_odom.pose.pose
        result.link = String(used_link)
        self._action_server.set_succeeded(result)

    def call_joy_priority_action(self):
        """Call the JoyPriority action server."""
        self._joy_action_server.send_goal(JoyPriorityGoal())

    def rotate_torso(self):
        """
        Rotate the robot's torso to face the human.

        Returns:
            bool: Whether the action completed successfully.
        """
        initial_rotation_required = self.get_angle_to_face_human()
        self._logger.log(
            "Rotation required: %f radians --- %i degrees"
            % (initial_rotation_required, math.degrees(initial_rotation_required))
        )

        # Set a goal pose with the same position, but the orientation after
        # applying the necessary rotation.
        goal_pose = self._current_odom.pose.pose
        print("Default orientation:", goal_pose.orientation)
        goal_pose.orientation = TFProvider.get_orientation_after_yaw_rotation(goal_pose.orientation, initial_rotation_required)
        print("After rotation:", goal_pose.orientation)

        navigation_goal = MoveBaseGoal()
        navigation_goal.target_pose = PoseStamped()
        navigation_goal.target_pose.header.frame_id = "map"
        navigation_goal.target_pose.header.stamp = rospy.Time.now()
        navigation_goal.target_pose.pose = goal_pose

        # Send a goal to the move base server and wait for completion.
        self._move_base_client.send_goal(navigation_goal)
        while not self._move_base_client.wait_for_result(rospy.Duration(0.5)):
            self.publish_feedback("torso")

        return True

    def get_relative_human_pose(self):
        """
        Get the relative human pose in the robot's frame.

        Returns:
            Pose: Relative human pose.
        """
        transform = self._tf_provider.get_tf(
            self._human_pose_topic, RobotLink.BASE.value
        )
        if not transform:
            self.abort("Transform unavailable, aborting.")
            return

        return TFProvider.get_transform_as_pose(transform)

    def point_head_at_human(self):
        """Point the robot's head towards the human."""
        self._head_controller.point_at(self.get_relative_human_pose().position)

    def execute_callback(self, goal):
        """
        Callback for handling action requests.

        If necessary rotates the robot's torso, but if possible only moves its head to look towards the human.

        Args:
            goal (TurnToHumanActionGoal): The goal message.
        """
        self._logger.log("New request received.")
        success = True

        if not self._initialized:
            self.abort("Server not ready, ignoring request.")
            self.publish_result("failure")
            return

        # Override default value with the goal if provided.
        goal_topic = goal.human_pose_topic.data
        if len(goal_topic) > 0:
            self._human_pose_topic = goal_topic

        required_angle = self.get_angle_to_face_human()
        self._logger.log(
            "Change in yaw: %f degrees." % math.degrees(required_angle),
        )

        # If possible, move just the head to face the speaker. If the angle is
        # out of its range of motion start by moving the the torso.
        max_head_rotation = rospy.get_param("max_head_rotation")
        used_link = None
        if abs(required_angle) > max_head_rotation:
            self._logger.log("Moving torso.")
            used_link = "torso"
            success = self.rotate_torso()
        else:
            used_link = "head"

        self._logger.log("Moving head.")
        self.point_head_at_human()

        if success:
            self.publish_result("success", used_link)
            self._logger.log("%s: succeeded." % self._action_name)
        else:
            self.publish_result("failure", used_link)
            self.abort("%s: aborted." % self._action_name)
            return

        self._logger.log("Finished execution.")


def run_server():
    # Initialize the node and create the server instance.
    rospy.init_node("turn_to_human_action_server")
    TurnToHumanActionServer()
    rospy.spin()


if __name__ == "__main__":
    run_server()
