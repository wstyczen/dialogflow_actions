#!/usr/bin/env python
import rospy
import math
from enum import Enum

from actionlib import SimpleActionClient, SimpleActionServer

# Msgs
from dialogflow_emergency_action_servers.msg import (
    TurnToHumanAction,
    TurnToHumanFeedback,
    TurnToHumanResult,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from twist_mux_msgs.msg import JoyPriorityAction, JoyPriorityGoal

# Local scripts
from logger import ActionServerLogger, Action, LogLevel
from tf_provider import TFProvider
from head_controller import HeadController
from utils import wait_until_server_ready

# Relevant robot links' tf frames.
class RobotLink(str, Enum):
    BASE = rospy.get_param("robot_base_tf")
    HEAD = rospy.get_param("robot_head_tf")
    TORSO = rospy.get_param("robot_torso_tf")


class TurnToHumanActionServer:
    def __init__(self):
        self._initialized = False

        # Create logger
        self._logger = ActionServerLogger(Action.TURN_TO_HUMAN)
        self._action_name = rospy.get_param("action_name")
        self._logger.log("Initializing %s server." % self._action_name)

        # Init the action server
        self._action_server = SimpleActionServer(
            self._action_name,
            TurnToHumanAction,
            self.execute_callback,
            auto_start=False,
        )
        self._action_server.start()

        # Init complementary action servers
        self._joy_action_server = SimpleActionClient(
            rospy.get_param("joy_priority_action"), JoyPriorityAction
        )

        # Subscribers
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("odometry_topic"), Odometry, self.robot_odometry_callback
        )
        self._joy_priority_subscriber = rospy.Subscriber(
            rospy.get_param("joy_priority_topic"), Bool, self.joy_priority_callback
        )

        # Publishers
        self._velocity_publisher = rospy.Publisher(
            rospy.get_param("command_velocity_topic"), Twist, queue_size=1000
        )

        # Transform provider
        self._tf_provider = TFProvider()

        # Head controller
        self._head_controller = HeadController()

        # Current state variables
        self._current_odom = Odometry()
        self._current_joy_priority = Bool()

        # Ensure action servers ready
        use_joy_action = rospy.get_param("use_joy_action")
        self._logger.log("Use joy priority action: %s." % str(use_joy_action))
        if use_joy_action:
            wait_until_server_ready(
                self._joy_action_server,
                rospy.get_param("joy_priority_action"),
                self._logger,
            )

        self._initialized = True
        self._logger.log("%s server initialization complete." % self._action_name)

    def abort(self, msg):
        self._action_server.set_aborted()
        self._logger.log(msg, LogLevel.ERROR)

    def robot_odometry_callback(self, message):
        self._current_odom = message

    def joy_priority_callback(self, message):
        if rospy.get_param("use_joy_action"):
            self._current_joy_priority = message
            self._logger.log(
                "Current joy priority: %s." % str(self._current_joy_priority.data)
            )

    def get_angle_to_face_human(self):
        # Return the yaw (z axis rotation), which will make the robot torso face
        # the human.
        human_position_in_robot_frame = self._tf_provider.get_tf(
            rospy.get_param("human_tf"), RobotLink.TORSO.value
        ).translation
        return math.atan2(
            human_position_in_robot_frame.y, human_position_in_robot_frame.x
        )

    def publish_feedback(self, link):
        feedback = TurnToHumanFeedback()
        feedback.link = String(link)
        feedback.robot_pose = self._current_odom.pose.pose
        self._action_server.publish_feedback(feedback)

    def publish_result(self, status, used_link="none"):
        result = TurnToHumanResult()
        result.status = String(status)
        result.robot_pose = self._current_odom.pose.pose
        result.link = String(used_link)
        self._action_server.set_succeeded(result)

    def publish_torso_velocity_command(self, angular_velocity):
        velocity = Twist()
        velocity.angular.z = angular_velocity
        self._velocity_publisher.publish(velocity)

    def call_joy_priority_action(self):
        goal = JoyPriorityGoal()
        self._joy_action_server.send_goal(goal)

    def rotate_torso(self):
        EPS = 0.05  # Acceptable orientation error
        r = rospy.Rate(30)  # Loop sleep rate
        success = True

        default_torso_velocity = rospy.get_param("torso_rotation_velocity")
        rotate_right = lambda: self.publish_torso_velocity_command(
            default_torso_velocity
        )
        rotate_left = lambda: self.publish_torso_velocity_command(
            -default_torso_velocity
        )
        stop = lambda: self.publish_torso_velocity_command(0.0)

        initial_rotation_required = self.get_angle_to_face_human()
        self._logger.log(
            "Rotation required: %f radians --- %i degrees"
            % (initial_rotation_required, math.degrees(initial_rotation_required))
        )

        locked_state = self._current_joy_priority.data
        if not locked_state:
            self.call_joy_priority_action()

        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested() or rospy.is_shutdown():
                self._logger.log("%s: preempted." % self._action_name)
                self._action_server.set_preempted()
                stop()
                success = False
                break

            angle_err = self.get_angle_to_face_human()
            self._logger.log(
                "Angle error: %f radians --- %i degrees"
                % (angle_err, math.degrees(angle_err))
            )
            # Rotate until within acceptable error or exceeded.
            if abs(angle_err) < EPS or angle_err * initial_rotation_required < 0:
                stop()
                break

            if angle_err > 0:
                rotate_right()
            else:
                rotate_left()
            self.publish_feedback("torso")

            r.sleep()

        self._head_controller.reset()

        if not locked_state:
            self.call_joy_priority_action()

        return success

    def get_relative_human_pose(self):
        # Return the relative human pose in robot's frame
        transform = self._tf_provider.get_tf(
            rospy.get_param("human_tf"), RobotLink.BASE.value
        )
        if not transform:
            self.abort("Transform unavailable, aborting.")
            return

        return TFProvider.get_as_pose(transform)

    def point_head_at_human(self):
        self._head_controller.point_at(self.get_relative_human_pose().position)

    def execute_callback(self, _):
        self._logger.log("New request received.")
        success = True

        if not self._initialized:
            self.abort("Server not ready, ignoring request.")
            self.publish_result("failure")
            return

        required_angle = self.get_angle_to_face_human()
        self._logger.log(
            "Change in yaw: %f degrees." % math.degrees(required_angle),
        )

        # If possible, move just the head to face the speaker. If the angle is
        # out of its range of motion move the torso.
        max_head_rotation = rospy.get_param("max_head_rotation")
        used_link = "torso"
        # if abs(required_angle) > max_head_rotation:
        self._logger.log("Moving torso.")
        success = self.rotate_torso()
        # else:
        # used_link = "head"
        # self._logger.log("Moving head.")
        # self.point_head_at_human()

        if success:
            self.publish_result("success", used_link)
            self._logger.log("%s: succeeded." % self._action_name)
        else:
            self.publish_result("failure", used_link)
            self.abort("%s: aborted." % self._action_name)
            return

        self._logger.log("Finished execution.")


def run_server():
    # Initialize the node and create the server instance
    rospy.init_node("turn_to_human_action_server")
    TurnToHumanActionServer()
    rospy.spin()


if __name__ == "__main__":
    run_server()
