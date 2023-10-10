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
from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from control_msgs.msg import PointHeadAction, PointHeadGoal

# from pal_common_msgs.msg import DisableAction, DisableGoal
from twist_mux_msgs.msg import JoyPriorityAction, JoyPriorityGoal

# Local scripts
from logger import ActionServerLogger, Action, LogLevel
from tf_provider import TFProvider

# Relevant robot links' tf frames.
class RobotLink(str, Enum):
    BASE = rospy.get_param("base_link_tf")
    HEAD = rospy.get_param("robot_head_tf")
    TORSO = rospy.get_param("robot_torso_tf")


# Head movement options.
class HeadMovement(Enum):
    RESET = 0
    FACE_HUMAN = 1


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
        self._head_action_server = SimpleActionClient(
            rospy.get_param("point_head_action"), PointHeadAction
        )
        # self._disable_action_server = SimpleActionClient(
        #     rospy.get_param("disable_autohead_action"), DisableAction
        # )

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

        # Current state variables
        self._current_odom = Odometry()
        self._current_joy_priority = Bool()

        # Ensure action servers ready
        def wait_until_server_ready(server, server_name):
            self._logger.log("Waiting for %s server..." % server_name)
            server.wait_for_server()
            self._logger.log("%s server ready." % server_name)

        self._logger.log("%s server ready." % self._action_name)
        wait_until_server_ready(
            self._head_action_server, rospy.get_param("point_head_action")
        )
        # wait_until_server_ready(
        #             self._disable_action_server, rospy.get_param("disable_autohead_action")
        # )

        use_joy_action = rospy.get_param("use_joy_action")
        self._logger.log("Use joy priority action: %s." % str(use_joy_action))
        if use_joy_action:
            wait_until_server_ready(
                self._joy_action_server, rospy.get_param("joy_priority_action")
            )

        self._initialized = True
        self._logger.log("%s server initialization complete." % self._action_name)

    def abort(self, msg):
        self._logger.log(msg, LogLevel.ERROR)
        self._action_server.set_aborted(
            result=TurnToHumanResult(status=String("aborted"))
        )

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

    def publish_feedback(self, state):
        feedback_ = TurnToHumanFeedback()
        feedback_.status = String(state)
        feedback_.orientation = self._current_odom.pose.pose.orientation
        self._action_server.publish_feedback(feedback_)

    def publish_status(self, state):
        result = TurnToHumanResult()
        result.status = String(state)
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
                self._head_action_server.stop_tracking_goal()
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
            self.publish_feedback("rotating_torso")

            r.sleep()

        self.point_head(HeadMovement.RESET)

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

        return TFProvider.get_as_pose(transform)

    def point_head(self, movement):
        goal = PointHeadGoal()
        goal.target.header.frame_id = RobotLink.BASE
        if movement == HeadMovement.RESET:
            goal.target = Point(1.0, 0.0, 1.0)
        elif movement == HeadMovement.FACE_HUMAN:
            human_pose = self.get_relative_human_pose()
            goal.target.point = human_pose.position
        else:
            self.abort("Invalid head movement requested.")

        goal.pointing_axis = Vector3(1.0, 0.0, 0.0)
        goal.pointing_frame = rospy.get_param(rospy.get_param("point_head_tf"))
        goal.max_velocity = rospy.get_param("head_rotation_velocity")
        self._head_action_server.send_goal(goal)
        self._head_action_server.wait_for_result(rospy.Duration(1.0))

        return True

    def execute_callback(self, goal):
        self._logger.log("New goal requested.")

        if not self._initialized:
            self.abort("Server not ready, ignoring request.")

        success = True
        # goal = DisableGoal()
        # self._disable_action_server.send_goal(goal)

        required_angle = self.get_angle_to_face_human()
        self._logger.log(
            "Change in yaw: %f degrees." % math.degrees(required_angle),
        )

        # If possible, move just the head to face the speaker. If the angle is
        # out of its range of motion move the torso.
        max_head_rotation = rospy.get_param("max_head_rotation")
        if abs(required_angle) > max_head_rotation:
            self._logger.log("Moving torso.")
            success = self.rotate_torso()
        else:
            self._logger.log("Moving head.")
            success = self.point_head(HeadMovement.FACE_HUMAN)

        if success:
            self.publish_status("finished")
            self._logger.log("%s: succeeded." % self._action_name)
        else:
            self.abort("%s: aborted." % self._action_name)

        self._logger.log("Finished execution.")


def run_server():
    # Initialize the node and create the server instance
    rospy.init_node("turn_to_human_action_server")
    TurnToHumanActionServer()
    rospy.spin()


if __name__ == "__main__":
    run_server()
