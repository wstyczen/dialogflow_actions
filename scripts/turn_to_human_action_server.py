#!/usr/bin/env python
import rospy
import math

from actionlib import SimpleActionClient, SimpleActionServer
from tf import TransformListener

# Msgs
from dialogflow_emergency_action_servers.msg import (
    TurnToHumanAction,
    TurnToHumanFeedback,
    TurnToHumanResult,
)
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.msg import PointHeadAction, PointHeadGoal
from pal_common_msgs.msg import DisableAction, DisableGoal
from twist_mux_msgs.msg import JoyPriorityAction, JoyPriorityGoal

from logger import ActionServerLogger, Action, LogLevel

PI_DEGREES = 180
PI_VALUE = 3.1415


class TurnToHumanActionServer:
    def __init__(self):
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

        # Init complementary action servers
        self._joy_action_server = SimpleActionClient(
            rospy.get_param("joy_priority_action"), JoyPriorityAction
        )
        self._head_action_server = SimpleActionClient(
            rospy.get_param("point_head_action"), PointHeadAction
        )
        self._disable_action_server = SimpleActionClient(
            rospy.get_param("disable_autohead_action"), DisableAction
        )

        # Subscribers
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("odometry_topic"), Odometry, self.robot_odometry_callback
        )
        self._joint_state_subscriber = rospy.Subscriber(
            rospy.get_param("joint_state_topic"),
            JointState,
            self.robot_joint_state_callback,
        )
        self._joy_priority_subscriber = rospy.Subscriber(
            rospy.get_param("joy_priority_topic"), Bool, self.joy_priority_callback
        )
        self._tf_listener = TransformListener()

        # Publishers
        self._velocity_publisher = rospy.Publisher(
            rospy.get_param("command_velocity_topic"), Twist, queue_size=1000
        )

        # Current state variables
        self._current_odom = Odometry()
        self._current_joint_state = JointState()
        self._current_joy_priority = Bool()

        # Start action servers
        self._action_server.start()
        self._logger.log("%s server ready." % self._action_name)

        self._logger.log(
            "Waiting for %s client..." % rospy.get_param("point_head_action")
        )
        self._head_action_server.wait_for_server()
        self._logger.log("%s client ready." % rospy.get_param("point_head_action"))

        self._logger.log(
            "Waiting for %s client..." % rospy.get_param("disable_autohead_action")
        )
        self._disable_action_server.wait_for_server()
        self._logger.log(
            "%s client ready." % rospy.get_param("disable_autohead_action")
        )

        use_joy_action = rospy.get_param("use_joy_action")
        self._logger.log("Use joy priority action: %s." % str(use_joy_action))
        if use_joy_action:
            self._logger.log(
                "Waiting for %s server..." % rospy.get_param("joy_priority_action")
            )
            self._joy_action_server.wait_for_server()
            self._logger.log(
                "%s client ready." % rospy.get_param("joy_priority_action")
            )

        self._initialized = True
        self._logger.log("%s server initialization complete." % self._action_name)

    def execute_callback(self, goal):
        self._logger.log("New goal requested.")

        if not self._initialized:
            self._logger.log("Server not ready, ignoring request.", LogLevel.WARNING)
            return

        success = True
        goal = DisableGoal()
        self._disable_action_server.send_goal(goal)

        required_angle = self.find_required_angle()
        self._logger.log(
            "Desired change in yaw: %f degrees."
            % ((PI_DEGREES * required_angle) / PI_VALUE),
        )

        # If possible, move just the head to face the speaker. If the angle is
        # out of its range of motion move the torso.
        max_head_rotation_radian = rospy.get_param("max_head_rotation")
        if abs(required_angle) > max_head_rotation_radian:
            self._logger.log("Moving torso.")
            success = self.move_torso()
        else:
            self._logger.log("Moving head.")
            success = self.move_head()

        self._logger.log("Finished execution.")

        if success:
            self.publish_status("finished")
            rospy.loginfo("%s: succeeded." % self._action_name)
        else:
            self._action_server.set_aborted()
            rospy.loginfo("%s: aborted." % self._action_name)

    def robot_odometry_callback(self, message):
        self._current_odom = message

    def robot_joint_state_callback(self, message):
        self._current_joint_state = message

    def joy_priority_callback(self, message):
        if rospy.get_param("use_joy_action"):
            self._current_joy_priority = message
            self._logger.log(
                "Current joy priority: %s." % str(self._current_joy_priority.data)
            )

    def publish_feedback(self, state):
        feedback_ = TurnToHumanFeedback()
        feedback_.status = state
        feedback_.orientation = self._current_odom.pose.pose.orientation
        self._action_server.publish_feedback(feedback_)

    def publish_status(self, state):
        result = TurnToHumanResult()
        result.status = state
        self._action_server.set_succeeded(result)

    def publish_torso_velocity_command(self, angular_velocity):
        velocity = Twist()
        velocity.angular.z = angular_velocity
        self._velocity_publisher.publish(velocity)

    def call_joy_priority_action(self):
        goal = JoyPriorityGoal()
        self._joy_action_server.send_goal(goal)

    def find_required_angle(self):
        tf = self._tf_listener.lookupTransform(
            rospy.get_param("human_tf"), rospy.get_param("base_link"), rospy.Time(0)
        )
        tfi = tf.inverse()
        return math.atan2(tfi.getOrigin().y(), tfi.getOrigin().x())

    def move_torso(self):
        success = True
        locked_state = self._current_joy_priority.data
        r = rospy.Rate(30)

        velocity = rospy.get_param("torso_turning_velocity")
        initial_angle = self.find_required_angle()

        if not locked_state:
            self.call_joy_priority_action()

        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested() or rospy.is_shutdown():
                self._logger.log("%s: preempted." % self._action_name)
                self._action_server.set_preempted()
                self._head_action_server.stop_tracking_goal()
                success = False
                break

            angle_change = self.find_required_angle()
            if abs(angle_change) < 0.05 or angle_change * initial_angle < 0:
                self.publish_torso_velocity_command(0.0)
                break

            self.publish_torso_velocity_command(
                velocity if angle_change > 0.0 else -velocity
            )
            self.publish_feedback("moving")
            r.sleep()

        self.publish_torso_velocity_command(0.0)

        if not locked_state:
            self.call_joy_priority_action()

        self.reset_head()

        return success

    def get_pose(self, point, origin):
        transformation = self._tf_listener.lookupTransform(point, origin, rospy.Time(0))
        inversed_transformation = transformation.inverse()

        pose = Pose()
        pose.position.x = inversed_transformation.getOrigin().x()
        pose.position.y = inversed_transformation.getOrigin().y()
        pose.position.z = inversed_transformation.getOrigin().z()
        pose.orientation.x = inversed_transformation.getRotation().x()
        pose.orientation.y = inversed_transformation.getRotation().y()
        pose.orientation.z = inversed_transformation.getRotation().z()
        pose.orientation.w = inversed_transformation.getRotation().w()

        return pose

    def move_head(self):
        goal = PointHeadGoal()
        pose = self.get_pose(rospy.get_param("human_tf"), rospy.get_param("base_link"))
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
        self._head_action_server.send_goal(goal)
        self._head_action_server.wait_for_result(rospy.Duration(1.0))
        return True

    def reset_head(self):
        goal = PointHeadGoal()
        goal.target.header.frame_id = "base_footprint"
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
        self._head_action_server.send_goal(goal)
        self._head_action_server.wait_for_result(rospy.Duration(1.0))


def run_server():
    # Initialize the node and create the server instance
    rospy.init_node("turn_to_human_action_server")
    TurnToHumanActionServer()
    rospy.spin()


if __name__ == "__main__":
    run_server()
