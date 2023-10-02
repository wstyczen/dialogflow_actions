import rospy
import math

from actionlib import SimpleActionClient, SimpleActionServer
from tf import TransformListener
# Msgs
from dialogflow_emergency_action_servers.msg import (
    TurnToHumanAction,
    TurnToHumanFeedback,
    TurnToHumanGoal,
    TurnToHumanResult,
)
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion, Pose
# Action servers
from twist_mux_msgs.msg import JoyPriorityAction, JoyPriorityGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from pal_common_msgs.msg import DisableAction, DisableGoal

PI_DEGREES = 180
PI_VALUE = 3.1415

class TurnToHumanActionServer:
    def __init__(self):
        # Init node
        rospy.init_node("turn_to_human_action_server")

        # Init the action server
        self._action_server = SimpleActionServer(
            "turn_to_human", TurnToHumanAction, self.execute_callback, auto_start=False
        )
        self._action_name = rospy.get_param("~served_action_name")

        # Init complementary action servers
        self._joy_action_server = SimpleActionClient(
            rospy.get_param("~joy_priority_action"), JoyPriorityAction
        )
        self._head_action_server = SimpleActionClient(
            rospy.get_param("~point_head_action"), PointHeadAction
        )
        self._disable_action_server = SimpleActionClient(
            rospy.get_param("~disable_autohead_action"), DisableAction
        )

        # Subscribers
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("~odometry_topic"), Odometry, self.robot_odometry_callback
        )
        self._joint_state_subscriber = rospy.Subscriber(
            rospy.get_param("~joint_state_topic"),
            JointState,
            self.robot_joint_state_callback,
        )
        self._joy_priority_subscriber = rospy.Subscriber(
            rospy.get_param("~joy_priority_topic"), Bool, self.joy_priority_callback
        )
        self._tf_listener = TransformListener()

        # Publishers
        self._velocity_publisher = rospy.Publisher(
            rospy.get_param("~command_velocity_topic"), Twist, queue_size=1000
        )

        # Current state variables
        self._current_odom = Odometry()
        self._current_joint_state = JointState()
        self._current_joy_priority = Bool()

        # Start action servers
        self._action_server.start()
        rospy.loginfo("%s server ready", rospy.get_param("~served_action_name"))

        self._head_action_server.wait_for_server()
        rospy.loginfo("%s client ready", rospy.get_param("~point_head_action"))

        self._disable_action_server.wait_for_server()
        rospy.loginfo("%s client ready", rospy.get_param("~disable_autohead_action"))

        if rospy.get_param("~use_joy_action"):
            rospy.loginfo("joy priority action is being used")
            self._joy_action_server.wait_for_server()
            rospy.loginfo("%s client ready", rospy.get_param("~joy_priority_action"))

    def execute_callback(self, goal: TurnToHumanGoal) -> None:
        rospy.loginfo("New goal requested")

        success = True;
        goal = DisableGoal()
        self._disable_action_server.send_goal(goal)

        required_angle = self.find_required_angle()
        rospy.loginfo("Desired change in yaw: %f degrees", (PI_DEGREES*required_angle)/PI_VALUE)

        # If possible, move just the head to face the speaker. If the angle is
        # out of its range of motion move the torso.
        max_head_rotation_radian = rospy.get_param("max_head_rotation")
        if abs(required_angle) > max_head_rotation_radian:
            rospy.loginfo("Moving torso")
            success = self.move_torso()
        else:
            rospy.loginfo("Moving head")
            success = self.move_head()

        rospy.loginfo("Finished executing.")

        if success:
          self.publish_status("finished")
        else:
          self._action_server.set_aborted()

    def robot_odometry_callback(self, message: Odometry) -> None:
        self._current_odom = message

    def robot_joint_state_callback(self, message: JointState) -> None:
        self._current_joint_state = message

    def joy_priority_callback(self, message: Bool) -> None:
        if rospy.get_param("~use_joy_action"):
            self._current_joy_priority = message
            rospy.loginfo(
                "Current joy priority: %s", str(self._current_joy_priority.data)
            )

    def publish_feedback(self, state: str) -> None:
        feedback_ = TurnToHumanFeedback()
        feedback_.status = state
        feedback_.orientation = self._current_odom.pose.pose.orientation
        self._action_server.publish_feedback(feedback_)

    def publish_status(self, state: str) -> None:
        result_ = TurnToHumanResult()
        result_.status = state
        self._action_server.set_succeeded(result_)
        rospy.loginfo("%s: Succeeded", self._action_name)

    def publish_torso_velocity_command(self, angular_velocity: float) -> None:
        velocity = Twist()
        velocity.angular.z = angular_velocity
        self._velocity_publisher.publish(velocity)

    def call_joy_priority_action(self) -> None:
        goal = JoyPriorityGoal()
        self._joy_action_server.send_goal(goal)

    def find_required_angle(self) -> float:
        tf = self._tf_listener.lookupTransform(
            rospy.get_param("~human_tf"), rospy.get_param("~base_link"), rospy.Time(0)
        )
        tfi = tf.inverse()
        return math.atan2(tfi.getOrigin().y(), tfi.getOrigin().x())

    def move_torso(self) -> bool:
        success = True
        locked_state = self._current_joy_priority.data
        r = rospy.Rate(30)

        velocity = rospy.get_param("~torso_turning_velocity")
        initial_angle = self.find_required_angle()

        if not locked_state:
            self.call_joy_priority_action()

        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo("%s: Preempted", self._action_name)
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
        pose = self.get_pose(
            rospy.get_param("~human_tf"), rospy.get_param("~base_link")
        )
        rospy.loginfo("%f %f %f", pose.position.x, pose.position.y, pose.position.z)
        goal.target.header.frame_id = rospy.get_param("~base_link")
        goal.target.point.x = pose.position.x
        goal.target.point.y = pose.position.y
        goal.target.point.z = pose.position.z
        goal.pointing_axis.x = 1.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 0.0
        goal.pointing_frame = rospy.get_param(
            "/head_controller/point_head_action/tilt_link"
        )
        goal.max_velocity = rospy.get_param("~head_turning_velocity")
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
        goal.max_velocity = rospy.get_param("~head_turning_velocity")
        self._head_action_server.send_goal(goal)
        self._head_action_server.wait_for_result(rospy.Duration(1.0))
