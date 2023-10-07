#!/usr/bin/env python
import rospy
import actionlib

# Msgs
from dialogflow_emergency_action_servers.msg import (
    TurnToHumanAction,
    TurnToHumanGoal,
    TurnToHumanResult,
)
from logger import ActionClientLogger, Action


class TurnToHumanActionClient:
    def __init__(self):
        self._action_name = rospy.get_param("action_name")
        self._logger = ActionClientLogger(Action.TURN_TO_HUMAN)
        self._logger.log("Initializing %s client." % self._action_name)

        self._action_client = actionlib.SimpleActionClient(
            self._action_name, TurnToHumanAction
        )

    def send_goal(self, goal):
        self._logger.log("Attempting to send a goal.")

        self._action_client.wait_for_server()
        self._logger.log("Connected to server.")

        self._sent_goal_handle = self._action_client.send_goal(
            goal,
            active_cb=self.goal_response_callback,
            feedback_cb=self.feedback_callback,
            done_cb=self.get_result_callback,
        )
        self._logger.log("Goal sent to server.")

    def goal_response_callback(self):
        self._logger.log("Goal accepted.")

    def feedback_callback(self, feedback_msg):
        orientation = feedback_msg.orientation
        orientation_str = "Result orientation: {x\: %f, y\: %f, z: %f, w: %f}" % (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self._logger.log(
            "Server feedback: {status\: %s, orientation\: %s}"
            % (feedback_msg.status, orientation_str)
        )

    def get_result_callback(self, state, result_msg):
        self._logger.log("Received result with state: %s" % state)

        self._logger.log("Result status: %s" % result_msg.status.data)

        rospy.signal_shutdown("Shutting down %s client." % self._action_name)
        self._logger.log("Shutting down %s client", self._action_name)

    def wait_for_result(self):
        self._logger.log("Waiting for result...")
        self._action_client.wait_for_result()


def run_client():
    rospy.init_node("turn_to_human_action_client")

    rospy.loginfo("STARTING TEST CLIENT.")

    client = TurnToHumanActionClient()

    human_pose = TurnToHumanGoal()
    client.send_goal(human_pose)

    client.wait_for_result()
    rospy.loginfo("TEST CLIENT FINISHED.")

    rospy.spin()


if __name__ == "__main__":
    run_client()
