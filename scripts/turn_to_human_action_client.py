#!/usr/bin/env python
import rospy
import actionlib

# Msgs
from dialogflow_actions.msg import (
    TurnToHumanAction,
    TurnToHumanGoal,
)

# Local scripts
from action_client_interface import ActionClientInterface
from logger import ActionClientLogger, Action
from utils import get_pose_string_representation


class TurnToHumanActionClient(ActionClientInterface):
    """
    Client for the 'TurnToHuman' action.

    This class implements the ActionClientInterface, providing all the
    necessary methods and attributes.

    Attributes:
        _action_name (str): The name of the 'TurnToHuman' action.
        _logger (ActionClientLogger): An instance of the logger for this action client.
        _action_client (actionlib.SimpleActionClient): An instance of the action client.
    """

    def __init__(self):
        """
        Initializes the TurnToHumanActionClient.
        """

        self._action_name = rospy.get_param("turn_to_human_action_name")
        self._logger = ActionClientLogger(Action.TURN_TO_HUMAN)
        self._logger.log("Initializing %s client." % self._action_name)

        self._action_client = actionlib.SimpleActionClient(
            self._action_name, TurnToHumanAction
        )

    def feedback_callback(self, feedback_msg):
        """
        Callback for handling feedback received server.

        Args:
            feedback_msg (TurnToHumanFeedback): Feedback message received from server.
        """
        self._logger.log(
            "Server feedback: {moved link: %s, robot's pose: %s}"
            % (
                feedback_msg.link.data,
                get_pose_string_representation(feedback_msg.robot_pose),
            )
        )

    def get_result_callback(self, _, result_msg):
        """
        Callback for handling the action result received server.

        Args:
            _: The result status (unused).
            result_msg (TurnToHumanFeedback): Result received from server.
        """
        self._logger.log(
            "Server returned result: {status: %s, link used: %s, robot's pose: %s}"
            % (
                result_msg.status.data,
                result_msg.link,
                get_pose_string_representation(result_msg.robot_pose),
            )
        )

        self.shut_down()


if __name__ == "__main__":
    rospy.init_node("turn_to_human_action_client")

    client = TurnToHumanActionClient()
    client.send_goal(TurnToHumanGoal())
    client.wait_for_result()

    rospy.spin()
