#!/usr/bin/env python
import rospy
import actionlib

# Msgs
from dialogflow_actions.msg import (
    MoveToHumanAction,
    MoveToHumanGoal,
)

# Local scripts
from action_client_interface import ActionClientInterface
from dialogflow_actions.utility.logger import ActionClientLogger, Action
from dialogflow_actions.utility.utils import get_pose_string_representation


class MoveToHumanActionClient(ActionClientInterface):
    """
    Client for the 'MoveToHuman' action.

    This class implements the ActionClientInterface, providing all the
    necessary methods and attributes.

    Attributes:
        _action_name (str): The name of the 'MoveToHuman' action.
        _logger (ActionClientLogger): An instance of the logger for this action client.
        _action_client (actionlib.SimpleActionClient): An instance of the action client.
    """

    def __init__(self):
        """
        Initializes the MoveToHumanActionClient.
        """
        self._action_name = rospy.get_param("move_to_human_action_name")
        self._logger = ActionClientLogger(Action.MOVE_TO_HUMAN)
        self._logger.log("Initializing %s client." % self._action_name)

        self._action_client = actionlib.SimpleActionClient(
            self._action_name, MoveToHumanAction
        )

    def feedback_callback(self, feedback_msg):
        """
        Callback for handling feedback received server.

        Args:
            feedback_msg (MoveToHumanFeedback): Feedback message received from server.
        """
        self._logger.log(
            "Server feedback -- current pose: %s"
            % get_pose_string_representation(feedback_msg.robot_pose),
        )

    def get_result_callback(self, _, result_msg):
        """
        Callback for handling the action result received from server.

        Args:
            _: The result status (unused).
            result_msg (MoveToHumanResult): The result message received from the action server.
        """

        self._logger.log(
            "Received end result:{status: %s, robot's pose: %s}"
            % (
                result_msg.status.data,
                get_pose_string_representation(result_msg.robot_pose),
            )
        )

        self.shut_down()


if __name__ == "__main__":
    rospy.init_node("move_to_human_action_client")

    client = MoveToHumanActionClient()
    client.send_goal(MoveToHumanGoal())
    client.wait_for_result()

    rospy.spin()
