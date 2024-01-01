#!/usr/bin/env python
import rospy

from abc import abstractmethod


class ActionClientInterface:
    """
    An interface for action clients which will mostly depend on
    actionlib.SimpleActionClient, while offering some common logging and
    hiding basic initialization.

    Attributes:
        _action_name (str): The name of the action.
        _logger (Logger): An implementation of Logger from logger.py.
        _action_client (actionlib.SimpleActionClient): An instance of actionlib.SimpleActionClient.

    All of the attributes need to be initialized in concrete implementations.
    """

    def send_goal(self, goal):
        """
        Send a goal to the action server.

        Args:
            goal: The goal to be sent to the action server.
        """
        self._logger.log("Attempting to send a goal.")

        self._logger.log("Connecting to %s server..." % self._action_name)
        self._action_client.wait_for_server()
        self._logger.log("Connected.")

        self._action_client.send_goal(
            goal,
            active_cb=self.goal_response_callback,
            feedback_cb=self.feedback_callback,
            done_cb=self.get_result_callback,
        )
        self._logger.log("Goal sent to server.")

    def goal_response_callback(self):
        """
        Called upon receiving callback from the action server.
        """
        self._logger.log("Goal accepted.")

    @abstractmethod
    def feedback_callback(self, feedback_msg):
        """
        Handle feedback received from the action server.

        Args:
            feedback_msg: The feedback message received from the action server.
        """
        pass

    @abstractmethod
    def get_result_callback(self, status, result_msg):
        """
        Handle the result received from the action server.

        Args:
            status: The result status.
            result_msg: The result message received from the action server.
        """
        pass

    def wait_for_result(self):
        """
        Wait until the action server returns a result.
        """
        self._logger.log("Waiting for result...")
        self._action_client.wait_for_result()
        self._logger.log("Action finished with status: %i" % self._action_client.get_state())

    def shut_down(self):
        """
        This method signals the ROS node to shut down and logs the event.
        """
        rospy.signal_shutdown("Shutting down %s client." % self._action_name)
        self._logger.log("Shutting down %s client." % self._action_name)
