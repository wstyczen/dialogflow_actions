#!/usr/bin/env python
import rospy
from rospkg import RosPack

import os
from datetime import datetime
from enum import Enum


def get_time_str():
    """
    Get the current time as a formatted string.

    Returns:
        str: The current time in the 'HH:MM:SS' format.
    """
    return datetime.now().strftime("%H:%M:%S")


class LogLevel(str, Enum):
    """
    Enumeration describing available log levels.
    """

    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class Logger(object):
    """
    Provides logging functionality.

    Class Attributes:
        PACKAGE_NAME (str): The name of the ROS package.
        PACKAGE_PATH (str): The path to the ROS package.
        LOGS_DIRECTORY (str): Directory to store logs in.

    Instance Attributes:
        _module_name (str): Name of the module or component for logging.
        _save_logs_to_file (bool): Whether the logs should be saved to a file.
        _init_timestamp (datetime): Timestamp of initialization.
        _file_name (str): Name of the log file to output the logs to.
    """

    PACKAGE_NAME = "dialogflow_actions"
    PACKAGE_PATH = RosPack().get_path(PACKAGE_NAME)
    LOGS_DIRECTORY = "logs"

    def __init__(self, module_name, save_logs_to_file=True):
        """
        Initialize the Logger.

        Args:
            module_name (str): Name of the module or component.
            save_logs_to_file (bool): Whether the logs should be saved to a file.
        """
        self._module_name = module_name
        self._save_logs_to_file = save_logs_to_file
        self._init_timestamp = datetime.now()
        self._file_name = self.get_file_name()

    def log(self, contents, level=LogLevel.INFO):
        """
        Log a message with specified contents and log level.

        Args:
            contents (str): Contents of the log message.
            level (LogLevel, optional): Log level (INFO by default).
        """
        self.output_to_screen(contents, level)
        if self._save_logs_to_file:
            self.output_to_file(contents, level)

    def get_logs_directory(self):
        """
        Get the path of the directory to save the logs to.

        Returns:
            str: Directory path.
        """
        return "%s/%s" % (Logger.PACKAGE_PATH, Logger.LOGS_DIRECTORY)

    def get_file_name(self):
        """
        Get a unique log file name, generated based on the module name and timestamp.

        Returns:
            str: File name.
        """
        return "%s_%s.log" % (
            self._module_name,
            self._init_timestamp.strftime("%H_%M_%S"),
        )

    def get_file_path(self):
        """
        Get a full path for to save a log file.

        Returns:
            str: Log file path.
        """
        return "%s/%s" % (self.get_logs_directory(), self.get_file_name())

    def output_to_screen(self, contents, level):
        """
        Output a log message to the screen.

        Args:
            contents (str): Contents of the message.
            level (LogLevel): Log level.
        """
        msg = "[%s] %s" % (self._module_name, contents)
        if level == LogLevel.INFO:
            rospy.loginfo(msg)
        elif level == LogLevel.WARNING:
            rospy.logwarn(msg)
        elif level == LogLevel.ERROR:
            rospy.logerr(msg)
        else:
            assert False, "Invalid log level."

    def output_to_file(self, contents, level):
        """
        Output a log message to the file.

        Args:
            contents (str): Contents of the message.
            level (LogLevel): Log level.
        """
        logs_dir = self.get_logs_directory()
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)

        with open(self.get_file_path(), "a") as logs_file:
            logs_file.write("[%s] [%s]: %s\n" % (level.value, get_time_str(), contents))


class Action(str, Enum):
    """
    Enumeration representing all the actions that should be logged.
    """

    TURN_TO_HUMAN = "turn_to_human"
    MOVE_TO_HUMAN = "move_to_human"


class ActionServerLogger(Logger):
    """
    Logger for action servers.
    """

    def __init__(self, action):
        super(ActionServerLogger, self).__init__("%s_server" % action.value)
        self._action = action


class ActionClientLogger(Logger):
    """
    Logger for action clients.
    """

    def __init__(self, action):
        super(ActionClientLogger, self).__init__("%s_client" % action.value)
        self._action = action
