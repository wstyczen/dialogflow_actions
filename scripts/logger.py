#!/usr/bin/env python
import rospy
from rospkg import RosPack

import os
from datetime import datetime
from enum import Enum


def get_time_str():
    return datetime.now().strftime("%H:%M:%S")


class Logger(object):
    PACKAGE_NAME = "dialogflow_emergency_action_servers"
    PACKAGE_PATH = RosPack().get_path(PACKAGE_NAME)
    LOGS_DIRECTORY = "logs"

    class Level(str, Enum):
        INFO = "INFO"
        WARNING = "WARNING"
        ERROR = "ERROR"

    def __init__(self, module_name, save_logs_to_file=True):
        self._module_name = module_name
        self._save_logs_to_file = save_logs_to_file
        self._init_timestamp = datetime.now()
        self._file_name = self.get_file_name()

    def log(self, contents, level=Level.INFO):
        self.output_to_screen(contents, level)
        if self._save_logs_to_file:
            self.output_to_file(contents, level)

    def get_logs_directory(self):
        return "%s/%s" % (Logger.PACKAGE_PATH, Logger.LOGS_DIRECTORY)

    def get_file_name(self):
        return "%s_%s.log" % (
            self._module_name,
            self._init_timestamp.strftime("%H_%M_%S_%f"),
        )

    def get_file_path(self):
        return "%s/%s" % (self.get_logs_directory(), self.get_file_name())

    def output_to_screen(self, contents, level):
        msg = "[%s]: %s" % (self._module_name, contents)
        if level == Logger.Level.INFO:
            rospy.loginfo(msg)
        elif level == Logger.Level.WARNING:
            rospy.logwarn(msg)
        elif level == Logger.Level.ERROR:
            rospy.logerr(msg)
        else:
            assert False, "Invalid log level."

    def output_to_file(self, contents, level):
        logs_dir = self.get_logs_directory()
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)

        with open(self.get_file_path(), "a") as logs_file:
            logs_file.write("[%s] [%s]: %s\n" % (level.value, get_time_str(), contents))


class ActionServerLogger(Logger):
    class Action(str, Enum):
        TURN_TO_HUMAN = "turn_to_human"
        MOVE_TO_HUMAN = "move_to_human"

    def __init__(self, action):
        super(ActionServerLogger, self).__init__(action.value)
        self._action = action


class ActionClientLogger:
    pass
    # Make it an observer to the Client, an print status change on change of feedback ??
