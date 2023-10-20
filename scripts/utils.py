#!/usr/bin/env python

import rospy


def wait_until_server_ready(server, server_name, logger, timeout=rospy.Duration()):
    """
    Waits until the server is ready and logs the process.

    Args:
        server (SimpleActionServer): The server handle.
        server_name (str): Server's name for logging purposes.
        server_name (Logger): Logger class instance.
        timeout(rospy.Duration, optional): The max timeout to wait.
    """
    logger.log("Waiting for %s server..." % server_name)
    if server.wait_for_server(timeout):
        logger.log("%s server ready." % server_name)
    else:
        logger.log("Failed to connect to %s." % server_name)


def get_pose_string_representation(pose):
    """
    Returns a string representation of a Pose message.

    Args:
        pose (Pose): The pose.

    Returns:
        str: A string representation of the given pose.
    """
    position_str = "{x: %f, y: %f, z: %f}" % (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )
    orientation_str = "{x: %f, y: %f, z: %f, w: %f}" % (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    return "{position: %s, orientation: %s}" % (position_str, orientation_str)
