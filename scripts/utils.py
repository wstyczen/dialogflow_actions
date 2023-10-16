#!/usr/bin/env python

import rospy


def wait_until_server_ready(server, server_name, logger, timeout=rospy.Duration()):
    logger.log("Waiting for %s server..." % server_name)
    if server.wait_for_server(timeout):
        logger.log("%s server ready." % server_name)
    else:
        logger.log("Failed to connect to %s." % server_name)


def get_pose_string_representation(pose):
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
