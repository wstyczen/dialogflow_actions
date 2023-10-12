#!/usr/bin/env python

import rospy


def wait_until_server_ready(server, server_name, logger, timeout=rospy.Duration()):
    logger.log("Waiting for %s server..." % server_name)
    if server.wait_for_server(timeout):
        logger.log("%s server ready." % server_name)
    else:
        logger.log("Failed to connect to %s." % server_name)
