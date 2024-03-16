#!/usr/bin/env python2.7

import json

import rospy
import tf

def get_pose_param(param_name):
    # If param is unavailable, try again for a few seconds. Maybe the node
    # responsible has not properly started yet.
    retries = 0
    while retries < 5:
        value = rospy.get_param(param_name, None)
        if value is not None:
            return value

        retries += 1
        rospy.logwarn("Failed to retrieve {}. Retrying...".format(param_name))
        rospy.sleep(1)

    rospy.logerr("Failed to retrieve {}. Exiting...".format(param_name))
    exit(1)

if __name__ == "__main__":
    rospy.init_node("publish_human_pose", anonymous=True)

    actor_name = rospy.get_param("~actor_name")
    topic = rospy.get_param("~topic")

    actor_pose_str = get_pose_param("/{}/pose".format(actor_name))
    actor_pose_dict = json.loads(actor_pose_str.replace("'", "\""))
    x = actor_pose_dict['x']
    y = actor_pose_dict['y']
    theta = actor_pose_dict['theta']

    # Based on: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
    rospy.loginfo("Publishing transform of {} to {} at position (x: {}. y: {})".format(actor_name, topic, x, y))
    pose_publisher = tf.TransformBroadcaster()

    pose_publisher.sendTransform((x, y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, theta),
                        rospy.Time.now(),
                        topic,
                       "map")

    rospy.spin()
