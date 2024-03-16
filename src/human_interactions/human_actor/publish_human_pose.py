#!/usr/bin/env python2.7

import json

import rospy
import tf

if __name__ == "__main__":
    rospy.init_node("publish_human_pose", anonymous=True)

    actor_name = rospy.get_param("~actor_name")
    topic = rospy.get_param("~topic")

    actor_pose_str = rospy.get_param("/{}/pose".format(actor_name))
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
