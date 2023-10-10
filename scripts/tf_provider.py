#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Pose, Transform


class TFProvider:
    def __init__(self):
        self._tf_listener = tf.TransformListener()

    def get_tf(self, target_frame, source_frame, inverse=True):
        if inverse:
            target_frame, source_frame = source_frame, target_frame

        try:
            self._tf_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(5.0)
            )
            (translation, rotation) = self._tf_listener.lookupTransform(
                target_frame, source_frame, rospy.Time(0)
            )

            # Convert to Transform message form
            transform = Transform()
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ) = translation
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ) = rotation
            return transform
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logerr("Failed to lookup transform.")
            return None

    def get_map_tf(self, target_frame, inverse=True):
        MAP_TF = "map"
        return sel.get_tf(target_frame, MAP_TF, inverse)

    @staticmethod
    def get_as_pose(transform):
        pose = Pose()

        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation.x = transform.rotation.x
        pose.orientation.y = transform.rotation.y
        pose.orientation.z = transform.rotation.z
        pose.orientation.w = transform.rotation.w

        return pose
