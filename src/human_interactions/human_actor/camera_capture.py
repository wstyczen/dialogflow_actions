#!/usr/bin/env python2.7
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraCapture:
    def __init__(self):
        self._cv_bridge = CvBridge()

        self._image_topic = rospy.get_param("image_topic")
        self._image_subscriber = rospy.Subscriber(
            self._image_topic, Image, self._update_camera_image
        )
        self._image = None

    def _update_camera_image(self, image_msg):
        self._image = self._cv_bridge.imgmsg_to_cv2(
            image_msg, desired_encoding="passthrough"
        )

    def has_frame(self):
        return self._image is not None

    def _save_last_frame(self, path):
        if self._image is None:
            rospy.logerr(
                "Camera image was not found on .{}, failed to save it to {}.".format(
                    self._image_topic, path
                )
            )
            return False

        cv2.imwrite(path, self._image)
        rospy.loginfo("Saved last received camera frame to {}.".format(path))
        return True

    def save_last_frame(self, path):
        rate = rospy.Rate(10)
        while not self.has_frame():
            rate.sleep()

        self._save_last_frame(path)


if __name__ == "__main__":
    rospy.init_node("camera_capture")

    camera_capture = CameraCapture()

    # Wait until a frame from camera is received and save it.
    print("Waiting for a frame from camera...")
    camera_capture.save_last_frame("/tmp/frame.png")
