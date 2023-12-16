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

    def save_last_frame(self, path):
        if self._image is None:
            rospy.logerr(
                "Camera image was not found on .{}, failed to save it to {}.".format(
                    self._image_topic, path
                )
            )

        cv2.imwrite(path, self._image)
        rospy.loginfo("Saved last received camera frame to {}.".format(path))


if __name__ == "__main__":
    rospy.init_node("camera_capture")

    camera_capture = CameraCapture()

    # Save the current camera frame every couple of seconds
    rate = rospy.Rate(1)  # Hz
    while True:
        camera_capture.save_last_frame("/tmp/image.png")

        rate.sleep()
