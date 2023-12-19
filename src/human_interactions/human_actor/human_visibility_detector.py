#!/usr/bin/env python2.7
import rospy
from pal_detection_msgs.msg import Detections2d


class HumanVisibilityDetector:
    def __init__(self):
        self._detections_subscriber = rospy.Subscriber(
            "/person_detector/detections", Detections2d, self._person_detection_cb
        )
        self._is_person_detected = False

    def _person_detection_cb(self, detections_msg):
        detections = detections_msg.detections
        self._is_person_detected = len(detections) != 0

    def is_visible(self):
        return self._is_person_detected


if __name__ == "__main__":
    rospy.init_node("human_visibility_detector")

    human_detector = HumanVisibilityDetector()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        print("\rHuman visible: {}".format(human_detector.is_visible()))
        rate.sleep()
