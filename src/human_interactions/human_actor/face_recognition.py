#!/usr/bin/env python2.7
import rospy
from camera_capture import CameraCapture
import face_recognition
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class FaceRegonition:
    INPUT_IMAGE_PATH = "/tmp/img.png"
    RESULT_IMAGE_PATH = "/tmp/face_recog.png"

    def __init__(self):
        self._camera_capture = CameraCapture()
        self._image = None

        self._face_detected = False

    def detect(self):
        print("Running face detection for '{}'.".format(self.INPUT_IMAGE_PATH))
        self._camera_capture.save_last_frame(self.INPUT_IMAGE_PATH)

        self._image = face_recognition.load_image_file(self.INPUT_IMAGE_PATH)
        self._face_locations = face_recognition.face_locations(self._image)

        self._save_frame()

        return len(self._face_locations) > 0

    def _save_frame(self):
        print("Saving face detection result '{}'.".format(self.RESULT_IMAGE_PATH))
        plt.imshow(self._image)

        for face_location in self._face_locations:
            top, right, bottom, left = face_location
            rect = patches.Rectangle(
                (left, top),
                right - left,
                bottom - top,
                linewidth=2,
                edgecolor="r",
                facecolor="none",
            )
            plt.gca().add_patch(rect)

        plt.savefig(self.RESULT_IMAGE_PATH)


if __name__ == "__main__":
    rospy.init_node("face_recognition")

    human_detector = FaceRegonition()
    print("Detected: '{}'.".format(human_detector.detect()))
