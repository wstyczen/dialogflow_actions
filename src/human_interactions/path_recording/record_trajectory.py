#!/usr/bin/env python

import os
import rospy
import signal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose
from human_interactions.utility.tf_provider import TFProvider


class TrajectoryRecorder:
    def __init__(self, output_file_path):
        self._odom_subscriber = rospy.Subscriber(
            "/mobile_base_controller/odom", Odometry, self.__add_new_pose
        )
        self.__current_pose = None

        self.__path = Path()
        self.__path.header.frame_id = 'map'

        self.__output_file_path = output_file_path

        self.__tf_provider = TFProvider()

    def __get_map_pose_transform(self):
        transform = TransformStamped()
        transform.transform = self.__tf_provider.get_tf("odom", "map")

        transformed_pose = do_transform_pose(self.__current_pose, transform)
        return transformed_pose

    def __add_new_pose(self, msg):

        self.__current_pose = PoseStamped()
        self.__current_pose.header = msg.header
        self.__current_pose.pose = msg.pose.pose

        self.__on_new_pose()

    def __on_new_pose(self):
        # Convert the pose to target frame and it to trajectory
        transformed_pose = self.__get_map_pose_transform()
        if transformed_pose is not None:
            self.__path.poses.append(transformed_pose)

    def save(self):
        with open(self.__output_file_path, "w") as f:
          f.write(str(self.__path))

        exit()

if __name__ == '__main__':
    rospy.init_node("record_trajectory", anonymous=True)

    index = rospy.get_param("index")
    out_file_path = os.path.join(os.path.dirname(__file__), "paths", "path{}.yaml".format(index))
    recorder = TrajectoryRecorder(out_file_path)

    # On Ctrl+C
    signal.signal(signal.SIGINT, lambda sig, frame: recorder.save())

    rospy.spin()
