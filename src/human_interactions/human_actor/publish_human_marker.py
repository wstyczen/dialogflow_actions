#!/usr/bin/env python3.6

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA


def parse_pose(string):
    pose_list = [float(x) for x in string.split()]

    pose = Pose()
    pose.position = Point(pose_list[0], pose_list[1], 0)
    pose.orientation = Quaternion(x=1.0, y=pose_list[4], z=pose_list[5], w=1.0)
    return pose


def get_marker(pose, color=None):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "/map"

    marker.ns = "human"
    marker.id = 0
    marker.type = 10
    marker.action = 0

    marker.pose = pose
    marker.scale = Vector3(1.0, 1.0, 1.0)
    marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0) if not color else ColorRGBA(*color)

    marker.mesh_resource = rospy.get_param("human_rviz_model")

    return marker


if __name__ == "__main__":
    rospy.init_node("marker_publisher", anonymous=True)

    marker_publisher = rospy.Publisher(rospy.get_param("~topic"), Marker, queue_size=10)
    pose = parse_pose(rospy.get_param("~pose"))
    marker_color = rospy.get_param("~color", None)
    if marker_color is not None:
        marker_color = [float(value) for value in marker_color.split()]

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        marker_publisher.publish(get_marker(pose, marker_color))
        rate.sleep()
