#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import yaml

def publish_path_from_file(file_path, topic_name):
    path_pub = rospy.Publisher(topic_name, Path, queue_size=10)

    # Read the YAML file containing the Path message
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    # Construct the Path message from the parsed YAML data
    path_msg = Path()
    path_msg.header.seq = data['header']['seq']
    path_msg.header.stamp.secs = data['header']['stamp']['secs']
    path_msg.header.stamp.nsecs = data['header']['stamp']['nsecs']
    path_msg.header.frame_id = data['header']['frame_id']

    poses = data['poses']
    for pose_data in poses:
        pose_stamped = PoseStamped()
        pose_stamped.header.seq = pose_data['header']['seq']
        pose_stamped.header.stamp.secs = pose_data['header']['stamp']['secs']
        pose_stamped.header.stamp.nsecs = pose_data['header']['stamp']['nsecs']
        pose_stamped.header.frame_id = pose_data['header']['frame_id']
        pose_stamped.pose.position.x = pose_data['pose']['position']['x']
        pose_stamped.pose.position.y = pose_data['pose']['position']['y']
        pose_stamped.pose.position.z = pose_data['pose']['position']['z']
        pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
        pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
        pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
        pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']
        path_msg.poses.append(pose_stamped)

    # Publish the reconstructed Path message
    rate = rospy.Rate(10)  # Publish at 10 Hz
    while not rospy.is_shutdown():
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("publish_path_from_file", anonymous=True)

    index = rospy.get_param("~index")
    yaml_path = os.path.join(os.path.dirname(__file__), "paths", "path{}.yaml".format(index))
    topic_name = "/path{}".format(index)

    publish_path_from_file(yaml_path, topic_name)
