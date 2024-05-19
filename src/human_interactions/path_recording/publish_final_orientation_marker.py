#!/usr/bin/env python

import os
import yaml

import rospy
from visualization_msgs.msg import Marker

def get_final_pose(index):
    yaml_path = os.path.join(os.path.dirname(__file__), "paths", "path{}.yaml".format(index))

    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
        return data['poses'][-1]['pose']

def create_arrow_marker(pose):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "final_orientation"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    marker.pose.position.x = pose['position']['x']
    marker.pose.position.y = pose['position']['y']
    marker.pose.position.z = pose['position']['z']
    marker.pose.orientation.x = pose['orientation']['x']
    marker.pose.orientation.y = pose['orientation']['y']
    marker.pose.orientation.z = pose['orientation']['z']
    marker.pose.orientation.w = pose['orientation']['w']

    # Set the scale of the arrow
    marker.scale.x = 0.3 # length
    marker.scale.y = 0.02 # width of shaft
    marker.scale.z = 0.05 # width of head

    # Set the color (r, g, b, a)
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

if __name__ == "__main__":
    rospy.init_node('publish_final_orientation_marker', anonymous=True)

    index = rospy.get_param("~index")
    final_pose = get_final_pose(index)

    topic_name = "/arrow{}".format(index)
    marker_pub = rospy.Publisher(topic_name, Marker, queue_size=10)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker = create_arrow_marker(final_pose)
        marker_pub.publish(marker)
        rate.sleep()
