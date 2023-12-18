#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("rotate_rico")

    velocity_publisher = rospy.Publisher("/joy_vel", Twist, queue_size=1000)

    def publish_rotation_velocity(rotation_velocity):
        velocity = Twist()
        velocity.angular.z = rotation_velocity
        velocity_publisher.publish(velocity)

    stop = lambda: publish_rotation_velocity(0.0)
    rotate = lambda: publish_rotation_velocity(
        rospy.get_param("~velocity", default=1.0)
    )

    loop_rate = rospy.Rate(20)
    while True:
        rotate()
        # stop()

        loop_rate.sleep()
