#!/usr/bin/env python

import rospy

from human_interactions.utility.head_controller import HeadController

if __name__ == "__main__":
    rospy.init_node("reset_head")

    HeadController().reset()
