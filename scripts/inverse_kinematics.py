#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3

from time import sleep

class inverse_kinematics:
    def __init__(self):

        rospy.init_node("inverse_kinematics")

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
            Image, self.image_callback)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0.0, 
                                math.radians(-60.0), 
                                math.radians(60.0),
                                math.radians(0.0)], wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)

    def kinematics(self):
        r = rospy.Rate(10)
        r.sleep()