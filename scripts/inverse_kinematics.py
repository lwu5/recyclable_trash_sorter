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

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # absolute position of end effector
        self.current_location = moveit_commander.move_group.MoveGroupCommander.get_current_pose()

        # Reset arm position
        self.move_group_arm.go([0.0, 
                                math.radians(-60.0), 
                                math.radians(60.0),
                                math.radians(0.0)], wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)

        self.current_joint_angles = moveit_commander.move_group.MoveGroupCommander.get_current_joint_values()
        self.images = None
        self.scans = None

    def image_callback(self, img):
        self.images = img
        
    def scan_callback(self, data):
        self.scans = data.ranges
    
    def find_goal_location(self):
        # TODO: from scan and image data, find the end position that we want the robot arm to move to
        if self.images is not None and self.scans is not None:
            # implement object recognition + transformation logic
            return

    def kinematics(self):
        # Uses gradient descent to compute how much to move arms.
        # TODO: compute gradient and then update the angles
        # reference: https://www.alanzucconi.com/2017/04/10/robotic-arms/

        r = rospy.Rate(10)
        r.sleep()
    
    def joint_dist(self):
        # TODO: calculates current position from our current joint angles (forward kinematics)
        #   use forward kinematics equations: inputs = joint angles 2, 3, 4
        #   outputs: array of distances x, y, z
        # OR current pose of the arm, through a method; outputs: x, y, z

        # TODO: then get goal position from camera/lidar data
        # TODO: compute distance between current pose and goal pose
        
        return
    
    def gradient_descent(self):
        # TODO: computes gradient based on joint distance of each joint.
        # distance from the target
        # reference: https://www.alanzucconi.com/2017/04/10/gradient-descent/
        delta = 1 # TODO: figure out good delta for gradient descent
        distance = self.joint_dist()
        new_joint_angles = []
        for angle in self.current_joint_angles:
            new_joint_angles.append(angle + delta)
        self.move_group_arm.go(new_joint_angles, wait=True)
        rospy.sleep(1)

        # check get_current_pose() if it actually uses forward kinematics
        new_location = moveit_commander.move_group.MoveGroupCommander.get_current_pose()
        self.joint_dist()
        
        return


        # find the current position and the end effector position
            # inputting in joint angles and then getting current position from forwards kinematics
            # OR get_current_pose to get current position, not using forward kinematiccs
        # gradient
            # distance given that we've moved the angles + delta
                # input in the new joint angles and then get hypothetical new position from forward kinematics equations
                # OR we have to get new_hypothetical_pose from get_current_pose AFTER making the robot move to those new angles