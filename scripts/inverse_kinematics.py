#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3

from time import sleep

class InverseKinematics:
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
        #self.current_location = moveit_commander.move_group.MoveGroupCommander.get_current_pose()

        self.goal_location = [0.230, 0.5, 0.288]

        # Reset arm position
        
        #self.move_group_arm.go([0.0, 
        #                        math.radians(-60.0), 
        #                        math.radians(60.0),
        #                        math.radians(0.0)], wait=True)
        angles = [-0.003, 0.313, -0.144, -0.476]
        self.move_group_arm.go(angles, wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)

        #pose_target = geometry_msgs.msg.Pose()
        #pose_target.orientation.w = 1.0
        #pose_target.position.x = goal_location[0]
        #pose_target.position.y = goal_location[1]
        #pose_target.position.z = goal_location[2]
        #group.move_group.MoveGroupCommander.set_pose_target(pose_target)


        self.current_joint_angles = angles #moveit_commander.move_group.MoveGroupCommander.get_current_joint_values()
        self.images = None
        self.scans = None
        
        print('finished initializing!')

    def image_callback(self, img):
        self.images = img
        
    def scan_callback(self, data):
        self.scans = data.ranges
    
    def find_goal_location(self):
        # TODO: from scan and image data, find the end position that we want the robot arm to move to
        if self.images is not None and self.scans is not None:
            # implement object recognition + transformation logic
            # self.goal_location = _________
            print('whatever')

    def get_current_location(self, angles):
        l_1 = .077/2 # distance from second joint to third joint (m)
        l_2 = .130 # distance from third joint to fourth joint (m)
        l_3 = .124 # distance from fourth joint to end effector (m)
        l_4 = .126
        #l_1 = 0.077
        #l_2 = 0.128
        #l_3 = 0.024 + 0.124
        #l_4 = 0.126
        j_1 = angles[0]
        j_2 = angles[1]
        j_3 = angles[2]
        j_4 = angles[3]
        #curr_x = (l_2 * math.cos(j_1) * math.cos(j_2)) + (l_3 * math.cos(j_1) * math.cos(j_2 + j_3))
        #curr_y = (l_2 * math.sin(j_1) * math.cos(j_2)) + (l_3 * math.sin(j_1) * math.cos(j_2 + j_3))
        #curr_z = l_1 + (l_2*math.sin(j_2)) + (l_3*math.sin(j_2 + j_3))
        curr_x = math.cos(j_1)*(l_2*math.cos(j_2)+l_3*math.cos(j_2 + j_3)) + l_4*math.cos(j_1)*math.cos(j_2+j_3+j_4)
        curr_y = math.sin(j_1)*(l_2*math.cos(j_2)+l_3*math.cos(j_2 + j_3)) + l_4*math.sin(j_1)*math.cos(j_2+j_3+j_4)
        curr_z = (l_1 + l_2*math.sin(j_2) + l_3*math.sin(j_2+j_3)) + l_4*math.sin(j_2+j_3+j_4)
        curr_z += 0.077/2 # to offset for the problematic angle

        current_location = [curr_x, curr_y, curr_z]
        return current_location

    def get_joint_dist(self, angles, goal_location):
        #print('angles', angles)
        #print('goal_location', goal_location)

        current_location = self.get_current_location(angles)
        curr_location_array = np.asarray(current_location)

        goal_location_array = np.asarray(goal_location)
        
        distance = np.linalg.norm(curr_location_array - goal_location_array) 
        return distance

    def gradient_descent(self, angle_idx):
        # Computes gradient based on joint distance of each joint.
        # distance from the target
        # reference: https://www.alanzucconi.com/2017/04/10/gradient-descent/
        delta = 0.2 # TODO: figure out good delta for gradient descent
        old_distance = self.get_joint_dist(self.current_joint_angles, self.goal_location)

        old_joint_angle = self.current_joint_angles[angle_idx]
        self.current_joint_angles[angle_idx] += delta
      
        new_distance = self.get_joint_dist(self.current_joint_angles, self.goal_location)

        gradient = (new_distance - old_distance) / delta
        print(gradient)
        print('finished gradient descent')
        # changing back the angle 
        self.current_joint_angles[angle_idx] = old_joint_angle
        return gradient
 
    def run(self):
        # Uses gradient descent to compute how much to move arms.
        # reference: https://www.alanzucconi.com/2017/04/10/robotic-arms/
        rospy.sleep(3)
        # TODO: choose good learning rate
        tau = 0.01 # learning rate
        # TODO: choose good distance threshold
        distance_threshold = 0.1
        reached_goal = False
        
        
        while not reached_goal:
            print('first one')
            print(self.get_current_location(self.current_joint_angles))
            if self.get_joint_dist(self.current_joint_angles, self.goal_location) < distance_threshold:
                print('reached goal')
                reached_goal = True
            else:
                for i in (range(len(self.current_joint_angles))): # make sure not to update last joint
                    gradient = self.gradient_descent(i)
                    self.current_joint_angles[i] -= tau * gradient
                #self.move_group_arm.go(self.current_joint_angles, wait=True)
                #rospy.sleep(1)
                print('update after gradient descent')
                print(self.get_current_location(self.current_joint_angles))
                if self.get_joint_dist(self.current_joint_angles, self.goal_location) < distance_threshold:
                    print('also reached goal')
                    # move the arm to match the goal
                    self.move_group_arm.go(self.current_joint_angles, wait=True)
                    reached_goal = True
            rospy.sleep(0.1)
        rospy.spin()

if __name__ == "__main__":
    node = InverseKinematics()
    node.run()

    '''      
    def get_joint_dist(self, current_location, goal_location):
        # TODO: calculates current position from our current joint angles (forward kinematics)
        #   use forward kinematics equations: inputs = joint angles 2, 3, 4
        #   outputs: array of distances x, y, z
        # OR current pose of the arm, through a method; outputs: x, y, z
        curr_location_array = np.asarray(current_location)

        # TODO: then get goal position from camera/lidar data
        goal_location_array = np.asarray(goal_location)

        # TODO: compute distance between current pose and goal pose
        distance = numpy.linalg.norm(curr_location_array - goal_location_array) 

        return distance
    
    def gradient_descent(self):
        # TODO: computes gradient based on joint distance of each joint.
        # distance from the target
        # reference: https://www.alanzucconi.com/2017/04/10/gradient-descent/
        delta = 1 # TODO: figure out good delta for gradient descent
        old_distance = self.get_joint_dist(self.current_location, self.goal_location)
        new_joint_angles = []
        for angle in self.current_joint_angles:
            new_joint_angles.append(angle + delta)
        self.move_group_arm.go(new_joint_angles, wait=True)
        rospy.sleep(1)

        # check get_current_pose() if it actually uses forward kinematics
        new_location = moveit_commander.move_group.MoveGroupCommander.get_current_pose()
        new_distance = self.get_joint_dist(new_location, self.goal_location)

        gradient = (new_distance - old_distance) / delta
        
        return gradient
    

        # find the current position and the end effector position
            # inputting in joint angles and then getting current position from forwards kinematics
            # OR get_current_pose to get current position, not using forward kinematiccs
        # gradient
            # distance given that we've moved the angles + delta
                # input in the new joint angles and then get hypothetical new position from forward kinematics equations
                # OR we have to get new_hypothetical_pose from get_current_pose AFTER making the robot move to those new angles
    '''
