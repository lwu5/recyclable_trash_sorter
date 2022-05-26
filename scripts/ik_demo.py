#!/usr/bin/env python3

import rospy
import numpy as np
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

from time import sleep

class InverseKinematics:
    def __init__(self):

        rospy.init_node("inverse_kinematics")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")        

        # Reset arm position
        angles = [0, -0.5, 0, 0]
        self.move_group_arm.go(angles, wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)
                
        # initializing an attribute that will hold the angles
        self.current_joint_angles = angles
        
        # set angle minimums and maximums to make sure that algorithm does not
        # finalize on angles that are outside the physical range of the arm
        self.angle_min = [math.radians(-162), math.radians(-103), math.radians(-53), math.radians(-100)]
        self.angle_max = [math.radians(162), math.radians(90), math.radians(79), math.radians(117)]
        
        print('finished initializing!')
    
    def get_current_location(self, angles):
        '''
        Calculates the x, y, z position of the end effector using 4-DOF forward kinematics equations
        for the turtlebot arm given the 4 joint angles corresponding to the revolute joints of the bot arm. 
        For a guide of the coordinate system: 
        
        x is forward-back (depth) relative to the robot 
            - at x = 0, end effector is roughly above the lidar scan. pos values = pushes effector forwards away from bot. 
              neg values = pushes effector further behind
        y is the yaw 
            - at y = 0, end effector is at 0 degrees. pos values = effector is counterclockwise relative to 0 deg. 
              neg values = the effector is clockwise
        z is the up-down (height) relative to the robot 
            - at z = 0, effector is on the ground. pos values = effector moves up, negative values unattainable
        '''
        
        # height of turtlebot
        turtlebot_height = .141 
        # accounting for the block that makes the robot arm go into an L shape
        # (between second and third joints)
        block = math.atan(.024/.130)

        # lengths of the robot arm
        l_1 = .042 # distance from first joint to second joint (m)
        l_2 = .130 # distance from second joint to third joint (m)
        l_3 = .124 # distance from third joint to fourth joint (m)
        l_4 = .126 # distance from fourth joint to end effector (m)

        # joint angles to input into forward kinematics equations
        # transformed to account for the L shape configuration of turtlebot arm
        j_1 = -1*(angles[0])
        j_2 = math.radians(90) - (angles[1] + block)
        j_3 = -1*(angles[2] + math.radians(90))
        j_4 = -1*(angles[3])

        # forward kinematics equations for 4-DOF, solves for x, y, z in real life coordinates
        curr_x = math.cos(j_1)*(l_2*math.cos(j_2)+l_3*math.cos(j_2 + j_3)) + l_4*math.cos(j_1)*math.cos(j_2+j_3+j_4)
        curr_y = math.sin(j_1)*(l_2*math.cos(j_2)+l_3*math.cos(j_2 + j_3)) + l_4*math.sin(j_1)*math.cos(j_2+j_3+j_4)
        curr_z = l_1 + l_2*math.sin(j_2) + l_3*math.sin(j_2+j_3) + l_4*math.sin(j_2+j_3+j_4)

        # need to adjust for the height of the turtlebot and 
        # the small block height (0.035 m) between the first joint and the flat top of the robot.
        curr_z += 0.035 + turtlebot_height 

        # to offset for the fact that x = 0 is around the center of the lidar scanner
        curr_x -= 0.075 
        
        # the curr_y is flipped in the robot arm coordinate system, 
        # need to account for that
        curr_y = curr_y *-1
        
        # return the x, y, z calculated from the input joint angles
        current_location = [curr_x, curr_y, curr_z]
        return current_location

    def get_joint_dist(self, angles, goal_location):
        '''
        Returns distance between the current xyz location of the end effector
        to the xyz of the goal location.
        '''
        
        current_location = self.get_current_location(angles)
        curr_location_array = np.asarray(current_location)

        goal_location_array = np.asarray(goal_location)
        
        distance = np.linalg.norm(curr_location_array - goal_location_array) 
        return distance
    
    def clamp_angles(self, angle_idx):
        '''
        Clamps the joint angles so that they never exceed the physical limits of the
        arm's joint motion when solving for an IK solution during gradient descent
        '''
        if self.current_joint_angles[angle_idx] < self.angle_min[angle_idx]:
            self.current_joint_angles[angle_idx] = self.angle_min[angle_idx]
        elif self.current_joint_angles[angle_idx] > self.angle_max[angle_idx]:
            self.current_joint_angles[angle_idx] = self.angle_max[angle_idx]


    def gradient_descent(self, angle_idx, goal_location):
        '''
        Computes the gradient for a given joint angle to see in which direction (+ or -) and how much
        the joint angle should change to minimize the distance of the current xyz position of the 
        end effector to the goal xyz position
        
        Reference: https://www.alanzucconi.com/2017/04/10/gradient-descent/
        '''
        # delta value controls how much we should change the joint angle to determine
        # the gradient 
        delta = 0.2 
        
        # calculates the distance between current xyz to goal xyz before adding delta
        old_distance = self.get_joint_dist(self.current_joint_angles, goal_location)

        # storing old joint angle
        old_joint_angle = self.current_joint_angles[angle_idx]
        # add delta to angle and then compute new post-delta xyz to goal xyz distance
        self.current_joint_angles[angle_idx] += delta
        new_distance = self.get_joint_dist(self.current_joint_angles, goal_location)

        # difference between distances / delta is the derivative, aka the gradient
        gradient = (new_distance - old_distance) / delta
        
        # changing back the angle 
        self.current_joint_angles[angle_idx] = old_joint_angle
        
        return gradient
 
    def run(self):
        '''
        Uses gradient descent to compute how much to move arms. This run function makes the end effector
        move in 7 locations. It first moves outwards, and then draw a star shape in the next 6 locations
        (starts from lower left corner, then apex corner of star, lower right corner, middle left corner, 
        middle right corner, then lower left corner again).
            
        reference: https://www.alanzucconi.com/2017/04/10/robotic-arms/
        '''
        
        rospy.sleep(3)
        # learning rate tau controls how much the angle changes after each iteration of gradient descent
        # since it is multiplied to the gradient
        tau = 0.05 
        # threshold (m) at which we will consider the algorithm to have converged;
        # current xyz position is close enough to goal xyz position
        distance_threshold = 0.075
        
        # a predefined set of locations to move to; this makes the effector draw a star!
        locations = [[0.3, 0, 0.2], [0.15, -0.15, 0.1], [0.05, 0, 0.4], [0.15, 0.15, .1], [0.1, -0.2, 0.3], [0.1, 0.2, 0.3], [0.1, -0.15, 0.1]]
        
        # iterate over each location to draw a star
        for goal_location in locations:
            reached_goal = False
            while not reached_goal:
                # check if we're already close enough to the goal location such that algorithm doesn't need to be run
                if self.get_joint_dist(self.current_joint_angles, goal_location) <= distance_threshold:
                    print('reached goal')
                    reached_goal = True
                else:
                    # iterate over all 4 joints
                    for i in (range(len(self.current_joint_angles))): 
                        # determine gradient for that joint
                        gradient = self.gradient_descent(i, goal_location)
                        # travel down the gradient and go towards a minimum (b/w current xyz and goal xyz)
                        # by adjusting the angle based on the calculated gradient
                        self.current_joint_angles[i] -= tau * gradient
                        # clamping to make sure that angle values do not go out of bounds
                        self.clamp_angles(i)
                    # check if converged, close enough to goal
                    if self.get_joint_dist(self.current_joint_angles, goal_location) < distance_threshold:
                        print('converged')
                        # move the arm to match the goal
                        self.move_group_arm.go(self.current_joint_angles, wait=True)
                        reached_goal = True
                        # sleep to make sure the arm movement is executed
                        rospy.sleep(3.5)
                rospy.sleep(0.01)
        rospy.spin()

if __name__ == "__main__":
    node = InverseKinematics()
    node.run()
