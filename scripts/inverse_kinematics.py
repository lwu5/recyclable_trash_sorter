#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

from sensor_msgs.msg import Image, LaserScan

from turtlebot3_msgs.msg import SensorState

from geometry_msgs.msg import Twist, Vector3

from time import sleep


class InverseKinematics:
    def __init__(self):

        rospy.init_node("inverse_kinematics")

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.button_sub = rospy.Subscriber('sensor_state', SensorState, self.button_callback)

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

        # TODO: initialize this as empty, should be found by the other function
        self.goal_location = [0.3, 0.1, 0.3]
        

        # Reset arm position
        #angles = [-0.721, 0.601, 0.086, -0.265]
        angles = [0, -0.5, 0, 0]
        #angles = [.495, .066, .048, .430]
        #angles = [-.555, .403, .187, .270]
        self.move_group_arm.go(angles, wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)
        
        # set angle minimums and maximums to make sure that algorithm does not
        # finalize on angles that are outside the range
        self.angle_min = [math.radians(-162), math.radians(-103), math.radians(-53), math.radians(-100)]
        self.angle_max = [math.radians(162), math.radians(90), math.radians(79), math.radians(117)]
        
        # initializing an attribute that will hold the angles
        self.current_joint_angles = angles

        # initializing attributes that will hold images and scan data
        self.images = None
        self.scans = None

        
        self.is_metal = 0 # 1 = metal
        
        print('finished initializing!')

    def image_callback(self, img):
        self.images = img
        
    def scan_callback(self, data):
        self.scans = data.ranges
    
    def find_goal_location(self):
        # TODO: from scan and image data, find the end position that we want the robot arm to move to
        if self.images is not None and self.scans is not None:
            # setting the angular z value parameters to explore the environment in the angular direction
            self.movement.angular.z = 0.2
            image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='bgr8') # loading the color image
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            #TODO: change this so that it's referring to objects later
            color = 'pink'
            #setting up the hsv ranges for the different colored objects 
            if color == 'blue':
                lower_bound = numpy.array([95, 90, 100]) 
                upper_bound = numpy.array([105, 110, 150]) 
            elif color == 'pink':
                lower_bound= numpy.array([155, 140, 120]) 
                upper_bound= numpy.array([165, 160, 170]) 
            elif color == 'green':
                lower_bound = numpy.array([30, 130, 90]) 
                upper_bound = numpy.array([40, 150, 150]) 
            
            # erases all the pixels in the image that aren't in that range
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            
            # determines the center of the colored pixels
            M = cv2.moments(mask)

            #dimensions of the image, used later for proportional control 
            h, w, d = image.shape


            # if it detected the color
            if M['m00'] > 0:
                # center of the colored pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # a red circle is visualized in the debugging window to indicate
                # the center point of the colored pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                # proportional control to orient towards the colored object
            cv2.imshow("window", image)
            cv2.waitKey(3)

    def button_callback(self, data):

        if (data.illumination == 2.0):
            self.is_metal = 0
        elif (data.illumination == 1.0):
            self.is_metal = 1
            print("button pressed!")
        else:
            print("Sensor ERROR!")


    def get_current_location(self, angles):
        # height of turtlebot
        turtlebot_height = .141 
        # block difference angle
        block = math.atan(.024/.130)

        # lengths of the robot arm
        l_1 = .042 # distance from first joint to second joint (m)
        l_2 = .130 # distance from second joint to third joint (m)
        l_3 = .124 # distance from third joint to fourth joint (m)
        l_4 = .126 # distance from fourth joint to end effector (m)
        #l_1 = 0.077
        #l_2 = 0.128
        #l_3 = 0.024 + 0.124
        #l_4 = 0.126
        #j_1 = -1 * angles[0]
        #j_2 = -1 * (angles[1] - block - math.radians(90))
        #j_3 = -1 * (angles[2] - j_2)
        #j_4 = -1 * (angles[3])
        # joint angles to input into forward kinematics equations
        j_1 = -1*(angles[0])
        j_2 = math.radians(90) - (angles[1] + block)
        j_3 = -1*(angles[2] + math.radians(90))
        j_4 = -1*(angles[3])

        print('the adjusted angles', [j_1, j_2, j_3, j_4])
        #curr_x = (l_2 * math.cos(j_1) * math.cos(j_2)) + (l_3 * math.cos(j_1) * math.cos(j_2 + j_3))
        #curr_y = (l_2 * math.sin(j_1) * math.cos(j_2)) + (l_3 * math.sin(j_1) * math.cos(j_2 + j_3))
        #curr_z = l_1 + (l_2*math.sin(j_2)) + (l_3*math.sin(j_2 + j_3))
        curr_x = math.cos(j_1)*(l_2*math.cos(j_2)+l_3*math.cos(j_2 + j_3)) + l_4*math.cos(j_1)*math.cos(j_2+j_3+j_4)
        curr_y = math.sin(j_1)*(l_2*math.cos(j_2)+l_3*math.cos(j_2 + j_3)) + l_4*math.sin(j_1)*math.cos(j_2+j_3+j_4)
        curr_z = l_1 + l_2*math.sin(j_2) + l_3*math.sin(j_2+j_3) + l_4*math.sin(j_2+j_3+j_4)
        #print('first term', l_1)
        #print('second_term', l_2*math.sin(j_2))
        #print('third_term', l_3*math.sin(j_2+j_3))
        #print('fourth_term', l_4*math.sin(j_2+j_3+j_4))
        # need to adjust for the height of the turtlebot; .03 is the measurement of the block between the first joint and the turtlebot height.
        #print('curr_z before adding tb_height', curr_z)
        curr_z += 0.035 + turtlebot_height # to offset for the problematic angle
        #print('curr_z after adding tb height', curr_z)

        #print('first term', math.cos(j_1)*(l_2*math.cos(j_2)))
        #print('second_term', math.cos(j_1)*(l_3*math.cos(j_2+j_3)))
        #print('third_term', math.cos(j_1)*(l_4*math.cos(j_2+j_3+j_4)))
        #print('offset', curr_x - .195)
        curr_x -= 0.075 # to offset for the fact that x = 0 is around the center of the lidar scanner
        #print('curr_x after accounting for tbot front edge', curr_x)
        
        # the curr_y is flipped in the robot arm coordinate system, 
        # need to account for that
        curr_y = curr_y *-1
        current_location = [curr_x, curr_y, curr_z]
        return current_location

    def get_joint_dist(self, angles, goal_location):
        current_location = self.get_current_location(angles)
        curr_location_array = np.asarray(current_location)

        goal_location_array = np.asarray(goal_location)
        
        distance = np.linalg.norm(curr_location_array - goal_location_array) 
        return distance
    
    def clamp_angles(self, angle_idx):
        if self.current_joint_angles[angle_idx] < self.angle_min[angle_idx]:
            self.current_joint_angles[angle_idx] = self.angle_min[angle_idx]
        elif self.current_joint_angles[angle_idx] > self.angle_max[angle_idx]:
            self.current_joint_angles[angle_idx] = self.angle_max[angle_idx]


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
        
        # changing back the angle 
        self.current_joint_angles[angle_idx] = old_joint_angle
        return gradient
 
    def run(self):
        # Uses gradient descent to compute how much to move arms.
        # reference: https://www.alanzucconi.com/2017/04/10/robotic-arms/
        rospy.sleep(3)
        # TODO: choose good learning rate
        tau = 0.05 # learning rate
        # TODO: choose good distance threshold
        distance_threshold = 0.075
        reached_goal = False
        
        while not reached_goal:
            print('first one')
            print("current location:", self.get_current_location(self.current_joint_angles))
            if self.get_joint_dist(self.current_joint_angles, self.goal_location) <= distance_threshold:
                print('reached goal')
                reached_goal = True
            else:
                
                for i in (range(len(self.current_joint_angles))): # make sure not to update last joint
                    gradient = self.gradient_descent(i)
                    self.current_joint_angles[i] -= tau * gradient
                    # clamping to make sure that angle values do not go out of bounds
                    self.clamp_angles(i)
                # self.move_group_arm.go(self.current_joint_angles, wait=True)
                # rospy.sleep(1)
                print('update after gradient descent')
                print(self.get_current_location(self.current_joint_angles))
                if self.get_joint_dist(self.current_joint_angles, self.goal_location) < distance_threshold:
                    print('also reached goal')
                    # move the arm to match the goal
                    self.move_group_arm.go(self.current_joint_angles, wait=True)
                    reached_goal = True
                    rospy.sleep(5)
                # return
            rospy.sleep(0.01)
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
