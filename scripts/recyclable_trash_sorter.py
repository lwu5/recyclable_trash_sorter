#!/usr/bin/env python3

from __future__ import annotations
import rospy, cv2, cv_bridge
import numpy as np
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from time import sleep
from turtlebot3_msgs.msg import SensorState

# dictionary that contains the AR tags info
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


class RecyclableTrashSorter:
    def __init__(self):

        rospy.init_node("recycable_trash_sorter")

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initializing movement publishing
        self.movement = Twist(linear=Vector3(), angular=Vector3())
        # for proportional control, the minimum distance away objects should be
        self.min_dist_away = .5

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # initialize the goal location
        self.goal_location = [0.3, 0, 0.2]
        
        # Reset arm position
        angles = [0, -0.5, 0, 0]
        self.move_group_arm.go(angles, wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)
        
        # initializing an attribute that will hold the angles
        self.current_joint_angles = angles

        # set angle minimums and maximums to make sure that algorithm does not
        # finalize on angles that are outside the range
        self.angle_min = [math.radians(-162), math.radians(-103), math.radians(-53), math.radians(-100)]
        self.angle_max = [math.radians(162), math.radians(90), math.radians(79), math.radians(117)]
        
        # initializing attributes that will hold images and scan data
        self.images = None
        self.scans = None

        # indication to start moving towards an object or AR tag
        self.start_moving_forward = 0

        # indication of whether the object that is gripped is metal or not 
        self.is_metal = 0 # 1 = metal

        # keeps track of how far the object in the front is
        self.front_distance = 1.0

        # state variable to check if it has rotated and adjusted itself towards the object
        self.rotated = False
        
        # finding object (0) -> orient facing object (1) -> go towards object (2)
        self.robot_state = 0


        # set lower and upper hsv bounds for 0 - blue, 1 - pink, and 2 - green
        self.color_dict = {
            0: (np.array([95, 90, 100]), np.array([105, 110, 150])),
            1: (np.array([155, 140, 120]), np.array([165, 160, 170])),
            2: (np.array([30, 130, 90]), np.array([40, 150, 150]))
        }
        
        # keep track of what color is found
        self.detected_color = False
        
        # initialize to keep track of which color was found, represents 
        # the key of the corresponding color in the color dictionary
        # valid color would only be 0, 1, 2;
        self.which_color = -1

        # initializing attributes for holding the center of the colored pixels 
        # or the width of image
        self.cx = 0
        self.cy = 0
        self.w = 0
        
        # what the image width is
        self.img_width = 100

        # initialize list that keeps track of which colors have been already placed
        self.finish_color = []
 
        # setting up subscriber for camera image
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        # setting up subscriber for lidar scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # setting up subscriber for the button pressed or not
        self.button_sub = rospy.Subscriber('sensor_state', SensorState, self.button_callback)
        # setting up publisher for linear or angular movement
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # check if the robot has been facing the object in a preset distance
        self.reached_object = False

        # all the objects have been sorted
        self.all_sorted = False

        print('finished initializing!')

    def image_callback(self, img):
        # Callback function for obtaining image data
        #print('entered image callback')
        self.images = img


    def scan_callback(self, data):
        # Callback function for obtaining lidar scan ranges
        self.scans = data.ranges
        self.front_distance = np.mean([data.ranges[i] for i in [0, 1, 2, 359, 358]])

    
    def find_color(self):
        # A method for the robot to find a colored object (metal or non-metal). 
        # It explores the environment to locate a colored object, and moves the robot towards the object location using
        # camera and LiDAR scan data to position it such that it is ready to pick up the object.
        
        #if the camera images are loaded from the callback function image_callback 
        if self.images is not None and self.front_distance is not None:
            # setting the angular z value parameters to explore the environment in the angular direction
            self.movement.angular.z = 0.2
            # loads the image and checks for color in image
            image = self.bridge.imgmsg_to_cv2(self.images, desired_encoding='bgr8') # loading the color image
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert to hsv

            #dimensions of the image, used later for proportional control 
            h, self.w, d = image.shape

            # check if a color has been detected, then start processes to move towards it
            if self.detected_color:
                # erases all the pixels in the image that aren't in that range
                lower_bound, upper_bound = self.color_dict[self.which_color]
                mask = cv2.inRange(hsv, lower_bound , upper_bound)

                # determines the center of the colored pixels
                M = cv2.moments(mask)

                # in the case that it lost the color (mask is empty), we want to 
                # reset and start exploring to look for a color again
                if M['m00'] <= 0:
                    self.detected_color = False
                    self.start_moving_forward = False
                    self.movement.linear.x = 0
                    self.movement.angular.z = 0.2
                else: # looking for the next color in camera view
                    # center of the colored pixels in the image
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.cx = cx
                    self.cy = cy
                    # a red circle is visualized in the debugging window to indicate
                    # the center point of the colored pixels
                    cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            
                    # proportional control to rotate towards the object
                    angular_error = ((self.w/2) - (self.cx))
                    angular_k = 0.001
                    angular_tol = 4.0
                    self.movement.angular.z = angular_error * angular_k
                    
                    # if the front of the robot is facing the object within a certain angular tolerance, 
                    # then it is sufficiently centered in the robot's camera view and 
                    # should start moving forward towards the object 
                    if abs(angular_error) <= angular_tol:
                        self.start_moving_forward = True
                
                    # if the bot should move forward, should use LiDAR scan data to figure out when to 
                    # stop moving if sufficiently close
                    if self.start_moving_forward:
                        # proportional control to get close to object within a set distance
                        linear_tol = 0.15
                        self.movement.linear.x = min((self.front_distance - 0.15) * 0.4, 0.5)
                        self.movement.angular.z = 0
                        
                        # sufficiently close, stop all motion, delete the color which was detected
                        # from the color dictionary, and set state variables to initiate next part 
                        # of robot movement
                        print("front dist", self.front_distance)
                        if (self.front_distance - 0.2) < linear_tol:
                            self.movement.linear.x = 0
                            self.movement.angular.z = 0
                            self.robot_state = 1 # orient facing object (1)
                            self.start_moving_forward = 0 # not moving forward
                            self.detected_color = False # color isnt detected yet
                            self.color_dict.pop(self.which_color) # delete the selected color from dictionary
                                                                  # of unsorted objects
                            if (self.color_dict is np.empty): # If all colors have been sorted
                                self.all_sorted = True
            else: # no color detected
                # go through all the colors in the dictionary to see if it is present in the camera view
                for color in self.color_dict:
                    # erases all the pixels in the image that aren't in that range
                    lower_bound, upper_bound = self.color_dict[color]
                    mask = cv2.inRange(hsv, lower_bound , upper_bound)
                    # determines the center of the colored pixels
                    M = cv2.moments(mask)
                    # if it detected the color, stop searching for other colors
                    # and track which color was found
                    if M['m00'] > 0:
                        self.detected_color = True
                        self.which_color = color
                        break
            # visualizing the image
            cv2.imshow("window", image)
            cv2.waitKey(3)
            
            # publishing the angular or linear movement
            self.vel_pub.publish(self.movement)


    def update_state(self):
        # checks which state the robot is in and  runs 
        # functions to find objects, pick things up, etc. based on that state
       
        if self.robot_state == 0: # needs to explore and find a new object 
            self.find_color()          
        elif self.robot_state == 1: # has oriented itself in front of an object, need to use arm to pickup
            self.pick_up_object()
        elif self.robot_state == 2: # has picked up the object, need to place in front of tag
            self.find_tag()
        elif self.robot_state == 3: # has found the tag and gone close to it, need to use arm to drop
            self.drop_object()
        elif self.robot_state == 4: # has completed an entire search, reset to look for a new object again
            self.robot_state = 0

    def find_tag(self):
        # Now that the robot has picked up the object, this is the method to find the correct AR tag 
        # using the aruco AR tag library. Based on the material identity fo the object (metal or non-metal)
        # it looks for a specific AR tag, and recognizes that tag with camera data and then moves 
        # towards it sufficently close with LiDAR data
        
        # put all the nonmetal objects at AR tag 1, metal objects at AR tag 3 
        if self.is_metal == 0:
            tag = 1
        else:
            tag = 3

        # explore the environment by turning around to find tag
        self.movement.angular.z = 0.1
                
        # check to see that images have been collected from the image_callback
        if self.images is not None:
            # AR tag recognition requires greyscale images
            grayscale_image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='mono8')
            
            # corners is a 4D array of shape (n, 4, 2), where n is the number of tags detected
            # each entry is a set of four (x, y) pixel coordinates corresponding to the
            # location of a tag's corners in the image
            # ids is a 2D array of shape (n, 1)
            # each entry is the id of a detected tag in the same order as in corners
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, aruco_dict)
            
            #if it finds ids and corners, matching the tag from the best policy to the tag it finds 
            if ids is not None and len(corners) != 0:
                ids = ids[0]
                corners = corners[0]
                tag_idx = np.argwhere(ids == tag)
                # moving toward the correct tag if it was detected
                if tag_idx.size > 0:
                    tag_idx = tag_idx[0]
                    # extracting the x, y coordinates of the tag's corner locations in the camera image
                    left_upper_corner = corners[tag_idx,0,:]
                    right_upper_corner = corners[tag_idx,1,:]
                    right_bottom_corner = corners[tag_idx,2,:]
                    left_bottom_corner = corners[tag_idx,3,:]
                    
                    # width and height of the AR tag
                    width = right_upper_corner[0,0] - left_upper_corner[0,0]
                    height = left_upper_corner[0,1] - left_bottom_corner[0,1]
                    
                    # extract the center coordinates of the tag
                    cx = int(left_upper_corner[0,0] + width/2)
                    cy = int(left_upper_corner[0,1] + height/2)
                    cv2.circle(grayscale_image, (cx,cy),20,(0,0,255),-1)
                    # shape of the entire grayscale image
                    h, w  = grayscale_image.shape
                    
                    # proportional control to orient towards the tag
                    angular_error = ((w/2) - (cx))
                    #angular k values found through testing 
                    angular_k = 0.001
                    self.movement.angular.z = angular_k * angular_error
                    
                    #if the front of the robot is facing the tag within a certain angular tolerance, 
                    # then it is sufficiently centered in the robot's camera view and 
                    # should start moving forward towards the tag 
                    tol = 30
                    if abs(angular_error) < tol:
                        self.start_moving_forward = 1

                    # if the bot should move forward, should use LiDAR scan data to figure out when to 
                    # stop moving if sufficiently close
                    if self.start_moving_forward:
                        # extracting distances of the objects or tags located 
                        # -10 to 10 degrees in front of the bot
                        print('should start moving forward')
                        if self.scans is not None: # if lidar is working
                            ranges = np.asarray(self.scans)
                            ranges[ranges == 0.0] = np.nan
                            slice_size = int(20)
                            first_half_angles = slice(0,int(slice_size/2))
                            second_half_angles = slice(int(-slice_size/2),0)
                    
                            # this is the mean distance of likely the tag that has been detected from the robot
                            slice_mean = np.nanmean(np.append(ranges[first_half_angles],ranges[second_half_angles]))
                            if np.isnan(slice_mean): # if all LiDAR measurements invalid (0.0) then the 
                                                     # mean is NaN, stop moving forward
                                self.movement.linear.x = 0
                            else:
                                linear_error = slice_mean - self.min_dist_away 
                                
                                # setting a constant linear velocity to move towards the tag, until it reaches within a certain tolerance 
                                self.movement.linear.x = 0.04
                                linear_tol = 0.005
                                # if tag found and bot is sufficiently close to it, change state variables to indicate that this has
                                # occurred and stop motion to initiate next step of robot movement (dropping)
                                if linear_error < linear_tol:
                                    self.start_moving_forward = 0
                                    self.movement.linear.x = 0
                                    self.movement.angular.z = 0
                                    self.robot_state = 3
                    else: # robot is close to the target
                        self.movement.linear.x = 0.0
            # to visualize the robot localizing to tags         
            cv2.imshow("window", grayscale_image)
            cv2.waitKey(3)

        # publish the movement
        self.vel_pub.publish(self.movement) 


    def button_callback(self, data):
        # a callback function that gets data in an 'illumination' attribute that 
        # actually represents the button state. it is 1.0 when it is pressed, and 2.0 when
        # it is not pressed, so the former represents when the gripper is holding a harder metal object
        # and the latter represents when it is holding a softer non-metal object
        if (data.illumination == 2.0): # button clicked
            self.is_metal = 0
        elif (data.illumination == 1.0): # button unclicked
            self.is_metal = 1
            print("button pressed!")
        else: # error - default value is 0.0
            print("Sensor ERROR!")


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
        curr_z += 0.035 + turtlebot_height # to offset for the problematic angle

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
        
        # distance from the robot gripper to the goal location
        distance = np.linalg.norm(curr_location_array - goal_location_array) 
        return distance
    
    def clamp_angles(self, angles, angle_idx):
        '''
        Clamps the joint angles so that they never exceed the physical limits of the
        arm's joint motion when solving for an IK solution during gradient descent
        '''

        if angles[angle_idx] < self.angle_min[angle_idx]: # if calculation exceeds the min joint angle
            angles[angle_idx] = self.angle_min[angle_idx]
        elif angles[angle_idx] > self.angle_max[angle_idx]: # if calculation exceeds the max joint angle
            angles[angle_idx] = self.angle_max[angle_idx]

    def partial_gradient(self, angle_idx):
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
        old_distance = self.get_joint_dist(self.current_joint_angles, self.goal_location)

        # storing old joint angle
        old_joint_angle = self.current_joint_angles[angle_idx]
        
        # add delta to angle and then compute new post-delta xyz to goal xyz distance
        self.current_joint_angles[angle_idx] += delta
        new_distance = self.get_joint_dist(self.current_joint_angles, self.goal_location)

        # difference between distances / delta is the derivative, aka the gradient
        gradient = (new_distance - old_distance) / delta
        
        # changing back the angle 
        self.current_joint_angles[angle_idx] = old_joint_angle
        return gradient

    def do_gradient_descent(self, angles, goal_location):
        '''
        Uses gradient descent to compute how much to move arms. 
            
        reference: https://www.alanzucconi.com/2017/04/10/robotic-arms/
        '''
        # learning rate tau controls how much the angle changes after each iteration of gradient descent
        # since it is multiplied to the gradient
        tau = 0.05 
        # threshold (m) at which we will consider the algorithm to have converged;
        # current xyz position is close enough to goal xyz position
        distance_threshold = 0.07

        # check if we're already close enough to the goal location such that algorithm doesn't need to be run
        if self.get_joint_dist(angles, goal_location) <= distance_threshold: # close enough to the goal location
                print('reached goal')
        else: # not close enough yet
            while self.get_joint_dist(angles, goal_location) > distance_threshold:
                # iterate over all 4 joints
                for i in (range(len(angles))): 
                    # determine gradient for that joint
                    gradient = self.partial_gradient(i)
                    # travel down the gradient and go towards a minimum (b/w current xyz and goal xyz)
                    # by adjusting the angle based on the calculated gradient
                    angles[i] -= tau * gradient
                    # clamping to make sure that angle values do not go out of bounds
                    self.clamp_angles(angles, i)
                if self.get_joint_dist(angles, goal_location) < distance_threshold: # check if converged, close enough to goal
                    print('converged')
    
    def pick_up_object(self):
        '''
        pick up the color
        '''

        # run gradient descent after finding the right xyz to pick up the object
        self.do_gradient_descent(self.current_joint_angles, self.goal_location) # also takes into account picking up object
        
        # after gradient descent has converged, move the angles to the location for pickup
        self.move_group_arm.go(self.current_joint_angles, wait=True)
        rospy.sleep(5)

        # parameters for the gripper joint (to grasp) found through trial and error testing 
        gripper_joint_goal = [-0.006, -0.006]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(1)

        # move the arm back up 
        # parameters for the arm joint (to lift up the object) found through trial and error testing 
        arm_joint_goal = [0.0, -.7, .1, -0.65]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        # TODO: future work, potentially could use
        #self.do_gradient_descent(self.current_joint_angles, [0.1,0,0.4]) # also takes into account picking up object, specify an xyz

        self.object_picked_up = 1
        self.robot_state = 2


    def drop_object(self):
        # once the robot has found the correct tag, method to drop the object 

        # TODO: potentially could use
        # goal_location will have been modified by find_tag 
        #self.do_gradient_descent(self.current_joint_angles, self.goal_location) # also takes into account picking up object, specify an xyz
    
        # arm joint parameters found through testing; brings the arm downwards to its initial position
        arm_joint_goal = [0.0, 0.4, 0.1, -0.65]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        # sleep for some time to make sure there is enough time for the arm to lower
        rospy.sleep(5)
        # then open the gripper to release the object
        gripper_joint_goal = [0.019,0.019]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        
        # moving the robot back once it has dropped the object, 
        # so that there is space for it to continue spinning and find the next object 
        self.movement.linear.x = -0.1
        self.vel_pub.publish(self.movement)
        rospy.sleep(1)
        self.movement.linear.x = 0.0
        self.vel_pub.publish(self.movement)

        # move arm back up to get ready to explore and pickup new object
        initial_angles = [0, -0.5, 0, 0]
        self.move_group_arm.go(initial_angles, wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)

        rospy.sleep(3)
        #updating control variables 
        self.object_dropped = 1
        self.robot_state = 4

    def run(self):
        # keeps looking for objects to sort and performing the sorting function
        # until all the objects have been sorted
        while not self.all_sorted:
            self.update_state()
            
if __name__ == "__main__":
    node = RecyclableTrashSorter()
    rospy.sleep(3)
    node.run() # keep the program running
