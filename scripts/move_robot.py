#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
import math

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3

from time import sleep

class Robot_Mover:
    def __init__(self):

        rospy.init_node("Robot_Mover")

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # putting images in gobal variable
        self.images = None

        # putting scans in a global variable
        self.scans = None

        # keep track of what color is found
        self.detected_color = False

        # keeps track of how far the object in the front is
        self.front_distance = 1.0

        # what the image width is
        self.img_width = 100

        print('finished initializing!')

    def image_callback(self, img):
        if img is None:
            print("there is no image")
        self.images = img

    def scan_callback(self, data):
        self.scan = data
        self.front_distance = np.mean([data.ranges[i] for i in [0, 1, 2, 359, 358]])

    def find_color(self, color):
        #if the camera images are loaded from the callback function image_callback 
        if self.images is not None:
            image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='bgr8') # loading the color image
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            #setting up the hsv ranges for the different colored objects 
            if color == 0: # for sake of code simplicity, 0 == blue
                lower_bound = np.array([95, 90, 100]) 
                upper_bound = np.array([105, 110, 150]) 
            elif color == 1: # for sake of code simplicity, 1 == pink
                lower_bound= np.array([155, 140, 120]) 
                upper_bound= np.array([165, 160, 170]) 
            elif color == 2: # for sake of code simplicity, 2 == green
                lower_bound = np.array([30, 130, 90]) 
                upper_bound = np.array([40, 150, 150])
        else:
            print("no image")

        # erases all the pixels in the image that aren't in that range
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        # determines the center of the colored pixels
        M = cv2.moments(mask)

        #dimensions of the image, used later for proportional control 
        h, w, d = image.shape

        # if it detected the color
        if M['m00'] > 0:

            self.detected_color = True
            # center of the colored pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
                
            # a red circle is visualized in the debugging window to indicate
            # the center point of the colored pixels
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            
            # proportional control to orient towards the colored object
            angular_error = ((w/2) - (cx))
            angular_k = 0.001
            lin = Vector3(0.0, 0.0, 0.0)
            ang = Vector3(0, 0, angular_k * angular_error)
            self.vel_pub.publish(Twist(linear=lin, angular=ang))
        else:
            self.detected_color = False
        
        cv2.imshow("window", image)
        cv2.waitKey(3)
    
    def find_goal_location(self, image):
        #TODO: Find the goal location using the robot lidar data and image recognition
        curr_color = 0
        while not self.detected_color:
            if curr_color < 3:
                self.find_color(curr_color)
                curr_color += 1
            else:
                lin = Vector3(0.0, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, .1)
                curr_color = 0
                self.vel_pub.publish(Twist(linear=lin, angular=ang))

    def move_to_object(self):
        if self.scans is not None:
            # go in front of object
            print("Robot mover: Approaching object")
            r = rospy.Rate(10)
            while not abs(self.front_distance - 0.15) < 0.02:
                lin = Vector3(min((self.front_distance - 0.15) * 0.4, 0.5), 0.0, 0.0)
                ang = Vector3(0.0, 0.0, (self.img_width) * 0.01)
            twist = Twist(linear=lin, angular=ang)
            self.vel_pub.publish(twist)
            
        

    def run(self):
        self.find_goal_location(self.images)
        sleep(3)
        # moves to object after orientation is done
        self.move_to_object()
        rospy.spin()

if __name__ == "__main__":
    node = Robot_Mover()
    sleep(3)
    node.run()
