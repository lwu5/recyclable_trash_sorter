#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
import math

from __future__ import annotations
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

        # finding object (0) -> orient facing object (1) -> go towards object (2)
        self.robot_state = 0

        # state variable to check if it has rotated and adjusted itself towards the object
        self.rotated = False

        # lower and upper bounds for blue, pink, and green
        self.color_dict = {
            0: (np.array([95, 90, 100], np.array([105, 110, 150]))),
            1: (np.array([155, 140, 120]), np.array([165, 160, 170])),
            2: (np.array([30, 130, 90]), np.array([40, 150, 150]))
        }

        self.cx = 0
        self.cy = 0
        self.w = 0

        print('finished initializing!')

    def image_callback(self, img: Image) -> None:
        # loads the image and checks for color in image
        image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='bgr8') # loading the color image
        assert image is not None
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #dimensions of the image, used later for proportional control 
        h, self.w, d = image.shape

        if self.robot_state == 0:
            for color in self.color_dict:
                self.find_color(hsv, image, color)


    def scan_callback(self, data):
        self.scan = data
        self.front_distance = np.mean([data.ranges[i] for i in [0, 1, 2, 359, 358]])
        if self.robot_state == 1:
            self.rotate_towards_object()
        elif self.robot_state == 2:
            self.move_to_object()

    def find_color(self, hsv: np.ndarray, img: np.ndarray, color: int) -> None:
        # erases all the pixels in the image that aren't in that range
        lower_bound, upper_bound = self.color_dict[color]
        mask = cv2.inRange(hsv, lower_bound , upper_bound)

        # determines the center of the colored pixels
        M = cv2.connectedComponentsWithStats(mask)

        # if it detected the color
        if M['m00'] > 0:

            self.detected_color = True
            # center of the colored pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.cx = cx
            self.cy = cy
            # a red circle is visualized in the debugging window to indicate
            # the center point of the colored pixels
            cv2.circle(img, (cx, cy), 20, (0,0,255), -1)

        else:
            self.detected_color = False
        
        cv2.imshow("window", img)
        cv2.waitKey(3)

    def update_state(self):
        if self.robot_state == 0 and self.detected_color:
            self.robot_state = 1
        elif self.robot_state == 1 and self.rotated:
            self.robot_state = 2
        elif self.robot_state == 2:
            self.robot_state = 0
            self.detected_color = False
            self.rotated = False

    def rotate_towards_object(self):
        #TODO: ROTATE OBJECT AND MAKE SURE ITS FACING OBJECT
        # proportional control to orient towards the colored object
        angular_error = ((self.w/2) - (self.cx))
        angular_k = 0.001
        while angular_error != 0:
            lin = Vector3(0, 0, 0)
            ang = Vector3(0, 0, angular_k * angular_error)
            twist = Twist(linear=lin, angular=ang)
            self.vel_pub.publish(twist)
        return

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
        while True:
            self.update_state()

if __name__ == "__main__":
    node = Robot_Mover()
    sleep(3)
    node.run()