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

        print('finished initializing!')

    def image_callback(self, img):
        if img is None:
            print("there is no image")
        self.images = img

    def scan_callback(self, data):
        print("reaches scan")
        self.scan = data

    
    def find_goal_location(self, image):
        #TODO: Find the goal location using the robot lidar data and image recognition
        return

    def run(self):
        self.find_goal_location(self.images)
        rospy.spin()

if __name__ == "__main__":
    node = Robot_Mover()
    sleep(3)
    node.run()
