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

        self.bridge = cv_bridge.CvBridge()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        self.images = None
        self.img_width = 100
        self.front_distances = [1.0 for _ in range(5)]
        self.front_distance = 1.0
        self.current_color = None
        self.colored_centers = [(-1, -1) for _ in range(3)]

        self.is_run = True
        print('finished initializing!')

    def image_callback(self, msg):
        print("reached image callback")
        if not msg.data:
            print("no image")
            return
        print("there is an image")
        self.img_width = msg.width

        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([67, 100, 75]), np.array([110, 255, 255]))

        cv2.imshow("window", self.image)
        # cv2.imshow("window", mask)
        cv2.waitKey(3)

        self.find_goal_location(self.image)

        rospy.sleep(3)

    def scan_callback(self, data):
        print("reaches scan")
        self.front_distances = [data.ranges[0]] + self.front_distances[1:]
        self.front_distance = np.mean(self.front_distances)

    
    def find_goal_location(self, image):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_ranges = [
            # pink
            (np.array([0, 0, 200]), np.array([60, 60, 255])),
            # green
            (np.array([0, 200, 0]), np.array([60, 255, 60])),
            # blue
            (np.array([200, 0, 0]), np.array([255, 60, 60])),
        ]

        centers = []

        for lower, upper in color_ranges:
            mask = cv2.inRange(hsv, lower, upper)
            M = cv2.moments(mask)

            # this limits our search scope to only view a slice of the image near the ground
            h, w, d = image.shape
            search_top = int(h/2)
            search_bot = int(h)
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            if M['m00'] > 100:
                # center of the yellow pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # a red circle is visualized in the debugging window to indicate
                # the center point of the yellow pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                centers.append((cx, cy))
            else:
                centers.append((-1, -1))

        self.colored_centers = centers

    def find_color(self):
        print("reaches find_color function")
        color_id = 0
        center = self.colored_centers[color_id]
        # go in front of object
        r = rospy.Rate(10)
        if self.images is not None:
            print("reaches when self.images is not None")
            while not (abs(center[1] - self.img_width / 2) < 10 and abs(self.front_distance - 0.2) < 0.02):
                center = self.colored_centers[color_id]
                # object is not detected, find another color
                while center == (-1, -1):
                    if color_id != 3:
                        color_id += 1
                        print('is it here?')
                        center = self.colored_centers[color_id]
                    else:
                        lin = Vector3(0.0, 0.0, 0.0)
                        ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[1]) * 0.5)
                        color_id = 0
                if center != (-1, -1):
                    self.current_color = color_id
                    lin = Vector3(0.0, 0.0, 0.0)
                    ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[1]) * 0.5)
                    self.is_run = False
                twist = Twist(linear=lin, angular=ang)
                self.vel_pub.publish(twist)
                r.sleep()
        else:
            print("no images")

 
    def run(self):
        # self.is_run = True
        # while self.is_run:
        self.find_color()

        rospy.sleep(1)

        rospy.spin()

if __name__ == "__main__":
    node = Robot_Mover()
    sleep(0.1)
    node.run()
