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

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        self.images = None
        self.scans = None
        
        self.current_color = None
        self.colored_centers = [(-1, -1) for _ in range(3)]
        
        print('finished initializing!')

    def image_callback(self, msg):
        print("got image")
        if not msg.data:
            # no image
            return

        self.img_width = msg.width

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        cv2.imshow("window", image)
        cv2.waitKey(3)

        self.set_colored_object_centers(image)
        
    def scan_callback(self, data):
        self.front_distances = [data.ranges[0]] + self.front_distances[1:]
        self.front_distance = np.mean(self.front_distances)

    
    def find_goal_location(self, image):
        color_ranges = [
            # pink
            (numpy.array([0, 0, 200]), numpy.array([60, 60, 255])),
            # green
            (numpy.array([0, 200, 0]), numpy.array([60, 255, 60])),
            # blue
            (numpy.array([200, 0, 0]), numpy.array([255, 60, 60])),
        ]

        centers = []

        for lower, upper in color_ranges:
            mask = cv2.inRange(image, lower, upper)
            center = cv2.moments(mask)

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
        color_id = 0
        center = self.colored_centers[color_id]
        # go in front of object
        r = rospy.Rate(10)
        while not (abs(center[1] - self.img_width / 2) < 10 and abs(self.front_distance - 0.2) < 0.02):
            center = self.colored_centers[color_id]
            # object is not detected, find another color
            while center == (-1, -1):
                if color_id != 3:
                    color_id += 1
                    center = self.colored_centers[color_id]
                else:
                    lin = Vector3(0.0, 0.0, 0.0)
                    ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[1]) * 0.5)
                    color_id = 0
            if center != (-1, -1):
                self.current_color = color_id
                lin = Vector3(0.0, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[1]) * 0.5)
            twist = Twist(linear=lin, angular=ang)
            self.vel_pub.publish(twist)
            r.sleep()

 
    def run(self):
        rospy.sleep(3)

if __name__ == "__main__":
    node = Robot_Mover()
    sleep(0.1)
    node.find_color()
    node.run()