#!/usr/bin/env python
from math import pi

import roslib
import rospy
import obstacle_detector
import cv2
import numpy as np
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud
from obstacle_detector.msg import Obstacle

import detector_functions as df

df.GROUPING_DISTANCE = .1
df.MIN_GROUP_SIZE = 5
#Do NOT use values with magnitude greater than pi/2.
df.START_ANGLE = -pi/4
df.END_ANGLE = pi/4
RANGE_STEP = 10
# Obstacle distance threshold.
MIN_DISTANCE = 0.8


class Detector(object):
    def __init__(self):
        self.stop_flag = False
        rospy.init_node("detector")
        self.obstacles_pub = rospy.Publisher("Obstacle", Obstacle)
        rospy.Subscriber("scan", LaserScan, self.update_obstacles)
        self.hulls = []
    def update_obstacles(self, scan):        
        points = df.clean_scan(scan.angle_min, scan.angle_max,
                               scan.angle_increment,
                               scan.range_min, scan.range_max,
                               scan.ranges, RANGE_STEP)
        points = df.polar2cart(points)
        groups = df.group_points(points)
        self.hulls = [df.graham_scan(g) for g in groups]
        image = np.zeros((800,800), np.dtype_uint8 )
        for h in hulls:
            for p in h:
                image[p[1]][p[0]] = 255
        cv2.imshow("hulls", image)
        closest = df.closest_point([p for hull in hulls for p in hull])
        self.stop_flag = closest < MIN_DISTANCE

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            self.obstacles_pub.publish(self.hulls)
            r.sleep()
    

if __name__ == "__main__":
    detector = Detector()
    detector.run()
