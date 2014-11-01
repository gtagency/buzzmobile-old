#!/usr/bin/env python
from math import pi

import roslib; roslib.load_manifest('obstacle_detector')
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan, PointCloud
from core_msgs.msg import ObstacleArrayStamped, Obstacle

import detector_functions as df

df.GROUPING_DISTANCE = .07
df.MIN_GROUP_SIZE = 2
#Do NOT use values with magnitude greater than pi/2.
df.START_ANGLE = -pi/2 + 3 * pi/180
df.END_ANGLE = pi/2 - 3 * pi/180
RANGE_STEP = 2 
# Obstacle distance threshold.
MIN_DISTANCE = 0.8


class Detector(object):

    def __init__(self):
        self.stop_flag = False
        rospy.init_node("detector")
        self.obstacles_pub = rospy.Publisher("obstacles", ObstacleArrayStamped)
        rospy.Subscriber("scan", LaserScan, self.update_obstacles)
        self.obstacles = [] #center, radius tuples
        self.ob_frame_id = ""

    def compute_center(self, pts): 
        avgx = sum(x for x,_ in pts)/len(pts)
        avgy = sum(y for _,y in pts)/len(pts)
        return (avgx, avgy) 

    def euclidean_dist(self, p1, p2):
        x1,y1 = p1
        x2,y2 = p2
        return ((x1-x2)**2 + (y1-y2)**2)**0.5

    def update_obstacles(self, scan):        
        points = df.clean_scan(scan.angle_min, scan.angle_max,
                               scan.angle_increment,
                               scan.range_min, scan.range_max,
                               scan.ranges, RANGE_STEP)
        points = df.polar2cart(points)
        groups = df.group_points(points)
        hulls = [df.graham_scan(g) for g in groups]
        centers = [self.compute_center(h) for h in hulls]
        radii   = [[self.euclidean_dist(c, pt) for pt in h] for c,h in zip(centers,hulls)]
        self.obstacles = [(c, max(r)) for c,r in zip(centers, radii)]
        self.ob_frame_id = scan.header.frame_id

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            msg = ObstacleArrayStamped()
            msg.header = Header()
            msg.header.frame_id = self.ob_frame_id
            msg.obstacles = [Obstacle(Point(c[0], c[1], 0),ra) for c,ra in self.obstacles]
            self.obstacles_pub.publish(msg)
            r.sleep()
    

if __name__ == "__main__":
    detector = Detector()
    detector.run()
