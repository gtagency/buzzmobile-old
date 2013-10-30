#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud

import detector_functions as df


class Detector(object):
    def __init__(self):
        self.stop_flag = False
        rospy.init_node("detector")
        self.obstacles_pub = rospy.Publisher("obstacle_stop", Bool)
        rospy.Subscriber("scan", LaserScan, self.update_obstacles)

    def update_obstacles(self, scan):        
	points = df.clean_scan(scan.angle_min, scan.angle_increment,
				  scan.range_min, scan.range_max, scan.ranges)
	points = df.polar2cart(points)

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
	    if self.stop_flag is not None:
            	self.obstacles_pub.publish(self.stop_flag)
            r.sleep()
    

if __name__ == "__main__":
    detector = Detector()
    detector.run()
