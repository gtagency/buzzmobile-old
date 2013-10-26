#!/usr/bin/env python
import roslib
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud

import detector_functions as df


class Detector(object):
    def __init__(self):
        self.obstacles = []
	self.point_cloud = None
        rospy.init_node("detector")
        self.obstacles_pub = rospy.Publisher("obstacles", PointCloud)
        rospy.Subscriber("scan", LaserScan, self.update_obstacles)

    def update_obstacles(self, scan):        
	points = df.clean_scan(scan.angle_min, scan.angle_increment,
				  scan.range_min, scan.range_max, scan.ranges)
	points = df.polar2cart(points)
	points = [Point32(p[0],p[1],0) for p in points]

	header = std_msgs.msg.Header()
	header.frame_id = "/laser"
	header.stamp = scan.header.stamp
	self.point_cloud = PointCloud(header, points, [])
	

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
	    if self.point_cloud is not None:
            	self.obstacles_pub.publish(self.point_cloud)
            r.sleep()
    

if __name__ == "__main__":
    detector = Detector()
    detector.run()
