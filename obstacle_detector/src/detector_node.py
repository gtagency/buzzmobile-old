#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

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
        groups = df.group_points(points)
        #TODO Find get distance from laser to groups.
        ## Faster, but still fairly slow even with c wrapper.
        ## Possible solution is to only use every nth theta. Seems
        ## to work quite well from testing will implement.
        

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
	    if self.stop_flag is not None:
            	self.obstacles_pub.publish(self.stop_flag)
            r.sleep()
    

if __name__ == "__main__":
    detector = Detector()
    detector.run()
