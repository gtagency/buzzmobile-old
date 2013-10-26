#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import LaserScan


class Detector(object):
    def __init__(self):
        self.obstacles = []
        rospy.init_node("detector")
        obstacles_pub = rospy.Publisher("obstacles", ObstacleArrayStamped)
        rospy.Subsciber("scan", LaserScan, self.update_obstacles)

    def update_obstacles(self, scan):        
        pass

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            self.pub.publish(self.obstacles)
            r.sleep()
    

if __name__ == "__main__":
    detector = Detector()
    detector.run()