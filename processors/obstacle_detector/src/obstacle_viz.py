#!/usr/bin/env python
from math import pi

import roslib; roslib.load_manifest('obstacle_detector')
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Point, PoseArray, Pose
from core_msgs.msg import ObstacleArrayStamped, Obstacle

class ObstacleViz(object):

    def __init__(self):
        self.stop_flag = False
        rospy.init_node("obstacle_viz")
        self.obstacle_poses_pub = rospy.Publisher("obstacle_poses", PoseArray)
        rospy.Subscriber("obstacles", ObstacleArrayStamped, self.update_obstacle_viz)

    def update_obstacle_viz(self, inmsg):
        msg = PoseArray()
        msg.header = Header()
        # since the obstacles are derived from the laser, specify that frame id
        msg.header.frame_id = "laser"
        msg.poses = [Pose(o.center, None) for o in inmsg.obstacles]
        self.obstacle_poses_pub.publish(msg)

    def run(self):
        rospy.spin();

if __name__ == "__main__":
    ObstacleViz().run()
