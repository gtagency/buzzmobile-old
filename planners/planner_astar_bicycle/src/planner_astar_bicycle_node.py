#!/usr/bin/env python

import roslib; roslib.load_manifest("planner_astar_bicycle")
import rospy
import path_planner_astar_bicycle
import math
from geometry_msgs.msg import Pose2D
from planner_astar_bicycle.msg import Path
#from lane_classifier.msg import LaneLabels
from core_msgs.msg import WorldRegion

class PlannerNode:
    def __init__(self, pixelToFeet, carWidth):
        rospy.init_node("planner_astar_bicycle");
        self.pixelToFeet = pixelToFeet
        self.carWidth = carWidth
        self.path_pub = rospy.Publisher("planned_path", Path)
        rospy.Subscriber("car_position", Pose2D, self.updatePosition)
        #rospy.Subscriber("image_driveable", LaneLabels, self.updatePlan)
        rospy.Subscriber("world_model", WorldRegion, self.updatePlan)
        self.posX = 0
        self.posY = 0
        self.posTheta = 0

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            r.sleep()

    def updatePlan(self, world_model):
        wm_array = self.convertWorldModelToArray(world_model)
        path = path_planner_astar_bicycle.planPath(wm_array, self.carWidth * world_model.resolution, world_model.resolution)
        path = self.convertPathToWorldFrame(path)
        msg = Path()
        msg.poses = path
        self.path_pub.publish(msg)

    def updatePosition(self, pose):
        self.posX = pose.x
        self.posY = pose.y
        self.posTheta = pose.theta

    def convertWorldModelToArray(self, world_model):
        height = world_model.height
        width = world_model.width
        #NOTE: labels is a character array, with \x01 and \x00 values
        # OK for now, probably want to fix/change this later
        arr = [world_model.labels[i*width:i*width+width] for i in range(height)]
        return arr

    def convertPathToWorldFrame(self, path):
        newPath = []
        for pose in path:
            x, y, theta = pose

            newPose = Pose2D()
            t = self.posTheta
            newPose.x = math.cos(t) * x - math.sin(t) * y + self.posX
            newPose.y = math.sin(t) * x + math.cos(t) * y + self.posY
            newPose.theta = (t + theta) % (2 * math.pi)

            newPath.append(newPose)

        return newPath


if __name__ == "__main__":
    pixelToMeter = rospy.get_param('pixelToMeter', 1)
    carWidth = rospy.get_param('carWidth', 1)

    planner = PlannerNode(pixelToMeter, carWidth)
    planner.run()
