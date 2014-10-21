#!/usr/bin/env python

import rospy
import planner_astar
import math
from geometry_msgs.msg import Pose2D
from driver_differential.msg import Path
from lane_classifier.msg import LaneLabels
from std_msgs.msg import Bool

class PlannerNode:
    def __init__(pixelToMeter):
        self.pixelToMeter = pixelToMeter
        self.carWidth = carWidth
        self.path_pub = rospy.Publisher("planned_path", Path)
        rospy.Subscriber("car_position", Pose2D, self.updatePosition)
        rospy.Subscriber("image_driveable", LaneLabels, self.updatePlan)
        rospy.Subscriber("running_flag", Bool, self.setDriveFlag)
        self.posX = 0
        self.posY = 0
        self.posTheta = 0
        self.run_flag = False

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            r.sleep()

    def setDriveFlag(self, flag):
        self.run_flag = flag.data
        if not flag.data:
            msg = Path()
            msg.poses = []
            path_pub.publish(msg)

    def updatePlan(self, img):
        if self.run_flag:
            img = convertImageToArray(img)
            path = planner_astar.planPath(img, self.carWidth/self.pixelToMeter)
            path = convertPathToWorldFrame(path)
            msg = Path()
            msg.poses = path
            self.path_pub.publish(msg)

    def updatePosition(self, pose):
        self.posX = pose.x
        self.posY = pose.y
        self.posTheta = pose.theta

    def convertImageToArray(self, img):
        height = img.height
        width = img.width
        arr = [img.labels[i*width:i*width+width] for i in range(height)]

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