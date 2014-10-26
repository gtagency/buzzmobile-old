#!/usr/bin/env python

import rospy
import bicycle_driver
from geometry_msgs.msg import Pose2D
from planner_astar_bicycle.msg import Path
from std_msgs.msg import Bool, Header
from core_msgs.msg import MotionCommand

class BicycleDriveNode:
    def __init__(sideWheelSeparation, targetV, toleranceDist, toleranceTheta):
        self.driver = bicycle_driver.BicycleDriver(sideWheelSeparation, targetV,
                                    toleranceDist, toleranceTheta)
        self.traj_pub = rospy.Publisher("motion_command", MotionCommand)
        rospy.Subscriber("car_position", Pose2D, self.updatePosition)
        rospy.Subscriber("planned_path", Path, self.updatePlan)
        rospy.Subscriber("brake", Bool, self.setStopFlag)
        self.brake_flag = True

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            r.sleep()

    def setStopFlag(self, flag):
        self.brake_flag = flag.data
        if flag.data:
            header = Header()
            header.time = rospy.get_rostime()
            msg = MotionCommand()
            msg.header = header
            msg.speed = 0
            msg.angle = 0
            traj_pub.publish(msg))

    def updatePlan(self, plan):
        if not self.brake_flag:
            velocity, angle = self.driver.updatePlan(list(plan.waypoints))
            header = Header()
            header.time = rospy.get_rostime()
            msg = MotionCommand()
            msg.header = header
            msg.speed = velocity
            msg.angle = angle
            traj_pub.publish(msg))

    def updatePosition(self, pose):
        if self.driver.updatePosition(float(pose.x), float(pose.y), float(pose.theta)):
            if not self.brake_flag:
                velocity, angle = driver.makeTrajectory()
                header = Header()
                header.time = rospy.get_rostime()
                msg = MotionCommand()
                msg.header = header
                msg.speed = velocity
                msg.angle = angel
                traj_pub.publish(msg))


if __name__ == "__main__":
    sideWheelSeparation = rospy.get_param('sideWheelSeparation', 0)
    targetV = rospy.get_param('targetV', 0)
    toleranceDist = rospy.get_param('toleranceDist', 0)
    toleranceTheta = rospy.get_param('toleranceTheta', 0)

    driver = BicycleDriveNode(sideWheelSeparation, targetV, toleranceDist, toleranceTheta)
    driver.run()