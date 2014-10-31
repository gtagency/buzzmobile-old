#!/usr/bin/env python

import roslib; roslib.load_manifest("driver_bicycle")
import rospy
import bicycle_driver
from geometry_msgs.msg import Pose2D
from planner_astar_bicycle.msg import Path
from std_msgs.msg import Bool, Header
from core_msgs.msg import MotionCommand

class BicycleDriveNode:
    def __init__(self, sideWheelSeparation, targetV, toleranceDist, toleranceTheta):
        self.driver = bicycle_driver.BicycleDriver(sideWheelSeparation, targetV,
                                    toleranceDist, toleranceTheta)
        self.traj_pub = rospy.Publisher("motion_command", MotionCommand)
        rospy.Subscriber("car_position", Pose2D, self.updatePosition)
        rospy.Subscriber("planned_path", Path, self.updatePlan)
        rospy.Subscriber("brake", Bool, self.setStopFlag)
        self.brake_flag = True
        self.driver.updatePosition(200, 400, 0)

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            r.sleep()

    def publishMotionCommand(self, speed, angle):
        header = Header()
#          header.time = rospy.get_rostime()
        msg = MotionCommand()
        msg.header = header
        msg.speed = speed 
        msg.angle = angle
        self.traj_pub.publish(msg)


    def setStopFlag(self, flag):
        self.brake_flag = flag.data
        if flag.data:
            self.publishMotionCommand(0, 0)

    def updatePlan(self, plan):
        if not self.brake_flag:
            waypoints = [(p.x,p.y,p.theta) for p in plan.poses]
            velocity, angle = self.driver.updatePlan(waypoints)
            self.publishMotionCommand(velocity, angle)

    def updatePosition(self, pose):
        if self.driver.updatePosition(float(pose.x), float(pose.y), float(pose.theta)):
            if not self.brake_flag:
                velocity, angle = driver.makeTrajectory()
                self.publishMotionCommand(velocity, angle)


if __name__ == "__main__":
    rospy.init_node("driver_bicycle")
    sideWheelSeparation = rospy.get_param('sideWheelSeparation', 1)
    targetV = rospy.get_param('targetV', 1)
    toleranceDist = rospy.get_param('toleranceDist', 0)
    toleranceTheta = rospy.get_param('toleranceTheta', 0)

    driver = BicycleDriveNode(sideWheelSeparation, targetV, toleranceDist, toleranceTheta)
    driver.run()
