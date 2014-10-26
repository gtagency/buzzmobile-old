#!/usr/bin/env python

import rospy
import differential_driver
from geometry_msgs.msg import Pose2D
from corobot_msgs.msg import MotorCommand
from planner_astar.msg import Path
from std_msgs.msg import Bool

class DifferentialDriveNode:
    def __init__(wheelRadius, wheelSeparation, speedAdjustment, targetV, toleranceDist, toleranceTheta):
        self.driver = DifferentialDriver(wheelRadius, wheelSeparation, speedAdjustment,
                      targetV, toleranceDist, toleranceTheta)
        self.traj_pub = rospy.Publisher("PhidgetMotor", MotorCommand)
        rospy.Subscriber("car_position", Pose2D, self.updatePosition)
        rospy.Subscriber("planned_path", Path, self.updatePlan)
        rospy.Subscriber("running_flag", Bool, self.setDriveFlag)
        self.run_flag = False

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            r.sleep()

    def setDriveFlag(self, flag):
        self.run_flag = flag.data
        if not flag.data:
            msg = MotorCommand()
            msg.leftSpeed = 0
            msg.rightSpeed = 0
            msg.secondsDuration = 0
            msg.acceleration = 0
            traj_pub.publish(msg))

    def updatePlan(self, plan):
        if self.run_flag:
            wheelSpeeds = self.driver.updatePlan(list(plan.waypoints))
            msg = MotorCommand()
            msg.leftSpeed = round(wheelSpeeds[0])
            msg.rightSpeed = round(wheelSpeeds[0])
            msg.secondsDuration = 0
            msg.acceleration = 100
            self.traj_pub.publish(msg)

    def updatePosition(self, pose):
        if self.driver.updatePosition(float(pose.x), float(pose.y), float(pose.theta)):
            if self.run_flag:
                wheelSpeeds = driver.makeTrajectory()
                msg = MotorCommand()
                msg.leftSpeed = round(wheelSpeeds[0])
                msg.rightSpeed = round(wheelSpeeds[0])
                msg.secondsDuration = 0
                msg.acceleration = 0
                self.traj_pub.publish(msg)


if __name__ == "__main__":
    # TODO confirm names
    wheelRadius = rospy.get_param('wheelRadius', 0)
    wheelSeparation = rospy.get_param('wheelSeparation', 0)
    speedAdjustment = rospy.get_param('speedAdjustment', 0)
    targetV = rospy.get_param('targetV', 0)
    toleranceDist = rospy.get_param('toleranceDist', 0)
    toleranceTheta = rospy.get_param('toleranceTheta', 0)

    driver = DifferentialDriveNode(wheelRadius, wheelSeparation, speedAdjustment, targetV, toleranceDist, toleranceTheta)
    driver.run()