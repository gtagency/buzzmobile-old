import math

class DifferentialDriver:
    def __init__(self, wheelRadius, wheelSeparation, speedAdjustment, targetV, toleranceDist, toleranceTheta):
        self.wheelRadius = wheelRadius
        self.wheelSeparation = wheelSeparation
        self.speedAdjustment = speedAdjustment
        self.targetV = targetV
        self.toleranceDist =  toleranceDist
        self.toleranceTheta = toleranceTheta

        self.x = 0
        self.y = 0
        self.theta = 0
        self.started = False

        self.waypoints = []

    """ Updates the stored position. If this is the first position or the position
        is within range of the current target waypoint then the method will return
        True as a trigger to update the trajectory. Otherwise the methods will
        return False."""
    def updatePosition(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

        if not self.started:
            self.started = True
            return True

        if self.waypoints:
            targetX, targetY, targetTheta = self.waypoints[0]
            if math.pow(x - targetX, 2) + math.pow(y - targetY, 2) < math.pow(self.toleranceDist, 2) and abs(theta - targetTheta) < self.toleranceTheta:
                self.waypoints.pop(0)
                return True

        return False

    """ Updates the projected plan. If a position has already been set then this
        method will return wheel speeds to follow. Otherwise it will return zero
        for the wheel speeds."""
    def updatePlan(self, waypoints):
        self.waypoints = waypoints
        if self.started:
            return self.makeTrajectory()
        return (0, 0)

    """ Makes a trajectory for a single waypoint. If it is possible to reach that
        point at the target velocity then this method will return wheel speeds to
        follow. Otherwise it will return zero for the wheel speeds. If no waypoint
        is provided, it will use the first point in the waypoints list. If the
        waypoints list is empty then the method will return zero for the wheel
        speeds."""
    def makeTrajectory(self, waypoint=None):
        if waypoint is None:
            if not self.waypoints:
                return (0, 0)
            waypoint = self.waypoints[0]
        dx, dy, dtheta = self.convertToDriverFrame(waypoint)

        v = self.targetV
        w = v / (dx / math.tan(dtheta) + dy)

        spdL = (v - w * self.wheelSeparation / 2) / (self.wheelRadius * self.speedAdjustment)
        spdR = (v + w * self.wheelSeparation / 2) / (self.wheelRadius * self.speedAdjustment)

        return (spdL, spdR)

    """ Converts an (x, y, theta) pose from the world coordinate frame to the
        driver coordinate frame. """
    def convertToDriverFrame(self, pose):
        xP, yP, thetaP = pose
        xR, yR, thetaR = (self.x, self.y, self.theta)

        xPR = (xP - xR) * math.cos(thetaR) + (yP - yR) * math.sin(thetaR)
        yPR = (xR - xP) * math.sin(thetaR) + (yP - yR) * math.cos(thetaR)
        thetaPR = thetaP - thetaR

        return (xPR, yPR, thetaPR)