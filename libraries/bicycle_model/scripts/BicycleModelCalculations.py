import math

""" Calculates the change in position in the robot frame
    v: forward velocity of the car
    angle: steering angle of the car
    time: time lapse for motion """
def changePosition(v, angle, time, carLength):
    return forwardKinematics(v, angularVelocity(v, angle, carLength), time)

""" Calculates the change in position in the robot frame
    v: forward velocity of the car
    w: angular velocity of the car
    time: time lapse for motion """
def forwardKinematics(v, w, time):
    dx = v * math.sin(w * time) / w
    dy = v * (1 - math.cos(w * time)) / w
    dtheta = w * time
    return (dx, dy, dtheta)

""" Calculates angular velocity from car inputs
    v: forward velocity of the car
    angle: steering angle of the car """
def angularVelocity(v, angle, carLength):
    return v * math.tan(angle) / carLength

""" Converts a points in the world frame (xW, yW, thetaW) to
    a point in the robot frame given the robot position in
    the world (carX, carY, carTheta) """
def toRobotFrame(xW, yW, thetaW, carX, carY, carTheta):
    xR = (xW - carX) * math.cos(carTheta) + (yW - carY) * math.sin(carTheta)
    yR = (carX - xW) * math.sin(carTheta) + (yW - carY) * math.cos(carTheta)
    thetaR = thetaW - carTheta
    return (xR, yR, thetaR)

""" Converts a points in the robot frame (xR, yR, thetaR) to
    a point in the world frame given the robot position in
    the world (carX, carY, carTheta) """
def toWorldFrame(xR, yR, thetaR, carX, carY, carTheta):
    xW = xR * math.cos(carTheta) - yR * math.sin(carTheta) + carX
    yW = xR * math.sin(carTheta) + yR * math.cos(carTheta) + carY
    thetaW = (thetaR + carTheta) % (2 * math.pi)
    return (xW, yW, thetaW)