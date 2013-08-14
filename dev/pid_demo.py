#!/usr/bin/python

import random
#plot import
#import matplotlib.pyplot as plt
import math
from math import (
  pi, sqrt, hypot, sin, cos, tan, asin, acos, atan, atan2, radians, degrees,
  floor, ceil
)


class Robot(object):
    def __init__(self, length = 20.0):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0
        self.x_data = []
        self.y_data = []

    def set_pose(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)
        self.x_data.append(self.x)
        self.y_data.append(self.y) 

    def set_noise(self, new_s_noise, new_d_noise):
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    def set_steering_drift(self, drift):
        self.steering_drift = drift

    # --------
    # move:
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative

    def move(self, steering, distance, tolerance = 0.001, max_steering_angle = pi / 4.0):
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x = self.x + (distance2 * cos(self.orientation))
            self.y = self.y + (distance2 * sin(self.orientation))
            self.orientation = (self.orientation + turn) % (2.0 * pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * pi)
            self.x = cx + (sin(self.orientation) * radius)
            self.y = cy - (cos(self.orientation) * radius)
        self.x_data.append(self.x)
        self.y_data.append(self.y) 

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)


def run(params, line, my_robot):
    speed = 1.0
    n = 100
    old_cte = my_robot.y - (my_robot.x * line[0] + line[1])
    int_cte = 0.0
    
    for i in range(n):
        #calculate error term based on distance from line
        #this should probobly be changed for line following robot..
        #to maybe calculating a lateral offset based on camere.
        cte = my_robot.y - (my_robot.x * line[0] + line[1])
        diff_cte = cte - old_cte
        int_cte += cte
        old_cte = cte
        steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
        my_robot.move(steer, speed)

#creates a bot and sets initial values
bot1 = Robot()
bot1.set_pose(0.0, 1.0, 0.0)
bot1.set_steering_drift(10.0 / 180 * pi) #10 degrees
bot2 = Robot()
bot2.set_pose(0.0, 1.0, 0.0)
bot2.set_steering_drift(10.0 / 180 * pi) #10 degrees
bot3 = Robot()
bot3.set_pose(0.0, 1.0, 0)
bot3.set_steering_drift(10.0 / 180 * pi) #10 degrees

#runs pid on th given robot with parameter 2,0,0 the robot will try and drive along line y=0x+0
run([2.0, 0.0, 0.0], [0, 0], bot1)
run([2.0, 9.0, 0.0], [0, 0], bot2)
run([2.0, 9.0, 0.1], [0, 0], bot3)

##plot code
##fig1 = plt.figure()
##ax1 = fig1.add_subplot(111)
##ax1.plot(bot1.x_data, bot1.y_data)
##ax2 = fig1.add_subplot(111)
##ax2.plot(bot2.x_data, bot2.y_data)
##ax3 = fig1.add_subplot(111)
##ax3.plot(bot3.x_data, bot3.y_data)
##plt.show()
