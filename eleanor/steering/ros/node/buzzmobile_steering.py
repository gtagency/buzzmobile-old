#!/usr/bin/env python
"""Linear Actuator control using Pololu 18v15 motor driver, powered by
a Teensy 2.0 and custom software.
"""

__author__ = 'Jesse Rosalia <jesse.rosalia@gatech.edu>'
__version__ = '1'

import roslib; roslib.load_manifest('buzzmobile_steering')
from core_srvs.srv import *
from core_msgs.msg import *
from std_msgs.msg import UInt32

import rospy
from threading import Timer
from ctypes import *
import serial

ser = None

invert_speed = True
linear = 0
feedbackSensor = 0
minAcceleration = 0
maxAcceleration = 0
minSpeed = -100
maxSpeed = 100
timer = 0
posdataPub = None
position = 0

def stop():
    print "Stopping at", position
    ser.write('s')
    return(True)

def bounded_value(val, minVal, maxVal):
    bv = val
    if bv > maxVal:
        bv = maxVal
    elif bv < minVal:
        bv = minVal
   
    return bv 

def move(request):
    """Cause the linear actuator to move or stop moving

    Request a common acceleration, wheel directions and wheel speeds

	NOTE: currently, speed and acceleration are ignored by the motor controller
    this is just to control direction -JR 03/11/14
    """
    global timer, invert_speed
    if timer:
        timer.cancel();
        rospy.logdebug(
            "Speed: %d", 
            request.speed
            )

    # NOTE: negate the speed so + is out and - is back
    # FIXME: this should be an attribute
    speed        = bounded_value(request.speed,        float(minSpeed), float(maxSpeed))
    if invert_speed:
        speed = -speed

    rospy.logdebug(
            "Speed: %d", 
            speed
            )

    ser.write('s' if speed == 0 else 'l' if speed < 0 else 'r')

    if request.secondsDuration != 0:
        timer = Timer(request.secondsDuration, stop)
        timer.start()
    return(True)

def setupMoveService():

    rospy.init_node(
            'buzzmobile_steering',
            log_level = rospy.DEBUG
            )
    
    device = rospy.get_param("~device", None)
    if device:
        rospy.loginfo("Using motor controller with device %s", device)
    else:
        rospy.loginfo("No device specified.  Exiting...")
        exit(1)

    global ser, timer, invert_speed, posdataPub
    timer = 0

    ser = serial.Serial(device, 9600)
    invert_speed = rospy.get_param('~invert_speed', False)

    phidgetMotorTopic = rospy.Subscriber("buzzmobile_steering", LinearCommand ,move)
    posdataPub = rospy.Publisher("position", UInt32, latch=True)

    r = rospy.Rate(100) #100hz - FAST

    while not rospy.is_shutdown():
        if posdataPub:
            posdataPub.publish(position)
        r.sleep()

if __name__ == "__main__":
    setupMoveService()

    ser.close()
