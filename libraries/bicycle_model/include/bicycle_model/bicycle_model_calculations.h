
#ifndef __BICYCLE_MODEL_CALCULATIONS_H
#define __BICYCLE_MODEL_CALCULATIONS_H

#include <geometry_msgs/Vector3.h>

/**
 *
 * returns dx,dy,dtheta in a Vector3
 */ 
geometry_msgs::Vector3 changePosition(double linearVel, double angle, double timeSecs, double carLength);

geometry_msgs::Vector3 forwardKinematics(double linearVel, double angle, double timeSecsrLength);

double angularVelocity(double linearVel, double steerAngle, double carLength);

#endif // __BICYCLE_MODEL_CALCULATIONS_H
