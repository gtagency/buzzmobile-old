
#include "bicycle_model/bicycle_model_calculations.h"

geometry_msgs::Vector3 changePosition(double linearVel, double steerAngle, double timeSecs, double carLength) {
  return forwardKinematics(linearVel, angularVelocity(linearVel, steerAngle, carLength), timeSecs);
}


geometry_msgs::Vector3 forwardKinematics(double linearVel, double angularVel, double timeSecs) {
  double dth = angularVel * timeSecs;
  double dx  = linearVel * sin(dth) / angularVel;
  double dy  = linearVel * (1 - cos(dth)) / angularVel;

  geometry_msgs::Vector3 msg;
  msg.x = dx;
  msg.y = dy;
  msg.z = dth;
  return msg;
}

double angularVelocity(double linearVel, double steerAngle, double carLength) {
  return linearVel * tan(steerAngle) / carLength;
}


