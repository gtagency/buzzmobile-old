
#include <ros/ros.h>
#include <vector>
#include <std_msgs/Bool.h>
//#include <geometry_msgs/Point32.h>
//#include <geometry_msgs/Polygon.h>
#include <core_msgs/ObstacleArrayStamped.h>
#include <core_msgs/Obstacle.h>
//#include <core_msgs/Pose2DAndVelStamped.h>

//#include "bicycle_model/bicycle_model_calculations.h"

#define CAR_WIDTH_METERS (1.5) //edit this
#define MIN_DIST_METERS (2)
ros::Publisher brake_pub;
/*
core_msgs::Pose2DAndVelStamped currentPoseAndVel;

geometry_msgs::Point32 computeCurrentCarPoint() {
  //TODO: use biccle model to extrapolate current pose out.  assume some tunable number of seconds to stop
  geometry_msgs::Point32 pt;
  pt.x = 0;
  pt.y = 0;
  pt.z = 0;
  return pt;
}
*/
bool isCloser(const core_msgs::Obstacle& obj1, const core_msgs::Obstacle& obj2) {
  return obj1.center.y < obj2.center.y;
}

void obstacleInDistanceCallback(const core_msgs::ObstacleArrayStamped::ConstPtr& inmsg) {
  std::vector<core_msgs::Obstacle> obstacles(inmsg->obstacles);
  if (obstacles.size() == 0) {
    return;
  }
  std::vector<core_msgs::Obstacle> inWidth;
  for (unsigned int i = 0; i < obstacles.size(); i++) {
    if (fabs(obstacles[i].center.y) < CAR_WIDTH_METERS) {
      inWidth.push_back(obstacles[i]);
    }
  }
  core_msgs::Obstacle closest = *std::max_element(inWidth.begin(), inWidth.end(), isCloser);
  std_msgs::Bool msg;
  if (closest.center.x > CAR_WIDTH_METERS/2 && closest.center.x < MIN_DIST_METERS) {
    msg.data = true;
    brake_pub.publish(msg);
  }
}

void obstacleCallback(const core_msgs::ObstacleArrayStamped::ConstPtr& msg) {
//  geometry_msgs::Point32 testPt = computeCurrentCarPoint();  
  /*
  for (std::vector<core_msgs::Obstacle>::const_iterator it = msg->obstacles.begin();
       it != msg->obstacles.end();
       it++) {
    for (std::vector<geometry_msgs::Polygon>::const_iterator jt = it->polygons.begin();
         jt != it->polygons.end();
         jt++) {
      if (hitTest(*jt, testPt)) {
        //NOTE: this node only publishes true.  It's up to another node (such as the teleop node) to clear the flag.
        std_msgs::Bool msg;
        msg.data = true;
        brake_pub.publish(msg);
        break;
      }
    }
  }*/
}
/*
void carPosAndVelCallback(const core_msgs::Pose2DAndVelStamped::ConstPtr& msg) {
  currentPoseAndVel.header = msg->header;
  currentPoseAndVel.pose = msg->pose;
  currentPoseAndVel.linearVel = msg->linearVel;
}
*/
int main(int argc, char **argv) {

  ros::init(argc, argv, "brake");
  ros::NodeHandle n;

  ros::Subscriber s = n.subscribe<core_msgs::ObstacleArrayStamped>("obstacles", 100, obstacleInDistanceCallback);
  //ros::Subscriber s2 = n.subscribe<core_msgs::Pose2DAndVelStamped>("car_pos_and_vel", 100, carPosAndVelCallback);
  brake_pub = n.advertise<std_msgs::Bool>("brake", 1000, true);

  ros::spin();
  return 0;
}
