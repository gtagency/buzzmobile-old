#include <ros/ros.h>
#include <core_msgs/WorldRegion.h>
#include <sensor_msgs/NavSatFix.h>

#define EARTH_RADIUS_METERS 6378137.0
#define PI 3.1415926535897932384626433832795028841971

struct Turn {
  double latitude;
  double longitude;
  double radius;
  bool direction; // Right is true
};

#define NUM_TURNS 1
const struct Turn turns[] = {{33.776913, -84.393633, 10.0, true}};


core_msgs::WorldRegion leftTurn, rightTurn;
ros::Publisher turn_region;

double toRadians(double value) {
  return value * (PI/180);
}

double computeDistance(const sensor_msgs::NavSatFix::ConstPtr& a, double turnLat, double turnLon) {
  double lat1 = toRadians(a->latitude);
  double lat2 = toRadians(turnLat);
  double delta = toRadians(turnLon - a->longitude);
  return acos(sin(lat1)*sin(lat2) + cos(lat1)*cos(lat2)*cos(delta)) * EARTH_RADIUS_METERS;
}

void leftTurnCallback(const core_msgs::WorldRegion::ConstPtr& left) {
  leftTurn = *left;
}

void rightTurnCallback(const core_msgs::WorldRegion::ConstPtr& right) {
  rightTurn = *right;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& fix) {
  for (uint i = 0; i < NUM_TURNS; ++i) {
    if (computeDistance(fix, turns[i].latitude, turns[i].longitude) < turns[i].radius) {
      std::cout << "Turning..." << std::endl;
      if (turns[i].direction) { //Right turn since right is true
	turn_region.publish(rightTurn);
      } else {
	turn_region.publish(leftTurn);
      }
    }
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "gps_turn_detector");
  ros::NodeHandle n;

  ros::Subscriber left = n.subscribe<core_msgs::WorldRegion>("turn/left", 100, leftTurnCallback);
  ros::Subscriber right = n.subscribe<core_msgs::WorldRegion>("turn/right", 100, rightTurnCallback);
  ros::Subscriber fix = n.subscribe<sensor_msgs::NavSatFix>("fix", 100, gpsCallback);
  
  turn_region = n.advertise<core_msgs::WorldRegion>("turn_region", 100);

  ros::spin();
  return 0;
}
