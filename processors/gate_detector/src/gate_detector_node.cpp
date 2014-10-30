/**
 * gate_detector_node.cpp
 * A ROS node that subscribes to an obstacle topic, finds the
 * furthest obstacle on either side of the car, and produces
 * a region that spans the car and those points.  This region
 * can be used for planning, to plan a path between the "goal
 * posts"
 */

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <core_msgs/ObstacleArrayStamped.h>
#include <core_msgs/Obstacle.h>
#include <core_msgs/WorldRegion.h>

#define MINIMUM_DISTANCE_METERS 2.5

ros::Publisher region_pub;

struct Params {
  double output_x_res;
  double output_y_res;
  double pixels_per_meter;
  double car_width_meters;
};

Params params;

bool isYPositive(const core_msgs::Obstacle& ob) {
  return ob.center.y > 0;
}

int line(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b, int x) {
  return a.y + (((b.y - a.y)/(double) (b.x - a.x)) * (x - a.x));
}

//distance from 0,0 = magnitude of point vector
double euclideanDistance(const geometry_msgs::Point32& pt) {
  return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

bool obsDistanceCompare(const core_msgs::Obstacle& ob1, const core_msgs::Obstacle& ob2) {
  return euclideanDistance(ob1.center) < euclideanDistance(ob2.center);
}

void obstaclesCallback(const core_msgs::ObstacleArrayStamped::ConstPtr& inmsg) {
  std::vector<core_msgs::Obstacle> obstacles(inmsg->obstacles);
  std::vector<core_msgs::Obstacle>::iterator bound = std::partition(obstacles.begin(), obstacles.end(), isYPositive);
  core_msgs::Obstacle post1 = *std::max_element(obstacles.begin(), bound, obsDistanceCompare);
  core_msgs::Obstacle post2 = *std::max_element(bound, obstacles.end(), obsDistanceCompare);
  core_msgs::WorldRegion msg;
  msg.width = params.output_x_res;
  msg.height = params.output_y_res;
  msg.resolution = params.pixels_per_meter;
  msg.labels.assign(msg.width * msg.height, 0);
  geometry_msgs::Point32 bottomLeft, bottomRight;
  bottomRight.y = (params.car_width_meters / 2) * msg.resolution;
  bottomLeft.y = -(params.car_width_meters / 2) * msg.resolution;
  bottomLeft.x = bottomRight.x = 0;

  post1.center.x *= msg.resolution;
  post1.center.y *= msg.resolution;

  post2.center.x *= msg.resolution;
  post2.center.y *= msg.resolution;

  for (int row = 0; row < msg.height; row++) {
    for (int col = 0; col < msg.width; col++) {
      int position = row * msg.width + col;
      int x = msg.height - row;
      // This assumes that y is positive to the right. Might want to fix in obstacle detector
      int y = col - (msg.width / 2);
      if (x > MINIMUM_DISTANCE_METERS * msg.resolution) {
	if (x > line(bottomRight, post2.center, y) && x > line(bottomLeft, post1.center, y)) {
	  msg.labels[position] = 1;
	} else {
	  msg.labels[position] = 0;
	}
       } else {
	  msg.labels[position] = 0;
       }
    }
  }
  region_pub.publish(msg);
}

#define DECLARE_PARAM(name, type, defVal)\
  type name;\
  if (!npriv.getParam(#name, name)) {\
    name = defVal;\
  }

int main(int argv, char **argc) {
  ros::init(argv, argc, "gate_detector");
  ros::NodeHandle n;
  ros::NodeHandle npriv("~");

  DECLARE_PARAM(car_width, double, 5);
  DECLARE_PARAM(output_x_res, double, 800);
  DECLARE_PARAM(output_y_res, double, 800);
  //Was 40 pixels/meter
  DECLARE_PARAM(pixels_per_meter, double, 10);

  params.car_width_meters = car_width;
  params.output_x_res = output_x_res;
  params.output_y_res = output_y_res;
  params.pixels_per_meter = pixels_per_meter;

  ros::Subscriber s = n.subscribe<core_msgs::ObstacleArrayStamped>("/obstacles", 100, obstaclesCallback);
  //TODO: occupancy map?
  region_pub = n.advertise<core_msgs::WorldRegion>("/goal_region", 100);

  ros::spin();
  return 0;
}
