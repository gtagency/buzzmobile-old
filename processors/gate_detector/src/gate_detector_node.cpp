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

#define MINIMUM_DISTANCE_METERS 0.0

ros::Publisher full_region_pub, left_turn_pub, right_turn_pub;

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

int line(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b, int y) {
  double div = b.y - a.y;
  // Handle vertical lines with grace
  if (div == 0) {
    div++;
  }
  return a.x + (((b.x - a.x)/div) * (y - a.y));
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
  if(obstacles.size() == 0) {
    return;
  }
  std::vector<core_msgs::Obstacle>::iterator bound = std::partition(obstacles.begin(), obstacles.end(), isYPositive);
  core_msgs::Obstacle post1 = *std::max_element(obstacles.begin(), bound, obsDistanceCompare);
  core_msgs::Obstacle post2 = *std::max_element(bound, obstacles.end(), obsDistanceCompare);
  core_msgs::WorldRegion full, leftTurn, rightTurn;
  full.width = leftTurn.width = rightTurn.width =  params.output_x_res;
  full.height = leftTurn.height = rightTurn.height = params.output_y_res;
  full.resolution = leftTurn.resolution = rightTurn.resolution = params.pixels_per_meter;

  full.labels.assign(full.width * full.height, 0);
  leftTurn.labels.assign(leftTurn.width * leftTurn.height, 0);
  rightTurn.labels.assign(rightTurn.width * rightTurn.height, 0);

  geometry_msgs::Point32 bottomLeft, bottomRight;
  bottomRight.y = -(params.car_width_meters / 2) * full.resolution;
  bottomLeft.y  = (params.car_width_meters / 2) * full.resolution;
  bottomLeft.x = bottomRight.x = 0;

  post1.center.x *= full.resolution;
  post1.center.y *= full.resolution;

  post2.center.x *= full.resolution;
  post2.center.y *= full.resolution;

  for (int row = 0; row < full.height; row++) {
    for (int col = 0; col < full.width; col++) {
      int position = row * full.width + col;
      int x = full.height - row - 1;
      int y = (full.width / 2) - col;
      if (x > MINIMUM_DISTANCE_METERS * full.resolution) {

        if (x >= line(bottomRight, post2.center, y) && x >= line(bottomLeft, post1.center, y)) {
          full.labels[position] = 1;
        } else {
          full.labels[position] = 0;
        }

	if (x >= line(bottomRight, post2.center, y) && x <= line(bottomRight, post2.center, y) + ((full.height >> 1) - bottomRight.y) + (params.car_width_meters * full.resolution)) {
	  rightTurn.labels[position] = 1;
	} else {
	  rightTurn.labels[position] = 0;
	}

	if (x >= line(bottomLeft, post1.center, y) && x <= line(bottomLeft, post1.center, y) + ((full.height >> 1) - bottomLeft.y) + (params.car_width_meters * full.resolution)) {
	  leftTurn.labels[position] = 1;
	} else {
	  leftTurn.labels[position] = 0;
	}

      } else {
        full.labels[position] = 0;
	leftTurn.labels[position] = 0;
	rightTurn.labels[position] = 0;
      }
    }
  }

  full_region_pub.publish(full);
  right_turn_pub.publish(rightTurn);
  left_turn_pub.publish(leftTurn);
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

  DECLARE_PARAM(car_width, double, 1);
  DECLARE_PARAM(output_x_res, double, 400);
  DECLARE_PARAM(output_y_res, double, 400);
  DECLARE_PARAM(pixels_per_meter, double, 23);

  params.car_width_meters = car_width;
  params.output_x_res = output_x_res;
  params.output_y_res = output_y_res;
  params.pixels_per_meter = pixels_per_meter;

  ros::Subscriber s = n.subscribe<core_msgs::ObstacleArrayStamped>("obstacles", 100, obstaclesCallback);
  //TODO: occupancy map?
  full_region_pub = n.advertise<core_msgs::WorldRegion>("gate_region", 100);
  left_turn_pub = n.advertise<core_msgs::WorldRegion>("turn/left", 100);
  right_turn_pub = n.advertise<core_msgs::WorldRegion>("turn/right", 100);

  ros::spin();
  return 0;
}
