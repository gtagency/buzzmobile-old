#include <ros/ros.h>
#include <core_msgs/ObstacleArrayStamped.h>
#include <core_msgs/WorldRegion.h>

ros::Publisher worldModel_pub;

void updateWorldModel();

core_msgs::WorldRegion gateRegion;

void gateRegionCallback(const core_msgs::WorldRegion::ConstPtr& region) {
  gateRegion = *region;
  updateWorldModel();
}
core_msgs::WorldRegion laneRegion;

void laneRegionCallback(const core_msgs::WorldRegion::ConstPtr& region) {
  laneRegion = *region;
  updateWorldModel();
}

std::vector<core_msgs::Obstacle> obstacles;
void obstaclesCallback(const core_msgs::ObstacleArrayStamped::ConstPtr& obsArray) {
  obstacles = obsArray->obstacles;
  std::cout << obstacles.size() << std::endl;
  updateWorldModel();
}

double euclideanDistance(int centerX, int centerY, int ptX, int ptY) {
  return sqrt(pow(centerX - ptX, 2) + pow(centerY - ptY, 2));
}

void updateWorldModel() {
  //TODO: current assumes all regions are the same size and resolution...this should be more flexible.
  //TODO: this also ignores timestamps, assuming that we're just going to get a lot of data...
  //
  // First: construct a region wherein everything is drivable except the obstacles
  std::cout << "Updating" << laneRegion.width << std::endl;
  core_msgs::WorldRegion obsregion;
  core_msgs::WorldRegion& baseRegion = gateRegion; //laneRegion;
  if (baseRegion.width == 0) {
    return;
  }
  std::cout << "I'm in" << std::endl;
  obsregion.width = baseRegion.width;
  obsregion.height = baseRegion.height;
  obsregion.resolution = 100; // baseRegion.resolution;
  int xres = 23;
  int yres = 23;
  obsregion.labels.assign(obsregion.width * obsregion.height, 1); //assume everything is drivable
  for (std::vector<core_msgs::Obstacle>::iterator it = obstacles.begin();
       it != obstacles.end();
      it++) {
    int centerX = (it->center.x * xres);
    int centerY = (it->center.y * yres);
    int radius = it->radius * xres;

    //std::cout << centerX << ", " << centerY << ", " << radius << std::endl;

    for (uint row = 0; row < obsregion.height; ++row) {
      for (uint col = 0; col < obsregion.width; ++col) {
	int position = row * obsregion.width + col;
	int x = obsregion.height - row;
	int y = col - (obsregion.width / 2);
	if (euclideanDistance(centerX, centerY, x, y) < radius) {
	  obsregion.labels[position] = 0;
	}
      }
    }
  }

  // Next: merge all 3 regions together
  core_msgs::WorldRegion merged;
  merged.width = gateRegion.width;
  merged.height = gateRegion.height;
  merged.resolution = gateRegion.resolution;
  merged.labels.assign(merged.height * merged.width, 0);
  for (unsigned int inx = 0; inx < merged.width * merged.height; inx++) {
    if (gateRegion.labels[inx] == 1 
	//&& laneRegion.labels[inx] == 1
        && obsregion.labels[inx] == 1) {
      merged.labels[inx] = 1;
    } else {
      merged.labels[inx] = 0;
    }
  }

  worldModel_pub.publish(merged);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "world_model");
  ros::NodeHandle n;

  worldModel_pub = n.advertise<core_msgs::WorldRegion>("world_model", 100);

  ros::Subscriber s  = n.subscribe<core_msgs::WorldRegion>("gate_region", 100, gateRegionCallback);
  ros::Subscriber s2 = n.subscribe<core_msgs::WorldRegion>("lane_region", 100, laneRegionCallback);
  ros::Subscriber s3 = n.subscribe<core_msgs::ObstacleArrayStamped>("obstacles", 100, obstaclesCallback);

  ros::spin();
  return 0;
}
