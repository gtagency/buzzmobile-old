
#include <ros/ros.h>

#include <core_msgs/ObstacleArrayStamped.h>
#include <core_msgs/WorldRegion.h>

ros::Publisher worldModel_pub;

void updateWorldModel();

struct RegionCallback {

  core_msgs::WorldRegion region;
  void operator()(const core_msgs::WorldRegion::ConstPtr& region) {
    this->region = *region;
    std::cout << "Receivin: " << this->region.width << std::endl;
    updateWorldModel();
  }
};

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
  core_msgs::WorldRegion& baseRegion = laneRegion;
  if (baseRegion.width == 0) {
    return;
  }
  std::cout << "I'm in" << std::endl;
  obsregion.width = baseRegion.width;
  obsregion.height = baseRegion.height;
  obsregion.resolution = 100; // baseRegion.resolution;
  int xres = 10;
  int yres = 10;
  obsregion.labels.assign(obsregion.width * obsregion.height, 1); //assume everything is drivable
  for (std::vector<core_msgs::Obstacle>::iterator it = obstacles.begin();
       it != obstacles.end();
       it++) {

    int centerX = obsregion.height - (it->center.x * xres);
    int centerY = (it->center.y * yres) - (obsregion.width / 2);
    int radius = it->radius * xres;

    for (int row = 0; row < obsregion.height; ++row) {
      for (int col = 0; col < obsregion.width; ++col) {
	int position = row * obsregion.width + col;
	int x = obsregion.height - row;
	int y = col - (obsregion.width / 2);
	if (euclideanDistance(centerX, centerY, x, y) < radius) {
	  obsregion.labels[position] = 0;
	} else {
	  obsregion.labels[position] = 1;
	}
      }
    }
    //FIXME: NOT SURE IF RIGHT THIS CODE IS TERRIBLE PLEASE DONT JUDGE ME
    /*unsigned int centerRow = obsregion.height - (it->center.x * obsregion.resolution); 
    unsigned int centerCol = obsregion.width / 2 + (it->center.y * yres);
    int radiusInPixels = (int)ceil(it->radius * obsregion.resolution);
    std::cout << it->center.x << "," << centerRow << "," << it->center.y << "," << centerCol << "," << radiusInPixels << std::endl;
    
    for (unsigned int row = std::min(obsregion.height - 1, centerRow + radiusInPixels); row <= std::max(0, (int)(centerRow - radiusInPixels)); --row) {
      int colDist = (int)ceil(sqrt(radiusInPixels * radiusInPixels - row * row));
      for (unsigned int col = std::max((unsigned int)0, centerCol - colDist); col <= std::min(obsregion.width - 1, centerCol + colDist); col++) {
        std::cout << row << "," << col << std::endl;
        obsregion.labels[row * obsregion.width + col] = 0;
      }
      }*/
  }

  // Next: merge all 3 regions together
  core_msgs::WorldRegion merged = gateRegion;
  for (unsigned int inx = 0; inx < merged.width * merged.height; inx++) {
    if (merged.labels[inx] != laneRegion.labels[inx]
        || merged.labels[inx] != obsregion.labels[inx]) {
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
