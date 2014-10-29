
#include <ros/ros.h>

#include <core_msgs/ObstacleArrayStamped.h>
#include <core_msgs/WorldRegion.h>

ros::Publisher worldModel_pub;

void updateWorldModel();

struct RegionCallback {

  core_msgs::WorldRegion region;
  void operator()(const core_msgs::WorldRegion::ConstPtr& region) {
    this->region = *region;
    updateWorldModel();
  }
};

RegionCallback gateRegionCallback;
RegionCallback laneRegionCallback;
std::vector<core_msgs::Obstacle> obstacles;
void obstaclesCallback(const core_msgs::ObstacleArrayStamped::ConstPtr& obsArray) {
  obstacles = obsArray->obstacles;
  updateWorldModel();
}

void updateWorldModel() {
  //TODO: current assumes all regions are the same size and resolution...this should be more flexible.
  //TODO: this also ignores timestamps, assuming that we're just going to get a lot of data...
  //
  // First: construct a region wherein everything is drivable except the obstacles
  core_msgs::WorldRegion obsregion;
  obsregion.width = gateRegionCallback.region.width;
  obsregion.height = gateRegionCallback.region.height;
  obsregion.resolution = gateRegionCallback.region.resolution;
  obsregion.labels.assign(obsregion.width * obsregion.height, 1); //assume everything is drivable
  for (std::vector<core_msgs::Obstacle>::iterator it = obstacles.begin();
       it != obstacles.end();
       it++) {
    int centerRow = obsregion.height - (it->center.x * obsregion.resolution); 
    int centerCol = obsregion.width - (it->center.y * obsregion.resolution);
    int radiusInPixels = (int)ceil(it->radius * obsregion.resolution);
    for (int row = centerRow - radiusInPixels; row <= centerRow + radiusInPixels; row++) {
      int colDist = (int)ceil(sqrt(radiusInPixels * radiusInPixels - row * row));
      for (int col = centerCol - colDist; col <= centerCol + colDist; col++) {
        obsregion.labels[row * obsregion.width + col] = 0;
      }
    }
  }

  // Next: merge all 3 regions together
  core_msgs::WorldRegion merged = gateRegionCallback.region;
  for (unsigned int inx = 0; inx < merged.width * merged.height; inx++) {
    if (merged.labels[inx] != laneRegionCallback.region.labels[inx]
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
