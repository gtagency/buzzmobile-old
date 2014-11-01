#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <core_msgs/WorldRegion.h>

ros::Publisher region_pub;

core_msgs::WorldRegion getFullRegion() {
  core_msgs::WorldRegion msg;
  msg.width = 400;
  msg.height = 400;
  msg.resolution = 23;
  msg.labels.assign(msg.width * msg.height, 1);
  return msg;
}

core_msgs::WorldRegion getVerticalHalfRegion() {
  core_msgs::WorldRegion msg;
  msg.width = 400;
  msg.height = 400;
  msg.resolution = 23;
  msg.labels.assign(msg.width * msg.height, 1);
  for (unsigned int row = 0; row < msg.height; ++row) {
    for (unsigned int col = 0; col < msg.width / 2; ++col) {
      msg.labels[row * msg.width + col] = 0;
    }
  }
  return msg;
}

core_msgs::WorldRegion getHorizontalHalfRegion() {
  core_msgs::WorldRegion msg;
  msg.width = 400;
  msg.height = 400;
  msg.resolution = 23;
  msg.labels.assign(msg.width * msg.height, 1);
  for (unsigned int row = 0; row < msg.height / 2; ++row) {
    for (unsigned int col = 0; col < msg.width; ++col) {
      msg.labels[row * msg.width + col] = 0;
    }
  }
  return msg;
}

core_msgs::WorldRegion getDiagonalRightRegion() {
  core_msgs::WorldRegion msg;
  msg.width = 400;
  msg.height = 400;
  msg.resolution = 23;
  msg.labels.assign(msg.width * msg.height, 0);
  for (unsigned int row = 0; row < msg.height; ++row) {
    unsigned int start = msg.width - 2*row/3;
    unsigned int end = std::min(msg.width - 1, msg.width - row/2 + 100);

    for (unsigned int col = start; col < end; ++col) {
      msg.labels[row * msg.width + col] = 1;
    }
  }
  return msg;
}

void regionNumberCallback(const std_msgs::UInt8::ConstPtr& num) {
  switch(num->data) {
    case 1:
      region_pub.publish(getVerticalHalfRegion());
      break;
    case 2:
      region_pub.publish(getHorizontalHalfRegion());
      break;
    case 3:
      region_pub.publish(getDiagonalRightRegion());
      break;
    case 0:
    default:
      region_pub.publish(getFullRegion());
      break;
  }
}
int main(int argv, char **argc) {
  ros::init(argv, argc, "test_region");
  ros::NodeHandle n;

  ros::Subscriber s = n.subscribe<std_msgs::UInt8>("region_number", 1, regionNumberCallback);
  region_pub = n.advertise<core_msgs::WorldRegion>("test_region", 1);
  ros::spin();
  return 0;
}
