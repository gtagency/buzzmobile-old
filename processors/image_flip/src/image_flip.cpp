#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "image_lib/image_lib.h"
using namespace image_lib;

ros::Publisher flipped_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& image) {

  cv_bridge::CvImageConstPtr cv_ptr;
  imageMsgToCvShare(image, cv_ptr);

  cv::Mat flipped(cv_ptr->image.size(), cv_ptr->image.type());

  flip(cv_ptr->image, flipped, -1);

  std_msgs::Header header;
  cv_bridge::CvImage flippedImage(header, getRosType(flipped.type()), flipped);

  flipped_pub.publish(flippedImage.toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_flip");
  ros::NodeHandle n;

  std::cout << "Image Flipper starting." << std::endl;
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image", 1, imageCallback);
  flipped_pub = n.advertise<sensor_msgs::Image>("image_flipped", 100);
  std::cout << "Image Flipper started." << std::endl;
  ros::spin();
  return 0;
}
