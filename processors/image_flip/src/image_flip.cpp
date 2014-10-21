#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;
ros::Publisher flipped_pub;

const std::string& getRosType(int cvType) {
  switch(cvType) {
    case CV_8UC3: return enc::BGR8;
    case CV_8UC1: return enc::MONO8;
    default:
      std::stringstream s;
      s << cvType;
      throw std::runtime_error("Unrecognized Opencv type [" + s.str() + "]");
  }
}

void imageMsgToCvShare(const sensor_msgs::ImageConstPtr& image, cv_bridge::CvImageConstPtr& cv_ptr) {
  try {
    if (enc::isColor(image->encoding))
      cv_ptr = cv_bridge::toCvShare(image, enc::BGR8);
    else
      cv_ptr = cv_bridge::toCvShare(image, enc::MONO8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

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
