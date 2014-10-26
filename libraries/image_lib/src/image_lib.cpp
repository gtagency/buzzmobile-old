#include <ros/ros.h>
#include "image_lib/image_lib.h"
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

const std::string& image_lib::getRosType(int cvType) {
  switch(cvType) {
    case CV_8UC3: return enc::BGR8;
    case CV_8UC1: return enc::MONO8;
    default:
      std::stringstream s;
      s << cvType;
      throw std::runtime_error("Unrecognized Opencv type [" + s.str() + "]");
  }
}

void image_lib::imageMsgToCvCopy(const sensor_msgs::ImageConstPtr& image, cv_bridge::CvImagePtr& cv_ptr) {
  try {
    if (enc::isColor(image->encoding))
      cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);     
    else
      cv_ptr = cv_bridge::toCvCopy(image, enc::MONO8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void image_lib::imageMsgToCvShare(const sensor_msgs::ImageConstPtr& image, cv_bridge::CvImageConstPtr& cv_ptr) {
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

