
#ifndef __CV_BRIDGE_H
#define __CV_BRIDGE_H

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace image_lib {

  const std::string& getRosType(int cvType);

  void imageMsgToCvCopy(const sensor_msgs::ImageConstPtr& image, cv_bridge::CvImagePtr& cv_ptr);
  void imageMsgToCvShare(const sensor_msgs::ImageConstPtr& image, cv_bridge::CvImageConstPtr& cv_ptr);
}

#endif //_CV_BRIDGE_H
