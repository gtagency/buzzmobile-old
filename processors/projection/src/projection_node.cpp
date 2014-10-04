
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "projection.h"

ros::Publisher proj_pub;

namespace enc = sensor_msgs::image_encodings;

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

//NOTE: values copied from Nick's original code, hand tuned to Eleanor.
proj::ProjectionParams params(800,800,120.0,120.0,21.0/12.0,1000.0,M_PI_2+.311);
void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
//    if (enc::isColor(image->encoding))
//      cv_ptr = cv_bridge::toCvShare(image, enc::BGR8);
//    else
//      cv_ptr = cv_bridge::toCvShare(image, enc::MONO8);
    cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat proj;
  proj::groundTransformProj(cv_ptr->image, params, proj);
  //JOSH INSERT CODE HERE
  std_msgs::Header header;
  cv_bridge::CvImage projImage(header, getRosType(proj.type()), proj);

  proj_pub.publish(projImage.toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "projection");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image_raw", 1000, imageCallback);
  proj_pub = n.advertise<sensor_msgs::Image>("image_projected", 100);

  ros::spin();
  return 0;
}
