
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "balance.hpp"

#include "projection.h"
#include "image_lib/image_lib.h" //include cv_bridge

using namespace image_lib;

ros::Publisher proj_pub;

//NOTE: values copied from Nick's original code, hand tuned to Eleanor.
proj::ProjectionParams *params = NULL;
void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
  cv_bridge::CvImageConstPtr cv_ptr;
  imageMsgToCvShare(image, cv_ptr);
  cv::Mat proj;
  proj::groundTransformProj(cv_ptr->image, *params, proj);
  balance::grayWorld(proj, proj);

  std_msgs::Header header;
  cv_bridge::CvImage projImage(header, getRosType(proj.type()), proj);

  proj_pub.publish(projImage.toImageMsg());
}

#define DECLARE_PARAM(name, type, defVal)\
  type name;\
  if (!n.getParam(#name, name)) {\
    name = defVal;\
  }

proj::ProjectionParams getParams() {

  ros::NodeHandle n("~");
  DECLARE_PARAM(output_x_res, int, 800);
  DECLARE_PARAM(output_y_res, int, 800);
  DECLARE_PARAM(ground_x_dim, double, 120.0);
  DECLARE_PARAM(ground_y_dim, double, 120.0);
  DECLARE_PARAM(ground_z_dim, double, 21.0/12.0);
  DECLARE_PARAM(camera_scale, double, 1000.0);
  DECLARE_PARAM(camera_pitch, double, M_PI_2);

  return proj::ProjectionParams(output_x_res,output_y_res,ground_x_dim,ground_y_dim,ground_z_dim,camera_scale,camera_pitch);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "projection");
  ros::NodeHandle n;

  proj::ProjectionParams localParams = getParams();
  params = &localParams;
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image_raw", 1000, imageCallback);
  proj_pub = n.advertise<sensor_msgs::Image>("image_projected", 100);

  ros::spin();
  return 0;
}
