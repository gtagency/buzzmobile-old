#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "image_lib/image_lib.h"

using namespace image_lib;

std::string outPath  = "./";
unsigned int frameId = 0;

ros::Time nextTime;
ros::Duration capturePeriod;

void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
  ros::Time curTime = ros::Time::now(); 
  if (curTime >= nextTime) {
    cv_bridge::CvImageConstPtr cv_ptr;
    imageMsgToCvShare(image, cv_ptr); 
    frameId++;
    std::ostringstream fname;
    fname << outPath << "/frame" << frameId << ".jpg";
    cv::imwrite(fname.str(), cv_ptr->image);
    nextTime = curTime + capturePeriod;
  }
}

#define DECLARE_PARAM(name, type, defVal)\
  type name;\
  if (!npriv.getParam(#name, name)) {\
    name = defVal;\
  }
int main(int argc, char **argv) {

  ros::init(argc, argv, "frame_capture");
  ros::NodeHandle n, npriv("~");

  DECLARE_PARAM(outPathParam, std::string, "./");
  DECLARE_PARAM(captureFreqParam, int, 1);

  outPath = outPathParam;
  capturePeriod = ros::Duration(1.0/captureFreqParam);
  ros::Subscriber s = n.subscribe<sensor_msgs::Image>("image", 100, imageCallback);

  ros::spin();
  return 0;
}
