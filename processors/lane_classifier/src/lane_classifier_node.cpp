#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "lane_trainer/LaneInstance.h"
#include "lane_trainer/LaneInstanceArray.h"
#include "image.h"
#include "classifier.h"
#include "score.h"
#include "profiler.h"

using namespace image;
using namespace lane_trainer;

namespace enc = sensor_msgs::image_encodings;

Profiler *profiler = new Profiler();
ros::Publisher driveable_pub;
ros::Publisher marker_pub;

Classifier *c;

void trainingCallback(const LaneInstanceArray::ConstPtr& training) {
  std::vector<Instance> instances;
  for (LaneInstanceArray::_instanceArray_type::const_iterator it = training->instanceArray.begin();
       it != training->instanceArray.end();
       it++) {
    uint8_t features[] = {it->h, it->s};
    instances.push_back(makeInstance(features, it->label));
  }
  c->addInstances(instances);
  //TODO: maybe cull/retire old instances
}

void imageMsgToCvCopy(const sensor_msgs::ImageConstPtr& image, cv_bridge::CvImagePtr& cv_ptr) {
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
}

void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
  
  cv_bridge::CvImagePtr cv_ptr;
  imageMsgToCvCopy(image, cv_ptr); 

  Mat& src = cv_ptr->image;
 //this classifies back into an image...instead we want to classify into a set of points
 // and then publish the points as a lane or something that can be interpreted as a lane
  Mat marked = Mat(src.size(), src.type());
  Mat cvt;
  if (getCvType() >= 0) {
      cvtColor(src, cvt, getCvType());
  } else {
      cvt = src;
  }
  uchar features[2];
  for(int row = 0; row < cvt.rows; ++row) {
    //cout << "Row: " << row << endl;
    Point3_<uchar> *p = cvt.ptr<Point3_<uchar> > (row);
    Point3_<uchar> *sp = marked.ptr<Point3_<uchar> >(row);
    //assumes CV_8UC3 LAB color image, with 3 values per pixel
    for(int col = 0; col < cvt.cols; ++col, ++p, ++sp) {
      getFeatures(p, features);
      const Instance inst = makeInstance(features, -1);
      int label = c->classify(inst); 
      sp->x = 0;
      //cout << countPositives << endl;
      if (label == 1) {
        sp->y = 0xFF;
        sp->z = 0;
      } else if (label == 2) {
        sp->y = 0;
        sp->z = 0xFF;
      } else {
        sp->y = 0;
        sp->z = 0;
      }
    }
  }   
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_classifier");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<LaneInstanceArray>("road_class_train", 1000, trainingCallback);
  sub = n.subscribe<sensor_msgs::Image>("image_projected", 1000, imageCallback);

  int k = 21;
  int data[2] = {0};
  Evaluation eval;
  eval._data  = data;
  eval._score = score::scoreHueAndSat;
  c = new Classifier(new Profiler(), k, eval);

  driveable_pub = n.advertise<sensor_msgs::Image>("image_drivable", 100);
//  marker_pub = n.advertise<sen>("image_drivable", 100);
  ros::spin();
  delete c;
  return 0;
}
