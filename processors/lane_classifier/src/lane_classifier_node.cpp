#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "lane_trainer/LaneInstance.h"
#include "lane_trainer/LaneInstanceArray.h"
#include "lane_classifier/LaneLabels.h"
#include "image.h"
#include "classifier.h"
#include "score.h"
#include "profiler.h"

using namespace image;
using namespace lane_trainer;

namespace enc = sensor_msgs::image_encodings;

ros::Publisher labels_pub;
ros::Publisher marker_pub;

Classifier *c = NULL;

class PointInstance : public Instance {
private:
  int row;
  int col;
public:
  PointInstance(int row, int col, Instance& inst)
    : Instance(inst), row(row), col(col) {}

  int getCol() const { return this->col; }
  int getRow() const { return this->row; }
};

void trainingCallback(const LaneInstanceArray::ConstPtr& training) {
  std::cout << (long)c << std::endl;
  std::vector<Instance> instances;
  for (LaneInstanceArray::_instanceArray_type::const_iterator it = training->instanceArray.begin();
       it != training->instanceArray.end();
       it++) {
    uint8_t features[] = {it->h, it->s};
    instances.push_back(makeInstance(features, it->label));
  }
//  c->pruneInstances(1);
  c->addInstances(instances);
  //TODO: maybe cull/retire old instances
}

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

  // Only proceed if we're initialized
  if (!c->isInitialized()) {
    std::cout << "Classifier not initialized!" << std::endl;
    return;
  } 

  PROFILER_START_FUN(profiler);
  std::cout << "Image received." << std::endl;
  cv_bridge::CvImageConstPtr cv_ptr;
  imageMsgToCvShare(image, cv_ptr); 

  Mat src = cv_ptr->image;
 //this classifies back into an image...instead we want to classify into a set of points
 // and then publish the points as a lane or something that can be interpreted as a lane
  Mat marked = Mat(src.size(), src.type());
  Mat cvt;
  if (getCvType() >= 0) {
      std::cout << "Converting type" << std::endl;
      cvtColor(src, cvt, getCvType());
  } else {
      std::cout << "Not converting type" << std::endl;
      cvt = src;
  }
  std::vector<Instance> toClassify;
  toClassify.reserve(800*800);

  std::cout << cvt.type() << std::endl;
  uchar features[2] = {0};
  for(int row = 0; row < cvt.rows; ++row) {
    //cout << "Row: " << row << endl;
    Point3_<uchar> *p = cvt.ptr<Point3_<uchar> > (row);
    int end = cvt.cols - 1;
    //scan for the end, which is the first color pixel from the right
    
    for(; end >= 0; --end) {
      if (p[end].x != 0 || p[end].y != 0 || p[end].z != 0) {
        break;
      }
    }


    //scan for the start, which is the first color pixel from the left
    int col = 0;
    for(; col <= end; ++p, ++col) {
      if (p[col].x != 0 || p[col].y != 0 || p[col].z != 0) {
        break;
      }
    }
    //build instances for all pixels in the middle
    for(; col <= end; ++p, ++col) {
      getFeatures(p, features);
      Instance inst = makeInstance(features, -1);
      inst.row = row;
      inst.col = col;
      toClassify.push_back(inst);
    }
  }
  std::vector<int>labels = c->classifyAll(toClassify);
  std::vector<Instance>::iterator it = toClassify.begin();
  std::vector<int>::iterator label = labels.begin();

  lane_classifier::LaneLabels labelsMsg;
  labelsMsg.width = marked.size().width;
  labelsMsg.height= marked.size().height;
  labelsMsg.labels.resize(labelsMsg.width * labelsMsg.height, 0);
  //row-order labels, which is what we're putting out already
//  std::swap(labelsMsg.labels, labels);
  for (std::vector<Instance>::iterator it = toClassify.begin();
       it != toClassify.end();
       it++) {
    labelsMsg.labels.at(it->row * labelsMsg.width + it->col) = it->label;
  }
  labels_pub.publish(labelsMsg);
  //markedImage.toImageMsg());
/*
  for (;
       it != toClassify.end();
       it++, label++) {

//  std::cout << "Marking image: " << it->row << "," << it->col << std::endl;
    Point3_<uchar> *sp = &marked.ptr<Point3_<uchar> >(it->row)[it->col];
    //assumes CV_8UC3 color image, with 3 values per pixel
    sp->x = 0;
    //cout << countPositives << endl;
    if (*label == 1) {
      sp->y = 0xFF;
      sp->z = 0;
    } else if (*label == 2) {
      sp->y = 0;
      sp->z = 0xFF;
    } else {
      sp->y = 0;
      sp->z = 0;
    }
  } 
  std_msgs::Header header;
  cv_bridge::CvImage markedImage(header, getRosType(marked.type()), marked);

  driveable_pub.publish(markedImage.toImageMsg());
  */
  std::cout << "Finished" << std::endl;
  PROFILER_STOP_FUN(profiler);
  profiler.printResults();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_classifier");
  ros::NodeHandle n;

  int k = 21;
  int data[2] = {0};
  Evaluation eval;
  eval._data  = data;
  eval._score = score::scoreHueAndSat;
  c = new ClusterBasedClassifier(); //(k, eval);

  std::cout << "Lane classifier starting." << std::endl;
  ros::Subscriber sub = n.subscribe<LaneInstanceArray>("road_class_train", 1000, trainingCallback);
  ros::Subscriber sub2 = n.subscribe<sensor_msgs::Image>("image_projected", 1, imageCallback);
  //driveable_pub = n.advertise<sensor_msgs::Image>("image_driveable", 100);
  labels_pub = n.advertise<lane_classifier::LaneLabels>("lane_labels", 100);
//  marker_pub = n.advertise<sen>("image_drivable", 100);
  std::cout << "Lane classifier started." << std::endl;
  ros::spin();
  delete c;
  return 0;
}
