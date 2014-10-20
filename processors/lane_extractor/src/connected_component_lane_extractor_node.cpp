#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "lane_classifier/LaneLabels.h"
#include "uf.h"
#include <opencv2/opencv.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

ros::Publisher labels_ex_pub;
ros::Publisher comp_img_pub;

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

void laneLabelsCallback(const lane_classifier::LaneLabelsConstPtr& labelMsg) {

  Size size(labelMsg->width, labelMsg->height);
  Mat marked = Mat(size, CV_8UC1);
  int ii = 0;
  int onVal = 255; //value to indicate pixel is on in binary image

  UF uf(labelMsg->width*labelMsg->height);
  int driveableLabel = 1;
  for (std::vector<unsigned char>::const_iterator it = labelMsg->labels.begin();
       it != labelMsg->labels.end();
       it++) {
    int row = ii / labelMsg->width;
    int col = ii % labelMsg->height;

    ii++;
    marked.ptr<uchar>(row)[col] = *it;
    if (*it == driveableLabel) {
      int set = uf.find(ii);
      if (row > 0 && marked.ptr<uchar>(row-1)[col] == driveableLabel) {
        uf.merge(uf.find(ii - labelMsg->width), set);
      }
      if (col > 0 && marked.ptr<uchar>(row)[col-1] == driveableLabel) {
        uf.merge(uf.find(ii - 1), set);
      }
    }
  }
  int outputSize = labelMsg->width * labelMsg->height;
  int *output = new int[outputSize];

  Mat compo = Mat(size, CV_8UC1);
  int compCount = uf.get(uf.max(), output, outputSize);
  for (int ii = 0; ii < compCount; ii++) {
    int row = output[ii] / labelMsg->width;
    int col = output[ii] % labelMsg->height;

    compo.ptr<uchar>(row)[col] = onVal; 
  } 
  delete [] output;
  std::vector<std::vector<Point> > contours;
  findContours(compo, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  std::vector<Point> compPoints = contours.front();
  Mat stuff = Mat::zeros(size, CV_8UC1);
  drawContours(stuff, contours, 0, Scalar(onVal), CV_FILLED);
  std_msgs::Header header;
  cv_bridge::CvImage compImage(header, getRosType(stuff.type()),stuff);

  comp_img_pub.publish(compImage.toImageMsg());

  lane_classifier::LaneLabels newLabelMsg;

  std::cout << " Doin stuff! " << std::endl;
  newLabelMsg.width = labelMsg->width;
  newLabelMsg.height = labelMsg->height;
  newLabelMsg.labels.reserve(labelMsg->width * labelMsg->height);
  for (int row = 0; row < labelMsg->height; row++) {
    for (int col = 0; col < labelMsg->width; col++) {
     newLabelMsg.labels.push_back(stuff.ptr<uchar>(row)[col] == onVal ? 1 : 0);
    }
  }
  /*
  for (int ii = 0; ii < labelMsg->width * labelMsg->height; ii++) {
    newLabelMsg.labels.push_back(0);
  }
  
  std::cout << newLabelMsg.labels.size() << std::endl;
  for (std::vector<Point>::iterator it = compPoints.begin();
       it != compPoints.end();
       it++) {
    int inx = it->y * labelMsg->width + it->x;
    newLabelMsg.labels[inx] = 1;
  }*/

  std::cout << newLabelMsg.labels.size() << std::endl;
//  cv::waitKey();
  labels_ex_pub.publish(newLabelMsg);
  std::cout << "Image Published" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_labels_image_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<lane_classifier::LaneLabels>("lane_labels", 1, laneLabelsCallback);
  labels_ex_pub = n.advertise<lane_classifier::LaneLabels>("lane_labels_ex", 100);
  comp_img_pub = n.advertise<sensor_msgs::Image>("component", 100);
  ros::spin();
  return 0;
}
