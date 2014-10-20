#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "lane_classifier/LaneLabels.h"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

ros::Publisher labels_ex_pub;

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
  Mat marked = Mat(size, CV_8U1);
  int ii = 0;
  for (std::vector<unsigned char>::const_iterator it = labelMsg->labels.begin();
       it != labelMsg->labels.end();
       it++) {
    int row = ii / labelMsg->width;
    int col = ii % labelMsg->height;

    ii++;
    marked.ptr<uchar>(row)[col] = *it;

  }
  labels_ex_pub.publish(markedImage.toImageMsg());
  std::cout << "Image Published" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_labels_image_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<lane_classifier::LaneLabels>("lane_labels", 1, laneLabelsCallback);
  labels_ex_pub = n.advertise<lane_classifier::LaneLabels>("lane_labels_ex", 100);
  ros::spin();
  return 0;
}
