#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "lane_classifier/LaneLabels.h"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

ros::Publisher labels_img_pub;

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

void laneLabelCallback(const lane_classifier::LaneLabelsConstPtr& labelMsg) {

  Size size(labelMsg->width, labelMsg->height);
  Mat marked = Mat(size, CV_8UC3);
  int ii = 0;
  for (std::vector<unsigned char>::const_iterator it = labelMsg->labels.begin();
       it != labelMsg->labels.end();
       it++) {
    int row = ii / labelMsg->width;
    int col = ii % labelMsg->height;
    ii++;
//  std::cout << "Marking image: " << it->row << "," << it->col << std::endl;
    Point3_<uchar> *sp = &marked.ptr<Point3_<uchar> >(row)[col];
    //assumes CV_8UC3 color image, with 3 values per pixel
    sp->x = 0;
    //cout << countPositives << endl;
//    std::cout << (int)*it << ", " << (*it == 1 ? "True" : "False") << ", " << (*it == 2 ? "True" : "False") << std::endl;

    if (*it == 1) {
      sp->y = 0xFF;
      sp->z = 0;
    } else if (*it == 2) {
      sp->y = 0;
      sp->z = 0xFF;
    } else {
      sp->y = 0;
      sp->z = 0;
    }
  } 
  std_msgs::Header header;
  cv_bridge::CvImage markedImage(header, getRosType(marked.type()), marked);

  labels_img_pub.publish(markedImage.toImageMsg());
  std::cout << "Image Published" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_labels_image_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<lane_classifier::LaneLabels>("lane_labels", 1, laneLabelCallback);
  labels_img_pub = n.advertise<sensor_msgs::Image>("image_driveable", 100);
  ros::spin();
  return 0;
}
