#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <core_msgs/WorldRegion.h>
#include <image_lib/image_lib.h>

using namespace cv;

ros::Publisher region_img_pub;

void worldRegionCallback(const core_msgs::WorldRegion::ConstPtr& regionMsg) {

  Size size(regionMsg->width, regionMsg->height);
  Mat marked = Mat(size, CV_8UC3);
  int ii = 0;
  for (std::vector<unsigned char>::const_iterator it = regionMsg->labels.begin();
       it != regionMsg->labels.end();
       it++) {
    int row = ii / regionMsg->width;
    int col = ii % regionMsg->height;
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
  cv_bridge::CvImage markedImage(header, image_lib::getRosType(marked.type()), marked);

  region_img_pub.publish(markedImage.toImageMsg());
  std::cout << "Image Published" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_region_image_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<core_msgs::WorldRegion>("region", 1, worldRegionCallback);
  region_img_pub = n.advertise<sensor_msgs::Image>("image", 100);
  ros::spin();
  return 0;
}
