#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_lib/image_lib.h"
#include "planner_astar_bicycle/Path.h"

ros::Publisher img_pub;

//FIXME: no explicit synchronization between path and image...this may or may not cause problems now.  It should definitely be addressed later.
planner_astar_bicycle::Path lastPath;

void plannedPathCallback(const planner_astar_bicycle::Path::ConstPtr& pathMsg) {
  lastPath = *pathMsg;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& imageMsg) {

  cv_bridge::CvImageConstPtr cv_ptr;
  image_lib::imageMsgToCvShare(imageMsg, cv_ptr);

  cv::Mat src = cv_ptr->image;
  
  for (std::vector<geometry_msgs::Pose2D>::iterator it = lastPath.poses.begin();
       it != lastPath.poses.end();
       it++) {
    cv::circle(src, cv::Point(it->x, it->y), 5, cv::Scalar(255, 0, 0), -1);
  }

  cv::imwrite("plan.jpg", src);
  std_msgs::Header header;
  cv_bridge::CvImage outImage(header, image_lib::getRosType(src.type()), src);

  img_pub.publish(outImage.toImageMsg());
  std::cout << "Image Published" << std::endl;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planned_image");
  ros::NodeHandle n;

  ros::Subscriber s = n.subscribe<planner_astar_bicycle::Path>("planned_path", 1, plannedPathCallback);
  ros::Subscriber s2 = n.subscribe<sensor_msgs::Image>("image", 1, imageCallback);

  img_pub = n.advertise<sensor_msgs::Image>("planned_image", 1);
  ros::spin();
  return 0;
}
