#include <ros/ros.h>
#include <image_lib/image_lib.h>
#include <sensor_msgs/Image.h>
#include <core_msgs/WorldRegion.h>
#include <opencv2/opencv.hpp>

ros::Publisher region_pub;
ros::Publisher img_pub;

#define YELLOW_LOW  cv::Scalar(0,150,150)
#define YELLOW_HIGH cv::Scalar(120,255,255)
void imageCallback(const sensor_msgs::Image::ConstPtr& image) {

  cv_bridge::CvImageConstPtr cv_ptr;

  image_lib::imageMsgToCvShare(image, cv_ptr);

  cv::Mat threshImg;
  cv::inRange(cv_ptr->image, YELLOW_LOW, YELLOW_HIGH, threshImg);
   
//  cv::cvtColor(threshImg, threshImg, CV_BGR2GRAY);
//  cv::threshold(threshImg, threshImg, 100, 255, cv::THRESH_BINARY);

  core_msgs::WorldRegion msg;
  msg.width = threshImg.cols;
  msg.height = threshImg.rows;
  msg.resolution = 23; //TODO
  msg.labels.assign(msg.width * msg.height, 1);

  int center = threshImg.cols / 2;
  for (int row = threshImg.rows - 1; row >= 0; --row) {
    int col = center;
    while (col >= 0 && threshImg.at<uchar>(row, col) == 0) {
      col--;
    }
    for (; col >= 0; --col) {
      if (row * msg.width + col >= msg.width * msg.height) {
        std::cout << row * msg.width + col << std::endl;
      }
      msg.labels[row * msg.width + col] = 0; 
    }
  }
/*
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(threshImg, lines, 1, CV_PI/180, 50, 50, 10 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    cv::Vec4i l = lines[i];
    cv::line( threshImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), 3, CV_AA);
  }
  */
  std_msgs::Header header;
  cv_bridge::CvImage markedImage(header, image_lib::getRosType(threshImg.type()), threshImg);

  img_pub.publish(markedImage.toImageMsg());
  std::cout << "Image Published" << std::endl;

  region_pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "yellow_line_detector");
  ros::NodeHandle n;

  region_pub = n.advertise<core_msgs::WorldRegion>("lane_region", 100);
  img_pub = n.advertise<sensor_msgs::Image>("image_region", 100);

  ros::Subscriber s = n.subscribe<sensor_msgs::Image>("image_projected", 100, imageCallback);

  ros::spin();
  return 0;
}

