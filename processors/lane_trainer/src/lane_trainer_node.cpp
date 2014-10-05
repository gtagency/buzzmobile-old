#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "lane_trainer/LaneInstanceArray.h"
#include "lane_trainer/LaneInstance.h"
#include <memory>

ros::Publisher pub;

void imageMsgToCvCopy(const sensor_msgs::Image::ConstPtr& image, cv_bridge::CvImagePtr& cv_ptr) {
  try {
//    if (enc::isColor(image->encoding))
//      cv_ptr = cv_bridge::toCvShare(image, enc::BGR8);
//    else
//      cv_ptr = cv_bridge::toCvShare(image, enc::MONO8);
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void generateTrainingSet(const sensor_msgs::Image::ConstPtr& image) {

  cv::Mat projectedImage, sobelImg, threshImg;
  const int SLICE_Y = 2*(projectedImage.rows/3);

  cv_bridge::CvImagePtr cv_ptr;
  imageMsgToCvCopy(image, cv_ptr); 
  projectedImage = cv_ptr->image;

  cv::GaussianBlur(projectedImage, projectedImage, cv::Size(0, 0), 1, 0);

  //cv::cvtColor(projectedImage, projectedImage, CV_BGR2HSV);
  cv::Mat slice(projectedImage, cv::Rect(0, SLICE_Y, projectedImage.cols, 1));
  
  cv::Mat HSVSlice;
  cv::cvtColor(slice, HSVSlice, CV_BGR2HSV);

  cv::inRange(slice, cv::Scalar(0, 150, 200), cv::Scalar(120, 255, 255), threshImg);
  //cv::inRange(slice, cv::Scalar(45, 255, 0), cv::Scalar(60, 135, 255), threshImg);

  cv::Sobel(slice, sobelImg, -1, 1, 0);
  cv::cvtColor(sobelImg, sobelImg, CV_BGR2GRAY);
  cv::threshold(sobelImg, sobelImg, 100, 255, cv::THRESH_BINARY);

  //cv::imshow("Sobel", sobelImg);

  int center = -1;
  std::vector<cv::Vec3b> linePts;

  for (int i = slice.cols-1; i >= 0; --i) {
    if (threshImg.at<uchar>(0, i) == 255) {
      linePts.push_back(HSVSlice.at<cv::Vec3b>(0, i));
      if (center == -1) {
	center = i;
      }
    }
  }

  //std::cout << center << std::endl;
  //std::cout << slice.cols/2 << std::endl;

  std::vector<cv::Vec3b> roadPts;
  bool stop = false;
  for (int i = center + 2; i < slice.cols; ++i) {
    if(!stop) {
      if(sobelImg.at<uchar>(0, i) < 200) {
	roadPts.push_back(HSVSlice.at<cv::Vec3b>(0, i));
	cv::circle(projectedImage, cv::Point(i, SLICE_Y), 1, cv::Scalar(255, 0, 0), -1);
      } else {
	stop = true;
      }
    }
  }

  lane_trainer::LaneInstanceArray lanes;
  for (uint i = 0; i < roadPts.size(); ++i) {
    lane_trainer::LaneInstance lane;
    lane.h = roadPts.at(i).val[0];
    lane.s = roadPts.at(i).val[1];
    lane.label = 1;
    lanes.instanceArray.push_back(lane);
  }

  for (uint i = 0; i < linePts.size(); ++i) {
    lane_trainer::LaneInstance lane;
    lane.h = roadPts.at(i).val[0];
    lane.s = roadPts.at(i).val[1];
    lane.label = 2;
    lanes.instanceArray.push_back(lane);
  }

  pub.publish(lanes);

  ros::spinOnce();

  //std::cout << roadPts.size() << std::endl;

  //cv::imshow("Projection", threshImg);
  //cv::imshow("Yellow Line", yellowLine);
  //cv::imshow("Result", projectedImage);
  
  //cv::waitKey();
}

int main(int argc, char *argv[]) {
  //cv::Mat projectedImage = cv::imread(argv[1]);
  
  ros::init(argc, argv, "lane_trainer_node");
  ros::start();
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image_projected", 1, &generateTrainingSet);
  pub = n.advertise<lane_trainer::LaneInstanceArray>("road_class_train", 1000);
}
