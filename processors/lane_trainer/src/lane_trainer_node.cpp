#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "lane_trainer/LaneInstanceArray.h"
#include "lane_trainer/LaneInstance.h"
#include <memory>

ros::Publisher pub;

namespace enc = sensor_msgs::image_encodings;

void imageMsgToCvCopy(const sensor_msgs::Image::ConstPtr& image, cv_bridge::CvImageConstPtr& cv_ptr) {
  try {
    if (enc::isColor(image->encoding))
      cv_ptr = cv_bridge::toCvShare(image, enc::BGR8);
    else
      cv_ptr = cv_bridge::toCvShare(image, enc::MONO8);
//    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

struct Vec3bCompare {
public:
  bool operator()(const cv::Vec3b &a, const cv::Vec3b &b) const {
    return a[0] < b[0] && a[1] < b[1] && a[2] < b[2];
  }
};

void addPointsToInstanceArray(const std::set<cv::Vec3b,Vec3bCompare>& pts, int label, lane_trainer::LaneInstanceArray& lanes) {
  for (std::set<cv::Vec3b>::iterator it = pts.begin();
       it != pts.end();
       ++it) {
    lane_trainer::LaneInstance lane;
    lane.h = it->val[0];
    lane.s = it->val[1];
    lane.label = label;
    lanes.instanceArray.push_back(lane);
  }
}

#define YELLOW_LOW  cv::Scalar(0,150,200)
#define YELLOW_HIGH cv::Scalar(120,255,255)
#define GREEN_LOW   cv::Scalar(70,80,40)
#define GREEN_HIGH  cv::Scalar(220,255,135)
void generateTrainingSet(const sensor_msgs::Image::ConstPtr& image) {

 //std::cout << "Image received" << std::endl;
  cv::Mat projectedImage, sobelImg, threshImg;

  cv_bridge::CvImageConstPtr cv_ptr;
  imageMsgToCvCopy(image, cv_ptr); 
  projectedImage = cv_ptr->image;

  const int SLICE_Y = 1*(projectedImage.rows/6);
  cv::Mat gauss, median;
//  cv::GaussianBlur(projectedImage, gauss, cv::Size(0, 0), 1, 0);
//  cv::imshow("gaussianBlur", gauss);
  cv::medianBlur(projectedImage, median, 7);

  cv::imshow("medianBlur", median);
 
  projectedImage = median; //gauss; 
  cv::Mat slice(projectedImage, cv::Rect(0, SLICE_Y, projectedImage.cols, 1));
  
  cv::Mat HSVSlice;
  cv::cvtColor(slice, HSVSlice, CV_BGR2HSV);

  cv::inRange(slice, GREEN_LOW, GREEN_HIGH, threshImg);
  //cv::inRange(slice, cv::Scalar(45, 255, 0), cv::Scalar(60, 135, 255), threshImg);

  slice.convertTo(slice, CV_16S);
  cv::Sobel(slice, sobelImg, -1, 1, 0);
 // cv::Mat detectedEdges;

 // sobelImg = cv::abs(sobelImg);
  sobelImg.convertTo(sobelImg, CV_8U);

 // int threshold = 50;
//  cv::Canny(slice, detectedEdges, threshold, threshold * 3, 3);
 // sobelImg = cv::Scalar::all(0);
 // slice.copyTo(sobelImg, detectedEdges);

  cv::cvtColor(sobelImg, sobelImg, CV_BGR2GRAY);
  cv::threshold(sobelImg, sobelImg, 100, 255, cv::THRESH_BINARY);

  //sobelImg = sobelImg + threshImg;
  //cv::imshow("Sobel", sobelImg);

  int leftEdge = -1;
  int rightEdge = -1;
  std::set<cv::Vec3b, Vec3bCompare> linePts;
  std::set<cv::Vec3b, Vec3bCompare> notRoadPts;
  std::set<cv::Vec3b, Vec3bCompare> roadPts;

//  for (int i = leftEdge; i < slice.cols; ++i) {
  //
  int center = 2*slice.cols /3;
  for (int i = center; i >= 0; --i) {
    if (threshImg.at<uchar>(0, i) == 255) {
  	  leftEdge = i;
    } else if (leftEdge != -1) {
      break;
    }
  }
  for (int i = leftEdge; i < slice.cols; ++i) {
    if (sobelImg.at<uchar>(0, i) == 255 || threshImg.at<uchar>(0, i) == 255) {
      linePts.insert(HSVSlice.at<cv::Vec3b>(0, i));
      cv::circle(projectedImage, cv::Point(i, SLICE_Y), 1, cv::Scalar(0, 255, 0), -1);
  	  leftEdge = i;
    } else {
      break;
    }
  }

  for (int i = leftEdge + 1; i < slice.cols; ++i) {
    if (sobelImg.at<uchar>(0, i) == 255 || threshImg.at<uchar>(0, i) == 255) {
      cv::circle(projectedImage, cv::Point(i, SLICE_Y), 1, cv::Scalar(0, 255, 0), -1);
      rightEdge = i;
    } else {
      if (rightEdge == -1) {
        roadPts.insert(HSVSlice.at<cv::Vec3b>(0, i));
      	cv::circle(projectedImage, cv::Point(i, SLICE_Y), 1, cv::Scalar(255, 0, 0), -1);
      } else {
        notRoadPts.insert(HSVSlice.at<cv::Vec3b>(0, i));
      	cv::circle(projectedImage, cv::Point(i, SLICE_Y), 1, cv::Scalar(0, 0, 255), -1);
      }
    }
  }
  std::cout << center << std::endl;
  std::cout << leftEdge << std::endl;
  std::cout << rightEdge << std::endl;
  std::cout << slice.cols/2 << std::endl;
/*
  bool stop = false;
  for (int i = center + 2; i < slice.cols; ++i) {
    if(!stop) {
      if(sobelImg.at<uchar>(0, i) < 200) {
	      roadPts.insert(HSVSlice.at<cv::Vec3b>(0, i));
      	cv::circle(projectedImage, cv::Point(i, SLICE_Y), 1, cv::Scalar(255, 0, 0), -1);
      } else {
      	stop = true;
      }
    }
  }
*/

  lane_trainer::LaneInstanceArray lanes;
  addPointsToInstanceArray(notRoadPts, 0, lanes);
  addPointsToInstanceArray(roadPts,    1, lanes);
  addPointsToInstanceArray(linePts,    2, lanes);

  pub.publish(lanes);

  //std::cout << roadPts.size() << std::endl;

  cv::imshow("threshImg", threshImg);
  cv::imshow("sobelImg", sobelImg);
  
  cv::imshow("projectedImage", projectedImage);
  cv::waitKey();
}

int main(int argc, char *argv[]) {
  //cv::Mat projectedImage = cv::imread(argv[1]);
  
  std::cout << "Initializing Lane Trainer" << std::endl;
  ros::init(argc, argv, "lane_trainer_node");
  ros::start();
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image_projected", 1, generateTrainingSet);
  pub = n.advertise<lane_trainer::LaneInstanceArray>("road_class_train", 1000);

  std::cout << "Lane trainer started." << std::endl;
  ros::spin();
  return 0;
}
