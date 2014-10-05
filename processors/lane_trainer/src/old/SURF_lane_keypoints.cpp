

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace cv;
void readme();
int main( int argc, char** argv )
{
  
  VideoCapture cap("road.mov");
  if(!cap.isOpened())
    {
      std::cout << "Cannot open file "<< std::endl;
      return -1;
    }

  double fps = cap.get(CV_CAP_PROP_FPS);
  std::cout << "Frame per seconds " << fps << std::endl;
  namedWindow("RoadVideo", CV_WINDOW_AUTOSIZE);

  //for(;;)
  // {
       Mat frame;
       cap >> frame;
       std::vector<KeyPoint> keypoints;
       SurfFeatureDetector detector(400);
       detector.detect(frame, keypoints);
       Mat img_keypoints;
       drawKeypoints( frame, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );       
       imshow("SURFKeypoints", img_keypoints);
       imwrite("SURFKeypoints.png", img_keypoints);
       // }

  waitKey(0);
  return 0;
}

void readme()
{ std::cout << " Usage: ./detector" << std::endl; }
