#include <ros/ros.h>

#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/Image.h>
#include "image_lib/image_lib.h"

#include "projection.h"

using namespace cv;
using namespace proj;
using namespace image_lib;

const int camera_scale_max = 2000; //unknown units
const int camera_pitch_coarse_max = 180; //0.1 degrees
const int camera_pitch_fine_max = 100; //0.1 degrees
int camera_scale = 1000;
int camera_pitch_coarse = 90; // pi/2
int camera_pitch_fine = 0; // pi/2

int output_res = 400;
//FIXME: magic numbers for the big car
ProjectionParams params(output_res, output_res, 120, 120, 3.5365, 1000.0, M_PI_2);

cv_bridge::CvImagePtr GlobalCvPtr;

void projectAndDisplay() {
  Mat proj = Mat::zeros(400, 400, CV_8UC3);
  if (GlobalCvPtr) {
    groundTransformProj(GlobalCvPtr->image, params, proj);
  }
  imshow( "Projection", proj );
  waitKey(3);
}
void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
  imageMsgToCvCopy(image, GlobalCvPtr);
  projectAndDisplay();
}

void on_trackbar( int, void* ) {
  params.camera_pitch = (camera_pitch_coarse + (camera_pitch_fine / 100.0)) * M_PI / 180.0;
  params.camera_scale = camera_scale;
  projectAndDisplay();
}

int main( int argc, char** argv ) {

  ros::init(argc, argv, "calibration_gui");
  ros::NodeHandle n;

  ros::Subscriber s = n.subscribe<sensor_msgs::Image>("image_projected", 100, imageCallback);

  /// Create Windows
  namedWindow("Projection", 1);

  createTrackbar( "Camera Pitch (coarse)", "Projection", &camera_pitch_coarse, camera_pitch_coarse_max, on_trackbar );
  createTrackbar( "Camera Pitch (fine)", "Projection", &camera_pitch_fine, camera_pitch_fine_max, on_trackbar );
  createTrackbar( "Camera Scale", "Projection", &camera_scale, camera_scale_max, on_trackbar );

  on_trackbar( 0, 0 );

  ros::spin();
  return 0;
}
