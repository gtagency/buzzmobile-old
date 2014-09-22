
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "projection.h"
#include <cmath>

using namespace cv;
using namespace proj;

Mat edges;

int min_threshold = 50;
int max_trackbar = 100;
const char* standard_name = "Standard Hough Lines";
int s_trackbar = max_trackbar;
int p_trackbar = max_trackbar;

void cvShowImageMat(const char *name, Mat& mat) 
{
        IplImage img = mat;
        cvShowImage(name, &img);
}
void flipHorizAndVert(Mat& img) 
{
        int flipMode = -1;
        IplImage ipl = img;
        cvFlip(&ipl, &ipl, flipMode);
}


void getLightInvariantBGR(const Mat& bgr, Mat& libgr) 
{
        libgr = bgr;
        Mat raw = bgr.clone(), lab;
        cvtColor(raw, lab, CV_BGR2Lab);
        Mat imgLi(bgr.size(), CV_8UC3);
        int from_to[] = { -1,0, 1,1, 2,2 };
        mixChannels( &lab, 1, &imgLi, 1, from_to, 3 );
        cvtColor(imgLi, libgr, CV_Lab2BGR);
}

void getHistogram(const Mat &src, MatND &histimg)
{
  Mat hsv_img;
  cvtColor( src, hsv_img, CV_BGR2HSV);
  int h_bins = 50; int s_bins = 60;
  int histSize[] = { h_bins, s_bins };
  float h_ranges[] = { 0, 256 };
  float s_ranges[] = { 0, 180 };
  const float* ranges[] = { h_ranges, s_ranges };
  int channels[] = { 0, 1 };
  
  calcHist( &hsv_img, 1, channels, Mat(), histimg, 2, histSize, ranges, true, false );
  normalize( histimg, histimg, 0, 1, NORM_MINMAX, -1, Mat() );
  namedWindow( "ColorHistogram", WINDOW_AUTOSIZE);
  imshow( "ColorHistogram", histimg);
}


//Hough
void Lane_Hough(Mat &standard_hough )
{
  vector<Vec2f> s_lines;
  cvtColor( edges, standard_hough, COLOR_GRAY2BGR );
  standard_hough = Scalar::all(0);
  HoughLines( edges, s_lines, 1, CV_PI/180, min_threshold + s_trackbar - 10, 0, 0 );
  std::cout << "Number of Lines " << s_lines.size() << std::endl;
  float t_prev = 0.0;
  float r_prev = 0.0; 
  std::vector<Point> point_x;
  for( size_t i = 0; i < s_lines.size(); i++ )
     {
      float r = s_lines[i][0], t = s_lines[i][1];           
	{
	  double cos_t = cos(t), sin_t = sin(t);
	  double x0 = r*cos_t, y0 = r*sin_t;
	  double alpha = 1000;
	  Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
	  Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
	  if(pt1.x > -500)
	    {    
	    line( standard_hough, pt1, pt2, Scalar(255,255,255), 3, CV_AA);	    	 
	    }
	}
      t_prev = t;
      r_prev = r;
     }
imshow( standard_name, standard_hough );
imwrite("Hough.png", standard_hough);
}


void lane_detect(const Mat &proj, Mat &hough_result, Mat &crop)
{
       blur(proj, edges, Size(3,3));
       Canny(edges, edges, 50, 200, 3 );       
       char thresh_label[50];
       sprintf( thresh_label, "Thres: %d + input", min_threshold );
       namedWindow( standard_name, WINDOW_AUTOSIZE );
       // createTrackbar( thresh_label, standard_name, &s_trackbar, max_trackbar, Lane_Hough);
       Lane_Hough(hough_result);
       Rect roi;
       roi.x = 165;
       roi.y = 400;
       roi.width = 500;
       roi.height = 80;
       crop = hough_result(roi);
       cv::Mat i_crop = proj(roi);
       namedWindow("ROI", WINDOW_AUTOSIZE);
       //       imshow("ROI", i_crop);
       imwrite("ROI.png", i_crop);       
       namedWindow("Lines" , WINDOW_AUTOSIZE);
       imwrite("Lines.png", crop);
      
       Mat dst, dst_norm, dst_norm_scaled;
       dst = Mat::zeros( src.size(), CV_32FC1 );
       /// Detector parameters
       int blockSize = 2;
       int apertureSize = 3;
       double k = 0.04;
       /// Detecting corners
       cornerHarris( crop, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
       /// Normalizing
       normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
       convertScaleAbs( dst_norm, dst_norm_scaled );
       /// Drawing a circle around corners
       for( int j = 0; j < dst_norm.rows ; j++ )
	 { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }
       /// Showing the result
       namedWindow("Corners", CV_WINDOW_AUTOSIZE );
       imshow( "Corners", dst_norm_scaled );

       
}

void label_frame(Mat &crop, std::vector<bool> &labels)
{
 
}


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
  
  cvNamedWindow("ReadFrame", 0);
  cvNamedWindow("IPM", 0);
  cvNamedWindow("Gray", 0);
  cvNamedWindow("Invariant", 0);
  int flipMode = -1;
  Mat imgA, imgGrayA;
  Mat frame;
  Mat histimg;

  cap >> imgA;
  imwrite("Input.png", imgA);
  cvtColor(imgA, imgGrayA, CV_BGR2GRAY);

  // while(true) 
  //  {
    Mat imgLi, proj, hough_result, drivable, crop;
    std::vector<bool> labels;
    getHistogram(imgA, histimg);
    getLightInvariantBGR(imgA, imgLi);
    groundTransformProj(imgGrayA, proj);
    cvShowImageMat( "ReadFrame", imgA);
    cvShowImageMat( "Gray", imgGrayA);
    cvShowImageMat( "Invariant", imgLi);
    cvShowImageMat( "IPM", proj);
    imwrite("IPM.png", proj);
    lane_detect(proj, hough_result, crop);     
    label_frame(crop, labels);
    // }
    waitKey(0);

    return 0;

} 
/*end-main*/

