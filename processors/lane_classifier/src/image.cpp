#include <iostream>
#include "image.h"
#include "profiler.h"

using namespace std;

extern Profiler *profiler;

#undef PROFILER_START_FUN
#undef PROFILER_STOP_FUN
#undef PROFILER_START
#undef PROFILER_STOP

#define PROFILER_START_FUN(p)
#define PROFILER_STOP_FUN(p)
#define PROFILER_START(p,e)
#define PROFILER_STOP(p,e)
namespace image {

  //prototypes
  void getFeaturesLab(const Point3_<uchar>* pt, uchar features[2]);
  void getFeaturesHLS(const Point3_<uchar>* pt, uchar features[2]);
  void getFeaturesHSV(const Point3_<uchar>* pt, uchar features[2]);
  void getFeaturesBGR(const Point3_<uchar>* pt, uchar features[2]);

    #define CV_BGR2BGR -1;
  int getCvType() { return  CV_BGR2HSV; }
  
  void (*getFeatures)(const Point3_<uchar>* pt, uchar features[2]) = getFeaturesHSV;

  void getFeaturesLab(const Point3_<uchar>* pt, uchar features[2]) {
    features[0] = pt->y;
    features[1] = pt->z;
  }

  void getFeaturesHLS(const Point3_<uchar>* pt, uchar features[2]) {
    features[0] = pt->x;
    features[1] = pt->z;
  }

  void getFeaturesHSV(const Point3_<uchar>* pt, uchar features[2]) {
    features[0] = pt->x;
    features[1] = pt->y;
  }

  void getFeaturesBGR(const Point3_<uchar>* pt, uchar features[2]) {
    features[0] = pt->x - pt->y - pt->z;
    features[1] = max(pt->x, max(pt->y, pt->z));
  }

  const Instance makeInstance(uchar imgFeatures[2], int pixelDist, int label) {
    ProfilerEvent *pe = profiler->startEvent("makeInstance");
    double features[] = {imgFeatures[0], imgFeatures[1], pixelDist};
    Instance inst(3, features, label); 
  #ifdef DEBUG
    cout << "New instance: " << features[0] << ", " << features[1] << ", " << features[2] << ", " << label << endl;
  #endif
    profiler->stopEvent(pe);
    return inst;
  }
  
  const Instance makeInstance(uchar imgFeatures[2], int label) {
    ProfilerEvent *pe = profiler->startEvent("makeInstance");
    double features[] = {imgFeatures[0], imgFeatures[1]};
    Instance inst(3, features, label); 
  #ifdef DEBUG
    cout << "New instance: " << features[0] << ", " << features[1] << ", " << label << endl;
  #endif
    profiler->stopEvent(pe);
    return inst;
  }

  void getLightInvariantBGR(const Mat& bgr, Mat& libgr) {

    libgr = bgr;
    Mat raw = bgr.clone(), lab;
    cvtColor(raw, lab, CV_BGR2HSV);
        
    for(int row = 0; row < lab.rows; ++row) {
        //cout << "Row: " << row << endl;
        Point3_<uchar> *p = lab.ptr<Point3_<uchar> > (row);
        for (int col = 0; col < lab.cols; ++col, ++p) {
         //         p->z = p->y;
     //       p->y = 100;
      p->z = 20;
        }
    }
    //Mat imgLi(bgr.size(), CV_8UC3);
        //  int from_to[] = { 0,0, -1,1, 2,2 };
      //    mixChannels( &lab, 1, &imgLi, 1, from_to, 3 );
    cvtColor(lab, libgr, CV_HSV2BGR);
  }

  void maskImageByDensity(Mat& img) {
    PROFILER_START_FUN(profiler);
      int boxHeight = 20;
      int boxWidth = 20;
      int boxHeightHalf = boxHeight >> 2;
      int boxWidthHalf = boxWidth >> 2;
      cv::Size size = img.size();

      int step = 1;
      int maxDensity = boxHeight * boxWidth;
    Mat gray;
    cvtColor(img, gray, CV_BGR2GRAY);
  
      for (int r = 0; r < size.height - boxHeight; r += step) {
      Point3_<uchar> *imgPtr = img.ptr<Point3_<uchar> >(r) + boxHeightHalf;
    for (int c = 0; c < size.width - boxWidth; c += step, imgPtr+=step) {
        Rect rect = Rect(c, r, boxWidth, boxHeight);
        Mat tile = Mat(gray, rect);
        double density = (double)sum(tile).val[0] / 255;
        //place this density in the center of the window
        double pxVal = density / maxDensity; //densities.push_back(std::pair<Point, int>(pt, density));
                //pxVal = 1 / (1 + pow(M_E, -pxVal));
        imgPtr->x = (imgPtr->x * pxVal);
        imgPtr->y = (imgPtr->y * pxVal);
        imgPtr->z = (imgPtr->z * pxVal);
    }
      }
    PROFILER_STOP_FUN(profiler);
    
  }
}
