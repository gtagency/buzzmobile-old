
#include <iostream>
#include "classifier.h"
#include "math.h"
#include <cassert>
#include "opencv/cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "profiler.h"
#include "projection.h"
#include "image.h"
#include "score.h"

using namespace cv;
using namespace std;
using namespace proj;
using namespace image;
using namespace score;


//#define DEBUG

Profiler *profiler;

void classifyAndMark(const Mat& src, Mat& marked, Classifier& c) {

    int inLaneStart = 389;
    
    marked = Mat(src.size(), src.type());
    Mat lab;
    if (getCvType() >= 0) {
        cvtColor(src, lab, getCvType());
    } else {
        lab = src;
    }
    uchar features[2];
    for(int row = 0; row < lab.rows; ++row) {
        //cout << "Row: " << row << endl;
        Point3_<uchar> *p = lab.ptr<Point3_<uchar> > (row);
        Point3_<uchar> *sp = marked.ptr<Point3_<uchar> >(row);
        //assumes CV_8UC3 LAB color image, with 3 values per pixel
        for(int col = 0; col < lab.cols; ++col, ++p, ++sp) {
            //throw away the L
//            cout << "Col: " << col << endl;

            getFeatures(p, features);
            const Instance inst = makeInstance(features, abs(col - inLaneStart), -1);
            int label = c.classify(inst);    
            if (row == 2) {
                //cout << "Col " << col << ": " << (int)p->x << ", " << (int)p->y << ", " << (int)p->z << ", " << label << endl;
            }
            sp->x = 0;
            //cout << countPositives << endl;
            if (label) {
                sp->y = 0xFF;
                sp->z = 0;
            } else {
                sp->y = 0;
                sp->z = 0;
            }
        }
    }   

}

struct Labeler {
	int inLaneStart;
	int inLaneEnd;

    Labeler() : inLaneStart(339), inLaneEnd(431) {}
    int operator()(int row, int col) {
        return col >= inLaneStart && col <= inLaneEnd ? 1 : 0; 
    }
};

int main(int argv, const char *argc[]) {

    std::vector<Instance> instances;

    Mat src = imread("frame5.png");
  
    Mat proj;
//    groundTransformProj(src, proj);
  //  src = proj;
    Mat li;
    //getLightInvariantBGR(src, li);
     
    //src = li; 
    profiler = new Profiler();
    Mat lab;
    //imwrite("movroadnext-proj.png", src);
    if (getCvType() >= 0) {
        cvtColor(src, lab, getCvType());
    } else {
        lab = src;
    }
    //take bottom 10% for training 
    cout << "Training" << endl;
    PROFILER_START(profiler, addInstances);
    int data[3] = {0};
    Labeler labeler;

    instances = extractInstances(lab, data, labeler);
    cout << "Building classifier" << endl;
    //cout << maxA << " " << maxB << " " << maxC << endl;
    int k = 21;
    Evaluation eval;
    eval._data  = data;
    eval._score = scoreHueAndSat;
    Classifier c(profiler, k, eval);
    c.addInstances(instances);
    addInstances__pe;
    PROFILER_STOP(profiler, addInstances);

    cout << "Testing" << endl;
    Mat marked;
    PROFILER_START(profiler, test_classify);
    src = imread("frame6.png");
    classifyAndMark(src, marked, c);
	maskImageByDensity(marked);
    PROFILER_STOP(profiler, test_classify);
   // time_t endTime;
   // time(&endTime);  
   // double seconds = difftime(endTime, startTime);
    imwrite("frame6-out.png", marked);
    profiler->printResults();
    
//    src = imread("movroad.png");
  //  groundTransformProj(src, proj);
    //classifyAndMark(proj, marked, c);
//    imwrite("movroad-proj.png", proj);
  //  imwrite("movroad-out.png", marked);

    delete profiler;    
}
