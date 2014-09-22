
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

    int inLaneStart = src.cols / 2;
    
    marked = Mat(src.size(), src.type());
    Mat lab;
    cvtColor(src, lab, getCvType());
    uchar features[2];
    for(int row = 0; row < lab.rows; ++row) {
        //cout << "Row: " << row << endl;
        Point3_<uchar> *p = lab.ptr<Point3_<uchar> > (row);
        Point3_<uchar> *sp = marked.ptr<Point3_<uchar> >(row);
        //assumes CV_8UC3 LAB color image, with 3 values per pixel
        for(int col = 0; col < lab.cols; ++col, p++, sp++) {
            //throw away the L
//            cout << "Col: " << col << endl;

            getFeatures(p, features);
            const Instance inst = makeInstance(features, abs(col - inLaneStart), -1);
            int label = c.classify(inst);    
            sp->x = 0;
            //cout << countPositives << endl;
            if (label) {
                sp->y = 0xFF;
                sp->z = 0;
            } else {
                sp->y = 0;
                sp->z = 0; //0xFF;
            }
        }
    }   

}

void cvShowImageMat(const char *name, Mat& mat) {
	IplImage img = mat;
	cvShowImage(name, &img);
}

struct Labeler {
    int inLaneStart;
    int inLaneEnd;

    Labeler() : inLaneStart(335), inLaneEnd(451) {}
    int operator()(int row, int col) {
        return col >= inLaneStart && col <= inLaneEnd ? 1 : 0; 
    }
};

int main(int argc, const char *argv[]) {

    std::vector<Instance> instances;

    //Mat src = imread("testroadcrop.png");
   
    char resolved_path[256];
    realpath(argv[1], resolved_path);
    printf("Opening movie file \n%s\n",resolved_path);
	
    // Load two images and allocate other structures
    VideoCapture cap(resolved_path); // open the default camera
    // VideoCapture cap("/Users/theJenix/Development/opencv_experiments/opencv_lk/recording1.mov"); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
        return -1; 

    Mat src;
    int skip = 5;
    cap.set(CV_CAP_PROP_POS_FRAMES, skip);  
    //imwrite("calib.png", src); 
    int maxFrame = 120 - 1;

    profiler = new Profiler();
    Mat lab;
    Mat proj;
    cap >> src;
    groundTransformProj(src, proj);
    src = proj;
    //GaussianBlur(src, src, Size(3, 3), 0.5);
    cvtColor(src, lab, getCvType());

    cvNamedWindow( "Frame", 0 );
    cvNamedWindow( "Projection", 0 );
    cvNamedWindow( "Cost Map", 0 );

    //take bottom 10% for training 
    cout << "Training" << endl;
    ProfilerEvent *pe = profiler->startEvent("trainingSet");
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

    profiler->stopEvent(pe);
    cout << "Testing" << endl;
    int ii = 0;
    int trainingFreq = 2; //2 * 30;
	int pruningFreq = 5;
	int pruningThresh = 1 + (k >> 2);
    int framesPerSec = 1;
    //double updateFreq = 0.5;
    int updateFreq = 1;
    while(--maxFrame >= 0) {
        cap >> src;
        //only process frames as fast as the algorithm can work
        if (ii % (int)(framesPerSec * updateFreq) == 0) {
            Mat proj;
            groundTransformProj(src, proj);
            
            if (ii % trainingFreq == 0) {
                cout << "Retraining" << endl;
                instances = extractInstances(lab, data, labeler); 
                c.addInstances(instances);
            }
            Mat marked;
            classifyAndMark(proj, marked, c);
            maskImageByDensity(marked);
            if (ii % pruningFreq == 0) {
                cout << "Pruning" << endl;
                int left = c.pruneInstances(pruningThresh);
                cout << left << "instances left after pruning" << endl;
            }
            
            cvShowImageMat("Frame", src);
            cvShowImageMat("Projection", proj);
            cvShowImageMat("Cost Map", marked);
        }
		ii++;
    }
   // time_t endTime;
   // time(&endTime);  
   // double seconds = difftime(endTime, startTime);
    profiler->printResults();
    delete profiler;
}
