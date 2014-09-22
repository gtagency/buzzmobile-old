
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

struct Statistics {
    int correct;
    int falsePositives;
    int falseNegatives;
};

//an evaluation for these specific images...we want to exclude
// the black parts to not give us an unfair advantage
int range(int row) {
    if (row < 156) {
        return 400;
    } else {
        double slope = (1.0 * 400 / (780 - 156));
        return 400 - row * slope;
    }
}
void computeStatistics(const Mat& pred, const Mat& truth, Statistics& stats) {
    Mat predGray;
    Mat truthGray;
    cvtColor(pred,  predGray,  CV_RGB2GRAY);
//    threshold( predGray, predGray, 20, 255, 0 );
    cvtColor(truth, truthGray, CV_RGB2GRAY);
  //  imwrite("frame6-t.png", predGray);
    for (int row = 0; row < 780; row++) {
        for (int col = 400 - range(row); col < (400 + range(row)); col++) {
            if (predGray.at<unsigned char>(row, col)) {
                if (truthGray.at<unsigned char>(row, col)) {
                    stats.correct++;
                } else {
                    stats.falsePositives++;
                }
            } else {
                if (truthGray.at<unsigned char>(row, col)) {
                    stats.falseNegatives++;
                } else {
                    stats.correct++;
                }
            }
        }
    }
}

int main(int argv, const char *argc[]) {

    std::vector<Instance> instances;

    const char *trainframes[] = {
        "frame5.png",
        "frame41.png",
    };
    
    const char *sequence[] = {
        "frame6.png",
        "frame8.png",
        "frame14.png",
        "frame21.png",
        "frame26.png",
        "frame32.png",
        "frame38.png",
        "frame41.png",
        "frame47.png"
    };
    
    const char *truth[] = {
        "frame6-truth.png",
        "frame8-truth.png",
        "frame14-truth.png",
        "frame21-truth.png",
        "frame26-truth.png",
        "frame32-truth.png",
        "frame38-truth.png",
        "frame41-truth.png",
        "frame47-truth.png"
    };

    //NOTE: set up data here, it will be updated below
    int k = 21;
    int data[3] = {0};
    Evaluation eval;
    eval._data  = data;
    eval._score = scoreHueAndSat;
    profiler = new Profiler();
    Classifier c(profiler, k, eval);

    Statistics stats = {0};
    //NOTE: training is cumulative
    int trainLen = sizeof(trainframes)/sizeof(trainframes[0]);
    for (int ii = 0; ii < trainLen; ii++) {
   
        // training
        cout << "Training" << endl;
        Mat src = imread(trainframes[ii]);
        Mat cvt; 
        if (getCvType() >= 0) {
            cvtColor(src, cvt, getCvType());
        } else {
            cvt = src;
        }
    
        //these images are already projected
        //Mat proj;
        //groundTransformProj(cvt, proj);
        cout << "Extracting Instances" << endl;
        std::vector<Instance> instances = extractInstances(cvt, data, Labeler());
        cout << "Adding Instances" << endl;
        c.addInstances(instances);

        int sequenceLen = sizeof(sequence)/sizeof(sequence[0]);
        for (int jj = 0; jj < sequenceLen; jj++) {

            cout << "Testing " << sequence[jj] << endl;
            src = imread(sequence[jj]);
            if (getCvType() >= 0) {
                cvtColor(src, cvt, getCvType());
            } else {
                cvt = src;
            }
            //these images are already projected
  //          Mat proj;
//            groundTransformProj(cvt, proj);
    
                
            Mat marked;
            classifyAndMark(src, marked, c);
	        maskImageByDensity(marked);
            char buf[256];
            sprintf(buf, "%s-pred.png", sequence[jj]);
            imwrite(buf, marked);
            Mat tm = imread(truth[jj]);
            computeStatistics(marked, tm, stats); 
        }
        //print out stats after each training/testing run
        int total = stats.correct + stats.falsePositives + stats.falseNegatives;
        cout << stats.correct << "," << stats.falsePositives << "," << stats.falseNegatives << "," << total << endl;
        double correctRate  = 1.0 * stats.correct/total;
        double fpRate   = 1.0 * stats.falsePositives/total;
        double fnRate   = 1.0 * stats.falseNegatives/total;
        cout << correctRate << "," << fpRate << "," << fnRate << endl;
    }
}
