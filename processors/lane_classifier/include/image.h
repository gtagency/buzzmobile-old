#ifndef __IMAGE_H
#define __IMAGE_H

#include "opencv/cv.h"
#include "instance.h"

using namespace cv;

namespace image {
	
	int getCvType();
	extern void (*getFeatures)(const Point3_<uchar>* pt, uchar features[2]);

	const Instance makeInstance(uchar imgFeatures[2], int pixelDist, int label);

	void getLightInvariantBGR(const Mat& bgr, Mat& libgr);
	void maskImageByDensity(Mat& img);
    
    template <typename Labeler>
	void extractInstances(const Mat& lab, int startRow, int endRow, int data[3], Labeler labeler, std::vector<Instance>& instances) {

	    uchar features[2];
	    for(int row = startRow; row < endRow; ++row) {
	        const Point3_<uchar> *p = lab.ptr<Point3_<uchar> > (row);
	        //cout << "Row: " << row << endl;
	        //assumes CV_8UC3 LAB color image, with 3 values per pixel
	        for(int col = 0; col < lab.cols; ++col, ++p) {
	            //throw away the L
	    //        if (col < inLaneStart) {
	      //          continue;
	        //    }
	            getFeatures(p, features);
	            uchar lbl = labeler(row, col);
	            int pixelDist = 0; //abs(col - inLaneStart);
	            instances.push_back(makeInstance(features, pixelDist, lbl));
	            if (data[0] < features[0]) data[0] = features[0];
	            if (data[1] < features[1]) data[1] = features[1];
	            if (data[2] < pixelDist)   data[2] = pixelDist;
	        }
	    }
    }

    template <typename Labeler>
	std::vector<Instance> extractInstances(const Mat& lab, int data[3], Labeler labeler) {

	    std::vector<Instance> instances;
		
		int trainingTenth = 5;
	    int trainRow = lab.rows * trainingTenth / 20;
	    int trainRowLast = lab.rows * (trainingTenth + 1) / 20;
	    instances.reserve((lab.rows / 10) * (1 + lab.cols));
//		memset(data, 0, 3 * sizeof(int));
		extractInstances(lab, lab.rows * 10/20, lab.rows * 11/20, data, labeler, instances);
		extractInstances(lab, lab.rows * 14/20, lab.rows * 15/20, data, labeler, instances);
	    return instances;
	}
}

#endif
