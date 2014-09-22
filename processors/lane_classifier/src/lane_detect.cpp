
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "projection.h"
#include "profiler.h"

using namespace cv;
using namespace proj;

Profiler *profiler;

void cvShowImageMat(const char *name, Mat& mat) {
	IplImage img = mat;
	cvShowImage(name, &img);
}
void flipHorizAndVert(Mat& img) {
	int flipMode = -1;
	IplImage ipl = img;
	cvFlip(&ipl, &ipl, flipMode);
}

void pullFrame(VideoCapture& cap, Mat& img, Mat& imgGray, void (*adjustFunc)(Mat& img)) {
	Mat frame;
    cap >> frame; // get a new frame from camera
	img = frame;

	if (adjustFunc) {
		adjustFunc(img);
	}
    imgGray = img.clone();
    cvtColor(imgGray, imgGray, CV_BGR2GRAY);
}

void getLightInvariantBGR(const Mat& bgr, Mat& libgr) {

    libgr = bgr;
        Mat raw = bgr.clone(), lab;
        cvtColor(raw, lab, CV_BGR2Lab);
        Mat imgLi(bgr.size(), CV_8UC3);
        int from_to[] = { -1,0, 1,1, 2,2 };
        mixChannels( &lab, 1, &imgLi, 1, from_to, 3 );
        cvtColor(imgLi, libgr, CV_Lab2BGR);
}

int main ( int argc, char **argv )
{
    char resolved_path[256];
    realpath(argv[1], resolved_path);
    printf("Opening movie file \n%s\n",resolved_path);
    imread(argv[1]);
#if 0	
    // Load two images and allocate other structures
    VideoCapture cap(resolved_path); // open the default camera
    // VideoCapture cap("/Users/theJenix/Development/opencv_experiments/opencv_lk/recording1.mov"); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
        return -1;

    cvNamedWindow( "Frame", 0 );
    cvNamedWindow( "IPM", 0 );

	int flipMode = -1;
	Mat imgA, imgGrayA;
	pullFrame(cap, imgA, imgGrayA, NULL); //flipHorizAndVert);
	while(true) {
        Mat imgLi, proj;
        getLightInvariantBGR(imgA, imgLi);

        groundTransformProj(imgLi, proj);
         
        cvShowImageMat( "Frame", imgLi);
        cvShowImageMat( "IPM", proj);
		pullFrame(cap, imgA, imgGrayA, NULL); //flipHorizAndVert);
        
	}
    waitKey(1);
#endif
}

