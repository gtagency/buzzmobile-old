
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "projection.h"

using namespace cv;
using namespace proj;

int main(int argc, const char **argv) {
    
    VideoCapture cap(argv[1]);
    int numFrames = atoi(argv[2]);
    int start = 0;
    int by = 1;
    int end = 0;
    if (argc > 3) {
        start = numFrames;
        numFrames = atoi(argv[3]);
    }
    if (argc > 4) {
        by = atoi(argv[4]);
    }
    end = start + numFrames * by;
    cap.set(CV_CAP_PROP_POS_FRAMES, start);
    for (int ii = start; ii < end; ii += by) {
        Mat src;
        cap >> src;
        char buf[256] = {0};
        Mat proj;
        groundTransformProj(src, proj);
        sprintf(buf, "frame%00000d.png", ii);
        imwrite(buf, proj);
    }
    

    return 0;
}
