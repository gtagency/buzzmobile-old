#include <opencv2/opencv.hpp>


class Line {
    public:
        Line(double slope, cv::Point intercept, double strength);
        double getSlope();
        cv::Point getIntercept();
        double getStrength(); //r^2 value
        
    protected:
        double slope;
        cv::Point intercept;
        double strength;
};
