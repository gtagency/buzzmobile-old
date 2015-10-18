#include "Line.hpp"


Line::Line(double slope, cv::Point intercept, double strength) {
    this->slope = slope;
    this->intercept = intercept;
    this->strength = strength;
}

double Line::getSlope() {
    return this->slope;
}

cv::Point Line::getIntercept() {
    return this->intercept;
}

double Line::getStrength() {
    return this->strength;
}
