#include <opencv2/opencv.hpp>

namespace balance {
    void grayWorld(const cv::Mat &input, cv::Mat &output) {
        double mean;
        cv::Scalar scalarMean = cv::mean(input);
        if (input.channels() == 3) {
            mean = (scalarMean.val[0] + scalarMean.val[1] + scalarMean.val[2]) / 3.0;
        } else {
            mean = scalarMean.val[0];
        }

        double scaleR = mean / scalarMean.val[2];
        double scaleG = mean / scalarMean.val[1];
        double scaleB = mean / scalarMean.val[0];

        if (input.channels() == 3) {
            for (uint x = 0; x < input.cols; ++x) {
                for (uint y = 0; y < input.rows; ++y) {
                    output.at<cv::Vec3b>(y, x).val[0] = input.at<cv::Vec3b>(y, x).val[0] * scaleB;
                    output.at<cv::Vec3b>(y, x).val[1] = input.at<cv::Vec3b>(y, x).val[1] * scaleG;
                    output.at<cv::Vec3b>(y, x).val[2] = input.at<cv::Vec3b>(y, x).val[2] * scaleR;
                }
            }
        } else {
            for (uint x = 0; x < input.cols; ++x) {
                for (uint y = 0; y < input.rows; ++y) {
                    output.at<uchar>(y, x) = input.at<uchar>(y, x) * mean;
                }
            }
        }
    }
}