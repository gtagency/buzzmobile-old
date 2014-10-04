#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]) {
  cv::Mat sobelImg, threshImg;
  cv::Mat projectedImage = cv::imread(argv[1]);
  const int SLICE_Y = 2*(projectedImage.rows/3);
  

  cv::GaussianBlur(projectedImage, projectedImage, cv::Size(0, 0), 1, 0);

  //cv::cvtColor(projectedImage, projectedImage, CV_BGR2HSV);
  cv::Mat slice(projectedImage, cv::Rect(0, SLICE_Y, projectedImage.cols, 1));

  cv::inRange(slice, cv::Scalar(0, 150, 200), cv::Scalar(120, 255, 255), threshImg);
  //cv::inRange(slice, cv::Scalar(45, 255, 0), cv::Scalar(60, 135, 255), threshImg);

  cv::Sobel(slice, sobelImg, -1, 1, 0);
  cv::cvtColor(sobelImg, sobelImg, CV_BGR2GRAY);
  cv::threshold(sobelImg, sobelImg, 100, 255, cv::THRESH_BINARY);

  //cv::imshow("Sobel", sobelImg);

  int center = -1;

  for (int i = slice.cols-1; i >= 0; --i) {
    if (threshImg.at<uchar>(0, i) == 255 && center == -1) {
      center = i;
    }
  }

  //std::cout << center << std::endl;
  //std::cout << slice.cols/2 << std::endl;

  std::vector<cv::Vec3b> roadPts;
  bool stop = false;
  for (int i = center + 2; i < slice.cols; ++i) {
    if(!stop) {
      if(sobelImg.at<uchar>(0, i) < 200) {
	roadPts.push_back(slice.at<cv::Vec3b>(0, i));
	cv::circle(projectedImage, cv::Point(i, SLICE_Y), 1, cv::Scalar(255, 0, 0), -1);
      } else {
	stop = true;
      }
    }
  }

  cv::Mat yellowLine;
  slice.copyTo(yellowLine, threshImg);

  //std::cout << roadPts.size() << std::endl;

  //cv::imshow("Projection", threshImg);
  cv::imshow("Yellow Line", yellowLine);
  cv::imshow("Result", projectedImage);
  
  cv::waitKey();
}
