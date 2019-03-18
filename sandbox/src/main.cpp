#include <opencv2/opencv.hpp>
#include <iostream>
#include "overloader.hpp"
#include "cv_helper.hpp"

int main() {
  cv::Mat img;
  img = cv::imread("../images/test_images/test1.jpg");
  cv::imshow("image", img);
  cv::waitKey();
}
