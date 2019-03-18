// Completed by Kolin Guo
#include "thresholds.hpp"
#include "cv_helper.hpp"

void abs_sobel_thresh(cv::Mat& src, cv::Mat& dst, char orient, int sobel_kernel=7, int thresh_min=40, int thresh_max=100) {
  cv::Mat gray, gray_Sobel;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

  if (orient == 'x') {
    cv::Sobel(gray, gray_Sobel, CV_64F, 1, 0, sobel_kernel);
  }
  else if (orient == 'y') {
    cv::Sobel(gray, gray_Sobel, CV_64F, 0, 1, sobel_kernel);
  }
  else {
    std::stringstream ss;
    ss << orient << "is invalid for argument orient. Must be either 'x' or 'y'.\n";
    throw std::runtime_error(ss.str());
  }

  gray_Sobel = abs(gray_Sobel);
  // Normalize to be between 0 and 255
  double minVal, maxVal;
  cv::minMaxLoc(gray_Sobel, &minVal, &maxVal);
  gray_Sobel = (gray_Sobel - minVal) * 255 / (maxVal - minVal);
  // Convert to CV_8UC1
  gray_Sobel.convertTo(gray_Sobel, CV_8UC1);
  // Apply threshold
  dst = (gray_Sobel > thresh_min) & (gray_Sobel < thresh_max);
}

void sobel_mag_dir_thresh(cv::Mat& src, cv::Mat& mag, cv::Mat& dir, int sobel_kernel, int mag_thresh_min=40, int mag_thresh_max=100, float dir_thresh_min=0.7, float dir_thresh_max=1.3) {
  cv::Mat gray, gradx, grady, grad_mag, grad_dir;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

  cv::Sobel(gray, gradx, CV_64F, 1, 0, sobel_kernel);
  gradx = abs(gradx);
  cv::Sobel(gray, grady, CV_64F, 0, 1, sobel_kernel);
  grady = abs(grady);
  // Calculate magnitude and direction
  cv::cartToPolar(gradx, grady, grad_mag, grad_dir);
  // Normalize magnitude to be between 0 and 255
  double minVal, maxVal;
  cv::minMaxLoc(grad_mag, &minVal, &maxVal);
  grad_mag = (grad_mag - minVal) * 255 / (maxVal - minVal);
  // Convert magnitude to CV_8UC1
  grad_mag.convertTo(grad_mag, CV_8UC1);
  // Apply threshold
  mag = (grad_mag > mag_thresh_min) & (grad_mag < mag_thresh_max);
  dir = (grad_dir > dir_thresh_min) & (grad_dir < dir_thresh_max);
}

void apply_thresholds(cv::Mat& src, cv::Mat& dst){
  cv::Mat gradx, grady, grad_mag, grad_dir, hls, lab;
  abs_sobel_thresh(src, gradx, 'x', 15, 60, 180);
  abs_sobel_thresh(src, grady, 'y', 15, 40, 100);
  sobel_mag_dir_thresh(src, grad_mag, grad_dir, 5, 20, 100, 0.0, 0.2);
  
  cv::cvtColor(src, hls, cv::COLOR_BGR2HLS);
  cv::cvtColor(src, lab, cv::COLOR_BGR2Lab);

  cv::Mat L, S, B, white, yellow1, yellow2;
  cv::extractChannel(hls, L, 1);       // L (white)
  cv::inRange(L, 210, 255, white);
  cv::extractChannel(hls, S, 2);       // S (yellow, not good shade)
  cv::inRange(S, 100, 255, yellow1);
  cv::extractChannel(lab, B, 2);       // B (yellow)
  cv::inRange(B, 150, 255, yellow2);
  
  /*
  cv::Mat test = gradx | white | (yellow1 & yellow2);
  std::vector<cv::Mat> axs;
  axs.push_back(src.clone());
  axs.push_back(gradx.clone());
  axs.push_back(grady.clone());
  axs.push_back(grad_mag.clone());
  axs.push_back(grad_dir.clone());
  axs.push_back(L.clone());
  axs.push_back(white.clone());
  axs.push_back(S.clone());
  axs.push_back(yellow1.clone());
  axs.push_back(B.clone());
  axs.push_back(yellow2.clone());
  axs.push_back(test.clone());
  subplot("Thresholds", axs, 4, 4);
  */

  dst = gradx | white | (yellow1 & yellow2);
}

