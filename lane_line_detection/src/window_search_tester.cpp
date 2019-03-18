#include <opencv2/opencv.hpp>
#include <iostream>
#include <matplotlibcpp.h>
#include "overloader.hpp"
#include "cv_helper.hpp"
#include "thresholds.hpp"
#include "window_search.hpp"
#include "signal_proc.hpp"
#include "lane_line.hpp"

namespace plt = matplotlibcpp;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: ./window_search_tester <path_to_thresholded_images\\*.png> \n";
    return 1;
  }
  std::vector<cv::String> image_paths;
  cv::glob(argv[1], image_paths);
  cv::Mat binary_warped, window_img, histogram;
  std::vector<cv::Mat> axs;

  for(cv::String path : image_paths)
  {
    binary_warped = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    cv::cvtColor(binary_warped, window_img, cv::COLOR_GRAY2BGR);

    std::vector<int> peaks;
    peaks = findPeaks(binary_warped, histogram);
    
    /*
    // Feel free to comment out the lines below. They are for visualizing histogram.
    std::vector<double> histogram_vis;
    histogram.copyTo(histogram_vis);
    plt::plot(histogram_vis);
    plt::show();
    */

    std::vector<std::unique_ptr<LaneLine>> lane_lines;
    for(unsigned int i = 0; i < peaks.size(); i++)
    {
      if(i == 0)
        lane_lines.push_back(std::unique_ptr<LaneLine>(
              new LaneLine(cv::Scalar(0, 0, 255), binary_warped.rows)));
      else if(i == 1)
        lane_lines.push_back(std::unique_ptr<LaneLine>(
              new LaneLine(cv::Scalar(255, 0, 0), binary_warped.rows)));
      else
        lane_lines.push_back(std::unique_ptr<LaneLine>(
              new LaneLine(cv::Scalar(10*i, 20*i, 30*i), binary_warped.rows)));
    }

    window_search(binary_warped, window_img, lane_lines, peaks, 9, 100, 75);
    axs.push_back(binary_warped.clone());
    axs.push_back(window_img.clone());
  }

  subplot("Window", axs, 4, 4);
}
