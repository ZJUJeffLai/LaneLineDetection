#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include "overloader.hpp"
#include "cv_helper.hpp"
#include <vector>

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: ./intrinsics_tester <yaml_file> <path/to/images/\\*.png> \n";
    return 1;
  }
  cv::FileStorage file(argv[1], cv::FileStorage::READ);
  cv::Mat dist, mtx;
  file["distortion coefficients"] >> dist;
  file["camera matrix"] >> mtx;

  std::cout << "dist: " << dist << std::endl;
  std::cout << "mtx: " << mtx << std::endl;

  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);

  cv::Mat img, img_dist;
  std::vector<cv::Mat> axs;

  for (cv::String path : image_paths) {
    img = cv::imread(path);

    cv::undistort(img, img_dist, mtx, dist);

    axs.push_back(img.clone());
    axs.push_back(img_dist.clone());
  }
  subplot("Undistort", axs, 6, 8);

  return 0;
}
