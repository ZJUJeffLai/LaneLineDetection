// Completed by Kolin Guo
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include "cv_helper.hpp"
#include "overloader.hpp"
#include "thresholds.hpp"

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: ./thresholds_tester > <path_to_warped_images\\*.png> \n";
    return 1;
  }

  // create save path for final thresholded images
  std::string save_path = "../images/thresholded_images/";
  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }
  std::vector<cv::String> image_paths;
  cv::glob(argv[1], image_paths);

  cv::Mat img, img_thresholded;
  std::vector<cv::Mat> axs;
  for (cv::String path : image_paths) {
    img = cv::imread(path);
    // apply thresholds
    apply_thresholds(img, img_thresholded);
    // save the thresholded image
    cv::imwrite(save_path+"/thresholded_"+path.substr(path.find_last_of('d')+2), img_thresholded);
    axs.push_back(img.clone());
    axs.push_back(img_thresholded.clone());
  }

  subplot("Sobel", axs, 4, 4);
}
