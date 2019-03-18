#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <utility>
#include "thresholds.hpp"
#include "signal_proc.hpp"
#include "window_search.hpp"
#include "lane_line.hpp"
#include "cv_helper.hpp"
#include "overloader.hpp"

int main(int argc, char* argv[]) {

  if (argc != 3) {
    std::cerr << "Usage: ./images_main <yaml_file> <path/to/images/\\*.png> \n";
    return 1;
  }

  // create save path for final birds-eye view images
  std::string save_path = "../images/output_images";
  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }

  // get image_paths
  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);

  // cv::Mat to hold the intermediate images
  cv::Mat img_in, img_undistort, img_warp, img_thresholded, 
    histogram, img_best_fit, img_best_fit_warp, img_out;

  /*
   * Phase 2.1
   *    Warp the images into bird-eye view
   *    File: perspective_transform_tester.cpp
   */
  // read in yaml file
  cv::FileStorage file(argv[1], cv::FileStorage::READ);
  // declare camera matrix and list of distortion coefficients
  cv::Mat dist, mtx;
  // declare source points and destination points
  std::vector<cv::Point2f> src_points, dst_points;

  file["distortion coefficients"] >> dist;
  file["camera matrix"] >> mtx;
  file["source points"] >> src_points;
  file["destination points"] >> dst_points;

  // calculate transform matrix
  cv::Mat transMat = cv::getPerspectiveTransform(src_points, dst_points);
  // calculate reverse transform matrix
  cv::Mat revTransMat = cv::getPerspectiveTransform(dst_points, src_points);

  // for each image
  for (cv::String path : image_paths) {
    // read in the image
    img_in = cv::imread(path);
    // undistort it
    cv::undistort(img_in, img_undistort, mtx, dist);
    // warp the image
    cv::warpPerspective(img_undistort, img_warp, transMat, img_in.size());

  /*
   * Phase 2.2
   *    Apply thresholds to the image to convert it into a binary image
   *    File: thresholds.cpp
   */
    apply_thresholds(img_warp, img_thresholded);

  /*
   * Phase 2.4
   *    Window Search
   *    File: window_search.cpp
   */
    cv::cvtColor(img_thresholded, img_best_fit, cv::COLOR_GRAY2BGR);

    std::vector<int> peaks;
    peaks = findPeaks(img_thresholded, histogram);
    
    std::vector<std::unique_ptr<LaneLine>> lane_lines;
    for(unsigned int i = 0; i < peaks.size(); i++)
    {
      if(i == 0)
        lane_lines.push_back(std::unique_ptr<LaneLine>(
              new LaneLine(cv::Scalar(0, 0, 255), img_thresholded.rows)));
      else if(i == 1)
        lane_lines.push_back(std::unique_ptr<LaneLine>(
              new LaneLine(cv::Scalar(255, 0, 0), img_thresholded.rows)));
      else
        lane_lines.push_back(std::unique_ptr<LaneLine>(
              new LaneLine(cv::Scalar(10*i, 20*i, 30*i), img_thresholded.rows)));
    }

    window_search(img_thresholded, img_best_fit, lane_lines, peaks, 9, 100, 75);

  /*
   * Phase 2.5
   *    Warp the image back, overlay with original image, and save it as a file
   */
    // warp the image back
    cv::warpPerspective(img_best_fit, img_best_fit_warp, revTransMat, img_in.size());

    // overlay with original image
    cv::addWeighted(img_in, 1.0, img_best_fit_warp, 0.5, 0.0, img_out);

    // save the final image
    std::string file_name = "/out_" + path.substr(path.find_last_of('/')+1);
    file_name.erase(file_name.end()-4, file_name.end());
    cv::imwrite(save_path+file_name+".png", img_out);
  }

  return 0;
}
