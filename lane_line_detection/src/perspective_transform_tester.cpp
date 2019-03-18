// Completed by Kolin Guo
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <array>
#include "overloader.hpp"
#include "cv_helper.hpp"

// draws the four source or destination points on an image
void draw_viewing_window(cv::Mat& dst, std::vector<cv::Point2f>& points, cv::Scalar color, int thickness);

int main(int argc, char* argv[]) {

  if (argc != 3) {
    std::cerr << "Usage: ./perspective_transform_tester <yaml_file> <path/to/images/\\*.png> \n";
    return 1;
  }

  // create save path for final birds-eye view images
  std::string save_path = "../images/warped_images";
  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }

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

  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);

  cv::Mat img, img_dist, img_warp;
  std::vector<cv::Mat> axs;
  for (cv::String path : image_paths) {
    img = cv::imread(path);
    cv::undistort(img, img_dist, mtx, dist);
    // draw green source points on your source image with thickness of 5
    draw_viewing_window(img_dist, src_points, cv::Scalar(0,255,0), 5);
    axs.push_back(img_dist.clone());
  }
  subplot("source points", axs, 4, 2);

  axs.clear();

  cv::Mat transMat = cv::getPerspectiveTransform(src_points, dst_points);

  for (cv::String path : image_paths) {
    img = cv::imread(path);
    cv::undistort(img, img_dist, mtx, dist);
    cv::warpPerspective(img_dist, img_warp, transMat, img.size());
    // save the warpped image
    std::string file_name = "/warped_" + path.substr(path.find_last_of('/')+1);
    file_name.erase(file_name.end()-4, file_name.end());
    cv::imwrite(save_path+file_name+".png", img_warp);
    // draw green source points on your source image with thickness of 5
    draw_viewing_window(img_warp, dst_points, cv::Scalar(0,255,0), 5);
    axs.push_back(img_warp.clone());
  }
  subplot("destination points", axs, 4, 2);

  return 0;
}

void draw_viewing_window(cv::Mat& dst, std::vector<cv::Point2f>& points, cv::Scalar color, int thickness) {
  if (points.size() != 4) {
    std::stringstream ss;
    ss << "Invalid number of points. There must be exactly 4 points.\n";
    throw std::runtime_error(ss.str());
  }
  cv::line(dst, points[0], points[1], color, thickness);
  cv::line(dst, points[1], points[2], color, thickness);
  cv::line(dst, points[2], points[3], color, thickness);
  cv::line(dst, points[3], points[0], color, thickness);
}
