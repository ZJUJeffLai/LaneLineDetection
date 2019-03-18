#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include "overloader.hpp"
#include "cv_helper.hpp"
#include <vector>
#include <string>
#define UDACITY 1

// cv::cvtColor, cv::findChessboardCorners, cv::drawChessboardCorners, cv::calibrateCamera, cv::Undistort

int main(int argc, char* argv[]) {

  if (argc != 3) {
    std::cerr << "Usage: ./calibrate_camera <yaml_file> <path/to/images/\\*.png> \n";
    return 1;
  }

  // distortion coefficients and camera matrix
  cv::Mat dist, mtx;

  // read in all checkerboard images
  std::vector<cv::String> image_paths;
  cv::glob(argv[2], image_paths);

  unsigned int nx;
  unsigned int ny;

  if(UDACITY)
  {
    nx = 9;
    ny = 6;
  }
  else
  {
    nx = 10;
    ny = 7;
  }

  cv::Size boardSize(nx, ny);
  // list of image points (locations of inner corners in pixel space)
  std::vector<cv::Point2f> imgp;
  // list of object points (locations of inner corners in real-world 3D space)
  std::vector<cv::Point3f> objp;

   for (float i = 0; i < ny; i++) {
     for (float j = 0; j < nx; j++) {
       objp.push_back(cv::Point3f(i, j, 0)); // as z-dimension will always be 0
     }
   }
  // List of list of object points
  std::vector<std::vector<cv::Point3f>> obj_points;
  // List of list of image points
  std::vector<std::vector<cv::Point2f>> img_points;
  // Input image
  cv::Mat image;
  // create save path for drawn checkerboard corners.
  std::string save_path = "../drawn_corners/";
  if(UDACITY)
    save_path += "Udacity/";
  else
    save_path += "Fisheye/";

  if (!boost::filesystem::exists(save_path)) {
    boost::filesystem::create_directory(save_path);
  }

  for (cv::String path : image_paths) {
    image = cv::imread(path);
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    vector<cv::Point2f> corners;
    bool ret = cv::findChessboardCorners(gray, cv::Size(nx,ny), corners);
    if (ret == true) {
      obj_points.push_back(objp);

      img_points.push_back(corners);

      cv::drawChessboardCorners(image, cv::Size(nx, ny), corners, ret);

      // save the image
      std::string file_name = path.substr(path.find_last_of('/')+1);
      file_name.erase(file_name.end()-4, file_name.end());
      cv::imwrite(save_path+file_name+".png", image);
    }
  }

  std::vector<cv::Mat> rvecs, tvecs;
  cv::calibrateCamera(obj_points, img_points, image.size(), mtx, dist, rvecs, tvecs, 0);

  // Save camera matrix and list of distortion coefficients to yaml file.
  if (dist.rows > 0 && mtx.rows > 0) {
    std::cout << "dist: " << dist << std::endl;
    std::cout << "mtx: " << mtx << std::endl;

    cv::FileStorage file(argv[1], cv::FileStorage::WRITE);
    file << "distortion coefficients" << dist;
    file << "camera matrix" << mtx;
    file.release();
  }

  return 0;
}
