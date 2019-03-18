#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>
#include "thresholds.hpp"
#include "signal_proc.hpp"
#include "window_search.hpp"
#include "lane_line.hpp"
#include "cv_helper.hpp"
#include "overloader.hpp"
#define DEBUG 0

int main(int argc, char* argv[]) {

  if (argc != 2) {
    std::cerr << "Usage: ./video_main <yaml_file> \n";
    return 1;
  }

  // Read in video
  cv::VideoCapture video_in("../videos/project_video.mp4");
  if(!video_in.isOpened())
  {
    cout << "Error opening video stream" << endl; 
    return -1; 
  }

  // Open video_out
  cv::VideoWriter video_out;
  if(DEBUG)
    video_out.open("../videos/video_out.avi", 
        VideoWriter::fourcc('M','J','P','G'),
        video_in.get(CAP_PROP_FPS), 
        cv::Size(video_in.get(CAP_PROP_FRAME_WIDTH), video_in.get(CAP_PROP_FRAME_HEIGHT)) );
  else
    video_out.open("../videos/video_out.mp4", 
        VideoWriter::fourcc('M','P','4','V'),
        video_in.get(CAP_PROP_FPS), 
        cv::Size(video_in.get(CAP_PROP_FRAME_WIDTH), video_in.get(CAP_PROP_FRAME_HEIGHT)) );

  // cv::Mat to hold the intermediate images
  cv::Mat frame, img_undistort, img_warp, img_thresholded, 
    histogram, img_best_fit, img_best_fit_warp, out_frame;

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

  // store the best-fit coefficients for averaging filter
  // Outer vector has size of 5 for 5 frames
  // Inner vector has size of (# of lane_lines on each frame)
  std::vector< std::vector<cv::Mat> > frame_best_fit_coeffs_pix, 
    frame_best_fit_coeffs_meters;

  video_in >> frame;
  for(int cur_frame_num = 0; !frame.empty() ; cur_frame_num++, video_in >> frame) {
    // undistort it
    cv::undistort(frame, img_undistort, mtx, dist);
    // warp the image
    cv::warpPerspective(img_undistort, img_warp, transMat, frame.size());

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
      // Use the aveage best-fit coefficient at [5]
      if(frame_best_fit_coeffs_pix.size() == 6)
      {
        if(i == 0)
          lane_lines.push_back(std::unique_ptr<LaneLine>(
                new LaneLine(cv::Scalar(0, 0, 255), img_thresholded.rows, 
                  frame_best_fit_coeffs_pix[5][i], 
                  frame_best_fit_coeffs_meters[5][i] )));
        else if(i == 1)
          lane_lines.push_back(std::unique_ptr<LaneLine>(
                new LaneLine(cv::Scalar(255, 0, 0), img_thresholded.rows, 
                  frame_best_fit_coeffs_pix[5][i], 
                  frame_best_fit_coeffs_meters[5][i] )));
        else
          lane_lines.push_back(std::unique_ptr<LaneLine>(
                new LaneLine(cv::Scalar(10*i, 20*i, 30*i), img_thresholded.rows, 
                  frame_best_fit_coeffs_pix[5][i], 
                  frame_best_fit_coeffs_meters[5][i] )));
      }
      else  // Less than 5 frames, no average
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
    }

    double center_offset = 
      window_search(img_thresholded, img_best_fit, lane_lines, peaks, 9, 100, 75);

  /*
   * Phase 2.5
   *    Warp the image back, overlay with original image, and save it as a file
   */
    // warp the image back
    cv::warpPerspective(img_best_fit, img_best_fit_warp, revTransMat, frame.size());

    // overlay with original image
    cv::addWeighted(img_undistort, 1.0, img_best_fit_warp, 0.5, 0.0, out_frame);

  /*
   * Optional Task
   *    Smooth the curve across five frames
   */
    // if less than 5 frames collected, push_back best_fit_coeffs
    if(frame_best_fit_coeffs_pix.size() < 5)
    {
      std::vector<cv::Mat> best_fit_coeffs_pix, best_fit_coeffs_meters;
      for(auto& p : lane_lines)
      {
        best_fit_coeffs_pix.push_back((p->current_fit_pix).clone());
        best_fit_coeffs_meters.push_back((p->current_fit_meters).clone());
      }

      frame_best_fit_coeffs_pix.push_back(best_fit_coeffs_pix);
      frame_best_fit_coeffs_meters.push_back(best_fit_coeffs_meters);
    }
    else  // else shift the 4 end, and overwrite the end
    {
      // Set frame_best_fit_coeffs_pix[5] to be the frame_best_fit_coeffs_pix[0]
      if(frame_best_fit_coeffs_pix.size() == 5)
      {
        frame_best_fit_coeffs_pix.push_back(frame_best_fit_coeffs_pix[0]);
        frame_best_fit_coeffs_meters.push_back(frame_best_fit_coeffs_meters[0]);
      }
      else
      {
        frame_best_fit_coeffs_pix[5] = frame_best_fit_coeffs_pix[0];
        frame_best_fit_coeffs_meters[5] = frame_best_fit_coeffs_meters[0];
      }

      // Add [1]~[4] to [5]
      for(int i = 1; i < 5; i++)
      {
        // for each lane_lines in frame[i]
        for(unsigned int j = 0; j < frame_best_fit_coeffs_pix[i].size(); j++)
        {
          frame_best_fit_coeffs_pix[5][j] += frame_best_fit_coeffs_pix[i][j];
          frame_best_fit_coeffs_meters[5][j] += frame_best_fit_coeffs_meters[i][j];
        }
      }

      // Divide by 5
      for(unsigned int i = 0; i < frame_best_fit_coeffs_pix[5].size(); i++)
      {
        frame_best_fit_coeffs_pix[5][i] /= 5;
        frame_best_fit_coeffs_meters[5][i] /= 5;
      }

      // overwrite the corresponding element
      std::vector<cv::Mat> best_fit_coeffs_pix, best_fit_coeffs_meters;
      for(auto& p : lane_lines)
      {
        best_fit_coeffs_pix.push_back((p->current_fit_pix).clone());
        best_fit_coeffs_meters.push_back((p->current_fit_meters).clone());
      }

      frame_best_fit_coeffs_pix[cur_frame_num%5] = best_fit_coeffs_pix;
      frame_best_fit_coeffs_meters[cur_frame_num%5] = best_fit_coeffs_meters;
    }

  /*
   * Optional Task
   *    Calculate and display curvature
   */
    //curvature is a simple equation, using ay^2 + by + c format
    double a, b, y, radius_of_curvature;
    std::vector<double> list_radii_of_curvature;
    for(auto& p : lane_lines)
    {
      y = (frame.rows-1) * p->ym_per_pix; //point on the line

      // If we don't have avg data
      if((p->avg_fit_pix).empty())
      {
        a = (p->current_fit_meters).at<double>(2, 0);
        b = (p->current_fit_meters).at<double>(1, 0);
      }
      else  // if we have
      {
        a = (p->avg_fit_meters).at<double>(2, 0);
        b = (p->avg_fit_meters).at<double>(1, 0);
      }

      radius_of_curvature = pow( (abs(1 + pow(2.0*a*y + b, 2.0))), 3.0/2.0) / abs(2.0*a);

      list_radii_of_curvature.push_back(radius_of_curvature);
    }

    // Take the more curved data
    for(auto& r : list_radii_of_curvature)
      if(r < radius_of_curvature)   // if more curved, use the more curved data
        radius_of_curvature = r;

    // Put radius_of_curvature on out_frame
    char buffer[50];
    sprintf(buffer, "%.2f", radius_of_curvature);
    cv::putText(out_frame, 
        "Radius of curvature: " + std::string(buffer) + "m", 
        cv::Point(30, 90), FONT_HERSHEY_TRIPLEX, 1.0, cv::Scalar(0,0,255), 2);

  /*
   * Optional Task
   *    Calculate and display the center offset at the bottom pixel
   */
    // Put center_offset on out_frame

    sprintf(buffer, "%.2f", center_offset);
    cv::putText(out_frame, 
        "Center Offset: " + std::string(buffer) + "m", 
        cv::Point(30, 130), FONT_HERSHEY_TRIPLEX, 1.0, cv::Scalar(0,0,255), 2);

    if(out_frame.empty())
    {
      std::cout << "Warning: Empty Frame!!!" << std::endl;
      continue;
    }
    
    // Display frame number
    std::cout << "Frame : " << cur_frame_num << std::endl;
    cv::putText(out_frame, "Frame: " + std::to_string(cur_frame_num),
        cv::Point(30, 50), FONT_HERSHEY_TRIPLEX, 1.0, cv::Scalar(0,0,255), 2);

    // write to video_out
    video_out.write(out_frame.clone());
  }
  video_in.release();
  video_out.release();

  return 0;
}
