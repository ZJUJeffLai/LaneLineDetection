#include <opencv2/opencv.hpp>
#include <iostream>
#include "window_search.hpp"
#include "cv_helper.hpp"

double window_search(cv::Mat& binary_warped, cv::Mat& window_img,
std::vector<std::unique_ptr<LaneLine>>& lane_lines, std::vector<int>& peaks,
int n_windows, int margin, int minpix) {
	int img_width = binary_warped.cols, 
      img_height = binary_warped.rows,
	    window_height = img_height/n_windows,
	    win_top, win_bottom, win_right, win_left;
  int x_current;

  for (unsigned int i = 0; i < lane_lines.size(); i++)
  {
    cv::Mat line_inds_x, line_inds_y;

    x_current = peaks[i];
    for (int window = 0; window < n_windows; window++)
    {
      // update top, bottom, left and right sides of window
      win_top = img_height - window * window_height;
      win_bottom = img_height - (window + 1) * window_height;
      win_left = x_current - margin;
      win_right = x_current + margin;

      // Out of bound check
      if(win_right > img_width-1)
        win_right = img_width-1;
      if(win_left < 0)
        win_left = 0;
      if(win_bottom < 0)
        win_bottom = 0;

      // draw window green
      cv::rectangle(window_img, cv::Point(win_left, win_bottom),
      cv::Point(win_right, win_top), cv::Scalar(0, 255, 0), 2);

      // color in pixels inside of window according to color of lane line
      window_img(cv::Range(win_bottom, win_top),
      cv::Range(win_left, win_right)).setTo(lane_lines[i]->color,
      binary_warped(cv::Range(win_bottom, win_top),
      cv::Range(win_left, win_right)) != 0);

      // Find non-zero pixel locations
      cv::Mat good_inds, good_inds_x, good_inds_y;
      cv::findNonZero(binary_warped(cv::Range(win_bottom, win_top), cv::Range(win_left, win_right)), good_inds);

      if(!good_inds.empty())
      {
        for(unsigned int j = 0; j < good_inds.total(); j++)
        {
          // extract all x positions from first channel
          good_inds_x.push_back(good_inds.at<Point>(j).x + win_left); 
          // extract all y positions from second channel 
          good_inds_y.push_back(win_top - good_inds.at<Point>(j).y); 
        }
        // Push x values to list(cv::Mat) of all x values
        line_inds_x.push_back(good_inds_x);
        // Push y values to list(cv::Mat) of all y values
        line_inds_y.push_back(good_inds_y);

        // if more than minimum number of pixels required to re-center window
        if((int)good_inds.total() > minpix)
          x_current = cv::mean(good_inds_x)[0];
      }
      /*
      // Save to yaml file.
      if (window == 2 && i == 0) {

        cv::FileStorage file("../yaml/Window.yaml", cv::FileStorage::WRITE);
        file << "good_inds " << good_inds ;
        file << "good_inds_x " << good_inds_x ;
        file << "good_inds_y " << good_inds_y ;
        file << "line_inds_x " << line_inds_x ;
        file << "line_inds_y " << line_inds_y ;
        file << "x_current " << x_current ;
        file.release();
      }
      */
    } // for each window

    line_inds_x.convertTo(line_inds_x, CV_64FC1);
    line_inds_y.convertTo(line_inds_y, CV_64FC1);
    lane_lines[i]->fit(line_inds_x, line_inds_y);
  } // for each line
  //showBestFit(window_img, lane_lines, margin);
  
  // generate image with only best-fit lines
  cv::Mat pts1, best_fit_img;
  cv::Mat fitx, ploty;
  best_fit_img = cv::Mat::zeros(window_img.rows, window_img.cols, window_img.type());

  // for each lane_line
  for(unsigned int i = 0; i < lane_lines.size(); i++)
  {
    // get x and y points for best-fit line
    fitx = lane_lines[i]->fitx_pix;
    ploty = lane_lines[i]->ploty_pix;
    cv::hconcat(fitx, ploty, pts1);
    pts1.convertTo(pts1, CV_32S);

    // draw thick best-fit line on blank image to be warped back for final result
    cv::polylines(best_fit_img, pts1, 0, lane_lines[i]->color, 20);
  }

  // generate waypoints
  std::vector<std::pair<double, double>> waypoints_meters = 
    generate_waypoints(best_fit_img, lane_lines, 0.05, 0.95, 8, 6, 
      lane_lines[0]->xm_per_pix, lane_lines[0]->ym_per_pix);

  window_img = best_fit_img.clone();

  // Calculate center_offset
  std::vector<double> center_offsets;
  double xVal, yVal = binary_warped.rows-1;

  cv::Mat fitx_left, fitx_right;
  int img_half_width = binary_warped.cols / 2;
  // for each pair of lane_line
  for(unsigned int i = 0; i < lane_lines.size() - 1; i++)
  {
    // get x points for best-fit line (multiple rows, one col)
    fitx_left = lane_lines[i]->fitx_pix;
    fitx_right = lane_lines[i+1]->fitx_pix;

    // calculate its x value
    xVal = (fitx_left.at<double>(fitx_left.rows-1, 0) + 
        fitx_right.at<double>(fitx_right.rows-1, 0)) / 2;

    center_offsets.push_back(-1 * (xVal - img_half_width) * lane_lines[0]->xm_per_pix);
  }

  return center_offsets[0];
}

void showBestFit(cv::Mat& window_img, 
    std::vector<std::unique_ptr<LaneLine>>& lane_lines, int margin)
{
  /*
   * NOTE:
   * In this context, left_line and right_line are not lane lines!
   * Instead, they define the left and right line boundaries of one lane line.
   * Also, margin is the margin variable used in the window search algorithm.
   */
  cv::Mat pts1, pts2, left_line, right_line, poly_img, best_fit_img;
  cv::Mat fitx, ploty;
  best_fit_img = cv::Mat::zeros(window_img.rows, window_img.cols, window_img.type());

  for(unsigned int i = 0; i < lane_lines.size(); i++)
  {
    // get x and y points for best-fit line
    fitx = lane_lines[i]->fitx_pix;
    ploty = lane_lines[i]->ploty_pix;
    // make left line boundary of lane line
    std::vector<cv::Mat> left_merge = {fitx-margin, ploty};
    cv::merge(left_merge, left_line);

    // make right line boundary of lane line
    std::vector<cv::Mat> right_merge = {fitx+margin, ploty};
    cv::merge(right_merge, right_line);
    cv::flip(right_line, right_line, 0);
    cv::vconcat(left_line, right_line, pts2);
    pts2.convertTo(pts2, CV_32S);
    std::array<cv::Mat,1> poly = { pts2 };

    // draw lane line as polygon
    poly_img = cv::Mat::zeros(window_img.rows, window_img.cols, window_img.type());
    cv::fillPoly(poly_img, poly, cv::Scalar(0,255,0));
    cv::hconcat(fitx, ploty, pts1);
    pts1.convertTo(pts1, CV_32S);

    // draw thin best-fit line on image with window search
    cv::polylines(window_img, pts1, 0, cv::Scalar(0,255,255), 2);
    cv::addWeighted(window_img, 1.0, poly_img, 0.3, 0.0, window_img);

    // draw thick best-fit line on blank image to be warped back for final result
    cv::polylines(best_fit_img, pts1, 0, lane_lines[i]->color, 20);
  }

  std::vector<cv::Mat> axs;
  axs.push_back(window_img.clone());
  axs.push_back(best_fit_img.clone());
  subplot("Final", axs, 2, 1);
}

std::vector<std::pair<double, double>> generate_waypoints(cv::Mat& overlay_img,
std::vector<std::unique_ptr<LaneLine>>& lane_lines, double start_fraction,
double stop_fraction, int num_waypoints, int radius, double xm_per_pix,
double ym_per_pix) {
  // Origin of waypoints_meters is at the bottom center of the image
  std::vector<std::pair<double, double>> waypoints_meters;
  std::vector<std::pair<double, double>> waypoints_pixels;

  cv::Mat fitx_left, fitx_right, ploty;
  // for each pair of lane_line
  for(unsigned int i = 0; i < lane_lines.size() - 1; i++)
  {
    // get x and y points for best-fit line (multiple rows, one col)
    fitx_left = lane_lines[i]->fitx_pix;
    fitx_right = lane_lines[i+1]->fitx_pix;
    ploty = lane_lines[i]->ploty_pix;

    double xVal, yVal;
    int yLoc = ploty.rows * start_fraction;
    int incYLoc = ploty.rows * (stop_fraction - start_fraction) / (num_waypoints-1);
    
    // for each waypoint
    for(int j = 0; j < num_waypoints; j++)
    {
      // calculate its y value
      yVal = ploty.at<double>(yLoc, 0);
      
      // calculate its x value
      xVal = (fitx_left.at<double>(yLoc, 0) + fitx_right.at<double>(yLoc, 0)) / 2;

      // store the x,y value
      waypoints_pixels.push_back(std::make_pair(xVal, yVal));

      // increment y index
      yLoc += incYLoc;
    }
  }

  // colors of waypoints
  std::vector<cv::Scalar> colors;
  colors.push_back(cv::Scalar(255,0,0));
  colors.push_back(cv::Scalar(0,255,0));
  colors.push_back(cv::Scalar(0,0,255));
  colors.push_back(cv::Scalar(255,255,0));
  colors.push_back(cv::Scalar(255,0,255));
  colors.push_back(cv::Scalar(0,255,255));

  // Visualize the waypoints
  for(unsigned int i = 0; i < waypoints_pixels.size(); i++)
  {
    cv::Point2d waypoint(waypoints_pixels[i].first, waypoints_pixels[i].second);
    cv::circle(overlay_img, waypoint, radius, colors[i%6], radius*2);
  }

  /*
  cv::imshow("overlay", overlay_img);
  cv::waitKey();
  */

  // Map pixel to meter
  int img_half_width = overlay_img.cols / 2, 
      img_height = overlay_img.rows;
  for(auto& point_pixels : waypoints_pixels)
    waypoints_meters.push_back(std::make_pair(
          (point_pixels.first - img_half_width) * xm_per_pix, 
          (img_height - point_pixels.second) * ym_per_pix ));

  return waypoints_meters;
}
