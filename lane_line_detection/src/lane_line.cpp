#include <opencv2/opencv.hpp>
#include <iostream>
#include "lane_line.hpp"
#include "signal_proc.hpp"

LaneLine::LaneLine(cv::Scalar color, double rows)
  : color(color){
    // generate y-values for plotting best-fit line
    linspace(0, rows, rows-1, ploty_pix);
    ploty_meters = ploty_pix * ym_per_pix;
} // Constructor

LaneLine::LaneLine(cv::Scalar color, double rows, 
    cv::Mat& avg_fit_pix, cv::Mat& avg_fit_meters)
  : color(color), avg_fit_pix(avg_fit_pix.clone()), 
  avg_fit_meters(avg_fit_meters.clone()) {
    // generate y-values for plotting best-fit line
    linspace(0, rows, rows-1, ploty_pix);
    ploty_meters = ploty_pix * ym_per_pix;
} // Constructor if more than 5 frames

// Updates best-fit line of lane line
void LaneLine::fit(cv::Mat& line_inds_x, cv::Mat& line_inds_y){
  // For unit in pixels
  // get best-fit coefficients of second-order polynomial
  polyFit(line_inds_y, line_inds_x, current_fit_pix, 2);

  // For unit in meters
  // get best-fit coefficients of second-order polynomial
  polyFit(line_inds_y * ym_per_pix, line_inds_x * xm_per_pix, current_fit_meters, 2);
    
  if(avg_fit_pix.empty())   // if no average data
  {
    // generate x values of best-fit line
    polyVal(ploty_pix, fitx_pix, current_fit_pix); 

    // generate x values of best-fit line
    polyVal(ploty_meters, fitx_meters, current_fit_meters); 
  }
  else
  {
    // generate x values of best-fit line
    polyVal(ploty_pix, fitx_pix, avg_fit_pix); 

    // generate x values of best-fit line
    polyVal(ploty_meters, fitx_meters, avg_fit_meters); 
  }
}
