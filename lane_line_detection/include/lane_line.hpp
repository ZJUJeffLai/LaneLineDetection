#ifndef LANE_LINE_HPP
#define LANE_LINE_HPP

class LaneLine {
 public:
  // Constructor
  LaneLine(cv::Scalar color, double rows);
  // Constructor if more than 5 frames
  LaneLine(cv::Scalar color, double rows, cv::Mat& avg_fit_pix, cv::Mat& avg_fit_meters);
  // Destructor
  ~LaneLine() {}
  // Updates best-fit line of lane line
  void fit(cv::Mat& line_inds_x, cv::Mat& line_inds_y);
  // y values over which to plot lane line in pixels
  cv::Mat ploty_pix;
  // y values over which to plot lane line in meters
  cv::Mat ploty_meters;
  // x values of current best-fit line in pixels
  cv::Mat fitx_pix;
  // x values of current best-fit line in meters
  cv::Mat fitx_meters;
  // color of lane lines pixels
  cv::Scalar color;
  // average best-fit coefficients for lane line in pixels based on past 5 frames
  cv::Mat avg_fit_pix;
  // average best-fit coefficients for lane line in meters based on past 5 frames
  cv::Mat avg_fit_meters;
  // current best-fit coefficients for lane line in pixels
  cv::Mat current_fit_pix;
  // currrent best-fit coefficients for lane line in meters
  cv::Mat current_fit_meters;
  // conversion factor for pixels to meters along x-axis
  double xm_per_pix = 3.7/700.;
  // conversion factor for pixels to meters along y-axis
  double ym_per_pix = 30./720.;
  // order of polynomial to be used for best-fit line
  int order = 2;
};

#endif /* LANE_LINE_HPP */
