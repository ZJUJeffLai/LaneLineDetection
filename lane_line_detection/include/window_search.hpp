#ifndef WINDOW_SEARCH_HPP
#define WINDOW_SEARCH_HPP
#include <utility>
#include "lane_line.hpp"

// window_search returns center_offset
double window_search(cv::Mat& binary_warped, cv::Mat& window_img, std::vector<std::unique_ptr<LaneLine>>& lane_lines, 
    std::vector<int>& peaks, int n_windows, int margin, int minpix);

std::vector<std::pair<double, double>> generate_waypoints(cv::Mat& overlay_img, 
    std::vector<std::unique_ptr<LaneLine>>& lane_lines, 
    double start_fraction, double stop_fraction, int num_waypoints, int radius, 
    double xm_per_pix, double ym_per_pix);

void showBestFit(cv::Mat& window_img, std::vector<std::unique_ptr<LaneLine>>& lane_lines, int margin);













#endif /* WINDOW_SEARCH_HPP */
