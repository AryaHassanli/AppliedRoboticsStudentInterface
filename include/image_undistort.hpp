#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

class ImageUndistort {

public:
  ImageUndistort();

  bool initialize(const cv::Size& img_size, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs);
  bool isInitialized() const;
  bool undistort(const cv::Mat& img_in, cv::Mat& img_out);

  ~ImageUndistort();

private:
  bool is_initialized_;
  cv::Mat cam_matrix_, dist_coeffs_;
  cv::Mat full_map1_, full_map2_;  // For faster version
};