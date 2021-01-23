#include "image_undistort.hpp"

#include <experimental/filesystem>

ImageUndistort::ImageUndistort() {
        is_initialized_ = false;
}

bool ImageUndistort::isInitialized() const { return is_initialized_; }

bool ImageUndistort::initialize(const cv::Size& img_size, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs) {
    if (is_initialized_) return true;

    cv::Mat R;
    cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_size, CV_16SC2, full_map1_, full_map2_);
    cam_matrix_ = cam_matrix.clone();
    dist_coeffs_ = dist_coeffs.clone();
    is_initialized_ = true;
    return is_initialized_;
}

bool ImageUndistort::undistort(const cv::Mat& img_in, cv::Mat& img_out) {
    if (!isInitialized()) return false;

    cv::remap(img_in, img_out, full_map1_, full_map2_, cv::INTER_LINEAR);
    return true;
}

ImageUndistort::~ImageUndistort() {}