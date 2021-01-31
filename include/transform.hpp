#pragma once

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

class Transform {
   public:
    Transform();

    bool initialize(const cv::Size &img_size, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs);
    bool isInitialized() const;
    bool undistort(const cv::Mat &img_in, cv::Mat &img_out);

    bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                        cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder);

    void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec,
                            const std::vector<cv::Point3f> &object_points_plane,
                            const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf,
                            const std::string &config_folder);

    void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder);
    
    ~Transform();

   private:
    bool is_initialized_;
    cv::Mat cam_matrix_, dist_coeffs_;
    cv::Mat full_map1_, full_map2_;
};