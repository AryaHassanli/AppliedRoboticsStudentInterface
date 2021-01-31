#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include "my_utils.hpp"
#include "utils.hpp"

class Map {
   public:
    struct MaskBound {
        cv::Scalar lower_bound;
        cv::Scalar upper_bound;
    };

    bool is_initialized;

    Map();

    void initialize();
    void detectMasks(const cv::Mat &img_in, std::string help_text, std::vector<MaskBound> &masks);
    static void applyMasks(const cv::Mat &img_in, std::vector<Map::MaskBound> &mask_bounds, cv::Mat &out_mask);

    ~Map();

    std::vector<MaskBound> obstacle_mask_bounds, victim_mask_bounds, gate_mask_bounds, robot_mask_bounds;
    cv::Mat obstacle_mask, victim_mask, robot_mask, gate_mask;

   private:
    struct Threshold {
        uchar h = 5;
        uchar s = 50;
        uchar v = 50;
    } threshold;
};