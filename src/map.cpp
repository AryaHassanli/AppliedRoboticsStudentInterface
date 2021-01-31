#include "map.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>

#include "my_utils.hpp"
#include "utils.hpp"

static MyUtils utils;

Map::Map() { is_initialized = false; }

void Map::initialize() {
    is_initialized = true;
}

void Map::detectMasks(const cv::Mat &img_in, std::string help_text, std::vector<MaskBound> &masks) {
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    std::vector<cv::Point2f> samples;
    samples = utils.pickNPoints(3, img_in, help_text);

    MaskBound tmp_mask_bound;
    for (int i = 0; i < 3; i++) {
        cv::Vec3b hsv_pixel = hsv_img.at<cv::Vec3b>(samples[i].y, samples[i].x);
        uchar h = hsv_pixel.val[0];
        uchar s = hsv_pixel.val[1];
        uchar v = hsv_pixel.val[2];

        uchar s_lower = std::min(255, std::max(0, s - threshold.s));
        uchar s_upper = std::min(255, std::max(0, s + threshold.s));
        uchar v_lower = std::min(255, std::max(0, v - threshold.v));
        uchar v_upper = std::min(255, std::max(0, v + threshold.v));

        if (h - threshold.h < 0) {
            tmp_mask_bound.lower_bound = cv::Scalar(0, s_lower, v_lower);
            tmp_mask_bound.upper_bound = cv::Scalar(h + threshold.h, s_upper, v_upper);
            masks.push_back(tmp_mask_bound);

            tmp_mask_bound.lower_bound = cv::Scalar(179 + h - threshold.h, s_lower, v_lower);
            tmp_mask_bound.upper_bound = cv::Scalar(179, s_upper, v_upper);
            masks.push_back(tmp_mask_bound);
        } else if (h + threshold.h > 179) {
            tmp_mask_bound.lower_bound = cv::Scalar(h - threshold.h, s_lower, v_lower);
            tmp_mask_bound.upper_bound = cv::Scalar(179, s_upper, v_upper);
            masks.push_back(tmp_mask_bound);

            tmp_mask_bound.lower_bound = cv::Scalar(0, s_lower, v_lower);
            tmp_mask_bound.upper_bound = cv::Scalar(h + threshold.h - 179, s_upper, v_upper);
            masks.push_back(tmp_mask_bound);
        } else {
            tmp_mask_bound.lower_bound = cv::Scalar(h - threshold.h, s_lower, v_lower);
            tmp_mask_bound.upper_bound = cv::Scalar(h + threshold.h, s_upper, v_upper);
            masks.push_back(tmp_mask_bound);
        }
    }
    return;
}

void Map::applyMasks(const cv::Mat &img_in, std::vector<Map::MaskBound> &mask_bounds, cv::Mat &out_mask) {
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    cv::Mat masks[6];

    for (int i = 0; i < mask_bounds.size(); i++) {
        cv::inRange(hsv_img, mask_bounds[i].lower_bound, mask_bounds[i].upper_bound, masks[i]);
    }

    out_mask = masks[0];
    for (int i = 1; i < mask_bounds.size(); i++) {
        cv::addWeighted(out_mask, 1.0, masks[i], 1.0, 0.0, out_mask);
    }
}

Map::~Map(){};