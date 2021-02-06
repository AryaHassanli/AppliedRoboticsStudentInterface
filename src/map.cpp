#include "map.hpp"

#include <experimental/filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "my_utils.hpp"
#include "utils.hpp"

static MyUtils utils;

struct Threshold {
    uchar h = 5;
    uchar s = 50;
    uchar v = 50;
} threshold;

Map::Map() { is_initialized = false; }

bool Map::isInitialized() const { return is_initialized; }

void Map::initialize(const cv::Mat &img_in, const std::string &config_folder) {
    if (is_initialized) {
        return;
    }
    is_initialized = true;
}

void Map::createMaskBounds(const cv::Mat &img_in, std::vector<MaskBound> &masks, std::string help_text) {
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

void Map::getObstaclesBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds,
                             const std::string &config_folder) {
    std::string obstacle_file_path = config_folder + "/obstacle_mask_bounds.csv";
    if (!std::experimental::filesystem::exists(obstacle_file_path)) {
        std::ofstream output(obstacle_file_path);
        if (!output.is_open()) {
            throw std::runtime_error("Cannot write file: " + obstacle_file_path);
        }
        createMaskBounds(img_in, mask_bounds, " For Obstacles");

        for (int i = 0; i < mask_bounds.size(); i++) {
            output << mask_bounds[i].lower_bound.val[0] << " ";
            output << mask_bounds[i].lower_bound.val[1] << " ";
            output << mask_bounds[i].lower_bound.val[2] << " ";
            output << mask_bounds[i].upper_bound.val[0] << " ";
            output << mask_bounds[i].upper_bound.val[1] << " ";
            output << mask_bounds[i].upper_bound.val[2] << std::endl;
        }
        output.close();
    } else {
        std::ifstream input(obstacle_file_path);
        while (!input.eof()) {
            double h1, s1, v1, h2, s2, v2;
            if (!(input >> h1 >> s1 >> v1 >> h2 >> s2 >> v2)) {
                if (input.eof())
                    break;
                else {
                    throw std::runtime_error("Malformed file: " + obstacle_file_path);
                }
            }
            MaskBound tmp_mask_bound;
            tmp_mask_bound.lower_bound = cv::Scalar(h1, s1, v1);
            tmp_mask_bound.upper_bound = cv::Scalar(h2, s2, v2);
            mask_bounds.push_back(tmp_mask_bound);
        }
    }
}
void Map::getGateBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds, const std::string &config_folder) {
    std::string gate_file_path = config_folder + "/gate_mask_bounds.csv";
    if (!std::experimental::filesystem::exists(gate_file_path)) {
        std::ofstream output(gate_file_path);
        if (!output.is_open()) {
            throw std::runtime_error("Cannot write file: " + gate_file_path);
        }
        Map::createMaskBounds(img_in, mask_bounds, " For gates");
        for (int i = 0; i < mask_bounds.size(); i++) {
            output << mask_bounds[i].lower_bound.val[0] << " ";
            output << mask_bounds[i].lower_bound.val[1] << " ";
            output << mask_bounds[i].lower_bound.val[2] << " ";
            output << mask_bounds[i].upper_bound.val[0] << " ";
            output << mask_bounds[i].upper_bound.val[1] << " ";
            output << mask_bounds[i].upper_bound.val[2] << std::endl;
        }
        output.close();
    } else {
        std::ifstream input(gate_file_path);
        while (!input.eof()) {
            double h1, s1, v1, h2, s2, v2;
            if (!(input >> h1 >> s1 >> v1 >> h2 >> s2 >> v2)) {
                if (input.eof())
                    break;
                else {
                    throw std::runtime_error("Malformed file: " + gate_file_path);
                }
            }
            MaskBound tmp_mask_bound;
            tmp_mask_bound.lower_bound = cv::Scalar(h1, s1, v1);
            tmp_mask_bound.upper_bound = cv::Scalar(h2, s2, v2);
            mask_bounds.push_back(tmp_mask_bound);
        }
    }
}

void Map::getVictimsBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds,
                           const std::string &config_folder) {
    std::string victim_file_path = config_folder + "/victim_mask_bounds.csv";
    if (!std::experimental::filesystem::exists(victim_file_path)) {
        std::ofstream output(victim_file_path);
        if (!output.is_open()) {
            throw std::runtime_error("Cannot write file: " + victim_file_path);
        }
        Map::createMaskBounds(img_in, mask_bounds, " For victims");
        for (int i = 0; i < mask_bounds.size(); i++) {
            output << mask_bounds[i].lower_bound.val[0] << " ";
            output << mask_bounds[i].lower_bound.val[1] << " ";
            output << mask_bounds[i].lower_bound.val[2] << " ";
            output << mask_bounds[i].upper_bound.val[0] << " ";
            output << mask_bounds[i].upper_bound.val[1] << " ";
            output << mask_bounds[i].upper_bound.val[2] << std::endl;
        }
        output.close();
    } else {
        std::ifstream input(victim_file_path);
        while (!input.eof()) {
            double h1, s1, v1, h2, s2, v2;
            if (!(input >> h1 >> s1 >> v1 >> h2 >> s2 >> v2)) {
                if (input.eof())
                    break;
                else {
                    throw std::runtime_error("Malformed file: " + victim_file_path);
                }
            }
            MaskBound tmp_mask_bound;
            tmp_mask_bound.lower_bound = cv::Scalar(h1, s1, v1);
            tmp_mask_bound.upper_bound = cv::Scalar(h2, s2, v2);
            mask_bounds.push_back(tmp_mask_bound);
        }
    }
}

void Map::getRobotBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds, const std::string &config_folder) {
    std::string robot_file_path = config_folder + "/robot_mask_bounds.csv";
    if (!std::experimental::filesystem::exists(robot_file_path)) {
        std::ofstream output(robot_file_path);
        if (!output.is_open()) {
            throw std::runtime_error("Cannot write file: " + robot_file_path);
        }
        Map::createMaskBounds(img_in, mask_bounds, " For robot");
        for (int i = 0; i < mask_bounds.size(); i++) {
            output << mask_bounds[i].lower_bound.val[0] << " ";
            output << mask_bounds[i].lower_bound.val[1] << " ";
            output << mask_bounds[i].lower_bound.val[2] << " ";
            output << mask_bounds[i].upper_bound.val[0] << " ";
            output << mask_bounds[i].upper_bound.val[1] << " ";
            output << mask_bounds[i].upper_bound.val[2] << std::endl;
        }
        output.close();
    } else {
        std::ifstream input(robot_file_path);
        while (!input.eof()) {
            double h1, s1, v1, h2, s2, v2;
            if (!(input >> h1 >> s1 >> v1 >> h2 >> s2 >> v2)) {
                if (input.eof())
                    break;
                else {
                    throw std::runtime_error("Malformed file: " + robot_file_path);
                }
            }
            MaskBound tmp_mask_bound;
            tmp_mask_bound.lower_bound = cv::Scalar(h1, s1, v1);
            tmp_mask_bound.upper_bound = cv::Scalar(h2, s2, v2);
            mask_bounds.push_back(tmp_mask_bound);
        }
    }
}

void Map::applyMasks(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds, cv::Mat &out_mask) {
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

void Map::findObstacles(const cv::Mat &mask_in, std::vector<Polygon> &obstacles, const double scale) {
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

    cv::findContours(mask_in, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 5, true);

        Polygon scaled_contour;

        for (const auto &pt : approx_curve) {
            scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
        }

        obstacles.push_back(scaled_contour);
    }
}

void Map::findGate(const cv::Mat &mask_in, Polygon &gate, const double scale) {
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

    cv::findContours(mask_in, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 20, true);

        if (approx_curve.size() != 4) {
            continue;
        }
        Polygon scaled_contour;

        for (const auto &pt : approx_curve) {
            scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
        }
        gate = scaled_contour;
    }
}

void Map::findVictims(const cv::Mat &mask_in, std::vector<std::pair<int, Polygon>> &victims, const double scale) {
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

    cv::findContours(mask_in, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 3, true);
        if (approx_curve.size() < 8) {
            continue;
        }
        Polygon scaled_contour;

        for (const auto &pt : approx_curve) {
            scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
        }
        // TODO: OCR

        victims.push_back(std::make_pair(i, scaled_contour));
    }
}

bool Map::findRobot(const cv::Mat &mask_in, Polygon &triangle, const double scale) {
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

    cv::findContours(mask_in, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 20, true);

        if (approx_curve.size() != 3) {
            continue;
        }
        Polygon scaled_contour;

        for (const auto &pt : approx_curve) {
            scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
        }
        triangle = scaled_contour;
    }
    if (triangle.size() == 3) {
        return true;
    } else {
        return false;
    }
}
Map::~Map(){};