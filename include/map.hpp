#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include "my_utils.hpp"
#include "utils.hpp"

struct MaskBound {
    cv::Scalar lower_bound;
    cv::Scalar upper_bound;
};

class Map {
   public:
    Map();

    bool isInitialized() const;
    void initialize(const cv::Mat &img_in, const std::string &config_folder);

    static void createMaskBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds, std::string help_text);
    static void applyMasks(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds, cv::Mat &out_mask);

    static void getObstaclesBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds,
                                   const std::string &config_folder);
    static void getGateBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds,
                              const std::string &config_folder);
    static void getVictimsBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds,
                                 const std::string &config_folder);

    static void findObstacles(const cv::Mat &mask_in, std::vector<Polygon> &obstacles, const double scale);
    static void findGate(const cv::Mat &mask_in, Polygon &gate, const double scale);
    static void findVictims(const cv::Mat &mask_in, std::vector<std::pair<int, Polygon>> &victims, const double scale);

    static void getRobotBounds(const cv::Mat &img_in, std::vector<MaskBound> &mask_bounds,
                               const std::string &config_folder);
    static bool findRobot(const cv::Mat &mask_in, Polygon &triangle, const double scale);

    ~Map();

   private:
    bool is_initialized;
};
