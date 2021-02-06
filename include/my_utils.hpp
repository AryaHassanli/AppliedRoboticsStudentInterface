#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "utils.hpp"

class MyUtils {
   public:
    MyUtils();

    bool pathExists(const std::string &path);
    static void mouseCallback(int event, int x, int y, int, void *p);
    std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img, std::string msg);

    double pointDistance(Point a, Point b);
    Point polyCenter(Polygon poly);
    ~MyUtils();

   private:
    cv::Mat bg_img;
    std::vector<cv::Point2f> result;
    std::string name;
    std::atomic<bool> done;
    int n;
    double show_scale;
};