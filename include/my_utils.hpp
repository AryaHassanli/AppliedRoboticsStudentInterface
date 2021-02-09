#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "utils.hpp"

#define MY_COLOR_RED cv::Scalar(0, 0, 255)
#define MY_COLOR_WHITE cv::Scalar(255, 255, 255)
#define MY_COLOR_BLACK cv::Scalar(0, 0, 0)
#define MY_COLOR_GREEN cv::Scalar(0, 255, 0)
#define MY_COLOR_BLUE cv::Scalar(255, 0, 0)

class MyUtils {
   public:
    MyUtils();

    bool pathExists(const std::string &path);
    static void mouseCallback(int event, int x, int y, int, void *p);
    std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img, std::string msg);

    static double pointDistance(Point a, Point b);
    static Point polyCenter(Polygon poly);

    static std::vector<cv::Point> cvPoly(Polygon poly);
    static cv::Point2f cvPoint(Point point);

    static Polygon scalePoly(Polygon poly, double scale);
    static Point scalePoint(Point point, double scale);

    static void drawPoly(cv::Mat &img, Polygon poly, double scale, cv::Scalar color, int thickness);
    static void drawPoints(cv::Mat &img, std::vector<Point> points, double scale, cv::Scalar color, int thickness);

    static bool segSegCollision(Point a1, Point a2, Point b1, Point b2);
    static bool segPolyCollision(Point a1, Point a2, Polygon poly);
    static bool segPolysCollision(Point a1, Point a2, std::vector<Polygon> polys);
    
    ~MyUtils();

   private:
    cv::Mat bg_img;
    std::vector<cv::Point2f> result;
    std::string name;
    std::atomic<bool> done;
    int n;
    double show_scale;
};