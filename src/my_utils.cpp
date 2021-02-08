#include "my_utils.hpp"

#include <math.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

MyUtils::MyUtils() { show_scale = 1.0; };

bool MyUtils::pathExists(const std::string &path) {
    std::ifstream f(path.c_str());
    return f.good();
};

void MyUtils::mouseCallback(int event, int x, int y, int, void *p) {
    MyUtils *t = (MyUtils *)p;

    if (event != cv::EVENT_LBUTTONDOWN || t->done.load()) return;

    t->result.emplace_back(x * t->show_scale, y * t->show_scale);

    cv::circle(t->bg_img, cv::Point(x, y), 11 / t->show_scale, cv::Scalar(255, 255, 255), 1);
    cv::circle(t->bg_img, cv::Point(x, y), 10 / t->show_scale, cv::Scalar(0, 0, 0), 1);
    cv::circle(t->bg_img, cv::Point(x, y), 9 / t->show_scale, cv::Scalar(255, 255, 255), 1);

    cv::circle(t->bg_img, cv::Point(x, y), 1 / t->show_scale, cv::Scalar(255, 255, 255), -1);
    cv::circle(t->bg_img, cv::Point(x, y), 2 / t->show_scale, cv::Scalar(0, 0, 0), 1);
    cv::circle(t->bg_img, cv::Point(x, y), 3 / t->show_scale, cv::Scalar(255, 255, 255), 1);

    cv::imshow(t->name.c_str(), t->bg_img);

    if (t->result.size() >= t->n) {
        usleep(500 * 1000);
        t->done.store(true);
    }
}

std::vector<cv::Point2f> MyUtils::pickNPoints(int n0, const cv::Mat &img, std::string msg) {
    result.clear();
    cv::Size small_size(img.cols / show_scale, img.rows / show_scale);
    cv::resize(img, bg_img, small_size);
    // bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points" + msg;
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), mouseCallback, this);
    while (!done.load()) {
        cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());
    return result;
}

double MyUtils::pointDistance(Point a, Point b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

Point MyUtils::polyCenter(Polygon poly) {
    double x = 0;
    double y = 0;
    for (int i = 0; i < poly.size(); i++) {
        x += poly[i].x;
        y += poly[i].y;
    }
    x /= poly.size();
    y /= poly.size();
    return Point(x, y);
}

std::vector<cv::Point> MyUtils::cvPoly(Polygon poly) {
    std::vector<cv::Point> tmp;
    for (int i = 0; i < poly.size(); i++) {
        tmp.push_back(cv::Point2f(poly[i].x, poly[i].y));
    }
    return tmp;
}

Polygon MyUtils::scalePoly(Polygon poly, double scale) {
    for (int i = 0; i < poly.size(); i++) {
        poly[i].x *= scale;
        poly[i].y *= scale;
    }
    return poly;
}

void MyUtils::drawPoly(cv::Mat &img, Polygon poly, double scale, cv::Scalar color, int thickness) {
    std::vector<std::vector<cv::Point>> tmp;

    poly = scalePoly(poly, scale);
    tmp.push_back(cvPoly(poly));
    cv::drawContours(img, tmp, -1, color, thickness);
}

void MyUtils::drawPoints(cv::Mat &img, std::vector<Point> points, double scale, cv::Scalar color, int thickness) {
    for (int i = 0; i < points.size(); i++) {
        double x = points[i].x * scale;
        double y = points[i].y * scale;
        cv::circle(img, cv::Point2f(x, y), 1, color, thickness);
    }
}

Point MyUtils::scalePoint(Point point, double scale) {
    double x = point.x * scale;
    double y = point.y * scale;
    return Point(x, y);
}

cv::Point2f MyUtils::cvPoint(Point point) { return cv::Point2f(point.x, point.y); }

MyUtils::~MyUtils(){

};