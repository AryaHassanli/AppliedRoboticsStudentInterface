#pragma once
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "utils.hpp"

struct Node {
    int id;
    double x;
    double y;
    std::vector<Node *> neighbours;

    bool visited;
    bool in_q;
    Node *parent;
    double distance;

    Node(int id, double x, double y) : id(id), x(x), y(y) {}
    Node() : Node(-1000, 0, 0) {
        visited = false;
        parent = NULL;
        distance = 10000;
    }
};

class Plan {
   public:
    Plan();
    ~Plan();

    void plan(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                    const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                    const float y, const float theta, Path &path, const std::string &config_folder);

   private:
    void showMap(cv::Mat map);
    void initMap(cv::Mat &img, const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                 const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                 const float y, const float theta);
    std::vector<Point> getHaltonPoints(int number_of_points, double width, double height);
    std::vector<Polygon> marginObstacles(std::vector<Polygon> obstacles);
    std::vector<Point> getFreePoints(std::vector<Point> points, const std::vector<Polygon> &margined_obstacle_list,
                                     const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate);
    void createGraph(cv::Mat &map, std::vector<Point> halton_points, const std::vector<Polygon> &margined_obstacle_list,
                     const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                     const float y);
    std::vector<Node *> findRoute(Node *source, Node *target);
    void drawRoute(cv::Mat &img, std::vector<Node *> route, int thickness);
    std::vector<Node *> smoothRoute(std::vector<Node *> route, const std::vector<Polygon> &margined_obstacle_list);
};
