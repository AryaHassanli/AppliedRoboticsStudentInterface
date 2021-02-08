
#include "plan.hpp"

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "halton.hpp"
#include "my_utils.hpp"
#include "utils.hpp"

Plan::Plan(){

};

Plan::~Plan(){

};

#define show_scale 600.0
#define NUM_HALTON_POINTS 499
#define OBST_MARGIN 0.06
#define MAX_K_NEIGHBOURS 4
#define MAP_SECTIONS 12
#define BORD_MARGIN 0.05
#define INF_DIST 10000

double map_width;
double map_height;

std::vector<Node *> nodes;
Node *gate_node;
Node *robot_node;
std::vector<Node *> victim_nodes;
std::vector<Node *> section_to_points[MAP_SECTIONS][MAP_SECTIONS];
std::map<Node *, std::pair<int, int>> point_to_section;

std::vector<std::vector<Node *>> routes;

bool nodeLessThan(Node *a, Node *b) { return a->distance < b->distance; }

bool nodeGreaterThan(Node *a, Node *b) { return a->distance > b->distance; }

bool victimLessThan(std::pair<int, Polygon> a, std::pair<int, Polygon> b) { return a.first < b.first; }

void Plan::initMap(cv::Mat &img, const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                   const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                   const float y, const float theta) {
    Polygon borders_scaled = MyUtils::scalePoly(borders, show_scale);
    Polygon gate_scaled = MyUtils::scalePoly(gate, show_scale);

    // Create a free map for further visualization
    cv::Rect map_rect_scaled = cv::boundingRect(MyUtils::cvPoly(borders_scaled));
    cv::Mat map(map_rect_scaled.height, map_rect_scaled.width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Draw Gate on Map
    MyUtils::drawPoly(map, gate, show_scale, cv::Scalar(0, 255, 0), 2);

    // Draw Victims on Map
    for (int i = 0; i < victim_list.size(); i++) {
        MyUtils::drawPoly(map, victim_list[i].second, show_scale, cv::Scalar(255, 255, 0), 1);

        Point victim_center = MyUtils::polyCenter(victim_list[i].second);
        cv::putText(map, std::to_string(victim_list[i].first),
                    cv::Point2f(victim_center.x * show_scale, victim_center.y * show_scale), cv::FONT_HERSHEY_PLAIN, 2,
                    cv::Scalar(255, 255, 0));
    }

    // Draw Robot
    cv::circle(map, cv::Point2f(x * show_scale, y * show_scale), 0.05 * show_scale, cv::Scalar(255, 0, 0), 1);
    cv::circle(map, cv::Point2f((x + 0.05 * std::cos(theta)) * show_scale, (y + 0.05 * std::sin(theta)) * show_scale),
               0.02 * show_scale, cv::Scalar(255, 0, 0), 1);

    // Draw Obstacles
    for (int i = 0; i < obstacle_list.size(); i++) {
        MyUtils::drawPoly(map, obstacle_list[i], show_scale, cv::Scalar(0, 0, 255), 1);
    }

    img = map.clone();
}

std::vector<Polygon> Plan::marginObstacles(std::vector<Polygon> obstacles) {
    std::vector<Polygon> margined_obstacle_list;
    for (int i = 0; i < obstacles.size(); i++) {
        Polygon obstacle = obstacles[i];
        Polygon margined_obstacle;

        Point obstacle_center = MyUtils::polyCenter(obstacle);
        for (int j = 0; j < obstacle.size(); j++) {
            Point node = obstacle[j];
            double dx = node.x - obstacle_center.x;
            double dy = node.y - obstacle_center.y;
            double sin_ang = dy / std::sqrt(dx * dx + dy * dy);
            double cos_ang = dx / std::sqrt(dx * dx + dy * dy);

            Point margined_point;
            margined_point.x = node.x + OBST_MARGIN * cos_ang;
            margined_point.y = node.y + OBST_MARGIN * sin_ang;
            margined_obstacle.push_back(margined_point);
        }
        margined_obstacle_list.push_back(margined_obstacle);
    }
    return margined_obstacle_list;
}

std::vector<Point> Plan::getHaltonPoints(int number_of_points, double width, double height) {
    std::vector<Point> tmp;
    double *halton_seq = halton_sequence(0, number_of_points - 1, 2);
    for (int i = 0; i < number_of_points; i++) {
        double x = BORD_MARGIN + halton_seq[i * 2 + 0] * ((double)width - 2.0 * BORD_MARGIN);
        double y = BORD_MARGIN + halton_seq[i * 2 + 1] * ((double)height - 2.0 * BORD_MARGIN);
        tmp.push_back(Point(x, y));
    }
    return tmp;
}

std::vector<Point> Plan::getFreePoints(std::vector<Point> points, const std::vector<Polygon> &margined_obstacle_list,
                                       const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate) {
    std::vector<Point> free_points;
    // Remove Points inside Obstacles
    bool valid = true;
    for (int i = 0; i < points.size(); i++) {
        valid = true;
        for (int j = 0; j < margined_obstacle_list.size(); j++) {
            cv::Point point = MyUtils::cvPoint(MyUtils::scalePoint(points[i], show_scale));

            std::vector<cv::Point> obstacle =
                MyUtils::cvPoly(MyUtils::scalePoly(margined_obstacle_list[j], show_scale));

            if (cv::pointPolygonTest(obstacle, point, false) >= 0) {
                valid = false;
                break;
            }
        }
        if (valid) {
            free_points.push_back(points[i]);
        }
    }

    /*
    // Remove Points inside Victims
    points = free_points;
    free_points.clear();
    valid = true;
    for (int i = 0; i < points.size(); i++) {
        valid = true;
        for (int j = 0; j < victim_list.size(); j++) {
            cv::Point point = MyUtils::cvPoint(MyUtils::scalePoint(points[i], show_scale));

            std::vector<cv::Point> obstacle = MyUtils::cvPoly(MyUtils::scalePoly(victim_list[j].second, show_scale));

            if (cv::pointPolygonTest(obstacle, point, false) >= 0) {
                valid = false;
                break;
            }
        }
        if (valid) {
            free_points.push_back(points[i]);
        }
    }

    // Remove Points inside Gate
    points = free_points;
    free_points.clear();
    for (int i = 0; i < points.size(); i++) {
        cv::Point point = MyUtils::cvPoint(MyUtils::scalePoint(points[i], show_scale));
        std::vector<cv::Point> obstacle = MyUtils::cvPoly(MyUtils::scalePoly(gate, show_scale));
        if (cv::pointPolygonTest(obstacle, point, false) < 0) {
            free_points.push_back(points[i]);
        }
    }
*/
    return free_points;
}

void Plan::createGraph(cv::Mat &map, std::vector<Point> halton_points,
                       const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                       const float y) {
    for (int i = 0; i < halton_points.size(); i++) {
        Point point = halton_points[i];
        int x_section = point.x * ((double)MAP_SECTIONS - 0.0001) / map_width;
        int y_section = point.y * ((double)MAP_SECTIONS - 0.0001) / map_height;

        Node *node = new Node(i, point.x, point.y);
        nodes.push_back(node);

        section_to_points[x_section][y_section].push_back(node);
        point_to_section[node] = std::make_pair(x_section, y_section);
    }

    // Add One node for each Victim, Robot, and Gate

    Point gate_center = MyUtils::polyCenter(gate);
    gate_node = new Node(-1, gate_center.x, gate_center.y);
    int x_section = gate_center.x * ((double)MAP_SECTIONS - 0.0001) / map_width;
    int y_section = gate_center.y * ((double)MAP_SECTIONS - 0.0001) / map_height;
    section_to_points[x_section][y_section].push_back(gate_node);
    point_to_section[gate_node] = std::make_pair(x_section, y_section);
    nodes.push_back(gate_node);

    robot_node = new Node(-2, x, y);
    x_section = x * ((double)MAP_SECTIONS - 0.0001) / map_width;
    y_section = y * ((double)MAP_SECTIONS - 0.0001) / map_height;
    section_to_points[x_section][y_section].push_back(robot_node);
    point_to_section[robot_node] = std::make_pair(x_section, y_section);
    nodes.push_back(robot_node);

    // Sort Victims
    std::vector<std::pair<int, Polygon>> victims = victim_list;
    std::sort(victims.begin(), victims.end(), victimLessThan);

    victim_nodes.clear();
    for (int i = 0; i < victims.size(); i++) {
        Point victim_center = MyUtils::polyCenter(victims[i].second);
        Node *victim_node = new Node(-3 - victims[i].first, victim_center.x, victim_center.y);
        int x_section = victim_center.x * ((double)MAP_SECTIONS - 0.0001) / map_width;
        int y_section = victim_center.y * ((double)MAP_SECTIONS - 0.0001) / map_height;
        section_to_points[x_section][y_section].push_back(victim_node);
        point_to_section[victim_node] = std::make_pair(x_section, y_section);
        victim_nodes.push_back(victim_node);
    }

    nodes.insert(nodes.end(), victim_nodes.begin(), victim_nodes.end());

    // Find neighbours
    for (int i = 0; i < nodes.size(); i++) {
        Node *node = nodes[i];
        int x_section = point_to_section[node].first;
        int y_section = point_to_section[node].second;

        // Find possible neighbours
        std::vector<Node *> possible_neighbours;
        for (int x_sec_it = std::max(x_section - 1, 0); x_sec_it <= std::min(x_section + 1, MAP_SECTIONS - 1);
             x_sec_it++) {
            for (int y_sec_it = std::max(y_section - 1, 0); y_sec_it <= std::min(y_section + 1, MAP_SECTIONS - 1);
                 y_sec_it++) {
                possible_neighbours.insert(possible_neighbours.end(), section_to_points[x_sec_it][y_sec_it].begin(),
                                           section_to_points[x_sec_it][y_sec_it].end());
            }
        }

        // Remove the node itself
        possible_neighbours.erase(std::remove(possible_neighbours.begin(), possible_neighbours.end(), node),
                                  possible_neighbours.end());

        // Keep nearest nodes
        for (int j = 0; j < possible_neighbours.size(); j++) {
            Node *nei = possible_neighbours[j];
            nei->distance = MyUtils::pointDistance(Point(node->x, node->y), Point(nei->x, nei->y));
        }
        std::sort(possible_neighbours.begin(), possible_neighbours.end(), nodeLessThan);

        node->neighbours.insert(
            node->neighbours.end(), possible_neighbours.begin(),
            possible_neighbours.begin() + std::min(MAX_K_NEIGHBOURS, (int)possible_neighbours.size()));

        for (int j = 0; j < node->neighbours.size(); j++) {
            cv::line(map, cv::Point(node->neighbours[j]->x * show_scale, node->neighbours[j]->y * show_scale),
                     cv::Point(node->x * show_scale, node->y * show_scale), MY_COLOR_BLUE, 1);
        }

        // TODO: Remove Collisions
    }
};

std::vector<Node *> Plan::findRoute(Node *source, Node *target) {
    std::vector<Node *> visited_list;
    for (int i = 0; i < nodes.size(); i++) {
        nodes[i]->visited = false;
        nodes[i]->parent = NULL;
        nodes[i]->distance = INF_DIST;
        nodes[i]->in_q = 0;
    }

    source->visited = true;
    source->parent = source;
    source->distance = 0;
    source->in_q = true;

    std::vector<Node *> q;
    q.push_back(source);
    while (q.size()) {
        std::sort(q.begin(), q.end(), nodeGreaterThan);
        Node *current = q[q.size() - 1];
        current->visited = true;
        q.pop_back();

        for (int i = 0; i < current->neighbours.size(); i++) {
            Node *nei = current->neighbours[i];

            if (!nei->visited) {
                double distance = MyUtils::pointDistance(Point(current->x, current->y), Point(nei->x, nei->y));
                if (current->distance + distance < nei->distance) {
                    nei->distance = current->distance + distance;
                    nei->parent = current;
                }
                if (!nei->in_q) {
                    q.push_back(nei);
                    nei->in_q = true;
                }
            }
        }
    }
    std::vector<Node *> route;

    Node *current = target;
    while (current != source) {
        route.push_back(current);
        current = current->parent;
    }
    route.push_back(current);
    std::reverse(route.begin(), route.end());
    return route;
};

void Plan::drawRoute(cv::Mat &img, std::vector<Node *> route) {
    Node *current = route[0];
    for (int i = 1; i < route.size(); i++) {
        cv::Point a = cv::Point(current->x * show_scale, current->y * show_scale);
        cv::Point b = cv::Point(route[i]->x * show_scale, route[i]->y * show_scale);
        cv::line(img, a, b, MY_COLOR_WHITE, 3);
        current = route[i];
    }
}

void Plan::initialize(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                      const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                      const float y, const float theta, Path &path, const std::string &config_folder) {
    cv::Mat map;

    // Create an initial Map
    initMap(map, borders, obstacle_list, victim_list, gate, x, y, theta);

    // Create map rectangle object
    Polygon borders_scaled = MyUtils::scalePoly(borders, show_scale);
    cv::Rect map_rect_scaled = cv::boundingRect(MyUtils::cvPoly(borders_scaled));
    map_width = (double)map_rect_scaled.width / show_scale;
    map_height = (double)map_rect_scaled.height / show_scale;
    showMap(map);

    // Make margin for Obstacles
    std::vector<Polygon> margined_obstacle_list = marginObstacles(obstacle_list);
    // Draw Margined Obstacles
    for (int i = 0; i < margined_obstacle_list.size(); i++) {
        MyUtils::drawPoly(map, margined_obstacle_list[i], show_scale, cv::Scalar(0, 0, 130), 1);
    }
    showMap(map);

    // Get Halton Points
    std::vector<Point> halton_points = getHaltonPoints(NUM_HALTON_POINTS, map_width, map_height);
    // Draw Halton Points
    MyUtils::drawPoints(map, halton_points, show_scale, cv::Scalar(0, 255, 255), 1);
    showMap(map);

    // Remove points with collision
    halton_points = getFreePoints(halton_points, margined_obstacle_list, victim_list, gate);
    MyUtils::drawPoints(map, halton_points, show_scale, MY_COLOR_WHITE, 2);
    showMap(map);

    // Create Graph nodes and map sections
    createGraph(map, halton_points, victim_list, gate, x, y);
    showMap(map);

    // Find Route
    std::cout << victim_nodes.size() << "\n";
    std::cout << victim_list.size() << "\n";

    if (victim_nodes.size()) {
        Node *current = robot_node;
        for (int i = 0; i < victim_nodes.size(); i++) {
            std::vector<Node *> route = findRoute(current, victim_nodes[i]);
            drawRoute(map, route);
            current = victim_nodes[i];
            showMap(map);
        }
        std::vector<Node *> route = findRoute(current, gate_node);
        drawRoute(map, route);
        showMap(map);
    }

    // Draw Route

    // TODO: Smooth
}

void Plan::showMap(cv::Mat map) {
    cv::imshow("Map", map);
    cv::waitKey(0);
    cv::destroyWindow("Map");
}