#include "plan.hpp"

#include <bitset>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "dubins.hpp"
#include "halton.hpp"
#include "my_utils.hpp"
#include "utils.hpp"

#define INF_DIST 100000

Plan::Plan(){

};

Plan::~Plan(){

};

double map_width;
double map_height;

std::vector<Node *> nodes;
Node *gate_node;
Node *robot_node;
std::vector<Node *> victim_nodes;
std::vector<Node *> section_to_points[MAP_SECTIONS][MAP_SECTIONS];
std::map<Node *, std::pair<int, int>> point_to_section;

bool nodeLessThan(Node *a, Node *b) { return a->distance < b->distance; }

bool nodeGreaterThan(Node *a, Node *b) { return a->distance > b->distance; }

bool victimLessThan(std::pair<int, Polygon> a, std::pair<int, Polygon> b) { return a.first < b.first; }

void Plan::initMap(cv::Mat &img, const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                   const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                   const float y, const float theta) {
    Polygon borders_scaled = MyUtils::scalePoly(borders, SHOW_SCALE);
    Polygon gate_scaled = MyUtils::scalePoly(gate, SHOW_SCALE);

    // Create a free map for further visualization
    cv::Rect map_rect_scaled = cv::boundingRect(MyUtils::cvPoly(borders_scaled));
    cv::Mat map(map_rect_scaled.height, map_rect_scaled.width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Draw Gate on Map
    MyUtils::drawPoly(map, gate, SHOW_SCALE, cv::Scalar(0, 255, 0), 2);

    // Draw Victims on Map
    for (int i = 0; i < victim_list.size(); i++) {
        MyUtils::drawPoly(map, victim_list[i].second, SHOW_SCALE, cv::Scalar(255, 255, 0), 1);

        Point victim_center = MyUtils::polyCenter(victim_list[i].second);
        cv::putText(map, std::to_string(victim_list[i].first),
                    cv::Point2f(victim_center.x * SHOW_SCALE, victim_center.y * SHOW_SCALE), cv::FONT_HERSHEY_PLAIN, 2,
                    cv::Scalar(255, 255, 0));
    }

    // Draw Robot
    cv::circle(map, cv::Point2f(x * SHOW_SCALE, y * SHOW_SCALE), 0.05 * SHOW_SCALE, cv::Scalar(255, 0, 0), 1);
    cv::circle(map, cv::Point2f((x + 0.05 * std::cos(theta)) * SHOW_SCALE, (y + 0.05 * std::sin(theta)) * SHOW_SCALE),
               0.02 * SHOW_SCALE, cv::Scalar(255, 0, 0), 1);

    // Draw Obstacles
    for (int i = 0; i < obstacle_list.size(); i++) {
        MyUtils::drawPoly(map, obstacle_list[i], SHOW_SCALE, cv::Scalar(0, 0, 255), 1);
    }

    img = map.clone();
}

std::vector<Polygon> Plan::marginObstacles(std::vector<Polygon> obstacles) {
    std::vector<Polygon> margined_obstacle_list;
    for (int i = 0; i < obstacles.size(); i++) {
        Polygon obstacle = obstacles[i];
        Polygon margined_obstacle;

        // cv::Moments obstacle_moment = cv::moments(MyUtils::cvPoly(obstacle), false);
        Point obstacle_center = /* Point(obstacle_moment.m10 / obstacle_moment.m00,
                                       obstacle_moment.m01 / obstacle_moment.m00);  */
            MyUtils::polyCenter(obstacle);

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
            cv::Point point = MyUtils::cvPoint(MyUtils::scalePoint(points[i], SHOW_SCALE));

            std::vector<cv::Point> obstacle =
                MyUtils::cvPoly(MyUtils::scalePoly(margined_obstacle_list[j], SHOW_SCALE));

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
            cv::Point point = MyUtils::cvPoint(MyUtils::scalePoint(points[i], SHOW_SCALE));

            std::vector<cv::Point> obstacle = MyUtils::cvPoly(MyUtils::scalePoly(victim_list[j].second, SHOW_SCALE));

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
        cv::Point point = MyUtils::cvPoint(MyUtils::scalePoint(points[i], SHOW_SCALE));
        std::vector<cv::Point> obstacle = MyUtils::cvPoly(MyUtils::scalePoly(gate, SHOW_SCALE));
        if (cv::pointPolygonTest(obstacle, point, false) < 0) {
            free_points.push_back(points[i]);
        }
    }
*/
    return free_points;
}

void Plan::createGraph(cv::Mat &map, std::vector<Point> halton_points,
                       const std::vector<Polygon> &margined_obstacle_list,
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

    // Add One node for Gate
    Point gate_center = MyUtils::polyCenter(gate);
    gate_node = new Node(-1, gate_center.x, gate_center.y);
    int x_section = gate_center.x * ((double)MAP_SECTIONS - 0.0001) / map_width;
    int y_section = gate_center.y * ((double)MAP_SECTIONS - 0.0001) / map_height;
    section_to_points[x_section][y_section].push_back(gate_node);
    point_to_section[gate_node] = std::make_pair(x_section, y_section);
    nodes.push_back(gate_node);

    // Add One node for Robot
    robot_node = new Node(-2, x, y);
    x_section = x * ((double)MAP_SECTIONS - 0.0001) / map_width;
    y_section = y * ((double)MAP_SECTIONS - 0.0001) / map_height;
    section_to_points[x_section][y_section].push_back(robot_node);
    point_to_section[robot_node] = std::make_pair(x_section, y_section);
    nodes.push_back(robot_node);

    // Sort Victims
    std::vector<std::pair<int, Polygon>> victims = victim_list;
    std::sort(victims.begin(), victims.end(), victimLessThan);
    // Add victim Nodes
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
                for (int k = 0; k < section_to_points[x_sec_it][y_sec_it].size(); k++) {
                    Node *nei = section_to_points[x_sec_it][y_sec_it][k];
                    if (!MyUtils::segPolysCollision(Point(node->x, node->y), Point(nei->x, nei->y),
                                                    margined_obstacle_list)) {
                        possible_neighbours.push_back(nei);
                    }
                }
            }
        }

        // Remove the node itself
        possible_neighbours.erase(std::remove(possible_neighbours.begin(), possible_neighbours.end(), node),
                                  possible_neighbours.end());

        // Keep nearest neighbours
        for (int j = 0; j < possible_neighbours.size(); j++) {
            Node *nei = possible_neighbours[j];
            nei->distance = MyUtils::pointDistance(Point(node->x, node->y), Point(nei->x, nei->y));
        }
        std::sort(possible_neighbours.begin(), possible_neighbours.end(), nodeLessThan);

        node->neighbours.insert(
            node->neighbours.end(), possible_neighbours.begin(),
            possible_neighbours.begin() + std::min(MAX_K_NEIGHBOURS, (int)possible_neighbours.size()));

        // Draw Neighbours
        for (int j = 0; j < node->neighbours.size(); j++) {
            cv::line(map, cv::Point(node->neighbours[j]->x * SHOW_SCALE, node->neighbours[j]->y * SHOW_SCALE),
                     cv::Point(node->x * SHOW_SCALE, node->y * SHOW_SCALE), cv::Scalar(100, 0, 0), 1);
        }
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
        if(current==target){
            break;
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

void Plan::drawRoute(cv::Mat &img, std::vector<Node *> route, int thickness) {
    Node *current = route[0];
    for (int i = 1; i < route.size(); i++) {
        cv::Point a = cv::Point(current->x * SHOW_SCALE, current->y * SHOW_SCALE);
        cv::Point b = cv::Point(route[i]->x * SHOW_SCALE, route[i]->y * SHOW_SCALE);
        cv::line(img, a, b, MY_COLOR_WHITE, thickness);
        current = route[i];
    }
}

std::vector<Node *> Plan::smoothRoute(std::vector<Node *> route, const std::vector<Polygon> &margined_obstacle_list) {
    std::vector<Node *> new_route;
    if (route.size() == 2) {
        return route;
    }
    Node *source = route[0];
    Node *target = route[route.size() - 1];
    if (!MyUtils::segPolysCollision(Point(source->x, source->y), Point(target->x, target->y), margined_obstacle_list)) {
        new_route.push_back(source);
        new_route.push_back(target);
        return new_route;
    }
    int half = route.size() / 2;
    std::vector<Node *> first_half;
    std::vector<Node *> second_half;
    first_half.insert(first_half.end(), route.begin(), route.begin() + half + 1);
    second_half.insert(second_half.end(), route.begin() + half, route.end());
    first_half = smoothRoute(first_half, margined_obstacle_list);
    second_half = smoothRoute(second_half, margined_obstacle_list);

    new_route.insert(new_route.end(), first_half.begin(), first_half.end());
    new_route.insert(new_route.end(), second_half.begin() + 1, second_half.end());
    return new_route;
}

int createDubinsSteps(double q[3], double x, void *user_data) {
    std::vector<Pose> *samples = (std::vector<Pose> *)user_data;
    samples->push_back(Pose(x, q[0], q[1], q[2], K_P));
    return 0;
}

void Plan::plan(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                const float y, const float theta, Path &path, const std::string &config_folder) {
    cv::Mat map;

    // Create an initial Map
    initMap(map, borders, obstacle_list, victim_list, gate, x, y, theta);

    // Create map rectangle object
    Polygon borders_scaled = MyUtils::scalePoly(borders, SHOW_SCALE);
    cv::Rect map_rect_scaled = cv::boundingRect(MyUtils::cvPoly(borders_scaled));
    map_width = (double)map_rect_scaled.width / SHOW_SCALE;
    map_height = (double)map_rect_scaled.height / SHOW_SCALE;
    showMap(map);

    // Make margin for Obstacles
    std::vector<Polygon> margined_obstacle_list = marginObstacles(obstacle_list);
    // Draw Margined Obstacles
    for (int i = 0; i < margined_obstacle_list.size(); i++) {
        MyUtils::drawPoly(map, margined_obstacle_list[i], SHOW_SCALE, cv::Scalar(0, 0, 130), 1);
    }
    showMap(map);

    // Get Halton Points
    std::vector<Point> halton_points = getHaltonPoints(NUM_HALTON_POINTS, map_width, map_height);
    // Draw Halton Points
    // MyUtils::drawPoints(map, halton_points, SHOW_SCALE, cv::Scalar(0, 255, 255), 1);
    // showMap(map);

    // Remove points with collision
    halton_points = getFreePoints(halton_points, margined_obstacle_list, victim_list, gate);
    MyUtils::drawPoints(map, halton_points, SHOW_SCALE, cv::Scalar(180, 180, 180), 2);
    showMap(map);

    // Create Graph nodes and map sections
    createGraph(map, halton_points, margined_obstacle_list, victim_list, gate, x, y);
    showMap(map);

    std::vector<std::vector<Node *>> route_sections;
    if (MISSION == 1) {
        // Create list of check_points
        std::vector<Node *> checkpoints;
        checkpoints.push_back(robot_node);
        for (int i = 0; i < victim_nodes.size(); i++) {
            checkpoints.push_back(victim_nodes[i]);
        }
        checkpoints.push_back(gate_node);

        // Find Route between checkpoints and Draw it
        Node *current = checkpoints[0];
        for (int i = 1; i < checkpoints.size(); i++) {
            std::vector<Node *> tmp_route = findRoute(current, checkpoints[i]);
            route_sections.push_back(tmp_route);
            drawRoute(map, tmp_route, 2);
            current = checkpoints[i];
            showMap(map);
        }
    } else if (MISSION == 2) { /*
         int num_of_vics = victim_nodes.size();
         std::bitset<10> victims_in_route = 0;
         for (int i = 0; i < (1 << num_of_vics); i++) {
             std::vector<Node *> tmp_victim_nodes;
             for (int j = 0; j < num_of_vics; j++) {

             }
         }*/
        // TODO
    }

    // Smooth
    for (int i = 0; i < route_sections.size(); i++) {
        std::vector<Node *> route = route_sections[i];
        route = smoothRoute(route, margined_obstacle_list);
        route = smoothRoute(route, margined_obstacle_list);
        route = smoothRoute(route, margined_obstacle_list);
        route_sections[i] = route;
        drawRoute(map, route, 3);
        showMap(map);
    }

    // Merge all routes to single list of nodes
    std::vector<Node *> route;
    route.push_back(route_sections[0][0]);
    for (int i = 0; i < route_sections.size(); i++) {
        route.insert(route.end(), route_sections[i].begin() + 1, route_sections[i].end());
    }

    // Create Actual Path using Dubins curves
    double begin_ang = theta;
    double end_ang = 0;
    Node *current = route[0];
    for (int i = 1; i < route.size(); i++) {
        Node *target = route[i];

        Node *next_target = i < route.size() - 1 ? route[i + 1] : NULL;
        end_ang = next_target != NULL ? std::atan2(next_target->y - target->y, next_target->x - target->x) : begin_ang;

        DubinsPath my_path;
        double a[3] = {current->x, current->y, begin_ang};
        double b[3] = {target->x, target->y, end_ang};
        dubins_shortest_path(&my_path, a, b, 1.0 / K_P);

        std::vector<Pose> my_path_samples;
        dubins_path_sample_many(&my_path, 0.001, createDubinsSteps, &my_path_samples);

        int path_type = dubins_path_type(&my_path);
        for (int k = 0; k < my_path_samples.size(); k++) {
            if (my_path_samples[k].s <= dubins_segment_length(&my_path, 0)) {
                if (path_type >= 2 && path_type <= 4) {
                    my_path_samples[k].kappa = -my_path_samples[k].kappa;
                }
            } else if (my_path_samples[k].s <=
                       dubins_segment_length(&my_path, 0) + dubins_segment_length(&my_path, 1)) {
                if (path_type >= 0 && path_type <= 3) {
                    my_path_samples[k].kappa = 0;
                } else if (path_type == 5) {
                    my_path_samples[k].kappa = -my_path_samples[k].kappa;
                }
            } else {
                if (path_type == 1 || path_type == 3 || path_type == 4) {
                    my_path_samples[k].kappa = -my_path_samples[k].kappa;
                }
            }
        }

        path.points.insert(path.points.end(), my_path_samples.begin(), my_path_samples.end());

        begin_ang = end_ang;
        current = target;
    }
}

void Plan::showMap(cv::Mat map) {
    cv::imshow("Map", map);
    cv::waitKey(0);
    cv::destroyWindow("Map");
}