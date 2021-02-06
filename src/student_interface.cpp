#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <experimental/filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "map.hpp"
#include "my_utils.hpp"
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "transform.hpp"

const static struct Show {
    bool robot = true;
    bool victims = false;
    bool gate = false;
    bool obstacles = false;
} show;

void createCalibrationXML(const std::string save_dir, int last_image, const std::string &config_folder) {
    std::ofstream xml_file;
    std::stringstream ss;
    ss << config_folder << "/list.xml";
    xml_file.open(ss.str(), std::ios::out);

    xml_file << "<?xml version=\"1.0\"?>" << std::endl << "<opencv_storage>" << std::endl << "<images>" << std::endl;

    for (int i = 0; i <= last_image; i++) {
        ss.str("");
        ss << save_dir << "/" << std::setw(3) << std::setfill('0') << i << ".jpg";
        xml_file << ss.str() << std::endl;
    }

    xml_file << "</images>" << std::endl << "</opencv_storage>" << std::endl;
    xml_file.close();
}

static MyUtils utils;
static Transform transform;
static Map map;

namespace student {

void loadImage(cv::Mat &img_out, const std::string &config_folder) {
    static bool initialized = false;
    static std::vector<cv::String> img_list;     // list of images to load
    static size_t idx = 0;                       // idx of the current img
    static size_t function_call_counter = 0;     // idx of the current img
    const static size_t freeze_img_n_step = 30;  // hold the current image for n iteration
    static cv::Mat current_img;                  // store the image for a period, avoid to load it from file every time

    if (!initialized) {
        const bool recursive = false;
        // Load the list of jpg image contained in the config_folder/img_to_load/
        cv::glob(config_folder + "/img_to_load/*.jpg", img_list, recursive);

        if (img_list.size() > 0) {
            initialized = true;
            idx = 0;
            current_img = cv::imread(img_list[idx]);
            function_call_counter = 0;
        } else {
            initialized = false;
        }
    }

    if (!initialized) {
        throw std::logic_error("Load Image can not find any jpg image in: " + config_folder + "/img_to_load/");
        return;
    }

    img_out = current_img;
    function_call_counter++;

    // If the function is called more than N times load increment image idx
    if (function_call_counter > freeze_img_n_step) {
        function_call_counter = 0;
        idx = (idx + 1) % img_list.size();
        current_img = cv::imread(img_list[idx]);
    }
}

void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder) {
    static bool init = false;
    static std::string save_dir;
    static int image_number = 0;

    if (!init) {
        std::cout << "Press <c> to save image for calibration.\n";

        bool exists = 1;
        int folder_number = 0;
        while (exists && folder_number < 1000) {
            std::stringstream ss;
            ss << config_folder << "/calibration_images_" << std::setw(3) << std::setfill('0') << folder_number;
            save_dir = ss.str();

            exists = utils.pathExists(save_dir);
            folder_number++;
        }

        if (!std::experimental::filesystem::create_directories(save_dir)) {
            throw std::logic_error("Cannot Create Dir\n");
        }
        if (folder_number >= 1000) {
            throw std::logic_error("Something went wrong in processing folders.\n");
        }
        init = true;
    }

    cv::imshow(topic, img_in);
    char c;
    c = cv::waitKey(30);

    std::string save_file;
    std::stringstream ss;

    switch (c) {
        case 's':  // TODO: Normal Save
            break;

        case 'c':
            ss << save_dir << "/" << std::setw(3) << std::setfill('0') << image_number << ".jpg";
            save_file = ss.str();
            cv::imwrite(save_file, img_in);
            std::cout << save_file << " Saved." << std::endl;
            createCalibrationXML(save_dir, image_number, config_folder);
            image_number++;
            break;

        default:
            break;
    }
}

//////////////////////////////////////////////////////////////////
/*                        Transforms                            */
//////////////////////////////////////////////////////////////////

void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs,
                    const std::string &config_folder) {
    // TODO: convert intrinsic_calibration.xml to camera_params.config automaticaly
    /*if (std::experimental::filesystem::exists(config_folder + "/img_to_load/")) {
        img_out = img_in;
        return;
    }*/
    if (!transform.isInitialized()) {
        transform.initialize(img_in.size(), cam_matrix, dist_coeffs);
    }

    transform.undistort(img_in, img_out);
}

bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                    cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder) {
    std::string file_path = config_folder + "/extrinsicCalib.csv";
    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path)) {
        std::experimental::filesystem::create_directories(config_folder);

        image_points = utils.pickNPoints(4, img_in, " For Extrinsic Calibration");
        std::ofstream output(file_path);
        if (!output.is_open()) {
            throw std::runtime_error("Cannot write file: " + file_path);
        }
        for (const auto pt : image_points) {
            output << pt.x << " " << pt.y << std::endl;
        }
        output.close();
    }

    return transform.extrinsicCalib(img_in, object_points, camera_matrix, rvec, tvec, config_folder);
}

void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec,
                        const std::vector<cv::Point3f> &object_points_plane,
                        const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf,
                        const std::string &config_folder) {
    transform.findPlaneTransform(cam_matrix, rvec, tvec, object_points_plane, dest_image_points_plane, plane_transf,
                                 config_folder);
}

void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder) {
    transform.unwarp(img_in, img_out, transf, config_folder);
}

//////////////////////////////////////////////////////////////////
/*                             Map                              */
//////////////////////////////////////////////////////////////////

bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list,
                std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder) {
    static cv::Mat obstacle_mask, victim_mask, gate_mask;
    static std::vector<MaskBound> obstacle_mask_bounds, victim_mask_bounds, gate_mask_bounds;

    cv::Mat gaussian;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2 * 2) + 1, (2 * 2) + 1));

    cv::GaussianBlur(img_in, gaussian, cv::Size(5, 5), 0);
    cv::dilate(gaussian, gaussian, kernel);
    cv::erode(gaussian, gaussian, kernel);

    if (!map.isInitialized()) {
        map.initialize(img_in, config_folder);
        map.getObstaclesBounds(img_in, obstacle_mask_bounds, config_folder);
        map.getVictimsBounds(img_in, victim_mask_bounds, config_folder);
        map.getGateBounds(img_in, gate_mask_bounds, config_folder);
    }

    map.applyMasks(gaussian, obstacle_mask_bounds, obstacle_mask);
    map.applyMasks(gaussian, victim_mask_bounds, victim_mask);
    map.applyMasks(gaussian, gate_mask_bounds, gate_mask);

    map.findObstacles(obstacle_mask, obstacle_list, scale);
    map.findVictims(victim_mask, victim_list, scale);
    map.findGate(gate_mask, gate, scale);

    if (show.victims) {
        cv::imshow("victims mask", victim_mask);
        cv::waitKey(1);
    }
    if (show.gate) {
        cv::imshow("gate mask", gate_mask);
        cv::waitKey(1);
    }
    if (show.obstacles) {
        cv::imshow("obstacles mask", obstacle_mask);
        cv::waitKey(1);
    }
    return true;
}

bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta,
               const std::string &config_folder) {
    static cv::Mat robot_mask;
    static std::vector<MaskBound> robot_mask_bounds;
    if (!map.isInitialized()) {
        map.initialize(img_in, config_folder);
        map.getRobotBounds(img_in, robot_mask_bounds, config_folder);
    }

    cv::Mat gaussian;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2 * 2) + 1, (2 * 2) + 1));

    cv::GaussianBlur(img_in, gaussian, cv::Size(5, 5), 0);
    cv::dilate(gaussian, gaussian, kernel);
    cv::erode(gaussian, gaussian, kernel);

    map.applyMasks(gaussian, robot_mask_bounds, robot_mask);

    if (map.findRobot(robot_mask, triangle, scale)) {
        x = 0;
        y = 0;
        for (int i = 0; i < 3; i++) {
            x += triangle[i].x;
            y += triangle[i].y;
        }
        x /= 3;
        y /= 3;

        double d[3];
        d[0] = utils.pointDistance(triangle[0], Point(x, y));
        d[1] = utils.pointDistance(triangle[1], Point(x, y));
        d[2] = utils.pointDistance(triangle[2], Point(x, y));

        int max_el = std::max_element(d, d + 3) - d;
        std::swap(triangle[0], triangle[max_el]);

        if (show.robot) {
            cv::circle(img_in, cv::Point(triangle[0].x * scale, triangle[0].y * scale), 6, cv::Scalar(255, 0, 255), 1);
            cv::circle(img_in, cv::Point(triangle[1].x * scale, triangle[1].y * scale), 4, cv::Scalar(255, 0, 255), 1);
            cv::circle(img_in, cv::Point(triangle[2].x * scale, triangle[2].y * scale), 2, cv::Scalar(255, 0, 255), 1);
            cv::circle(img_in, cv::Point(x * scale, y * scale), 2, cv::Scalar(255, 0, 255), 5);
            cv::imshow("Robot", img_in);
            cv::waitKey(1);
        }

        theta = std::atan2(y - triangle[0].y, x - triangle[0].x);
        return true;
    } else {
        return false;
    }
}

bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
              const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
              const float y, const float theta, Path &path) {
    throw std::logic_error("STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED");
}

}  // namespace student
   // student
