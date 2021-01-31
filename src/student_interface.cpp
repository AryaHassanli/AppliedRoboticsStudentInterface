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

namespace student {

static MyUtils utils;
static Transform transform;
static Map map;

void loadImage(cv::Mat &img_out, const std::string &config_folder) {
    throw std::logic_error("STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED");
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

        image_points = utils.pickNPoints(4, img_in, "");
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
    if (!map.is_initialized) {
        map.initialize();
        map.detectMasks(img_in, " For Obstacles", map.obstacle_mask_bounds);
        map.detectMasks(img_in, " For Victim", map.victim_mask_bounds);
        map.detectMasks(img_in, " For Gate", map.gate_mask_bounds);
        map.detectMasks(img_in, " For Robot", map.robot_mask_bounds);
    }

    cv::Mat gaussian;
    cv::GaussianBlur(img_in, gaussian, cv::Size(5, 5), 0);

    Map::applyMasks(gaussian, map.obstacle_mask_bounds, map.obstacle_mask);
    Map::applyMasks(gaussian, map.victim_mask_bounds, map.victim_mask);
    Map::applyMasks(gaussian, map.gate_mask_bounds, map.gate_mask);

    cv::imshow("Gaus Obs", map.obstacle_mask);
    cv::imshow("Gaus Gate", map.gate_mask);
    cv::imshow("Gaus Victim", map.victim_mask);
    
    cv::waitKey(20000);
    return true;
}

bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta,
               const std::string &config_folder) {
    throw std::logic_error("STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED");
}

bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
              const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
              const float y, const float theta, Path &path) {
    throw std::logic_error("STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED");
}

}  // namespace student
   // student
