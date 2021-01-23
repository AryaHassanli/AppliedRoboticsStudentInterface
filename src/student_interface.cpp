#include <sys/stat.h>
#include <unistd.h>

#include <experimental/filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

inline bool pathExists(const std::string &name) {
    std::ifstream f(name.c_str());
    return f.good();
}

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

            exists = pathExists(save_dir);
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
        case 's': //TODO: Normal Save
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

void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs,
                    const std::string &config_folder) {

    // TODO: convert intrinsic_calibration.xml to camera_params.config automaticaly

    static bool maps_initialized = false;
    static cv::Mat full_map1, full_map2;

    if (!maps_initialized) {
        cv::Mat R;
        cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, full_map1,
                                    full_map2);
        maps_initialized = true;
    }

    cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);
}

bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                    cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder) {
    throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED");
}

void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec,
                        const std::vector<cv::Point3f> &object_points_plane,
                        const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf,
                        const std::string &config_folder) {
    throw std::logic_error("STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED");
}

void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder) {
    throw std::logic_error("STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED");
}

bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list,
                std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder) {
    throw std::logic_error("STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED");
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
