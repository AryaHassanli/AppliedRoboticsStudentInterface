#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <experimental/filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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

//-------------------------------------------------------------------------
//          EXTRINSIC CALIB IMPLEMENTATION
//-------------------------------------------------------------------------

// Defintion of the function pickNPoints and the callback mouseCallback.
// The function pickNPoints is used to display a window with a background
// image, and to prompt the user to select n points on this image.
static cv::Mat bg_img;
static std::vector<cv::Point2f> result;
static std::string name;
static std::atomic<bool> done;
static int n;
static double show_scale = 1.0;

void mouseCallback(int event, int x, int y, int, void *p) {
    if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

    result.emplace_back(x * show_scale, y * show_scale);
    cv::circle(bg_img, cv::Point(x, y), 20 / show_scale, cv::Scalar(0, 0, 255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n) {
        usleep(500 * 1000);
        done.store(true);
    }
}

std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img) {
    result.clear();
    cv::Size small_size(img.cols / show_scale, img.rows / show_scale);
    cv::resize(img, bg_img, small_size);
    // bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load()) {
        cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());
    return result;
}

bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                    cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder) {
    std::string file_path = config_folder + "/extrinsicCalib.csv";

    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path)) {
        std::experimental::filesystem::create_directories(config_folder);

        image_points = pickNPoints(4, img_in);
        // SAVE POINT TO FILE
        // std::cout << "IMAGE POINTS: " << std::endl;
        // for (const auto pt: image_points) {
        //   std::cout << pt << std::endl;
        // }
        std::ofstream output(file_path);
        if (!output.is_open()) {
            throw std::runtime_error("Cannot write file: " + file_path);
        }
        for (const auto pt : image_points) {
            output << pt.x << " " << pt.y << std::endl;
        }
        output.close();
    } else {
        // LOAD POINT FROM FILE
        std::ifstream input(file_path);
        if (!input.is_open()) {
            throw std::runtime_error("Cannot read file: " + file_path);
        }
        while (!input.eof()) {
            double x, y;
            if (!(input >> x >> y)) {
                if (input.eof())
                    break;
                else {
                    throw std::runtime_error("Malformed file: " + file_path);
                }
            }
            image_points.emplace_back(x, y);
        }
        input.close();
    }

    cv::Mat dist_coeffs;
    dist_coeffs = (cv::Mat1d(1, 4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    // cv::Mat Rt;
    // cv::Rodrigues(rvec_, Rt);
    // auto R = Rt.t();
    // auto pos = -R * tvec_;

    if (!ok) {
        std::cerr << "FAILED SOLVE_PNP" << std::endl;
    }

    return ok;
}

void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec,
                        const std::vector<cv::Point3f> &object_points_plane,
                        const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf,
                        const std::string &config_folder) {
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
}

void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder) {
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
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
