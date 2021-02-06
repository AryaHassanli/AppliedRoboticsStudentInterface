#include "transform.hpp"

#include <experimental/filesystem>

Transform::Transform() { is_initialized_ = false; };

bool Transform::isInitialized() const { return is_initialized_; }

bool Transform::initialize(const cv::Size &img_size, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs) {
    if (is_initialized_) {
        return true;
    }
    cv::Mat R;
    cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_size, CV_16SC2, full_map1_, full_map2_);
    cam_matrix_ = cam_matrix.clone();
    dist_coeffs_ = dist_coeffs.clone();
    is_initialized_ = true;
    return is_initialized_;
}

bool Transform::undistort(const cv::Mat &img_in, cv::Mat &img_out) {
    if (!isInitialized()) {
        return false;
    }
    cv::remap(img_in, img_out, full_map1_, full_map2_, cv::INTER_LINEAR);
    return true;
}

bool Transform::extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points,
                               const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec,
                               const std::string &config_folder) {
    std::string file_path = config_folder + "/extrinsicCalib.csv";
    std::vector<cv::Point2f> image_points;

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

    cv::Mat dist_coeffs;
    dist_coeffs = (cv::Mat1d(1, 4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!ok) {
        std::cerr << "FAILED SOLVE_PNP" << std::endl;
    }

    return ok;
}

void Transform::findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec,
                                   const std::vector<cv::Point3f> &object_points_plane,
                                   const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf,
                                   const std::string &config_folder) {
    cv::Mat image_points;

    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
}

void Transform::unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
                       const std::string &config_folder) {
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
}
Transform::~Transform(){};