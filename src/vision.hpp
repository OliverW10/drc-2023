#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "controller.hpp"

cv::Point posToMap(const Eigen::Vector3d& position);

// takes input from sensors and produces a map of the enviroment
class Vision{
public:
    Vision(int img_width, int img_height);
    CarState process(const cv::Mat& image, const SensorValues& sensor_input);
private:
    int m_frame_counter = 0;
    cv::Mat m_track_map;
    cv::Mat m_perspective_transform;
    // as members to reuse buffer between frames
    cv::Mat m_image_corrected,
        m_hsv_ground,
        m_mask_yellow,
        m_mask_blue,
        m_track_combined,
        m_track_annotated,
        m_track_yellow,
        m_track_blue;
    std::chrono::high_resolution_clock::time_point m_last_time;
};

const double pixels_per_meter = 100;
const double map_width  = 4;
const double map_height = 4;
const int map_width_p  = (int)(map_width  * pixels_per_meter);
const int map_height_p = (int)(map_height * pixels_per_meter);

cv::Scalar getConfigHsvScalarHigh(std::string name);
cv::Scalar getConfigHsvScalarLow(std::string name);

#endif