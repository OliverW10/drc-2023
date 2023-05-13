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
    std::chrono::high_resolution_clock::time_point m_last_time;
};

#endif