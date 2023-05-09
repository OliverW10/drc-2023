#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "controller.hpp"

// takes input from sensors and produces a map of the enviroment
class Vision{
public:
    Vision(int img_width, int img_height);
    CarState process(const cv::Mat& image, const SensorValues& sensor_input);
private:
    int frame_counter = 0;
    double process_total = 0;
    cv::Mat track_map;
    cv::Mat perspective_transform;
};

#endif