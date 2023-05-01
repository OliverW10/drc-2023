#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "controller.hpp"


Eigen::Matrix3d intrinsics;

// takes input from sensors and produces a map of the enviroment
class Vision{
public:
    Vision();
    CarState process(const cv::Mat& image, CarState cur_state);
private:
    int frame_counter = 0;
    double process_total = 0;
    cv::Mat track_map;
    cv::Mat perspective_transform;
};

#endif