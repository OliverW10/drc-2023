#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <opencv2/core/mat.hpp>

void find_obsticles(
    const cv::Mat& hsv_ground,
    cv::Mat& purple_mask,
    cv::Mat& red_mask,
    cv::Mat& purple_obstacle,
    cv::Mat& red_obstacle,
    cv::Mat& output
);

#endif