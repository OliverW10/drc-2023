#ifndef ARROW_H
#define ARROW_H

#include <opencv2/core/mat.hpp>

double find_arrow(const cv::Mat& hsv_ground, double& out);

#endif