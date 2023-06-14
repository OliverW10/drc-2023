#ifndef ARROW_H
#define ARROW_H

#include <opencv2/core/mat.hpp>

void find_arrow(const cv::Mat& hsv_ground, double& confidence_out);

#endif