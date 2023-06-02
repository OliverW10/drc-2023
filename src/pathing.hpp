#ifndef PATHING_H
#define PATHING_H
#include <opencv2/core/mat.hpp>
#include <Eigen/Core>

namespace pathing{

// get samples along the arc in map pixels
std::vector<cv::Point> getArcPixels(
    const Eigen::Vector3d& start, double curvature, double dist, int num_samples = 50
);

// find arc that best follows the ridge
double getBestCurvature(
    const cv::Mat& track_map,
    const Eigen::Vector3d& start,
    double max_curvature,
    double arc_dist,
    double bias_center,
    double bias_strength
);

}
#endif