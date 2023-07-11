#include <vector>
#include "pathing.hpp"
#include "vision.hpp"
#include "controller.hpp"
#include <stdint.h>
#include <math.h>
#include <iostream>

namespace pathing{

// get samples along the arc in map pixels
std::vector<cv::Point> getArcPixels(const Eigen::Vector3d& start, double curvature, double dist, int num_samples){
    std::vector<cv::Point> pixels(num_samples);
    // pixels forwards per sample
    double dx = dist/num_samples;
    for(int i = 0; i < num_samples; i++){
        double d = i*dx;
        pixels[i] = posToMap(getDistForwards(curvature, d, start));
    }
    return pixels;
}

// gets the average track weight along arc
double getArcFittness(const cv::Mat& track_map, const Eigen::Vector3d& pos, double curvature, double dist){
    float total = 0;
    int num_samples = 40;
    double dx = dist/num_samples;
    for(int i = 0; i < num_samples; i++){
        cv::Point point = posToMap(getDistForwards(curvature, i*dx, pos));
        if(point.x >= 0 && point.x < track_map.cols && point.y >= 0 && point.y < track_map.rows){
            total += pow(track_map.at<uint8_t>(point.y, point.x), 0.5);
        }
    }
    return total;
}

const double max_curvature = 1.0;

// find arc that best follows the ridge
double getBestCurvature(
    const cv::Mat& track_map,
    const Eigen::Vector3d& start,
    double arc_dist,
    double bias_center,
    double bias_strength
){
    double best_curvature = 0;
    double best_score = 0;
    int curves_num = 15;
    for(int i = 0; i < curves_num; i++){
        double p = ((double)i)/(curves_num-1);
        double curvature = max_curvature * (2.0*p -1.0);

        double current_score = getArcFittness(track_map, start, curvature, arc_dist);
        current_score *= 1-(bias_strength * abs(curvature-bias_center));

        if(current_score > best_score){
            best_curvature = curvature;
            best_score = current_score;
        }
    }
    return best_curvature;
}

}