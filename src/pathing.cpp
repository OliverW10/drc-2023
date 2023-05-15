#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#include "vision.hpp"

namespace pathing{

// get samples along the arc in map pixels
std::vector<cv::Point> getArcPixels(const Eigen::Vector3d& start, double curvature, double dist, int num_samples = 50){
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
    // TODO: this is kinda slow (may only be showing up in profiler because its the only hot part thats not library code)
    std::vector<cv::Point> points = getArcPixels(pos, curvature, dist);
    double total = 0;
    for(auto point : points){
        if(point.x >= 0 && point.x < track_map.cols && point.y >= 0 && point.y < track_map.rows){
            total += track_map.at<float>(point.y, point.x);
        }
    }
    return total / points.size() / dist;
}

// find arc that best follows the ridge
double getBestCurvature(const cv::Mat& track_map, const Eigen::Vector3d& start, double max_curvature, double arc_dist){
    int best_idx = 0;
    double best = 0;
    int curves_num = 20;
    // TODO: proritise low curvature
    for(int i = 0; i < curves_num; i++){
        double curvature = max_curvature * (2.0*(double)i/(curves_num-1.0) - 1.0);
        double cur = getArcFittness(track_map, start, curvature, arc_dist);
        if(cur > best){
            best_idx = i;
            best = cur;
        }
    }
    return best;
}

}