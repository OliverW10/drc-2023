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
            total += track_map.at<double>(point.y, point.x);
        }
    }
    return total;
}

// find arc that best follows the ridge
double getBestCurvature(
    const cv::Mat& track_map,
    const Eigen::Vector3d& start,
    double max_curvature,
    double arc_dist,
    double bias_center,
    double bias_strength
){
    double best_curvature = 0;
    double best_score = 0;
    int curves_num = 21;
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
    std::cout << "best score: " << best_score << "\n";
    return best_curvature;
}

}