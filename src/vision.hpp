#ifndef VISION_H
#define VISION_H

#include <opencv2/core/mat.hpp>
#include <Eigen/Core>
#include "controller.hpp"
#include <thread>

cv::Point posToMap(const Eigen::Vector3d& position);

// takes input from sensors and produces a map of the enviroment
class Vision{
public:
    Vision(int img_width, int img_height);
    // process image and return desired car state
    CarState process(const cv::Mat& image, const SensorValues& sensor_input, bool print_timings);
    // clear finished state and start moving
    void forceStart();
    void detachThreads();
    void printTimings();
private:
    int m_frame_counter = 1;
    cv::Mat m_track_map;
    cv::Mat m_perspective_transform;
    // as members to reuse buffer between frames
    cv::Mat m_image_corrected,
        m_hsv_ground,
        m_mask_yellow,
        m_mask_blue,
        m_track_combined,
        m_track_yellow,
        m_track_blue,
        m_annotated_image,
        m_red_mask,
        m_purple_mask,
        m_red_obstacles,
        m_purple_obstacles,
        m_obstacle_map;
    std::thread m_arrow_thread, m_stream_thread, m_annotate_thread, m_map_mover_thread;
    std::chrono::high_resolution_clock::time_point m_last_time;
};

const double pixels_per_meter = 100;
const double map_width  = 3;
const double map_height = 3;
const int map_width_p  = (int)(map_width  * pixels_per_meter);
const int map_height_p = (int)(map_height * pixels_per_meter);

#endif