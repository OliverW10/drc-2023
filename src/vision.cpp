#include "vision.hpp"
#include "car_state.hpp"
#include "streamer.hpp"
#include "util.hpp"
#include "config.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>
#include "pathing.hpp"
#include "camera.hpp"
#include "arrow.hpp"
#include "obstacle.hpp"
#include "finish_line.hpp"

// convert a position relative to car in meters to pixel position in track_map
cv::Point posToMap(const Eigen::Vector3d& position){
    // car is at bottom middle of map
    return cv::Point(map_width_p/2 - (position(1)*pixels_per_meter), map_height_p - (position(0)*pixels_per_meter));
}


void getPotentialTrackFromMask(const cv::Mat& tape_mask, bool allowed_x_sign, cv::Mat& track_out, cv::Mat& map){
    /*
    gets a track center line by offsetting the outside of each contour by half the track width
    then draws a thick line along the center line to mark it as driveable
    */
    double track_mid_dist = getConfigDouble("track_mid_dist");
    double track_width = getConfigDouble("track_width");
    int track_inner_stroke = track_width * pixels_per_meter;

    int map_accumulate = (int)getConfigDouble("map_accumulate");

    track_out.setTo(cv::Scalar(0));
    std::vector<std::vector<cv::Point>> all_contours;
    std::vector<cv::Point> center_line;
    std::vector<int> good_contours_idxs;
    cv::findContours(tape_mask, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const int undersample = 4;
    int i = -1;
    for(auto contour : all_contours){
        i++;
        const int sample_dist = 3;
        const int sample_num = 3;
        const int total_averaging_length = 2 * sample_num * sample_dist;
        if(contour.size() < 5 * total_averaging_length){
            continue;
        }
        good_contours_idxs.push_back(i);
        for(size_t i = 0; i < contour.size(); i += undersample){
            cv::Point center = contour[i];
            Eigen::Vector2d total = Eigen::Vector2d::Zero();
            for(int s = 1; s <= sample_num; s++){
                cv::Point a = contour[(i + s*sample_dist) % contour.size()];
                cv::Point b = contour[(i - s*sample_dist) % contour.size()];
                cv::Point delta = a-b;
                total += Eigen::Vector2d(delta.x, delta.y).normalized();
            }
            Eigen::Vector2d normal(-total(1), total(0));
            normal.normalize();
            normal *= track_mid_dist * pixels_per_meter;
            center_line.push_back(cv::Point(center.x + normal(0), center.y + normal(1)));
        }
        cv::polylines(track_out, center_line, false, cv::Scalar(map_accumulate), track_inner_stroke, cv::LINE_4);
    }
    // std::cout << "good num: " << good_contours_idxs.size() << "\n";
    for(int idx : good_contours_idxs){
        cv::drawContours(map, all_contours, idx, cv::Scalar(0, 0, 0), 7);
    }
}

void getPotentialTrackFromHsv(
    const cv::Mat& hsv_ground,
    bool allowed_sign,
    cv::Scalar mask_low,
    cv::Scalar mask_high,
    cv::Mat& mask,
    cv::Mat& color_track,
    cv::Mat& real_map
){
    cv::inRange(hsv_ground, mask_low, mask_high, mask);
    getPotentialTrackFromMask(mask, allowed_sign, color_track, real_map);
}

cv::Mat getMovementTransform(CarState state, double dt){
    double speed_scaler = getConfigDouble("map_move_drive_scaler");
    double turn_scaler = getConfigDouble("map_move_turn_scaler");
    CarState scaled_state {state.speed * speed_scaler, state.curvature * turn_scaler};
    Eigen::Vector3d delta = scaled_state.getTimeForwards(dt);

    cv::Point2f src[3];
    src[0] = posToMap(Eigen::Vector3d(0, 0, 0));
    src[1] = posToMap(Eigen::Vector3d(1, 0, 0));
    src[2] = posToMap(Eigen::Vector3d(0, 1, 0));
    cv::Point2f dst[3];
    dst[0] = posToMap(-delta);
    dst[1] = posToMap(-delta + Eigen::Vector3d(std::cos(-delta(2)), std::sin(-delta(2)), 0));
    dst[2] = posToMap(-delta + Eigen::Vector3d(std::sin(-delta(2)), std::cos(-delta(2)), 0));
    cv::Mat ret = cv::getAffineTransform(src, dst);
    return ret;
}

// move map backwards and rotate to account for car moving forwards and turning
void moveMap(CarState state, double dt, cv::Mat& map){
    cv::Mat movement_transform = getMovementTransform(state, dt);
    // TODO: look at effect linear has on quality and/or performance
    cv::warpAffine(map, map, movement_transform, cv::Size(map_width_p, map_height_p), cv::INTER_NEAREST);
}

void annotateMap(const cv::Mat& track_map, double chosen_curvature, double lookahead, double bias_curvature, double finish_conf, cv::Mat& track_annotated){
    cv::cvtColor(track_map, track_annotated, cv::COLOR_GRAY2BGR);
    // bias line
    std::vector<cv::Point> bias_points = pathing::getArcPixels(Eigen::Vector3d(0, 0, 0), bias_curvature, lookahead, 10);
    cv::polylines(track_annotated, bias_points, false, cv::Scalar(200, 0, 0), 1, cv::LINE_4);
    // chosen curvature line
    std::vector<cv::Point> path_points = pathing::getArcPixels(Eigen::Vector3d(0, 0, 0), chosen_curvature, lookahead, 10);
    cv::polylines(track_annotated, path_points, false, cv::Scalar(0, 255, 0), 1, cv::LINE_4);

    // cv::Scalar col;
    // if(arrow_conf > 0){
    //     col = cv::Scalar(0, 255, 0);
    //     cv::rectangle(track_annotated, cv::Rect(map_width_p/2, 0, arrow_conf*map_width_p/2, map_height_p*0.1), col, -1);
    // }else{
    //     col = cv::Scalar(255, 0, 0);
    //     cv::rectangle(track_annotated, cv::Rect((arrow_conf/2+0.5)*map_width_p, 0, arrow_conf*map_width_p/2, map_height_p*0.1), col, -1);
    // }
    // cv::putText(track_annotated, std::to_string(arrow_conf), cv::Point(map_width_p*0.4, 20), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
    if(finish_conf > 0){
        cv::putText(track_annotated, "done", cv::Point(map_width_p*0.4, map_width_p*0.3), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
    }
    streamer::imshow("map", track_annotated);
}

double chosen_curvature = 0;

bool has_finished = false;
std::chrono::time_point<std::chrono::high_resolution_clock> started_time = std::chrono::high_resolution_clock::now();

TIME_INIT(process)
TIME_INIT(perspective)
TIME_INIT(finish_line)
TIME_INIT(track)
TIME_INIT(obstacle_wait)
TIME_INIT(map_combine)
TIME_INIT(arrow_wait)
TIME_INIT(plan)
TIME_INIT(stream)

#define time(x) TIME_START(stream) x TIME_STOP(stream)

TIME_INIT(waiting)

CarState Vision::process(const cv::Mat& image, const SensorValues& sensor_input, bool print_timings = true){
    TIME_STOP(waiting)
    TIME_START(process)

    double dt = 1.0/30.0;

    m_annotate_thread.join();
    m_map_mover_thread = std::thread(moveMap, sensor_input.state, dt, std::ref(m_track_map));

    double finish_line_confidence;
    std::thread finish_line_thread(find_finish_line, std::cref(image), std::ref(finish_line_confidence));

    /* Correct for perspective of ground */
    TIME_START(perspective)
    if(config_may_have_changed){
        double camera_angle = getConfigDouble("camera_angle");
        double camera_height = getConfigDouble("camera_height");
        camera::Camera cam{
            camera::getIntrinsics(image.cols, image.rows),
            camera::carToCameraTransform(camera_angle, camera_height),
            image.cols, image.rows
        };
        m_perspective_transform = camera::getPerspectiveTransform(cam);
    }
    cv::warpPerspective(
        image,
        m_image_corrected,
        m_perspective_transform,
        cv::Size(map_width_p, map_height_p),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(127, 127, 127)
    );
    cv::cvtColor(m_image_corrected, m_hsv_ground, cv::COLOR_BGR2HSV);
    time(streamer::imshow("ground", m_image_corrected);)
    TIME_STOP(perspective)

    TIME_START(finish_line)
    finish_line_thread.join();
    bool has_finish_line = finish_line_confidence > 0.9;
    auto started_ago = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - m_last_time).count();
    bool just_started = started_ago < 5;
    if(has_finish_line && !just_started){
        has_finished = true;
    }
    cv::Scalar rect_col = finish_line_confidence > 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 150, 0);
    cv::rectangle(image, get_finish_line_roi(image.size()), rect_col, 3);
    time(streamer::imshow("input", image);)
    TIME_STOP(finish_line)

    /* Find arrow */
    double arrow_confidence = 0;
    m_arrow_thread = std::thread(find_arrow, std::cref(m_hsv_ground), std::ref(arrow_confidence));

    /* Find obstacles */
    std::thread obstacle_thread(find_obsticles, 
        std::cref(m_hsv_ground), 
        std::ref(m_purple_mask), std::ref(m_red_mask), std::ref(m_purple_obstacles), std::ref(m_red_obstacles),
        std::ref(m_obstacle_map)
    );

    /* Find track areas*/
    TIME_START(track)

    getPotentialTrackFromHsv(
        m_hsv_ground,
        false, 
        getConfigHsvScalarLow("yellow"),
        getConfigHsvScalarHigh("yellow"),
        m_mask_yellow,
        m_track_yellow,
        m_track_map
    );
    getPotentialTrackFromHsv(
        m_hsv_ground,
        true, 
        getConfigHsvScalarLow("blue"),
        getConfigHsvScalarHigh("blue"),
        m_mask_blue,
        m_track_blue,
        m_track_map
    );

    time(streamer::imshow("blue", m_mask_blue);)
    time(streamer::imshow("yellow", m_mask_yellow);)
    TIME_STOP(track)

    TIME_START(obstacle_wait)
    obstacle_thread.join();
    TIME_STOP(obstacle_wait)

    /* Combine left and right tracks and obstacle map and average over time*/
    TIME_START(map_combine)

    m_map_mover_thread.join();
    m_track_map += m_track_yellow + m_track_blue;
    streamer::imshow("cur", m_track_combined);
    m_track_map &= ~m_purple_obstacles;
    m_track_map -= (int)getConfigDouble("map_decay");

    // clamp from 0-1
    cv::threshold(m_track_map, m_track_map, 255, 255, cv::THRESH_TRUNC);
    cv::threshold(m_track_map, m_track_map, 0, 0, cv::THRESH_TOZERO);

    TIME_STOP(map_combine)

    TIME_START(arrow_wait)
    m_arrow_thread.join();
    TIME_STOP(arrow_wait)

    /* Plan path */
    TIME_START(plan)

    double arrow_bias_amount = getConfigDouble("arrow_bias_amount");
    double bias_center = arrow_confidence * arrow_bias_amount;

    // Get bias strength
    double base_bias_strength = getConfigDouble("base_bias_strength");
    double arrow_bias_strength = getConfigDouble("arrow_bias_strength");
    double bias_strength = rescale(std::abs(arrow_confidence), 0, 1, base_bias_strength, arrow_bias_strength);

    double lookahead = 2.0;
    double _chosen_curvature = pathing::getBestCurvature(m_track_map, Eigen::Vector3d(0, 0, 0), lookahead, bias_center, bias_strength);
    const double turn_alpha = 0.1;
    chosen_curvature = chosen_curvature * (1-turn_alpha) + _chosen_curvature * turn_alpha;

    const double max_speed = 3;
    const double min_speed = 1;
    double corner_speed = rescale(std::abs(chosen_curvature), 0.0, 1.5, max_speed, min_speed);
    const double max_accel = 1; // per second
    double chosen_speed = std::min(corner_speed, sensor_input.state.speed + max_accel*dt);
    // std::cout << chosen_speed << ", " << chosen_curvature << "\n";

    TIME_STOP(plan)
    m_annotate_thread = std::thread(annotateMap, std::cref(m_track_map), chosen_curvature, lookahead, bias_center, finish_line_confidence, std::ref(m_annotated_image));

    TIME_STOP(process)

    if(m_frame_counter % 30 == 0 && print_timings){
        printTimings();
    }
    m_frame_counter ++;
    TIME_START(waiting)
    if(!has_finished){
        return CarState{chosen_speed, chosen_curvature};
    }else{
        return CarState{0, 0};
    }
}

void Vision::printTimings(){
    printf("\n\n");
    TIME_PRINT(waiting)
    TIME_PRINT(process)
    printf("\n");
    TIME_PRINT(perspective)
    TIME_PRINT(finish_line)
    TIME_PRINT(track)
    TIME_PRINT(obstacle_wait)
    TIME_PRINT(map_combine)
    TIME_PRINT(arrow_wait)
    TIME_PRINT(plan)
    // std::cout << "avg per call: ";
    // TIME_PRINT(stream)
}

void Vision::forceStart(){
    has_finished = false;
    started_time = std::chrono::high_resolution_clock::now();
}

Vision::Vision(int img_width, int img_height){
    double camera_angle = getConfigDouble("camera_angle");
    double camera_height = getConfigDouble("camera_height");
    camera::Camera cam{
        camera::getIntrinsics(img_width, img_height),
        camera::carToCameraTransform(camera_angle, camera_height),
        img_width, img_height
    };
    m_perspective_transform = camera::getPerspectiveTransform(cam);

    m_track_map = cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    streamer::initStreaming();

    m_mask_blue =        cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_mask_yellow =      cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_purple_mask =      cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_purple_obstacles = cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_red_obstacles =    cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);

    m_image_corrected =  cv::Mat::zeros(map_height_p, map_width_p, CV_8UC3);
    m_hsv_ground =       cv::Mat::zeros(map_height_p, map_width_p, CV_8UC3);

    m_track_combined =   cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_track_yellow =     cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_track_blue =       cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_obstacle_map =     cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);

    m_annotate_thread = std::thread([](){});
}

void Vision::detachThreads(){
    m_annotate_thread.join();
    streamer::closeThread();
}
