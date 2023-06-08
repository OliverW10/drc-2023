#include "vision.hpp"
#include "controller.hpp"
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

// gets the matrix to pass to warpPerspective that corrects for the perspective of the
// ground and maps the image to the map
cv::Mat getPerspectiveTransform(const camera::Camera& cam){
    cv::Point2f source[4];
    source[0] = camera::projectPointCv(cam, Eigen::Vector4d(0,          -map_width/2, 0, 1));
    source[1] = camera::projectPointCv(cam, Eigen::Vector4d(0,          map_width/2 , 0, 1));
    source[2] = camera::projectPointCv(cam, Eigen::Vector4d(map_height, -map_width/2, 0, 1));
    source[3] = camera::projectPointCv(cam, Eigen::Vector4d(map_height, map_width/2 , 0, 1));
    cv::Point2f dest[4];
    dest[0] = cv::Point2f(map_width_p, map_height_p);
    dest[1] = cv::Point2f(0,           map_height_p);
    dest[2] = cv::Point2f(map_width_p, 0);
    dest[3] = cv::Point2f(0,           0);

    return cv::getPerspectiveTransform(source, dest);
}

// convert a position relative to car in meters to pixel position in track_map
cv::Point posToMap(const Eigen::Vector3d& position){
    // car is at bottom middle of map
    return cv::Point(map_width_p/2 - (position(1)*pixels_per_meter), map_height_p - (position(0)*pixels_per_meter));
}

void getPotentialTrackFromMask(const cv::Mat& tape_mask, bool allowed_x_sign, cv::Mat& track_out){
    /*
    gets a track center line by offsetting the outside of each contour by half the track width
    then draws a thick line along the center line to mark it as driveable
    */
    double track_mid_dist = 0.4;
    double track_width = 0.5;
    int track_inner_stroke = track_width * pixels_per_meter;

    track_out.setTo(cv::Scalar(0));
    std::vector<std::vector<cv::Point>> all_contours;
    std::vector<cv::Point> center_line;
    cv::findContours(tape_mask, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const int undersample = 4;
    for(auto contour : all_contours){
        const int sample_dist = 3;
        const int sample_num = 3;
        const int total_averaging_length = 2 * sample_num * sample_dist;
        if(contour.size() < 3 * total_averaging_length){
            continue;
        }
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
            if( (normal(0) > 0) == allowed_x_sign ){
                if(center_line.size() > 3){
                    cv::polylines(track_out, center_line, false, cv::Scalar(1), track_inner_stroke, cv::LINE_4);
                }
                center_line.clear();
                continue;
            }
            normal.normalize();
            normal *= track_mid_dist * pixels_per_meter;
            center_line.push_back(cv::Point(center.x + normal(0), center.y + normal(1)));
        }
        cv::polylines(track_out, center_line, false, cv::Scalar(1), track_inner_stroke, cv::LINE_4);
    }
}


cv::Mat getMovementTransform(CarState state, double dt){
    Eigen::Vector3d delta = state.getTimeForwards(dt);

    cv::Point2f src[3];
    src[0] = posToMap(Eigen::Vector3d(0, 0, 0));
    src[1] = posToMap(Eigen::Vector3d(1, 0, 0));
    src[2] = posToMap(Eigen::Vector3d(0, 1, 0));
    cv::Point2f dst[3];
    dst[0] = posToMap(-delta);
    dst[1] = posToMap(-delta + Eigen::Vector3d(std::cos(delta(2)), std::sin(delta(2)), 0));
    dst[2] = posToMap(-delta + Eigen::Vector3d(std::sin(delta(2)), std::cos(delta(2)), 0));
    cv::Mat ret = cv::getAffineTransform(src, dst);
    return ret;
}

cv::Scalar getConfigHsvScalarLow(std::string name){
    return cv::Scalar(
        getConfigDouble(name+"_h_low"), 
        getConfigDouble(name+"_s_low"), 
        getConfigDouble(name+"_v_low")
    );
}

cv::Scalar getConfigHsvScalarHigh(std::string name){
    return cv::Scalar(
        getConfigDouble(name+"_h_high"), 
        getConfigDouble(name+"_s_high"), 
        getConfigDouble(name+"_v_high")
    );
}

void getPotentialTrackFromHsv(
    const cv::Mat& hsv_ground,
    bool allowed_sign,
    cv::Scalar mask_low,
    cv::Scalar mask_high,
    cv::Mat& mask,
    cv::Mat& track
){
    cv::inRange(hsv_ground, mask_low, mask_high, mask);
    getPotentialTrackFromMask(mask, allowed_sign, track);
}

void moveMap(CarState state, double dt, cv::Mat& map){
    // move map backwards by current car state
    cv::Mat movement_transform = getMovementTransform(state, dt);
    cv::warpAffine(map, map, movement_transform, cv::Size(map_width_p, map_height_p), cv::INTER_NEAREST);
}

void annotateMap(const cv::Mat& track_map, double chosen_curvature, double lookahead, cv::Mat& track_annotated){
    cv::cvtColor(track_map, track_annotated, cv::COLOR_GRAY2BGR);
    std::vector<cv::Point> path_points = pathing::getArcPixels(Eigen::Vector3d(0, 0, 0), chosen_curvature, lookahead, 10);
    cv::polylines(track_annotated, path_points, false, cv::Scalar(1), 1, cv::LINE_4);
    streamer::imshow("map", track_annotated);
}


TIME_INIT(process)
TIME_INIT(start_threads)
TIME_INIT(perspective)
TIME_INIT(track)
TIME_INIT(map)
TIME_INIT(arrow_wait)
TIME_INIT(plan)
TIME_INIT(stream)

#define time(x) TIME_START(stream) x TIME_STOP(stream)

TIME_INIT(waiting)

CarState Vision::process(const cv::Mat& image, const SensorValues& sensor_input){
    TIME_STOP(waiting)
    TIME_START(process)

    time(streamer::imshow("input", image);)

    int dt_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - m_last_time).count();
    m_last_time = std::chrono::high_resolution_clock::now();
    double dt = 1/30; //((double)dt_us) / 1000 / 1000;

    /* Correct for perspective of ground */
    TIME_START(perspective)
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

    /* Start threads to detect arrow and move map*/
    TIME_START(start_threads)
    m_annotate_thread.join();
    m_map_mover_thread = std::thread(moveMap, sensor_input.state, dt, std::ref(m_track_map));
    double arrow_confidence = 0;
    m_arrow_thread = std::thread(find_arrow, std::cref(m_hsv_ground), std::ref(arrow_confidence));
    TIME_STOP(start_threads)

    /* Find track areas from hsv ground*/
    TIME_START(track)
    // convert to hsv

    std::thread yellow_thread = std::thread(
        getPotentialTrackFromHsv,
        std::cref(m_hsv_ground),
        false,
        getConfigHsvScalarLow("yellow"),
        getConfigHsvScalarHigh("yellow"),
        std::ref(m_mask_yellow),
        std::ref(m_track_yellow)
    );
    std::thread blue_thread = std::thread(
        getPotentialTrackFromHsv,
        std::cref(m_hsv_ground),
        true,
        getConfigHsvScalarLow("blue"),
        getConfigHsvScalarHigh("blue"),
        std::ref(m_mask_blue),
        std::ref(m_track_blue)
    );
    
    yellow_thread.join();
    blue_thread.join();
    time(streamer::imshow("blue", m_mask_blue);)
    time(streamer::imshow("yellow", m_mask_yellow);)
    m_track_combined = m_track_yellow + m_track_blue;
    TIME_STOP(track)

    TIME_START(map)
    m_map_mover_thread.join();

    // accumulate exponentially
    double accumulate_alpha = 0.01;
    double decay_alpha = 0.995;
    m_track_map = decay_alpha * m_track_map + accumulate_alpha * m_track_combined;

    // clamp from 0-1
    cv::threshold(m_track_map, m_track_map, 1, 1, cv::THRESH_TRUNC);
    cv::threshold(m_track_map, m_track_map, 0, 0, cv::THRESH_TOZERO);
    // time(streamer::imshow("cur", m_track_combined);)
    TIME_STOP(map)

    TIME_START(arrow_wait)
    m_arrow_thread.join();
    TIME_STOP(arrow_wait)

    /* Plan path forwards */
    TIME_START(plan)
    double lookahead = 2.0;
    double chosen_curvature = pathing::getBestCurvature(m_track_map, Eigen::Vector3d(0, 0, 0), M_PI_2, lookahead, 0, 0);
    m_annotate_thread = std::thread(annotateMap, std::cref(m_track_map), chosen_curvature, lookahead, std::ref(m_annotated_image));
    TIME_STOP(plan)

    TIME_STOP(process)

    if(m_frame_counter % 30 == 0){
        printf("\n\n");
        TIME_PRINT(waiting)
        TIME_PRINT(process)
        printf("\n");
        TIME_PRINT(perspective)
        TIME_PRINT(start_threads)
        TIME_PRINT(track)
        TIME_PRINT(map)
        TIME_PRINT(arrow_wait)
        TIME_PRINT(plan)
        // std::cout << "avg per call: ";
        // TIME_PRINT(stream)
    }
    m_frame_counter ++;
    TIME_START(waiting)
    return CarState{1, 0};
}


Vision::Vision(int img_width, int img_height){
    camera::Camera cam{camera::getIntrinsics(img_width, img_height), camera::carToCameraTransform(15), img_width, img_height};
    m_perspective_transform = getPerspectiveTransform(cam);
    m_track_map = cv::Mat::zeros(map_height_p, map_width_p, CV_32F);
    streamer::initStreaming();

    m_image_corrected = cv::Mat::zeros(map_height_p, map_width_p, CV_8UC3);
    m_hsv_ground = cv::Mat::zeros(map_height_p, map_width_p, CV_8UC3);
    m_mask_blue = cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_mask_yellow = cv::Mat::zeros(map_height_p, map_width_p, CV_8UC1);
    m_track_combined = cv::Mat::zeros(map_height_p, map_width_p, CV_32F);
    m_track_yellow = cv::Mat::zeros(map_height_p, map_width_p, CV_32F);
    m_track_blue = cv::Mat::zeros(map_height_p, map_width_p, CV_32F);

    m_annotate_thread = std::thread([](){});
}

void Vision::detachThreads(){
    m_annotate_thread.join();
    streamer::closeThread();
}
