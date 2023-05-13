#include "vision.hpp"
#include "controller.hpp"
#include "streamer.hpp"
#include "util.hpp"
#include <Eigen/Dense>
#include <cmath>

#include "camera.cpp"
#include "pathing.cpp"

double pixels_per_meter = 100;
double map_width  = 4;
double map_height = 4;
int map_width_p  = (int)(map_width  * pixels_per_meter);
int map_height_p = (int)(map_height * pixels_per_meter);

// gets the matrix to pass to warpPerspective that corrects for the perspective of the
// ground and maps the image to the map
cv::Mat getPerspectiveTransform(const camera::Camera& cam){
    cv::Point2f source[4];
    source[0] = camera::projectPointCv(cam, Eigen::Vector4d(0,          -map_width/2, 0, 1));
    source[1] = camera::projectPointCv(cam, Eigen::Vector4d(0,          map_width/2 , 0, 1));
    source[2] = camera::projectPointCv(cam, Eigen::Vector4d(map_height, -map_width/2, 0, 1));
    source[3] = camera::projectPointCv(cam, Eigen::Vector4d(map_height, map_width/2 , 0, 1));
    cv::Point2f dest[4];
    dest[0] = cv::Point2f(0,           map_height_p);
    dest[1] = cv::Point2f(map_width_p, map_height_p);
    dest[2] = cv::Point2f(0,           0);
    dest[3] = cv::Point2f(map_width_p, 0);

    cv::Mat ret = cv::getPerspectiveTransform(source, dest);
    return ret;
}

// convert a position relative to car in meters to pixel position in track_map
cv::Point posToMap(const Eigen::Vector3d& position){
    // car is at bottom middle of map
    return cv::Point(map_width_p/2 - (position(1)*pixels_per_meter), map_height_p - (position(0)*pixels_per_meter));
}

/*
TODO: this is the slowest part of process atm
should try using contours with offset in direction of normal
would also allow better filtering of erroneous blobs
*/
cv::Mat getPotentialTrack(cv::Mat tape_mask, double track_mid_dist = 1, double track_width = 1.5){
    /*
    track_mid_dist: typical distance from tape to track center line
    track_width: width of area to mark as track
    */
    cv::erode(tape_mask, tape_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    // distance transform finds distnace to zero's, invert so it finds distance to 1's
    tape_mask = 255 - tape_mask;
    cv::Mat dists;
    cv::distanceTransform(tape_mask, dists, cv::DIST_L2, 3);
    dists = dists / pixels_per_meter;
    // how close to center of track 0 to 1
    cv::Mat track = 1 - cv::abs(track_mid_dist - dists)/(track_width/2);
    cv::threshold(track, track, 0, 0, cv::THRESH_TOZERO);
    return track;
}


cv::Mat getMovementAffineTransform(CarState state, double dt){
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

TIME_INIT(process)
TIME_INIT(perspective)
TIME_INIT(threshold)
TIME_INIT(track)
TIME_INIT(plan)

TIME_INIT(outside)

CarState Vision::process(const cv::Mat& image, const SensorValues& sensor_input){
    TIME_STOP(outside)
    TIME_START(process)

    /* Correct for perspective of ground */
    TIME_START(perspective)
    cv::Mat image_corrected;
    cv::warpPerspective(image, image_corrected, m_perspective_transform, cv::Size(map_width_p, map_height_p));
    TIME_STOP(perspective)

    /* Get masks for various colours */
    TIME_START(threshold)
    // convert to hsv
    cv::Mat hsv_ground;
    cv::cvtColor(image_corrected, hsv_ground, cv::COLOR_BGR2HSV);

    cv::Mat mask_yellow;
    cv::inRange(hsv_ground, cv::Scalar(25, 40, 30), cv::Scalar(40, 255, 255), mask_yellow);
    cv::Mat mask_blue;
    cv::inRange(hsv_ground, cv::Scalar(100, 40, 50), cv::Scalar(140, 255, 255), mask_blue);
    cv::Mat mask_black;
    cv::inRange(hsv_ground, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 5), mask_black);
    cv::Mat mask_purple;
    cv::inRange(hsv_ground, cv::Scalar(130, 40, 50), cv::Scalar(150, 255, 255), mask_purple);
    cv::Mat mask_red;
    cv::inRange(hsv_ground, cv::Scalar(10, 40, 50), cv::Scalar(170, 255, 255), mask_red);
    mask_red = 255-mask_red;
    // TODO: cut top off contours in obsticles to make them have a maximum depth and subtract from track
    // TODO: do obsticle and arrow stuff in seperate threads
    TIME_STOP(threshold)

    cv::imshow("input", image);
    streamer::imshow("abc", image_corrected);
    cv::imshow("perspective", image_corrected);
    cv::imshow("blue_mask", mask_blue);

    /* Estimate driveable area from masks and accumulate over time */
    TIME_START(track)
    cv::Mat track_right = getPotentialTrack(mask_yellow);
    cv::Mat track_left = getPotentialTrack(mask_blue);
    cv::Mat track_combined = track_left + track_right;

    // move map backwards by current car state
    int dt_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - m_last_time).count();
    m_last_time = std::chrono::high_resolution_clock::now();
    double dt = ((double)dt_us) / 1000 / 1000;
    cv::Mat movement_transform = getMovementAffineTransform(sensor_input.state, dt);
    cv::warpAffine(m_track_map, m_track_map, movement_transform, cv::Size(map_width_p, map_height_p));

    // decay previous map
    double decay_time = 5; // time to decay from fully saturated to 0
    m_track_map -= dt / decay_time;
    // add new one on top
    double accumulate_time = 0.2; // time for map to full saturation for pixels of 100% confidnece
    m_track_map += track_combined * (0.5 * dt / accumulate_time);
    // clamp from 0-1
    cv::threshold(m_track_map, m_track_map, 1, 1, cv::THRESH_TRUNC);
    cv::threshold(m_track_map, m_track_map, 0, 0, cv::THRESH_TOZERO);

    TIME_STOP(track)

    cv::imshow("cur_track", track_combined);

    /* Plan path forwards */
    TIME_START(plan)
    double lookahead = 2;
    double chosen_curvature = pathing::getBestCurvature(m_track_map, Eigen::Vector3d(0, 0, 0), M_PI_2, lookahead);
    // draw planned path on map
    cv::Mat track_annotated;
    m_track_map.convertTo(track_annotated, CV_32F);
    cv::cvtColor(track_annotated, track_annotated, cv::COLOR_GRAY2BGR);
    std::vector<cv::Point> path_points = pathing::getArcPixels(Eigen::Vector3d(0, 0, 0), chosen_curvature, lookahead, 10);
    cv::polylines(track_annotated, path_points, false, cv::Scalar(255, 0, 0));

    path_points = pathing::getArcPixels(Eigen::Vector3d(0, 0, 0), 0.1, lookahead, 10);
    cv::polylines(track_annotated, path_points, false, cv::Scalar(0, 255, 0));

    cv::imshow("track_map", track_annotated);

    TIME_STOP(plan)
    TIME_STOP(process)

    if(m_frame_counter % 30 == 0){
        printf("\n\n");
        TIME_PRINT(outside)
        TIME_PRINT(process)
        TIME_PRINT(perspective)
        TIME_PRINT(threshold)
        TIME_PRINT(track)
        TIME_PRINT(plan)
    }
    m_frame_counter ++;
    TIME_START(outside)
    return CarState{1, 0};
}


Vision::Vision(int img_width, int img_height){
    camera::Camera cam{camera::getIntrinsics(), camera::carToCameraTransform(), img_width, img_height};
    m_perspective_transform = getPerspectiveTransform(cam);
    m_track_map = cv::Mat::zeros(map_height_p, map_width_p, CV_64F);
    streamer::initStreaming();
    // cv::Mat image_right = cv::imread(argv[2]);
    // cv::cvtColor(image_right, image_right, cv::COLOR_BGR2GRAY);

    // cv::Mat image_left = cv::imread(argv[1]);
    // cv::cvtColor(image_left, image_left, cv::COLOR_BGR2GRAY);
}
