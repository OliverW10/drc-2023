#include "vision.hpp"
#include "controller.hpp"
#include "streamer.hpp"
#include "util.hpp"
#include "config.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "camera.cpp"
#include "pathing.cpp"
#include "arrow.cpp"

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

cv::Mat getPotentialTrackNormalOffset(const cv::Mat& tape_mask, bool allowed_x_sign){
    /*
    gets a track center line by offsetting the outside of each contour by half the track width
    then draws a thick line along the center line to mark it as driveable
    */
    double track_mid_dist = 0.4;
    double track_width = 0.5;
    int track_inner_stroke = track_width * pixels_per_meter;

    cv::Mat track(tape_mask.size(), CV_32F, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(tape_mask, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const int undersample = 4;
    int num_good = 0;
    for(auto contour : all_contours){
        const int sample_dist = 3;
        const int sample_num = 3;
        const int total_averaging_length = 2 * sample_num * sample_dist;
        if(contour.size() < 3 * total_averaging_length){
            continue;
        }
        num_good ++;
        std::vector<cv::Point> center_line;
        for(size_t i = 0; i < contour.size(); i += undersample){
            cv::Point center = contour[i];
            Eigen::Vector2d total = Eigen::Vector2d::Zero();
            for(int s = 1; s < sample_num; s++){
                cv::Point a = contour[(i + s*sample_dist) % contour.size()];
                cv::Point b = contour[(i - s*sample_dist) % contour.size()];
                cv::Point delta = a-b;
                total += Eigen::Vector2d(delta.x, delta.y).normalized();
            }
            Eigen::Vector2d normal(-total(1), total(0));
            if( (normal(0) > 0) == allowed_x_sign ){
                if(center_line.size() > 3){
                    cv::polylines(track, center_line, false, cv::Scalar(1), track_inner_stroke, cv::LINE_4);
                }
                center_line.clear();
                continue;
            }
            normal.normalize();
            normal *= track_mid_dist * pixels_per_meter;
            center_line.push_back(cv::Point(center.x + normal(0), center.y + normal(1)));
        }
        cv::polylines(track, center_line, false, cv::Scalar(1), track_inner_stroke, cv::LINE_4);
    }
    return track;
}

cv::Mat getPotentialTrackDistField(const cv::Mat& tape_mask, bool _){
    /*
    gets a distance field where each pixel is the distance to closest positive bit of the mask
    then change that to be the distance from track_mid_dist dist away (i.e. the center of the track)
    */
    // typical distance from tape to track center line
    double track_mid_dist = 0.5;
    // width of area to mark as track
    double track_width = 0.75;
    cv::erode(tape_mask, tape_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    // distance transform finds distnace to zero's, invert so it finds distance to 1's
    cv::Mat dists;
    cv::distanceTransform(255 - tape_mask, dists, cv::DIST_L2, 3);
    dists = dists / pixels_per_meter;
    // how close to center of track 0 to 1
    cv::Mat track = 1 - cv::abs(track_mid_dist - dists)/(track_width/2);
    cv::threshold(track, track, 0, 0, cv::THRESH_TOZERO);
    return track;
}

constexpr cv::Mat (*getPotentialTrack)(const cv::Mat&, bool) = &getPotentialTrackNormalOffset;


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
        getConfig(name+"_h_low"), 
        getConfig(name+"_s_low"), 
        getConfig(name+"_v_low")
    );
}

cv::Scalar getConfigHsvScalarHigh(std::string name){
    return cv::Scalar(
        getConfig(name+"_h_high"), 
        getConfig(name+"_s_high"), 
        getConfig(name+"_v_high")
    );
}

TIME_INIT(process)
TIME_INIT(perspective)
TIME_INIT(threshold)
TIME_INIT(arrow)
TIME_INIT(track)
TIME_INIT(plan)
TIME_INIT(stream)

TIME_INIT(waiting)

CarState Vision::process(const cv::Mat& image, const SensorValues& sensor_input){
    TIME_STOP(waiting)
    TIME_START(process)

    /* Correct for perspective of ground */
    TIME_START(perspective)
    cv::Mat image_corrected;
    cv::warpPerspective(
        image,
        image_corrected,
        m_perspective_transform,
        cv::Size(map_width_p, map_height_p),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(127, 127, 127)
    );
    TIME_STOP(perspective)

    /* Get masks for various colours */
    TIME_START(threshold)
    // convert to hsv
    cv::Mat hsv_ground;
    cv::cvtColor(image_corrected, hsv_ground, cv::COLOR_BGR2HSV);

    cv::Mat mask_yellow;
    cv::inRange(hsv_ground, getConfigHsvScalarLow("yellow"), getConfigHsvScalarHigh("yellow"), mask_yellow);
    cv::Mat mask_blue;
    cv::inRange(hsv_ground, getConfigHsvScalarLow("blue"), getConfigHsvScalarHigh("blue"), mask_blue);
    // cv::Mat mask_purple;
    // cv::inRange(hsv_ground, getConfigHsvScalarLow("purple"), getConfigHsvScalarHigh("purple"), mask_purple);
    // cv::Mat mask_red;
    // cv::inRange(hsv_ground, getConfigHsvScalarLow("red"), getConfigHsvScalarHigh("red"), mask_red);
    // mask_red = 255-mask_red;
    // TODO: cut top off contours in obsticles to make them have a maximum depth and subtract from track
    // TODO: do obsticle and arrow stuff in seperate threads
    TIME_STOP(threshold)

    TIME_START(arrow)
    // find_arrow(hsv_ground);
    TIME_STOP(arrow)

    /* Estimate driveable area from masks and accumulate over time */
    TIME_START(track)
    cv::Mat track_combined = getPotentialTrack(mask_yellow, false) + getPotentialTrack(mask_blue, true);

    // move map backwards by current car state
    int dt_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - m_last_time).count();
    m_last_time = std::chrono::high_resolution_clock::now();
    double dt = ((double)dt_us) / 1000 / 1000;
    cv::Mat movement_transform = getMovementTransform(sensor_input.state, dt);
    cv::warpAffine(m_track_map, m_track_map, movement_transform, cv::Size(map_width_p, map_height_p));

    // accumulate addidavely
    /*
    // decay previous map
    double decay_time = 5; // time to decay from fully saturated to 0
    m_track_map -= dt / decay_time;
    // add new one on top
    double accumulate_time = 0.2; // time for map to full saturation for pixels of 100% confidnece
    m_track_map += track_combined * (0.5 * dt / accumulate_time);
    */

    // accumulate exponentially
    double x = 0.6;
    double alpha = pow(x, dt);
    m_track_map *= alpha;
    m_track_map += track_combined * (1-alpha);

    // clamp from 0-1
    cv::threshold(m_track_map, m_track_map, 1, 1, cv::THRESH_TRUNC);
    cv::threshold(m_track_map, m_track_map, 0, 0, cv::THRESH_TOZERO);

    TIME_STOP(track)


    /* Plan path forwards */
    TIME_START(plan)
    double lookahead = 2.0;
    double chosen_curvature = pathing::getBestCurvature(m_track_map, Eigen::Vector3d(0, 0, 0), M_PI_2, lookahead, 0, 0);
    // draw planned path on map
    cv::Mat track_annotated;
    cv::cvtColor(m_track_map, track_annotated, cv::COLOR_GRAY2BGR);
    std::vector<cv::Point> path_points = pathing::getArcPixels(Eigen::Vector3d(0, 0, 0), chosen_curvature, lookahead, 10);
    cv::polylines(track_annotated, path_points, false, cv::Scalar(1), 1, cv::LINE_4);


    TIME_STOP(plan)

    TIME_START(stream)
    streamer::imshow("cur", track_combined);
    streamer::imshow("map", track_annotated);
    // streamer::imshow("input", image);
    streamer::imshow("ground", image_corrected);
    streamer::imshow("blue", mask_blue);
    streamer::imshow("yellow", mask_yellow);
    TIME_STOP(stream)

    TIME_STOP(process)

    if(m_frame_counter % 30 == 0){
        printf("\n\n");
        TIME_PRINT(waiting)
        TIME_PRINT(process)
        TIME_PRINT(perspective)
        TIME_PRINT(threshold)
        TIME_PRINT(arrow)
        TIME_PRINT(track)
        TIME_PRINT(plan)
        TIME_PRINT(stream)
    }
    m_frame_counter ++;
    TIME_START(waiting)
    return CarState{1, 0};
}


Vision::Vision(int img_width, int img_height){
    camera::Camera cam{camera::getIntrinsics(img_width, img_height), camera::carToCameraTransform(), img_width, img_height};
    m_perspective_transform = getPerspectiveTransform(cam);
    m_track_map = cv::Mat::zeros(map_height_p, map_width_p, CV_32F);
    streamer::initStreaming();
    // cv::Mat image_right = cv::imread(argv[2]);
    // cv::cvtColor(image_right, image_right, cv::COLOR_BGR2GRAY);

    // cv::Mat image_left = cv::imread(argv[1]);
    // cv::cvtColor(image_left, image_left, cv::COLOR_BGR2GRAY);
}
