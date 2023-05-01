#include "vision.hpp"
#include "util.h"
#include <Eigen/Dense>

struct Camera{
    Eigen::Matrix3d intrinsics;
    Eigen::Matrix4d extrinsics;
    int frame_width;
    int frame_height;
};

// converts coordinate system that the car uses to what the camera uses 
void viewToScreen(Eigen::Vector4d& target){
    Eigen::Matrix4d transform;
    transform << 0, -1, 0, 0,
                 0, 0, -1, 0,
                 1, 0, 0, 0,
                 0, 0, 0, 1;
    target = transform * target;
}

// projects a 3d point onto the screen
Eigen::Vector3d projectPoint(const Camera& cam, const Eigen::Vector4d& p){
    // transform to be relative to camera, in car coordinate system
    Eigen::Vector4d pos = cam.extrinsics.inverse() * p;
    // covert to camera coordinate system
    viewToScreen(pos);
    // std::cout << "\nintr:\n" << intrinsics << "\ninvr:\n" << intrinsics.inverse() << "\n";
    // std::cout << "\nrel:\n" << pos << "\n";
    // apply perspective
    pos = pos / pos(2);
    // convert from meters to pixels
    return cam.intrinsics * pos.block<3, 1>(0, 0);
}

// projects a 3d point onto the screen as a cv::Point2f
cv::Point2f projectPointCv(const Camera& cam, Eigen::Vector4d p){
    Eigen::Vector3d pixel = projectPoint(cam, p);
    return cv::Point2f(pixel(0), pixel(1));
}


int getHorizon(const Camera& cam, double dist = 10){
    Eigen::Vector4d p(dist, 0, 0, 1);
    Eigen::Vector3d screenPoint = projectPoint(cam, p);
    return (int)screenPoint(1);
}

bool rayFloorIntersect(Eigen::Vector3d rayA, Eigen::Vector3d rayB, Eigen::Vector4d& ret){
    Eigen::Vector3d rayDir = rayB - rayA;
    // ray towards ground
    if(sign(rayDir(2)) != sign(-rayA(2))){
        return false;
    }
    ret(0) = rayA(0) + (-rayA(2) / rayDir(2)) * rayDir(0);
    ret(1) = rayA(1) + (-rayA(2) / rayDir(2)) * rayDir(1);
    ret(2) = 0;
    ret(3) = 1;
    return true;
}

// gets the ray direction vector from a pixel
void pixelToRayDir(const Eigen::Vector2d& pixel, const Camera& cam, Eigen::Vector4d& rayDir){
    rayDir(0) = cam.intrinsics(0, 0);
    rayDir(1) = -(pixel(0) - cam.intrinsics(0, 2));
    rayDir(2) = -(pixel(1) - cam.intrinsics(1, 2));
    rayDir(3) = 1;
    rayDir = cam.extrinsics*rayDir;
}

// gets the position on the floor that the pixel is looking at
bool pixelToFloorPos(Eigen::Vector2d pixel, const Camera& cam, Eigen::Vector4d& ret){
    Eigen::Vector4d rayStart = cam.extrinsics * Eigen::Vector4d(0, 0, 0, 1);

    Eigen::Vector4d rayEnd;
    pixelToRayDir(pixel, cam, rayEnd);
    return rayFloorIntersect(rayStart.block<3, 1>(0, 0), rayEnd.block<3, 1>(0, 0), ret);
}

Eigen::Vector3d getIntrinsics(){
    // TODO: load from config
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Zero();
    intrinsics(0, 0) = 753.5;
    intrinsics(1, 1) = 754.3;
    intrinsics(0, 2) = 318.8;
    intrinsics(1, 2) = 244.6;
    intrinsics(2, 2) = 1;
    return intrinsics;
}

Eigen::Matrix4d carToCameraTransform(){
    Eigen::Matrix4d extrinsics;
    double camera_angle = radians(15); // 15-20 seems good
    extrinsics.block<3,3>(0, 0) =  Eigen::AngleAxisd(camera_angle, Eigen::Vector3d::UnitY()).matrix();
    extrinsics.block<3,1>(0, 3) = Eigen::Vector3d(0, 0, 0.3);
    extrinsics(3, 3) = 1;
    return extrinsics;
}

double pixels_per_meter = 100;
double map_width  = 5;
double map_height = 5;
int map_width_p  = (int)(map_width  * pixels_per_meter);
int map_height_p = (int)(map_height * pixels_per_meter);

// gets the matrix to pass to warpPerspective that corrects for the perspective of the
// ground and maps the image to the map
cv::Mat getPerspectiveTransformFromView(const Camera& cam){
    double view_depth = cam.frame_width / pixels_per_meter;
    double view_width = cam.frame_height / pixels_per_meter;
    cv::Point2f source[4];
    source[0] = projectPointCv(cam, Eigen::Vector3d(0,          -map_width/2));
    source[1] = projectPointCv(cam, Eigen::Vector3d(0,          map_width/2));
    source[2] = projectPointCv(cam, Eigen::Vector3d(map_height, -map_width/2));
    source[3] = projectPointCv(cam, Eigen::Vector3d(map_height, map_width/2));
    cv::Point2f dest[4];
    dest[0] = cv::Point2f(map_width_p, map_height_p);
    dest[1] = cv::Point2f(0,           map_height_p);
    dest[2] = cv::Point2f(map_width_p, 0);
    dest[3] = cv::Point2f(0,           0);

    // for(auto x : input){
    //     std::cout << x << "\n";
    // }
    return cv::getPerspectiveTransform(source, dest);
}

// convert a position relative to car to pixel position in track_map
cv::Point posToMap(const Eigen::Vector3d& position){
    // car is at bottom middle of map
    return cv::Point(map_width/2 - (position(1)*pixels_per_meter), map_height - (position(0)*pixels_per_meter));
}

cv::Mat getPotentialTrack(cv::Mat tape_mask, double track_mid_dist = 1, double track_width = 1.5){
    /*
    track_mid_dist: typical distance from tape to track center line
    track_width: width of area to mark as track
    */
    cv::erode(tape_mask, tape_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));
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


// get samples along the arc
std::vector<Eigen::Vector3d> getArcPoints(const Eigen::Vector3d& start, double curvature, double dist, int num_samples = 50){
    std::vector<Eigen::Vector3d> points;
    // pixels forwards per sample
    double dx = dist/num_samples;
    for(int i = 0; i < num_samples; i++){
        double d = i*dx;
        points.push_back(getDistForwards(curvature, d, start));
    }
    return points;
}

// get samples along the arc in map pixels
std::vector<cv::Point> getArcPixels(const Eigen::Vector3d& start, double curvature, double dist, int num_samples = 50){
    std::vector<Eigen::Vector3d> points = getArcPoints(start, curvature, dist, num_samples);
    std::vector<cv::Point> pixels(points.size());
    for(int i = 0; i < points.size(); i++){
        pixels[i] = posToMap(points[i]);
    }
    return pixels;
}

// gets the average track weight along arc
double getArcFittness(const cv::Mat& track_map, const Eigen::Vector3d& pos, double curvature, double dist){
    std::vector<cv::Point> points = getArcPixels(pos, curvature, dist);
    double total = 0;
    for(auto point : points){
        total += pow(track_map.at<float>(point.y, point.x), 1); // 2);
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

CarState Vision::process(const cv::Mat& image, CarState cur_state){
    auto process_start_time = std::chrono::system_clock::now();
    cv::Mat image_corrected;
    // undo perspective
    cv::warpPerspective(image, image_corrected, perspective_transform, cv::Size(640, 480));
    // convert to hsv
    cv::Mat hsv;
    cv::cvtColor(image_corrected, hsv, cv::COLOR_BGR2HSV);
    // get masks for yellow, blue and purple
    cv::Mat mask_yellow;
    cv::inRange(hsv, cv::Scalar(25, 40, 30), cv::Scalar(40, 255, 255), mask_yellow);
    cv::Mat mask_blue;
    cv::inRange(hsv, cv::Scalar(100, 40, 50), cv::Scalar(140, 255, 255), mask_blue);
    cv::Mat mask_purple;
    cv::inRange(hsv, cv::Scalar(130, 40, 50), cv::Scalar(150, 255, 255), mask_purple);
    cv::Mat mask_red;
    // have to do not red and invert it because red is at both ends of hue 
    cv::inRange(hsv, cv::Scalar(10, 40, 50), cv::Scalar(170, 255, 255), mask_red);
    mask_red = 255-mask_red;

    // get drivable areas
    cv::Mat track_right = getPotentialTrack(mask_yellow);
    cv::Mat track_left = getPotentialTrack(mask_blue);
    cv::Mat track_combined = (track_left + track_right)/2;

    // TODO:
    // move past map by -carstate * dt
    // add track_combined to map

    double lookahead = 2;
    double chosen_curvature = getBestCurvature(track_combined, Eigen::Vector3d(0, 0, 0), M_PI_2, lookahead);
    // draw planned path on map
    cv::Mat track_annotated = track_combined.clone();
    cv::cvtColor(track_annotated, track_annotated, cv::COLOR_GRAY2BGR);
    std::vector<cv::Point> path_points = getArcPixels(Eigen::Vector3d(map_width/2, map_height, 0), chosen_curvature, lookahead);
    cv::polylines(track_annotated, path_points, false, cv::Scalar(255, 0, 0));

    process_total += (std::chrono::system_clock::now() - process_start_time).count();

    if(frame_counter % 10 == 0){
        printf("process time: %fms\n", to_ms(process_total)/10);
        process_total = 0;
    }
    frame_counter ++;
    return CarState{1, chosen_curvature};
}


Vision::Vision(){
    // cv::Mat image_right = cv::imread(argv[2]);
    // cv::cvtColor(image_right, image_right, cv::COLOR_BGR2GRAY);

    // cv::Mat image_left = cv::imread(argv[1]);
    // cv::cvtColor(image_left, image_left, cv::COLOR_BGR2GRAY);
}
