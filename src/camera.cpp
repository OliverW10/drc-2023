#include "camera.hpp"
#include "util.hpp"
#include <opencv2/opencv.hpp>
#include "vision.hpp"

namespace camera{

void viewToScreen(Eigen::Vector4d& target){
    Eigen::Matrix4d transform;
    transform << 0, -1, 0, 0,
                 0, 0, -1, 0,
                 1, 0, 0, 0,
                 0, 0, 0, 1;
    target = transform * target;
}

Eigen::Vector3d projectPoint(const Camera& cam, const Eigen::Vector4d& p){
    // transform to be relative to camera, in car coordinate system
    Eigen::Vector4d pos = cam.extrinsics.inverse() * p;
    // covert to camera coordinate system
    viewToScreen(pos);
    // apply perspective
    pos = pos / pos(2);
    // convert from meters to pixels
    return cam.intrinsics * pos.block<3, 1>(0, 0);
}

cv::Point2f projectPointCv(const Camera& cam, Eigen::Vector4d p){
    Eigen::Vector3d pixel = projectPoint(cam, p);
    return cv::Point2f(pixel(0), pixel(1));
}


int getHorizon(const Camera& cam, double dist){
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

void pixelToRayDir(const Eigen::Vector2d& pixel, const Camera& cam, Eigen::Vector4d& rayDir){
    rayDir(0) = cam.intrinsics(0, 0);
    rayDir(1) = -(pixel(0) - cam.intrinsics(0, 2));
    rayDir(2) = -(pixel(1) - cam.intrinsics(1, 2));
    rayDir(3) = 1;
    rayDir = cam.extrinsics*rayDir;
}

bool pixelToFloorPos(Eigen::Vector2d pixel, const Camera& cam, Eigen::Vector4d& ret){
    Eigen::Vector4d rayStart = cam.extrinsics * Eigen::Vector4d(0, 0, 0, 1);

    Eigen::Vector4d rayEnd;
    pixelToRayDir(pixel, cam, rayEnd);
    return rayFloorIntersect(rayStart.block<3, 1>(0, 0), rayEnd.block<3, 1>(0, 0), ret);
}

Eigen::Matrix3d getIntrinsics(int width, int height){
    // TODO: load from config
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = 754;
    intrinsics(1, 1) = 754;
    intrinsics(0, 2) = width/2;
    intrinsics(1, 2) = height/2;
    return intrinsics;
}

Eigen::Matrix4d carToCameraTransform(double angle){
    // TODO: load from config
    Eigen::Matrix4d extrinsics = Eigen::Matrix4d::Identity();
    double camera_angle = radians(angle); // 15-20 seems good
    extrinsics.block<3,3>(0, 0) =  Eigen::AngleAxisd(camera_angle, Eigen::Vector3d::UnitY()).matrix();
    extrinsics.block<3,1>(0, 3) = Eigen::Vector3d(0, 0, 0.4);
    return extrinsics;
}

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

}