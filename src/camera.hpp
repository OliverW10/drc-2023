#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>
#include "util.hpp"
#include <opencv2/core/types.hpp>


namespace camera{

// utils for 3d
struct Camera{
    Eigen::Matrix3d intrinsics;
    Eigen::Matrix4d extrinsics;
    int frame_width;
    int frame_height;
};

// projects a 3d point onto the screen
Eigen::Vector3d projectPoint(const Camera& cam, const Eigen::Vector4d& p);

// projects a 3d point onto the screen as a cv::Point2f
cv::Point2f projectPointCv(const Camera& cam, Eigen::Vector4d p);

int getHorizon(const Camera& cam, double dist = 10);

// gets the ray direction vector from a pixel
void pixelToRayDir(const Eigen::Vector2d& pixel, const Camera& cam, Eigen::Vector4d& rayDir);

// gets the position on the floor that the pixel is looking at
bool pixelToFloorPos(Eigen::Vector2d pixel, const Camera& cam, Eigen::Vector4d& ret);

Eigen::Matrix3d getIntrinsics(int width, int height);

Eigen::Matrix4d carToCameraTransform(double angle, double height);

cv::Mat getPerspectiveTransform(const camera::Camera& cam);

}
#endif