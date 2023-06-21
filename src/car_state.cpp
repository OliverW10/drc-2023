#include "car_state.hpp"
#include <Eigen/Dense>

Eigen::Vector3d getDistForwards(double curvature, double d){
    if(abs(curvature) > 0.001){
        return Eigen::Vector3d(
            (1/curvature) * sin(d*curvature), // dx
            (1/curvature) * (1-cos(d*curvature)), // dy
            curvature * d  // heading
        );
    }else{
        return Eigen::Vector3d(d, 0, 0);
    }
}

Eigen::Vector3d getDistForwards(double curvature, double d, const Eigen::Vector3d& start){
    Eigen::Rotation2Dd rot_mat(start(2));
    Eigen::Vector3d delta = getDistForwards(curvature, d);
    Eigen::Vector3d end = Eigen::Vector3d::Ones();
    end = start;
    // add position delta rotated by initial heading
    end.block<2, 1>(0, 0) += rot_mat * delta.block<2, 1>(0, 0);
    end(2) += delta(2);
    return end;
}

Eigen::Vector3d CarState::getTimeForwards(double t, const Eigen::Vector3d& start){
    double d = speed * t;
    return getDistForwards(curvature, d, start);
}

Eigen::Vector3d CarState::getTimeForwards(double t){
    double d = speed * t;
    return getDistForwards(curvature, d);
}