#include "controller.hpp"
#include <Eigen/Dense>
#include <stdio.h>

Eigen::Vector3d getDistForwards(double curvature, double d){
    Eigen::Vector3d delta;
    delta << 1/curvature * sin(d*curvature), // x
             1/curvature * (1-cos(d*curvature)), // y
             curvature * d; // heading
    return delta;
}

Eigen::Vector3d getDistForwards(double curvature, double d, const Eigen::Vector3d& start){
    Eigen::Vector3d delta = getDistForwards(curvature, d);
    Eigen::Vector3d end;
    // add to initial position
    end.block<2, 0>(0, 0) = start.block<2,0>(0, 0);
    // add position delta rotated by initial heading
    end.block<2, 0>(0, 0) = Eigen::Rotation2D<double>(start(2)).matrix() * delta.block<2, 0>(0, 0);
    // final heading
    end(2) = delta(2) + start(2);
    return end;
}

Eigen::Vector3d CarState::getTimeForwards(double t, const Eigen::Vector3d& start){
    double d = speed * t;
    return getDistForwards(curvature, d, start);
}

Controller::Controller(){

}

void Controller::commandState(CarState state){
    // printf("commanding state: vx: %f, omega: %f\n", state.speed, state.curvature);
}
SensorValues Controller::getSensorValues(){
    return SensorValues{0, 0};
}