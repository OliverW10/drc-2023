#include "controller.hpp"
#include <Eigen/Dense>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
    Eigen::Vector3d delta = getDistForwards(curvature, d);
    Eigen::Vector3d end = Eigen::Vector3d::Ones();
    end = start;
    // add position delta rotated by initial heading
    end.block<2, 1>(0, 0) += delta.block<2, 1>(0, 0);
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

Controller::Controller(){

}

void Controller::commandState(CarState state){
    // printf("commanding state: vx: %f, omega: %f\n", state.speed, state.curvature);
}

SensorValues Controller::getSensorValues(){
    return SensorValues{0.5, 0};
}
