#include "controller.hpp"
#include <Eigen/Dense>
#include <stdlib.h>
#include <math.h>
#include <iostream>

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

// https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html#ascii-protocol

const double chassis_width = 0.2;
const double wheel_diameter = 0.1;
const double wheel_circumference = M_PI * wheel_diameter;

#define DO_COMMAND 0

Controller::Controller() {
    #if DO_COMMAND
        m_serial_file.open("/dev/ttyACM0");
    #endif
}

void Controller::commandState(CarState state){
    double turn_rate = state.curvature * state.speed;
    double left_wheel_speed  = (state.speed + turn_rate * chassis_width) / wheel_circumference;
    double right_wheel_speed = (state.speed - turn_rate * chassis_width) / wheel_circumference;
    #if DO_COMMAND
        m_serial_file << "v 0 " <<  left_wheel_speed << "\n";
        m_serial_file << "v 1 " <<  -right_wheel_speed << "\n";
    #endif
    std::cout << "sending wheel speeds: " << left_wheel_speed << ", " << right_wheel_speed << "\n";

    // TODO: write pwm to a gpio for servo
    
    // TODO: apply smoothing to better estimate actuals speeds
    m_commanded_state = state;
}

Controller::~Controller(){
    m_serial_file.close();
}

SensorValues Controller::getSensorValues(){
    return SensorValues{ m_commanded_state };
}
