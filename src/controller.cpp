#include "controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <termios.h>
#include "util.hpp"

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


const double chassis_width = 0.2;
const double wheel_diameter = 0.1;
const double wheel_circumference = M_PI * wheel_diameter;

const int servo_output_pin = 33;
const int servo_frequency = 50;
// period in ms, for most servos is 20ms
const double servo_period = 1000/servo_frequency;

// percent duty cycle (0-100) for 90 degree turn
// 2ms
const double servo_full_right = 2/servo_period;
// 1ms
const double servo_full_left = 1/servo_period;

double getServoDutyCycleFromAngle(double servo_angle){
    double t = (servo_angle/(M_PI*2))+0.5;
    // TODO: check signs
    return t * servo_full_right + (1-t) * servo_full_left;
}

double getServoDutyCycle(double curvature){
    // TODO: convert from servo angle to turning curvature
    return getServoDutyCycleFromAngle(curvature);
}

Controller::Controller() {
    #if DO_COMMAND
        m_serial_file = fopen("/dev/ttyACM0", "w");

        // https://stackoverflow.com/questions/4968529/how-can-i-set-the-baud-rate-to-307-200-on-linux
        termios options;
        tcgetattr(fileno(m_serial_file), &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        tcsetattr(fileno(m_serial_file), TCSANOW, &options);

        // https://github.com/pjueon/JetsonGPIO/blob/master/samples/simple_pwm.cpp
        GPIO::setmode(GPIO::BOARD);
        GPIO::setup(servo_output_pin, GPIO::OUT, GPIO::HIGH);
        m_pwm = new GPIO::PWM(servo_output_pin, servo_frequency);
        m_pwm->start(50);
    #endif
}

void Controller::commandState(CarState state){
    double turn_rate = state.curvature * state.speed;
    double left_wheel_speed  = (state.speed + turn_rate * chassis_width) / wheel_circumference;
    double right_wheel_speed = (state.speed - turn_rate * chassis_width) / wheel_circumference;
    double servo_duty_cycle = getServoDutyCycle(state.curvature);
    #if DO_COMMAND
        // https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html#ascii-protocol
        fprintf(m_serial_file, "v 0 %", roundPlaces(left_wheel_speed, 2));
        fprintf(m_serial_file, "v 1 %", roundPlaces(right_wheel_speed, 2));
        m_pwm->ChangeDutyCycle(servo_duty_cycle);
    #endif


    // std::cout
    //     << "wheel speeds: "
    //     << left_wheel_speed
    //     << ", "
    //     << right_wheel_speed
    //     << "\t pwm: "
    //     << servo_duty_cycle
    //     << "\n";

    // TODO: apply smoothing to better estimate actuals speeds
    m_commanded_state = state;
}

Controller::~Controller(){
    #if DO_COMMAND
        fclose(m_serial_file);
        m_pwm->stop();
        GPIO::cleanup();
    #endif    
}

SensorValues Controller::getSensorValues(){
    return SensorValues{ m_commanded_state };
}
