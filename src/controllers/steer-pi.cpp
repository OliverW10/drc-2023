#include "steer.hpp"
#include "../util.hpp"
#include "pigpio.h"
#include "signal.h"
#include <algorithm>
#include "../config.hpp"


const int servo_output_pin = 4;

// maxiumum +- 53 of servo, equates to ~+-60 turning
const double servo_max = 1.05;

const int drive_left_output_pin = 27;
const int drive_right_output_pin = 22;

double getDriveDutyCycle(double wheel_speed){
    double drive_stop_pwm = getConfigDouble("drive_stop");
    // duty cyle to go at 1 meter/s, will extrapolate for higher speeds
    double drive_1mps_pwm = getConfigDouble("drive_one");
    // maxiumum speed as a duty cycle
    double drive_min_pwm = getConfigDouble("drive_min");
    double drive_max_pwm = getConfigDouble("drive_max");

    double unclamped_pwm = rescale(wheel_speed,
        0, 1,
        drive_stop_pwm, drive_1mps_pwm,
        false
    );
    return std::clamp(unclamped_pwm, drive_min_pwm, drive_max_pwm);
}


double curvatureToAngle(double curvature){
    // servo angle to curvature is pretty close to linear for low turn angles
    return curvature;
}

double getServoDutyCycle(double curvature){
    double servo_center_pwm = getConfigDouble("steer_center");
    double servo_right_pwm = getConfigDouble("steer_right");
    return rescale(curvatureToAngle(curvature),
                   0, servo_max,
                   servo_center_pwm, servo_right_pwm,
                   false
    );
}


void stop(int signum){
    gpioTerminate();
    puts("Called gpioTerminate");
    exit(0);
}

void setup_steer(){
    if (gpioInitialise() < 0) exit(-1);
    gpioSetSignalFunc(SIGINT, stop);
}


const double chassis_width = 0.2;
const double wheel_diameter = 0.15;
const double wheel_circumference = M_PI * wheel_diameter;

void set_steer(CarState state){
    double servo_pulse_width = getServoDutyCycle(state.curvature);
    gpioServo(servo_output_pin, servo_pulse_width);

    double turn_rate = state.curvature * state.speed;

    double left_wheel_speed  = (state.speed + turn_rate * chassis_width) / wheel_circumference;
    gpioServo(drive_left_output_pin, getDriveDutyCycle(left_wheel_speed));

    double right_wheel_speed = (state.speed - turn_rate * chassis_width) / wheel_circumference;
    gpioServo(drive_right_output_pin, getDriveDutyCycle(right_wheel_speed));
}
