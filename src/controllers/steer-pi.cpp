#include "steer.hpp"
#include "../util.hpp"
#include "pigpio.h"
#include "signal.h"


const int servo_output_pin = 7;

// pulses in us generally from 1-2
const double servo_full_right_pulse = 1900;
const double servo_full_left_pulse = 1100;

// maxiumum +- 53 of servo, equates to ~+-60 turning

const double servo_full_right_curvature = 1.5;
const double servo_full_left_curvature = -1.5;

double curvatureToAngle(double curvature){
    return curvature;
}

double getServoDutyCycle(double curvature){
    // approximates akermann stuff i don't want to work out as linear
    return rescale(curvatureToAngle(curvature),
                   servo_full_left_curvature, servo_full_right_curvature,
                   servo_full_left_pulse, servo_full_right_pulse,
                   true
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

void set_steer(double curvature){
    double servo_pulse_width = getServoDutyCycle(curvature);
    gpioServo(servo_output_pin, servo_pulse_width);
}
