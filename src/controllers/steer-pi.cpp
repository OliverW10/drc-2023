#include "steer.hpp"
#include "util.hpp"
#include <pigpio>


const int servo_output_pin = 17;

// pulses in us generally from 1-2
const double servo_full_right_pulse = 1900;
const double servo_full_left_pulse = 1100;

// maxiumum +- 53 of servo, equates to ~+-60 turning

const double servo_full_right_curvature = 1.5;
const double servo_full_left_curvature = -1.5;

double getServoDutyCycle(double curvature){
    // approximates akermann stuff i don't want to work out as linear
    return rescale(curvature,
                   servo_full_left_curvature, servo_full_right_curvature,
                   servo_full_left_pulse, servo_full_right_pulse,
                   true
    );
}

void setup_steer(){
    if (gpioInitialise() < 0) exit(-1);
    gpioSetSignalFunc(SIGINT, stop);
}

void set_steer(){
    double servo_pulse_width = getServoDutyCycle();
    gpioServo(servo_output_pin, servo_pulse_width);
}

void cleanup_steer(){
    gpioTerminate();
}