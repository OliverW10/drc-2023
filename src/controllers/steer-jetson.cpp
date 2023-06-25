#include "steer.hpp"
#include <JetsonGPIO.h>
#include <math.h>
#include "../util.hpp"

const int servo_output_pin = 33;
const int servo_frequency = 50;

// pulses in ms generally from 1-2
const double servo_full_right_pulse = 1.9;
const double servo_full_left_pulse = 1.1;

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

GPIO::PWM* pwm;

void setup_steer(){
    // https://github.com/pjueon/JetsonGPIO/blob/master/samples/simple_pwm.cpp
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(servo_output_pin, GPIO::OUT, GPIO::HIGH);
    pwm = new GPIO::PWM(servo_output_pin, servo_frequency);
    pwm->start(50);
}

void set_steer(double curvature){
    double servo_duty_cycle = getServoDutyCycle(curvature);
    pwm->ChangeDutyCycle(servo_duty_cycle);
}

void cleanup_steer(){
    pwm->stop();
    GPIO::cleanup();
}