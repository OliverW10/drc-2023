#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <Eigen/Core>
#include "car_state.hpp"

#define DO_JETSON 0
#define DO_PI 0
#define DO_SERIAL 0

#if DO_JETSON
    #include <JetsonGPIO.h>
#endif
#if DO_PI
    #include <pigpio>
#endif

// will communicate with arduino with something like serial which will control the motors and other actuators
// the arduino may also read some sensors
class Controller{
public:
    Controller();
    ~Controller();
    void commandState(CarState state);
    SensorValues getSensorValues();
private:
    FILE* m_serial_file;
    CarState m_commanded_state;

#if DO_COMMAND
    GPIO::PWM* m_pwm;
#endif
};

#endif