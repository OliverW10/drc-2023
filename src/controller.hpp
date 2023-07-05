#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "car_state.hpp"

// handles controlling the hardware
class Controller{
public:
    Controller();
    void commandState(CarState state);
    SensorValues getSensorValues();
};

#endif