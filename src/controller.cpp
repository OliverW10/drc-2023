#include "controller.hpp"
#include "util.hpp"
#include "car_state.hpp"

#include "controllers/steer.hpp"

CarState m_estimated_state;

Controller::Controller(){
    setup_steer();
}

void Controller::commandState(CarState state){
    set_steer(state);

    m_estimated_state = state;
}

SensorValues Controller::getSensorValues(){
    return SensorValues{ m_estimated_state, true };
}
