#include "controller.hpp"
#include "util.hpp"
#include "car_state.hpp"

#include "controllers/drive.hpp"
#include "controllers/steer.hpp"

CarState m_estimated_state;

Controller::Controller(){
    setup_steer();
    setup_drive();
}

void Controller::commandState(CarState state){
    set_steer(state.curvature);
    set_drive(state);
    
    // TODO: apply smoothing to better estimate actuals speeds
    m_estimated_state = state;
}

Controller::~Controller(){
    cleanup_steer();
    cleanup_drive();
}

SensorValues Controller::getSensorValues(){
    return SensorValues{ m_estimated_state, true };
}
