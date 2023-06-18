#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <Eigen/Core>

#define DO_COMMAND 0

#if DO_COMMAND
    #include <JetsonGPIO.h>
#endif

Eigen::Vector3d getDistForwards(double curvature, double d);
Eigen::Vector3d getDistForwards(double curvature, double d, const Eigen::Vector3d& start);

struct CarState{
    // speed m/s
    double speed;
    // curvature, angle change per meter forwards, 1/radius
    double curvature;
    Eigen::Vector3d getTimeForwards(double t);
    Eigen::Vector3d getTimeForwards(double t, const Eigen::Vector3d& start);
};

struct SensorValues{
    CarState state;
    bool toggle;
};

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