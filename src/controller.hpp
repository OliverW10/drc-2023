// this file probably needs a better name
#ifndef CONTROLLER_H
#define CONTROllER_H

#include <Eigen/Core>

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

// interfaces with hardware
class Controller{
    public:
    Controller();
    void commandState(CarState state);
    CarState getState();
};

#endif