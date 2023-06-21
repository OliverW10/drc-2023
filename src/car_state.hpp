#ifndef CARSTATE_H
#define CARSTATE_H

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

struct SensorValues{
    CarState state;
    bool toggle;
};

#endif