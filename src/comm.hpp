#ifndef COMM_H
#define COMM_H

#include "controller.hpp"

struct Message {
    double speed;
    double turn;
    bool enabled;

    CarState toCarState();
};

void startServer();

// returns if message recent
bool getLatestMessage(Message& message);

#endif