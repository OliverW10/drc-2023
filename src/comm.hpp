#ifndef COMM_H
#define COMM_H

#include "controller.hpp"
#include <stdint.h>

struct Message {
    double speed;
    double turn;
    bool enabled;
    unsigned int id;

    CarState toCarState();
};

void runDriveServer();

// returns if message recent
bool getLatestMessage(Message& message);

#endif