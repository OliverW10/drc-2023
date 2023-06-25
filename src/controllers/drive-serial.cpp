#include "drive.hpp"
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include "../util.hpp"

FILE* m_serial_file;


const double chassis_width = 0.2;
const double wheel_diameter = 0.1;
const double wheel_circumference = M_PI * wheel_diameter;

void setup_drive(){
    const char* filename = "/dev/ttyACM0";
    m_serial_file = fopen(filename, "w");
    if(!m_serial_file){
        printf("serial file %s does not exist\n", filename);
        exit(-1);
    }

    // https://stackoverflow.com/questions/4968529/how-can-i-set-the-baud-rate-to-307-200-on-linux
    struct termios options;
    tcgetattr(fileno(m_serial_file), &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    tcsetattr(fileno(m_serial_file), TCSANOW, &options);
}

void set_drive(CarState state){
    double turn_rate = state.curvature * state.speed;
    double left_wheel_speed  = (state.speed + turn_rate * chassis_width) / wheel_circumference;
    double right_wheel_speed = (state.speed - turn_rate * chassis_width) / wheel_circumference;
    // https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html#ascii-protocol
    fprintf(m_serial_file, "v 0 %f", roundPlaces(left_wheel_speed, 2));
    fprintf(m_serial_file, "v 1 %f", roundPlaces(right_wheel_speed, 2));
}

void cleanup_drive(){
    fclose(m_serial_file);
}