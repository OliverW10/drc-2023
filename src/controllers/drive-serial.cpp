#include "drive.hpp"
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include "../util.hpp"

FILE* m_serial_file;


const double chassis_width = 0.2;
const double wheel_diameter = 0.15;
const double wheel_circumference = M_PI * wheel_diameter;

void cleanup_drive(){
    fprintf(m_serial_file, "v 0 0\n");
    fprintf(m_serial_file, "v 1 0\n");
    printf("set odrive speeds to 0\n");
    fclose(m_serial_file);
}

void setup_drive(){
    const char* filename = "/dev/ttyACM0";
    m_serial_file = fopen(filename, "w");
    if(!m_serial_file){
        printf("could not open file %s\n", filename);
        exit(-1);
    }
    printf("Successfully opened serial file %s\n", filename);

    // https://stackoverflow.com/questions/4968529/how-can-i-set-the-baud-rate-to-307-200-on-linux
    struct termios options;
    tcgetattr(fileno(m_serial_file), &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    tcsetattr(fileno(m_serial_file), TCSANOW, &options);

    atexit(cleanup_drive);
}

void set_drive(CarState state){
    // std::cout << "state (speed, curv): " << state.speed << ", " << state.curvature;
    double turn_rate = state.curvature * state.speed;
    double left_wheel_speed  = (state.speed + turn_rate * chassis_width) / wheel_circumference;
    double right_wheel_speed = (state.speed - turn_rate * chassis_width) / wheel_circumference;
    // std::cout << "\nright: " << right_wheel_speed << ", left: " << left_wheel_speed << "\n";
    // https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html#ascii-protocol
    fprintf(m_serial_file, "v 0 %f\n", roundPlaces(left_wheel_speed/wheel_circumference, 2));
    fprintf(m_serial_file, "v 1 %f\n", roundPlaces(right_wheel_speed/wheel_circumference, 2));
}
