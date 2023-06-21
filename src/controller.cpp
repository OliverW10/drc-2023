#include "controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <termios.h>
#include "util.hpp"
#include "car_state.hpp"


const double chassis_width = 0.2;
const double wheel_diameter = 0.1;
const double wheel_circumference = M_PI * wheel_diameter;

const int servo_output_pin = 33;
const int servo_frequency = 50;
// period in ms, for most servos is 20ms
const double servo_period = 1000/servo_frequency;

// percent duty cycle (0-100) for 90 degree turn
// 2ms
const double servo_full_right = 2/servo_period;
// 1ms
const double servo_full_left = 1/servo_period;

double getServoDutyCycleFromAngle(double servo_angle){
    double t = (servo_angle/(M_PI*2))+0.5;
    // TODO: check signs
    return t * servo_full_right + (1-t) * servo_full_left;
}

double getServoDutyCycle(double curvature){
    // TODO: convert from servo angle to turning curvature
    return getServoDutyCycleFromAngle(curvature);
}

Controller::Controller() {
    #if DO_SERIAL
        const char* filename = "/dev/ttyACM0";
        m_serial_file = fopen(filename, "w");
        if(!m_serial_file){
            printf("serial file %s does not exist\n", filename);
            exit(-1);
        }

        // https://stackoverflow.com/questions/4968529/how-can-i-set-the-baud-rate-to-307-200-on-linux
        termios options;
        tcgetattr(fileno(m_serial_file), &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        tcsetattr(fileno(m_serial_file), TCSANOW, &options);
    #endif

    #if DO_JETSON
        // https://github.com/pjueon/JetsonGPIO/blob/master/samples/simple_pwm.cpp
        GPIO::setmode(GPIO::BOARD);
        GPIO::setup(servo_output_pin, GPIO::OUT, GPIO::HIGH);
        m_pwm = new GPIO::PWM(servo_output_pin, servo_frequency);
        m_pwm->start(50);
    #endif

    #if DO_PI
        if (gpioInitialise() < 0) return -1;
        gpioSetSignalFunc(SIGINT, stop);
    #endif
}

void Controller::commandState(CarState state){
    double turn_rate = state.curvature * state.speed;
    double left_wheel_speed  = (state.speed + turn_rate * chassis_width) / wheel_circumference;
    double right_wheel_speed = (state.speed - turn_rate * chassis_width) / wheel_circumference;
    double servo_duty_cycle = getServoDutyCycle(state.curvature);
    #if DO_SERIAL
        // https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html#ascii-protocol
        fprintf(m_serial_file, "v 0 %d", roundPlaces(left_wheel_speed, 2));
        fprintf(m_serial_file, "v 1 %d", roundPlaces(right_wheel_speed, 2));
    #endif

    #if DO_JETSON
        m_pwm->ChangeDutyCycle(servo_duty_cycle);
    #endif

    #if DO_PI
        gpioServo();
    #endif

    // std::cout
    //     << "wheel speeds: "
    //     << left_wheel_speed
    //     << ", "
    //     << right_wheel_speed
    //     << "\t pwm: "
    //     << servo_duty_cycle
    //     << "\n";

    // TODO: apply smoothing to better estimate actuals speeds
    m_commanded_state = state;
}

Controller::~Controller(){
    #if DO_SERIAL
        fclose(m_serial_file);
    #endif    
    #if DO_JETSON
        m_pwm->stop();
        GPIO::cleanup();
    #endif
    #if DO_PI
        gpioTerminate();
    #endif
}

SensorValues Controller::getSensorValues(){
    return SensorValues{ m_commanded_state };
}
