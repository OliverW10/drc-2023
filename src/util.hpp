#ifndef UTIL_H
#define UTIL_H

#include <iostream>

#define PI 3.14159265
double radians(double degrees){
    return degrees * (PI / 180);
}

double sign(double x){
    return x>=0 ? 1 : -1;
}

#define TIMING

// should call TIME_INIT once with the name of the timer, then call alternate calling start and stop
// then call TIME_PRINT to print out the average time and reset
#ifdef TIMING
    #define TIME_INIT(NAME) auto NAME ## _start_time = std::chrono::high_resolution_clock::now(); \
        double NAME ## _total_time = 0; \
        int NAME ## _iters = 0;
    #define TIME_START(NAME) NAME ## _start_time = std::chrono::high_resolution_clock::now();
    #define TIME_STOP(NAME) NAME ## _total_time += std::chrono::duration_cast<std::chrono::microseconds>( \
        std::chrono::high_resolution_clock::now() - NAME ## _start_time \
        ).count(); \
        NAME ## _iters ++;
    #define TIME_PRINT(NAME) std::cout << #NAME ": " << NAME ## _total_time / NAME ## _iters / 1000 << "ms/iter\n" ; \
        NAME ## _total_time = 0; \
        NAME ## _iters = 0;
#else
    #define TIME_INIT(NAME)
    #define TIME_START(NAME)
    #define TIME_STOP(NAME)
    #define TIME_PRINT(NAME)
#endif // TIMING

#endif // UTIL_H