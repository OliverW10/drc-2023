#ifndef UTIL_H
#define UTIL_H

double to_ms(long x){
    return (double)(x/1000)/1000;
}

#define PI 3.14159265
double radians(double degrees){
    return degrees * (PI / 180);
}

double sign(double x){
    return x>=0 ? 1 : -1;
}

#endif