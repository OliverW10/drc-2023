# UTS DRC 2023

Software to control a car for the [QUT Droid racing Challenge (DRC)](https://qutrobotics.com/drc)


video streams run on port 8080

## To build on ubuntu or similar:
install OpenCV (libopencv-dev) and Eigen (libeigen3-dev) from apt

install https://github.com/nadjieb/cpp-mjpeg-streamer

if running on a raspberry pi
    install https://github.com/joan2937/pigpio
    i gave up on using cmake properly so just hardcode the include path (found in install_manifest.txt) in the include_directories on line 28 and add the lib path to LD_LIBRARY_PATH, if pigpio wants sudo add the ld library path [export to bashrc](https://unix.stackexchange.com/a/242886) so sudo gets it

if running on jetson
    cmake should automatically setup https://github.com/pjueon/JetsonGPIO


build with cmake and pass either -DForPI=ON or -DForJetson=ON or neither to use mocks


### Source files:
 - `main.cpp`: runs the vision pipeline from a camera
 - `replay.cpp`: runs the vision pipeline on a video
 - `vision`: main vision processing, generates a map of the track and gets a path, calls all other modules
 - `arrow`: detect and classify arrows
 - `camera`: functions to calculate a perspective transform
 - `comm`: communicates with computer for manual control using udp
 - `config`: hot reloading config file
 - `controller`: communication with hardware to control motors/servos and to get sensor data
 - `controllers/`: hardware specific controller implementation
 - `obsticle`: detect obstacles (other cars and boxes)
 - `pathing`: functions for planning a path given a map
 - `streamer`: streams debug views over http
