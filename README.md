# UTS DRC 2023


### To run on ubuntu or similar:
install OpenCV (libopencv-dev) and Eigen (libeigen3-dev) from apt

install https://github.com/nadjieb/cpp-mjpeg-streamer

build with cmake and run ./Main


### Source files:
 - `vision.cpp`: main vision processing, generates a map of the track and gets a path
 - `pathing.cpp`: functions for planning a path given a map
 - `camera.cpp`: functions to calculate a perspective transform
 - `config.cpp`: hot reloading config file
 - `controller.cpp`: communication with microcontroller to control motors/servos and to get sensor data
 - `streamer.cpp`: streams debug views over http
 - `replay.cpp`: runs the vision pipeline on a video
 - `main.cpp`: runs the vision pipeline from a camera
