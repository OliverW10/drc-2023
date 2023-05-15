# UTS DRC 2023


### To run on ubuntu or similar:
install libopencv-dev and libeigen3-dev from apt

install libgtest-dev (https://zwarrior.medium.com/install-google-test-framework-gtest-on-ubuntu-20-04-368eb6951b12)

install https://github.com/nadjieb/cpp-mjpeg-streamer (git clone it, run cmake, build with make then sudo make install)

then build this with
```sh
mkdir build
cd build
cmake ..
make -j4
./Main
```
