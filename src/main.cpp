#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "vision.hpp"
#include "controller.hpp"


int main(int argc, char** argv )
{
    Vision vis(640, 480);
    Controller controller;

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,480);
    if(!cap.isOpened())
        puts("cannot open camera");

    cv::Mat image;
    while(true){
        cap >> image;
        if(image.empty()){
            puts("didnt recive frame");
            return -1;
        }
        SensorValues sensor_values = controller.getSensorValues();
        CarState desired_state = vis.process(image, sensor_values);
        controller.commandState(desired_state);

        char c = (char)cv::waitKey(1);
        if(c==27) break;
    }
    return 0;
}
