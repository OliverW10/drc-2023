#include <filesystem>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include "vision.hpp"
#include "controller.hpp"
#include "config.hpp"
#include "comm.hpp"

enum DriveState{
    NODRIVE,
    AUTODRIVE,
    MANUALDRIVE
};

int main(int argc, char** argv )
{
    startServer();

    Vision vis(640, 480);
    Controller controller;

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,480);
    if(!cap.isOpened())
        puts("cannot open camera");

    cv::Mat image;
    Message net_message;
    while(true){
        cap >> image;
        if(image.empty()){
            puts("didnt recive frame");
            return -1;
        }
        tryUpdateConfig();
        SensorValues sensor_values = controller.getSensorValues();
        CarState autodrive_desired_state = vis.process(image, sensor_values);

        bool is_connected = getLatestMessage(net_message);

        /*
            switch on, disconnected - auto
            switch on, connected, enabled - auto
            switch on, connected, disabled - manual
            switch off, disconnected - off
            switch off, connected, enabled - manual
            switch off, connected, disabled - off
        */
        DriveState mode;
        if(sensor_values.toggle){
            if(is_connected && !net_message.enabled){
                mode = MANUALDRIVE;
            }else{
                mode = AUTODRIVE;
            }
        }else{
            if(is_connected && net_message.enabled){
                mode = MANUALDRIVE;
            }else{
                mode = NODRIVE;
            }
        }

        switch(mode){
            case AUTODRIVE:
                controller.commandState(autodrive_desired_state);
                break;
            case MANUALDRIVE:
                controller.commandState(net_message.toCarState());
                break;
            case NODRIVE:
            default:
                controller.commandState(CarState{0, 0});
                break;
        }

        char c = (char)cv::waitKey(1);
        if(c==27) break;
    }
    return 0;
}
