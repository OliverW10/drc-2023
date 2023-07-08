#include <filesystem>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include "vision.hpp"
#include "controller.hpp"
#include "car_state.hpp"
#include "config.hpp"
#include "comm.hpp"

enum DriveState{
    NODRIVE,
    AUTODRIVE,
    MANUALDRIVE
};

int main(int argc, char** argv )
{
    std::thread driveServerThread(runDriveServer);
    bool last_enabled = false;
    int w = 640;
    int h = 480;
    tryUpdateConfig();
    Vision vis(w, h);
    Controller controller;

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
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
        CarState autodrive_desired_state = vis.process(image, sensor_values, true);

        bool is_connected = getLatestMessage(net_message);
        if(is_connected && net_message.enabled && !last_enabled){
            vis.forceStart();
        }

        DriveState mode;
        if(is_connected && net_message.enabled){
            mode = AUTODRIVE;
        }else{
            mode = MANUALDRIVE;
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
