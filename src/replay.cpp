#include <filesystem>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "vision.hpp"
#include "controller.hpp"
#include "config.hpp"

int main(int argc, char** argv )
{

    std::string vid_path;
    if(argc == 2){
        vid_path = argv[1];
    }else{
        std::filesystem::path project_dir = std::filesystem::path(__FILE__).parent_path().parent_path();
        vid_path = project_dir / "images" / "test3.mov";
    }
    cv::VideoCapture cap(vid_path);
    if(!cap.isOpened())
        puts("cannot open camera");
    cv::Mat image;
    cap >> image;
    if(image.empty()){
        puts("didnt recive frame");
        return -1;
    }
    int height = image.rows;
    int width = image.cols;
    Vision vis(width, height);
    Controller controller;



    while(true){
        tryUpdateConfig();
        cap >> image;
        if(image.empty()){
            puts("didnt recive frame");
            // break;
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }
        SensorValues sensor_values = controller.getSensorValues();
        CarState desired_state = vis.process(image, sensor_values);
        controller.commandState(desired_state);

        char c = (char)cv::waitKey(1);
        if(c==27) break;
    }
    vis.detachThreads();
    return 0;
}
