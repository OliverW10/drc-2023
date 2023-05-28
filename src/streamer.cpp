#include "streamer.hpp"
#include <nadjieb/mjpeg_streamer.hpp>
#include <thread>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <atomic>
#include <mutex>
namespace streamer{

static std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
static nadjieb::MJPEGStreamer streamer;

void initStreaming(){
    streamer.start(8080);
}

#define DO_CV_IMSHOW true
#define DO_STREAM true


static std::mutex stream_data_mutex;
static bool running = true;
static cv::Mat stream_image;
static std::string stream_name;
static bool stream_data_ready = false;

void imshow(std::string name, const cv::Mat& image){
    if(DO_CV_IMSHOW){
        cv::imshow(name, image);
    }
    if(DO_STREAM){
        auto start_time = std::chrono::high_resolution_clock::now();
        stream_data_mutex.lock();
        auto wait_duration = std::chrono::high_resolution_clock::now() - start_time;
        if(wait_duration > std::chrono::microseconds(100)){
            std::cout
                << name
                << " waited for stream thread for (us): "
                << std::chrono::duration_cast<std::chrono::microseconds>(wait_duration).count()
                << "\n";
        }
        // std::cout << "waited to stream: " << i * wait << "\n";
        stream_image = image;
        stream_name = name;
        stream_data_ready = true;
    }
}

void streamThreadLoop(){
    while(running){
        if(stream_data_ready){
            std::vector<uchar> buff;
            cv::imencode(".jpg", stream_image, buff, params);
            streamer.publish("/"+stream_name, std::string(buff.begin(), buff.end()));
            stream_data_ready = false;
            stream_data_mutex.unlock();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}
static std::thread stream_thread(streamThreadLoop);

void closeThread(){
    running = false;
    stream_thread.join();
}
}